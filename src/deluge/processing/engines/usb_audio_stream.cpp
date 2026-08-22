/*
 * Copyright © 2026 Synthstrom Audible Limited
 *
 * This file is part of The Synthstrom Audible Deluge Firmware.
 *
 * The Synthstrom Audible Deluge Firmware is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <https://www.gnu.org/licenses/>.
 */

#include "processing/engines/usb_audio_stream.h"
#include "definitions_cxx.hpp"
#include "dsp/stereo_sample.h"
#include "io/debug/print.h"
#include "io/midi/sysex.h"
#include <cstdint>
#include <cstring>

extern "C" {
#include "RZA1/usb/r_usb_basic/r_usb_basic_if.h"
#include "deluge/drivers/usb/usb_setup_trace.h"
#include "deluge/drivers/usb/userdef/r_usb_paudio_config.h"

// Declared here rather than by including r_usb_extern.h, which carries an inline function that only compiles as C.
// midi_engine.cpp reaches into the same driver the same way.
extern uint16_t g_usb_peri_connected;
extern uint16_t g_usb_pstd_alt_num[];
usb_er_t usb_pstd_transfer_start(usb_utr_t* ptr);
usb_regadr_t usb_hstd_get_usb_ip_adr(uint16_t ipno);
}

namespace {

/// Must match the AudioStreaming interface's number in r_usb_pmidi_descriptor.c.
constexpr uint16_t kAudioInterfaceNumber = 2;
constexpr uint16_t kStreamingAltSetting = 1;

constexpr uint32_t kChannels = USB_CFG_PAUDIO_CHANNELS;
constexpr uint32_t kSubframeBytes = 2;
constexpr uint32_t kFrameBytes = kChannels * kSubframeBytes;
constexpr uint32_t kSampleRate = 44100;

/// 44.1 kHz does not divide into 1 ms frames. Packets carry 44 audio frames, and a 45th every tenth packet, which
/// averages to exactly 44100 rather than drifting 100 samples a second.
constexpr uint32_t kFramesPerPacket = kSampleRate / 1000;
constexpr uint32_t kFrameRemainder = kSampleRate % 1000;
constexpr uint32_t kMaxPacketBytes = (kFramesPerPacket + 1) * kFrameBytes;

static_assert(kMaxPacketBytes <= USB_CFG_PAUDIO_BUF_BYTES,
              "Audio packet does not fit the pipe buffer declared in r_usb_paudio_config.h");

/// Frames held between the audio routine and this task. A power of two so the wrap is a mask.
///
/// The producer writes a whole render window at once and windows reach 128 frames, so the ring only has to
/// outrun a burst rather than hold latency. 1024 leaves eight bursts of margin for 22 KB of SDRAM.
constexpr uint32_t kRingFrames = 1024u;
constexpr uint32_t kRingMask = kRingFrames - 1u;

/// How far ahead of the host the ring tries to stay before the first packet goes out.
///
/// This is the stream's latency and the only thing standing between an engine that ran late and an audible
/// gap. 128 frames is ~2.9 ms -- one worst-case render window. A DAW compensates for a reported latency and
/// does not care about a few ms, so this starts far tighter than the 768 frames the CV sockets need.
constexpr uint32_t kLeadFrames = 128u;

/// Past this the host is not draining us and the ring is heading for a lap. Snap back to the lead instead:
/// one discontinuity, immediately recovered, rather than a wrap that silently reorders a second of audio.
constexpr uint32_t kResyncFrames = kRingFrames - (kRingFrames / 4u);

/// Interleaved, one frame per host audio frame. Only channels 0 and 1 are ever written, so the rest are
/// zeroed once at startup and left alone -- per-track routing fills them in a later cut.
PLACE_SDRAM_BSS int16_t ring[kRingFrames * kChannels];

/// Written by the audio routine, read by this task, and vice versa. Each is single-writer and 32-bit
/// aligned, which is atomic on this core, so no lock is needed between the two.
volatile uint32_t writeFrame = 0;
volatile uint32_t readFrame = 0;
bool ringCleared = false;
bool primed = false;

/// Whether a host currently has the streaming interface selected. Read by the producer so that a machine with
/// nothing plugged in does not pay for the conversion, and set in one place so the two sides cannot disagree.
bool streamActive = false;

/// Copied out of the ring before submitting, because the driver reads this memory after the call returns and
/// the producer must stay free to write. One buffer suffices: only one transfer is ever in flight.
alignas(4) uint8_t packet[kMaxPacketBytes] = {};

usb_utr_t transfer;
bool transferInitialised = false;
bool transferInFlight = false;
uint32_t frameAccumulator = 0;

/// Stream health, reported over the SysEx debug channel. Silence costs nothing to render, so these only mean
/// anything once real audio is flowing -- which is exactly what makes them the first evidence on T3.
uint32_t statUnderruns = 0;
uint32_t statResyncs = 0;
uint32_t statPacketsSent = 0;
uint32_t statFramesSent = 0;
uint32_t statReportCountdown = 0;

/// Branch counters. "Nothing was sent" cannot distinguish the task never running, the task
/// returning because a transfer is still outstanding, and the task sending silence because the
/// ring looked empty - and those want three different fixes. Counting the branch taken is what
/// separates them; counting completions tests whether the driver ever hands the transfer back.
uint32_t statAlt1 = 0;
uint32_t statInFlight = 0;
uint32_t statPrimeSilence = 0;
uint32_t statCompletes = 0;
uint32_t statSubmits = 0;
uint32_t statLastErr = 0xFFFF;
uint32_t statWedges = 0;

/// How many passes a transfer may stay outstanding before it is treated as lost.
///
/// The task runs about every half millisecond and the endpoint is polled every millisecond, so
/// a packet still outstanding after three passes is not coming back. Without this a single
/// failed submit stops the stream for good, which is exactly what happened - the driver
/// guards in the transfer-start path are compiled out, so a rejection reports nothing.
constexpr uint32_t kInFlightPassLimit = 3;
uint32_t inFlightPasses = 0;

void transferComplete(usb_utr_t* /*ptr*/, uint16_t /*data1*/, uint16_t /*data2*/) {
	statCompletes++;
	transferInFlight = false;
}

/// Submits one packet and reports whether the driver accepted it. The flag is only held when it
/// did: setting it unconditionally is what turned a single rejected transfer into a dead stream.
void submit(const uint8_t* data, uint32_t bytes) {
	transfer.keyword = USB_CFG_PAUDIO_ISO_IN;
	transfer.tranlen = bytes;
	transfer.p_tranadr = (void*)data;
	statSubmits++;
	const usb_er_t err = usb_pstd_transfer_start(&transfer);
	statLastErr = (uint32_t)err;
	transferInFlight = (err == USB_OK);
	inFlightPasses = 0;
}

/// Diagnostic for USB Audio Class bring-up: reports every control request the host sends, over the SysEx debug
/// channel. One per call so a burst cannot monopolise the task. Enable the channel with F0 00 21 7B 01 03 00 01 F7.
void drainSetupTrace() {
	if (Debug::midiDebugCable == nullptr) {
		return;
	}
	UsbSetupTraceEntry entry;
	if (!usbSetupTracePop(&entry)) {
		return;
	}
	// Hand-formatted rather than via snprintf, which drags in newlib stdio that this firmware does not link.
	char line[48];
	char* p = line;
	auto emit = [&p](const char* s) {
		while (*s != '\0') {
			*p++ = *s++;
		}
	};
	auto emitHex = [&p](uint32_t v, int digits) {
		for (int shift = (digits - 1) * 4; shift >= 0; shift -= 4) {
			*p++ = "0123456789ABCDEF"[(v >> shift) & 0xF];
		}
	};

	if (entry.isMark) {
		// A point reached in the driver, not a request off the wire. Tags are in usb_setup_trace.h.
		emit("MK ");
		emitHex(entry.sequence, 4);
		emit(" t");
		emitHex(entry.type, 2);
		emit(" ");
		emitHex(entry.value, 4);
		emit(" ");
		emitHex(entry.index, 4);
		emit(" ");
		emitHex(entry.length, 4);
		*p = '\0';
		Debug::sysexDebugPrint(*Debug::midiDebugCable, line, true);
		return;
	}

	// bmRequestType and bRequest share one register: low byte then high byte.
	emit("SU ");
	emitHex(entry.sequence, 4);
	emit(" rt");
	emitHex(entry.type & 0xFF, 2);
	emit(" rq");
	emitHex((entry.type >> 8) & 0xFF, 2);
	emit(" v");
	emitHex(entry.value, 4);
	emit(" i");
	emitHex(entry.index, 4);
	emit(" l");
	emitHex(entry.length, 4);
	emit(" a");
	emitHex(g_usb_pstd_alt_num[kAudioInterfaceNumber], 2);
	*p = '\0';

	Debug::sysexDebugPrint(*Debug::midiDebugCable, line, true);
}

/// One line a second while the debug channel is on: how much audio actually left, and how often the ring was
/// empty or had to be snapped back. A gap-free stream reports rising packets with underruns and resyncs at zero.
void reportStats() {
	if (Debug::midiDebugCable == nullptr) {
		return;
	}
	if (statReportCountdown-- != 0) {
		return;
	}
	statReportCountdown = 1000;

	char line[64];
	char* p = line;
	auto emit = [&p](const char* t) {
		while (*t != '\0') {
			*p++ = *t++;
		}
	};
	auto emitDec = [&p](uint32_t v) {
		char tmp[11];
		int n = 0;
		do {
			tmp[n++] = (char)('0' + (v % 10u));
			v /= 10u;
		} while (v != 0u);
		while (n-- > 0) {
			*p++ = tmp[n];
		}
	};

	emit("AU a1:");
	emitDec(statAlt1);
	emit(" inf");
	emitDec(statInFlight);
	emit(" sil");
	emitDec(statPrimeSilence);
	emit(" cpl");
	emitDec(statCompletes);
	emit(" sub");
	emitDec(statSubmits);
	emit(" err");
	emitDec(statLastErr);
	emit(" wdg");
	emitDec(statWedges);
	emit(" pkt");
	emitDec(statPacketsSent);
	emit(" frm");
	emitDec(statFramesSent);
	emit(" ur");
	emitDec(statUnderruns);
	emit(" rs");
	emitDec(statResyncs);
	emit(" lead");
	emitDec((writeFrame - readFrame) & kRingMask);
	*p = '\0';
	Debug::sysexDebugPrint(*Debug::midiDebugCable, line, true);

	statPacketsSent = 0;
	statFramesSent = 0;
	statAlt1 = 0;
	statInFlight = 0;
	statPrimeSilence = 0;
	statCompletes = 0;
	statSubmits = 0;
	statWedges = 0;
}

} // namespace

namespace deluge::processing::engines {

void USBAudioStream::routine() {
	drainSetupTrace();
	reportStats();

	if (!ringCleared) {
		// Channels beyond the first two are never written, so zeroing once here is what keeps them silent.
		memset(ring, 0, sizeof(ring));
		ringCleared = true;
	}

	// The host parks the interface at alt 0 when it is not streaming, which is what frees the isochronous bandwidth.
	if (!g_usb_peri_connected || g_usb_pstd_alt_num[kAudioInterfaceNumber] != kStreamingAltSetting) {
		transferInFlight = false;
		frameAccumulator = 0;
		primed = false;
		streamActive = false;
		readFrame = writeFrame;
		return;
	}
	streamActive = true;
	statAlt1++;

	if (transferInFlight) {
		statInFlight++;
		if (++inFlightPasses < kInFlightPassLimit) {
			return;
		}
		// Treated as lost rather than waited on any longer. Counted, so a stream that only works
		// because of this reads as broken rather than as healthy.
		statWedges++;
		transferInFlight = false;
	}
	inFlightPasses = 0;

	if (!transferInitialised) {
		transfer.complete = transferComplete;
		transfer.p_setup = nullptr;
		transfer.segment = USB_TRAN_END;
		transfer.ip = USB_CFG_USE_USBIP;
		transfer.ipp = usb_hstd_get_usb_ip_adr(USB_CFG_USE_USBIP);
		transferInitialised = true;
	}

	// Nominal packet size, used only while there is nothing real to send. 44.1 kHz does not divide into 1 ms
	// frames, so 44 frames with a 45th every tenth packet averages exactly 44100 rather than drifting.
	uint32_t frames = kFramesPerPacket;
	frameAccumulator += kFrameRemainder;
	if (frameAccumulator >= 1000) {
		frames++;
		frameAccumulator -= 1000;
	}

	uint32_t available = (writeFrame - readFrame) & kRingMask;

	// Hold the first packets back until there is a window's worth of slack, so the very first late render does
	// not produce a gap the host has to hear.
	if (!primed) {
		if (available < kLeadFrames) {
			statPrimeSilence++;
			memset(packet, 0, frames * kFrameBytes);
			submit(packet, frames * kFrameBytes);
			return;
		}
		primed = true;
	}

	// The host is not draining us fast enough to keep up. A rate trim cannot recover this in useful time, so
	// put the read pointer back at the lead and take the one discontinuity.
	if (available >= kResyncFrames) {
		readFrame = (writeFrame - kLeadFrames) & kRingMask;
		available = kLeadFrames;
		statResyncs++;
	}

	if (available == 0) {
		// The engine has not produced anything since the last poll. Send correctly sized silence rather than
		// nothing, so the endpoint keeps answering, and do not advance the read pointer.
		statUnderruns++;
		memset(packet, 0, frames * kFrameBytes);
		submit(packet, frames * kFrameBytes);
		return;
	}

	// Send what the engine actually produced. Never a partial audio frame: a host that appends one rotates the
	// channel mapping permanently and silently.
	uint32_t send = available;
	if (send > kFramesPerPacket + 1u) {
		send = kFramesPerPacket + 1u;
	}

	const uint32_t start = readFrame & kRingMask;
	const uint32_t firstRun = (start + send > kRingFrames) ? (kRingFrames - start) : send;
	memcpy(packet, &ring[start * kChannels], firstRun * kFrameBytes);
	if (firstRun < send) {
		memcpy(&packet[firstRun * kFrameBytes], &ring[0], (send - firstRun) * kFrameBytes);
	}
	readFrame = (readFrame + send) & kRingMask;

	statPacketsSent++;
	statFramesSent += send;

	submit(packet, send * kFrameBytes);
}

void USBAudioStream::feedMix(const StereoSample* mix, uint32_t numSamples) {
	// Nothing is listening, so do not pay for the conversion. While stopped the consumer parks the read pointer
	// on the write pointer every pass, so the ring cannot look full to the first host that arrives.
	if (!streamActive || numSamples == 0 || mix == nullptr) {
		return;
	}

	// Producer and consumer are both cooperative tasks on one core, so neither can interleave with the other and
	// the single-writer pointers below need no barrier. That is a property of the scheduler, not of the pointers:
	// moving either side into an interrupt would break it.

	uint32_t w = writeFrame;
	for (uint32_t i = 0; i < numSamples; i++) {
		int16_t* const frame = &ring[(w & kRingMask) * kChannels];
		// The samples handed here are already at output scale, so this is only a width reduction.
		frame[0] = (int16_t)(mix[i].l >> 16);
		frame[1] = (int16_t)(mix[i].r >> 16);
		w++;
	}
	writeFrame = w & kRingMask;
}

} // namespace deluge::processing::engines

extern "C" void usbAudioStreamRoutine() {
	deluge::processing::engines::USBAudioStream::routine();
}
