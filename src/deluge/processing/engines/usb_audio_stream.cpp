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
#include "io/debug/print.h"
#include "io/midi/sysex.h"
#include <cstdint>

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

constexpr uint32_t kChannels = 8;
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

/// Nothing renders into this yet, so every packet sends the same silence.
alignas(4) uint8_t silence[kMaxPacketBytes] = {};

usb_utr_t transfer;
bool transferInitialised = false;
bool transferInFlight = false;
uint32_t frameAccumulator = 0;

void transferComplete(usb_utr_t* /*ptr*/, uint16_t /*data1*/, uint16_t /*data2*/) {
	transferInFlight = false;
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

} // namespace

namespace deluge::processing::engines {

void USBAudioStream::routine() {
	drainSetupTrace();

	// The host parks the interface at alt 0 when it is not streaming, which is what frees the isochronous bandwidth.
	if (!g_usb_peri_connected || g_usb_pstd_alt_num[kAudioInterfaceNumber] != kStreamingAltSetting) {
		transferInFlight = false;
		frameAccumulator = 0;
		return;
	}

	if (transferInFlight) {
		return;
	}

	if (!transferInitialised) {
		transfer.complete = transferComplete;
		transfer.p_setup = nullptr;
		transfer.segment = USB_TRAN_END;
		transfer.ip = USB_CFG_USE_USBIP;
		transfer.ipp = usb_hstd_get_usb_ip_adr(USB_CFG_USE_USBIP);
		transferInitialised = true;
	}

	uint32_t frames = kFramesPerPacket;
	frameAccumulator += kFrameRemainder;
	if (frameAccumulator >= 1000) {
		frames++;
		frameAccumulator -= 1000;
	}

	transfer.keyword = USB_CFG_PAUDIO_ISO_IN;
	transfer.tranlen = frames * kFrameBytes;
	transfer.p_tranadr = silence;

	transferInFlight = true;
	usb_pstd_transfer_start(&transfer);
}

} // namespace deluge::processing::engines

extern "C" void usbAudioStreamRoutine() {
	deluge::processing::engines::USBAudioStream::routine();
}
