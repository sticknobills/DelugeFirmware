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
#include "model/usb_route.h"
#include "util/functions.h"
#include <cstdint>
#include <cstring>

extern "C" {
#include "OSLikeStuff/timers_interrupts/clock_type.h"
#include "OSLikeStuff/timers_interrupts/timers_interrupts.h"
#include "RZA1/cpu_specific.h"
#include "RZA1/intc/devdrv_intc.h"
#include "RZA1/mtu/mtu.h"
#include "RZA1/ostm/ostm.h"
#include "RZA1/system/iodefine.h"
#include "RZA1/system/iodefines/dmac_iodefine.h"
#include "RZA1/system/iodefines/usb20_iodefine.h"
#include "RZA1/usb/r_usb_basic/r_usb_basic_if.h"
#include "RZA1/usb/r_usb_basic/src/hw/inc/r_usb_bitdefine.h"
#include "definitions.h"
#include "deluge/drivers/usb/usb_setup_trace.h"
#include "deluge/drivers/usb/userdef/r_usb_paudio_config.h"
#include "drivers/dmac/dmac.h"

// Declared here rather than by including r_usb_extern.h, which carries an inline function that only compiles as C.
// midi_engine.cpp reaches into the same driver the same way.
extern uint16_t g_usb_peri_connected;
extern uint16_t g_usb_pstd_alt_num[];
usb_er_t usb_pstd_transfer_start(usb_utr_t* ptr);
void usb_pstd_forced_termination(uint16_t pipe, uint16_t status);
usb_regadr_t usb_hstd_get_usb_ip_adr(uint16_t ipno);
uint16_t usb_cstd_is_set_frdy_rohan(uint16_t pipe);
uint8_t* usb_pstd_write_fifo(uint16_t count, uint16_t pipemode, uint8_t* write_p);
void hw_usb_set_bval(usb_utr_t* ptr, uint16_t pipemode);
uint16_t hw_usb_read_fifoctr(usb_utr_t* ptr, uint16_t pipemode);

// What the driver recorded writing to each pipe's configuration registers. Those registers are only reachable
// through a shared selector, so these are printed alongside a single hardware read: equal says nothing, unequal
// would say the configuration never landed.
// The driver's shadow of the shared FIFO port selector. Restoring the port after borrowing it needs both this
// and the register itself put back, or the driver's own next call believes the port is already where it wants.
extern uint16_t fifoSels[];

extern uint16_t pipeCfgs[];
extern uint16_t pipeBufs[];
extern uint16_t pipeMaxPs[];
void __disable_irq(void);
void __enable_irq(void);
}

namespace {

/// Whether this build carries its instruments.
///
/// Off is the shipping stream: no counters, no SysEx report, no cycle timing, and - the part that matters to a
/// listener - channels 3 upwards carry audio rather than the producer stamp, the mark and the packet number.
/// On restores every instrument the transport was built with, which is what the bench tools read: delivery.py
/// and seams.py both decode the producer stamp, so a build with this off can only be judged by ear.
///
/// A constant rather than a preprocessor flag so both halves stay compiled and cannot rot apart. Everything it
/// guards is discarded by the optimiser, so an off build pays nothing for carrying it.
constexpr bool kDiagnostics = true;

/// Whether the stamps ride in the audio channels.
///
/// Separate from kDiagnostics because the two make different claims on the machine. The instruments cost CPU and
/// nothing else; these cost *channels* - 3 upwards cannot carry a stamp and a track at once. A build measuring
/// what routing costs needs the instruments on and the stamps off, which one flag could not express. Implies
/// kDiagnostics: the encoder and the marks are compiled with the rest of the instruments.
constexpr bool kAudioStamps = false;

/// DIAGNOSTIC. Puts the Deluge's own finished mix on channels 7 and 8, in place of whatever stems are routed
/// there.
///
/// The one thing the outgoing stream cannot otherwise show: a stem is captured before the master chain and the
/// return is summed in after it, so nothing on the wire contains the returned audio. Without this the round trip
/// can only be judged by ear, and a round-trip *latency* cannot be judged by ear at all.
///
/// Off for anything a listener judges - it costs two channels of real routing. On for the latency and unity-gain
/// measurements, which is the only reason it exists. Routing the finished mix as a *feature* is stage D, and is a
/// different decision from this one.
constexpr bool kMixOnChannels78 = false;
static_assert(!kAudioStamps || kDiagnostics, "The stamps are part of the instruments and need them compiled in");

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

/// The cap on a real packet is the endpoint's, not the sample rate's: a Full Speed isochronous endpoint carries
/// 1023 bytes, which is 46 whole audio frames at eleven channels rather than 45.
///
/// That one frame moves break-even. The transmit path must land 44100 / kMaxFramesPerPacket writes a second to
/// carry what the engine renders - 980/s at 45 frames, 959/s at 46 - and it measures 900-904 under load. Derived
/// from kFrameBytes rather than written down, so changing the channel count moves this with it.
constexpr uint32_t kIsoLimitBytes = 1023;
static_assert(USB_CFG_PAUDIO_MAX_FRAMES * kFrameBytes <= kIsoLimitBytes,
              "A Full Speed isochronous packet cannot exceed 1023 bytes");

/// Every packet is a whole number of 32-byte blocks, because that is the unit the DMA controller moves in.
///
/// Moving four bytes per bus cycle took 130 us mean and 675 us worst from arming a transfer to committing the
/// packet, against 60.7 us for the CPU copy it replaced, and 27% of packets were committed a whole host frame
/// after they were armed - measured 2026-08-29. A packet committed after the host has asked for it waits for the
/// next token, which is what doubled the interruption rate. Thirty-two bytes a cycle is eight times fewer.
///
/// A frame is kFrameBytes, so a block is exactly two frames and every packet size here is even. The ceiling
/// drops from 63 frames to 62, which raises break-even from 880 writes a second to about 890 against the 985
/// measured.
constexpr uint32_t kDmaBlockBytes = 32u;
constexpr uint32_t kFramesPerBlock = kDmaBlockBytes / kFrameBytes;
static_assert(kDmaBlockBytes % kFrameBytes == 0, "A DMA block must be a whole number of audio frames");
/// Taken from the endpoint's own declaration rather than from the bus ceiling, so the packet the builder may
/// emit and the packet the host reserved bandwidth for are one number. They were two, and differed by a frame.
constexpr uint32_t kMaxFramesPerPacket = USB_CFG_PAUDIO_MAX_FRAMES / kFramesPerBlock * kFramesPerBlock;
constexpr uint32_t kMaxPacketBytes = kMaxFramesPerPacket * kFrameBytes;
static_assert(kMaxPacketBytes % kDmaBlockBytes == 0, "Packets must be whole DMA blocks");

static_assert(kMaxFramesPerPacket > kFramesPerPacket,
              "A packet must hold more than the nominal frame count or the stream can never keep up");

/// The diagnostics write channels 3-6 by fixed offset into the audio frame, so a smaller frame would put the
/// producer stamp and the packet number outside it. Fails the build rather than corrupting the frame quietly.
/// Only binding while they are compiled in - a shipping build is free to be narrower than six channels.
static_assert(!kAudioStamps || kChannels >= 6, "Channels 3-6 carry the producer stamp, the mark and the packet number");

/// The stems fill the frame. With the instruments on they are not written at all - channels 3-6 carry stamps
/// instead, and 1-2 carry the main mix, which is the pair every measurement to date was taken on.
static_assert(kAudioStamps || kChannels >= deluge::processing::engines::USBAudioStream::kStemChannels,
              "Every channel carries one mono track");

static_assert(kMaxPacketBytes <= USB_CFG_PAUDIO_BUF_BYTES,
              "Audio packet does not fit the pipe buffer declared in r_usb_paudio_config.h");

/// Frames held between the audio routine and this task. A power of two so the wrap is a mask.
///
/// Sized to outlast a stalled drain, not a single render burst. Under a dense song the task rate falls far
/// enough that the ring reaches the resync threshold ~60 times a second and discards 14.5 ms each time, which
/// is audible. 8192 frames is 186 ms of SDRAM headroom for 176 KB, and costs no latency: the lead below sets
/// that, not the ring. It does raise how far behind the stream can drift before a resync, so a drain deficit
/// that is sustained rather than bursty will show as lag here instead of glitching.
constexpr uint32_t kRingFrames = 8192u;
constexpr uint32_t kRingMask = kRingFrames - 1u;

/// How far ahead of the host the ring stays, at every moment -- not just before the first packet goes out.
///
/// This is the stream's latency and the only thing standing between an engine that ran late and an audible
/// gap. A DAW compensates for a reported latency and does not care about a few ms.
///
/// 256 frames, ~5.8 ms, two worst-case render windows. Raising this from 128 to 512 on 2026-08-27 changed
/// nothing, because until the builder below was changed it was not a set-point at all: it was read in the
/// priming check and the resync snap-back and nowhere in the steady-state path, so the lead started here and
/// decayed to zero on the first packet. The builder now holds it, which is what makes the number mean
/// something. Two windows rather than one because several builds can bunch up inside a single 2.9 ms gap
/// between the engine's 128-frame bursts, and one window is emptied by exactly that.
constexpr uint32_t kLeadFrames = 256u;

/// How far the lead may stray before the packet size is trimmed a block to bring it back.
///
/// The band is what keeps the correction off the wire: inside it every packet is the nominal 44.1 frames, so
/// the host sees a rate that does not move. Wide enough to swallow the engine's 128-frame burst and the four
/// packets the builder fills in one pass, narrow enough that the lead never approaches either end of its range
/// - simulated at 184-328 frames around the 256 set-point.
constexpr uint32_t kLeadBandFrames = 64u;

/// Past this the lead is not drifting, it is recovering, and the packet size steps four blocks instead of one.
///
/// This was kMaxFramesPerPacket - 62 frames, which is *below* the band above, so the recovery branch shadowed
/// the gentle one entirely and the one-block trim was dead code. Measured on hardware 2026-08-29 (evening):
/// 8.8% of packets left at 52 frames, an 18% rate excursion arriving whenever the lead crossed 62, and three of
/// them landed in the first second where the host's own margin is thinnest. Rahul heard exactly that second and
/// nothing else in the recording. A whole set-point's worth of error is what "recovering" should mean.
constexpr uint32_t kLeadCatchUpFrames = kLeadFrames;

/// Past this the writer is about to lap the reader, which silently reorders a second of audio. Everything short
/// of it is caught up rather than discarded: the builder's catch-up branch sends four blocks above nominal,
/// draining ~52,000 frames a second against the 44,100 the engine produces, so a ring filled by a 100 ms stall
/// returns to the set-point in about 0.6 s with no audio thrown away. That is slower than the old rule's 0.4 s
/// and it is the price of keeping the correction off the wire.
///
/// At three quarters of the ring this fired first and binned the backlog instead. A kit load starves the task
/// for longer than the ring's 186 ms, and the recovery discarded up to 113,639 frames in a 1.7 s interval while
/// the engine kept feeding 44,100 a second throughout - measured 2026-08-28. The margin left above the threshold
/// is one lead plus one packet, which is what the build after the check may consume before the writer reaches it.
constexpr uint32_t kResyncFrames = kRingFrames - kLeadFrames - kMaxFramesPerPacket;

/// Interleaved, one frame per host audio frame. Only channels 0 and 1 are ever written, so the rest are
/// zeroed once at startup and left alone -- per-track routing fills them in a later cut.
PLACE_SDRAM_BSS int16_t ring[kRingFrames * kChannels];

/// One render window per stem channel, summed to mono, accumulated while the outputs render and read back by the
/// output stage that also hands over the main mix.
///
/// Ordinary memory rather than SDRAM: 3 KB, written once per track per window and read once per output sample, so
/// it is the wrong thing to put behind the slower bus. The ring is large and touched twice; this is small and
/// touched constantly.
constexpr uint32_t kStemWindowSamples = SSI_TX_BUFFER_NUM_SAMPLES;
constexpr uint32_t kStemChannels = deluge::processing::engines::USBAudioStream::kStemChannels;
int32_t stemAccumulator[kStemChannels][kStemWindowSamples];
int32_t stemSnapshot[kStemWindowSamples * 2];
uint32_t stemWindowSamples = 0;

/// Which channels have been written this window, so the first clip to name a channel assigns and any after it add.
///
/// The assigning walk is 3.8x the accumulating one, because assignment lets the compiler keep the whole loop in
/// vector registers. Sharing a channel is legal and costs the difference, but only on channels actually shared.
uint16_t channelsWritten = 0;

/// Trim applied to every stem on the way to 16 bit. Held as a multiplier so the hot path is one multiply and a
/// shift rather than a table lookup or a branch.
uint32_t stemTrimSetting = deluge::processing::engines::USBAudioStream::kTrimDefault;
int32_t stemTrimMultiplier = 1 << 24;

/// The loudest thing any stem has carried since it was last read, at capture scale.
///
/// Taken before the width reduction, so it reads true on a stem the current trim would clip - which is the whole
/// point of it. A peak read off a clipped recording only ever says "full scale".
int32_t stemPeakAll = 0;
uint32_t stemPeakCountdown = 0;

/// Free-running, not masked to the ring - they are masked only where they index it. Masked pointers make an
/// overtaking reader indistinguishable from a full ring, which cost this build a whole session on 2026-08-26.
///
/// writeFrame is advanced by the audio routine, readFrame by whoever builds the next packet - the task for the
/// first of a stream, the completion interrupt for the rest. Each is single-writer and 32-bit aligned, which is
/// atomic on this core, so no lock is needed between the two.
volatile uint32_t writeFrame = 0;
volatile uint32_t readFrame = 0;
bool ringCleared = false;
bool primed = false;

/// Whether a host currently has the streaming interface selected. Read by the producer so that a machine with
/// nothing plugged in does not pay for the conversion, and set in one place so the two sides cannot disagree.
bool streamActive = false;

/* ---- The return, host to device ---------------------------------------------------------------------------
 *
 * Stage B. Nothing here is audible: the packets are counted, decoded into a ring and drained at the render rate,
 * so that what B3 will hear can be measured before anything is connected to the mix.
 *
 * The driver's own CPU read path carries this rather than the DMA controller the roadmap named. The return is
 * 176 bytes a frame against the outgoing 990, so the copy the transmit side offloaded costs about a ninth here -
 * and offloading it there bought a lower CPU share at the price of latency (2026-08-29). Receive has no such
 * bargain to strike until the cost is measured, and usb_pstd_fifo_to_buf is the path the vendor driver already
 * gets right. Priced in B6, revisited then.
 */

/// Must match the second AudioStreaming interface's number in r_usb_pmidi_descriptor.c.
constexpr uint16_t kReturnInterfaceNumber = 3;

constexpr uint32_t kRxChannels = USB_CFG_PAUDIO_RX_CHANNELS;
constexpr uint32_t kRxFrameBytes = kRxChannels * kSubframeBytes;
constexpr uint32_t kRxMaxPacketBytes = USB_CFG_PAUDIO_RX_PACKET_BYTES;
constexpr uint32_t kRxMaxFrames = kRxMaxPacketBytes / kRxFrameBytes;

static_assert(kRxMaxPacketBytes <= USB_CFG_PAUDIO_RX_BUF_BYTES, "A return packet must fit the pipe buffer");
static_assert(kRxMaxFrames > kSampleRate / 1000, "A host may send a 45-frame packet and it must fit");

/// 93 ms of return audio. Far more than the path needs, and sized like the outgoing ring for the same reason: a
/// card load freezes every task for longer than one render window, and a ring that wraps during one loses audio
/// silently rather than reporting a gap.
constexpr uint32_t kRxRingFrames = 4096u;
constexpr uint32_t kRxRingMask = kRxRingFrames - 1u;

/// How far behind the arriving audio the reader sits, which is this direction's latency and the whole of its
/// tolerance for a host that is late. 128 frames, 2.9 ms, one render window.
///
/// The device is adaptive here and cannot ask the host to change rate - no feedback endpoint is possible at Full
/// Speed with both isochronous pipes carrying audio - so this buffer is the only thing absorbing the mismatch.
/// B4 measures the drift before anything tries to correct it.
constexpr uint32_t kRxLeadFrames = 128u;

PLACE_SDRAM_BSS int16_t rxRing[kRxRingFrames * kRxChannels];

/// Free-running like the outgoing pair, and masked only where they index. rxWriteFrame is advanced by the
/// completion interrupt and rxReadFrame by the render path, so each has one writer.
volatile uint32_t rxWriteFrame = 0;
volatile uint32_t rxReadFrame = 0;
bool rxRingCleared = false;
bool rxPrimed = false;

/// Whether the host currently holds the return interface at its streaming setting.
bool returnActive = false;

/// Two landing areas, alternating. The driver reads into this memory across the interrupt that completes the
/// previous packet, so the buffer being decoded is never the one being filled.
alignas(4) uint8_t rxPackets[2][kRxMaxPacketBytes] = {};
uint32_t rxPacketSlot = 0;

usb_utr_t rxTransfer = {};
bool rxTransferInitialised = false;
bool rxTransferInFlight = false;

/// DIAGNOSTIC.
uint32_t statRxPackets = 0;  ///< completions seen
uint32_t statRxFrames = 0;   ///< audio frames decoded out of them
uint32_t statRxEmpty = 0;    ///< completions carrying no audio at all
uint32_t statRxPartial = 0;  ///< a byte count that is not a whole number of frames - never expected
uint32_t statRxOverrun = 0;  ///< frames dropped because the reader had not taken the last ones
uint32_t statRxUnderrun = 0; ///< render windows the ring could not fill
uint32_t statRxDrained = 0;  ///< frames handed to the render path
uint32_t statRxArms = 0;     ///< transfers armed
uint32_t statRxArmErr = 0;   ///< arms the driver refused
uint32_t statRxLastErr = 0;
uint32_t statRxSizes[4] = {}; ///< 43 and under / 44 / 45 / 46 and over, in frames
int32_t rxPeak[kRxChannels] = {};

/// Largest signed magnitude on each channel since the last report. The one instrument that says the audio is real
/// rather than merely arriving: a host that opens the endpoint and sends silence and one that sends music produce
/// the same packet count and the same frame count.
void trackReturnPeak(const int16_t* frames, uint32_t numFrames) {
	for (uint32_t f = 0; f < numFrames; f++) {
		for (uint32_t c = 0; c < kRxChannels; c++) {
			int32_t v = frames[f * kRxChannels + c];
			if (v < 0) {
				v = -v;
			}
			if (v > rxPeak[c]) {
				rxPeak[c] = v;
			}
		}
	}
}

/// Copies one arrived packet into the ring, oldest-first, and reports what would not fit.
///
/// Runs in the completion interrupt and is the ring's only writer. An overrun is counted in frames rather than
/// packets, because what a listener would lose is frames and a packet count cannot be turned into one later.
void ingestReturnPacket(const uint8_t* data, uint32_t bytes) {
	const uint32_t frames = bytes / kRxFrameBytes;
	if (bytes % kRxFrameBytes != 0u) {
		statRxPartial++;
	}
	if (frames == 0u) {
		statRxEmpty++;
		return;
	}
	statRxPackets++;
	statRxFrames += frames;
	if constexpr (kDiagnostics) {
		const uint32_t nominal = kSampleRate / 1000u;
		const uint32_t bucket = frames < nominal ? 0u : (frames == nominal ? 1u : (frames == nominal + 1u ? 2u : 3u));
		statRxSizes[bucket]++;
	}

	const int16_t* samples = (const int16_t*)data;
	trackReturnPeak(samples, frames);

	// The reader's position bounds what may be written, so a stalled reader loses the newest audio rather than
	// having the oldest torn out from under it mid-window.
	const uint32_t held = rxWriteFrame - rxReadFrame;
	const uint32_t room = held >= kRxRingFrames ? 0u : kRxRingFrames - held;
	const uint32_t toWrite = frames <= room ? frames : room;
	if (toWrite < frames) {
		statRxOverrun += frames - toWrite;
	}

	uint32_t w = rxWriteFrame;
	for (uint32_t f = 0; f < toWrite; f++) {
		const uint32_t slot = (w + f) & kRxRingMask;
		for (uint32_t c = 0; c < kRxChannels; c++) {
			rxRing[slot * kRxChannels + c] = samples[f * kRxChannels + c];
		}
	}
	rxWriteFrame = w + toWrite;
}

void armReturnTransfer();

/// The driver hands a finished isochronous read back here with the *remaining* count in tranlen, so what arrived
/// is what was asked for less what is left. Re-arms immediately: an isochronous pipe left un-armed simply does
/// not answer, and the host has no way to notice or retry.
void returnTransferComplete(usb_utr_t* ptr, uint16_t /*data1*/, uint16_t /*data2*/) {
	rxTransferInFlight = false;
	const uint32_t remaining = ptr != nullptr ? (uint32_t)ptr->tranlen : kRxMaxPacketBytes;
	const uint32_t received = remaining > kRxMaxPacketBytes ? 0u : kRxMaxPacketBytes - remaining;
	ingestReturnPacket(rxPackets[rxPacketSlot], received);
	rxPacketSlot ^= 1u;
	if (returnActive) {
		armReturnTransfer();
	}
}

/// Points the driver at the next landing area and asks it to fill it.
void armReturnTransfer() {
	if (!rxTransferInitialised) {
		rxTransfer.complete = returnTransferComplete;
		rxTransfer.p_setup = nullptr;
		rxTransfer.segment = USB_TRAN_END;
		rxTransfer.ip = USB_CFG_USE_USBIP;
		rxTransfer.ipp = usb_hstd_get_usb_ip_adr(USB_CFG_USE_USBIP);
		rxTransferInitialised = true;
	}
	rxTransfer.keyword = USB_CFG_PAUDIO_ISO_OUT;
	rxTransfer.tranlen = kRxMaxPacketBytes;
	rxTransfer.p_tranadr = (void*)rxPackets[rxPacketSlot];
	statRxArms++;
	const usb_er_t err = usb_pstd_transfer_start(&rxTransfer);
	statRxLastErr = (uint32_t)err;
	// Held only when the driver took it, for the same reason the transmit side does: a flag set on a refused
	// transfer is a stream that never re-arms and never says why.
	rxTransferInFlight = (err == USB_OK);
	if (err != USB_OK) {
		statRxArmErr++;
	}
}

/// Follows the host in and out of the return interface. Mirrors the outgoing half rather than sharing it: the
/// host selects the two interfaces independently, and a DAW recording stems without sending anything back is an
/// ordinary way to use this.
void serviceReturn() {
	if (!rxRingCleared) {
		memset(rxRing, 0, sizeof(rxRing));
		rxRingCleared = true;
	}

	if (!g_usb_peri_connected || g_usb_pstd_alt_num[kReturnInterfaceNumber] != kStreamingAltSetting) {
		returnActive = false;
		rxTransferInFlight = false;
		rxPrimed = false;
		rxReadFrame = rxWriteFrame;
		return;
	}

	returnActive = true;
	if (!rxTransferInFlight) {
		// Interrupts off across the driver's own FIFO access. Receive shares the CPU FIFO port with USB MIDI and
		// with the outgoing pipe's re-arm, and a pipe selected on that port by two writers at once is what
		// truncates a packet.
		DISABLE_ALL_INTERRUPTS();
		armReturnTransfer();
		ENABLE_ALL_INTERRUPTS();
	}
}

/// Copied out of the ring before submitting, because the driver reads this memory after the call returns and the
/// producer must stay free to write.
///
/// Two of them, alternating. The pipe is double-buffered and delivery is measured at a fixed two frames from
/// completion to next packet, so keeping two packets loaded is the only way to fill every frame - and that means
/// two submissions can be outstanding, which one staging buffer cannot serve.
alignas(4) uint8_t packets[2][kMaxPacketBytes] = {};
uint32_t packetSlot = 0;

/// The staging buffer the next packet is built into. Alternated per submission rather than per completion, so the
/// buffer handed to the driver is never the one being refilled.
uint8_t* nextPacketBuffer() {
	uint8_t* p = packets[packetSlot];
	packetSlot ^= 1u;
	return p;
}

usb_utr_t transfer;
bool transferInitialised = false;
bool transferInFlight = false;
uint32_t frameAccumulator = 0;

/// Stream health, reported over the SysEx debug channel. Silence costs nothing to render, so these only mean
/// anything once real audio is flowing -- which is exactly what makes them the first evidence on T3.
uint32_t statUnderruns = 0;
/// Builds that found the ring below the cushion and sent a short packet. Read against statUnderruns: this is
/// what an under-run turns into once the builder stops draining to empty, and it costs nothing audible.
uint32_t statLeadShort = 0;
uint32_t statResyncs = 0;
uint32_t statPacketsSent = 0;
uint32_t statFramesSent = 0;
/// Frames the audio routine wrote in, and frames a resync threw away. With statFramesSent these three have
/// to balance: what goes into the ring is either sent or discarded. They do not, and the gap is the fault.
uint32_t statFramesIn = 0;
uint32_t statFramesDiscarded = 0;
/// A resync fires on `available >= kResyncFrames`, and `available` is a masked subtraction that cannot go
/// negative: if the reader passes the writer by one frame it reads as the whole ring rather than as -1.
/// These separate the two, which today are the same number and opposite situations. Signed distance from
/// the writer to the reader, sampled at each resync.
/// Every entry to buildNextPacket, against statPacketsSent which counts only the path that carries real
/// audio. The sequence stamp advances once per entry and the host reads it advancing at twice the rate
/// statPacketsSent reports, so one of the two is wrong about how often the builder runs.
/// Packets built by the task and waiting for the timer interrupt to push them into the pipe.
///
/// The build is a ~990-byte copy out of the ring, and v0.14.0 showed this machine cannot afford that inside a
/// 1 kHz interrupt. So the task builds - it already has the headroom - and the interrupt does nothing but the
/// FIFO write. Single producer (task), single consumer (timer interrupt), free-running indices.
constexpr uint32_t kQueueSlots = 4u;
/// The payload is written through the uncached mirror and read by the DMA controller, so nothing else may share
/// a cache line with it: a cached write to a neighbouring field would write the whole line back and undo the
/// uncached bytes sitting in it. The length therefore lives in its own array rather than beside the payload, and
/// each slot is padded to a whole number of cache lines.
struct QueuedPacket {
	alignas(CACHE_LINE_SIZE) uint8_t data[(kMaxPacketBytes + CACHE_LINE_SIZE - 1) / CACHE_LINE_SIZE * CACHE_LINE_SIZE];
};
QueuedPacket packetQueue[kQueueSlots];
uint16_t packetQueueBytes[kQueueSlots];

/// One packet of silence, kept ready so the wire never goes quiet while the builder cannot run.
///
/// A card load freezes the audio task, so no packet is built and the timer finds an empty queue - and an
/// endpoint that answers nothing is what makes the host's reader overtake and splice in a quarter-second-old
/// fragment. Measured 2026-08-29 (evening) under a load stress: 103 dry-ring events on the device turned into
/// 91 audible splices at the host, all of them inside the 90 seconds of loading.
///
/// Silence here is honest rather than a patch: the engine genuinely produced nothing, so a brief mute is what
/// happened. It is marked as the device's own silence, so the recording still tells it apart from a packet
/// that never arrived. The ring is not drained by it, so no audio is lost - it goes out in the next real
/// packet.
QueuedPacket standbyPacket;
uint16_t standbyBytes = 0;
bool standbyReady = false;
/// Set while the controller is moving the standby packet, so its completion does not release a queue slot that
/// was never consumed.
bool audioDmaStandby = false;
uint32_t statStandbySent = 0;

/// The same slot seen by the CPU with the cache bypassed. The builder writes here and the DMA controller reads
/// the ordinary address, so neither has to flush anything - the pattern the PIC and SSI transmit buffers already
/// use. A whole-cache flush per packet, which is what the vendor driver does, would cost far more than the copy
/// this change exists to remove.
uint8_t* uncachedSlot(uint32_t slot) {
	return (uint8_t*)((uint32_t)packetQueue[slot].data + UNCACHED_MIRROR_OFFSET);
}
volatile uint32_t queueWrite = 0;
volatile uint32_t queueRead = 0;

/// How often to try the write, and the single number that sets the write rate.
///
/// This was written as an *offset* into the host's frame, on the strength of a window measured at 700-799 us.
/// It is not one: the timer is cleared by its own compare match (timerControlSetup(timerNo, 1, scale) in
/// timers_interrupts.c), so the value is the retry period, and the number of attempts per frame is 1000/it.
/// Every reading this build ever took of "the window" was confounded with how many times it was trying.
///
/// Swept across the whole frame on hardware, 2026-08-27 (evening), 4 passes:
///
///     100 us  997 writes/s   7.8 attempts/frame        600 us  691/s   1.0
///     200 us  998 writes/s   4.1 attempts/frame        700 us  712/s   1.0
///     400 us  990 writes/s   2.0 attempts/frame        800 us  926/s   0.9
///     500 us  872 writes/s   1.4 attempts/frame        950 us  467/s   0.6
///
/// Attempts win; position does not. 400 us takes the same write rate as 100 us for a quarter of the interrupt
/// load, against a break-even of 700 writes/s at eight channels.
constexpr uint32_t kTimerTicksPerMs = DELUGE_CLOCKS_PER / 1000u;

/// b13 of INTENB0: the frame-update interrupt. Off in device mode by default - the driver only ever set it for
/// host mode - and the peripheral handler's frame branch already existed doing nothing.
constexpr uint16_t kIntEnbSofe = 0x2000u;
bool sofEnabled = false;

uint32_t sofToWriteTicks = (kTimerTicksPerMs * 2u) / 5u; ///< 400 us
uint32_t statSofSeen = 0;

/// The adaptive walk is gone, and it was working against the stream the whole time.
///
/// It stepped the period *up* by 50 us after four consecutive failures and wrapped at the top. Past ~400 us a
/// longer period means fewer attempts per frame, which means more failures, which means another step: positive
/// feedback, not a search. It is the best explanation this build has for v0.27.0 degrading as it streams -
/// forced teardowns 2 -> 1,194 while writes fell 872/s -> 695/s within one session - and for both captures
/// tonight finding it parked at 699 and 649 us, in the trough of the sweep above.
///
/// Still clocked from SOF, which is what the zeroing in usbAudioStreamStartOfFrame() is for: the period is
/// ours but the phase is the host's, so the attempts stay spread across the frame rather than drifting into a
/// beat against it.

/// DIAGNOSTIC, and it must not ship. Marches the write offset through the whole frame and records how many
/// fires wrote at each, so the window is swept rather than read off wherever the adaptive walk happened to
/// park. The walk chooses the offset in response to the very shuts being measured, so every reading taken
/// while it is running is of a point it selected - which is not a measurement of the window at all.
///
/// Three captures give offset 799 -> 939 writes/s, 699 -> 720, 649 -> 649. Monotonic, three points, and
/// useless as evidence for exactly that reason. This settles which way the causation runs.
///
/// While this is on, the adaptive walk is disabled: two things must not move the offset at once.
constexpr bool kSweepOffsets = false;
constexpr uint32_t kSweepFirstUs = 100u;
constexpr uint32_t kSweepStepUs = 50u;
constexpr uint32_t kSweepBins = 18u;         ///< 100 us to 950 us inclusive
constexpr uint32_t kSweepDwellFrames = 500u; ///< half a second per bin, ~9 s per pass

uint32_t sweepBin = 0;
uint32_t sweepFrameCount = 0;
uint32_t sweepPasses = 0;
/// Cumulative across passes and deliberately never cleared: the question is the shape of the whole sweep, and
/// one pass of 500 frames per bin is too few to read a rate off. sweepFires is the denominator, and the two
/// have to be read together or not at all.
uint32_t sweepFires[kSweepBins] = {};
uint32_t sweepWrites[kSweepBins] = {};

uint32_t sweepOffsetTicks(uint32_t bin) {
	return (kSweepFirstUs + bin * kSweepStepUs) * kTimerTicksPerMs / 1000u;
}

uint32_t statFrdyImmediate = 0;     ///< ready inside the vendored 100 ns
uint32_t statFrdyNever = 0;         ///< not ready
uint32_t statCatchUp = 0;           ///< second writes in one fire, recovering a frame whose fire never happened
constexpr int kAudioWriteTimer = 3; ///< Timer 3 is the only unused MTU2 channel.
uint32_t statTimerFires = 0;
uint32_t statTimerWrote = 0;
uint32_t statTimerShut = 0;
uint32_t statTimerEmpty = 0;
uint32_t statQueueBuilt = 0;

/// DIAGNOSTIC. How many frames each packet carried, in bins of eight, over the report interval.
///
/// The mean is already derivable from frm/pkt and it hides the thing in question. Under the old rule the
/// builder sent `available - kLeadFrames` and buildQueuedPackets() filled all four slots in one pass, so every
/// build after the first in a burst found the ring at the cushion and took an eighth of it: one full packet
/// followed by several short ones, at the same mean as an even flow. This is the histogram that shows whether
/// the trimmed builder actually flattened that - every bin should now sit on 44-52 frames.
constexpr uint32_t kSizeBins = 8u;
uint32_t statSizeHist[kSizeBins] = {};
bool audioTimerRunning = false;
/// The driver's submit path writes the same shared FIFO port as the timer interrupt, and usb_pstd_write_fifo
/// writes what it is handed with no space check while also changing the port's access width partway through.
/// Two writers interleaving there truncate a packet, and a part-frame packet rotates the host's channel
/// mapping - measured as channel 11's constant arriving on channel 1, 34 samples in 60 s on v0.16.0.
///
/// So exactly one packet per stream goes through the driver, to establish its transfer; everything after is the
/// timer's. Writes were landing at 950/s against 2.6 submits/s already, so the driver's path was contributing
/// nothing but the race.
bool firstPacketSubmitted = false;
/// Driver re-arms abandoned because a DMA transfer would not retire in time to hand the FIFO port over.
/// Declared and reported since the timer work but never incremented until the DMA path gave it a meaning.
uint32_t statSubmitSkipped = 0;
/// readFrame has exactly three writers. The counters say nothing is built and the timer never writes, and the
/// lead still reads 1 - so something advances it that is not accounted for, or one of these readings is false.
/// Tagged per site, with both pointers printed raw: the difference is what has been ambiguous.
uint32_t statReadAdvBuild = 0;  ///< buildNextPacket consuming frames
uint32_t statReadAdvResync = 0; ///< the resync snapping back to the lead
uint32_t statReadAdvPark = 0;   ///< the not-streaming branch parking read on write
uint32_t statPortBorrowed = 0;  ///< Writes that had to take the port off another user and give it back.
uint32_t statBuilds = 0;
uint32_t statResyncGenuine = 0;
uint32_t statResyncOvertaken = 0;
int32_t statWorstOvertake = 0;
uint32_t statReportCountdown = 0;

/// DIAGNOSTIC. What each hot path actually costs, in CPU cycles, so the engine load this build carries can be
/// attributed rather than inferred.
///
/// The 2026-08-29 reading that the cost is interrupt-bound came from one build that moved four things at once -
/// timer fires, FIFO writes, packet builds and ring traffic - so it cannot say which of them the instrument pays
/// for. These do, directly. Cortex-A9 PMU cycle counter, 400 MHz, 2.5 ns a tick.
///
/// Two of the six are nested inside others and must not be added to them: fifoWrite sits inside timer, and build
/// and submit both sit inside service.
struct PathCost {
	uint32_t cycles;
	uint32_t calls;
};
PathCost costTimer = {};       ///< the whole write-timer interrupt, every branch including the ones that do nothing
PathCost costFifoWrite = {};   ///< the FIFO copy alone, nested inside costTimer
PathCost costReturnMix = {};   ///< summing the return into the song's mix, the only cost stage B adds to the render
PathCost costService = {};     ///< all of service(), which the audio task is billed for and direness is read from
PathCost costBuild = {};       ///< buildQueuedPackets, ring memcpy included, nested inside costService
PathCost costSubmit = {};      ///< the driver's own submit path, nested inside costService
PathCost costFeedMix = {};     ///< the producer writing the SDRAM ring, inside the audio routine
PathCost costDmaStart = {};    ///< arming the DMA controller, which is what replaces the copy
PathCost costDmaComplete = {}; ///< committing the packet once the controller has moved it

/// The per-track routing path, which is the reason this instrument was extended.
///
/// These three are top-level: they run inside the audio engine's render, not inside service() or feedMix(), so
/// they are added to the wall-time total rather than nested in one of the others. The two are split because a
/// path that is expensive per call and a path that is merely called often want opposite fixes - the snapshot is
/// one copy per routed track per window, the capture is a walk of the same length, and if they do not come back
/// roughly equal then one of them is not doing what its shape says.
PathCost costStemSnapshot = {}; ///< copying the mix aside before a routed track renders
PathCost costStemCapture = {};  ///< walking the window again to recover that track alone
PathCost costStemClear = {};    ///< zeroing the accumulators once per render window

/// The audio task as a whole, and the per-output render loop inside it, in processor cycles.
///
/// The scheduler's own figure for this same task is a wall-clock duration, so it counts every interrupt that
/// lands mid-task as if the task had spent the time. These do not. If the two agree the routing cost is real and
/// hiding somewhere the per-path timers do not cover; if they diverge, the task is being interrupted, not slowed.
PathCost costEngine = {};
PathCost costOutputs = {};

/// DMA transmit state. audioDmaBusy is set by the timer that arms a transfer and cleared by the interrupt that
/// retires it, so it is the only thing standing between the builder and a slot still being read.
bool audioDmaConfigured = false;
bool audioDmaInterruptArmed = false;
volatile bool audioDmaBusy = false;
uint32_t statDmaStarted = 0;
uint32_t statDmaCompleted = 0;
uint32_t statDmaBusySkips = 0;
uint32_t statDmaLateRetires = 0;

/// DIAGNOSTIC. How long a packet spends between being handed to the controller and being committed, and - the
/// question that matters - whether it is still committed inside the host frame it was armed in.
///
/// The CPU copy set the buffer-valid flag the instant it finished, inside one interrupt. The controller's
/// transfer and its completion interrupt are two separate delays after the arm, and a packet committed after the
/// host has already asked for it waits a whole frame. Interruptions doubled on the change, 2.1/s to 4.3-5.2/s,
/// with the device failing to deliver 201 blocks against 11 - so the delay is the first suspect and this is the
/// measurement that convicts or clears it.
uint32_t statDmaArmCycles = 0;     ///< cycle counter at the arm, for the span below
uint16_t statDmaArmFrame = 0;      ///< host frame number at the arm
uint32_t statDmaSpanSum = 0;       ///< summed arm-to-commit cycles
uint32_t statDmaSpanMax = 0;       ///< worst arm-to-commit, cycles
uint32_t statDmaFrameSpan[3] = {}; ///< committed in the same host frame, one later, two or more later

/// Cycle count at the last report, so every figure above is divided by a clock rather than by an assumed second.
/// The report fires every 1000 task passes and the task rate breathes, which has manufactured a phantom result
/// here before.
uint32_t costIntervalStart = 0;
bool costCounterEnabled = false;

/// Unsigned throughout, so the counter's 10.7 s wrap needs no detecting: the difference is right across it as
/// long as the span itself is shorter than that, which every span here is by four orders of magnitude.
[[gnu::always_inline]] inline void addCost(PathCost& c, uint32_t start) {
	if constexpr (kDiagnostics) {
		c.cycles += Debug::readCycleCounter() - start;
		c.calls++;
	}
}

/// The cycle counter read that opens a timed span. Reads nothing when the instruments are compiled out, so a
/// shipping build makes no PMU access on the write timer's path at all.
///
/// Not to be used for anything but timing: parkAudioDmaPort's deadline is a real one and reads the counter
/// directly, which is also why Debug::init() below stays unconditional.
[[gnu::always_inline]] inline uint32_t costStart() {
	if constexpr (kDiagnostics) {
		return Debug::readCycleCounter();
	}
	return 0u;
}

/// Counters, compiled out with everything else they feed. Declared as ordinary values above rather than wrapped
/// in a type, so the shipping build differs from the measured one by these calls and nothing else.
[[gnu::always_inline]] inline void bump(uint32_t& c) {
	if constexpr (kDiagnostics) {
		c++;
	}
}

[[gnu::always_inline]] inline void bump(uint32_t& c, uint32_t n) {
	if constexpr (kDiagnostics) {
		c += n;
	}
}

/// Level applied to the returning audio, 0-50 on the same 1.2 dB ladder as the outgoing trim, and whether it is
/// summed in at all. Per-machine: like the trim, it describes the gain staging of what is on the other end of the
/// cable rather than anything about the song.
uint32_t returnLevelSetting = deluge::processing::engines::USBAudioStream::kReturnLevelDefault;
bool returnEnabled = true;

/// The whole conversion from an arriving 16-bit sample back to the mix's own scale, held as one multiplier so the
/// hot path is a multiply and a shift.
///
/// Derived from the outgoing stem conversion rather than guessed at, so the round trip is nominally unity: a stem
/// leaves as ((mix * trim) >> 24 >> 1) << AUDIO_OUTPUT_GAIN_DOUBLINGS >> 16, a net right shift of kReturnShift
/// with the trim ratio applied, and this is its inverse at whatever the trim currently is.
///
/// Nominally, not exactly - B3.5 owns the measured unity round trip, and until it lands the level control is how
/// a rig disagrees.
constexpr uint32_t kReturnShift = 1u + 16u - AUDIO_OUTPUT_GAIN_DOUBLINGS;

/// Sixteen fractional bits, not eight. At eight the trim's inverse rounded 1008.35 to 1008, which is a systematic
/// -0.0066 dB on every sample of every round trip - small, but a gain error rather than noise, so it compounds
/// through a chain rather than averaging out. Sixteen takes the same figure to -0.0016 dB. Modelled over the
/// whole input range before the change, not judged by ear.
constexpr uint32_t kReturnMultiplierBits = 16u;
int32_t returnMultiplierQ16 = 0;

/// Ramp applied on top, in Q16, so a stream that stops or a ring that runs dry fades rather than steps.
///
/// A step from a live sample to zero is a click, and the host is entitled to stop sending at any moment without
/// saying so - an isochronous endpoint has no way to announce it. 128 samples is about 3 ms.
constexpr int32_t kReturnFadeFull = 1 << 16;
constexpr int32_t kReturnFadeStep = kReturnFadeFull / 128;
int32_t returnFade = 0;

void recomputeReturnMultiplier() {
	if (stemTrimMultiplier <= 0 || !returnEnabled || returnLevelSetting == 0) {
		returnMultiplierQ16 = 0;
		return;
	}
	// The trim's own inverse: the outgoing side multiplied by trimMultiplier / 2^24.
	const uint32_t trimInverse =
	    (uint32_t)(((uint64_t)1u << (24u + kReturnMultiplierBits)) / (uint32_t)stemTrimMultiplier);
	// Then the user's level, built on the same 1.2 dB ladder as the trim so the two controls feel alike.
	int32_t levelMultiplier = 1 << 24;
	for (uint32_t step = returnLevelSetting; step < deluge::processing::engines::USBAudioStream::kReturnLevelMax;
	     step++) {
		levelMultiplier = (int32_t)(((int64_t)levelMultiplier * 57139) >> 16);
	}
	returnMultiplierQ16 = (int32_t)(((int64_t)trimInverse * levelMultiplier) >> 24);
}

/// One arriving sample, back at the mix's own scale.
///
/// Saturating, and not merely for tidiness. The multiplier is the trim's inverse, so a low trim means a large
/// one: at a trim of 10 a full-scale return lands at 4.0e9, which wraps an int32 to a large negative number. A
/// gain error is quiet and a wrap is a polarity inversion at full scale, so the guard is worth its comparison.
[[gnu::always_inline]] inline int32_t returnToMixScale(int16_t sample) {
	const int64_t wide = ((int64_t)((int32_t)sample << kReturnShift) * returnMultiplierQ16) >> kReturnMultiplierBits;
	if (wide > INT32_MAX) {
		return INT32_MAX;
	}
	if (wide < INT32_MIN) {
		return INT32_MIN;
	}
	return (int32_t)wide;
}

/// Sums one render window of returning audio into the mix, and advances the ring by exactly what it took.
///
/// The ring's only reader, which is what makes the pointer pair safe: the completion interrupt writes and this
/// reads, one writer each.
void mixReturnInto(StereoSample* buffer, uint32_t numSamples) {
	if (!returnActive) {
		// Parked on the writer, so the next host does not arrive to a ring that already looks full.
		rxReadFrame = rxWriteFrame;
		rxPrimed = false;
	}
	else if (!rxPrimed) {
		// Nothing is read until the ring has built its lead, so the first window does not start already behind.
		if ((rxWriteFrame - rxReadFrame) >= kRxLeadFrames + numSamples) {
			rxPrimed = true;
		}
	}

	const uint32_t held = rxWriteFrame - rxReadFrame;
	const bool haveAudio = returnActive && returnEnabled && rxPrimed && held >= numSamples;
	if (returnActive && returnEnabled && rxPrimed && held < numSamples) {
		statRxUnderrun++;
	}

	// Nothing to add and nothing left to fade: the ordinary state of a machine with nothing plugged in, and it
	// costs one comparison.
	if (!haveAudio && returnFade == 0) {
		return;
	}

	const uint32_t start = costStart();
	const uint32_t r = rxReadFrame;
	for (uint32_t i = 0; i < numSamples; i++) {
		// One step a sample in whichever direction the state calls for, so arriving and leaving cost the same
		// 3 ms and neither is a step change.
		if (haveAudio) {
			returnFade += kReturnFadeStep;
			if (returnFade > kReturnFadeFull) {
				returnFade = kReturnFadeFull;
			}
		}
		else {
			returnFade -= kReturnFadeStep;
			if (returnFade < 0) {
				returnFade = 0;
			}
			// Fading out over audio that is no longer arriving is fading out over nothing, so the ramp simply
			// runs down against silence.
			continue;
		}
		const int16_t* const frame = &rxRing[((r + i) & kRxRingMask) * kRxChannels];
		const int32_t left = returnToMixScale(frame[0]);
		const int32_t right = returnToMixScale(frame[kRxChannels > 1 ? 1 : 0]);
		if (returnFade >= kReturnFadeFull) {
			buffer[i].l += left;
			buffer[i].r += right;
		}
		else {
			buffer[i].l += (int32_t)(((int64_t)left * returnFade) >> 16);
			buffer[i].r += (int32_t)(((int64_t)right * returnFade) >> 16);
		}
	}
	if (haveAudio) {
		rxReadFrame = r + numSamples;
		statRxDrained += numSamples;
	}
	addCost(costReturnMix, start);
}

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

/// DIAGNOSTIC. Stamps the producer's own frame count into the ring, so a recording says what fraction of the
/// audio the Deluge rendered actually reached the host.
///
/// The previous stamp was written by the packet builder into the packet, which made it circular: it counted our
/// own output, so a stream delivering half the audio and one delivering all of it read alike at the host. This
/// counts the other end of the pipeline. A frame the builder discards at a resync simply never appears, and the
/// gap in the numbers is the loss.
///
/// Stamped per block of frames rather than per frame, because the host's capture path is not sample-exact: a
/// value that changed every frame could not survive it, while a run of identical values loses at most its edges.
/// 64 frames is 1.45 ms of resolution, far finer than the effect being measured.
constexpr uint32_t kStampBlockShift = 6;

/// Thirteen bits per channel at a step of 8, split low and high across two channels: 26 bits of block count is
/// 25 hours before it wraps, so a capture never has to reason about a wrap at all.
constexpr uint32_t kStampBits = 13;
constexpr uint32_t kStampMask = (1u << kStampBits) - 1u;

/// The capture path dithers every sample by +/-1, so every value here has to survive that. A step of 8 does, and
/// the marks sit far from each other and far from the zero a host writes when it substitutes silence.
constexpr int16_t kStampScale = 8;

/// What wrote this frame. The host's own substituted silence carries none of these, which is what separates
/// "we never sent it" from "we sent silence deliberately" - two very different faults that a zeroed channel
/// cannot tell apart.
constexpr int16_t kMarkRendered = 8000;       ///< carries audio the engine produced
constexpr int16_t kMarkDeviceSilence = 24000; ///< a packet we sent with nothing to put in it

/// Centred on the signed range so all 13 bits are usable.
int16_t encodeStamp(uint32_t value) {
	return (int16_t)((int32_t)(value & kStampMask) * kStampScale - 32768);
}

/// DIAGNOSTIC. Which packet each frame was built into, stamped once per packet on channel 6.
///
/// The producer stamp says which rendered frame this is; this says which packet carried it. Together they
/// separate the two ways the same audio can arrive twice, which no counter on either side of the wire can tell
/// apart: the same packet number twice means the hardware re-sent a buffer plane we wrote once, and two
/// different packet numbers over the same audio means we built the same ring frames into two packets.
/// Measured 2026-08-27 at ~8,400 frames/s on both write paths, and each duplicated frame costs a real one.
uint32_t packetCounter = 0;

/// Cached so the encoding runs once per block rather than once per frame.
uint32_t stampBlock = 0xFFFFFFFFu;
int16_t stampLow = 0;
int16_t stampHigh = 0;

/// The audio pipe's own state, straight off the hardware. Five readings of the driver source have produced one
/// right answer and four wrong ones on this stream; these registers are what the chip itself says.
struct PipeRegisters {
	uint16_t pipectr; ///< PID, buffer status, INBUFM - whether the pipe is armed and holding data.
	uint16_t bempenb; ///< Which pipes may still raise a send-complete interrupt.
	uint16_t nrdysts; ///< Not-ready events. Latched by the chip whether or not the interrupt is unmasked.
	uint16_t frmnum;  ///< Frame counter, and the OVRN bit an isochronous over- or under-run sets.
};

PipeRegisters readPipeRegisters() {
	usb_regadr_t reg = usb_hstd_get_usb_ip_adr(USB_CFG_USE_USBIP);
	// PIPEnCTR are contiguous from PIPE1CTR, one per pipe, which is how the driver indexes them itself.
	volatile uint16_t* pipectr = &reg->PIPE1CTR + (USB_CFG_PAUDIO_ISO_IN - 1);
	return PipeRegisters{*pipectr, reg->BEMPENB, reg->NRDYSTS, reg->FRMNUM};
}

/// CFIFOCTR and CFIFOSEL bits, named here rather than by including the driver's private headers.
constexpr uint16_t kFifoCtrFrdy = 0x2000u;    ///< b13: a buffer plane is free to be written into.
constexpr uint16_t kFifoSelCurPipe = 0x000Fu; ///< b2-0: which pipe the shared CPU FIFO port points at.
constexpr uint16_t kFifoSelIsel = 0x0020u;    ///< b5: read or write direction of that port.

/// What the CPU-side FIFO port says immediately after a packet has been written and committed.
///
/// The pipe holds two planes. FRDY set afterwards means one is still free, FRDY clear means both hold data - so
/// a submit that reached a second plane reads clear and a submit that overwrote the first reads set. That is the
/// outcome the second-plane question turns on, and the distinction a counter on the loading code cannot make.
///
/// fifosel is carried with it because the port is shared with MIDI: a reading taken while it points at another
/// pipe describes that pipe's FIFO, and is discarded rather than counted.
struct FifoProbe {
	uint16_t fifoctr;
	uint16_t fifosel;
	uint16_t pipectr;
};

/// Three volatile reads, no writes and no selector steering, so this cannot change what the pipe does. Steering
/// the selector here would - the driver's own send path owns it.
FifoProbe readFifoProbe() {
	usb_regadr_t reg = usb_hstd_get_usb_ip_adr(USB_CFG_USE_USBIP);
	volatile uint16_t* pipectr = &reg->PIPE1CTR + (USB_CFG_PAUDIO_ISO_IN - 1);
	return FifoProbe{reg->CFIFOCTR, reg->CFIFOSEL, *pipectr};
}

bool fifoProbeIsOurs(const FifoProbe& probe) {
	return (probe.fifosel & kFifoSelCurPipe) == USB_CFG_PAUDIO_ISO_IN;
}

/// Every submit's outcome. Index 1 is a plane still free afterwards, index 0 both planes full. One packet is in
/// flight at a time, so this reads all-1 unless the depth ever actually grows.
uint32_t statFrdyAfterSubmit[2] = {};
uint32_t statProbeNotOurs = 0;

/// The audio pipe's static configuration - how the endpoint is set up, as against what it is doing. Delivery sits
/// at exactly one packet every two frames with the task running 1840 times a second and the ring never empty,
/// which is a shape no amount of CPU explains; these are the registers that decide how many packets the pipe can
/// hold at once and how much FIFO it owns, and they have never been read.
struct PipeConfig {
	uint16_t pipecfg;  ///< Type, direction, and DBLB - whether the pipe is double-buffered or holds one packet.
	uint16_t pipebuf;  ///< Which 64-byte FIFO block the pipe starts at, and how many blocks it owns.
	uint16_t pipemaxp; ///< Largest packet the hardware will send in one transaction.
	uint16_t pipeperi; ///< Isochronous interval and the buffer-flush bit.
};

PipeConfig pipeConfig = {};
bool pipeConfigRead = false;

/// Steers the chip's shared pipe selector to the audio pipe, reads its configuration, and puts the selector back.
///
/// Interrupts are off across the sequence and the previous selection is restored, because the driver's own send
/// path steers the same register - leaving it pointing elsewhere would corrupt the next write rather than this
/// read. Called once per stream rather than per report for the same reason: the configuration cannot change
/// without the driver rewriting it, and the shadow copies below report that for free.
void readPipeConfig() {
	usb_regadr_t reg = usb_hstd_get_usb_ip_adr(USB_CFG_USE_USBIP);
	__disable_irq();
	const uint16_t previousSelection = reg->PIPESEL;
	reg->PIPESEL = USB_CFG_PAUDIO_ISO_IN;
	const PipeConfig read = {reg->PIPECFG, reg->PIPEBUF, reg->PIPEMAXP, reg->PIPEPERI};
	reg->PIPESEL = previousSelection;
	__enable_irq();
	pipeConfig = read;
	pipeConfigRead = true;
}

/// PIPEnCTR bits and the driver's transfer-status codes, named here rather than by including the driver's private
/// headers, which do not compile as C++.
constexpr uint16_t kPipeCtrInBufM = 0x4000u; ///< The IN buffer still holds data for the host.
constexpr uint16_t kPipeCtrPBusy = 0x0020u;  ///< A transaction is in progress on the pipe.
constexpr uint16_t kUsbDataStop = 8u;        ///< USB_DATA_STOP, r_usb_basic_define.h.

/// The same registers frozen at the last completion the driver handed back. The stream wedges with the host still
/// polling, so the live values are the dead state and these are the working one. A pipe that went bad before it
/// stopped and a pipe that looks healthy and simply stopped being asked want opposite fixes, and only the pair
/// tells them apart.
PipeRegisters regsAtLastCompletion = {};
bool regsSnapshotTaken = false;

/// Consecutive passes on which the pipe has looked abandoned, how many such passes there have been, and how often
/// one was acted on. Recovering on a single reading would risk ending a transfer that is genuinely live, which is
/// what froze the machine on 2026-08-22; two readings a millisecond apart cost nothing and cannot catch a transient.
///
/// A recovery runs the completion callback through the driver, so statCompletes counts it too - real completions
/// are statCompletes minus statAbandonedEnded.
uint32_t stuckPasses = 0;
uint32_t statAbandonedSeen = 0;
uint32_t statAbandonedEnded = 0;

/// Where the packet write lands inside the host's frame, and how far apart completions are.
///
/// The chip only makes a written packet sendable at the next SOF (hardware manual 28.4.9(4)), and answers an IN
/// token with a zero-length packet plus an underrun when the buffer is not transmission-enabled - which is the
/// state FRMNUM's OVRN bit has reported on every reading so far. Writing from the completion should still land
/// inside the frame and be sendable at the next one, which predicts a packet every frame; the machine delivers
/// one every two. These two histograms are the difference between those, and neither is inferable from source.
uint32_t statWriteFrameDelta[3] = {}; ///< Frames between entering the completion and the write returning: 0, 1, 2+.
uint32_t statCompletionGap[5] = {};   ///< Frames between consecutive completions: 0, 1, 2, 3, 4+.
uint32_t lastCompletionFrame = 0;
bool lastCompletionFrameValid = false;

constexpr uint16_t kPipeCtrBsts = 0x8000u; ///< b15: the CPU may access the pipe's FIFO buffer.
constexpr uint16_t kFifoCtrBval = 0x8000u; ///< b15: the plane just written holds a complete packet.
constexpr uint16_t kUseCpuFifo = 0u;       ///< USB_CUSE - the CPU FIFO port, as against either DMA port.
constexpr uint16_t kFifoError = 0xFFu;     ///< USB_FIFOERROR: the port never became ready.

/// Its own buffer rather than one of the alternating pair. A direct write copies into the FIFO before it
/// returns, so this is free again immediately - but the driver reads the pair after submit() returns, and
/// handing it a buffer this path is refilling is the kind of overlap that is invisible until it is audible.
alignas(4) uint8_t extraPacket[kMaxPacketBytes] = {};

uint32_t statExtraWritten = 0;  ///< Packets loaded into a second plane from the task.
uint32_t statExtraRefused = 0;  ///< Windows that closed before the port came ready.
uint32_t statExtraNoWindow = 0; ///< Task passes with a transfer in flight and no plane writable.

/// Whether a buffer plane is ever writable while a packet is still pending.
///
/// Everything measured so far read the pipe immediately after a write, which is one instant of a two-frame
/// cycle. DMA does not bypass this - its request line is "transmit FIFO empty", so the hardware asks for a
/// write only when it would accept one. If a plane never becomes writable while a packet is outstanding, DMA
/// is never asked and cannot help; if one opens briefly, DMA catches a window an interrupt-driven CPU write
/// cannot. That is the whole question, and it decides whether the DMA work is worth doing.
///
/// Indexed (BSTS << 1) | INBUFM: 0 neither, 1 data pending and the CPU locked out, 2 empty and writable,
/// 3 data pending and writable - which is the state being looked for.
uint32_t statReadyState[4] = {};

/// The same question at a resolution the task cannot reach. The task runs ~1840 times a second against a
/// 2 ms cycle, so it samples roughly four times per packet and a window of a few microseconds would fall
/// between its passes. This polls in a bounded loop straight after the write instead.
constexpr uint32_t kReadyPollIterations = 256u;
uint32_t statPollFound = 0;                    ///< Polls where a plane became writable while one was pending.
uint32_t statPollMissed = 0;                   ///< Polls where it never did.
uint32_t statPollFirstIteration = 0xFFFFFFFFu; ///< Fewest iterations taken to see it. Latched, never cleared.

constexpr uint16_t kFrameNumberMask = 0x07FFu; ///< FRMNUM b10-0; b15 is OVRN and b14 CRCE.

/// FRMNUM alone rather than the whole register set, because this runs twice per completion inside the interrupt.
/// PIPEnCTR alone. Directly addressable, so unlike the configuration registers this needs no steering of the
/// shared pipe selector and is safe to read from an interrupt at any rate.
uint16_t readPipeCtr() {
	usb_regadr_t reg = usb_hstd_get_usb_ip_adr(USB_CFG_USE_USBIP);
	volatile uint16_t* pipectr = &reg->PIPE1CTR + (USB_CFG_PAUDIO_ISO_IN - 1);
	return *pipectr;
}

uint16_t readFrameNumber() {
	usb_regadr_t reg = usb_hstd_get_usb_ip_adr(USB_CFG_USE_USBIP);
	return (uint16_t)(reg->FRMNUM & kFrameNumberMask);
}

void sendNextPacket();

void transferComplete(usb_utr_t* /*ptr*/, uint16_t /*data1*/, uint16_t /*data2*/) {
	bump(statCompletes);
	transferInFlight = false;

	if constexpr (kDiagnostics) {
		regsAtLastCompletion = readPipeRegisters();
		regsSnapshotTaken = true;

		const uint16_t frameAtCompletion = readFrameNumber();
		if (lastCompletionFrameValid) {
			const uint32_t gap = (uint32_t)((frameAtCompletion - lastCompletionFrame) & kFrameNumberMask);
			// Bucketed rather than averaged: a steady two and an alternating one-and-three give the same mean and
			// mean opposite things about where the packet is being lost.
			statCompletionGap[gap > 4u ? 4u : gap]++;
		}
		lastCompletionFrame = frameAtCompletion;
		lastCompletionFrameValid = true;
	}

	// Deliberately does NOT build or submit a packet. It used to, and that made the ring's read pointer shared
	// between this interrupt and the task: both do readFrame = readFrame + send, and a completion landing inside
	// the task's build leaves the two claiming the same frames. The result is the same audio built into two
	// different packets, and because each duplicate displaces a real packet, every frame sent twice costs a frame
	// that is never sent at all.
	//
	// Measured 2026-08-27 with a packet number stamped alongside the producer stamp: 10,179 frames/s duplicated,
	// 91.8% of it carrying two different packet numbers over the same audio. It read the same on v0.15.0 and
	// v0.21.0 - not because the fault was in the hardware, as the invariance first suggested, but because this
	// build path is the half those two share.
	//
	// The task is now the only builder. The timer still writes bytes already in memory, which is what an
	// interrupt can afford; the ~990-byte copy this used to do at 1 kHz is the same cost that starved the audio
	// engine on v0.14.0.
	if (kDiagnostics && streamActive && transferInitialised) {
		// Measured across the write rather than timed, because what the chip cares about is which frame the write
		// finished in, not how long it took - a fast write that lands after the SOF is as late as a slow one.
		const uint32_t delta = (uint32_t)((readFrameNumber() - lastCompletionFrame) & kFrameNumberMask);
		statWriteFrameDelta[delta > 2u ? 2u : delta]++;

		// Bounded rather than open: this runs inside the interrupt, and a loop whose length comes from the
		// hardware is how the machine froze on 2026-08-22. 256 register reads is a few microseconds against the
		// audio engine's 2.9 ms deadline, and the underrun and resync counters are what say whether it cost
		// anything. Reads only - nothing here writes a register.
		uint32_t iteration = 0;
		for (; iteration < kReadyPollIterations; iteration++) {
			const uint16_t ctr = readPipeCtr();
			if ((ctr & kPipeCtrInBufM) == 0u) {
				// The packet has already gone. Past this point a writable plane says nothing about whether one
				// was ever free *while* a packet was pending, which is the question.
				break;
			}
			if ((ctr & kPipeCtrBsts) != 0u) {
				statPollFound++;
				if (iteration < statPollFirstIteration) {
					statPollFirstIteration = iteration;
				}
				break;
			}
		}
		if (iteration >= kReadyPollIterations) {
			statPollMissed++;
		}
	}
}

/// Submits one packet and reports whether the driver accepted it. The flag is only held when it
/// did: setting it unconditionally is what turned a single rejected transfer into a dead stream.
void submit(const uint8_t* data, uint32_t bytes) {
	transfer.keyword = USB_CFG_PAUDIO_ISO_IN;
	transfer.tranlen = bytes;
	transfer.p_tranadr = (void*)data;
	bump(statSubmits);
	const usb_er_t err = usb_pstd_transfer_start(&transfer);
	if constexpr (kDiagnostics) {
		statLastErr = (uint32_t)err;
		// submit() returns with the packet already in a plane and BVAL set, so this reads whether a plane is left
		// free rather than whether the write happened at all.
		const FifoProbe probe = readFifoProbe();
		if (fifoProbeIsOurs(probe)) {
			statFrdyAfterSubmit[(probe.fifoctr & kFifoCtrFrdy) != 0u ? 1u : 0u]++;
		}
		else {
			statProbeNotOurs++;
		}
	}
	// Always USB_OK in practice - every guard in that function is compiled out - so this records the
	// code rather than relying on it. Kept as one call site so the flag cannot be set in two places.
	transferInFlight = (err == USB_OK);
}

/// DIAGNOSTIC. Marks a packet the device sent with nothing real in it.
///
/// A packet of deliberate silence and a packet that never arrived reach the host as the same zeroes, and they
/// mean opposite things - one is the transport keeping the endpoint answering, the other is the transport
/// failing. Only the device can write this mark, so the recording separates them.
///
/// Channels 3 upwards carry diagnostics and nothing real, so this costs no audio. Channels 1 and 2 are
/// untouched: the mix has to stay listenable for the recording to be worth anything else.
void markDeviceSilence(uint8_t* buffer, uint32_t frames) {
	if constexpr (!kAudioStamps) {
		return;
	}
	int16_t* samples = (int16_t*)buffer;
	for (uint32_t f = 0; f < frames; f++) {
		samples[f * kChannels + 2] = 0;
		samples[f * kChannels + 3] = 0;
		samples[f * kChannels + 4] = kMarkDeviceSilence;
	}
}

/// DIAGNOSTIC. Numbers the packet itself, at the moment it is built - not when it is written, so that a plane
/// the hardware sends twice carries one number and appears twice.
void stampPacketNumber(uint8_t* buffer, uint32_t frames) {
	if constexpr (!kAudioStamps) {
		return;
	}
	int16_t* samples = (int16_t*)buffer;
	const int16_t number = encodeStamp(packetCounter++);
	for (uint32_t f = 0; f < frames; f++) {
		samples[f * kChannels + 5] = number;
	}
}

/// Fills the standby packet once, at stream start. Everything in it is constant except the packet number,
/// which the timer stamps as it goes out so the host-side tools still see a sequence rather than one long
/// repeated value.
void prepareStandbyPacket() {
	const uint32_t frames = kFramesPerPacket / kFramesPerBlock * kFramesPerBlock;
	uint8_t* buffer = (uint8_t*)((uint32_t)standbyPacket.data + UNCACHED_MIRROR_OFFSET);
	memset(buffer, 0, frames * kFrameBytes);
	markDeviceSilence(buffer, frames);
	standbyBytes = (uint16_t)(frames * kFrameBytes);
	standbyReady = true;
}

/// Numbers the standby packet on its way out. Forty-odd 16-bit stores through the uncached view, and only when
/// the queue is empty, which by construction means the task is not running and the interrupt has nothing to
/// compete with.
void stampStandbyPacket() {
	if constexpr (!kAudioStamps) {
		return;
	}
	int16_t* samples = (int16_t*)((uint32_t)standbyPacket.data + UNCACHED_MIRROR_OFFSET);
	const int16_t number = encodeStamp(packetCounter++);
	const uint32_t frames = standbyBytes / kFrameBytes;
	for (uint32_t f = 0; f < frames; f++) {
		samples[f * kChannels + 5] = number;
	}
}

/// Builds one packet from the ring and hands it to the endpoint.
///
/// Called from the task for the first packet of a stream and after a recovery, and from the completion for every
/// packet after that. Both are single-threaded against each other: the task only builds when no transfer is
/// outstanding, and a completion can only arrive when one is.
///
/// submit() is deliberately the last statement, because the completion for it can arrive before this returns.
/// Builds one packet into the caller's buffer and returns its size in audio frames.
///
/// Split out of sendNextPacket() because two paths now need it - the submit path and the direct second-plane
/// write - and the sizing, priming, resync and underrun rules must be identical for both. Duplicating them is
/// how a rule enforced at two sites ends up enforced at one.
uint32_t buildNextPacket(uint8_t* buffer) {
	bump(statBuilds);
	// Nominal packet size, used only while there is nothing real to send. 44.1 kHz does not divide into 1 ms
	// frames, so 44 frames with a 45th every tenth packet averages exactly 44100 rather than drifting.
	// Even sizes only, so the long-run average is held by adding a whole block every twentieth packet rather than
	// a single frame every tenth: 100 per packet against a 2000 threshold is 44 + 2/20 = 44.1, the same average
	// the odd-sized version produced.
	uint32_t frames = kFramesPerPacket / kFramesPerBlock * kFramesPerBlock;
	frameAccumulator += kFrameRemainder;
	if (frameAccumulator >= 1000u * kFramesPerBlock) {
		frames += kFramesPerBlock;
		frameAccumulator -= 1000u * kFramesPerBlock;
	}

	// Signed, and the pointers are free-running rather than masked to the ring: a masked subtraction cannot
	// go negative, so a reader one frame past the writer read as a full ring and triggered a resync that threw
	// away thousands of frames that never existed. Measured firing 2.5 times a second under load, 2026-08-26.
	const int32_t available32 = (int32_t)(writeFrame - readFrame);
	uint32_t available = (available32 > 0) ? (uint32_t)available32 : 0u;

	// Hold the first packets back until there is a window's worth of slack, so the very first late render does
	// not produce a gap the host has to hear.
	if (!primed) {
		// One packet above the cushion, not level with it. This mattered when the builder sent
		// `available - kLeadFrames` and starting level with the cushion left the first packet nothing to
		// send; the trimmed builder would simply send its nominal size. Kept because the headroom costs
		// nothing and the stream still wants a full window in hand before the first packet leaves.
		if (available < kLeadFrames + kMaxFramesPerPacket) {
			bump(statPrimeSilence);
			memset(buffer, 0, frames * kFrameBytes);
			markDeviceSilence(buffer, frames);
			stampPacketNumber(buffer, frames);
			return frames;
		}
		primed = true;
		// Start centred. Priming waits for a whole packet *above* the cushion, so without this the first real
		// packet begins its life one packet clear of the set-point and the trim spends the opening second
		// working it back down - which is when the host has least margin to spare. Nothing is discarded: the
		// frames between here and the write pointer have not been sent, and the lead is what the stream carries
		// rather than what it owes.
		readFrame = writeFrame - kLeadFrames;
		bump(statReadAdvResync);
		available = kLeadFrames;
	}

	// The host is not draining us fast enough to keep up. A rate trim cannot recover this in useful time, so
	// put the read pointer back at the lead and take the one discontinuity.
	if (available >= kResyncFrames) {
		// Signed, deliberately: the masked `available` above cannot tell "the ring holds 8191 frames" from "the
		// reader is one frame past the writer". This can, and the two want opposite responses.
		if constexpr (kDiagnostics) {
			const int32_t signedLead = available32;
			if (signedLead < 0) {
				statResyncOvertaken++;
				if (signedLead < statWorstOvertake) {
					statWorstOvertake = signedLead;
				}
			}
			else {
				statResyncGenuine++;
			}
		}
		bump(statFramesDiscarded, available - kLeadFrames);
		readFrame = writeFrame - kLeadFrames;
		bump(statReadAdvResync);
		available = kLeadFrames;
		bump(statResyncs);
	}

	if (available < frames) {
		// Not enough for a full-sized packet. Send silence rather than a runt.
		//
		// This threshold was one block - two frames - so a ring holding four frames produced a four-frame
		// packet. Measured on the load stress of 2026-08-29 (evening): in the half-second windows that still
		// carried a break, the smallest packet had a median of 4 frames against 44 everywhere else, and 98% of
		// the remaining breaks sat within a second of a stall. It is the same fault the even-packet rule was
		// written to remove, surviving in the one branch that rounding down to whole blocks could still reach:
		// a run of runts starves the host exactly as a run of short packets did.
		//
		// The audio is not lost and the read pointer does not move - those frames go out in the next packet,
		// once there are enough of them to fill one. A silent packet costs the ring nothing, and the engine
		// refills it at 44.1 frames a millisecond, so this clears itself within a packet or two of the stall
		// ending.
		bump(statUnderruns);
		memset(buffer, 0, frames * kFrameBytes);
		markDeviceSilence(buffer, frames);
		stampPacketNumber(buffer, frames);
		return frames;
	}

	// A near-constant packet size, trimmed a block at a time towards the lead set-point.
	//
	// This used to send `available - kLeadFrames`, which regulated the lead perfectly and put the whole of the
	// engine's burstiness onto the wire. Read back off the 2026-08-29 capture: packets ranged from 2 frames to
	// 62 with a median of 60, so the instantaneous rate the host saw swung between roughly 2,000 and 62,000
	// frames a second while averaging exactly 44,100. That average is what every device-side counter reports,
	// and it is why all of them read clean.
	//
	// A host input ring cannot absorb that. Its client reads at a fixed rate behind the device's writes, and
	// when a run of short packets exhausts the margin the reader overtakes the writer and serves audio one ring
	// lap - ~250 ms - old until the device catches up. That is exactly the shape measured in the recording: 18
	// stale runs of 2-108 frames at a median depth of 11,008 frames, each with a step across the join 33x the
	// music's own. Confirmed by ear on 2026-08-29 (evening), blind, against a repaired copy.
	//
	// So the ring absorbs the burstiness instead, which is what it is for. The nominal size above already holds
	// 44.1 frames a packet; the lead is corrected a block at a time when it strays outside a band, which tracks
	// the engine's real long-run rate without ever putting a step onto the wire. Simulated against the engine's
	// 128-frame bursts before flashing: packet sizes 44-48 against 2-62, delivered rate 44,099 frames/s against
	// 44,091, and the lead holding 184-328 around its 256 set-point with no underrun and no resync.
	//
	// Never a partial audio frame: a host that appends one rotates the channel mapping permanently and
	// silently.
	const int32_t leadError = (int32_t)available - (int32_t)kLeadFrames;
	uint32_t send = frames;
	if (leadError > (int32_t)kLeadCatchUpFrames) {
		// Far above the set-point - a stalled task, or the recovery after one. Four blocks, not the whole
		// backlog: 52 frames a packet drains ~7,800 frames a second faster than the engine produces, so the
		// ring returns to the set-point without ever putting a step on the wire. Simulated against a 100 ms
		// task starve: 0.57 s to recover at four blocks against 1.26 s at two, for a rate excursion of 18%
		// against the 40% the old rule took on every ordinary packet.
		send = frames + 4u * kFramesPerBlock;
	}
	else if (leadError > (int32_t)kLeadBandFrames) {
		send = frames + kFramesPerBlock;
	}
	else if (leadError < -(int32_t)kLeadBandFrames) {
		// Below the cushion, which after priming means the engine ran late. One block short rather than a
		// fraction of what is left: the previous rule took an eighth here, which is a 4-frame packet, and a
		// run of those is precisely what makes the host's reader overtake.
		send = (frames > kFramesPerBlock) ? frames - kFramesPerBlock : kFramesPerBlock;
		bump(statLeadShort);
	}
	if (send > available) {
		send = available;
	}
	if (send > kMaxFramesPerPacket) {
		send = kMaxFramesPerPacket;
	}
	// Down to a whole block, never up: rounding up would read frames the engine has not written yet. The frame
	// left behind is sent by the next packet, so nothing is lost - the builder is self-regulating and takes
	// whatever the ring holds above the cushion.
	send = send / kFramesPerBlock * kFramesPerBlock;
	if (send == 0u) {
		send = kFramesPerBlock;
	}

	const uint32_t start = readFrame & kRingMask;
	const uint32_t firstRun = (start + send > kRingFrames) ? (kRingFrames - start) : send;
	memcpy(buffer, &ring[start * kChannels], firstRun * kFrameBytes);
	if (firstRun < send) {
		memcpy(&buffer[firstRun * kFrameBytes], &ring[0], (send - firstRun) * kFrameBytes);
	}
	readFrame = readFrame + send;
	bump(statReadAdvBuild);

	bump(statPacketsSent);
	bump(statFramesSent, send);
	if constexpr (kDiagnostics) {
		// Bins of eight, so bin 7 is 56-63 frames - a packet at the endpoint's ceiling.
		statSizeHist[(send >> 3u) < kSizeBins ? (send >> 3u) : (kSizeBins - 1u)]++;
	}

	stampPacketNumber(buffer, send);

	return send;
}

void sendNextPacket() {
	uint8_t* buffer = nextPacketBuffer();
	const uint32_t frames = buildNextPacket(buffer);
	submit(buffer, frames * kFrameBytes);
}

/// Ends a transfer the hardware has already finished with but never reported.
///
/// An isochronous IN the device is not ready for ends with a not-ready event rather than the buffer-empty one that
/// carries our completion, and usb_pstd_interrupt_handler clears every not-ready bit and returns "all dealt with"
/// before usb_pstd_interrupt is ever called. So nothing downstream can see one, and a transfer ended that way stays
/// outstanding for good.
///
/// Detected from the pipe instead: an empty IN buffer and an idle pipe, while we still hold the in-flight flag, is
/// the hardware saying it has nothing outstanding while we think it has. That is the measured stall state exactly
/// (PIPE1CTR 0x8001, 2026-08-23), and it does not need the not-ready bit, which cannot be caught from a task -
/// polling for it saw it on 5 to 9 passes a second out of a thousand because the handler wipes it in between.
///
/// Two consecutive passes, because there is a microsecond between the host taking a packet and the interrupt that
/// says so, in which a healthy transfer looks like this one. A millisecond apart, both readings cannot land in it.
void endAbandonedTransfer() {
	const PipeRegisters regs = readPipeRegisters();

	if ((regs.pipectr & (kPipeCtrInBufM | kPipeCtrPBusy)) != 0) {
		stuckPasses = 0;
		return;
	}
	bump(statAbandonedSeen);

	if (++stuckPasses < 2) {
		return;
	}
	stuckPasses = 0;

	bump(statAbandonedEnded);
	// Through the driver's own path rather than by clearing our flag: that releases g_p_usb_pipe too, and
	// submitting over a pipe the driver still believes is busy is what froze the machine on 2026-08-22.
	usb_pstd_forced_termination(USB_CFG_PAUDIO_ISO_IN, kUsbDataStop);
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

/// DIAGNOSTIC. The loudest thing any channel has carried since the last line, at capture scale and therefore
/// before the trim and the width reduction that would clip it.
///
/// Its own line rather than a field on the cost line, which already runs 243 of its 256 bytes with three of its
/// counters still climbing - a debug line that outgrew its buffer froze this machine once already.
///
/// This is the instrument the trim is set from: 32767 here means the stem would have flat-topped at unity, and
/// what the trim has to do is bring it under that with room to spare.
void reportStemPeak() {
	if (!kDiagnostics || Debug::midiDebugCable == nullptr) {
		return;
	}
	// Measured at roughly three lines a second at 600, so the scheduler runs this nearer 2 kHz than the few
	// hundred hertz assumed. A peak-hold wants reading at about the rate a person can read it.
	if (++stemPeakCountdown < 4000u) {
		return;
	}
	stemPeakCountdown = 0;

	// "SP pk4294967295 tr50" is 20 characters. Sized for both fields at their full width rather than for the line
	// as it reads today.
	char line[32];
	uint32_t at = 0;
	const auto put = [&](const char* text) {
		for (uint32_t i = 0; text[i] != 0 && at + 1 < sizeof(line); i++) {
			line[at++] = text[i];
		}
	};
	const auto putNumber = [&](uint32_t value) {
		char digits[12];
		uint32_t n = 0;
		do {
			digits[n++] = (char)('0' + (value % 10));
			value /= 10;
		} while (value != 0 && n < sizeof(digits));
		while (n > 0 && at + 1 < sizeof(line)) {
			line[at++] = digits[--n];
		}
	};

	const int32_t peak = deluge::processing::engines::USBAudioStream::readAndClearStemPeak();
	put("SP pk");
	// The same scale a channel reaches the host on, but deliberately NOT through the saturating conversion: a
	// figure that pins at 32767 cannot say how far past 32767 it is, which is the only question the trim needs
	// answered. Anything above 32767 here is what the trim has to bring down.
	putNumber((uint32_t)((peak >> 9) < 0 ? 0 : (peak >> 9)));
	put(" tr");
	putNumber(deluge::processing::engines::USBAudioStream::getTrim());
	line[at] = 0;
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

	// Once the host has actually selected the streaming interface: before that the driver has not configured the
	// pipe, so an earlier read would report the state of an endpoint that does not exist yet.
	if (!pipeConfigRead && streamActive && transferInitialised) {
		readPipeConfig();
	}

	// Sized for every counter at its full width rather than for the line as it reads today. Growing this line past
	// its buffer is the likeliest cause of the freeze on 2026-08-22.
	//
	// 512 rather than 256: it was already claiming to be sized for full width and was not. Measured at 243 bytes
	// of 256 on 2026-08-27, with `ur`, `rs` and `frm` cumulative and still climbing - a long enough stream would
	// have smashed the stack on its own, once a second, with no code change at all. 42 fields at a five-character
	// tag and ten digits apiece is 630 worst case; 512 covers every width these counters reach in practice and
	// leaves room for the next one.
	char line[512];
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
	emit(" pkt");
	emitDec(statPacketsSent);
	emit(" frm");
	emitDec(statFramesSent);
	emit(" bmp");
	emitDec(usbBempNonZeroCount);
	emit(" bau");
	emitDec(usbBempAudioCount);
	emit(" bbt");
	emitDec(usbBempBitsSeen);
	emit(" bst");
	emitDec(usbBempAudioStall);
	emit(" brd");
	emitDec(usbBrdyNonZeroCount);
	emit(" bra");
	emitDec(usbBrdyAudioCount);
	emit(" ur");
	emitDec(statUnderruns);
	emit(" lsh");
	emitDec(statLeadShort);
	emit(" nrs");
	emitDec(statAbandonedSeen);
	emit(" nre");
	emitDec(statAbandonedEnded);
	emit(" rs");
	emitDec(statResyncs);
	emit(" brw");
	emitDec(statPortBorrowed);
	emit(" ssk");
	emitDec(statSubmitSkipped);
	emit(" tf");
	emitDec(statTimerFires);
	emit(" tw");
	emitDec(statTimerWrote);
	emit(" tsh");
	emitDec(statTimerShut);
	emit(" sof");
	emitDec(statSofSeen);
	emit(" fri");
	emitDec(statFrdyImmediate);
	emit(" frn");
	emitDec(statFrdyNever);
	emit(" cup");
	emitDec(statCatchUp);
	emit(" sofo");
	emitDec(sofToWriteTicks * 1000u / kTimerTicksPerMs); ///< the settled offset, in us
	emit(" tem");
	emitDec(statTimerEmpty);
	// Packets of marked silence sent because the builder was frozen. Read against ur: an under-run is the ring
	// dry with the task still running, this is the task not running at all.
	emit(" sby");
	emitDec(statStandbySent);
	emit(" qb");
	emitDec(statQueueBuilt);
	emit(" bld");
	emitDec(statBuilds);
	emit(" rgen");
	emitDec(statResyncGenuine);
	emit(" rovr");
	emitDec(statResyncOvertaken);
	emit(" wovr");
	emitDec((uint32_t)(-statWorstOvertake));
	emit(" fin");
	emitDec(statFramesIn);
	emit(" fdis");
	emitDec(statFramesDiscarded);
	emit(" lead");
	emitDec((uint32_t)(int32_t)(writeFrame - readFrame));
	*p = '\0';
	Debug::sysexDebugPrint(*Debug::midiDebugCable, line, true);

	// DIAGNOSTIC. Its own line: the counters line above already runs near its buffer at full counter width.
	char histLine[128];
	p = histLine;
	emit("AUS");
	for (uint32_t bin = 0; bin < kSizeBins; bin++) {
		emit(" h");
		emitDec(bin);
		emit(":");
		emitDec(statSizeHist[bin]);
	}
	*p = '\0';
	Debug::sysexDebugPrint(*Debug::midiDebugCable, histLine, true);

	// Second line rather than more fields on the first: at full counter width that one already runs near its
	// buffer, and growing a fixed-size debug line past it is the likeliest cause of the freeze on 2026-08-22.
	// Live values first, then the same registers as they stood at the last completion.
	char regLine[128];
	p = regLine;
	auto emitHex = [&p](uint32_t v) {
		for (int shift = 12; shift >= 0; shift -= 4) {
			*p++ = "0123456789ABCDEF"[(v >> shift) & 0xF];
		}
	};

	const PipeRegisters now = readPipeRegisters();
	emit("AUR pc");
	emitHex(now.pipectr);
	emit(" be");
	emitHex(now.bempenb);
	emit(" nr");
	emitHex(now.nrdysts);
	emit(" fn");
	emitHex(now.frmnum);
	// Dashes rather than zeros when no completion has ever happened: a zero reading and no reading at all are
	// different findings, and this build exists because they have been confused before.
	if (regsSnapshotTaken) {
		emit(" Lpc");
		emitHex(regsAtLastCompletion.pipectr);
		emit(" Lbe");
		emitHex(regsAtLastCompletion.bempenb);
		emit(" Lnr");
		emitHex(regsAtLastCompletion.nrdysts);
		emit(" Lfn");
		emitHex(regsAtLastCompletion.frmnum);
	}
	else {
		emit(" Lpc---- Lbe---- Lnr---- Lfn----");
	}
	*p = '\0';
	Debug::sysexDebugPrint(*Debug::midiDebugCable, regLine, true);

	// Third line rather than more fields on the second: that one already runs near half its buffer at full counter
	// width, and growing a fixed-size debug line past its buffer is the likeliest cause of the freeze on 2026-08-22.
	if (pipeConfigRead) {
		char cfgLine[128];
		p = cfgLine;
		emit("AUC cf");
		emitHex(pipeConfig.pipecfg);
		emit(" bf");
		emitHex(pipeConfig.pipebuf);
		emit(" mp");
		emitHex(pipeConfig.pipemaxp);
		emit(" pr");
		emitHex(pipeConfig.pipeperi);
		// Read fresh every report, unlike the hardware values above: if the driver reconfigures the pipe mid-stream
		// these move and the snapshot does not, which is the only way to notice without steering the selector again.
		emit(" Dcf");
		emitHex(pipeCfgs[USB_CFG_PAUDIO_ISO_IN]);
		emit(" Dbf");
		emitHex(pipeBufs[USB_CFG_PAUDIO_ISO_IN]);
		emit(" Dmp");
		emitHex(pipeMaxPs[USB_CFG_PAUDIO_ISO_IN]);
		*p = '\0';
		Debug::sysexDebugPrint(*Debug::midiDebugCable, cfgLine, true);
	}

	// Fourth line, own buffer for the same reason as the third. wd is how many frames the write straddled; cg is
	// how many frames apart consecutive completions landed.
	char timingLine[128];
	p = timingLine;
	emit("AUT wd0:");
	emitDec(statWriteFrameDelta[0]);
	emit(" wd1:");
	emitDec(statWriteFrameDelta[1]);
	emit(" wd2+:");
	emitDec(statWriteFrameDelta[2]);
	emit(" cg0:");
	emitDec(statCompletionGap[0]);
	emit(" cg1:");
	emitDec(statCompletionGap[1]);
	emit(" cg2:");
	emitDec(statCompletionGap[2]);
	emit(" cg3:");
	emitDec(statCompletionGap[3]);
	emit(" cg4+:");
	emitDec(statCompletionGap[4]);
	*p = '\0';
	Debug::sysexDebugPrint(*Debug::midiDebugCable, timingLine, true);

	// Fifth line, own buffer for the same reason as the third and fourth. fr1/fr0 are every submit's outcome; the
	// latched f/pc pair is the second-plane answer, and FRDY (0x2000) still set in f2 says a plane was free after
	// the second write, which means it did not reach one.
	// 256 rather than 160: every counter on this line is cumulative and none of them is cleared, so it has to be
	// sized for ten digits apiece rather than for the widths it prints today. Growing a fixed-size debug line past
	// its buffer is the likeliest cause of the freeze on 2026-08-22.
	char planeLine[256];
	p = planeLine;
	// dfr is cumulative and never cleared: whether a second plane has *ever* held data is the question, not how
	// often, and a per-report count of zero would look the same as never having been armed.
	emit("AUP dfr:");
	emitDec(usbBempAudioDeferred);
	// Cumulative for the same reason as dfr: whether the FIFO has ever refused a write is the question.
	emit(" we:");
	emitDec(usbAudioWriteEnd[0]);
	emit(",");
	emitDec(usbAudioWriteEnd[1]);
	emit(",");
	emitDec(usbAudioWriteEnd[2]);
	emit(",");
	emitDec(usbAudioWriteEnd[3]);
	emit(" ft:");
	emitDec(usbAudioForcedTerm);
	emit(" fr1:");
	emitDec(statFrdyAfterSubmit[1]);
	emit(" fr0:");
	emitDec(statFrdyAfterSubmit[0]);
	emit(" bad:");
	emitDec(statProbeNotOurs);
	// Started against completed says whether every transfer the timer armed actually retired; skips say how often
	// a fire found the controller still working. Per interval, unlike the cumulative fields beside them.
	emit(" dma:");
	emitDec(statDmaStarted);
	emit("/");
	emitDec(statDmaCompleted);
	emit(" dsk:");
	emitDec(statDmaBusySkips);
	emit(" dlr:");
	emitDec(statDmaLateRetires);
	// Mean and worst arm-to-commit in microseconds, then how many of those commits landed in the host frame they
	// were armed in, one frame later, and two or more later. A packet committed late waits for the next token.
	emit(" dus:");
	emitDec(statDmaCompleted ? (statDmaSpanSum / statDmaCompleted / 400u) : 0u);
	emit("/");
	emitDec(statDmaSpanMax / 400u);
	emit(" dfs:");
	emitDec(statDmaFrameSpan[0]);
	emit(",");
	emitDec(statDmaFrameSpan[1]);
	emit(",");
	emitDec(statDmaFrameSpan[2]);
	*p = '\0';
	Debug::sysexDebugPrint(*Debug::midiDebugCable, planeLine, true);

	// DIAGNOSTIC, removed with kSweepOffsets. One field per offset bin, writes/fires, cumulative across passes.
	// Both halves are printed because a ratio alone cannot show a bin that never ran - and a bin that never ran
	// is the failure this whole line exists to make visible.
	if (kSweepOffsets) {
		// 512 for the same reason the AU line is, computed rather than guessed: 18 bins at a three-digit offset
		// and ten digits apiece either side of the slash is 495 bytes worst case. Static rather than stack -
		// this report already carries ~1.6 KB of buffers in one frame, and the sweep is not worth a stack
		// overflow on a path that runs once a second. Single caller, no reentrancy.
		static char sweepLine[512];
		p = sweepLine;
		emit("AUS p");
		emitDec(sweepPasses);
		emit(" b");
		emitDec(sweepBin);
		for (uint32_t i = 0; i < kSweepBins; i++) {
			emit(" ");
			emitDec(kSweepFirstUs + i * kSweepStepUs);
			emit(":");
			emitDec(sweepWrites[i]);
			emit("/");
			emitDec(sweepFires[i]);
		}
		*p = '\0';
		Debug::sysexDebugPrint(*Debug::midiDebugCable, sweepLine, true);
	}

	// Sixth line, own buffer for the reason the others carry. ts is the task-side 2x2 over the whole cycle;
	// pf/pm/pi is the bounded poll straight after the write, at a resolution the task cannot reach.
	char readyLine[192];
	p = readyLine;
	emit("AUW ts0:");
	emitDec(statReadyState[0]);
	emit(" ts1:");
	emitDec(statReadyState[1]);
	emit(" ts2:");
	emitDec(statReadyState[2]);
	emit(" ts3:");
	emitDec(statReadyState[3]);
	emit(" pf:");
	emitDec(statPollFound);
	emit(" pm:");
	emitDec(statPollMissed);
	emit(" xw:");
	emitDec(statExtraWritten);
	emit(" xr:");
	emitDec(statExtraRefused);
	emit(" xn:");
	emitDec(statExtraNoWindow);
	emit(" pi:");
	if (statPollFirstIteration == 0xFFFFFFFFu) {
		// Dashes rather than a sentinel number: never seen and seen at iteration 4294967295 are different
		// findings, and confusing the two is what this build exists to avoid.
		emit("----");
	}
	else {
		emitDec(statPollFirstIteration);
	}
	*p = '\0';
	Debug::sysexDebugPrint(*Debug::midiDebugCable, readyLine, true);

	{
		char ptrLine[160];
		p = ptrLine;
		emit("AUD wf");
		emitDec(writeFrame);
		emit(" rf");
		emitDec(readFrame);
		emit(" qw");
		emitDec(queueWrite);
		emit(" qr");
		emitDec(queueRead);
		emit(" advB");
		emitDec(statReadAdvBuild);
		emit(" advR");
		emitDec(statReadAdvResync);
		emit(" advP");
		emitDec(statReadAdvPark);
		emit(" alt");
		emitDec(g_usb_pstd_alt_num[kAudioInterfaceNumber]);
		emit(" con");
		emitDec(g_usb_peri_connected);
		*p = '\0';
		Debug::sysexDebugPrint(*Debug::midiDebugCable, ptrLine, true);
	}

	// DIAGNOSTIC. The return, host to device. Own line and own buffer, so the outgoing half's line does not grow
	// and the two directions can be read apart.
	//
	// Counted rather than claimed, field by field, because a debug line that outgrew its stack buffer froze this
	// machine once already and the comment saying it was sized for the worst case is what stopped anyone
	// re-checking. Tags including their leading space, then ten digits for each unsigned counter:
	//   AUI alt 17 | act 14 | inf 14 | brdy 15 | arm 14 | aerr 15 | err 14 | pkt 14 | frm 14 | mt 13 |
	//   part 15 | ovr 14 | unr 14 | drn 14 | wf 13 | rf 13 | held 15 | pr 4
	//   sz 3 + 4 buckets at 11 = 47 | pk 3 + kRxChannels at 11 = 25 at two channels
	// 335 worst case, plus the terminator. 384 leaves room for two more fields; widening the return past two
	// channels adds 11 apiece and must be re-counted here.
	{
		char rxLine[384];
		p = rxLine;
		emit("AUI alt");
		emitDec(g_usb_pstd_alt_num[kReturnInterfaceNumber]);
		emit(" act");
		emitDec(returnActive ? 1u : 0u);
		emit(" inf");
		emitDec(rxTransferInFlight ? 1u : 0u);
		// Interrupts the chip raised for this pipe, counted in the driver. A zero here with a live host is the
		// host not sending; a rising count with frm stuck at zero is the read path rather than the wire.
		emit(" brdy");
		emitDec(usbBrdyReturnCount);
		emit(" arm");
		emitDec(statRxArms);
		emit(" aerr");
		emitDec(statRxArmErr);
		emit(" err");
		emitDec(statRxLastErr);
		emit(" pkt");
		emitDec(statRxPackets);
		emit(" frm");
		emitDec(statRxFrames);
		emit(" mt");
		emitDec(statRxEmpty);
		emit(" part");
		emitDec(statRxPartial);
		emit(" ovr");
		emitDec(statRxOverrun);
		emit(" unr");
		emitDec(statRxUnderrun);
		emit(" drn");
		emitDec(statRxDrained);
		// The ring's own position. held is the reader's lead in frames, which is the latency B3 inherits.
		emit(" wf");
		emitDec(rxWriteFrame);
		emit(" rf");
		emitDec(rxReadFrame);
		emit(" held");
		emitDec(rxWriteFrame - rxReadFrame);
		emit(" pr");
		emitDec(rxPrimed ? 1u : 0u);
		// Packet sizes, in audio frames: under 44 / 44 / 45 / over 45. A host that is not sending 44.1 kHz shows
		// here before anything else notices.
		emit(" sz");
		for (uint32_t i = 0; i < 4; i++) {
			emit(i == 0 ? ":" : ",");
			emitDec(statRxSizes[i]);
		}
		// Peak per channel. The only field that separates a host sending silence from a host sending music -
		// every other number above reads the same either way.
		emit(" pk");
		for (uint32_t c = 0; c < kRxChannels; c++) {
			emit(c == 0 ? ":" : ",");
			emitDec((uint32_t)rxPeak[c]);
		}
		*p = '\0';
		Debug::sysexDebugPrint(*Debug::midiDebugCable, rxLine, true);
	}

	// DIAGNOSTIC. Where the CPU this build costs the instrument actually goes, so the next change is aimed at a
	// measured share rather than at the one candidate that got tested. Own buffer for the reason the others carry.
	//
	// 256: "AUX el" plus ten digits is 16, and six fields of a four-character tag, a colon and three ten-digit
	// numbers separated by commas is 6 x 38 = 228. 244 worst case, and the two refusal lines are shorter than
	// either.
	{
		// Eleven fields of <tag>:<permille>,<calls>,<mean>. Worst case a tag is 7 characters, permille 4 digits,
		// calls and mean 10 each, plus two commas: 33. Eleven of those is 363, and the header "AUX el" with a
		// ten-digit interval is 16 more. 448 leaves room for two further fields. Nothing below bounds-checks, and
		// a debug line that outgrew its buffer froze this machine once already - so this is counted, not claimed.
		char costLine[448];
		p = costLine;
		// A counter that was never enabled reads a constant, and a constant produces a confident zero on every
		// field below rather than an error. Two reads that come back equal mean the instrument is dead, and it
		// says so instead of answering.
		const uint32_t live0 = Debug::readCycleCounter();
		const uint32_t live1 = Debug::readCycleCounter();
		const uint32_t elapsed = live1 - costIntervalStart;
		// Nested paths deliberately excluded: fifoWrite is inside timer, build and submit are inside service.
		const uint64_t topLevel = (uint64_t)costTimer.cycles + (uint64_t)costService.cycles
		                          + (uint64_t)costFeedMix.cycles + (uint64_t)costStemSnapshot.cycles
		                          + (uint64_t)costStemCapture.cycles + (uint64_t)costStemClear.cycles;
		if (live1 == live0) {
			emit("AUX dead");
		}
		else if (elapsed == 0u || topLevel > (uint64_t)elapsed) {
			// The interval outran the counter's 10.7 s wrap, or something is being double-counted. Either way the
			// ratio would be arithmetic on a wrong denominator, so print the raw cycles and refuse the ratio.
			emit("AUX bad el");
			emitDec(elapsed);
			emit(" top");
			emitDec((uint32_t)topLevel);
		}
		else {
			// Each field is <tag>:<parts per thousand of wall time>,<calls this interval>,<mean cycles per call>.
			// Per mille rather than per cent because two of these are expected to land under 1%, and a mean per
			// call because a path that is expensive per event and a path that is merely frequent want opposite
			// fixes.
			auto emitCost = [&emit, &emitDec, elapsed](const char* tag, const PathCost& c) {
				emit(tag);
				emitDec((uint32_t)(((uint64_t)c.cycles * 1000u) / elapsed));
				emit(",");
				emitDec(c.calls);
				emit(",");
				emitDec(c.calls ? (c.cycles / c.calls) : 0u);
			};
			emit("AUX el");
			emitDec(elapsed / 400u); ///< the interval in us, at 400 MHz
			emitCost(" tim:", costTimer);
			emitCost(" fifo:", costFifoWrite);
			emitCost(" svc:", costService);
			emitCost(" bld:", costBuild);
			emitCost(" sub:", costSubmit);
			emitCost(" mix:", costFeedMix);
			emitCost(" dst:", costDmaStart);
			emitCost(" dcp:", costDmaComplete);
			emitCost(" snap:", costStemSnapshot);
			emitCost(" cap:", costStemCapture);
			emitCost(" clr:", costStemClear);
			emitCost(" eng:", costEngine);
			emitCost(" outs:", costOutputs);
		}
		*p = '\0';
		Debug::sysexDebugPrint(*Debug::midiDebugCable, costLine, true);
		costIntervalStart = live1;
		costTimer = PathCost{};
		costFifoWrite = PathCost{};
		costService = PathCost{};
		costBuild = PathCost{};
		costSubmit = PathCost{};
		costFeedMix = PathCost{};
		costDmaStart = PathCost{};
		costDmaComplete = PathCost{};
		costStemSnapshot = PathCost{};
		costStemCapture = PathCost{};
		costStemClear = PathCost{};
		costEngine = PathCost{};
		costOutputs = PathCost{};
	}

	statPacketsSent = 0;
	statFramesSent = 0;
	statFramesIn = 0;
	statFramesDiscarded = 0;
	statSubmitSkipped = 0;
	statPortBorrowed = 0;
	statReadAdvBuild = 0;
	statReadAdvResync = 0;
	statReadAdvPark = 0;
	statTimerFires = 0;
	statTimerWrote = 0;
	statTimerShut = 0;
	statSofSeen = 0;
	statFrdyImmediate = 0;
	statFrdyNever = 0;
	statCatchUp = 0;
	statTimerEmpty = 0;
	statStandbySent = 0;
	statQueueBuilt = 0;
	statBuilds = 0;
	for (uint32_t i = 0; i < kSizeBins; i++) {
		statSizeHist[i] = 0;
	}
	statResyncGenuine = 0;
	statResyncOvertaken = 0;
	// statWorstOvertake is deliberately not cleared: the furthest the reader has ever got past the writer is
	// the question, not how far it got in the last second.
	statAlt1 = 0;
	statInFlight = 0;
	statPrimeSilence = 0;
	// Per-interval, unlike statUnderruns beside it on the line: the question is how often the builder is short
	// right now, not how many times it has ever been.
	statLeadShort = 0;
	statCompletes = 0;
	statSubmits = 0;
	statAbandonedSeen = 0;
	statAbandonedEnded = 0;
	usbBempNonZeroCount = 0;
	usbBempAudioCount = 0;
	usbBrdyNonZeroCount = 0;
	usbBrdyAudioCount = 0;
	for (uint32_t i = 0; i < 3; i++) {
		statWriteFrameDelta[i] = 0;
	}
	for (uint32_t i = 0; i < 5; i++) {
		statCompletionGap[i] = 0;
	}
	for (uint32_t i = 0; i < 4; i++) {
		statReadyState[i] = 0;
	}
	statPollFound = 0;
	statPollMissed = 0;
	statExtraWritten = 0;
	statExtraRefused = 0;
	statExtraNoWindow = 0;
	// statPollFirstIteration is deliberately not cleared: the fastest the window has ever been seen to open is
	// the question, not how often it did in the last second.
	statFrdyAfterSubmit[0] = 0;
	statFrdyAfterSubmit[1] = 0;
	statProbeNotOurs = 0;
	statDmaStarted = 0;
	statDmaCompleted = 0;
	statDmaBusySkips = 0;
	statDmaLateRetires = 0;
	statDmaSpanSum = 0;
	statDmaSpanMax = 0;
	for (uint32_t i = 0; i < 3; i++) {
		statDmaFrameSpan[i] = 0;
	}
	// Every counter on the AUI line is per-interval, so the whole line reads as one instrument at one rate. The
	// exceptions are stated on the line itself: wf, rf and held are positions rather than counts, and err is the
	// last code rather than a tally.
	statRxPackets = 0;
	statRxFrames = 0;
	statRxEmpty = 0;
	statRxPartial = 0;
	statRxOverrun = 0;
	statRxUnderrun = 0;
	statRxDrained = 0;
	statRxArms = 0;
	statRxArmErr = 0;
	usbBrdyReturnCount = 0;
	for (uint32_t i = 0; i < 4; i++) {
		statRxSizes[i] = 0;
	}
	for (uint32_t c = 0; c < kRxChannels; c++) {
		rxPeak[c] = 0;
	}

	// usbBempBitsSeen is deliberately not cleared: which pipes are live at all is the question, not how often.
}

/// Channel 0, the first of the two the vendor's USB configuration already names and one of the five the CPU's own
/// channel map (RZA1/cpu_specific.h) lists as unallocated.
constexpr uint32_t kAudioDmaChannel = 0;

/// The request line the USB peripheral raises for D0FIFO on USB0, taken from the vendor driver's own table
/// (r_usb_dma.c:1370). It means "the transmit FIFO will accept data", so the controller moves a packet only at a
/// moment the CPU could have written one - the same window, without the CPU in it.
constexpr uint32_t kAudioDmaRequest = 0x00000083u;

/// Values rather than an include: the vendor's r_usb_dmac.h declares a whole driver's worth of types alongside
/// these and has never been compiled in this fork. Each is named here as it is named there, r_usb_dmac.h:68-131.
constexpr uint32_t kDmaChcfgAmBusCycle = 0x00000200u;   ///< AM = bus cycle mode
constexpr uint32_t kDmaChcfgLevel = 0x00000040u;        ///< LVL: the request is a level, not an edge
constexpr uint32_t kDmaChcfgHighEnable = 0x00000020u;   ///< HIEN: active high
constexpr uint32_t kDmaChcfgSelCh0 = 0x00000000u;       ///< SEL = channel 0
constexpr uint32_t kDmaChcfgDest256 = 0x00050000u;      ///< DDS: 256-bit, a 32-byte block per bus cycle
constexpr uint32_t kDmaChcfgDestFixed = 0x00200000u;    ///< DAD: the FIFO register does not advance
constexpr uint32_t kDmaChcfgSrc256 = 0x00005000u;       ///< SDS: 256-bit reads from the packet
constexpr uint32_t kDmaChcfgReqDest = 0x00000008u;      ///< REQD: the requesting module is the destination
constexpr uint32_t kDmaChctrlClearEnd = 0x00000020u;    ///< CLREND
constexpr uint32_t kDmaChctrlClearTc = 0x00000040u;     ///< CLRTC
constexpr uint32_t kDmaChctrlSwReset = 0x00000008u;     ///< SWRST
constexpr uint32_t kDmaChctrlSetEnable = 0x00000001u;   ///< SETEN
constexpr uint32_t kDmaChctrlClearEnable = 0x00000002u; ///< CLREN

/// D0FBCFG's 32-byte continuous access mode, USB_DFACC_32 in the vendor's bit definitions.
constexpr uint16_t kFifoAccess32Byte = 0x2000u;

/// 32-byte blocks either side, destination fixed on the FIFO's block registers, source incrementing through the
/// packet. kMaxFramesPerPacket keeps every packet a whole number of blocks, so there is never a remainder.
///
/// The FIFO port's own access mode must agree with this or the two disagree about what a request is worth; that
/// is kFifoAccess32Byte, written to D0FBCFG in configureAudioDmaFifo.
constexpr uint32_t kAudioDmaConfig = kDmaChcfgAmBusCycle | kDmaChcfgLevel | kDmaChcfgHighEnable | kDmaChcfgSelCh0
                                     | kDmaChcfgDest256 | kDmaChcfgDestFixed | kDmaChcfgSrc256 | kDmaChcfgReqDest;

/// Points the D0FIFO port at the audio pipe and leaves it there for the life of the stream.
///
/// This is the half of the change that matters beyond CPU cost: D0FIFO is a second, independent port, so the
/// audio path stops sharing the CPU FIFO port with MIDI. The borrow-and-restore around every write, the interrupt
/// masking that protected it, and the part-frame packet that rotates the host's channel mapping when two writers
/// collide all cease to be possible rather than being guarded against.
void configureAudioDmaFifo() {
	usb_regadr_t reg = usb_hstd_get_usb_ip_adr(USB_CFG_USE_USBIP);
	const uint16_t want = (uint16_t)(USB_CFG_PAUDIO_ISO_IN | USB_MBW_32);
	uint16_t seen;
	do {
		reg->D0FIFOSEL = want;
		seen = reg->D0FIFOSEL;
	} while ((seen & USB_CURPIPE) != USB_CFG_PAUDIO_ISO_IN);
	// Separately and after the pipe select has taken, matching the vendor driver: the request line must not be
	// armed while the port still points somewhere else.
	// The port's access mode is set while the request line is still down and before any transfer is armed.
	reg->D0FBCFG = kFifoAccess32Byte;
	reg->D0FIFOSEL = (uint16_t)(want | USB_DREQE);
	audioDmaConfigured = true;
}

/// Commits the packet the controller has just finished moving.
///
/// A transmit DMA leaves the buffer plane loaded but unsent - the hardware cannot know whether a short packet is
/// finished or merely paused - so software sets the buffer-valid flag to release it. Two register writes, which is
/// the whole CPU cost of a packet now.
void retireAudioDma() {
	if (!audioDmaBusy) {
		return;
	}
	DMACn(kAudioDmaChannel).CHCTRL_n = kDmaChctrlClearEnd | kDmaChctrlClearTc;
	usb_regadr_t reg = usb_hstd_get_usb_ip_adr(USB_CFG_USE_USBIP);
	reg->D0FIFOCTR |= USB_BVAL;
	if constexpr (kDiagnostics) {
		// Read after the commit, so the span covers everything that stands between the plane being free and the
		// packet being released - the transfer and the interrupt that retires it.
		const uint32_t span = Debug::readCycleCounter() - statDmaArmCycles;
		statDmaSpanSum += span;
		if (span > statDmaSpanMax) {
			statDmaSpanMax = span;
		}
		const uint32_t frames =
		    (uint32_t)((uint16_t)(reg->FRMNUM & kFrameNumberMask) - statDmaArmFrame) & kFrameNumberMask;
		statDmaFrameSpan[frames > 2u ? 2u : frames]++;
	}
	if (audioDmaStandby) {
		audioDmaStandby = false;
	}
	else {
		queueRead++;
	}
	bump(statTimerWrote);
	bump(statDmaCompleted);
	audioDmaBusy = false;
}

extern "C" void audioDmaTransferComplete(uint32_t /*intSense*/) {
	const uint32_t start = costStart();
	retireAudioDma();
	addCost(costDmaComplete, start);
}

/// Parks the D0FIFO port so the driver can write the same pipe through the CPU port.
///
/// One pipe must not be selected on two FIFO ports at once. The timer's writes and the driver's have always
/// shared a pipe; until now they also shared a port, and interrupt masking was enough to keep them apart. They
/// are on different ports now, so the port itself has to be handed over. This runs on the driver's re-arm, which
/// is about once a second, not on the write path.
///
/// Returns false if a transfer would not retire in time, in which case the caller must not write: a bounded spin
/// that gives up is the only safe answer, since this runs with interrupts masked and the completion interrupt
/// that would normally retire the transfer cannot run.
bool parkAudioDmaPort() {
	const uint32_t deadline = Debug::readCycleCounter() + 4000u; ///< 10 us at 400 MHz
	while ((DMACn(kAudioDmaChannel).CHSTAT_n & 0x05u) != 0u) {
		if ((int32_t)(Debug::readCycleCounter() - deadline) > 0) {
			return false;
		}
	}
	// The controller has finished but its interrupt is masked by the caller, so the packet is sitting in the
	// plane uncommitted. Retire it here rather than leaving it for an interrupt that will arrive after the
	// driver has already written the pipe.
	retireAudioDma();
	usb_regadr_t reg = usb_hstd_get_usb_ip_adr(USB_CFG_USE_USBIP);
	reg->D0FIFOSEL = USB_MBW_32;
	audioDmaConfigured = false;
	return true;
}

/// Hands one packet to the DMA controller. Returns without arming if the channel is still working on the last one.
bool startAudioDma(const uint8_t* source, uint16_t bytes) {
	const uint32_t start = costStart();
	// EN or TACT still set means the previous transfer has not retired. Skipped rather than waited on: the next
	// timer fire is 400 us away and spinning here would hold the interrupt open for exactly the reason this
	// change exists.
	if ((DMACn(kAudioDmaChannel).CHSTAT_n & 0x05u) != 0u) {
		bump(statDmaBusySkips);
		return false;
	}
	// The ordinary, cached address of the packet: whoever filled it wrote through the uncached mirror, so what
	// is in memory is already current and the controller reads the same bytes from the normal one.
	DMACn(kAudioDmaChannel).N0SA_n = (uint32_t)source;
	// The block registers rather than the single-word port: a 32-byte transfer lands across D0FIFOB0-B7 in one
	// bus cycle, which is the whole point of the change.
	DMACn(kAudioDmaChannel).N0DA_n = (uint32_t)&(usb_hstd_get_usb_ip_adr(USB_CFG_USE_USBIP)->D0FIFOB0);
	DMACn(kAudioDmaChannel).N0TB_n = bytes;
	DMACn(kAudioDmaChannel).CHCFG_n = kAudioDmaConfig;
	DMACn(kAudioDmaChannel).CHITVL_n = 0u;
	setDMARS(kAudioDmaChannel, kAudioDmaRequest);
	DMACn(kAudioDmaChannel).CHCTRL_n = kDmaChctrlSwReset;
	if constexpr (kDiagnostics) {
		statDmaArmFrame = (uint16_t)(usb_hstd_get_usb_ip_adr(USB_CFG_USE_USBIP)->FRMNUM & kFrameNumberMask);
		statDmaArmCycles = Debug::readCycleCounter();
	}
	audioDmaBusy = true;
	DMACn(kAudioDmaChannel).CHCTRL_n = kDmaChctrlSetEnable;
	bump(statDmaStarted);
	addCost(costDmaStart, start);
	return true;
}

/// Pushes one pre-built packet into the pipe, from the timer interrupt.
///
/// Does no building and touches neither the ring nor the sequence counter: those belong to the task, because a
/// ~990-byte copy out of the ring inside a 1 kHz interrupt is what starved the audio engine on v0.14.0. Since the
/// DMA controller took the copy over, all this does is check whether a buffer plane is free and arm a transfer.
void audioWriteTimerBody() {
	timerClearCompareMatchTGRA(kAudioWriteTimer);
	bump(statTimerFires);

	if (!streamActive || !transferInitialised) {
		return;
	}
	// Counted before the queue check, so the denominator is every fire this bin got rather than every fire that
	// happened to find work. A bin whose fires differ from the others is not comparable and sweep.py says so.
	if (kSweepOffsets && sweepBin < kSweepBins) {
		sweepFires[sweepBin]++;
	}
	const bool queueEmpty = (queueWrite == queueRead);
	if (queueEmpty) {
		bump(statTimerEmpty);
		if (!standbyReady) {
			return;
		}
	}

	// One packet per fire now, not up to two. The catch-up second write existed because a CPU write completes
	// inside the interrupt and a second one could follow it; a DMA transfer is still in flight when this returns,
	// so there is nothing to double up on. It was recovering 6-8 frames a second.
	if (audioDmaBusy) {
		// A transfer the controller has finished but whose interrupt has not run leaves the packet uncommitted
		// and audioDmaBusy set, which would stall the stream permanently on a single missed interrupt. Retiring
		// it here makes the interrupt an optimisation rather than the only path. The two are still told apart:
		// costDmaComplete counts only the interrupt's retires, so dcp near zero against dma completes near a
		// thousand says the interrupt never fires at all.
		if ((DMACn(kAudioDmaChannel).CHSTAT_n & 0x05u) != 0u) {
			bump(statDmaBusySkips);
			return;
		}
		retireAudioDma();
		bump(statDmaLateRetires);
	}

	// The audio pipe now has D0FIFO to itself - MIDI is on the CPU port - so this reads the plane's readiness
	// without steering a shared selector, and neither the borrow-and-restore nor the interrupt masking that
	// protected it is needed any more. That polling was measured at 14.2 us per fire on 2026-08-29, on top of the
	// 60.7 us copy.
	usb_regadr_t reg = usb_hstd_get_usb_ip_adr(USB_CFG_USE_USBIP);
	if ((reg->D0FIFOCTR & USB_FRDY) == 0u) {
		bump(statFrdyNever);
		bump(statTimerShut);
		return;
	}
	bump(statFrdyImmediate);

	if (queueEmpty) {
		// The builder is frozen - a card load, or the audio task starved. Keep the endpoint answering with
		// marked silence rather than letting the wire go quiet, which is the thing the host cannot absorb.
		stampStandbyPacket();
		audioDmaStandby = true;
		if (!startAudioDma(standbyPacket.data, standbyBytes)) {
			audioDmaStandby = false;
			bump(statTimerShut);
			return;
		}
		bump(statStandbySent);
		return;
	}

	const uint32_t slot = queueRead % kQueueSlots;
	if (!startAudioDma(packetQueue[slot].data, packetQueueBytes[slot])) {
		bump(statTimerShut);
		return;
	}
	// queueRead is advanced by the completion handler, not here: the controller is still reading this slot and
	// releasing it now would let the builder overwrite a packet mid-flight.
	if (kSweepOffsets && sweepBin < kSweepBins) {
		sweepWrites[sweepBin]++;
	}
}

/// DIAGNOSTIC. Split from the body above only so that every one of its early returns is timed, rather than the
/// branches that do work being the only ones counted. A fire that finds nothing to do still costs an interrupt
/// entry and exit, and whether that is the expense is the question this build exists to answer.
void audioWriteTimerInterrupt(uint32_t /*intSense*/) {
	const uint32_t start = costStart();
	audioWriteTimerBody();
	addCost(costTimer, start);
}

/// Fills the queue from the task, where the copy is affordable.
void buildQueuedPackets() {
	const uint32_t start = costStart();
	while ((uint32_t)(queueWrite - queueRead) < kQueueSlots) {
		const uint32_t slot = queueWrite % kQueueSlots;
		// Built straight into the uncached view of the slot, so the bytes are in memory the instant they are
		// written and the DMA controller needs no cache maintenance to see them. A whole-cache flush per packet,
		// which is what the vendor driver does, would cost more than the copy this change exists to remove.
		const uint32_t frames = buildNextPacket(uncachedSlot(slot));
		packetQueueBytes[slot] = (uint16_t)(frames * kFrameBytes);
		// The slot's contents must be visible before the index that publishes it.
		__asm volatile("DSB" ::: "memory");
		queueWrite++;
		bump(statQueueBuilt);
	}
	// Counted once per call rather than once per packet built, so the mean below is the cost of a whole refill -
	// which is what the audio task actually waits for.
	addCost(costBuild, start);
}

/// Called from the peripheral interrupt handler's frame branch, on the host's own clock, 1000 times a second.
///
/// Does no work beyond re-arming the timer: two register writes. v0.14.0 put the ~990-byte ring copy and the
/// FIFO write in here and starved the audio engine, clipping channels 1-2 to full scale. The task builds the
/// packets now and the timer only pushes bytes already in memory, so the expensive half of that is gone and the
/// cheap half happens 750 us from here rather than at the boundary, where the plane is reliably shut.
extern "C" void usbAudioStreamStartOfFrame(void) {
	bump(statSofSeen);
	if (!audioTimerRunning) {
		return;
	}
	// Zeroing the count is what locks the phase: the timer now measures from the host's frame boundary rather
	// than from its own last fire, so its crystal sets only the offset's accuracy and never accumulates against
	// the host's rate.
	if (kSweepOffsets) {
		// Advanced here rather than from the timer, so each bin gets exactly kSweepDwellFrames of the host's
		// frames whether or not the writes in it succeeded. Dwelling by successful writes would give the good
		// offsets more frames than the bad ones and bias the very comparison being made.
		if (++sweepFrameCount >= kSweepDwellFrames) {
			sweepFrameCount = 0;
			if (++sweepBin >= kSweepBins) {
				sweepBin = 0;
				sweepPasses++;
			}
		}
		sofToWriteTicks = sweepOffsetTicks(sweepBin);
	}
	*TCNT[kAudioWriteTimer] = 0;
	*TGRA[kAudioWriteTimer] = (uint16_t)sofToWriteTicks;
}

void startAudioWriteTimer() {
	if (audioTimerRunning) {
		return;
	}
	setupTimerWithInterruptHandler(kAudioWriteTimer, 1, audioWriteTimerInterrupt, 5);
	*TGRA[kAudioWriteTimer] = (uint16_t)kTimerTicksPerMs;
	R_INTC_Enable(INTC_ID_TGIA[kAudioWriteTimer]);
	enableTimer(kAudioWriteTimer);
	audioTimerRunning = true;
}

void stopAudioWriteTimer() {
	if (!audioTimerRunning) {
		return;
	}
	disableTimer(kAudioWriteTimer);
	R_INTC_Disable(INTC_ID_TGIA[kAudioWriteTimer]);
	audioTimerRunning = false;
}

} // namespace

namespace deluge::processing::engines {

void USBAudioStream::routine() {
	if constexpr (kDiagnostics) {
		drainSetupTrace();
		reportStats();
		reportStemPeak();
	}
	service();
}

/// Split from routine() so the audio engine can service the transport without also emitting a SysEx report at the
/// engine's own rate. The diagnostics stay on the scheduler's ~600 Hz; this runs at both that and the engine's.
///
/// DIAGNOSTIC. The body is separated from USBAudioStream::service() below only so that its early returns are
/// timed with the rest. This is the path the audio task is billed for, and direness is read straight off that
/// bill, so it is the single most important of the six figures.
static void serviceBody() {
	// Independent of everything below: a host may hold either interface without the other, and a DAW recording
	// stems while sending nothing back is the ordinary case.
	serviceReturn();

	if (!ringCleared) {
		memset(ring, 0, sizeof(ring));
		if constexpr (kAudioStamps) {
			// Channels 3 upwards get a distinct constant, written here and never touched by the render path, so
			// one recording separates three cases that silence cannot: transport dead (everything zero),
			// transport alive but capture broken (these present, mix channels zero), and working (both). The
			// differing value per channel proves the channel mapping at the same time. It permanently occupies
			// those channels, which is why per-track routing needs the instruments off.
			for (uint32_t f = 0; f < kRingFrames; f++) {
				for (uint32_t c = 2; c < kChannels; c++) {
					ring[f * kChannels + c] = (int16_t)((c + 1u) * 2000u);
				}
			}
		}
		ringCleared = true;
	}

	// The host parks the interface at alt 0 when it is not streaming, which is what frees the isochronous bandwidth.
	if (!g_usb_peri_connected || g_usb_pstd_alt_num[kAudioInterfaceNumber] != kStreamingAltSetting) {
		transferInFlight = false;
		frameAccumulator = 0;
		primed = false;
		streamActive = false;
		// The driver reconfigures the pipe on the next SetInterface, so the snapshot is only valid for the stream
		// it was taken from.
		pipeConfigRead = false;
		lastCompletionFrameValid = false;
		readFrame = writeFrame;
		bump(statReadAdvPark);
		queueRead = queueWrite;
		firstPacketSubmitted = false;
		// Stop the channel and drop the request line with it: a stream that ends between arming and completion
		// would otherwise leave a live request against a pipe nothing is servicing.
		if (audioDmaConfigured) {
			DMACn(kAudioDmaChannel).CHCTRL_n = kDmaChctrlClearEnable;
			usb_hstd_get_usb_ip_adr(USB_CFG_USE_USBIP)->D0FIFOSEL = USB_MBW_32;
			audioDmaConfigured = false;
		}
		audioDmaBusy = false;
		stopAudioWriteTimer();
		if (sofEnabled) {
			usb_hstd_get_usb_ip_adr(USB_CFG_USE_USBIP)->INTENB0 &= (uint16_t)~kIntEnbSofe;
			sofEnabled = false;
		}
		return;
	}
	streamActive = true;
	bump(statAlt1);

	// Unconditionally, before any branch. This used to live inside the in-flight branch alone, which gave the
	// task a state it could never leave: once a transfer ended without a new one starting, every pass took the
	// other branch, the queue was never refilled, and the timer starved on it forever. Measured 2026-08-26
	// evening as inf0 / qb0 / tw0 with the ring lead climbing past 1.2 million frames, and again 2026-08-27 with
	// 0.45 s of unique audio delivered in 30 s. Keeping packets ready is the task's job in every state, not only
	// in the one where the driver happens to be busy.
	buildQueuedPackets();

	if (transferInFlight) {
		bump(statInFlight);
		if constexpr (kDiagnostics) {
			// Sampled here rather than at the top of the routine: a transfer is outstanding, which is the only
			// condition under which "is a plane writable" is the question being asked.
			const uint16_t ctr = readPipeCtr();
			statReadyState[(((ctr & kPipeCtrBsts) != 0u) ? 2u : 0u) | (((ctr & kPipeCtrInBufM) != 0u) ? 1u : 0u)]++;
			// Data pending and a plane writable: the second plane can be loaded now, and this is the only moment
			// it can be. Anything else is either nothing to double up on or a port that would refuse the write.
			if ((ctr & (kPipeCtrBsts | kPipeCtrInBufM)) != (kPipeCtrBsts | kPipeCtrInBufM)) {
				statExtraNoWindow++;
			}
		}
		endAbandonedTransfer();
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

	// Re-armed through the driver every time nothing is in flight, which is what v0.15.0 does and what the
	// timer route stopped doing. The rule it replaces - one submit per stream, everything after it the timer's -
	// rested on the pipe staying armed once the driver's transfer had gone. Measured 2026-08-27: it does not.
	// The timer found the plane shut on 1446 fires out of 1446, with zero submits and zero completions after the
	// first, because with no transfer in flight no plane ever opens. The window it was written against was only
	// ever observed while a transfer was outstanding.
	//
	// The timer keeps its job: filling the second plane when a window does appear. It is now an addition to a
	// working transport rather than a replacement for one.
	//
	// Interrupts off across the driver's own FIFO write, which is also what keeps the timer out of it: the two
	// share a FIFO port, and two writers there truncate a packet into a part-frame that rotates the host's
	// channel mapping permanently.
	const uint32_t submitStart = costStart();
	DISABLE_ALL_INTERRUPTS();
	// The port is handed over rather than shared, and the write is abandoned if a transfer will not retire.
	// Skipping one re-arm costs a packet; writing a pipe selected on two ports is what rotates the host's
	// channel mapping permanently.
	if (audioDmaConfigured && !parkAudioDmaPort()) {
		ENABLE_ALL_INTERRUPTS();
		bump(statSubmitSkipped);
		addCost(costSubmit, submitStart);
		return;
	}
	sendNextPacket();
	ENABLE_ALL_INTERRUPTS();
	addCost(costSubmit, submitStart);
	firstPacketSubmitted = transferInFlight;
	// Started only once a host is actually streaming and the driver's own first transfer is set up, so an idle
	// Deluge pays nothing and the timer never fires against an unconfigured pipe.
	if (!audioDmaConfigured) {
		if (!standbyReady) {
			prepareStandbyPacket();
		}
		configureAudioDmaFifo();
		if (!audioDmaInterruptArmed) {
			// Priority 5, matching the write timer that arms it: this hands one packet over and returns, and
			// there is no reason for it to outrank the CV SPI transfers already at that level.
			setupAndEnableInterrupt(audioDmaTransferComplete, INTC_ID_DMAINT0 + kAudioDmaChannel, 5);
			audioDmaInterruptArmed = true;
		}
	}
	startAudioWriteTimer();
	if (!sofEnabled && transferInitialised) {
		usb_hstd_get_usb_ip_adr(USB_CFG_USE_USBIP)->INTENB0 |= kIntEnbSofe;
		sofEnabled = true;
	}
}

void USBAudioStream::service() {
	// The cycle counter is off in a release build - Debug::init() is only reached under a trace flag - so an
	// uninitialised counter reads a constant. Enabled once, here, and unconditionally: parkAudioDmaPort spins on
	// a real 10 us deadline read from this counter with interrupts masked, so a stopped counter would hang the
	// machine there rather than merely zeroing a figure. It costs one register write per boot.
	if (!costCounterEnabled) {
		Debug::init();
		costCounterEnabled = true;
		costIntervalStart = Debug::readCycleCounter();
	}
	const uint32_t start = costStart();
	serviceBody();
	addCost(costService, start);
}

bool USBAudioStream::stemsWanted() {
	// The host holding the interface is the whole test. A build nobody is streaming from does not capture, which is
	// what keeps an unplugged Deluge at stock cost.
	return streamActive;
}

void USBAudioStream::beginRender(uint32_t numSamples) {
	// Set whatever the stream is doing. A clip that has left the main mix has left it whether or not a computer
	// is listening - gating this on the stream made the Deluge's own outputs change when a cable was plugged in,
	// which is not a routing decision the user made.
	stemWindowSamples = (numSamples > kStemWindowSamples) ? kStemWindowSamples : numSamples;
	channelsWritten = 0;
	if (!streamActive) {
		return;
	}
	const uint32_t start = costStart();
	for (uint32_t c = 0; c < kStemChannels; c++) {
		memset(stemAccumulator[c], 0, stemWindowSamples * sizeof(int32_t));
	}
	addCost(costStemClear, start);
}

void USBAudioStream::snapshotBeforeTrack(const int32_t* mixNow, uint32_t numSamples) {
	if (numSamples > stemWindowSamples) {
		return;
	}
	const uint32_t start = costStart();
	memcpy(stemSnapshot, mixNow, numSamples * 2 * sizeof(int32_t));
	addCost(costStemSnapshot, start);
}

/// Built at -O3 for the vector unit alone. At the project's -O2 the compiler half-vectorises these and then hands
/// each result back to the main processor one sample at a time to store it, which stalls this core's pipeline; at
/// -O3 it deinterleaves and stores in vector registers throughout. Confirmed in the generated code, not assumed.
///
/// Four walks rather than one branchy one, for the same reason: a test inside the loop puts the whole thing back
/// on the main processor. Mono sums the track's two sides; stereo takes one side. Assign is the first writer to a
/// channel, add is every writer after it.
namespace {

[[gnu::optimize("O3")]] void walkMonoAssign(int32_t* __restrict out, const int32_t* __restrict mixNow,
                                            const int32_t* __restrict snapshot, uint32_t numSamples) {
	for (uint32_t i = 0; i < numSamples; i++) {
		// The track's own contribution, taken as what it added to the mix rather than by rendering it a second
		// time. Halved on the way to mono so a centred track keeps its level rather than doubling it.
		const int32_t l = mixNow[i * 2] - snapshot[i * 2];
		const int32_t r = mixNow[i * 2 + 1] - snapshot[i * 2 + 1];
		out[i] = (l >> 1) + (r >> 1);
	}
}

[[gnu::optimize("O3")]] void walkMonoAdd(int32_t* __restrict out, const int32_t* __restrict mixNow,
                                         const int32_t* __restrict snapshot, uint32_t numSamples) {
	for (uint32_t i = 0; i < numSamples; i++) {
		const int32_t l = mixNow[i * 2] - snapshot[i * 2];
		const int32_t r = mixNow[i * 2 + 1] - snapshot[i * 2 + 1];
		out[i] += (l >> 1) + (r >> 1);
	}
}

/// side is 0 for left, 1 for right. Not halved: a stereo pair keeps each side whole, because nothing is being
/// summed into it.
[[gnu::optimize("O3")]] void walkSideAssign(int32_t* __restrict out, const int32_t* __restrict mixNow,
                                            const int32_t* __restrict snapshot, uint32_t numSamples, uint32_t side) {
	for (uint32_t i = 0; i < numSamples; i++) {
		out[i] = mixNow[i * 2 + side] - snapshot[i * 2 + side];
	}
}

[[gnu::optimize("O3")]] void walkSideAdd(int32_t* __restrict out, const int32_t* __restrict mixNow,
                                         const int32_t* __restrict snapshot, uint32_t numSamples, uint32_t side) {
	for (uint32_t i = 0; i < numSamples; i++) {
		out[i] += mixNow[i * 2 + side] - snapshot[i * 2 + side];
	}
}

/// Its own walk, and only while the peak instrument is compiled in. Folded into the walks above it reintroduces
/// exactly the register transfer they were rewritten to avoid - measured in the generated code, not assumed.
void trackPeak(const int32_t* out, uint32_t numSamples) {
	int32_t peak = stemPeakAll;
	for (uint32_t i = 0; i < numSamples; i++) {
		const int32_t m = out[i];
		const int32_t magnitude = (m < 0) ? -m : m;
		if (magnitude > peak) {
			peak = magnitude;
		}
	}
	stemPeakAll = peak;
}

} // namespace

void USBAudioStream::captureStem(uint16_t route, const int32_t* mixNow, uint32_t numSamples) {
	if (numSamples > stemWindowSamples || (route & UsbRoute::ANY_USB) == 0) {
		return;
	}
	const uint32_t costCaptureStart = costStart();
	const int32_t* const snapshot = stemSnapshot;

	for (uint32_t c = 0; c < kStemChannels; c++) {
		if ((route & (UsbRoute::FIRST_MONO << c)) == 0) {
			continue;
		}
		int32_t* const out = stemAccumulator[c];
		const uint16_t bit = (uint16_t)(1u << c);
		if ((channelsWritten & bit) == 0) {
			walkMonoAssign(out, mixNow, snapshot, numSamples);
			channelsWritten |= bit;
		}
		else {
			walkMonoAdd(out, mixNow, snapshot, numSamples);
		}
		if constexpr (kDiagnostics) {
			trackPeak(out, numSamples);
		}
	}

	for (uint32_t pair = 0; pair < kStemChannels / 2; pair++) {
		if ((route & (UsbRoute::PAIR12 << pair)) == 0) {
			continue;
		}
		for (uint32_t side = 0; side < 2; side++) {
			const uint32_t c = pair * 2 + side;
			int32_t* const out = stemAccumulator[c];
			const uint16_t bit = (uint16_t)(1u << c);
			if ((channelsWritten & bit) == 0) {
				walkSideAssign(out, mixNow, snapshot, numSamples, side);
				channelsWritten |= bit;
			}
			else {
				walkSideAdd(out, mixNow, snapshot, numSamples, side);
			}
			if constexpr (kDiagnostics) {
				trackPeak(out, numSamples);
			}
		}
	}

	addCost(costStemCapture, costCaptureStart);
}

void USBAudioStream::removeTrackFromMix(int32_t* mixNow, uint32_t numSamples) {
	if (numSamples > stemWindowSamples) {
		return;
	}
	// x - (x - y) = y in two's complement, including on overflow, so putting the snapshot back is exactly
	// subtracting the track that rendered since it was taken - and it is a copy rather than a walk.
	const uint32_t start = costStart();
	memcpy(mixNow, stemSnapshot, numSamples * 2 * sizeof(int32_t));
	addCost(costStemSnapshot, start);
}

void USBAudioStream::mixReturn(StereoSample* buffer, uint32_t numSamples) {
	if (buffer == nullptr || numSamples == 0) {
		return;
	}
	mixReturnInto(buffer, numSamples);
}

void USBAudioStream::setReturnLevel(uint32_t level) {
	if (level > kReturnLevelMax) {
		level = kReturnLevelMax;
	}
	returnLevelSetting = level;
	recomputeReturnMultiplier();
}

uint32_t USBAudioStream::getReturnLevel() {
	return returnLevelSetting;
}

void USBAudioStream::setReturnEnabled(bool enabled) {
	returnEnabled = enabled;
	recomputeReturnMultiplier();
}

bool USBAudioStream::getReturnEnabled() {
	return returnEnabled;
}

void USBAudioStream::setTrim(uint32_t trim) {
	if (trim > kTrimMax) {
		trim = kTrimMax;
	}
	stemTrimSetting = trim;
	// 1.2 dB a step, the same ladder the AUX send levels use, so the two features feel like one control when they
	// merge. Full scale at the top, silence at the bottom.
	if (trim == 0) {
		stemTrimMultiplier = 0;
		return;
	}
	int32_t multiplier = 1 << 24;
	for (uint32_t step = trim; step < kTrimMax; step++) {
		// 1.2 dB is a factor of 0.871. 57139/65536 to stay in integers.
		multiplier = (int32_t)(((int64_t)multiplier * 57139) >> 16);
	}
	stemTrimMultiplier = multiplier;
	// The return's conversion is this one's inverse, so the two cannot be set independently.
	recomputeReturnMultiplier();
}

uint32_t USBAudioStream::getTrim() {
	return stemTrimSetting;
}

int32_t USBAudioStream::readAndClearStemPeak() {
	const int32_t peak = stemPeakAll;
	stemPeakAll = 0;
	return peak;
}

uint32_t USBAudioStream::costMark() {
	return costStart();
}

void USBAudioStream::costEngineRoutine(uint32_t start) {
	addCost(costEngine, start);
}

void USBAudioStream::costOutputLoop(uint32_t start) {
	addCost(costOutputs, start);
}

void USBAudioStream::feedMix(const StereoSample* mix, uint32_t numSamples, uint32_t renderOffset) {
	// Taken before the early return, not after: what an installed-but-idle build costs is one of the things being
	// separated, and a timer that starts after the guard can never see it.
	const uint32_t feedStart = costStart();
	// Nothing is listening, so do not pay for the conversion. While stopped the consumer parks the read pointer
	// on the write pointer every pass, so the ring cannot look full to the first host that arrives.
	if (!streamActive || numSamples == 0 || mix == nullptr) {
		addCost(costFeedMix, feedStart);
		return;
	}

	// The consumer runs in the USB completion interrupt, so it can interleave with this at any instruction. What
	// makes that safe is the pointers rather than the scheduler: one writer each, 32-bit aligned, and each is
	// advanced only after its own data is in place, which is the ordinary single-producer/single-consumer ring.
	// Both sides run on the same core, so there is no cache to reconcile - only the compiler, which volatile holds.

	bump(statFramesIn, numSamples);

	uint32_t w = writeFrame;
	for (uint32_t i = 0; i < numSamples; i++) {
		int16_t* const frame = &ring[(w & kRingMask) * kChannels];
		// The samples handed here are already at output scale, so this is only a width reduction.
		frame[0] = (int16_t)(mix[i].l >> 16);
		frame[1] = (int16_t)(mix[i].r >> 16);
		if constexpr (!kAudioStamps) {
			// One mono track a channel, across the whole frame, at the scale the mix would have reached the host
			// on. The mix written just above is overwritten by channels 1 and 2 of this - it stays in the code
			// because the instruments below need it and because a mix source is what an assignment interface will
			// route here.
			//
			// The song's own volume, its master filters and its compressor are all applied after the point these
			// were captured, so a stem is the track as the song mixes it and not as the master fader leaves it -
			// which is what an individual track output is for. Turning the master down does not turn these down.
			const uint32_t stemIndex = renderOffset + i;
			if (stemIndex < stemWindowSamples) {
				for (uint32_t c = 0; c < kStemChannels; c++) {
					// Trimmed before the saturation, not after: a stem is taken before the master compressor and
					// runs about three times hotter than the mix, so at unity it flat-tops where the mix does not
					// and the true peak becomes unreadable. Scaling first is what makes the saturation a guard
					// rather than the normal case.
					const int32_t trimmed =
					    (int32_t)(((int64_t)stemAccumulator[c][stemIndex] * stemTrimMultiplier) >> 24);
					frame[c] = (int16_t)(lshiftAndSaturate<AUDIO_OUTPUT_GAIN_DOUBLINGS>(trimmed >> 1) >> 16);
				}
			}
			else {
				for (uint32_t c = 0; c < kStemChannels; c++) {
					frame[c] = 0;
				}
			}
		}
		if constexpr (kMixOnChannels78) {
			// After the stem walk above, so these two win: the channels carry the mix rather than a stem. The
			// same width reduction the mix pair uses, so a level read here is directly comparable with one read
			// off a stem - which is the whole point of measuring a round trip this way.
			frame[kChannels - 2] = (int16_t)(mix[i].l >> 16);
			frame[kChannels - 1] = (int16_t)(mix[i].r >> 16);
		}
		if constexpr (kAudioStamps) {
			// The producer's own frame count, travelling with the audio it belongs to. Whatever reaches the host
			// carries the number of the frame the engine rendered, so the recording alone says which frames were
			// delivered and which were dropped on the way. It is what delivery.py and seams.py decode.
			const uint32_t block = w >> kStampBlockShift;
			if (block != stampBlock) {
				stampBlock = block;
				stampLow = encodeStamp(block);
				stampHigh = encodeStamp(block >> kStampBits);
			}
			frame[2] = stampLow;
			frame[3] = stampHigh;
			frame[4] = kMarkRendered;
		}
		w++;
	}
	writeFrame = w;
	addCost(costFeedMix, feedStart);
}

} // namespace deluge::processing::engines

extern "C" void usbAudioStreamRoutine() {
	deluge::processing::engines::USBAudioStream::routine();
}
