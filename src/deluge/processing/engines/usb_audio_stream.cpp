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
#include "RZA1/system/iodefine.h"
#include "RZA1/system/iodefines/usb20_iodefine.h"
#include "RZA1/usb/r_usb_basic/r_usb_basic_if.h"
#include "deluge/drivers/usb/usb_setup_trace.h"
#include "deluge/drivers/usb/userdef/r_usb_paudio_config.h"

// Declared here rather than by including r_usb_extern.h, which carries an inline function that only compiles as C.
// midi_engine.cpp reaches into the same driver the same way.
extern uint16_t g_usb_peri_connected;
extern uint16_t g_usb_pstd_alt_num[];
usb_er_t usb_pstd_transfer_start(usb_utr_t* ptr);
void usb_pstd_forced_termination(uint16_t pipe, uint16_t status);
usb_regadr_t usb_hstd_get_usb_ip_adr(uint16_t ipno);

// What the driver recorded writing to each pipe's configuration registers. Those registers are only reachable
// through a shared selector, so these are printed alongside a single hardware read: equal says nothing, unequal
// would say the configuration never landed.
extern uint16_t pipeCfgs[];
extern uint16_t pipeBufs[];
extern uint16_t pipeMaxPs[];
void __disable_irq(void);
void __enable_irq(void);
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

/// writeFrame is advanced by the audio routine, readFrame by whoever builds the next packet - the task for the
/// first of a stream, the completion interrupt for the rest. Each is single-writer and 32-bit aligned, which is
/// atomic on this core, so no lock is needed between the two.
volatile uint32_t writeFrame = 0;
volatile uint32_t readFrame = 0;
bool ringCleared = false;
bool primed = false;

/// Whether the pipe's second buffer plane has been loaded for this stream. One shot: each completion refills one
/// plane, so loading the second once is what takes the depth from one to two and keeps it there.
bool secondPlaneLoaded = false;

/// Whether a host currently has the streaming interface selected. Read by the producer so that a machine with
/// nothing plugged in does not pay for the conversion, and set in one place so the two sides cannot disagree.
bool streamActive = false;

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
uint32_t statSecondPlaneLoads = 0; ///< Should read 1 per stream. More than that means the depth is growing.

/// Stamped into channel 3 of every audio frame of every packet, so the host's own recording says which packets
/// actually arrived rather than the driver saying which it thinks it sent.
///
/// Delivery runs at exactly half the host's frame rate and cpl, sub and pkt all agree with each other - but those
/// are all our side of the wire. A host substitutes silence for a packet that never lands, so a contiguous
/// sequence in the recording and a sequence missing every other value look identical in every counter we have.
/// This is the first instrument here that reads what the host received rather than what we submitted.
uint16_t packetSequence = 0;

/// Set for the one packet that loads the pipe's second buffer plane, which stamps channel 4 as well.
///
/// The plane-loading build was inert and its counter only proved the code path ran, not that the write reached a
/// second plane - the distinction the whole question turns on. If a packet tagged this way never appears in the
/// recording, the second write was clobbered or discarded; if it appears, the write reached a plane that the
/// hardware went on to transmit.
bool taggingSecondPlane = false;

/// The capture path dithers every sample by +/-1, so all three of these have to survive that. The scale puts a
/// step of 8 between consecutive sequence numbers, and the two marks sit far from each other and far from the
/// zero the host writes when it substitutes silence for a packet that never arrived.
constexpr int16_t kSequenceScale = 8;
constexpr int16_t kSecondPlaneMark = 0x7FF0;
constexpr int16_t kNormalMark = 8000;

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

/// The readings either side of the one packet that loads the second plane, latched rather than counted.
///
/// It happens once per stream, and the marker built for the same question last session fired outside the
/// recording window - where zero of them is indistinguishable from it never having happened. These are held from
/// the moment they are taken and printed on every report after it, so no window can miss them.
FifoProbe planeProbeFirst = {};
FifoProbe planeProbeSecond = {};
bool planeProbeTaken = false;

/// The gaps of the three completions that follow the one-shot load, in order, latched for the same reason.
///
/// Two resident packets go out on consecutive frames and only the second empties the pipe, so the completion
/// covering them arrives later than the steady two frames. Delivery is a metronome at two, so a three here is a
/// deviation the stream does not otherwise produce. Three of them rather than one: the depth returns to one
/// straight after, and the gaps either side are what say the deviation belongs to the load.
uint32_t planeGapAfter[3] = {};
uint32_t planeGapAfterCount = 0;
bool planeGapArmed = false;

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

constexpr uint16_t kFrameNumberMask = 0x07FFu; ///< FRMNUM b10-0; b15 is OVRN and b14 CRCE.

/// FRMNUM alone rather than the whole register set, because this runs twice per completion inside the interrupt.
uint16_t readFrameNumber() {
	usb_regadr_t reg = usb_hstd_get_usb_ip_adr(USB_CFG_USE_USBIP);
	return (uint16_t)(reg->FRMNUM & kFrameNumberMask);
}

void sendNextPacket();

void transferComplete(usb_utr_t* /*ptr*/, uint16_t /*data1*/, uint16_t /*data2*/) {
	statCompletes++;
	transferInFlight = false;
	regsAtLastCompletion = readPipeRegisters();
	regsSnapshotTaken = true;

	const uint16_t frameAtCompletion = readFrameNumber();
	if (lastCompletionFrameValid) {
		const uint32_t gap = (uint32_t)((frameAtCompletion - lastCompletionFrame) & kFrameNumberMask);
		// Bucketed rather than averaged: a steady two and an alternating one-and-three give the same mean and
		// mean opposite things about where the packet is being lost.
		statCompletionGap[gap > 4u ? 4u : gap]++;
		if (planeGapArmed && planeGapAfterCount < 3u) {
			planeGapAfter[planeGapAfterCount++] = gap;
		}
	}
	lastCompletionFrame = frameAtCompletion;
	lastCompletionFrameValid = true;

	// Hand the next packet over here rather than waiting for the task to come round again, so a transfer is
	// outstanding at every instant. This was written to lift delivery above the ~470 packets/s measured against
	// the host's 1000/s and did not move it at all - the shortfall is not the task's cadence. It earns its place
	// for what it did do: the abandoned-transfer recovery below went from firing several times a second to never,
	// on an idle machine.
	//
	// The copy this costs the interrupt is ~990 bytes at 1 kHz. Measured effect on the audio engine's own 2.9 ms
	// deadline: see the underrun and resync counters, which is what the numbers below are for.
	if (streamActive && transferInitialised) {
		sendNextPacket();
		// Measured across the write rather than timed, because what the chip cares about is which frame the write
		// finished in, not how long it took - a fast write that lands after the SOF is as late as a slow one.
		const uint32_t delta = (uint32_t)((readFrameNumber() - frameAtCompletion) & kFrameNumberMask);
		statWriteFrameDelta[delta > 2u ? 2u : delta]++;

		// Load the pipe's second buffer plane too, once per stream, which takes the depth from one packet to two.
		//
		// Measured 2026-08-26: the write always finishes inside the frame it started (wd1 zero across ~1100
		// packets), and a packet still reaches the host exactly two frames after the previous one (cg2 total, cg1
		// and cg3 zero). The turnaround is fixed, so refilling faster cannot touch it - the frame in between has
		// nothing loaded to send, and FRMNUM's OVRN bit reads set on every sample, which the hardware manual
		// defines as an IN token answered with a zero-length packet. Manual 28.4.9(4) and figure 28.11: data ready
		// before the token gives a packet every frame, data prepared after it gives exactly the interleaved
		// zero-length packets measured here.
		//
		// Here rather than in the task because submit() is the last statement of sendNextPacket() and a completion
		// can arrive before it returns - two calls from the task can be split by this interrupt, two calls from
		// inside it cannot, since a second completion on the same pipe cannot preempt this handler.
		//
		// Once per stream: from then on each completion refills exactly one plane and the depth holds at two.
		if (!secondPlaneLoaded) {
			secondPlaneLoaded = true;
			statSecondPlaneLoads++;
			// Either side of the second write, so the pair says what that write changed rather than what the state
			// happened to be. The first is taken after this completion's own refill has already gone in.
			const FifoProbe before = readFifoProbe();
			taggingSecondPlane = true;
			sendNextPacket();
			const FifoProbe after = readFifoProbe();
			planeProbeFirst = before;
			planeProbeSecond = after;
			planeProbeTaken = true;
			// Armed at the end of this completion, so the next one's gap is the first one latched.
			planeGapAfterCount = 0;
			planeGapArmed = true;
		}
	}
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
	// submit() returns with the packet already in a plane and BVAL set, so this reads whether a plane is left
	// free rather than whether the write happened at all.
	const FifoProbe probe = readFifoProbe();
	if (fifoProbeIsOurs(probe)) {
		statFrdyAfterSubmit[(probe.fifoctr & kFifoCtrFrdy) != 0u ? 1u : 0u]++;
	}
	else {
		statProbeNotOurs++;
	}
	// Always USB_OK in practice - every guard in that function is compiled out - so this records the
	// code rather than relying on it. Kept as one call site so the flag cannot be set in two places.
	transferInFlight = (err == USB_OK);
}

/// Writes this packet's sequence number into channel 3 of every audio frame, and marks channel 4 when this is the
/// packet that loads the second buffer plane.
///
/// Channels 3 upwards already carry the diagnostic constants and nothing real, so this costs no audio. Channels 1
/// and 2 are untouched - the mix has to stay listenable for the recording to be worth anything else.
void stampSequence(uint8_t* buffer, uint32_t frames) {
	int16_t* samples = (int16_t*)buffer;
	// Scaled, because the capture path is not bit-exact: every constant channel comes back dithered by +/-1 LSB,
	// and an unscaled counter cannot tell that wobble from a real step to the next packet. A step of 8 survives it.
	// 12 bits of sequence at 8x fills the 15-bit range, which wraps every ~8 s at 500 packets/s - the decoder
	// takes the step modulo that.
	const int16_t sequence = (int16_t)((packetSequence & 0x0FFFu) * kSequenceScale);
	const int16_t mark = taggingSecondPlane ? kSecondPlaneMark : kNormalMark;
	for (uint32_t f = 0; f < frames; f++) {
		samples[f * kChannels + 2] = sequence;
		samples[f * kChannels + 3] = mark;
	}
	packetSequence++;
	taggingSecondPlane = false;
}

/// Builds one packet from the ring and hands it to the endpoint.
///
/// Called from the task for the first packet of a stream and after a recovery, and from the completion for every
/// packet after that. Both are single-threaded against each other: the task only builds when no transfer is
/// outstanding, and a completion can only arrive when one is.
///
/// submit() is deliberately the last statement, because the completion for it can arrive before this returns.
void sendNextPacket() {
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
			uint8_t* buffer = nextPacketBuffer();
			memset(buffer, 0, frames * kFrameBytes);
			stampSequence(buffer, frames);
			submit(buffer, frames * kFrameBytes);
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
		uint8_t* buffer = nextPacketBuffer();
		memset(buffer, 0, frames * kFrameBytes);
		stampSequence(buffer, frames);
		submit(buffer, frames * kFrameBytes);
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
	uint8_t* buffer = nextPacketBuffer();
	memcpy(buffer, &ring[start * kChannels], firstRun * kFrameBytes);
	if (firstRun < send) {
		memcpy(&buffer[firstRun * kFrameBytes], &ring[0], (send - firstRun) * kFrameBytes);
	}
	readFrame = (readFrame + send) & kRingMask;

	statPacketsSent++;
	statFramesSent += send;

	stampSequence(buffer, send);
	submit(buffer, send * kFrameBytes);
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
	statAbandonedSeen++;

	if (++stuckPasses < 2) {
		return;
	}
	stuckPasses = 0;

	statAbandonedEnded++;
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
	char line[256];
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
	emit(" nrs");
	emitDec(statAbandonedSeen);
	emit(" nre");
	emitDec(statAbandonedEnded);
	emit(" rs");
	emitDec(statResyncs);
	emit(" lead");
	emitDec((writeFrame - readFrame) & kRingMask);
	*p = '\0';
	Debug::sysexDebugPrint(*Debug::midiDebugCable, line, true);

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
	emit(" p2:");
	emitDec(statSecondPlaneLoads);
	*p = '\0';
	Debug::sysexDebugPrint(*Debug::midiDebugCable, timingLine, true);

	// Fifth line, own buffer for the same reason as the third and fourth. fr1/fr0 are every submit's outcome; the
	// latched f/pc pair is the second-plane answer, and FRDY (0x2000) still set in f2 says a plane was free after
	// the second write, which means it did not reach one.
	char planeLine[160];
	p = planeLine;
	// dfr is cumulative and never cleared: whether a second plane has *ever* held data is the question, not how
	// often, and a per-report count of zero would look the same as never having been armed.
	emit("AUP dfr:");
	emitDec(usbBempAudioDeferred);
	emit(" fr1:");
	emitDec(statFrdyAfterSubmit[1]);
	emit(" fr0:");
	emitDec(statFrdyAfterSubmit[0]);
	emit(" bad:");
	emitDec(statProbeNotOurs);
	if (planeProbeTaken) {
		emit(" f1:");
		emitHex(planeProbeFirst.fifoctr);
		emit(" pc1:");
		emitHex(planeProbeFirst.pipectr);
		emit(" f2:");
		emitHex(planeProbeSecond.fifoctr);
		emit(" pc2:");
		emitHex(planeProbeSecond.pipectr);
		emit(" s:");
		emitHex(planeProbeSecond.fifosel);
		emit(" g:");
		for (uint32_t i = 0; i < 3u; i++) {
			if (i < planeGapAfterCount) {
				emitDec(planeGapAfter[i]);
			}
			else {
				emit("-");
			}
			if (i < 2u) {
				emit(",");
			}
		}
	}
	else {
		// Dashes rather than zeros: the pair not having been taken and a pair of zero readings are different
		// findings, which is the confusion this whole build exists to end.
		emit(" f1---- pc1---- f2---- pc2---- s---- g:-,-,-");
	}
	*p = '\0';
	Debug::sysexDebugPrint(*Debug::midiDebugCable, planeLine, true);

	statPacketsSent = 0;
	statFramesSent = 0;
	statAlt1 = 0;
	statInFlight = 0;
	statPrimeSilence = 0;
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
	statFrdyAfterSubmit[0] = 0;
	statFrdyAfterSubmit[1] = 0;
	statProbeNotOurs = 0;
	// usbBempBitsSeen is deliberately not cleared: which pipes are live at all is the question, not how often.
}

} // namespace

namespace deluge::processing::engines {

void USBAudioStream::routine() {
	drainSetupTrace();
	reportStats();

	if (!ringCleared) {
		memset(ring, 0, sizeof(ring));
		// DIAGNOSTIC. Channels 3 upwards get a distinct constant, written here and never touched by the
		// render path, so one recording separates three cases that silence cannot: transport dead
		// (everything zero), transport alive but capture broken (these present, mix channels zero), and
		// working (both). The differing value per channel proves the channel mapping at the same time.
		// Remove once real audio is confirmed - it permanently occupies those channels.
		for (uint32_t f = 0; f < kRingFrames; f++) {
			for (uint32_t c = 2; c < kChannels; c++) {
				ring[f * kChannels + c] = (int16_t)((c + 1u) * 2000u);
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
		secondPlaneLoaded = false;
		planeProbeTaken = false;
		planeGapArmed = false;
		planeGapAfterCount = 0;
		readFrame = writeFrame;
		return;
	}
	streamActive = true;
	statAlt1++;

	if (transferInFlight) {
		statInFlight++;
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

	sendNextPacket();
}

void USBAudioStream::feedMix(const StereoSample* mix, uint32_t numSamples) {
	// Nothing is listening, so do not pay for the conversion. While stopped the consumer parks the read pointer
	// on the write pointer every pass, so the ring cannot look full to the first host that arrives.
	if (!streamActive || numSamples == 0 || mix == nullptr) {
		return;
	}

	// The consumer runs in the USB completion interrupt, so it can interleave with this at any instruction. What
	// makes that safe is the pointers rather than the scheduler: one writer each, 32-bit aligned, and each is
	// advanced only after its own data is in place, which is the ordinary single-producer/single-consumer ring.
	// Both sides run on the same core, so there is no cache to reconcile - only the compiler, which volatile holds.

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
