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
#include "OSLikeStuff/timers_interrupts/clock_type.h"
#include "OSLikeStuff/timers_interrupts/timers_interrupts.h"
#include "RZA1/mtu/mtu.h"
#include "RZA1/ostm/ostm.h"
#include "RZA1/system/iodefine.h"
#include "RZA1/system/iodefines/usb20_iodefine.h"
#include "RZA1/usb/r_usb_basic/r_usb_basic_if.h"
#include "definitions.h"
#include "deluge/drivers/usb/usb_setup_trace.h"
#include "deluge/drivers/usb/userdef/r_usb_paudio_config.h"

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
constexpr uint32_t kMaxFramesPerPacket = kIsoLimitBytes / kFrameBytes;
constexpr uint32_t kMaxPacketBytes = kMaxFramesPerPacket * kFrameBytes;

static_assert(kMaxFramesPerPacket > kFramesPerPacket,
              "A packet must hold more than the nominal frame count or the stream can never keep up");

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
struct QueuedPacket {
	alignas(4) uint8_t data[kMaxPacketBytes];
	uint16_t bytes;
};
QueuedPacket packetQueue[kQueueSlots];
volatile uint32_t queueWrite = 0;
volatile uint32_t queueRead = 0;

/// The window opens 700-799 us into the host's frame on 84% of observations, and never at the frame boundary -
/// which is why driving writes from the frame interrupt itself delivered 0.42x. Rather than track the host's
/// phase with a control loop, the timer walks its own period forward until a write lands and then holds. A
/// search that stops when it succeeds cannot oscillate; a loop that integrates phase error can.
constexpr uint32_t kTimerTicksPerMs = DELUGE_CLOCKS_PER / 1000u;

constexpr uint32_t kPhaseStepTicks = kTimerTicksPerMs / 20u; ///< 50 us

/// b13 of INTENB0: the frame-update interrupt. Off in device mode by default - the driver only ever set it for
/// host mode - and the peripheral handler's frame branch already existed doing nothing.
constexpr uint16_t kIntEnbSofe = 0x2000u;
bool sofEnabled = false;

/// How long after the host's frame boundary to attempt the write.
///
/// This is the whole point of clocking from SOF. Three free-running versions of this timer have now been
/// measured and every one of them lost frames: 990 fires/s at a nominal millisecond, 1006 at 3% short, and
/// nothing in between tracks the host, because the host's frame clock is its crystal and this timer's is ours.
/// A search for the window in that reference frame has to re-find it forever. Measured from SOF instead, the
/// window is stationary - 700-799 us into the frame on 84% of observations, 2026-08-26 - so the offset
/// converges once and holds.
uint32_t sofToWriteTicks = (kTimerTicksPerMs * 3u) / 4u; ///< 750 us
constexpr uint32_t kSofOffsetMin = kTimerTicksPerMs / 10u;
constexpr uint32_t kSofOffsetMax = (kTimerTicksPerMs * 19u) / 20u;
uint32_t statSofSeen = 0;

/// Only leave a working offset after several frames of failure. The first cut stepped on every miss and wrapped
/// at the top, so it cycled instead of converging - it reported 649 us against a window measured at 700-799.
uint32_t consecutiveShut = 0;
constexpr uint32_t kShutBeforeStepping = 4u;

/// How long to keep waiting for the FIFO to report itself ready, in ticks of the 29.6 ns superfast timer.
/// 128 ticks is ~3.8 us - well inside the frame, and bounded.
constexpr uint16_t kFrdyWaitTicks = 128u;
uint32_t statFrdyImmediate = 0;     ///< ready inside the vendored 100 ns
uint32_t statFrdyLate = 0;          ///< ready only because we waited longer - what the old timeout was discarding
uint32_t statFrdyNever = 0;         ///< genuinely not ready at all
uint32_t statCatchUp = 0;           ///< second writes in one fire, recovering a frame whose fire never happened
constexpr int kAudioWriteTimer = 3; ///< Timer 3 is the only unused MTU2 channel.
uint32_t statTimerFires = 0;
uint32_t statTimerWrote = 0;
uint32_t statTimerShut = 0;
uint32_t statTimerEmpty = 0;
uint32_t statQueueBuilt = 0;
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
	}
	lastCompletionFrame = frameAtCompletion;
	lastCompletionFrameValid = true;

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
	if (streamActive && transferInitialised) {
		// Measured across the write rather than timed, because what the chip cares about is which frame the write
		// finished in, not how long it took - a fast write that lands after the SOF is as late as a slow one.
		const uint32_t delta = (uint32_t)((readFrameNumber() - frameAtCompletion) & kFrameNumberMask);
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

/// DIAGNOSTIC. Marks a packet the device sent with nothing real in it.
///
/// A packet of deliberate silence and a packet that never arrived reach the host as the same zeroes, and they
/// mean opposite things - one is the transport keeping the endpoint answering, the other is the transport
/// failing. Only the device can write this mark, so the recording separates them.
///
/// Channels 3 upwards carry diagnostics and nothing real, so this costs no audio. Channels 1 and 2 are
/// untouched: the mix has to stay listenable for the recording to be worth anything else.
void markDeviceSilence(uint8_t* buffer, uint32_t frames) {
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
	int16_t* samples = (int16_t*)buffer;
	const int16_t number = encodeStamp(packetCounter++);
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
	statBuilds++;
	// Nominal packet size, used only while there is nothing real to send. 44.1 kHz does not divide into 1 ms
	// frames, so 44 frames with a 45th every tenth packet averages exactly 44100 rather than drifting.
	uint32_t frames = kFramesPerPacket;
	frameAccumulator += kFrameRemainder;
	if (frameAccumulator >= 1000) {
		frames++;
		frameAccumulator -= 1000;
	}

	// Signed, and the pointers are free-running rather than masked to the ring: a masked subtraction cannot
	// go negative, so a reader one frame past the writer read as a full ring and triggered a resync that threw
	// away thousands of frames that never existed. Measured firing 2.5 times a second under load, 2026-08-26.
	const int32_t available32 = (int32_t)(writeFrame - readFrame);
	uint32_t available = (available32 > 0) ? (uint32_t)available32 : 0u;

	// Hold the first packets back until there is a window's worth of slack, so the very first late render does
	// not produce a gap the host has to hear.
	if (!primed) {
		if (available < kLeadFrames) {
			statPrimeSilence++;
			memset(buffer, 0, frames * kFrameBytes);
			markDeviceSilence(buffer, frames);
			stampPacketNumber(buffer, frames);
			return frames;
		}
		primed = true;
	}

	// The host is not draining us fast enough to keep up. A rate trim cannot recover this in useful time, so
	// put the read pointer back at the lead and take the one discontinuity.
	if (available >= kResyncFrames) {
		// Signed, deliberately: the masked `available` above cannot tell "the ring holds 8191 frames" from "the
		// reader is one frame past the writer". This can, and the two want opposite responses.
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
		statFramesDiscarded += available - kLeadFrames;
		readFrame = writeFrame - kLeadFrames;
		statReadAdvResync++;
		available = kLeadFrames;
		statResyncs++;
	}

	if (available == 0u) {
		// The engine has not produced anything since the last poll. Send correctly sized silence rather than
		// nothing, so the endpoint keeps answering, and do not advance the read pointer.
		statUnderruns++;
		memset(buffer, 0, frames * kFrameBytes);
		markDeviceSilence(buffer, frames);
		stampPacketNumber(buffer, frames);
		return frames;
	}

	// Send what the engine actually produced. Never a partial audio frame: a host that appends one rotates the
	// channel mapping permanently and silently.
	uint32_t send = available;
	if (send > kMaxFramesPerPacket) {
		send = kMaxFramesPerPacket;
	}

	const uint32_t start = readFrame & kRingMask;
	const uint32_t firstRun = (start + send > kRingFrames) ? (kRingFrames - start) : send;
	memcpy(buffer, &ring[start * kChannels], firstRun * kFrameBytes);
	if (firstRun < send) {
		memcpy(&buffer[firstRun * kFrameBytes], &ring[0], (send - firstRun) * kFrameBytes);
	}
	readFrame = readFrame + send;
	statReadAdvBuild++;

	statPacketsSent++;
	statFramesSent += send;

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
	emit(" frl");
	emitDec(statFrdyLate);
	emit(" frn");
	emitDec(statFrdyNever);
	emit(" cup");
	emitDec(statCatchUp);
	emit(" sofo");
	emitDec(sofToWriteTicks * 1000u / kTimerTicksPerMs); ///< the settled offset, in us
	emit(" tem");
	emitDec(statTimerEmpty);
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
	*p = '\0';
	Debug::sysexDebugPrint(*Debug::midiDebugCable, planeLine, true);

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
	statFrdyLate = 0;
	statFrdyNever = 0;
	statCatchUp = 0;
	statTimerEmpty = 0;
	statQueueBuilt = 0;
	statBuilds = 0;
	statResyncGenuine = 0;
	statResyncOvertaken = 0;
	// statWorstOvertake is deliberately not cleared: the furthest the reader has ever got past the writer is
	// the question, not how far it got in the last second.
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
	// usbBempBitsSeen is deliberately not cleared: which pipes are live at all is the question, not how often.
}

/// Pushes one pre-built packet into the pipe, from the timer interrupt.
///
/// Does no building and touches neither the ring nor the sequence counter: those belong to the task, because a
/// ~990-byte copy out of the ring inside a 1 kHz interrupt is what starved the audio engine on v0.14.0. All this
/// does is steer the shared FIFO port and push bytes that are already sitting in memory.
/// Puts the shared FIFO port back exactly as it was found.
///
/// usb_cstd_is_set_frdy_rohan re-points the port at our pipe and never restores it. That is harmless when only
/// the driver's own single-threaded paths use it, and not harmless at all from a 1 kHz interrupt: landing inside
/// a MIDI FIFO write leaves the rest of MIDI's bytes going into our pipe, which reaches the host as a part-frame
/// packet and rotates its channel mapping. Measured as channel constants arriving on channel 1, ~40 samples a
/// minute. The port's access width lives in the same register, so restoring it wholesale covers that too.
void restoreFifoPort(uint16_t savedSel, uint16_t savedShadow) {
	usb_regadr_t reg = usb_hstd_get_usb_ip_adr(USB_CFG_USE_USBIP);
	fifoSels[kUseCpuFifo] = savedShadow;
	uint16_t seen;
	do {
		reg->CFIFOSEL = savedSel;
		seen = reg->CFIFOSEL;
	} while ((seen & (kFifoSelIsel | kFifoSelCurPipe)) != (savedSel & (kFifoSelIsel | kFifoSelCurPipe)));
}

/// Keeps waiting where the vendored call gives up.
///
/// usb_cstd_is_set_frdy_rohan re-points the shared FIFO port at our pipe and then allows FRDY just 5 ticks of
/// the 29.6 ns timer - 100 ns - to assert. The port change is already done by the time it returns an error, so
/// this carries on polling the same register. If a large share of writes are rescued here, the "plane shut"
/// this build has been chasing was never the plane: it was a timeout too short for the port change to settle,
/// and it read as a stable 16-19% loss on every version of this timer, free-running or SOF-clocked.
uint16_t extendFifoWait() {
	volatile uint16_t* const ctr = (volatile uint16_t*)&(USB201.CFIFOCTR);
	const uint16_t start = *TCNT[TIMER_SYSTEM_SUPERFAST];
	while ((uint16_t)(*TCNT[TIMER_SYSTEM_SUPERFAST] - start) < kFrdyWaitTicks) {
		const uint16_t buffer = *ctr;
		if ((buffer & kFifoCtrFrdy) != 0u) {
			statFrdyLate++;
			return buffer;
		}
	}
	statFrdyNever++;
	return kFifoError;
}

void audioWriteTimerInterrupt(uint32_t /*intSense*/) {
	timerClearCompareMatchTGRA(kAudioWriteTimer);
	statTimerFires++;

	if (!streamActive || !transferInitialised) {
		return;
	}
	if (queueWrite == queueRead) {
		statTimerEmpty++;
		return;
	}

	// Up to one extra write per fire, to recover a frame whose fire never happened. 61 of the host's 1000
	// frames a second produced no timer interrupt at all under load - the handler is delayed past its own next
	// deadline and two compare matches collapse into one - and one write per fire can never make that up.
	//
	// It cannot overrun the host: a plane only becomes free when the host has taken a packet, so the free plane
	// is the host's clock rather than ours. If nothing is free the second attempt costs one register read.
	for (uint32_t writesThisFire = 0; writesThisFire < 2u; writesThisFire++) {
		if (writesThisFire > 0u && queueWrite == queueRead) {
			break;
		}

		// The FIFO port is shared with MIDI and the readiness check steers it to our pipe, so nothing may run in
		// between. This interrupt can itself be preempted, so the guard is explicit rather than assumed.
		DISABLE_ALL_INTERRUPTS();
		usb_regadr_t reg = usb_hstd_get_usb_ip_adr(USB_CFG_USE_USBIP);
		const uint16_t savedSel = reg->CFIFOSEL;
		const uint16_t savedShadow = fifoSels[kUseCpuFifo];
		// Restore only if the port was pointing somewhere else, which is the only case where anything was borrowed.
		// Restoring unconditionally costs a pipe change on every write, and usb_cstd_is_set_frdy_rohan gives up if
		// the port is not ready within 100 ns - measured as 753 refusals a second against 37 writes.
		const bool borrowed = (savedSel & kFifoSelCurPipe) != USB_CFG_PAUDIO_ISO_IN;
		uint16_t status = usb_cstd_is_set_frdy_rohan(USB_CFG_PAUDIO_ISO_IN);
		if (status == kFifoError) {
			status = extendFifoWait();
		}
		else {
			statFrdyImmediate++;
		}
		if (status == kFifoError) {
			if (borrowed) {
				restoreFifoPort(savedSel, savedShadow);
			}
			ENABLE_ALL_INTERRUPTS();
			statTimerShut++;
			// Nothing writable now, so a second attempt this fire cannot succeed either.
			// Step the offset from the frame boundary, not the timer's period. Against SOF the window does not
			// move, so this converges and then stops - unlike a period walk, which corrects a phase error that the
			// two crystals immediately reintroduce.
			if (++consecutiveShut >= kShutBeforeStepping) {
				consecutiveShut = 0;
				sofToWriteTicks += kPhaseStepTicks;
				if (sofToWriteTicks > kSofOffsetMax) {
					sofToWriteTicks = kSofOffsetMin;
				}
			}
			return;
		}

		QueuedPacket& q = packetQueue[queueRead % kQueueSlots];
		usb_pstd_write_fifo(q.bytes, kUseCpuFifo, q.data);
		if ((hw_usb_read_fifoctr(nullptr, kUseCpuFifo) & kFifoCtrBval) == 0u) {
			hw_usb_set_bval(nullptr, kUseCpuFifo);
		}
		if (borrowed) {
			restoreFifoPort(savedSel, savedShadow);
			statPortBorrowed++;
		}
		ENABLE_ALL_INTERRUPTS();
		queueRead++;
		statTimerWrote++;
		statCatchUp += (writesThisFire > 0u) ? 1u : 0u;

		// A write landed, so this offset is the right one. Hold it; SOF re-arms the timer for the next frame.
		consecutiveShut = 0;
	}
}

/// Fills the queue from the task, where the copy is affordable.
void buildQueuedPackets() {
	while ((uint32_t)(queueWrite - queueRead) < kQueueSlots) {
		QueuedPacket& q = packetQueue[queueWrite % kQueueSlots];
		const uint32_t frames = buildNextPacket(q.data);
		q.bytes = (uint16_t)(frames * kFrameBytes);
		// The slot's contents must be visible before the index that publishes it.
		__asm volatile("DSB" ::: "memory");
		queueWrite++;
		statQueueBuilt++;
	}
}

/// Called from the peripheral interrupt handler's frame branch, on the host's own clock, 1000 times a second.
///
/// Does no work beyond re-arming the timer: two register writes. v0.14.0 put the ~990-byte ring copy and the
/// FIFO write in here and starved the audio engine, clipping channels 1-2 to full scale. The task builds the
/// packets now and the timer only pushes bytes already in memory, so the expensive half of that is gone and the
/// cheap half happens 750 us from here rather than at the boundary, where the plane is reliably shut.
extern "C" void usbAudioStreamStartOfFrame(void) {
	statSofSeen++;
	if (!audioTimerRunning) {
		return;
	}
	// Zeroing the count is what locks the phase: the timer now measures from the host's frame boundary rather
	// than from its own last fire, so its crystal sets only the offset's accuracy and never accumulates against
	// the host's rate.
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
		readFrame = writeFrame;
		statReadAdvPark++;
		queueRead = queueWrite;
		firstPacketSubmitted = false;
		stopAudioWriteTimer();
		if (sofEnabled) {
			usb_hstd_get_usb_ip_adr(USB_CFG_USE_USBIP)->INTENB0 &= (uint16_t)~kIntEnbSofe;
			sofEnabled = false;
		}
		return;
	}
	streamActive = true;
	statAlt1++;

	// Unconditionally, before any branch. This used to live inside the in-flight branch alone, which gave the
	// task a state it could never leave: once a transfer ended without a new one starting, every pass took the
	// other branch, the queue was never refilled, and the timer starved on it forever. Measured 2026-08-26
	// evening as inf0 / qb0 / tw0 with the ring lead climbing past 1.2 million frames, and again 2026-08-27 with
	// 0.45 s of unique audio delivered in 30 s. Keeping packets ready is the task's job in every state, not only
	// in the one where the driver happens to be busy.
	buildQueuedPackets();

	if (transferInFlight) {
		statInFlight++;
		// Sampled here rather than at the top of the routine: a transfer is outstanding, which is the only
		// condition under which "is a plane writable" is the question being asked.
		const uint16_t ctr = readPipeCtr();
		statReadyState[(((ctr & kPipeCtrBsts) != 0u) ? 2u : 0u) | (((ctr & kPipeCtrInBufM) != 0u) ? 1u : 0u)]++;
		// Data pending and a plane writable: the second plane can be loaded now, and this is the only moment it
		// can be. Anything else is either nothing to double up on or a port that would refuse the write.
		if ((ctr & (kPipeCtrBsts | kPipeCtrInBufM)) != (kPipeCtrBsts | kPipeCtrInBufM)) {
			statExtraNoWindow++;
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
	DISABLE_ALL_INTERRUPTS();
	sendNextPacket();
	ENABLE_ALL_INTERRUPTS();
	firstPacketSubmitted = transferInFlight;
	// Started only once a host is actually streaming and the driver's own first transfer is set up, so an idle
	// Deluge pays nothing and the timer never fires against an unconfigured pipe.
	startAudioWriteTimer();
	if (!sofEnabled && transferInitialised) {
		usb_hstd_get_usb_ip_adr(USB_CFG_USE_USBIP)->INTENB0 |= kIntEnbSofe;
		sofEnabled = true;
	}
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

	statFramesIn += numSamples;

	uint32_t w = writeFrame;
	for (uint32_t i = 0; i < numSamples; i++) {
		int16_t* const frame = &ring[(w & kRingMask) * kChannels];
		// The samples handed here are already at output scale, so this is only a width reduction.
		frame[0] = (int16_t)(mix[i].l >> 16);
		frame[1] = (int16_t)(mix[i].r >> 16);
		// DIAGNOSTIC. The producer's own frame count, travelling with the audio it belongs to. Whatever reaches
		// the host carries the number of the frame the engine rendered, so the recording alone says which frames
		// were delivered and which were dropped on the way. Remove with the rest of the diagnostics.
		const uint32_t block = w >> kStampBlockShift;
		if (block != stampBlock) {
			stampBlock = block;
			stampLow = encodeStamp(block);
			stampHigh = encodeStamp(block >> kStampBits);
		}
		frame[2] = stampLow;
		frame[3] = stampHigh;
		frame[4] = kMarkRendered;
		w++;
	}
	writeFrame = w;
}

} // namespace deluge::processing::engines

extern "C" void usbAudioStreamRoutine() {
	deluge::processing::engines::USBAudioStream::routine();
}
