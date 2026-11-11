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

#include "drivers/usb/usb_audio.h"
#include "definitions.h"
#include "io/debug/log.h"
#include "memory/general_memory_allocator.h"
#include "model/usb_route.h"
#include "storage/flash_storage.h"
#include "timers_interrupts/timers_interrupts.h"
#include "util/functions.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <cstring>
#include <new>

extern "C" {
#include "RZA1/intc/devdrv_intc.h"
#include "RZA1/system/iodefine.h"
#include "RZA1/usb/r_usb_basic/r_usb_basic_if.h"
#include "RZA1/usb/r_usb_basic/src/hw/inc/r_usb_bitdefine.h"
#include "drivers/dmac/dmac.h"
#include "drivers/ssi/ssi.h"
#include "drivers/usb/userdef/r_usb_paudio_config.h"

void usb_cstd_set_buf(usb_utr_t* ptr, uint16_t pipe);
void usb_cstd_set_nak(usb_utr_t* ptr, uint16_t pipe);
void usb_cstd_clr_stall(usb_utr_t* ptr, uint16_t pipe);
void v7_dma_flush_range(uint32_t start, uint32_t end);
}

namespace deluge::usb_audio {
// Defaults make the Deluge work as an audio interface out of the box
bool directMonitoring = true;
OutputSource outputSource = OutputSource::MAIN_MIX;
uint8_t compensationMs = 0;
} // namespace deluge::usb_audio

namespace {

// Pipe 1 (from host) is read through D1FIFO by DMA channel 1; pipe 2 (to host) is written through D0FIFO by channel 0
constexpr uint16_t kPipeFromHost = USB_CFG_PAUDIO_ISO_OUT;
constexpr uint16_t kPipeToHost = USB_CFG_PAUDIO_ISO_IN;
constexpr uint16_t kPipeBits = (1 << kPipeFromHost) | (1 << kPipeToHost);
constexpr uint32_t kMaxPacketBytes = USB_PAUDIO_MAX_PACKET_BYTES; // From the host, in either mode
constexpr uint32_t kBytesPerFrame = USB_PAUDIO_BYTES_PER_FRAME;
static_assert(kBytesPerFrame == sizeof(StereoSample), "USB audio frames are copied straight to and from StereoSamples");
// To the host, the larger of the two modes' packets
constexpr uint32_t kToHostMaxPacketBytes = std::max<uint32_t>(kMaxPacketBytes, USB_PAUDIO_MT_TO_HOST_MAX_PACKET_BYTES);

constexpr uint32_t kMtChannelsToHost = USB_PAUDIO_MT_CHANNELS_TO_HOST;
constexpr uint32_t kMtToHostBytesPerFrame = USB_PAUDIO_MT_TO_HOST_BYTES_PER_FRAME;
static_assert(kMtToHostBytesPerFrame % 4 == 0, "packets must stay a whole number of 32-bit words for the FIFO DMA");

constexpr uint8_t kInterruptPriority = 9; // Same as the USB interrupt, so neither can interrupt the other

// DMA request sources for USB0's D0FIFO and D1FIFO - the same as r_usb_dma.c uses
constexpr uint32_t kDmarsD0Fifo = 0x83;
constexpr uint32_t kDmarsD1Fifo = 0x87;

// CHCFG: requests come from the USB module, level-detected, active high, in bus cycle mode; 32-bit accesses at both
// ends; transfer end interrupt enabled
constexpr uint32_t kChcfgCommon = (0b010 << 8)            // AM: bus cycle mode
                                  | (1 << 6)              // LVL: level detection
                                  | (1 << 5)              // HIEN: active high
                                  | (0b010 << 12)         // SDS: 32-bit
                                  | (0b010 << 16);        // DDS: 32-bit
constexpr uint32_t kChcfgToHost = kChcfgCommon | (1 << 3) // REQD: the request comes from the destination (the FIFO)
                                  | (1 << 21)             // DAD: destination address fixed
                                  | (USB_AUDIO_TX_DMA_CHANNEL & 7);
constexpr uint32_t kChcfgFromHost = kChcfgCommon | (1 << 20) // SAD: source address fixed
                                    | (USB_AUDIO_RX_DMA_CHANNEL & 7);

// Each direction has one packet's worth of staging memory for the DMA. Whole cache lines, so nothing else shares them,
// and the CPU only touches them through the uncached mirror
alignas(CACHE_LINE_SIZE) uint32_t toHostStaging[kToHostMaxPacketBytes / sizeof(uint32_t)];
alignas(CACHE_LINE_SIZE) uint32_t fromHostStaging[kMaxPacketBytes / sizeof(uint32_t)];
static_assert(sizeof(toHostStaging) % CACHE_LINE_SIZE == 0);

template <typename T>
T* uncached(T* address) {
	return reinterpret_cast<T*>(reinterpret_cast<uint32_t>(address) + UNCACHED_MIRROR_OFFSET);
}

// Copies between a ring (indexed by a free-running frame count) and a flat buffer, in at most two contiguous runs
template <size_t N>
void copyFromRing(std::array<StereoSample, N> const& ring, uint32_t start, StereoSample* destination, size_t count) {
	static_assert((N & (N - 1)) == 0);
	uint32_t index = start & (N - 1);
	size_t first = std::min<size_t>(count, N - index);
	std::memcpy(destination, &ring[index], first * sizeof(StereoSample));
	std::memcpy(destination + first, &ring[0], (count - first) * sizeof(StereoSample));
}

template <size_t N>
void copyToRing(std::array<StereoSample, N>& ring, uint32_t start, StereoSample const* source, size_t count) {
	static_assert((N & (N - 1)) == 0);
	uint32_t index = start & (N - 1);
	size_t first = std::min<size_t>(count, N - index);
	std::memcpy(&ring[index], source, first * sizeof(StereoSample));
	std::memcpy(&ring[0], source + first, (count - first) * sizeof(StereoSample));
}

// Frames from the host, written by the DMA-complete interrupt, read by readFromHost() in the audio routine
constexpr uint32_t kFromHostRingFrames = 1024;
std::array<StereoSample, kFromHostRingFrames> fromHostRing;
std::atomic<uint32_t> fromHostWritten{0}; // Free-running frame counts
std::atomic<uint32_t> fromHostRead{0};
std::atomic<uint32_t> fromHostGeneration{0}; // Bumped each time the stream (re)starts

// Frames to the host, written by the audio routine, read by the SOF interrupt. Stereo frames in stereo mode;
// multitrack mode points this at a ring of its own frames, allocated when it starts
constexpr uint32_t kToHostRingFrames = 1024;
std::array<StereoSample, kToHostRingFrames> toHostStereoRing;
uint8_t* toHostRing = reinterpret_cast<uint8_t*>(toHostStereoRing.data());
uint32_t toHostBytesPerFrame = sizeof(StereoSample);
// The to-host endpoint's wMaxPacketSize in the current mode, and the size of the packet being sent
uint32_t toHostMaxPacketBytes = USB_PAUDIO_MAX_PACKET_BYTES;
uint32_t toHostDmaBytes = 0;

// Copies frames out of toHostRing, in at most two contiguous runs
void copyFromToHostRing(uint32_t start, void* destination, size_t count) {
	uint32_t index = start & (kToHostRingFrames - 1);
	size_t first = std::min<size_t>(count, kToHostRingFrames - index);
	auto* dest = static_cast<uint8_t*>(destination);
	std::memcpy(dest, &toHostRing[index * toHostBytesPerFrame], first * toHostBytesPerFrame);
	std::memcpy(dest + first * toHostBytesPerFrame, toHostRing, (count - first) * toHostBytesPerFrame);
}

std::atomic<uint32_t> toHostWritten{0};
std::atomic<uint32_t> toHostRead{0};
// Maps frame counts in toHostRing to positions in the SSI TX buffer: (count + this) % SSI_TX_BUFFER_NUM_SAMPLES. -1
// until sendToHost() has given us something to go by
std::atomic<int32_t> toHostTxOffset{-1};

volatile bool fromHostStreaming = false;
volatile bool toHostStreaming = false;
volatile bool fromHostDmaBusy = false;
volatile bool toHostDmaBusy = false;
uint32_t fromHostDmaBytes;

// The SOF interrupt (every 1ms while either direction's streaming) doubles as a watchdog, so that nothing that goes
// wrong - a BRDY that slipped past, a DMA that never finished or whose interrupt never came, a pipe that stopped
// accepting packets - can leave a stream dead until the host restarts it. These count the SOF interrupts each DMA's
// been running for
constexpr uint32_t kDmaTimeoutFrames = 4; // A packet's DMA takes microseconds, so this is never a slow one
uint32_t fromHostDmaFrames = 0;
uint32_t toHostDmaFrames = 0;
// How many times the watchdog had to step in - for looking at in a debugger
uint32_t fromHostRecoveries = 0;
uint32_t toHostRecoveries = 0;

// SSI-RX-shaped ring holding the audio from the host, for the rest of the Deluge to read like an input
PLACE_SDRAM_BSS alignas(CACHE_LINE_SIZE) int32_t inputRing[SSI_RX_BUFFER_NUM_SAMPLES * NUM_MONO_INPUT_CHANNELS];
uint32_t inputRingWriteFrame = 0; // Where fillInputRing() got up to, in frames from the start
// How many frames of silence have been written since the host last sent anything. Once it's the whole ring, there's
// nothing left to clear
uint32_t inputRingSilentFrames = 0;

bool interruptsSetUp = false;

// Latched from FlashStorage::usbAudioMode each time USB starts as a peripheral
uint8_t mode = USB_PAUDIO_MODE_OFF;
bool active = false;
bool multitrack = false;

/*********************************************************************************************************************
 * Multitrack mode's memory, allocated the first time it starts and kept from then on, so nobody using stereo mode or
 * no USB audio pays for it
 */

constexpr uint32_t kRenderWindowFrames = SSI_TX_BUFFER_NUM_SAMPLES;

struct MultitrackBuffers {
	// The tracks routed to each of the host's channels, for the current render window, at the song mix's scale
	std::array<std::array<int32_t, kRenderWindowFrames>, kMtChannelsToHost> channels;
	// The mix before the track being rendered, so what it added can be found
	std::array<StereoSample, kRenderWindowFrames> snapshot;
	// Frames to the host, in the format on the wire
	std::array<uint8_t, kToHostRingFrames * kMtToHostBytesPerFrame> toHostRing;
	// Tracks playing the host's audio this window, added to the mix after the delay compensation
	std::array<StereoSample, kRenderWindowFrames> heldBack;
	// Channels 3-4 from the host, laid out like inputRing
	// Only the CPU touches it, so it needs no more alignment than the allocator gives
	std::array<int32_t, SSI_RX_BUFFER_NUM_SAMPLES * NUM_MONO_INPUT_CHANNELS> inputRing2;
};

// The compensation delay: 8192 frames, against a 100ms (4410-frame) longest delay
constexpr uint32_t kCompensationFrames = 8192;
static_assert(deluge::usb_audio::kCompensationMaxMs * USB_PAUDIO_SAMPLE_RATE / 1000 < kCompensationFrames);
struct CompensationBuffers {
	std::array<StereoSample, kCompensationFrames> mix;
	std::array<int32_t, kCompensationFrames> reverbSend;
};

MultitrackBuffers* mt = nullptr;
CompensationBuffers* compensation = nullptr; // In SDRAM - it's big and only streamed through

bool allocateMultitrackBuffers() {
	if (mt != nullptr) {
		return true;
	}
	auto& allocator = GeneralMemoryAllocator::get();
	void* buffers = allocator.allocMaxSpeed(sizeof(MultitrackBuffers));
	if (buffers == nullptr) {
		return false;
	}
	void* line = allocator.allocLowSpeed(sizeof(CompensationBuffers));
	if (line == nullptr) {
		allocator.dealloc(buffers);
		return false;
	}
	mt = new (buffers) MultitrackBuffers{};
	compensation = new (line) CompensationBuffers{};
	return true;
}

// Which channels the current render window has written, as bits. The first track to reach a channel sets it, any after
// that add to it - so nothing needs clearing for a channel nobody routes to
uint16_t channelsWritten = 0;
size_t captureWindowFrames = 0;
bool anythingHeldBack = false;

uint32_t compensationWrite = 0;
uint32_t compensationFramesNow = 0; // The delay the line's running at. 0 while compensation's off

/*********************************************************************************************************************
 * DMA, and the interrupts driving it
 */

void stopDma(int32_t channel) {
	DMACn(channel).CHCTRL_n = DMAC0_CHCTRL_n_CLREN | DMAC0_CHCTRL_n_CLRRQ;
	for (int32_t i = 0; i < 10000 && (DMACn(channel).CHSTAT_n & (DMAC0_CHSTAT_n_EN | DMAC0_CHSTAT_n_TACT)); i++) {}
	DMACn(channel).CHCTRL_n = DMAC0_CHCTRL_n_SWRST;
}

bool dmaFinished(int32_t channel) {
	return DMACn(channel).CHSTAT_n & (DMAC0_CHSTAT_n_END | DMAC0_CHSTAT_n_TC); // Cleared by startDma()
}

void startDma(int32_t channel, uint32_t source, uint32_t destination, uint32_t numBytes) {
	auto& dma = DMACn(channel);
	dma.CHCTRL_n = DMAC0_CHCTRL_n_SWRST | DMAC0_CHCTRL_n_CLRTC | DMAC0_CHCTRL_n_CLREND;
	dma.N0SA_n = source;
	dma.N0DA_n = destination;
	dma.N0TB_n = numBytes;
	dma.CHCTRL_n = DMAC0_CHCTRL_n_SETEN;
}

// Points a FIFO port at a pipe, ready for 32-bit access, and waits for the controller to confirm
bool selectPipe(volatile uint16_t& fifoSel, uint16_t pipe) {
	fifoSel = USB_MBW_32 | pipe;
	for (int32_t i = 0; i < 10000; i++) {
		if ((fifoSel & USB_CURPIPE) == pipe) {
			return true;
		}
	}
	return false;
}

// If a packet from the host is waiting in the pipe's buffer, start the DMA reading it out
void startFromHostDma() {
	if (fromHostDmaBusy || !fromHostStreaming) {
		return;
	}
	uint16_t fifoCtr = USB200.D1FIFOCTR;
	if (!(fifoCtr & USB_FRDY)) {
		return;
	}
	uint32_t numBytes = fifoCtr & USB_DTLN;
	if (numBytes == 0 || numBytes > kMaxPacketBytes) {
		USB200.D1FIFOCTR = USB_BCLR; // Nothing we can use - throw it away
		return;
	}
	fromHostDmaBytes = numBytes;
	fromHostDmaBusy = true;
	fromHostDmaFrames = 0;
	startDma(USB_AUDIO_RX_DMA_CHANNEL, reinterpret_cast<uint32_t>(&USB200.D1FIFO.UINT32),
	         reinterpret_cast<uint32_t>(fromHostStaging), (numBytes + 3) & ~3u);
	USB200.D1FIFOSEL |= USB_DREQE;
}

void fromHostDmaComplete(uint32_t) {
	if (!fromHostDmaBusy || !dmaFinished(USB_AUDIO_RX_DMA_CHANNEL)) {
		return; // Late - the watchdog's already dealt with this transfer
	}
	USB200.D1FIFOSEL &= ~USB_DREQE;

	if (fromHostStreaming) {
		auto const* frames = uncached(reinterpret_cast<StereoSample const*>(fromHostStaging));
		uint32_t numFrames = fromHostDmaBytes / kBytesPerFrame;
		uint32_t written = fromHostWritten.load(std::memory_order_relaxed);
		uint32_t space = kFromHostRingFrames - (written - fromHostRead.load(std::memory_order_acquire));
		numFrames = std::min(numFrames, space); // If the reader's that far behind, it'll be resyncing anyway
		copyToRing(fromHostRing, written, frames, numFrames);
		fromHostWritten.store(written + numFrames, std::memory_order_release);
	}

	fromHostDmaBusy = false;
	startFromHostDma(); // The pipe is double buffered, so the next packet might already be there
}

// How many frames are waiting in the SSI TX buffer for the codec to play, up to and including txFrame (an index into
// it). 1 to SSI_TX_BUFFER_NUM_SAMPLES - the audio engine counts both ends being in the same place as it being full
uint32_t txFramesQueued(uint32_t txFrame) {
	uint32_t playing =
	    (reinterpret_cast<int32_t*>(getTxBufferCurrentPlace()) - getTxBufferStart()) / NUM_MONO_OUTPUT_CHANNELS;
	return ((txFrame - playing - 1) & (SSI_TX_BUFFER_NUM_SAMPLES - 1)) + 1;
}

// How many frames to send the host this USB frame. 44.1 on average, nudged by one either way to follow our sample
// clock - which a host doing implicit feedback then copies, so this sets the rate both ways.
//
// What we steer by is the backlog: frames the codec's played that we haven't sent yet. The audio routine renders
// anywhere up to a TX buffer (128 frames) ahead of the codec, depending on how busy it is, so how full toHostRing is
// swings around by that much on its own - which a busy Deluge made us chase, running dry and stopping to rebuild. The
// backlog only moves with the codec's clock against the USB one
constexpr int32_t kToHostTargetBacklog = 96;
constexpr int32_t kToHostTolerance = 24;
uint32_t toHostRateAccumulator = 0;
int32_t toHostAverageBacklog = 0; // 24.8 fixed point
bool toHostPrimed = false;

uint32_t framesForNextPacket(int32_t backlog) {
	if (!toHostPrimed) {
		if (backlog < kToHostTargetBacklog) {
			return 0;
		}
		toHostPrimed = true;
		toHostAverageBacklog = backlog * 256;
	}

	toHostAverageBacklog += (backlog * 256 - toHostAverageBacklog) >> 6;

	toHostRateAccumulator += USB_PAUDIO_SAMPLE_RATE / 100;
	uint32_t numFrames = toHostRateAccumulator / 10;
	toHostRateAccumulator %= 10;

	int32_t error = (toHostAverageBacklog >> 8) - kToHostTargetBacklog;
	if (error > kToHostTolerance) {
		numFrames++;
	}
	else if (error < -kToHostTolerance) {
		numFrames--;
	}
	return numFrames;
}

void toHostDmaComplete(uint32_t) {
	if (!toHostDmaBusy || !dmaFinished(USB_AUDIO_TX_DMA_CHANNEL)) {
		return; // Late - the watchdog's already dealt with this transfer
	}
	USB200.D0FIFOSEL &= ~USB_DREQE;
	// A short packet has to be ended by hand. One that fills the endpoint's max packet size is sent by the controller
	// on its own, and ending it again would send a zero-length packet from the pipe's other buffer
	if (toHostDmaBytes < toHostMaxPacketBytes) {
		USB200.D0FIFOCTR = USB_BVAL;
	}
	toHostDmaBusy = false;
}

void setUpInterrupts() {
	if (interruptsSetUp) {
		return;
	}
	v7_dma_flush_range(reinterpret_cast<uint32_t>(toHostStaging),
	                   reinterpret_cast<uint32_t>(toHostStaging) + sizeof(toHostStaging));
	v7_dma_flush_range(reinterpret_cast<uint32_t>(fromHostStaging),
	                   reinterpret_cast<uint32_t>(fromHostStaging) + sizeof(fromHostStaging));
	setupAndEnableInterrupt(toHostDmaComplete, INTC_ID_DMAINT0 + USB_AUDIO_TX_DMA_CHANNEL, kInterruptPriority);
	setupAndEnableInterrupt(fromHostDmaComplete, INTC_ID_DMAINT0 + USB_AUDIO_RX_DMA_CHANNEL, kInterruptPriority);
	interruptsSetUp = true;
}

// Gives up on a DMA that's never going to finish, throwing away the packet it was moving
void abortFromHostDma() {
	stopDma(USB_AUDIO_RX_DMA_CHANNEL);
	USB200.D1FIFOSEL &= ~USB_DREQE;
	DMACn(USB_AUDIO_RX_DMA_CHANNEL).CHCFG_n = kChcfgFromHost;
	USB200.D1FIFOCTR = USB_BCLR;
	fromHostDmaBusy = false;
}

void abortToHostDma() {
	stopDma(USB_AUDIO_TX_DMA_CHANNEL);
	USB200.D0FIFOSEL &= ~USB_DREQE;
	DMACn(USB_AUDIO_TX_DMA_CHANNEL).CHCFG_n = kChcfgToHost;
	USB200.D0FIFOCTR = USB_BCLR;
	toHostDmaBusy = false;
}

// Called each SOF while a DMA's running. Returns whether it had to step in
bool checkDma(int32_t channel, uint32_t& framesRunning, void (*complete)(uint32_t), void (*abort)()) {
	if (++framesRunning < kDmaTimeoutFrames) {
		return false;
	}
	if (dmaFinished(channel)) {
		complete(0); // Finished, but its interrupt never came
	}
	else {
		abort();
	}
	return true;
}

// Puts a pipe back to accepting packets, if anything's set it to NAK or STALL behind our back
bool ensurePipeAccepting(uint16_t pipe) {
	volatile uint16_t* pipeCtr = &USB200.PIPE1CTR + (pipe - USB_PIPE1);
	uint16_t pid = *pipeCtr & USB_PID;
	if (pid == USB_PID_BUF) {
		return false;
	}
	if (pid & USB_PID_STALL) {
		usb_cstd_clr_stall(USB_NULL, pipe);
	}
	usb_cstd_set_buf(USB_NULL, pipe);
	return true;
}

// The watchdog, for audio from the host
void serviceFromHost() {
	if (!fromHostStreaming) {
		return;
	}
	bool recovered = ensurePipeAccepting(kPipeFromHost);
	if (fromHostDmaBusy) {
		recovered |= checkDma(USB_AUDIO_RX_DMA_CHANNEL, fromHostDmaFrames, fromHostDmaComplete, abortFromHostDma);
	}
	// If a packet's waiting and nothing's reading it - because its BRDY came while the DMA was busy, and the buffer
	// wasn't ready yet when the DMA finished - then nothing else would ever start reading it, since there's no new
	// BRDY once both buffers are full
	if (!fromHostDmaBusy) {
		startFromHostDma();
		recovered |= fromHostDmaBusy;
	}
	if (recovered) {
		fromHostRecoveries++;
	}
}

void setSofInterrupt() {
	if (fromHostStreaming || toHostStreaming) {
		if (!(USB200.INTENB0 & USB_SOFE)) {
			USB200.INTSTS0 = (uint16_t)~USB_SOFR;
			USB200.INTENB0 |= USB_SOFE;
		}
	}
	else {
		USB200.INTENB0 &= ~USB_SOFE;
	}
}

/*********************************************************************************************************************
 * Starting and stopping the streams. These run in the USB task, not an interrupt
 */

void stopFromHost() {
	CriticalSectionGuard guard;
	fromHostStreaming = false;
	setSofInterrupt();
	USB200.BRDYENB &= ~(1 << kPipeFromHost);
	usb_cstd_set_nak(USB_NULL, kPipeFromHost);
	if (interruptsSetUp) {
		stopDma(USB_AUDIO_RX_DMA_CHANNEL);
	}
	fromHostDmaBusy = false;
	selectPipe(USB200.D1FIFOSEL, USB_PIPE0); // Let go of the pipe, so it can be reconfigured
}

void startFromHost() {
	setUpInterrupts();

	CriticalSectionGuard guard;
	stopDma(USB_AUDIO_RX_DMA_CHANNEL);
	if (!selectPipe(USB200.D1FIFOSEL, kPipeFromHost)) {
		D_PRINTLN("USB audio: couldn't select pipe %d", kPipeFromHost);
		return;
	}
	setDMARS(USB_AUDIO_RX_DMA_CHANNEL, kDmarsD1Fifo);
	DMACn(USB_AUDIO_RX_DMA_CHANNEL).CHCFG_n = kChcfgFromHost;
	DMACn(USB_AUDIO_RX_DMA_CHANNEL).CHITVL_n = 0;
	DMACn(USB_AUDIO_RX_DMA_CHANNEL).CHEXT_n = 0;

	fromHostDmaBusy = false;
	fromHostGeneration.fetch_add(1, std::memory_order_relaxed);
	fromHostStreaming = true;

	USB200.BRDYSTS = (uint16_t)~(1 << kPipeFromHost);
	USB200.BRDYENB |= (1 << kPipeFromHost);
	usb_cstd_set_buf(USB_NULL, kPipeFromHost); // Start accepting packets
	setSofInterrupt();                         // For the watchdog
}

void stopToHost() {
	CriticalSectionGuard guard;
	toHostStreaming = false;
	setSofInterrupt();
	usb_cstd_set_nak(USB_NULL, kPipeToHost);
	if (interruptsSetUp) {
		stopDma(USB_AUDIO_TX_DMA_CHANNEL);
	}
	toHostDmaBusy = false;
	selectPipe(USB200.D0FIFOSEL, USB_PIPE0);
}

void startToHost() {
	setUpInterrupts();

	CriticalSectionGuard guard;
	stopDma(USB_AUDIO_TX_DMA_CHANNEL);
	if (!selectPipe(USB200.D0FIFOSEL, kPipeToHost)) {
		D_PRINTLN("USB audio: couldn't select pipe %d", kPipeToHost);
		return;
	}
	setDMARS(USB_AUDIO_TX_DMA_CHANNEL, kDmarsD0Fifo);
	DMACn(USB_AUDIO_TX_DMA_CHANNEL).CHCFG_n = kChcfgToHost;
	DMACn(USB_AUDIO_TX_DMA_CHANNEL).CHITVL_n = 0;
	DMACn(USB_AUDIO_TX_DMA_CHANNEL).CHEXT_n = 0;

	// Throw away anything left over from last time
	toHostRead.store(toHostWritten.load(std::memory_order_relaxed), std::memory_order_relaxed);
	toHostTxOffset.store(-1, std::memory_order_relaxed);
	toHostPrimed = false;
	toHostRateAccumulator = 0;
	toHostDmaBusy = false;
	toHostStreaming = true;

	usb_cstd_set_buf(USB_NULL, kPipeToHost);
	setSofInterrupt(); // We send a packet each USB frame
}

/*********************************************************************************************************************
 * Reading the audio from the host. The endpoint's asynchronous with implicit feedback, so a host that supports that
 * sends us exactly as many frames as we send it, which follows our sample clock - and they can be copied straight
 * through. Hosts that ignore it send at their own clock's 44.1kHz, which drifts slowly against ours (around 4 frames a
 * second at 100ppm), so as a fallback we drop or repeat a single frame now and then to keep the ring at a steady fill.
 */

// How far ahead of the codec's playback we aim to have the host's audio, in frames. That's what's in the ring past the
// read position, plus what's already queued in the TX buffer in front of the render reading it - which is anywhere from
// almost nothing to a whole TX buffer (128 frames), depending on how busy the audio routine is. Steering by the fill
// alone, as we used to, chased those swings: a busy Deluge kept running the ring dry and restarting it. This only moves
// with the host's clock against ours. A render window can take up to 128 frames, and needs them all there already -
// the worst case for queued + window is ~190 - and packets bring ~44 at a time
constexpr uint32_t kFromHostTargetLatency = 256;
// How far the (averaged) latency can drift before we drop or repeat a frame. Packet and render timing jitter alone move
// it a lot less than this once averaged
constexpr int32_t kFromHostTolerance = 48;
// At most one correction per this many frames (~23ms): ~1000ppm of correction - far more than any clock mismatch - but
// spread out, so it's never more than a single, isolated frame
constexpr uint32_t kFromHostCorrectionInterval = 1024;

struct {
	uint32_t generation = 0;
	bool primed = false;
	uint32_t readFrame = 0;
	int32_t averageLatency = kFromHostTargetLatency << 8; // 24.8 fixed point
	uint32_t framesSinceCorrection = 0;
} fromHostReader;

// Throws away numFrames frames of audio from the host - for when the audio engine has skipped ahead
void skipFromHost(uint32_t numFrames) {
	auto& reader = fromHostReader;
	if (!reader.primed || reader.generation != fromHostGeneration.load(std::memory_order_relaxed)) {
		return; // It'll start from the latest audio anyway
	}
	uint32_t written = fromHostWritten.load(std::memory_order_acquire);
	if (numFrames >= written - reader.readFrame) {
		reader.readFrame = written;
		reader.primed = false;
	}
	else {
		reader.readFrame += numFrames;
	}
	fromHostRead.store(reader.readFrame, std::memory_order_release);
}

// Moves count frames from fromHostRing, starting at frame start, to the input rings: as they are in stereo mode, where
// each is already a StereoSample; channels 1-2 to output and 3-4 to output2, widened from 16 bits, in multitrack mode
void deliverFromHost(uint32_t start, StereoSample* output, StereoSample* output2, size_t count) {
	if (!multitrack) {
		copyFromRing(fromHostRing, start, output, count);
		return;
	}
	for (size_t i = 0; i < count; i++) {
		auto const* frame = reinterpret_cast<int16_t const*>(&fromHostRing[(start + i) & (kFromHostRingFrames - 1)]);
		output[i] = {.l = frame[0] << 16, .r = frame[1] << 16};
		output2[i] = {.l = frame[2] << 16, .r = frame[3] << 16};
	}
}

// Copies numFrames frames of audio from the host, interleaved stereo, into output - and in multitrack mode, channels
// 3-4 into output2. txQueued is how many frames the codec has to play before it gets to these
void readFromHost(int32_t* output, int32_t* output2, size_t numFrames, uint32_t txQueued) {
	size_t const bytes = numFrames * sizeof(StereoSample);
	auto& reader = fromHostReader;
	uint32_t generation = fromHostGeneration.load(std::memory_order_relaxed);
	if (reader.generation != generation) {
		// A new stream - anything left in the ring is from the last one
		reader.generation = generation;
		reader.readFrame = fromHostWritten.load(std::memory_order_acquire);
		reader.primed = false;
	}

	uint32_t written = fromHostWritten.load(std::memory_order_acquire);
	uint32_t fill = written - reader.readFrame;

	if (!reader.primed || fill > kFromHostRingFrames - 64) {
		// Waiting for enough to arrive - or way behind, because the host sped up enormously or we stalled. Either way,
		// (re)start the read position at the target fill
		if (fill > kFromHostRingFrames) {
			reader.readFrame = written; // Must have been from the previous stream
			fill = 0;
		}
		if (fill + txQueued < kFromHostTargetLatency) {
			std::memset(output, 0, bytes);
			if (output2 != nullptr) {
				std::memset(output2, 0, bytes);
			}
			fromHostRead.store(reader.readFrame, std::memory_order_release);
			return;
		}
		fill = kFromHostTargetLatency - txQueued;
		reader.readFrame = written - fill;
		reader.primed = true;
		reader.averageLatency = kFromHostTargetLatency << 8;
		reader.framesSinceCorrection = 0;
	}

	// Fallback for hosts not following our clock: nudge the read position by a frame if the latency's drifted
	reader.averageLatency += ((int32_t)((fill + txQueued) << 8) - reader.averageLatency) >> 5;
	reader.framesSinceCorrection += numFrames;
	if (reader.framesSinceCorrection >= kFromHostCorrectionInterval) {
		int32_t error = (reader.averageLatency >> 8) - (int32_t)kFromHostTargetLatency;
		if (error > kFromHostTolerance && fill > 0) {
			reader.readFrame++; // Host's faster - drop a frame
			fill--;
			reader.framesSinceCorrection = 0;
		}
		else if (error < -kFromHostTolerance && fill > numFrames) {
			reader.readFrame--; // Host's slower - repeat a frame
			fill++;
			reader.framesSinceCorrection = 0;
		}
	}

	size_t numToCopy = std::min<size_t>(numFrames, fill);
	auto* frames = reinterpret_cast<StereoSample*>(output);
	auto* frames2 = reinterpret_cast<StereoSample*>(output2);
	deliverFromHost(reader.readFrame, frames, frames2, numToCopy);
	reader.readFrame += numToCopy;

	if (numToCopy < numFrames) {
		// Ran out - the host's stopped sending, or fallen behind. Silence until there's enough again
		std::memset(&frames[numToCopy], 0, (numFrames - numToCopy) * sizeof(StereoSample));
		if (frames2 != nullptr) {
			std::memset(&frames2[numToCopy], 0, (numFrames - numToCopy) * sizeof(StereoSample));
		}
		reader.primed = false;
		D_PRINTLN("USB audio: input underrun");
	}

	fromHostRead.store(reader.readFrame, std::memory_order_release);
}

} // namespace

/*********************************************************************************************************************
 * Hooks for the Renesas USB driver
 */

extern "C" uint8_t usbAudioStartPeripheral(void) {
	usbAudioStopAll(); // Nothing should be streaming as USB starts, but the ring's about to change under it if it was
	mode = FlashStorage::usbAudioMode;
	if (mode == USB_PAUDIO_MODE_MULTITRACK && !allocateMultitrackBuffers()) {
		D_PRINTLN("USB audio: no memory for multitrack, using stereo");
		mode = USB_PAUDIO_MODE_STEREO;
	}
	active = (mode != USB_PAUDIO_MODE_OFF);
	multitrack = (mode == USB_PAUDIO_MODE_MULTITRACK);
	if (multitrack) {
		toHostRing = mt->toHostRing.data();
		toHostBytesPerFrame = kMtToHostBytesPerFrame;
		toHostMaxPacketBytes = USB_PAUDIO_MT_TO_HOST_MAX_PACKET_BYTES;
	}
	else {
		toHostRing = reinterpret_cast<uint8_t*>(toHostStereoRing.data());
		toHostBytesPerFrame = sizeof(StereoSample);
		toHostMaxPacketBytes = USB_PAUDIO_MAX_PACKET_BYTES;
	}
	return mode;
}

extern "C" bool usbAudioActive(void) {
	return active;
}

extern "C" bool usbAudioMultitrackActive(void) {
	return multitrack;
}

extern "C" void usbAudioSetInterface(uint16_t interfaceNumber, uint16_t alternateSetting) {
	if (!active) {
		return; // MIDI-only configuration - interface numbers don't mean the same
	}
	// Both of the out stream's streaming alternate settings use the same pipe and format. Only how the host times its
	// packets differs - readFromHost() copes with either
	switch (interfaceNumber) {
	case USB_PAUDIO_INTERFACE_STREAM_OUT:
		stopFromHost();
		if (alternateSetting) {
			startFromHost();
		}
		break;

	case USB_PAUDIO_INTERFACE_STREAM_IN:
		stopToHost();
		if (alternateSetting) {
			startToHost();
		}
		break;

	default:
		break;
	}
}

extern "C" void usbAudioStopAll(void) {
	if (fromHostStreaming) {
		stopFromHost();
	}
	if (toHostStreaming) {
		stopToHost();
	}
}

extern "C" uint16_t usbAudioHandleBrdy(uint16_t pipeBits) {
	uint16_t ours = pipeBits & kPipeBits;
	if (ours) {
		USB200.BRDYSTS = (uint16_t)~ours;
		if (ours & (1 << kPipeFromHost)) {
			startFromHostDma();
		}
		// The to-host pipe's BRDY just says a buffer's free. We don't enable it - SOF paces that side - but it still
		// gets flagged, and we don't want the MIDI code thinking it's for it
	}
	return ours;
}

extern "C" void usbAudioHandleSof(void) {
	serviceFromHost();

	if (!toHostStreaming) {
		return;
	}
	bool recovered = ensurePipeAccepting(kPipeToHost);
	if (toHostDmaBusy) {
		recovered |= checkDma(USB_AUDIO_TX_DMA_CHANNEL, toHostDmaFrames, toHostDmaComplete, abortToHostDma);
	}
	if (recovered) {
		toHostRecoveries++;
	}
	if (toHostDmaBusy) {
		return;
	}
	if (!(USB200.D0FIFOCTR & USB_FRDY)) {
		return; // Both buffers already hold packets the host hasn't collected
	}

	uint32_t read = toHostRead.load(std::memory_order_relaxed);
	uint32_t written = toHostWritten.load(std::memory_order_acquire);
	int32_t txOffset = toHostTxOffset.load(std::memory_order_relaxed);
	if (txOffset < 0) {
		return; // Nothing's been rendered for the host yet - it gets a zero-length packet
	}
	uint32_t fill = written - read;
	uint32_t txQueued = txFramesQueued(written + txOffset); // Of what's in the ring, how much the codec's yet to play
	if (fill > kToHostRingFrames * 3 / 4) {
		// Way behind - the host's been ignoring us. Skip ahead rather than send stale audio
		fill = txQueued + kToHostTargetBacklog;
		read = written - fill;
	}

	uint32_t numFrames = framesForNextPacket((int32_t)fill - (int32_t)txQueued);
	if (numFrames == 0) {
		toHostRead.store(read, std::memory_order_release);
		return; // The host gets a zero-length packet
	}

	// We keep sending a steady 44 or 45 frames even if the audio routine's fallen behind (when it's so busy the codec's
	// run out), padding with silence - a host doing implicit feedback takes its timing from these, so stopping to
	// catch up would throw that, and the audio coming back from it, into disarray
	uint32_t numToCopy = std::min(numFrames, fill);
	auto* staging = uncached(reinterpret_cast<uint8_t*>(toHostStaging));
	copyFromToHostRing(read, staging, numToCopy);
	std::memset(&staging[numToCopy * toHostBytesPerFrame], 0, (numFrames - numToCopy) * toHostBytesPerFrame);
	toHostRead.store(read + numToCopy, std::memory_order_release);

	toHostDmaBusy = true;
	toHostDmaFrames = 0;
	toHostDmaBytes = numFrames * toHostBytesPerFrame;
	startDma(USB_AUDIO_TX_DMA_CHANNEL, reinterpret_cast<uint32_t>(toHostStaging),
	         reinterpret_cast<uint32_t>(&USB200.D0FIFO.UINT32), toHostDmaBytes);
	USB200.D0FIFOSEL |= USB_DREQE;
}

/*********************************************************************************************************************
 * For the audio engine
 */

namespace deluge::usb_audio {

bool receivingFromHost() {
	return fromHostStreaming;
}

bool sendingToHost() {
	return toHostStreaming;
}

// Pair 1's ring exists only in multitrack mode. Without it, pair 0's ring stands in, so anything reading it still reads
// real memory - see fillInputRing() for what it hears
int32_t* inputRingFor(uint32_t pair) {
	return (pair == 1 && multitrack) ? mt->inputRing2.data() : inputRing; // Not filled outside multitrack mode
}

int32_t* getInputRingStart(uint32_t pair) {
	return inputRingFor(pair);
}

int32_t* getInputRingEnd(uint32_t pair) {
	return inputRingFor(pair) + SSI_RX_BUFFER_NUM_SAMPLES * NUM_MONO_INPUT_CHANNELS;
}

int32_t* toInputRing(int32_t const* rxPosition, uint32_t pair) {
	return inputRingFor(pair) + (rxPosition - getRxBufferStart());
}

void fillInputRing(uint32_t rxPosition, size_t numSamples, size_t txQueued) {
	constexpr uint32_t kRingFrames = SSI_RX_BUFFER_NUM_SAMPLES;
	uint32_t startFrame = (reinterpret_cast<int32_t*>(rxPosition) - getRxBufferStart()) / NUM_MONO_INPUT_CHANNELS;
	// Channels 3-4 have a ring of their own only in multitrack mode. A song that asks for them outside it reads pair
	// 0's ring instead (inputRingFor()), and gets channels 1-2 - which is still audio from the host, and is what the
	// stereo stream carries
	int32_t* ring2 = multitrack ? mt->inputRing2.data() : nullptr;

	// Normally we pick up exactly where the last render left off. If the RX position has jumped (see
	// AudioEngine::slowRoutine()), clear the gap, so anything reading back through it gets silence rather than old
	// audio. The jump's because the audio routine fell a whole TX buffer behind the codec, which the host's audio
	// didn't, so skip the same amount of that too - or it'd end up that much further ahead of the codec
	uint32_t gap = (startFrame - inputRingWriteFrame) & (kRingFrames - 1);
	if (gap != 0) {
		if (gap < kRingFrames / 2) {
			for (uint32_t frame = inputRingWriteFrame; frame != startFrame; frame = (frame + 1) & (kRingFrames - 1)) {
				inputRing[frame * 2] = 0;
				inputRing[frame * 2 + 1] = 0;
				if (ring2 != nullptr) {
					ring2[frame * 2] = 0;
					ring2[frame * 2 + 1] = 0;
				}
			}
			if (fromHostStreaming) {
				skipFromHost(gap);
			}
		}
		inputRingWriteFrame = startFrame;
	}

	while (numSamples) {
		size_t numThisTime = std::min<size_t>(numSamples, kRingFrames - inputRingWriteFrame);
		int32_t* output = &inputRing[inputRingWriteFrame * 2];
		int32_t* output2 = (ring2 != nullptr) ? &ring2[inputRingWriteFrame * 2] : nullptr;
		if (fromHostStreaming) {
			readFromHost(output, output2, numThisTime, txQueued);
			inputRingSilentFrames = 0;
		}
		else if (inputRingSilentFrames < kRingFrames) {
			std::memset(output, 0, numThisTime * sizeof(StereoSample));
			if (output2 != nullptr) {
				std::memset(output2, 0, numThisTime * sizeof(StereoSample));
			}
			inputRingSilentFrames += numThisTime;
		}
		inputRingWriteFrame = (inputRingWriteFrame + numThisTime) & (kRingFrames - 1);
		txQueued += numThisTime;
		numSamples -= numThisTime;
	}
}

namespace {

// Publishes frames written to toHostRing up to written, which go out to the codec just before txPosition
void commitToHost(uint32_t written, int32_t const* txPosition) {
	// The offset only changes when the stream (re)starts or the ring overflowed - every frame rendered normally goes
	// both here and to the TX buffer. Leaving it alone otherwise means the SOF interrupt can't catch it out of step
	// with toHostWritten, between the two stores
	uint32_t txFrame = (txPosition - getTxBufferStart()) / NUM_MONO_OUTPUT_CHANNELS;
	auto txOffset = (int32_t)((txFrame - written) & (SSI_TX_BUFFER_NUM_SAMPLES - 1));
	if (txOffset != toHostTxOffset.load(std::memory_order_relaxed)) {
		toHostTxOffset.store(txOffset, std::memory_order_relaxed);
	}
	toHostWritten.store(written, std::memory_order_release);
}

size_t toHostSpace(uint32_t written) {
	return kToHostRingFrames - (written - toHostRead.load(std::memory_order_acquire));
}

} // namespace

void sendToHost(std::span<const StereoSample> frames, int32_t const* txPosition) {
	if (!toHostStreaming || multitrack) {
		return;
	}
	uint32_t written = toHostWritten.load(std::memory_order_relaxed);
	size_t numFrames = std::min<size_t>(frames.size(), toHostSpace(written));
	copyToRing(toHostStereoRing, written, frames.data(), numFrames);
	commitToHost(written + numFrames, txPosition);
}

/*********************************************************************************************************************
 * Multitrack
 */

bool multitrackActive() {
	return multitrack;
}

bool multitrackSending() {
	return multitrack && toHostStreaming;
}

void beginTrackCapture(size_t numSamples) {
	channelsWritten = 0;
	anythingHeldBack = false;
	captureWindowFrames = std::min<size_t>(numSamples, kRenderWindowFrames);
}

void snapshotBeforeTrack(StereoSample const* mix, size_t numSamples) {
	if (!multitrack || numSamples > captureWindowFrames) {
		return;
	}
	std::memcpy(mt->snapshot.data(), mix, numSamples * sizeof(StereoSample));
}

namespace {

// What a track added to the mix, into one channel: one side of it (0 left, 1 right), or both summed to mono (-1). The
// first track to reach a channel in a window sets it, so it never needs clearing; later ones add
[[gnu::optimize("O3")]] void captureInto(uint32_t channel, StereoSample const* __restrict__ mix, size_t numSamples,
                                         int32_t side) {
	int32_t* __restrict__ out = mt->channels[channel].data();
	StereoSample const* __restrict__ before = mt->snapshot.data();
	uint16_t const bit = 1u << channel;
	bool const first = !(channelsWritten & bit);
	channelsWritten |= bit;

	for (size_t i = 0; i < numSamples; i++) {
		int32_t const l = mix[i].l - before[i].l;
		int32_t const r = mix[i].r - before[i].r;
		// Halved to mono, so a centred track keeps its level
		int32_t const value = (side < 0) ? (l >> 1) + (r >> 1) : (side == 0 ? l : r);
		out[i] = first ? value : out[i] + value;
	}
}

} // namespace

void captureTrack(uint16_t route, StereoSample const* mix, size_t numSamples) {
	if (!multitrack || numSamples > captureWindowFrames) {
		return;
	}
	for (uint32_t channel = 0; channel < kMtChannelsToHost; channel++) {
		if (route & (UsbRoute::CH1 << channel)) {
			captureInto(channel, mix, numSamples, -1);
		}
	}
	for (uint32_t pair = 0; pair < kMtChannelsToHost / 2; pair++) {
		if (route & (UsbRoute::PAIR12 << pair)) {
			captureInto(pair * 2, mix, numSamples, 0);
			captureInto(pair * 2 + 1, mix, numSamples, 1);
		}
	}
}

void removeTrackFromMix(StereoSample* mix, size_t numSamples) {
	if (!multitrack || numSamples > captureWindowFrames) {
		return;
	}
	// x - (x - y) = y in two's complement, so putting the snapshot back is exactly taking the track out
	std::memcpy(mix, mt->snapshot.data(), numSamples * sizeof(StereoSample));
}

void holdBackFromCompensation(StereoSample* mix, size_t numSamples) {
	if (!multitrack || numSamples > captureWindowFrames) {
		return;
	}
	StereoSample const* before = mt->snapshot.data();
	StereoSample* held = mt->heldBack.data();
	for (size_t i = 0; i < numSamples; i++) {
		StereoSample const added = {.l = mix[i].l - before[i].l, .r = mix[i].r - before[i].r};
		held[i] = anythingHeldBack ? StereoSample{.l = held[i].l + added.l, .r = held[i].r + added.r} : added;
		mix[i] = before[i];
	}
	anythingHeldBack = true;
}

namespace {

// From the song mix's scale to the 16 bits on the wire. A track is taken before the master volume and compressor, so
// it runs hotter than the finished mix: it gets the output stage's gain (AUDIO_OUTPUT_GAIN_DOUBLINGS, 8 doublings),
// halved, and is trimmed by 12 dB first - ten of the Deluge's 1.2 dB volume steps - which leaves about 5 dB of headroom
// on a dense song. Same levels as the USB Audio 1.0 firmware, which were set by measurement
constexpr int32_t kTrackTrim = [] {
	int64_t multiplier = 1 << 24;
	for (int32_t step = 0; step < 10; step++) {
		multiplier = (multiplier * 57139) >> 16; // One 1.2 dB step down, over 2^16
	}
	return (int32_t)multiplier;
}();

[[gnu::always_inline]] inline int16_t toWire(int32_t value) {
	auto const trimmed = (int32_t)(((int64_t)value * kTrackTrim) >> 24);
	return (int16_t)(lshiftAndSaturate<8>(trimmed >> 1) >> 16);
}

} // namespace

void sendTracksToHost(size_t renderOffset, size_t numFrames, int32_t const* txPosition) {
	if (!multitrackSending()) {
		return;
	}
	uint32_t written = toHostWritten.load(std::memory_order_relaxed);
	numFrames = std::min<size_t>(numFrames, toHostSpace(written));
	for (size_t i = 0; i < numFrames; i++) {
		auto* frame =
		    reinterpret_cast<int16_t*>(&toHostRing[((written + i) & (kToHostRingFrames - 1)) * kMtToHostBytesPerFrame]);
		size_t const at = renderOffset + i;
		for (uint32_t channel = 0; channel < kMtChannelsToHost; channel++) {
			bool const has = (channelsWritten & (1u << channel)) && at < captureWindowFrames;
			frame[channel] = has ? toWire(mt->channels[channel][at]) : 0;
		}
	}
	commitToHost(written + numFrames, txPosition);
}

void compensate(std::span<StereoSample> mix, int32_t* reverbSend) {
	if (!multitrack) {
		return;
	}
	uint32_t const want = std::min<uint32_t>(compensationMs, kCompensationMaxMs) * USB_PAUDIO_SAMPLE_RATE / 1000;
	if (want != compensationFramesNow) {
		// The line is only kept up to date while it's in use, so it's cleared when compensation comes on rather than
		// play back what it held from before. While it's running, a new delay just reads from a different point
		if (compensationFramesNow == 0) {
			std::memset(compensation, 0, sizeof(CompensationBuffers));
		}
		compensationFramesNow = want;
	}
	if (compensationFramesNow == 0) {
		// Nothing's held back while compensation is off (see Song::renderAudio())
		return;
	}
	constexpr uint32_t kMask = kCompensationFrames - 1;
	uint32_t w = compensationWrite;
	for (size_t i = 0; i < mix.size(); i++) {
		uint32_t const from = (w - compensationFramesNow) & kMask;
		uint32_t const to = w & kMask;
		StereoSample const in = mix[i];
		mix[i] = compensation->mix[from];
		compensation->mix[to] = in;
		int32_t const send = reverbSend[i];
		reverbSend[i] = compensation->reverbSend[from];
		compensation->reverbSend[to] = send;
		w++;
	}
	compensationWrite = w;

	if (anythingHeldBack) {
		StereoSample const* held = mt->heldBack.data();
		for (size_t i = 0; i < mix.size() && i < captureWindowFrames; i++) {
			mix[i].l += held[i].l;
			mix[i].r += held[i].r;
		}
	}
}

} // namespace deluge::usb_audio
