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

#pragma once

#include <cstdint>

struct StereoSample;

namespace deluge::processing::engines {

/// Keeps the USB Audio Class isochronous endpoint answering the host.
///
/// An isochronous endpoint has no NAK: a pipe left idle simply does not respond, and the host reads that as a dead
/// endpoint and refuses to start the stream. So a transfer has to be in flight whenever the host has selected the
/// streaming interface, whether or not there is anything to say.
class USBAudioStream {
public:
	/// Submits the next packet if the previous one has completed. Safe to call when no host is streaming.
	///
	/// The scheduler's entry point: the diagnostics report from here, then the transport is serviced.
	static void routine();

	/// The transport half of routine(), without the SysEx diagnostics.
	///
	/// Called from the audio engine as well as the scheduler, because a card load hand-runs the audio task by id
	/// and no other registered task gets a pass until it finishes. This is the only servicing the stream sees
	/// across a kit load.
	static void service();

	/// Hands the finished main mix to the stream, from the output stage that already walks these samples.
	///
	/// The Deluge is the clock master here: a device-to-host isochronous stream carries whatever the source
	/// produced and the host adapts, so packets are sized from what this leaves in the ring rather than from a
	/// nominal rate. Costs nothing when no host is streaming.
	/// renderOffset is where these samples sit in the render window the stems were captured against, so the two
	/// halves of a frame come from the same instant. The output stage drains a window across several calls.
	static void feedMix(const StereoSample* mix, uint32_t numSamples, uint32_t renderOffset);

	/// One mono track per channel, across the whole width of the stream.
	///
	/// Mono rather than stereo pairs by default: it keeps every channel independently assignable and the far end
	/// recombines. A clip that wants true stereo selects a pair instead, which is a per-clip choice rather than a
	/// global mode. No pair is reserved for the main mix - decided 2026-08-22, revisited 2026-08-30 and reserved
	/// for stage D.
	static constexpr uint32_t kStemChannels = 8;
	static constexpr uint32_t kNoStem = 0xFFFFFFFFu;

	/// Whether any of the above is worth capturing. False unless a host holds the streaming interface, so a
	/// machine with nothing plugged in pays for none of it.
	static bool stemsWanted();

	/// Clears the stem accumulators. Once per render window, before any output has rendered into the mix.
	static void beginRender(uint32_t numSamples);

	/// Remembers the mix as it stands before one track renders into it.
	///
	/// One buffer, taken and consumed inside a single output's render and never outliving it. The window bound is
	/// enforced here rather than at the call site, so there is one place that can get it wrong.
	static void snapshotBeforeTrack(const int32_t* mixNow, uint32_t numSamples);

	/// Copies one track out to every destination its clip asked for, as the difference between the mix now and the
	/// snapshot above.
	///
	/// route is a UsbRoute mask. A mono channel gets the sum of the track's two sides; a pair gets left on the
	/// lower channel and right on the upper. Several clips may name one channel, in which case they add.
	static void captureStem(uint16_t route, const int32_t* mixNow, uint32_t numSamples);

	/// Puts the mix back as the snapshot found it, undoing one track's render. For a clip that has left the main
	/// mix: x - (x - y) = y holds in two's complement, so this is a copy rather than a subtraction.
	static void removeTrackFromMix(int32_t* mixNow, uint32_t numSamples);

	/// Reconciliation instruments. The scheduler reports the audio task's wall-clock duration, which absorbs every
	/// interrupt that fires inside it; these time the same code in processor cycles, which does not. Two readings
	/// of one thing, and the gap between them is the answer.
	static uint32_t costMark();
	static void costEngineRoutine(uint32_t start);
	static void costOutputLoop(uint32_t start);

	/// Sums one render window of returning audio into the song's mix.
	///
	/// Called immediately after every track has summed and before anything song-level, so the return gets the
	/// song's mod FX, delay, reverb send, master filters, volume and compressor exactly as the Deluge's own audio
	/// does - which is what makes a device on the other end of the cable behave like an insert rather than a
	/// separate instrument. Costs nothing when no host is sending.
	static void mixReturn(StereoSample* buffer, uint32_t numSamples);

	/// Level applied to the returning audio, 0-50, 1.2 dB a step. Zero is silence.
	///
	/// Unity sits at 40 rather than at the top, so the control can lift a quiet source by up to 12 dB as well as
	/// hold back a loud one - a control whose maximum is unity can only ever attenuate, which is what it did
	/// before 2026-09-08. At unity a device that hands back what it was given is nominally level-transparent;
	/// nominally, because the measured unity round trip is B3.5 and this is how a rig disagrees until then.
	static constexpr uint32_t kReturnLevelMax = 50;
	static constexpr uint32_t kReturnLevelUnity = 40;
	static constexpr uint32_t kReturnLevelDefault = kReturnLevelUnity;
	static void setReturnLevel(uint32_t level);
	static uint32_t getReturnLevel();

	/// How many stereo pairs the return carries. Two at the four channels the cable fits alongside eight going
	/// out; the widths either side of that are a broken one and one the host refuses.
	static uint32_t numReturnPairs();

	/// Which pair is summed into the song's own mix, 1-based, or zero for none.
	///
	/// One pair at a time - decided 2026-09-08, so the other stays available to a track rather than arriving in
	/// the song twice. A pair a track has taken is not summed here for the same reason.
	static void setMasterReturnPair(uint32_t pair);
	static uint32_t getMasterReturnPair();

	/// Adds one returning pair into a track's own render, and takes that pair off the master for this window.
	///
	/// pair is 1-based. amplitude is the track's volume across the window, on the same scale a monitored line
	/// input uses, applied per sample so a moving fader does not step. Returns whether anything was added; false
	/// means nothing is arriving, and the pair is claimed either way so a momentary gap does not bounce the audio
	/// back to the master and away again.
	static bool readReturnPair(uint32_t pair, StereoSample* buffer, uint32_t numSamples, int32_t amplitudeStart,
	                           int32_t amplitudeEnd);

	/// The same, for one arriving channel on its own. channel is 1-based and lands equally on both sides, the way
	/// a single line input does. It takes its whole pair off the master: the master sums pairs, and leaving it the
	/// other side would put that channel in the song twice.
	static bool readReturnChannel(uint32_t channel, StereoSample* buffer, uint32_t numSamples, int32_t amplitudeStart,
	                              int32_t amplitudeEnd);

	/// How many channels the return carries.
	static uint32_t numReturnChannels();

	/// Largest magnitude any stem has reached since the last read, at capture scale and therefore before the
	/// width reduction that would clip it. The instrument the trim is set from.
	static int32_t readAndClearStemPeak();
};

} // namespace deluge::processing::engines

extern "C" {
/// Task-scheduler entry point.
void usbAudioStreamRoutine();
}
