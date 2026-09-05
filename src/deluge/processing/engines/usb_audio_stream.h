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

	/// Trim applied to every stem on the way to 16 bit, 0-50, 1.2 dB a step.
	///
	/// A stem is captured before the master compressor, so it runs about three times hotter than the mix and
	/// clips where the mix does not. This is the one control that fixes that, and it is per-machine rather than
	/// per-song because it describes the gain staging of whatever is on the other end of the cable.
	static constexpr uint32_t kTrimMax = 50;
	/// Set from measurement, 2026-08-30: at unity the reference song's stems peak 1.8x past full scale, and at 36
	/// they peaked at 10,498 of 32,767 - a third of the range in use and about 10 dB thrown away. 40 lands them
	/// near 18,000, which keeps roughly 5 dB for a song hotter than that one.
	static constexpr uint32_t kTrimDefault = 40;
	static void setTrim(uint32_t trim);
	static uint32_t getTrim();

	/// Sums one render window of returning audio into the song's mix.
	///
	/// Called immediately after every track has summed and before anything song-level, so the return gets the
	/// song's mod FX, delay, reverb send, master filters, volume and compressor exactly as the Deluge's own audio
	/// does - which is what makes a device on the other end of the cable behave like an insert rather than a
	/// separate instrument. Costs nothing when no host is sending.
	static void mixReturn(StereoSample* buffer, uint32_t numSamples);

	/// DIAGNOSTIC. One frame of the return, per sample that actually leaves the machine. Returns false and zeroes
	/// its outputs when there is nothing to give.
	static bool takeReturnFrame(int32_t* left, int32_t* right);

	/// Level applied to the returning audio, 0-50, 1.2 dB a step, and whether it is summed in at all.
	///
	/// The default is unity against the outgoing trim's own inverse, so a device that returns what it was given
	/// is nominally level-transparent. Nominally: the measured unity round trip is B3.5, and this is how a rig
	/// disagrees until then.
	static constexpr uint32_t kReturnLevelMax = 50;
	static constexpr uint32_t kReturnLevelDefault = 50;
	static void setReturnLevel(uint32_t level);
	static uint32_t getReturnLevel();
	static void setReturnEnabled(bool enabled);
	static bool getReturnEnabled();

	/// Largest magnitude any stem has reached since the last read, at capture scale and therefore before the
	/// width reduction that would clip it. The instrument the trim is set from.
	static int32_t readAndClearStemPeak();
};

} // namespace deluge::processing::engines

extern "C" {
/// Task-scheduler entry point.
void usbAudioStreamRoutine();
}
