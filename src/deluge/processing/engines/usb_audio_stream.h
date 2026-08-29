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
	/// Mono rather than stereo pairs: it keeps every channel independently assignable and the far end recombines,
	/// which is worth more than a track's own panning over a cable that carries eight of them at most. No pair is
	/// reserved for the main mix - that was decided 2026-08-22 and the mix is a source like any other once there
	/// is an interface to assign one.
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

	/// Adds one track to a stem channel, as the difference between the mix now and the snapshot above. Summed to
	/// mono, at the scale the main mix reaches the host on.
	static void captureStem(uint32_t channel, const int32_t* mixNow, uint32_t numSamples);

	/// Names the track a stem channel is carrying, for the map report. Called from the render loop with the
	/// output's own display name.
	static void noteStemTrack(uint32_t channel, const char* name);

	/// Which stem channel a track occupies, from its position in the song's output list.
	///
	/// Provisional: the first six tracks in song order, so the path can be proven on hardware before an assignment
	/// interface exists. What replaces it is stage D, and T8 is its constraint.
	static uint32_t stemChannelForOutput(uint32_t outputIndex);
};

} // namespace deluge::processing::engines

extern "C" {
/// Task-scheduler entry point.
void usbAudioStreamRoutine();
}
