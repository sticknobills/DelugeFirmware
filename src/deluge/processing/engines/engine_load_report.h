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

#include <cstddef>
#include <cstdint>

namespace deluge::processing::engines {

/// DIAGNOSTIC. Reports what the audio engine costs, over the SysEx debug channel.
///
/// Nothing here changes behaviour: every figure it prints is one the engine already computes for its own
/// culling decisions. It exists so a build carrying extra work can be compared against a stock control on
/// numbers rather than on how the two sound, and it is written to compile into a stock build unchanged.
///
/// The report also carries the firmware version string, which is the only thing on the debug channel that
/// identifies which binary is running. Field names identify a family of builds, not a build.
class EngineLoadReport {
public:
	/// Takes one render's cost, at the point the engine has already worked it out for setDireness().
	///
	/// `windowSamples` is how many samples that render was asked for. `dspTimeSamples` is the corrected figure
	/// direness would be decided from; `dspTimeRaw` is the scheduler's own per-call average before that
	/// correction, and `rendersPerCallX100` is what separates them. All three are reported so the correction can
	/// be checked rather than trusted.
	///
	/// `effectiveSamples` is the number the direness and culling tests are **actually** made against - the same
	/// figure after the second render of a call has its predecessor's window subtracted. It is not equal to
	/// either of the other two on every render, and until 2026-09-05 it was reported nowhere, so every published
	/// mean described a number no decision was taken on.
	static void recordRender(int32_t dspTimeSamples, int32_t dspTimeRaw, size_t windowSamples, int32_t direness,
	                         uint32_t rendersPerCallX100, int32_t effectiveSamples);

	/// DIAGNOSTIC. Takes which way cullVoices() went on one render, so the decision is recorded and not only its
	/// input.
	///
	/// Reported as counts per interval on the EH line. `gated` is whether the voice-count gate let the main path
	/// run at all; the rest are mutually exclusive outcomes within whichever path ran. A build can then be
	/// compared on which branch moved rather than on a total that several branches feed.
	static void recordCullDecision(bool gated, bool hardCull, bool softEligible, bool softCulled, bool belowMinCull);

	/// DIAGNOSTIC. Takes one allowedToStartVoice() answer.
	///
	/// Under maximum direness the engine admits one voice per render, so a voice the song asked for can be
	/// refused rather than culled - which no existing counter distinguishes from a voice that never played.
	static void recordVoiceStart(bool allowed);

	/// Takes voices the engine gave up on because it ran out of time, by the route it used.
	static void recordCull(uint32_t forceReleased, uint32_t terminated, uint32_t killed);

	/// Called at the top of the audio routine, and measures the gap since the previous one itself.
	///
	/// The codec's buffer holds one window of audio, so a gap wider than that window is audio the codec did not
	/// receive and replayed instead - which is what a click is. Upstream already tests this condition and prints
	/// a warning; this counts it, so a build can be compared against a control on a number rather than by ear
	/// against a song that clicks on its own account. The clock is read here rather than passed in, so the caller
	/// needs nothing but this header.
	static void recordRoutineEntry();

	/// Emits the report and clears the interval's counters. Scheduler entry point.
	static void routine();
};

} // namespace deluge::processing::engines

extern "C" {
/// Task-scheduler entry point.
void engineLoadReportRoutine();
}
