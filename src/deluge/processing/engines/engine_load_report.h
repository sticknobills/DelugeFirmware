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
	/// `dspTimeSamples` is the scheduler's running average for the audio task expressed in samples, which is
	/// the figure culling is decided against; `windowSamples` is how many samples that render was asked for.
	static void recordRender(int32_t dspTimeSamples, size_t windowSamples, int32_t direness);

	/// Takes voices the engine gave up on because it ran out of time, by the route it used.
	static void recordCull(uint32_t forceReleased, uint32_t terminated, uint32_t killed);

	/// Emits the report and clears the interval's counters. Scheduler entry point.
	static void routine();
};

} // namespace deluge::processing::engines

extern "C" {
/// Task-scheduler entry point.
void engineLoadReportRoutine();
}
