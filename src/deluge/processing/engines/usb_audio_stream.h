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
	static void feedMix(const StereoSample* mix, uint32_t numSamples);
};

} // namespace deluge::processing::engines

extern "C" {
/// Task-scheduler entry point.
void usbAudioStreamRoutine();
}
