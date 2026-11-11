/*
 * Copyright © 2015-2023 Synthstrom Audible Limited
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

namespace deluge::processing::engines {

/// Assignable audio outputs on the two CV sockets.
///
/// The DAC feeding the CV sockets is a real 16-bit converter on a fast SPI bus, and its
/// chip-select sits on P6_1 -- the SPI block's own SSL00 pin, already configured as such at
/// boot. That lets the transfer engine feed it continuously with no processor involvement,
/// so the sockets can carry audio rather than only the occasional note voltage.

/// False on OLED models, where the DAC shares its SPI channel with the display and
/// this cannot work. Everything that acts on a Clip's output checks this first, so
/// on an OLED model every Clip goes to MAIN.
bool cvOutputsAvailable();

/// True while the CV sockets are being streamed to.
bool cvStreamIsRunning();

/// Where one track goes. Each track has exactly one output, chosen per track and saved on the
/// Clip, so it survives a preset change or a change of track type. A CV output takes the track
/// out of the main mix entirely, reverb send included.
enum class CvOutput : uint8_t {
	MAIN = 0,   ///< the main outputs, as in stock firmware
	CV1 = 1,    ///< CV1 alone, as the track's mono sum
	CV2 = 2,    ///< CV2 alone, as the track's mono sum
	STEREO = 3, ///< CV1 left and CV2 right, placed by the track's pan
};
constexpr uint8_t kNumCvOutputs = 4;

/// True while a playing track goes to the pair in stereo. The engine renders in stereo while
/// this holds, the same as it does for headphones or the right main output: in a mono render
/// pan is skipped, so a stereo output would carry the mono sum on both sockets. CV1 and CV2
/// alone do not set it, because their mono sum is the same either way. With only the left main
/// output connected, that output carries the left channel alone while this holds, as it does in
/// stock firmware while recording internally.
bool cvStreamNeedsStereo();

/// Hands one Clip's isolated audio to its output: left to CV1 and right to CV2 for STEREO, or
/// the mono sum to one socket. `sourceId` identifies the Clip, so the correction filter can be
/// reset when a socket's set of sources changes. `lastGain` is owned by the Clip and ramps the
/// track in over its first window, so a track switched to CV does not click on.
///
/// The Clip is isolated here rather than by the caller: `post` is the interleaved mix after
/// the Clip rendered, `pre` the snapshot taken before it did, and the difference is taken
/// per sample inside the accumulate loop that was going to read them anyway.
void cvStreamCapture(uint32_t sourceId, const int32_t* post, const int32_t* pre, uint32_t numSamples, int32_t* lastGain,
                     CvOutput output);

/// Somewhere to render a track's reverb send that nothing reads. A track sent to CV renders its
/// reverb send here, so its reverb tail cannot reach the main outputs. Holds
/// kCvMaxWindowSamples samples, the longest window a track can be sent to CV in.
int32_t* cvStreamReverbDiscard();
constexpr uint32_t kCvMaxWindowSamples = 256;

/// Whether the global stereo split of AUX Sends 1.0.x was switched off on this unit. Read from
/// its old settings byte only so that songs saved by 1.0.x load onto the right output.
void cvSetLegacySplitOff(bool off);
bool cvGetLegacySplitOff();

/// Snapshot space for the mix as it stood before a Clip rendered.
int32_t* cvStreamCaptureScratch();

/// Converts everything captured this window and queues it for output.
void cvStreamPump(uint32_t numSamples);

/// Called once when the render loop has finished, so the full set of Clips feeding
/// each socket is known. Resets the correction filter where that set changed, and
/// starts or stops the stream to match -- nothing else has to.
void cvStreamRenderComplete();

/// Unity gain for the capture's ramp, Q16.
constexpr int32_t kCvGainUnity = 65536;

} // namespace deluge::processing::engines
