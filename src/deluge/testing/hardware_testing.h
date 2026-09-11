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

void ramTestUart();
void ramTestLED(bool stuffAlreadySetUp = false);
void autoPilotStuff();

/// Diagnostic: 13 tones at a constant safe level, to measure the CV output's
/// frequency response. Blocking, ~10s, non-destructive.
void cvFrequencyResponse();

/// Diagnostic: steps gate 1 (P2_7) through a 50% square wave at eleven frequencies
/// from 176 kHz to 5.6 MHz, to find where the gate driver stops keeping up. Read
/// as a DC average on a multimeter -- the frequency at which the average sags is
/// the driver's speed limit, and that decides whether the gate sockets can carry
/// I2S.
///
/// One call advances one step and returns immediately; SSI3 keeps clocking in
/// hardware in the meantime, so the main loop is never stalled and the display
/// still updates. Call again to move on, and once past the last step everything
/// is shut down and the pins go back to the gate engine.
///
/// The first two steps park the socket statically high then low, using GPIO only.
/// They involve SSI3 not at all, so they isolate "the chord and the pin work"
/// from "SSI3 is configured correctly".
///
/// Non-destructive. Gates 2 and 4 are untouched throughout.
///
/// Currently not bound to a chord -- SHIFT + CV runs i2sAdvance instead. The
/// binary that used it is kept as resources/deluge-GATE-SWEEP.bin.
void gateSpeedSweepAdvance();

/// Diagnostic: sends real I2S out of gates 1, 2 and 3 for an external PCM5102 DAC.
/// One call advances one step and returns; the DMA loops a fixed buffer forever,
/// so nothing needs refilling and the CPU is not involved once it is running.
///
/// Steps: PAT1 and PAT2 are DC patterns designed to be verified with a multimeter
/// alone, before any DAC exists -- the data and word select lines both become
/// square waves at the frame rate, which reads as half rail. TON1 and TON2 play a
/// tone on the left channel only, for once a DAC is connected. Then OFF restores
/// all three pins to the gate engine.
///
/// PAT1/TON1 run at 44.1 kHz (1.4112 MHz bit clock), PAT2/TON2 at 22.05 kHz.
/// Gate 4 is untouched throughout.
void i2sAdvance();

/// Diagnostic: the Gate PWM falsification test. One call advances one step of
/// fixed pulse patterns on gate 2, each read as a DC average on a multimeter, then
/// OFF restores the pins. See the table in hardware_testing.cpp for what each
/// label means.
void pulseTestAdvance();

/// False on OLED models, where the DAC shares its SPI channel with the display and
/// this cannot work. Everything that acts on a Clip's routing checks this first, so
/// on an OLED model every Clip behaves as if it were routed to MAIN alone.
bool cvOutputsAvailable();
/// True while the CV sockets are being streamed to.
bool cvStreamIsRunning();
/// True while a playing Clip is routed with STEREO SPLIT on.
///
/// INTENDED BEHAVIOUR, not an oversight. STEREO SPLIT only has anything to split when
/// the engine is rendering in stereo, which it does only with headphones or the right
/// main output connected, or while internally recording. Rendering in mono is not just a missing pan -- stereo
/// samples have their channels combined and every voice costs about half as much. So
/// this must not be used to force stereo on: that would make a CV routing choice
/// change the main mix's level and its CPU cost, which is far worse than the split
/// quietly falling back to the mono sum on both sockets.
bool cvStreamNeedsStereo();
/// Hands one Clip's isolated audio to the sockets its sends name. A send above zero is what
/// "routed to that socket" means. `sourceId` identifies the Clip, so the correction filter
/// can be reset when a socket's set of sources changes.
///
/// The Clip is isolated here rather than by the caller: `post` is the interleaved mix after
/// the Clip rendered, `pre` the snapshot taken before it did, and the difference is taken
/// per sample inside the accumulate loop that was going to read them anyway.
void cvStreamCapture(uint32_t sourceId, const int32_t* post, const int32_t* pre, uint32_t numSamples,
                     const int32_t* sendGain, int32_t* lastSendGain);
/// Snapshot space for the mix as it stood before a Clip rendered.
int32_t* cvStreamCaptureScratch();
/// Converts everything captured this window and queues it for output.
void cvStreamPump(uint32_t numSamples);
/// Called once when the render loop has finished, so the full set of Clips feeding
/// each socket is known. Resets the correction filter where that set changed, and
/// starts or stops the stream to match -- nothing else has to.
void cvStreamRenderComplete();

/// Stereo split, GLOBAL rather than per Clip. It describes how the two cables are plugged
/// in -- one stereo destination, or two mono ones -- which is a property of the rig and not
/// of any clip. Per-Clip it permitted a state where one clip treated the pair as stereo
/// while another treated them as two mono outs, which is never what anyone meant.
bool cvGetStereoSplit();
void cvSetStereoSplit(bool on);

/// AUX MASTER, one per socket. Set once per render window from the song's two master params,
/// which is where the level now lives -- the flash bytes behind cvGetOutputLevel are gone.
///
/// Displayed 0-50 like every other param on the machine, and 0 is a true mute rather than the
/// bottom of the taper: "quiet enough" is not silence, and a mute is what an off position is
/// for. 50 is x256; each step below it is a fixed 1.2 dB, so the steps sound evenly spaced the
/// whole way down. The default 40 is the x64 that was tuned by ear on 2026-08-04, so a song
/// that has never touched this comes up exactly where the old flash default did.
void cvSetMasterFromParams(int32_t cv1Value, int32_t cv2Value);
/// Which socket the AUX MASTER gold knob is currently editing, 0 or 1. Only meaningful when
/// split is off; with split on there is one master and the knob always drives CV1's param.
/// Deliberately not saved anywhere -- it is where a knob is pointing, not a setting.
extern uint8_t cvMasterKnobSocket;
/// Menu display value, 0-50, for a raw master param value. The menu and the gold knob edit one
/// param between them, so this is only a rendering of it -- there is no second copy to drift.
int32_t cvMasterParamToDisplay(int32_t paramValue);
int32_t cvMasterDisplayToParam(int32_t display);
constexpr int32_t kCvMasterDisplayMax = 50;

/// Per-Clip send amount, Q16. Attenuation only, so this is the ceiling.
constexpr int32_t kCvSendGainUnity = 65536;

/// Convert a send param value into the Q16 gain the capture path wants. Squared, which is
/// how the Deluge treats every other volume-ish param -- a linear send feels dead until the
/// top of its travel.
int32_t cvSendParamToGain(int32_t paramValue);
