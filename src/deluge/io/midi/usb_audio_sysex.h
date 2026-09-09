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

class MIDICable;
class JsonDeserializer;

/// Lets a host read and set the USB audio channel map over the JSON SysEx channel.
///
/// Eight audio channels leave the Deluge with nothing on the wire to say what is on them, so a host sees eight
/// anonymous inputs and the user has to walk to the instrument and read a menu per track to find out. This is the
/// conversation that answers it: what is on each channel, what tracks exist, and requests to change either.
///
/// Host-neutral by construction. Every message is about channels, tracks and widths; nothing here names a
/// particular product on the other end, and a laptop, a tablet or an outboard processor all speak the same words.
namespace deluge::io::midi::usb_audio {

/// Bumped when a message changes shape. Both ends announce it and the lower governs; unknown keys are ignored
/// rather than treated as errors, because the two ends will not update together.
constexpr uint32_t kVocabularyVersion = 1;

/// Handles one "usbAudio" request. The caller has consumed the tag name and left the reader on its value.
void handleRequest(MIDICable& cable, JsonDeserializer& reader);

/// Pushes state to a subscribed host: the map when it changes, the song when it changes, the held track when it
/// changes, and a keep-alive otherwise. Returns immediately when nobody has subscribed.
void routine();

/// What the connected host called itself, for the read-only line in the USB settings menu. Empty when nothing has
/// spoken recently - which is how a working cable with a dead control channel is told from a working one.
[[nodiscard]] const char* hostName();
[[nodiscard]] uint32_t hostVocabularyVersion();

} // namespace deluge::io::midi::usb_audio
