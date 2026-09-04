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

/// Which USB audio channels a track is copied out to, plus whether it stays in the machine's own main mix.
///
/// One bitmask rather than a channel number: a track may feed several destinations, and several tracks may feed one
/// channel. MAIN is here rather than beside it because leaving the main mix is the same kind of routing decision -
/// and when this merges with the AUX sends line there must be exactly one MAIN bit, not one per feature.
namespace UsbRoute {
enum : uint16_t {
	MAIN = 1 << 0, ///< Stays in the Deluge's own outputs. Set by default; clearing it makes USB the only way out.
	CH1 = 1 << 1,
	CH2 = 1 << 2,
	CH3 = 1 << 3,
	CH4 = 1 << 4,
	CH5 = 1 << 5,
	CH6 = 1 << 6,
	CH7 = 1 << 7,
	CH8 = 1 << 8,
	/// A pair is stereo - left to the lower channel, right to the upper. A single channel gets the mono sum, so
	/// selecting 1 and 2 separately is not the same thing as selecting 1-2, and both may be on at once.
	PAIR12 = 1 << 9,
	PAIR34 = 1 << 10,
	PAIR56 = 1 << 11,
	PAIR78 = 1 << 12,

	FIRST_MONO = CH1,
	MONO_MASK = CH1 | CH2 | CH3 | CH4 | CH5 | CH6 | CH7 | CH8,
	PAIR_MASK = PAIR12 | PAIR34 | PAIR56 | PAIR78,
	ANY_USB = MONO_MASK | PAIR_MASK,
	ALL = MAIN | ANY_USB,
	DEFAULT = MAIN, ///< A song that has never been set up puts nothing on the cable.
};
} // namespace UsbRoute
