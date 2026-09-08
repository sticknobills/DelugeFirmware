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

#include "gui/context_menu/context_menu.h"
#include "processing/audio_output.h"

namespace deluge::gui::context_menu {

/// Which arriving USB channel or pair this track takes as its input.
///
/// A level below the input selector rather than six more entries in it: the four singles and two pairs would be
/// most of that list, and the four inputs the machine has on jacks would be buried under the ones it has on one
/// cable.
class UsbInputSelector final : public ContextMenu {
	enum class Value;

public:
	UsbInputSelector() = default;
	void selectEncoderAction(int8_t offset) override;
	bool setupAndCheckAvailability() override;
	bool canSeeViewUnderneath() override { return true; }
	AudioOutput* audioOutput;

	/// The channel this menu is entered on when the track has never had a USB input.
	static AudioInputChannel defaultChannel();

	char const* getTitle() override;
	std::span<const char*> getOptions() override;
};

extern UsbInputSelector usbInputSelector;
} // namespace deluge::gui::context_menu
