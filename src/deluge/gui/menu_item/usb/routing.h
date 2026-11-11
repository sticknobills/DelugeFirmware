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

#include "drivers/usb/usb_audio.h"
#include "gui/menu_item/submenu.h"
#include "gui/menu_item/toggle.h"
#include "model/output.h"
#include "model/song/song.h"
#include "model/usb_route.h"

namespace deluge::gui::menu_item::usb {

/// One of the host's channels, or pairs of them, for the current track - multitrack mode's USB routing
class RouteToggle final : public Toggle {
public:
	RouteToggle(l10n::String newName, l10n::String title, uint16_t newBit) : Toggle(newName, title), bit(newBit) {}

	void readCurrentValue() override {
		Output* output = getCurrentOutput();
		this->setValue(output != nullptr && (output->usbRouting & bit) != 0);
	}

	void writeCurrentValue() override {
		Output* output = getCurrentOutput();
		if (output == nullptr) {
			return;
		}
		if (this->getValue()) {
			output->usbRouting |= bit;
		}
		else {
			output->usbRouting &= static_cast<uint16_t>(~bit);
		}
	}

private:
	uint16_t const bit;
};

/// Whether the current track stays in the Deluge's own outputs. Off leaves USB as its only way out
class MainToggle final : public Toggle {
public:
	using Toggle::Toggle;

	void readCurrentValue() override {
		Output* output = getCurrentOutput();
		this->setValue(output == nullptr || output->isInMainMix());
	}

	void writeCurrentValue() override {
		Output* output = getCurrentOutput();
		if (output == nullptr) {
			return;
		}
		if (this->getValue()) {
			output->usbRouting |= UsbRoute::MAIN;
		}
		else {
			output->usbRouting &= static_cast<uint16_t>(~UsbRoute::MAIN);
		}
	}
};

/// Multitrack mode's per-track USB routing. Shown only in multitrack mode, and only for tracks that make audio: not
/// MIDI or CV. A kit is routed as a whole, from its menu with AFFECT ENTIRE on
class RoutingSubmenu final : public Submenu {
public:
	using Submenu::Submenu;

	bool isRelevant(ModControllableAudio* modControllable, int32_t whichThing) const override {
		OutputType const type = getCurrentOutputType();
		return usbAudioMultitrackActive() && type != OutputType::MIDI_OUT && type != OutputType::CV;
	}
};

} // namespace deluge::gui::menu_item::usb
