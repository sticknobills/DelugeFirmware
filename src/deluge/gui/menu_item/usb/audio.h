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
#include "gui/menu_item/integer.h"
#include "gui/menu_item/selection.h"
#include "gui/menu_item/toggle.h"
#include "hid/display/display.h"
#include "storage/flash_storage.h"

namespace deluge::gui::menu_item::usb {

/// What the Deluge offers a computer over USB: MIDI only, MIDI and stereo audio, or MIDI and multitrack audio. The host
/// only reads our descriptors when we connect, so this takes effect the next time USB starts up - at power-on
class Mode final : public Selection {
public:
	using Selection::Selection;
	void readCurrentValue() override { this->setValue(FlashStorage::usbAudioMode); }
	void writeCurrentValue() override {
		FlashStorage::usbAudioMode = static_cast<uint8_t>(this->getValue());
		bool const multitrackWanted = (FlashStorage::usbAudioMode == USB_PAUDIO_MODE_MULTITRACK);
		if ((FlashStorage::usbAudioMode != USB_PAUDIO_MODE_OFF) != usbAudioActive()
		    || multitrackWanted != usbAudioMultitrackActive()) {
			display->displayPopup(l10n::get(l10n::String::STRING_FOR_RESTART_TO_APPLY));
		}
	}
	deluge::vector<std::string_view> getOptions(OptType optType) override {
		(void)optType;
		using enum l10n::String;
		// In USB_PAUDIO_MODE_ order
		return {l10n::getView(STRING_FOR_OFF), l10n::getView(STRING_FOR_STEREO),
		        l10n::getView(STRING_FOR_USB_MULTITRACK)};
	}
};

/// Delay compensation, in ms: holds the Deluge's own audio back so a track sent out through an external processor and
/// returned lands in time with the rest. Multitrack mode only
class Compensation final : public Integer {
public:
	using Integer::Integer;
	[[nodiscard]] int32_t getMinValue() const override { return 0; }
	[[nodiscard]] int32_t getMaxValue() const override { return usb_audio::kCompensationMaxMs; }
	bool isRelevant(ModControllableAudio* modControllable, int32_t whichThing) const override {
		return usbAudioMultitrackActive();
	}
	void readCurrentValue() override { this->setValue(usb_audio::compensationMs); }
	void writeCurrentValue() override { usb_audio::compensationMs = static_cast<uint8_t>(this->getValue()); }
};

/// Whether audio from the host is mixed straight into the Deluge's outputs
class DirectMonitoring final : public Toggle {
public:
	using Toggle::Toggle;
	void readCurrentValue() override { this->setValue(usb_audio::directMonitoring); }
	void writeCurrentValue() override { usb_audio::directMonitoring = this->getValue(); }
};

/// What the host receives from the Deluge: its main mix, or its line input. Not in multitrack mode, where it receives
/// the tracks routed to it
class OutputSource final : public Selection {
public:
	using Selection::Selection;
	bool isRelevant(ModControllableAudio* modControllable, int32_t whichThing) const override {
		return !usbAudioMultitrackActive();
	}
	void readCurrentValue() override { this->setValue(usb_audio::outputSource); }
	void writeCurrentValue() override { usb_audio::outputSource = this->getValue<usb_audio::OutputSource>(); }
	deluge::vector<std::string_view> getOptions(OptType optType) override {
		(void)optType;
		using enum l10n::String;
		return {l10n::getView(STRING_FOR_MAIN_MIX), l10n::getView(STRING_FOR_LINE_IN)};
	}
};

} // namespace deluge::gui::menu_item::usb
