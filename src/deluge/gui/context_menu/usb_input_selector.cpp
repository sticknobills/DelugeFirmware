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

#include "gui/context_menu/usb_input_selector.h"
#include "definitions_cxx.hpp"
#include "gui/l10n/l10n.h"
#include "processing/audio_output.h"

extern AudioInputChannel defaultAudioOutputInputChannel;

namespace deluge::gui::context_menu {

// In the order the four singles then the two pairs appear in AudioInputChannel, so the option index is the enum
// offset and neither list can be reordered without the other.
enum class UsbInputSelector::Value {
	CHANNEL_1,
	CHANNEL_2,
	CHANNEL_3,
	CHANNEL_4,
	PAIR_1_2,
	PAIR_3_4,
};
constexpr size_t kNumValues = 6;

UsbInputSelector usbInputSelector{};

AudioInputChannel UsbInputSelector::defaultChannel() {
	return AudioInputChannel::USB_1;
}

char const* UsbInputSelector::getTitle() {
	using enum l10n::String;
	return l10n::get(STRING_FOR_USB_INPUT);
}

std::span<const char*> UsbInputSelector::getOptions() {
	using enum l10n::String;
	static const char* options[] = {
	    l10n::get(STRING_FOR_USB_CHANNEL_1), l10n::get(STRING_FOR_USB_CHANNEL_2), l10n::get(STRING_FOR_USB_CHANNEL_3),
	    l10n::get(STRING_FOR_USB_CHANNEL_4), l10n::get(STRING_FOR_USB_PAIR_12),   l10n::get(STRING_FOR_USB_PAIR_34),
	};
	return {options, kNumValues};
}

bool UsbInputSelector::setupAndCheckAvailability() {
	AudioInputChannel channel = audioOutput->inputChannel;
	if (!isUsbReturnInput(channel)) {
		channel = defaultChannel();
	}
	currentOption = static_cast<int32_t>(channel) - static_cast<int32_t>(AudioInputChannel::USB_1);
	scrollPos = currentOption;
	return true;
}

void UsbInputSelector::selectEncoderAction(int8_t offset) {
	if (currentUIMode != 0u) {
		return;
	}

	ContextMenu::selectEncoderAction(offset);

	// Applied as the encoder turns, like the selector above this one, so the choice is made by ear rather than by
	// reading the screen and committing.
	audioOutput->inputChannel =
	    static_cast<AudioInputChannel>(static_cast<int32_t>(AudioInputChannel::USB_1) + currentOption);
	defaultAudioOutputInputChannel = audioOutput->inputChannel;

	if (display->haveOLED()) {
		renderUIsForOled();
	}
}

} // namespace deluge::gui::context_menu
