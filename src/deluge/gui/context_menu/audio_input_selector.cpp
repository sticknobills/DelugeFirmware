/*
 * Copyright © 2019-2023 Synthstrom Audible Limited
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

#include "gui/context_menu/audio_input_selector.h"
#include "definitions_cxx.hpp"
#include "gui/context_menu/usb_input_selector.h"
#include "gui/l10n/l10n.h"
#include "gui/ui/root_ui.h"
#include "gui/views/session_view.h"
#include "model/song/song.h"
#include "processing/audio_output.h"

extern AudioInputChannel defaultAudioOutputInputChannel;

namespace deluge::gui::context_menu {

enum class AudioInputSelector::Value {
	OFF,
	LEFT,
	RIGHT,
	STEREO,
	BALANCED,
	MASTER,
	OUTPUT,
	TRACK,
	USB,
};
constexpr size_t kNumValues = 9;

AudioInputSelector audioInputSelector{};

namespace {
// A saved source pointer can become stale if its instrument leaves the active song list, e.g. by being hibernated.
Output* getRecordableOutputInSong(AudioOutput* audioOutput, Output* selectedOutput) {
	if (!audioOutput->canRecordFrom(selectedOutput)) {
		return nullptr;
	}

	for (Output* output = currentSong->firstOutput; output; output = output->next) {
		if (output == selectedOutput) {
			return selectedOutput;
		}
	}

	return nullptr;
}

// Used when entering Track mode without a valid previous source.
Output* getFirstRecordableOutput(AudioOutput* audioOutput) {
	for (Output* output = currentSong->firstOutput; output; output = output->next) {
		if (audioOutput->canRecordFrom(output)) {
			return output;
		}
	}

	return nullptr;
}
} // namespace

char const* AudioInputSelector::getTitle() {
	using enum l10n::String;
	return l10n::get(STRING_FOR_AUDIO_SOURCE);
}

std::span<const char*> AudioInputSelector::getOptions() {
	using enum l10n::String;
	static const char* options[] = {
	    l10n::get(STRING_FOR_DISABLED),     l10n::get(STRING_FOR_LEFT_INPUT),     l10n::get(STRING_FOR_RIGHT_INPUT),
	    l10n::get(STRING_FOR_STEREO_INPUT), l10n::get(STRING_FOR_BALANCED_INPUT), l10n::get(STRING_FOR_MIX_PRE_FX),
	    l10n::get(STRING_FOR_MIX_POST_FX),  l10n::get(STRING_FOR_TRACK),          l10n::get(STRING_FOR_USB_INPUT),
	};
	return {options, kNumValues};
}

bool AudioInputSelector::setupAndCheckAvailability() {
	Value valueOption = Value::OFF;

	switch (audioOutput->inputChannel) {
	case AudioInputChannel::LEFT:
		valueOption = Value::LEFT;
		break;

	case AudioInputChannel::RIGHT:
		valueOption = Value::RIGHT;
		break;

	case AudioInputChannel::STEREO:
		valueOption = Value::STEREO;
		break;

	case AudioInputChannel::BALANCED:
		valueOption = Value::BALANCED;
		break;

	case AudioInputChannel::MIX:
		valueOption = Value::MASTER;
		break;

	case AudioInputChannel::OUTPUT:
		valueOption = Value::OUTPUT;
		break;

	case AudioInputChannel::SPECIFIC_OUTPUT:
		valueOption = Value::TRACK;
		break;

	case AudioInputChannel::USB_1:
	case AudioInputChannel::USB_2:
	case AudioInputChannel::USB_3:
	case AudioInputChannel::USB_4:
	case AudioInputChannel::USB_1_2:
	case AudioInputChannel::USB_3_4:
		valueOption = Value::USB;
		break;

	default:
		valueOption = Value::OFF;
	}

	currentOption = static_cast<int32_t>(valueOption);

	scrollPos = currentOption;
	return true;
}

bool AudioInputSelector::getGreyoutColsAndRows(uint32_t* cols, uint32_t* rows) {
	*rows = getRootUI()->getGreyedOutRowsNotRepresentingOutput(audioOutput);
	return true;
}

void AudioInputSelector::selectEncoderAction(int8_t offset) {
	if (currentUIMode != 0u) {
		return;
	}

	ContextMenu::selectEncoderAction(offset);

	auto valueOption = static_cast<Value>(currentOption);
	if (display->haveOLED() && (valueOption == Value::TRACK || valueOption == Value::USB)) {
		// Keep Track on the first visible row so the selected track name has room below it.
		scrollPos = currentOption;
	}

	// When switching away from SPECIFIC_OUTPUT, clear the recording-from state
	// so the previously-selected track is no longer silently muted
	if (audioOutput->inputChannel == AudioInputChannel::SPECIFIC_OUTPUT && valueOption != Value::TRACK) {
		audioOutput->clearRecordingFrom();
	}

	switch (valueOption) {

	case Value::LEFT:
		audioOutput->inputChannel = AudioInputChannel::LEFT;
		break;

	case Value::RIGHT:
		audioOutput->inputChannel = AudioInputChannel::RIGHT;
		break;

	case Value::STEREO:
		audioOutput->inputChannel = AudioInputChannel::STEREO;
		break;

	case Value::BALANCED:
		audioOutput->inputChannel = AudioInputChannel::BALANCED;
		break;

	case Value::MASTER:
		audioOutput->inputChannel = AudioInputChannel::MIX;
		break;

	case Value::OUTPUT:
		audioOutput->inputChannel = AudioInputChannel::OUTPUT;
		break;
	case Value::TRACK: {
		audioOutput->inputChannel = AudioInputChannel::SPECIFIC_OUTPUT;
		// Preserve the chosen source if possible; only choose a default when the previous source is gone.
		Output* recordFrom = getRecordableOutputInSong(audioOutput, audioOutput->getOutputRecordingFrom());
		if (!recordFrom) {
			recordFrom = getFirstRecordableOutput(audioOutput);
		}
		audioOutput->setOutputRecordingFrom(recordFrom);
		break;
	}

	case Value::USB:
		// Keep whichever channel the track already had, so scrolling past USB and back does not move it. Only a
		// track that has never had one picks up the default.
		if (!isUsbReturnInput(audioOutput->inputChannel)) {
			audioOutput->inputChannel = UsbInputSelector::defaultChannel();
		}
		break;

	default:
		audioOutput->inputChannel = AudioInputChannel::NONE;
	}

	defaultAudioOutputInputChannel = audioOutput->inputChannel;

	if (display->haveOLED()) {
		renderUIsForOled();
	}
}

/// Pressing SELECT on USB goes a level down rather than closing, which is what the arrow on that row means.
///
/// Every other row is its own answer and closing on it is correct - this is the only row that is a door.
bool AudioInputSelector::acceptCurrentOption() {
	if (static_cast<Value>(currentOption) != Value::USB) {
		return false; // Closes, which is what every other row does.
	}
	usbInputSelector.audioOutput = audioOutput;
	if (!usbInputSelector.setupAndCheckAvailability()) {
		return false;
	}
	display->setNextTransitionDirection(1);
	openUI(&usbInputSelector);
	return true;
}

// if they're in session view and press a clip's pad, record from that output
ActionResult AudioInputSelector::padAction(int32_t x, int32_t y, int32_t on) {
	if (on && getUIUpOneLevel() == &sessionView) {
		auto track = (&sessionView)->getOutputFromPad(x, y);
		if (audioOutput->canRecordFrom(track)) {
			audioOutput->inputChannel = AudioInputChannel::SPECIFIC_OUTPUT;
			audioOutput->setOutputRecordingFrom(track);
			if (display->have7SEG()) {
				// OLED shows this persistently in renderOLED(); 7SEG still needs popup feedback.
				display->popupTextTemporary(track->name.get());
			}
			// sets scroll to the position of specific output
			scrollPos = static_cast<int32_t>(Value::TRACK);
			currentOption = scrollPos;
			renderUIsForOled();
		}
		else if (track && track == audioOutput) {
			display->popupTextTemporary("Can't record self!");
		}
		else if (track) {
			display->popupTextTemporary("Can't record MIDI or CV!");
		}

		return ActionResult::DEALT_WITH;
	}
	return ContextMenu::padAction(x, y, on);
}

void AudioInputSelector::renderOLED(deluge::hid::display::oled_canvas::Canvas& canvas) {
	ContextMenu::renderOLED(canvas);

	if (audioOutput->inputChannel != AudioInputChannel::SPECIFIC_OUTPUT
	    && !isUsbReturnInput(audioOutput->inputChannel)) {
		return;
	}

	// Show the selected target in the context menu footer.
	char const* trackName;
	if (isUsbReturnInput(audioOutput->inputChannel)) {
		// Which channel USB currently means, so the row is not a mystery until you open it.
		trackName = usbInputSelector.getOptions()[static_cast<int32_t>(audioOutput->inputChannel)
		                                          - static_cast<int32_t>(AudioInputChannel::USB_1)];
	}
	else {
		Output* recordFrom = getRecordableOutputInSong(audioOutput, audioOutput->getOutputRecordingFrom());
		trackName = recordFrom ? recordFrom->name.get() : "No track";
	}

	int32_t windowHeight = 40;
	int32_t windowMinY = (OLED_MAIN_HEIGHT_PIXELS - windowHeight) >> 1;
	int32_t textPixelY = windowMinY + 20 + kTextSpacingY;
	canvas.drawString(trackName, 22, textPixelY, kTextSpacingX, kTextSpacingY, 0, OLED_MAIN_WIDTH_PIXELS - 26);
}

} // namespace deluge::gui::context_menu
