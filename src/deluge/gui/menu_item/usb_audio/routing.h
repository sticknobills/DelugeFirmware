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

#include "gui/menu_item/integer.h"
#include "gui/menu_item/selection.h"
#include "gui/menu_item/submenu.h"
#include "gui/menu_item/toggle.h"
#include "model/output.h"
#include "model/song/song.h"
#include "model/usb_route.h"
#include "processing/engines/usb_audio_stream.h"
#include "storage/flash_storage.h"

namespace deluge::gui::menu_item::usb_audio {

/// One USB destination for the track being edited. A mono channel or a stereo pair; the bit says which.
///
/// Several tracks may name one channel and they add there. A single channel carries the mono sum of the track's two
/// sides, a pair carries left and right - so selecting 1 and 2 separately is not the same as selecting 1-2.
class ChannelToggle final : public Toggle {
public:
	ChannelToggle(l10n::String newName, l10n::String title, uint16_t newBit) : Toggle(newName, title), bit(newBit) {}

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
			output->usbRouting &= (uint16_t)~bit;
		}
	}

private:
	const uint16_t bit;
};

/// Whether the track still reaches the Deluge's own outputs. On by default; off makes the cable its only way out.
///
/// Its reverb tail reaches the main outputs either way - that goes into a buffer shared with every other track
/// during the render and cannot be unpicked afterwards.
class MainToggle final : public Toggle {
public:
	using Toggle::Toggle;

	void readCurrentValue() override {
		Output* output = getCurrentOutput();
		// A track that is not there is in the mix: this is the state that changes nothing.
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
			output->usbRouting &= (uint16_t)~UsbRoute::MAIN;
		}
	}
};

/// Hides itself on tracks with no audio of their own. A MIDI or CV track renders nothing into the mix, so the
/// difference this routing is taken as would always be silence.
class RoutingSubmenu final : public Submenu {
public:
	using Submenu::Submenu;

	bool isRelevant(ModControllableAudio* modControllable, int32_t whichThing) override {
		const OutputType type = getCurrentOutputType();
		return type != OutputType::MIDI_OUT && type != OutputType::CV;
	}
};

/// Which returning pair is summed into the song's own mix: none, 1-2, or 3-4.
///
/// One pair at a time, decided 2026-09-08. The other stays available to a track, and a pair a track has taken is
/// not summed here as well - returning audio is in the song once rather than twice.
class ReturnPair final : public Selection {
public:
	using Selection::Selection;

	void readCurrentValue() override {
		this->setValue((int32_t)deluge::processing::engines::USBAudioStream::getMasterReturnPair());
	}

	void writeCurrentValue() override {
		deluge::processing::engines::USBAudioStream::setMasterReturnPair((uint32_t)this->getValue());
		FlashStorage::usbAudioReturnPair = (uint8_t)this->getValue();
	}

	deluge::vector<std::string_view> getOptions(OptType optType) override {
		return {l10n::getView(l10n::String::STRING_FOR_OFF), l10n::getView(l10n::String::STRING_FOR_USB_PAIR_12),
		        l10n::getView(l10n::String::STRING_FOR_USB_PAIR_34)};
	}
};

/// Level applied to the returning audio, 0-50, 1.2 dB a step. Zero is silence, 40 is unity, 50 is 12 dB up.
///
/// Per-machine rather than per-song: it describes the gain staging of whatever is on the other end of the cable
/// rather than anything about what is being played.
class ReturnLevel final : public Integer {
public:
	using Integer::Integer;

	[[nodiscard]] int32_t getMinValue() const override { return 0; }
	[[nodiscard]] int32_t getMaxValue() const override {
		return (int32_t)deluge::processing::engines::USBAudioStream::kReturnLevelMax;
	}

	void readCurrentValue() override {
		this->setValue((int32_t)deluge::processing::engines::USBAudioStream::getReturnLevel());
	}

	void writeCurrentValue() override {
		deluge::processing::engines::USBAudioStream::setReturnLevel((uint32_t)this->getValue());
		FlashStorage::usbAudioReturnLevel = (uint8_t)this->getValue();
	}
};

} // namespace deluge::gui::menu_item::usb_audio
