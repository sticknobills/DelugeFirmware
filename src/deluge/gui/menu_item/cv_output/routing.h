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
#include "gui/l10n/l10n.h"
#include "gui/menu_item/selection.h"
#include "model/clip/clip.h"
#include "model/song/song.h"
#include "processing/engines/cv_audio_stream.h"

namespace deluge::gui::menu_item::cv_output {

/// The track's output: MAIN, CV1, CV2 or CVSTEREO. One choice per track, stored on the Clip,
/// so it survives a preset change or a change of track type. Like every other selection on the
/// machine, turning the knob applies the choice at once.
///
/// Two instances exist. The one on the Sound root hides itself on kits, because that root is
/// also the Drum editor and a track's output belongs to the whole kit; the kit's instance lives
/// in KIT GLOBAL FX.
class Output final : public Selection {
public:
	Output(l10n::String newName, l10n::String title, bool hideOnKits)
	    : Selection(newName, title), hideOnKits_(hideOnKits) {}

	void readCurrentValue() override {
		Clip* clip = getCurrentClip();
		this->setValue(clip != nullptr ? clip->cvOutput : 0);
	}

	void writeCurrentValue() override {
		Clip* clip = getCurrentClip();
		if (clip != nullptr) {
			clip->cvOutput = static_cast<uint8_t>(this->getValue());
		}
	}

	deluge::vector<std::string_view> getOptions(OptType optType) override {
		using enum l10n::String;
		return {l10n::getView(STRING_FOR_CV_OUTPUT_MAIN), l10n::getView(STRING_FOR_CV_OUTPUT_CV1),
		        l10n::getView(STRING_FOR_CV_OUTPUT_CV2), l10n::getView(STRING_FOR_CV_OUTPUT_STEREO)};
	}

	bool isRelevant(ModControllableAudio* modControllable, int32_t whichThing) const override {
		return processing::engines::cvOutputsAvailable() && !(hideOnKits_ && getCurrentOutputType() == OutputType::KIT);
	}

private:
	const bool hideOnKits_;
};

} // namespace deluge::gui::menu_item::cv_output
