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

#include "processing/engines/engine_load_report.h"
#include "definitions_cxx.hpp"
#include "io/debug/print.h"
#include "io/midi/sysex.h"
#include "processing/engines/audio_engine.h"
#include "version.h"

namespace deluge::processing::engines {

namespace {

/// One second of rendered audio, not of wall clock. audioSampleTimer advances with samples the engine actually
/// produced, so an interval means the same thing however late the reporting task itself runs.
constexpr uint32_t kReportIntervalSamples = kSampleRate;

uint32_t intervalStart = 0;
bool intervalStarted = false;

uint32_t statRenders = 0;
uint64_t statDspSum = 0;
int32_t statDspMax = 0;
uint64_t statWindowSum = 0;
uint32_t statWindowMax = 0;
int32_t statDirenessMax = 0;
uint32_t statForceReleased = 0;
uint32_t statTerminated = 0;
uint32_t statKilled = 0;

void clearInterval() {
	statRenders = 0;
	statDspSum = 0;
	statDspMax = 0;
	statWindowSum = 0;
	statWindowMax = 0;
	statDirenessMax = 0;
	statForceReleased = 0;
	statTerminated = 0;
	statKilled = 0;
}

} // namespace

void EngineLoadReport::recordRender(int32_t dspTimeSamples, size_t windowSamples, int32_t direness) {
	statRenders++;
	if (dspTimeSamples < 0) {
		dspTimeSamples = 0;
	}
	statDspSum += (uint32_t)dspTimeSamples;
	if (dspTimeSamples > statDspMax) {
		statDspMax = dspTimeSamples;
	}
	statWindowSum += windowSamples;
	if ((uint32_t)windowSamples > statWindowMax) {
		statWindowMax = (uint32_t)windowSamples;
	}
	if (direness > statDirenessMax) {
		statDirenessMax = direness;
	}
}

void EngineLoadReport::recordCull(uint32_t forceReleased, uint32_t terminated, uint32_t killed) {
	statForceReleased += forceReleased;
	statTerminated += terminated;
	statKilled += killed;
}

void EngineLoadReport::routine() {
	const uint32_t now = AudioEngine::audioSampleTimer;
	if (!intervalStarted) {
		intervalStart = now;
		intervalStarted = true;
		clearInterval();
		return;
	}
	if ((uint32_t)(now - intervalStart) < kReportIntervalSamples) {
		return;
	}
	intervalStart = now;

	// Cleared whether or not anything printed, so an interval always covers one second of audio rather than
	// however long it has been since a host last listened.
	if (Debug::midiDebugCable == nullptr) {
		clearInterval();
		return;
	}

	// The version string is the widest field at around 24 characters; the twelve numeric fields reach about ten
	// digits each behind a four-character tag. 320 covers that with room for another field.
	char line[320];
	char* p = line;
	auto emit = [&p](const char* t) {
		while (*t != '\0') {
			*p++ = *t++;
		}
	};
	auto emitDec = [&p](uint32_t v) {
		char tmp[11];
		int n = 0;
		do {
			tmp[n++] = (char)('0' + (v % 10u));
			v /= 10u;
		} while (v != 0u);
		while (n-- > 0) {
			*p++ = tmp[n];
		}
	};

	const uint32_t renders = statRenders;
	const uint32_t dspMean = (renders != 0u) ? (uint32_t)(statDspSum / renders) : 0u;
	const uint32_t windowMean = (renders != 0u) ? (uint32_t)(statWindowSum / renders) : 0u;

	// Identifies the binary, which no other line on this channel does: field names separate families of builds
	// and two builds one commit apart print identical ones.
	emit("EL fw:");
	emit(kFirmwareVersionString);
	emit(" n");
	emitDec(renders);
	// Time the audio task took, expressed in samples, against the 80 at which the engine starts culling. It is
	// the scheduler's running average rather than this render's own cost, so a single late render is smoothed.
	emit(" dsp");
	emitDec(dspMean);
	emit(" dmx");
	emitDec((uint32_t)statDspMax);
	emit(" win");
	emitDec(windowMean);
	emit(" wmx");
	emitDec(statWindowMax);
	emit(" dir");
	emitDec((uint32_t)statDirenessMax);
	emit(" dnw");
	emitDec((uint32_t)(AudioEngine::cpuDireness < 0 ? 0 : AudioEngine::cpuDireness));
	// Sampled here rather than per render: counting voices walks every sound, which is too expensive to do at
	// render rate. Instantaneous, not a peak.
	emit(" vox");
	emitDec((uint32_t)AudioEngine::getNumVoices());
	emit(" aud");
	emitDec((uint32_t)AudioEngine::getNumAudio());
	emit(" cfr");
	emitDec(statForceReleased);
	emit(" ctm");
	emitDec(statTerminated);
	emit(" ckl");
	emitDec(statKilled);
	*p = '\0';
	Debug::sysexDebugPrint(*Debug::midiDebugCable, line, true);

	clearInterval();
}

} // namespace deluge::processing::engines

extern "C" void engineLoadReportRoutine() {
	deluge::processing::engines::EngineLoadReport::routine();
}
