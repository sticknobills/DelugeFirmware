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
#include "definitions.h"
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
uint64_t statDspRawSum = 0;
uint64_t statRpcSum = 0;
int32_t statDspMax = 0;
uint64_t statWindowSum = 0;
uint32_t statWindowMax = 0;
int32_t statDirenessMax = 0;
uint32_t statForceReleased = 0;
uint32_t statTerminated = 0;
uint32_t statKilled = 0;

/// Histogram of the figure the direness and culling tests are actually made against.
///
/// A mean cannot see this: the decision is a threshold crossing, so what matters is how much of the
/// distribution sits past 50 (direness) and past 80 (the base cull limit), not where its centre is. Measured
/// 2026-09-05: opening the USB stream lowers the mean by 4% while culling rises, which a mean and a peak
/// together cannot explain.
///
/// Edges chosen from the constants the engine actually tests, not from round numbers: direnessThreshold is 50,
/// numSamplesLimit is 80, and a hard cull needs 32 past that limit.
constexpr int32_t kEffBucketEdges[] = {40, 50, 60, 70, 80, 96, 112};
constexpr size_t kNumEffBuckets = (sizeof(kEffBucketEdges) / sizeof(kEffBucketEdges[0])) + 1;
uint32_t statEffBuckets[kNumEffBuckets] = {};
uint64_t statEffSum = 0;
int32_t statEffMax = 0;

uint32_t statCullGated = 0;
uint32_t statCullHard = 0;
uint32_t statCullSoftEligible = 0;
uint32_t statCullSoftCulled = 0;
uint32_t statCullBelowMin = 0;
uint32_t statVoiceStartAllowed = 0;
uint32_t statVoiceStartDenied = 0;

/// The codec runs at a fixed rate whatever the engine does, so a gap between renders converts to samples the
/// codec consumed. 400 MHz against 44.1 kHz.
constexpr uint32_t kCyclesPerSample = 9070u;

/// One codec buffer. A gap wider than this is audio that was not there when the hardware asked for it.
constexpr uint32_t kStarveGapSamples = SSI_TX_BUFFER_NUM_SAMPLES;

uint32_t statStarves = 0;
uint32_t statStarveSamples = 0;
uint32_t statGapMax = 0;

/// The cycle counter has to be switched on before it reads anything, and a counter that is off reads a constant
/// zero rather than an error - which would report a machine that never starves. Enabled here rather than relied
/// on from elsewhere, so this file still measures when it is compiled into a stock control.
bool gapCounterEnabled = false;
uint32_t lastEntryCycles = 0;

/// The gap the current entry measured, handed from recordRoutineEntry() to recordBufferBacklog() so the two
/// halves of the starve reading come from the same entry. Not a statistic; cleared by neither.
uint32_t entryGapSamples = 0;
bool entryGapValid = false;

/// The engine's masked backlog reading, at its worst in the interval. Honest only while no lap has happened,
/// which is exactly when it is not needed - reported so the two readings can be compared rather than trusted.
uint32_t statBacklogMax = 0;

/// Entries where the clock says the codec consumed a whole buffer or more since the last one. The mask cannot
/// express this, so these are the starves no reading derived from the backlog can see.
uint32_t statLaps = 0;
uint32_t statLapSamples = 0;

/// Widest disagreement between the clock and the mask on an entry the clock says did not lap. With no lap the
/// two measure the same quantity, so this is the instrument's own error - if it grows to the size of the effect,
/// nothing else on this line about starving is worth reading.
uint32_t statBacklogDisagreeMax = 0;

void clearInterval() {
	statRenders = 0;
	statDspSum = 0;
	statDspRawSum = 0;
	statRpcSum = 0;
	statDspMax = 0;
	statWindowSum = 0;
	statWindowMax = 0;
	statDirenessMax = 0;
	statForceReleased = 0;
	statTerminated = 0;
	statKilled = 0;
	for (uint32_t& bucket : statEffBuckets) {
		bucket = 0;
	}
	statEffSum = 0;
	statEffMax = 0;
	statCullGated = 0;
	statCullHard = 0;
	statCullSoftEligible = 0;
	statCullSoftCulled = 0;
	statCullBelowMin = 0;
	statVoiceStartAllowed = 0;
	statVoiceStartDenied = 0;
	statStarves = 0;
	statStarveSamples = 0;
	statGapMax = 0;
	statBacklogMax = 0;
	statLaps = 0;
	statLapSamples = 0;
	statBacklogDisagreeMax = 0;
}

} // namespace

void EngineLoadReport::recordRender(int32_t dspTimeSamples, int32_t dspTimeRaw, size_t windowSamples, int32_t direness,
                                    uint32_t rendersPerCallX100, int32_t effectiveSamples) {
	statRenders++;
	if (effectiveSamples < 0) {
		effectiveSamples = 0;
	}
	statEffSum += (uint32_t)effectiveSamples;
	if (effectiveSamples > statEffMax) {
		statEffMax = effectiveSamples;
	}
	{
		size_t bucket = kNumEffBuckets - 1;
		for (size_t i = 0; i < kNumEffBuckets - 1; i++) {
			if (effectiveSamples < kEffBucketEdges[i]) {
				bucket = i;
				break;
			}
		}
		statEffBuckets[bucket]++;
	}
	statDspRawSum += (uint32_t)(dspTimeRaw < 0 ? 0 : dspTimeRaw);
	statRpcSum += rendersPerCallX100;
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

void EngineLoadReport::recordRoutineEntry() {
	if (!gapCounterEnabled) {
		Debug::init();
		gapCounterEnabled = true;
		lastEntryCycles = Debug::readCycleCounter();
		entryGapValid = false;
		return;
	}
	const uint32_t nowCycles = Debug::readCycleCounter();
	// The difference, never the absolute value: the counter wraps every 10.7 s and a gap is milliseconds, so
	// unsigned subtraction is right across the wrap and detecting it is not needed.
	const uint32_t gapSamples = (uint32_t)(nowCycles - lastEntryCycles) / kCyclesPerSample;
	lastEntryCycles = nowCycles;
	entryGapSamples = gapSamples;
	entryGapValid = true;
	if (gapSamples > statGapMax) {
		statGapMax = gapSamples;
	}
	if (gapSamples <= kStarveGapSamples) {
		return;
	}
	statStarves++;
	statStarveSamples += gapSamples - kStarveGapSamples;
}

void EngineLoadReport::recordBufferBacklog(uint32_t backlogSamples) {
	if (backlogSamples > statBacklogMax) {
		statBacklogMax = backlogSamples;
	}
	if (!entryGapValid) {
		return;
	}
	entryGapValid = false;
	if (entryGapSamples >= kStarveGapSamples) {
		// The codec consumed at least a whole buffer while the engine was away, so at least this much of what it
		// played was never written this time round. The masked backlog reads small here, which is the blindness
		// this counter exists to route around; the excess is a floor on the damage, not an estimate of it.
		statLaps++;
		statLapSamples += entryGapSamples - kStarveGapSamples;
		return;
	}
	// No lap, so the clock and the mask are measuring the same distance and any difference is instrument error.
	const uint32_t disagreement =
	    (entryGapSamples > backlogSamples) ? (entryGapSamples - backlogSamples) : (backlogSamples - entryGapSamples);
	if (disagreement > statBacklogDisagreeMax) {
		statBacklogDisagreeMax = disagreement;
	}
}

void EngineLoadReport::recordCull(uint32_t forceReleased, uint32_t terminated, uint32_t killed) {
	statForceReleased += forceReleased;
	statTerminated += terminated;
	statKilled += killed;
}

void EngineLoadReport::recordCullDecision(bool gated, bool hardCull, bool softEligible, bool softCulled,
                                          bool belowMinCull) {
	statCullGated += gated ? 1u : 0u;
	statCullHard += hardCull ? 1u : 0u;
	statCullSoftEligible += softEligible ? 1u : 0u;
	statCullSoftCulled += softCulled ? 1u : 0u;
	statCullBelowMin += belowMinCull ? 1u : 0u;
}

void EngineLoadReport::recordVoiceStart(bool allowed) {
	if (allowed) {
		statVoiceStartAllowed++;
	}
	else {
		statVoiceStartDenied++;
	}
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

	// Worst case, counted at the time of writing rather than asserted: "EL fw:" (6) plus a 24-character version,
	// plus 22 numeric fields at a five-character tag and ten digits each (330), plus the terminator - 361. 512
	// leaves room for ten more fields. Re-count when adding one: an earlier version of this line outgrew its
	// array and smashed the stack once a second.
	char line[512];
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
	emit(" dspr");
	emitDec((renders != 0u) ? (uint32_t)(statDspRawSum / renders) : 0u);
	emit(" rpc");
	emitDec((renders != 0u) ? (uint32_t)(statRpcSum / renders) : 0u);
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
	// A render happened, so time passed. A widest gap of zero means the clock is not running, not that the engine
	// is never late - so the instrument says so instead of reporting a machine that never starves.
	emit(" stv");
	if (statRenders != 0u && statGapMax == 0u) {
		emit("?");
	}
	else {
		emitDec(statStarves);
	}
	emit(" stvs");
	emitDec(statStarveSamples);
	emit(" gmx");
	emitDec(statGapMax);
	// The engine's own backlog reading at its worst. Masked to the buffer, so it climbs towards 128 as the
	// machine gets tight and then falls back to a small number once it laps - it reads best exactly when things
	// are worst, which is why the three fields after it exist.
	emit(" bmx");
	emitDec(statBacklogMax);
	// Laps: entries where the clock says a whole buffer or more was consumed while the engine was away. These
	// are the starves the masked reading cannot represent at all.
	emit(" lap");
	emitDec(statLaps);
	emit(" laps");
	emitDec(statLapSamples);
	// The instrument's own error, and the reason to believe or discard the three fields above: with no lap the
	// clock and the mask measure the same distance, so this should stay small. If it approaches the buffer, the
	// clock constant is wrong and nothing here about starving means anything.
	emit(" mxd");
	emitDec(statBacklogDisagreeMax);
	*p = '\0';
	Debug::sysexDebugPrint(*Debug::midiDebugCable, line, true);

	// Second line rather than more fields on the first, so every capture taken before 2026-09-05 stays
	// comparable field-for-field with one taken after it, and neither line goes near the 1024-byte SysEx buffer
	// that silently truncates.
	//
	// Worst case: "EH " (3) plus seventeen fields, each a leading space, a tag of at most 3 characters and ten
	// digits (17 x 14 = 238), plus the terminator - 242 into 384. Counted here rather than asserted; a comment
	// claiming a buffer is big enough is what let the previous one reach 243 of 256.
	p = line;
	emit("EH");
	// The figure the direness and culling tests are actually made against, which dsp and dspr are not.
	emit(" eff");
	emitDec((renders != 0u) ? (uint32_t)(statEffSum / renders) : 0u);
	emit(" emx");
	emitDec((uint32_t)statEffMax);
	// Its distribution, in the buckets the engine's own thresholds fall in: b0 under 40, then 40, 50, 60, 70,
	// 80, 96, and b7 at 112 and over. Direness starts at b2, the base cull limit at b5, a hard cull at b7.
	for (size_t i = 0; i < kNumEffBuckets; i++) {
		emit(" b");
		emitDec((uint32_t)i);
		emit(":");
		emitDec(statEffBuckets[i]);
	}
	// Which way cullVoices() went, so a change of branch is visible rather than only a change of total.
	emit(" cgt");
	emitDec(statCullGated);
	emit(" chd");
	emitDec(statCullHard);
	emit(" cse");
	emitDec(statCullSoftEligible);
	emit(" csc");
	emitDec(statCullSoftCulled);
	emit(" cbm");
	emitDec(statCullBelowMin);
	// Voices the song asked for and the engine refused to start. Under maximum direness the limit is one per
	// render, and a refused voice is indistinguishable from a voice that was never played in any other counter.
	emit(" vsy");
	emitDec(statVoiceStartAllowed);
	emit(" vsn");
	emitDec(statVoiceStartDenied);
	*p = '\0';
	Debug::sysexDebugPrint(*Debug::midiDebugCable, line, true);

	clearInterval();
}

} // namespace deluge::processing::engines

extern "C" void engineLoadReportRoutine() {
	deluge::processing::engines::EngineLoadReport::routine();
}
