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

#include "testing/hardware_testing.h"
#include "definitions_cxx.hpp"
#include "drivers/pic/pic.h"
#include "gui/ui/load/load_song_ui.h"
#include "gui/ui/root_ui.h"
#include "hid/buttons.h"
#include "hid/display/display.h"
#include "hid/display/oled.h"
#include "hid/encoders.h"
#include "hid/led/indicator_leds.h"
#include "hid/matrix/matrix_driver.h"
#include "io/debug/log.h"
#include "io/midi/midi_engine.h"
#include "processing/engines/audio_engine.h"
#include "model/clip/clip.h"
#include "model/song/song.h"
#include "processing/engines/cv_engine.h"
#include "util/cfunctions.h"
#include "util/functions.h"
#include <math.h>
#include <string.h>

extern "C" {
#include "RZA1/gpio/gpio.h"
#include "RZA1/oled/oled_low_level.h"
#include "RZA1/system/iodefines/cpg_iodefine.h"
#include "RZA1/system/iodefines/rspi_iodefine.h"
#include "RZA1/system/iodefines/ssif_iodefine.h"
#include "drivers/dmac/dmac.h"
#include "RZA1/uart/sio_char.h"
#include "drivers/ssi/ssi.h"
#include "drivers/uart/uart.h"
}

namespace encoders = deluge::hid::encoders;

void ramTestUart() {
	// Test the RAM
	uint32_t lastErrorAt = 0;
	uint32_t* address;

	while (1) {

		// while (1) {
		D_PRINTLN("writing to ram");
		address = (uint32_t*)EXTERNAL_MEMORY_BEGIN;
		while (address != (uint32_t*)EXTERNAL_MEMORY_END) {
			*address = (uint32_t)address;
			address++;
		}
		//}

		// while (1) {
		D_PRINTLN("reading back from ram. Checking for errors every megabyte");
		address = (uint32_t*)EXTERNAL_MEMORY_BEGIN;
		while (address != (uint32_t*)EXTERNAL_MEMORY_END) {
			if (*address != (uint32_t)address) {

				uint32_t errorAtBlockNow = ((uint32_t)address) & (0xFFF00000);
				if (errorAtBlockNow != lastErrorAt) {
					while (uartGetTxBufferFullnessByItem(UART_ITEM_MIDI) > 100) {
						;
					}
					D_PRINTLN("error at  %d . got  %d", (uint32_t)address, *address);
					// while(1);
					lastErrorAt = errorAtBlockNow;
				}
			}
			address++;
		}
		//}
		D_PRINTLN("finished checking ram");
	}
}

bool inputStateLastTime = false;

bool nextIsDepress = false;
int16_t encoderTestPos = 128;

void setupSquareWave() {
	// Send square wave
	int32_t count = 0;
	for (int32_t* address = getTxBufferStart(); address < getTxBufferEnd(); address++) {
		if (count < SSI_TX_BUFFER_NUM_SAMPLES) {
			*address = std::numeric_limits<int32_t>::max();
		}
		else {
			*address = std::numeric_limits<int32_t>::min();
			;
		}

		count++;
	}
}

int32_t hardwareTestWhichColour = 0;

void sendColoursForHardwareTest(bool testButtonStates[9][16]) {
	for (int32_t x = 0; x < 9; x++) {

		std::array<RGB, 16> colours{};
		for (int32_t y = 0; y < 16; y++) {
			std::array<uint8_t, 3> raw_colour{};
			for (int32_t c = 0; c < 3; c++) {
				int32_t value = 0;
				if (testButtonStates[x][y]) {
					value = 255;
				}
				else if (c == hardwareTestWhichColour) {
					value = 64;
				}
				raw_colour[c] = value;
			}
			colours[y] = {raw_colour[0], raw_colour[1], raw_colour[2]};
		}

		PIC::setColourForTwoColumns(x, colours);
	}

	PIC::flush();
}

bool anythingProbablyPressed = false;

void readInputsForHardwareTest(bool testButtonStates[9][16]) {
	bool outputPluggedInL = readInput(LINE_OUT_DETECT_L.port, LINE_OUT_DETECT_L.pin);
	bool outputPluggedInR = readInput(LINE_OUT_DETECT_R.port, LINE_OUT_DETECT_R.pin);
	bool headphoneNow = readInput(HEADPHONE_DETECT.port, HEADPHONE_DETECT.pin);
	bool micNow = !readInput(MIC_DETECT.port, MIC_DETECT.pin);
	bool lineInNow = readInput(LINE_IN_DETECT.port, LINE_IN_DETECT.pin);
	bool gateInNow = readInput(ANALOG_CLOCK_IN.port, ANALOG_CLOCK_IN.pin);

	bool inputStateNow = (outputPluggedInL == outputPluggedInR == headphoneNow == micNow == lineInNow == gateInNow);

	if (inputStateNow != inputStateLastTime) {
		indicator_leds::setLedState(IndicatorLED::TAP_TEMPO, !inputStateNow);
		inputStateLastTime = inputStateNow;
	}

	uint8_t value;
	bool anything = uartGetChar(UART_ITEM_PIC, (char*)&value);
	if (anything) {
		if (value == 252) {
			nextIsDepress = true;
		}
		else if (value < 180) {

			int32_t y = (uint32_t)value / 9;
			int32_t x = value - y * 9;
			if (y < kDisplayHeight * 2) {

				testButtonStates[x][y] = !nextIsDepress;
				sendColoursForHardwareTest(testButtonStates);
			}

			if (nextIsDepress) {

				// Send silence
				if (!HARDWARE_TEST_MODE) {
					int32_t count = 0;
					for (int32_t* address = getTxBufferStart(); address < getTxBufferEnd(); address++) {
						*address = 1024;
						count++;
					}
				}

				nextIsDepress = false;
				anythingProbablyPressed = false;
			}
			else {
				if (!HARDWARE_TEST_MODE) {
					setupSquareWave();
				}

				anythingProbablyPressed = true;
			}
		}
		else if (value == oledWaitingForMessage && display->haveOLED()) {
			// delayUS(2500); // TODO: fix
			if (value == 248) {
				oledSelectingComplete();
			}
			else {
				oledDeselectionComplete();
			}
		}
	}

	midiEngine.checkIncomingSerialMidi();
	midiEngine.flushMIDI();

	encoders::readEncoders();

	anything = false;
	for (int32_t e = 0; e < 4; e++) {
		auto& encoder = deluge::hid::encoders::getEncoder(static_cast<deluge::hid::encoders::EncoderName>(e));
		if (encoder.detentPos != 0) {
			encoderTestPos += encoder.detentPos;
			encoder.detentPos = 0;
			anything = true;
		}
	}
	for (int32_t e = 0; e < 2; e++) {
		auto& encoder = deluge::hid::encoders::getEncoder(static_cast<deluge::hid::encoders::EncoderName>(e + 4));
		if (encoder.encPos != 0) {
			encoderTestPos += encoder.encPos;
			encoder.encPos = 0;
			anything = true;
		}
	}

	if (anything) {
		if (encoderTestPos > 128) {
			encoderTestPos = 128;
		}
		else if (encoderTestPos < 0) {
			encoderTestPos = 0;
		}

		indicator_leds::setKnobIndicatorLevel(1, encoderTestPos);
	}

	if (display->haveOLED()) {
		oledRoutine();
	}
	PIC::flush();
	uartFlushIfNotSending(UART_ITEM_MIDI);
}

void ramTestLED(bool stuffAlreadySetUp) {
	bool testButtonStates[9][16];
	memset(testButtonStates, 0, sizeof(testButtonStates));

	// Send CV 10V
	cvEngine.sendVoltageOut(0, 65520);
	cvEngine.sendVoltageOut(1, 65520);

	if (display->haveOLED()) {
		deluge::hid::display::OLED::clearMainImage();
		deluge::hid::display::oled_canvas::Canvas& canvas = deluge::hid::display::OLED::main;

		canvas.invertArea(0, OLED_MAIN_WIDTH_PIXELS, OLED_MAIN_TOPMOST_PIXEL, OLED_MAIN_HEIGHT_PIXELS - 1);
		deluge::hid::display::OLED::sendMainImage();
	}

	midiEngine.midiThru = true;

	if (!HARDWARE_TEST_MODE) {
		setupSquareWave();
	}

	PIC::setFlashLength(100);

	// Switch on numeric display
	PIC::update7SEG({0xFF, 0xFF, 0xFF, 0xFF});

	// Switch on level indicator LEDs
	indicator_leds::setKnobIndicatorLevel(0, 128);
	indicator_leds::setKnobIndicatorLevel(1, 128);

	// Switch on all round-button LEDs
	for (int32_t x = 1; x < 9; x++) {
		if (x == 4) {
			continue; // Skip icecube LEDs
		}
		for (int32_t y = 0; y < 4; y++) {
			PIC::setLEDOn(x + y * 9);
		}
	}

	PIC::flush();

	// Codec
	setPinAsOutput(CODEC.port, CODEC.pin);
	setOutputState(CODEC.port, CODEC.pin, 1); // Switch it on

	// Speaker / amp control
	setPinAsOutput(SPEAKER_ENABLE.port, SPEAKER_ENABLE.pin);
	setOutputState(SPEAKER_ENABLE.port, SPEAKER_ENABLE.pin, 1); // Switch it on

	setPinAsInput(HEADPHONE_DETECT.port, HEADPHONE_DETECT.pin); // Headphone detect
	setPinAsInput(LINE_IN_DETECT.port, LINE_IN_DETECT.pin);     // Line in detect
	setPinAsInput(MIC_DETECT.port, MIC_DETECT.pin);             // Mic detect

	setPinAsOutput(BATTERY_LED.port, BATTERY_LED.pin);    // Battery LED control
	setOutputState(BATTERY_LED.port, BATTERY_LED.pin, 1); // Switch it off (1 is off for open-drain)

	setPinMux(VOLT_SENSE.port, VOLT_SENSE.pin, 1); // Analog input for voltage sense

	setPinAsInput(ANALOG_CLOCK_IN.port, ANALOG_CLOCK_IN.pin); // Gate input

	setPinAsOutput(SYNCED_LED.port, SYNCED_LED.pin);    // Synced LED
	setOutputState(SYNCED_LED.port, SYNCED_LED.pin, 0); // Switch it off

	// Line out detect pins
	setPinAsInput(LINE_OUT_DETECT_L.port, LINE_OUT_DETECT_L.pin);
	setPinAsInput(LINE_OUT_DETECT_R.port, LINE_OUT_DETECT_R.pin);

	// Test the RAM
	uint32_t* address;
	bool ledState = true;

	while (1) {

		sendColoursForHardwareTest(testButtonStates);

		hardwareTestWhichColour = (hardwareTestWhichColour + 1) % 3;

		// Write Synced LED
		setOutputState(SYNCED_LED.port, SYNCED_LED.pin, true);

		// Write gate outputs
		for (int32_t i = 0; i < NUM_GATE_CHANNELS; i++) {
			cvEngine.gateChannels[i].on = ledState;
			cvEngine.physicallySwitchGate(i);
		}

		// Send MIDI
		// midiEngine.sendNote(ledState, 50, 64, 0, true);

		ledState = !ledState;

		address = (uint32_t*)0x0C000000;
		while (address != (uint32_t*)0x10000000) {

			if (((uint32_t)address & 4095) == 0) {
				readInputsForHardwareTest(testButtonStates);
				// AudioEngine::routine(false);
			}
			*address = (uint32_t)address;
			address++;
		}

		setOutputState(SYNCED_LED.port, SYNCED_LED.pin, false);

		address = (uint32_t*)0x0C000000;
		while (address != (uint32_t*)0x10000000) {
			if (((uint32_t)address & 4095) == 0) {
				readInputsForHardwareTest(testButtonStates);
				// AudioEngine::routine(false);
			}

			if (*address != (uint32_t)address) {

				// Error!!!
				while (1) {
					readInputsForHardwareTest(testButtonStates);

					setOutputState(SYNCED_LED.port, SYNCED_LED.pin, true);
					delayMS(50);
					delayMS(50);
					setOutputState(SYNCED_LED.port, SYNCED_LED.pin, false);
					delayMS(50);
					delayMS(50);
					setOutputState(SYNCED_LED.port, SYNCED_LED.pin, true);
					delayMS(50);
					delayMS(50);
					setOutputState(SYNCED_LED.port, SYNCED_LED.pin, false);
					delayMS(50);
					delayMS(50);
					delayMS(50);
					delayMS(50);
					delayMS(50);
					delayMS(50);
					delayMS(50);
					delayMS(50);
					delayMS(50);
					delayMS(50);
				}
			}
			address++;
		}
	}
}

#if AUTOPILOT_TEST_ENABLED
#define AUTOPILOT_NONE 0
#define AUTOPILOT_HOLDING_EDIT_PAD 1
#define AUTOPILOT_HOLDING_AUDITION_PAD 2
#define AUTOPILOT_IN_MENU 3
#define AUTOPILOT_IN_SONG_SAVER 4
#define AUTOPILOT_IN_SONG_LOADER 5

int32_t autoPilotMode = 0;
int32_t autoPilotX;
int32_t autoPilotY;

uint32_t timeNextAutoPilotAction = 0;

void autoPilotStuff() {
	using namespace deluge::hid::button;

	if (!playbackHandler.recording)
		return;

	int32_t timeTilNextAction = timeNextAutoPilotAction - AudioEngine::audioSampleTimer;
	if (timeTilNextAction > 0)
		return;

	int32_t randThing;

	switch (autoPilotMode) {

	case 0:

		if (true) { // getCurrentUI() == &instrumentClipView && getCurrentOutputType() == OutputType::KIT) {
			if (!currentUIMode) {
				randThing = getRandom255();

				// Maybe press an edit pad?
				if (randThing < 70) {
					autoPilotMode = AUTOPILOT_HOLDING_EDIT_PAD;
					autoPilotX = getRandom255() % kDisplayWidth;
					autoPilotY = getRandom255() % kDisplayHeight;

					matrixDriver.padAction(autoPilotX, autoPilotY, true);
				}

				// Or an audition pad?
				else if (randThing < 180) {
					autoPilotMode = AUTOPILOT_HOLDING_AUDITION_PAD;
					autoPilotY = getRandom255() % kDisplayHeight;
					matrixDriver.padAction(kDisplayWidth + 1, autoPilotY, true);
				}

				// Or change sample mode
				else if (randThing < 220) {
					Buttons::buttonAction(SHIFT, true, false);
					matrixDriver.padAction(0, getRandom255() % 4, true);
					Buttons::buttonAction(SHIFT, false, false);

					autoPilotMode = AUTOPILOT_IN_MENU;
				}

				// Or toggle playback
				else if (randThing < 230) {
					Buttons::buttonAction(PLAY, true, false);
				}

				// Or save song
				/*
				else {
				    autoPilotMode = AUTOPILOT_IN_SONG_SAVER;
				    Buttons::buttonAction(SAVE, true, false);
				    Buttons::buttonAction(SAVE, false, false);

				    QwertyUI::enteredText.set("T001");

				    //saveSongUI.performSave(true);
				}
				*/

				// Or load song
				else {
					autoPilotMode = AUTOPILOT_IN_SONG_LOADER;
					openUI(&loadSongUI);
				}
			}
		}
		break;

	case AUTOPILOT_HOLDING_EDIT_PAD:
		autoPilotMode = AUTOPILOT_NONE;
		matrixDriver.padAction(autoPilotX, autoPilotY, false);
		break;

	case AUTOPILOT_HOLDING_AUDITION_PAD:
		randThing = getRandom255();

		// Maybe just release it
		if (randThing < 128) {
			autoPilotMode = AUTOPILOT_NONE;
			matrixDriver.padAction(kDisplayWidth + 1, autoPilotY, false);
		}

		// Or maybe load a sample
		else {
			autoPilotMode = AUTOPILOT_IN_MENU;
			Buttons::buttonAction(KIT, true, false);
		}

		break;

	case AUTOPILOT_IN_MENU:

		// Maybe we already actually exited
		if (getCurrentUI() == getRootUI()) {
			autoPilotMode = AUTOPILOT_NONE;
			break;
		}

		randThing = getRandom255();

		// Maybe turn knob
		if (randThing < 200) {
			randThing = getRandom255();
			int32_t direction = (randThing >= 128) ? 1 : -1;
			getCurrentUI()->selectEncoderAction(direction);
		}

		// Maybe press back
		else if (randThing < 220) {
			Buttons::buttonAction(BACK, true, false);
		}

		// Maybe press encoder button
		else {
			Buttons::buttonAction(SELECT_ENC, true, false);
		}

		break;

	case AUTOPILOT_IN_SONG_SAVER:

		// Maybe we already actually exited
		if (getCurrentUI() == getRootUI()) {
			autoPilotMode = AUTOPILOT_NONE;
			break;
		}

		Buttons::buttonAction(SAVE, true, false);
		Buttons::buttonAction(SAVE, false, false);

		break;

	case AUTOPILOT_IN_SONG_LOADER:

		if (currentUIMode)
			break;

		// Maybe we already actually exited
		if (getCurrentUI() == getRootUI()) {
			autoPilotMode = AUTOPILOT_NONE;
			break;
		}

		randThing = getRandom255();

		// Maybe turn knob
		if (randThing < 200) {
			randThing = getRandom255();
			int32_t direction = (randThing >= 128) ? 1 : -1;
			getCurrentUI()->selectEncoderAction(direction);
		}

		// Maybe press back
		else if (randThing < 220) {
			Buttons::buttonAction(BACK, true, false);
		}

		// Maybe press load button
		else {
			// matrixDriver.buttonAction(LOAD, true, false);
			// matrixDriver.buttonAction(LOAD, false, false);

			loadSongUI.performLoad(storageManager);
		}

		break;
	}

	timeNextAutoPilotAction = AudioEngine::audioSampleTimer + getRandom255() * 100;
}

#endif

// ============================================================================
// CV output diagnostics  --  non-destructive, no files, no persistent state
// ----------------------------------------------------------------------------
// Characterises the CV path (DAC -> MC33078 -> jack) with an external recorder.
// Earlier passes established that the path is a single-pole low-pass with its
// corner near 1 kHz, and that it compresses somewhere above a third of full
// swing. These two routines pin down both numbers precisely.
//
//   cvFrequencyResponse()  13 tones, constant safe level    SHIFT + CV
//   cvLevelSweep()         4 kHz at 1.5 dB level steps      SHIFT + SYNTH
//
// Both block for ~10s then return; normal operation resumes afterwards.
// ============================================================================

namespace {

constexpr uint32_t kCvTestTicksPerSample = 12; // 528 kHz system tick / 12 = 44 kHz
constexpr uint32_t kCvTestSampleRate = 44000;
constexpr uint16_t kCvTestCentre = 32768; // mid-rail, so we can swing both ways
constexpr uint32_t kCvSineTableSize = 256;
constexpr uint32_t kCvTestRampSamples = kCvTestSampleRate / 5; // 200 ms
constexpr uint32_t kCvTestFadeSamples = 500;                   // ~11 ms, kills block-boundary clicks

int16_t cvTestSineTable[kCvSineTableSize];
uint16_t cvTestNextTick;

/// Comfortably below where the output began compressing in the previous pass,
/// so the frequency response is measured in the linear region.
constexpr int32_t kCvSafeAmplitude = 6000;

/// Spaced to resolve the shape of the filter across the range we can realistically correct.
constexpr uint32_t kCvResponseFreqs[] = {100, 150, 220, 330, 470, 680, 1000, 1500, 2200, 3300, 4700, 6800, 10000};

/// 1.5 dB apart, bracketing the compression onset seen in the previous pass.
constexpr int32_t kCvLevelSteps[] = {6000, 7135, 8485, 10091, 12000, 14270, 16971, 20182, 24000};
constexpr uint32_t kCvLevelTestFreq = 4000;

/// A short sawtooth bassline, used to judge the corrected output by ear rather
/// than by graph. Semitone offsets from A1; a saw is used because it has strong
/// harmonics all the way up, so the filtering is immediately audible.
constexpr int32_t kCvMusicNotes[] = {0, 0, 12, 0, 7, 0, 10, 0, 0, 12, 0, 3, 0, 7, 5, 0};
constexpr float kCvMusicRootHz = 55.0f;
constexpr uint32_t kCvMusicStepSamples = 5060; // ~115 ms, 16ths at 130 bpm
constexpr uint32_t kCvMusicRepeats = 3;
constexpr float kCvMusicAmplitude = 2500.0f;
constexpr int32_t kCvMusicClamp = 14000; // safety net; keeps us out of the squashed region
constexpr float kCvMusicEnvDecay = 0.999408f;

/// Inverse of the measured output filter, as a one-pole/one-zero shelf, at four
/// strengths. Pole comes from the measured 700 Hz corner; the limit caps the
/// boost so we stay inside the linear range found by the level sweep.
/// Flat up to roughly 1.2 kHz, 2.3 kHz and 4.1 kHz respectively.
constexpr float kCvEqPole = 0.9049f;
struct CvEqSetting {
	char const* name;
	float limit;
	float scale;
};
constexpr CvEqSetting kCvEqSettings[] = {
    {"EQA", 0.78601f, 2.25016f}, // x2.40, flat to ~1.5 kHz
    {"EQB", 0.75857f, 2.53870f}, // x2.75, flat to ~1.8 kHz
    {"EQC", 0.70251f, 3.12818f}, // x3.50, flat to ~2.3 kHz
    {"EQD", 0.61663f, 4.03123f}, // x4.75, flat to ~3.3 kHz
};
/// Busy-wait until the free-running 16-bit system timer reaches `target`.
/// Signed comparison so counter wraparound is handled for free.
inline void cvTestWaitUntil(uint16_t target) {
	while ((int16_t)((uint16_t)MTU2.TCNT_0 - target) < 0) {}
}

void cvTestBegin(char const* label) {
	display->setText(label);
	for (uint32_t i = 0; i < kCvSineTableSize; i++) {
		cvTestSineTable[i] = (int16_t)(32767.0f * sinf(2.0f * 3.1415926535f * (float)i / (float)kCvSineTableSize));
	}
	cvTestNextTick = (uint16_t)MTU2.TCNT_0;
}

/// Fade between 0 V and the centre voltage so nothing downstream sees a step.
void cvTestRamp(bool up) {
	for (uint32_t i = 0; i < kCvTestRampSamples; i++) {
		cvTestNextTick += kCvTestTicksPerSample;
		cvTestWaitUntil(cvTestNextTick);
		uint32_t position = up ? i : (kCvTestRampSamples - i);
		cvEngine.sendVoltageOut(0, (uint16_t)((uint32_t)kCvTestCentre * position / kCvTestRampSamples));
	}
}

/// One tone block, topped and tailed with a short fade so the recording has
/// clean edges to measure against.
void cvTestEmitTone(uint32_t freq, int32_t amplitude, uint32_t numSamples) {
	uint32_t phase = 0;
	const uint32_t phaseIncrement = (uint32_t)(((uint64_t)freq << 32) / kCvTestSampleRate);

	for (uint32_t i = 0; i < numSamples; i++) {
		cvTestNextTick += kCvTestTicksPerSample;
		cvTestWaitUntil(cvTestNextTick);

		int32_t envelope = amplitude;
		if (i < kCvTestFadeSamples) {
			envelope = (int32_t)((int64_t)amplitude * i / kCvTestFadeSamples);
		}
		else if (i >= numSamples - kCvTestFadeSamples) {
			envelope = (int32_t)((int64_t)amplitude * (numSamples - i) / kCvTestFadeSamples);
		}

		int32_t sample = ((int32_t)cvTestSineTable[phase >> 24] * envelope) >> 15;
		phase += phaseIncrement;
		cvEngine.sendVoltageOut(0, (uint16_t)((int32_t)kCvTestCentre + sample));
	}
}

} // namespace

/// Thirteen tones at one constant, safe level. The envelope of the recording is
/// the filter's frequency response, measured where the output is still linear.
void cvFrequencyResponse() {
	cvTestBegin("CVFR");
	cvTestRamp(true);

	for (uint32_t freq : kCvResponseFreqs) {
		cvTestEmitTone(freq, kCvSafeAmplitude, (kCvTestSampleRate * 7) / 10); // 700 ms
	}

	cvTestRamp(false);
	display->setText("DONE");
}

/// One frequency, nine levels 1.5 dB apart. The step at which the recording
/// stops gaining a full 1.5 dB is the top of the usable range.
void cvLevelSweep() {
	cvTestBegin("CVLV");
	cvTestRamp(true);

	for (int32_t amplitude : kCvLevelSteps) {
		cvTestEmitTone(kCvLevelTestFreq, amplitude, kCvTestSampleRate); // 1 s
	}

	cvTestRamp(false);
	display->setText("DONE");
}

// ============================================================================
// Gate driver speed diagnostic  --  SHIFT + CV advances one step per press
// ----------------------------------------------------------------------------
// Answers one question: how fast can a gate socket's output driver actually
// switch? That decides whether the gate sockets can carry an I2S digital audio
// link -- a route that would work on both hardware variants and need none of
// the analogue correction the CV route needs.
//
// The method is a square wave read as a DC average on an ordinary multimeter.
// A meter set to DC volts reports the mean of what it sees. A 50% duty square
// wave the driver keeps up with averages half the rail. The gate driver is
// asymmetric -- it pulls down hard through a transistor but pulls up slowly
// through a resistor -- so once the rising edge can no longer finish inside
// half a period the waveform goes lopsided and the average sags. The frequency
// where the average starts falling IS the driver's speed limit.
//
// The square wave comes from SSI3's bit clock, because gate 1 is P2_7 and
// P2_7's alternate function 3 is SSISCK3. That gives an exact frequency
// straight from a crystal and a divider, with nothing depending on how fast
// this code happens to run.
//
// NOTHING HERE BLOCKS. An earlier version ran the whole sweep inside the button
// handler with delays between steps, and that was wrong twice over: the display
// cannot update while the main loop is stalled (setText only fills the UART
// buffer -- PIC::flush happens in the main loop, which is why freezeWithError
// has to call it by hand), and a stall that long looks exactly like a crash.
// Since SSI3 generates its clock in hardware, the CPU can set a divider up and
// walk away, so each press configures one step and returns immediately. The
// Deluge stays live and the display works.
//
// The first two steps are deliberately GPIO-only and involve SSI3 not at all.
// They prove the chord fires, the display updates and the pin can be driven,
// so that if a later step misbehaves it is isolated to the SSI3 setup.
//
// Non-destructive: nothing is written to flash or the card, and the pins are
// handed back to the gate engine on the last press.
// ============================================================================

namespace {

constexpr uint8_t kGateSweepPort = 2;
constexpr uint8_t kGateSweepClockPin = 7; // gate 1, SSISCK3
constexpr uint8_t kGateSweepWsPin = 9;    // gate 3, SSIWS3
constexpr uint32_t kGateSweepSsiChannel = 3;

/// The SSI3 signals are alternate function 2 on these pins, NOT function 3.
///
/// This cost a debugging round. Mux 3 is right for every other serial pin on this
/// board -- P6_0 RSPCK0, P6_1 SSL00, P6_2 MOSI0, P6_8 SSISCK0, P6_9 SSIWS0 -- so
/// 3 got assumed here too. The datasheet pin table (Table 2.2) disagrees:
///
///   P2_7   F1 CS0   F2 SSISCK3   F3 TIOC1A   F4 IRQ2
///   P2_8   F1 RD    F2 SSITxD3   F3 TIOC0A   F5 CAN0TX
///   P2_9   F1 A0    F2 SSIWS3    F3 SCK0     F4 IRQ1   F5 CAN0RX
///
/// against, for comparison, the ones that really are function 3:
///
///   P6_1   F1 D17   F2 LCD0_DATA9    F3 SSL00     F4 TCLKB
///   P6_8   F1 D24   F2 LCD0_DATA16   F3 SSISCK0   F4 IRQ1
///
/// Selecting 3 here put P2_7 on TIOC1A and P2_9 on SCK0 -- alternate functions
/// with nothing driving them -- so the sockets sat at a static level and looked
/// exactly like an SSI that had failed to start.
constexpr uint8_t kGateSweepPinMux = 2;

/// stb.c documents CPG.STBCR11 as [1],[1],SSIF0,SSIF1,SSIF2,SSIF3,SSIF4,SSIF5,
/// so SSIF3 is bit 2. In Renesas standby registers 1 means "module stopped",
/// which is why waking SSI3 is a clear and not a set.
constexpr uint8_t kStbcr11Ssif3Mask = (1u << 2);

/// TTRG exactly as SSI0 uses it, both FIFO resets RELEASED, no interrupts enabled.
///
/// The released resets are the whole point. The first version of this used 0xA3,
/// copying SSI_SSIFCR_BASE_INIT_VALUE, which holds TFRST and RFRST asserted --
/// and a transmitter held in reset never clocks, which is exactly what the
/// hardware showed: the pin went under SSI control but sat at a DC level.
/// ssiInit2 uses 0xA3 as an intermediate state and ssiStart then clears bits 1
/// and 0 before enabling; 0xA0 is the value SSI0 actually runs at.
///
/// Interrupt enables are deliberately left off. ssiStart sets TIE and RIE for
/// SSI0, which has handlers registered; SSI3 has none.
constexpr uint32_t kGateSweepSsifcr = 0xA0u;

enum class GateSweepMode : uint8_t {
	GPIO_HIGH, ///< pin driven so the socket sits high -- a static control, no SSI
	GPIO_LOW,  ///< pin driven so the socket sits low -- a static control, no SSI
	SSI,       ///< SSI3 clocking at one divider
};

struct GateSweepStep {
	GateSweepMode mode;
	uint8_t ckdv;      ///< CKDV field value, ignored unless mode is SSI
	char const* label; ///< what the display shows, and what to write down
};

/// CKDV is SSICR[7:4], and its encoding is not a plain power of two -- values 8
/// to 12 interleave /6, /12, /24, /48 and /96 among the binary dividers, which is
/// how a 22.5792 MHz crystal reaches exactly 44.1 kHz. Four are pinned down by
/// the comments in drv_ssif_user.h (3 = /8, 4 = /16, 9 = /12, 2 = /4) and the
/// rest follow that table. Labels are AUDIO_X1 (22.5792 MHz) over the divider,
/// in kHz, rounded to fit the four-character display.
constexpr GateSweepStep kGateSweepSteps[] = {
    {GateSweepMode::GPIO_HIGH, 0, "HI"}, // control: socket parked high, expect the full rail
    {GateSweepMode::GPIO_LOW, 0, "LO"},  // control: socket parked low, expect near zero
    {GateSweepMode::SSI, 7, "GT3"},      // same as the 176 step; probe gate 3 for SSIWS3 at 2.75 kHz
    {GateSweepMode::SSI, 7, "176"},      // /128
    {GateSweepMode::SSI, 12, "235"},     // /96
    {GateSweepMode::SSI, 6, "353"},      // /64
    {GateSweepMode::SSI, 11, "470"},     // /48
    {GateSweepMode::SSI, 5, "706"},      // /32
    {GateSweepMode::SSI, 10, "941"},     // /24
    {GateSweepMode::SSI, 4, "1411"},     // /16 -- I2S at a 16-bit system word
    {GateSweepMode::SSI, 9, "1882"},     // /12
    {GateSweepMode::SSI, 3, "2822"},     // /8  -- I2S at a 32-bit system word, full rate
    {GateSweepMode::SSI, 8, "3763"},     // /6
    {GateSweepMode::SSI, 2, "5645"},     // /4
};

constexpr int32_t kGateSweepNumSteps = sizeof(kGateSweepSteps) / sizeof(kGateSweepSteps[0]);

/// Which step is live. -1 means the diagnostic is off and both pins are back
/// under the gate engine.
int32_t gateSweepStep = -1;

void gateSweepStopSsi() {
	SSIF3.SSICR &= ~0b11u;            // TEN and REN off, clock stops
	CPG.STBCR11 |= kStbcr11Ssif3Mask; // back to standby
	(void)CPG.STBCR11;
}

/// Brings SSI3 up as clock master at one divider and leaves it running. Only the
/// clock matters here, so the FIFO stays in reset -- in master mode SSISCK runs
/// whether or not there is data to send.
void gateSweepStartSsi(uint8_t ckdv) {
	// Wake the module. It boots into standby: stb.c parks every SSI except SSI0.
	// Touching an SSI3 register before its clock is running would hang on the bus,
	// so give it a moment rather than only the dummy read stb.c uses.
	CPG.STBCR11 &= (uint8_t)~kStbcr11Ssif3Mask;
	(void)CPG.STBCR11;
	delayMS(1);

	// Software-reset SSI3 exactly as ssiInit2 does for SSI0.
	CPG.SWRSTCR1 |= (uint8_t)(1 << (6 - kGateSweepSsiChannel));
	(void)CPG.SWRSTCR1;
	CPG.SWRSTCR1 &= (uint8_t)~(1 << (6 - kGateSweepSsiChannel));
	(void)CPG.SWRSTCR1;

	SSIF3.SSITDMR = 0;

	// CKS = 0 selects AUDIO_X1, the same 22.5792 MHz source SSI0 already runs from.
	// SCKD and SWSD make us clock master, which is what actually puts a clock on
	// the pin. Word lengths match SSI0's and do not affect the bit clock rate.
	SSIF3.SSICR = (0u << 30)     // CKS: AUDIO_X1
	              | (0u << 22)   // CHNL: 1 channel per system word
	              | (0x1u << 19) // DWL: 16-bit data word
	              | (0x3u << 16) // SWL: 32-bit system word
	              | (1u << 15)   // SCKD: bit clock master
	              | (1u << 14)   // SWSD: word select master
	              | ((uint32_t)ckdv << 4);

	SSIF3.SSIFCR = kGateSweepSsifcr;

	// Prime the transmit FIFO before enabling. A master's clock should run
	// regardless of whether there is data, but starting an empty transmitter is
	// the kind of thing that can stall a peripheral, and these writes cost nothing.
	// The pattern is irrelevant -- SSITxD3 is on P2_8, which is never muxed here,
	// so gate 2 goes on working as a gate.
	for (int32_t i = 0; i < 8; i++) {
		SSIF3.SSIFTDR.LONG = 0xAAAAAAAAu;
	}

	// TEN and REN together, matching what ssiStart does for SSI0. Enabling the
	// receiver costs nothing -- SSIRxD3 is on P2_6 and is never muxed -- and it
	// keeps this as close as possible to the one SSI configuration on this board
	// that is known to work.
	SSIF3.SSICR |= 0b11u; // the clock starts here
}

/// Hands both pins back to the gate engine exactly as CVEngine::init left them.
void gateSweepReleasePins() {
	setPinAsOutput(kGateSweepPort, kGateSweepClockPin);
	setPinAsOutput(kGateSweepPort, kGateSweepWsPin);
	cvEngine.physicallySwitchGate(0); // gate 1
	cvEngine.physicallySwitchGate(2); // gate 3
}

} // namespace

/// One press, one step. Read the meter, press again. The label on the display is
/// the frequency in kHz, or HI/LO for the two static controls at the start.
/// Pressing past the last step shuts everything down and shows OFF.
void gateSpeedSweepAdvance() {
	// Whatever was running, stop it before setting up the next thing.
	if (gateSweepStep >= 0 && gateSweepStep < kGateSweepNumSteps
	    && kGateSweepSteps[gateSweepStep].mode == GateSweepMode::SSI) {
		gateSweepStopSsi();
	}

	gateSweepStep++;

	if (gateSweepStep >= kGateSweepNumSteps) {
		gateSweepReleasePins();
		gateSweepStep = -1;
		display->setText("OFF");
		return;
	}

	GateSweepStep const& step = kGateSweepSteps[gateSweepStep];
	display->setText(step.label);

	switch (step.mode) {
	case GateSweepMode::GPIO_HIGH:
		setPinAsOutput(kGateSweepPort, kGateSweepClockPin);
		// The driver inverts: a low on the pin turns the transistor off and lets
		// the pull-up take the socket high.
		setOutputState(kGateSweepPort, kGateSweepClockPin, 0);
		break;

	case GateSweepMode::GPIO_LOW:
		setPinAsOutput(kGateSweepPort, kGateSweepClockPin);
		setOutputState(kGateSweepPort, kGateSweepClockPin, 1);
		break;

	case GateSweepMode::SSI:
		setPinMux(kGateSweepPort, kGateSweepClockPin, kGateSweepPinMux); // SSISCK3 on gate 1
		setPinMux(kGateSweepPort, kGateSweepWsPin, kGateSweepPinMux);    // SSIWS3 on gate 3
		gateSweepStartSsi(step.ckdv);
		break;
	}
}

// ============================================================================
// I2S audio out of the gate sockets  --  SHIFT + CV
// ----------------------------------------------------------------------------
// Sends real I2S out of gates 1, 2 and 3 into an external PCM5102 DAC:
//
//   gate 1  P2_7  SSISCK3  bit clock    -> DAC BCK
//   gate 2  P2_8  SSITxD3  data         -> DAC DIN
//   gate 3  P2_9  SSIWS3   word select  -> DAC LCK
//
// All three at alternate function 2 (NOT 3 -- see the note on kGateSweepPinMux).
// Gate 4 is untouched and stays a gate.
//
// Format: 16-bit data word in a 16-bit system word, two channels per frame, so
// 32 bits per frame -- 32FS, which is what the PCM5102A's internal PLL accepts.
// CKDV 4 gives a 1.4112 MHz bit clock (44.1 kHz) and CKDV 5 gives 705.6 kHz
// (22.05 kHz). Both are inside the chip's timing spec by a wide margin: it asks
// for bit clock pulses of at least 16 ns and ours are hundreds of ns, and it
// specifies no duty cycle limit at all -- which matters, because the gate
// driver's slow turn-off makes the clock markedly asymmetric.
//
// Nothing here is fed by the audio engine yet. It plays one of two fixed
// patterns out of a buffer the DMA loops forever, which needs no refilling, no
// write pointer and no interrupt -- the whole thing runs on the transfer engine
// with the CPU asleep. That keeps the first hardware test to "is the signal
// right", with none of the render-path plumbing able to confuse the answer.
// ============================================================================

/// Pushes the CPU's writes out of cache so the transfer engine sees them. Declared
/// again here because the CV section further down declares it below this point.
extern "C" void v7_dma_flush_range(uint32_t start, uint32_t end);

namespace {

constexpr uint8_t kI2sPort = 2;
constexpr uint8_t kI2sSckPin = 7; // gate 1, bit clock
constexpr uint8_t kI2sTxPin = 8;  // gate 2, data
constexpr uint8_t kI2sWsPin = 9;  // gate 3, word select
constexpr uint32_t kI2sSsiChannel = 3;
/// Channel 9. Claimed channels are 2 and 3 (SD card), 4 (OLED SPI), 5 (CV stream),
/// 6 and 7 (audio codec) and 10 to 14 (PIC and MIDI). 8, 9 and 15 are the only ones
/// with no reference anywhere in the tree.
///
/// Channel 3 looks free -- sd_cfg.h even comments SD0_DMA_CHANNEL as "not actually
/// used", since the Deluge runs on SD port 1. But sd_dev_low.c still calls
/// sd_DMAC_Open, sd_DMAC_Close and sd_DMAC_Get_Endflag on it, so its registers do
/// get programmed. Sharing a DMA channel with anything that touches the SD card is
/// not a risk worth taking for the sake of a channel number.
constexpr int32_t kI2sDmaChannel = 9;

/// 512 stereo frames, ONE 32-bit buffer word each.
///
/// Not two. With a 16-bit data word in a 16-bit system word, each 32-bit FIFO entry
/// carries two samples, and PDTA = 0 sends the lower half first. The first version
/// wrote one sample per entry, left-justified, so the SSI read the buffer as left
/// 0x0000, right 0x0000, left 0x0000, right 0xFFFF -- three-quarters of the slots
/// silent. On the meter that showed as 73.7% duty on the data line against the 50%
/// intended, which is how it was caught.
///
/// Sized so a whole number of sine cycles fits exactly, which is what lets the DMA
/// loop the buffer forever without a seam.
constexpr uint32_t kI2sFrames = 512;
constexpr uint32_t kI2sWords = kI2sFrames;

/// Cycles of the test tone per buffer. 5 over 512 frames is 430.7 Hz at 44.1 kHz
/// and half that at 22.05 kHz -- and hearing the pitch halve when the rate is
/// switched is itself a confirmation that the divider really changed.
constexpr uint32_t kI2sToneCycles = 5;
constexpr float kI2sToneAmplitude = 28000.0f; // ~85% of full scale, leaves headroom

/// The gate driver inverts, so every word is pre-inverted here to cancel it.
/// The bit clock and word select are inverted back in hardware instead, by SCKP
/// and SWSP -- data is the one line with no polarity bit, so it has to be done
/// in software. IF THE FIRST TEST GIVES NOISE INSTEAD OF A TONE, THIS FLAG IS
/// THE FIRST THING TO TRY FLIPPING.
constexpr bool kI2sInvertData = true;

PLACE_SDRAM_DATA uint32_t i2sBuffer[kI2sWords] __attribute__((aligned(CACHE_LINE_SIZE)));

const uint32_t i2sDmaLinkDescriptor[] __attribute__((aligned(CACHE_LINE_SIZE))) = {
    0b1101,                                                                    // Header
    (uint32_t)i2sBuffer,                                                       // Source
    (uint32_t)&SSIF3.SSIFTDR.LONG,                                             // Destination
    sizeof(i2sBuffer),                                                         // Transaction size
    0b10000001001000100010001000101000 | DMA_LVL_FOR_SSI | (kI2sDmaChannel & 7), // Config
    0,                                                                         // Interval
    0,                                                                         // Extension
    (uint32_t)i2sDmaLinkDescriptor                                             // Next link: itself
};

struct I2sStep {
	uint8_t ckdv;      ///< 4 = /16 = 44.1 kHz, 5 = /32 = 22.05 kHz
	bool tone;         ///< false: DC pattern for the multimeter. true: audible tone
	char const* label;
};

/// The two DC-pattern steps come first because they are the ones that can be
/// checked with nothing but a multimeter, before any DAC exists.
/// Labels are two characters, leaving room on the 7SEG for two diagnostic digits:
/// the transmit FIFO level and the DMA channel state.
constexpr I2sStep kI2sSteps[] = {
    {4, false, "P1"}, // 44.1 kHz, meter pattern
    {5, false, "P2"}, // 22.05 kHz, meter pattern
    {4, true, "T1"},  // 44.1 kHz, tone
    {5, true, "T2"},  // 22.05 kHz, tone
};
constexpr int32_t kI2sNumSteps = sizeof(kI2sSteps) / sizeof(kI2sSteps[0]);

/// -1 means stopped and the pins are back under the gate engine.
int32_t i2sStep = -1;

/// One stereo frame packed into a single 32-bit FIFO entry, both samples
/// pre-inverted. PDTA = 0 sends the lower half first, so the left channel goes in
/// the low 16 bits and the right in the high 16.
inline uint32_t i2sFrameWord(int32_t left, int32_t right) {
	uint16_t l = (uint16_t)(int16_t)left;
	uint16_t r = (uint16_t)(int16_t)right;
	if (kI2sInvertData) {
		l = (uint16_t)~l;
		r = (uint16_t)~r;
	}
	return ((uint32_t)r << 16) | (uint32_t)l;
}

/// Left channel all ones, right channel all zeros. At the socket that is a square
/// wave at the frame rate on the data line -- 44.1 or 22.05 kHz, slow enough that
/// the gate driver reproduces it almost perfectly, so a multimeter reads half rail.
/// The word select line is a square wave at the same rate for the same reason.
/// Two independent half-rail readings, with no DAC and no external parts.
/// The frame i2sFillPattern writes. Left all ones, right all zeros unless the
/// pulse-width test has set another before starting a step.
uint32_t i2sPatternWord = i2sFrameWord(-1, 0);

void i2sFillPattern() {
	for (uint32_t f = 0; f < kI2sFrames; f++) {
		i2sBuffer[f] = i2sPatternWord;
	}
}

/// Tone on the left channel, silence on the right. Deliberately not both: if the
/// tone comes out of the right speaker instead, the word select polarity is
/// inverted, and that is a one-bit fix rather than a mystery.
void i2sFillTone() {
	for (uint32_t f = 0; f < kI2sFrames; f++) {
		float phase = 2.0f * 3.1415926535f * (float)(kI2sToneCycles * f) / (float)kI2sFrames;
		i2sBuffer[f] = i2sFrameWord((int32_t)(kI2sToneAmplitude * sinf(phase)), 0);
	}
}

void i2sStop() {
	// Stop the transfer engine before touching the SSI, or it keeps writing into a
	// register being reconfigured.
	DMACn(kI2sDmaChannel).CHCTRL_n |= DMAC0_CHCTRL_n_CLREN;

	SSIF3.SSICR &= ~0b11u;            // TEN is bit 1 and REN is bit 0 -- clear both
	CPG.STBCR11 |= kStbcr11Ssif3Mask; // SSI3 back to standby
	(void)CPG.STBCR11;

	// Hand all three pins back exactly as CVEngine::init left them.
	setPinAsOutput(kI2sPort, kI2sSckPin);
	setPinAsOutput(kI2sPort, kI2sTxPin);
	setPinAsOutput(kI2sPort, kI2sWsPin);
	cvEngine.physicallySwitchGate(0);
	cvEngine.physicallySwitchGate(1);
	cvEngine.physicallySwitchGate(2);
}

void i2sStart(I2sStep const& step) {
	if (step.tone) {
		i2sFillTone();
	}
	else {
		i2sFillPattern();
	}
	// The DMA reads this out of SDRAM, so the CPU's writes have to be pushed out of
	// cache first. The buffer is never touched again afterwards, so this once is enough.
	v7_dma_flush_range((uint32_t)i2sBuffer, (uint32_t)i2sBuffer + sizeof(i2sBuffer));

	setPinMux(kI2sPort, kI2sSckPin, kGateSweepPinMux);
	setPinMux(kI2sPort, kI2sTxPin, kGateSweepPinMux);
	setPinMux(kI2sPort, kI2sWsPin, kGateSweepPinMux);

	// Wake SSI3 out of standby, then give the module clock a moment to start before
	// any of its registers are touched -- reading one too early hangs on the bus.
	CPG.STBCR11 &= (uint8_t)~kStbcr11Ssif3Mask;
	(void)CPG.STBCR11;
	delayMS(1);

	CPG.SWRSTCR1 |= (uint8_t)(1 << (6 - kI2sSsiChannel));
	(void)CPG.SWRSTCR1;
	CPG.SWRSTCR1 &= (uint8_t)~(1 << (6 - kI2sSsiChannel));
	(void)CPG.SWRSTCR1;

	SSIF3.SSITDMR = 0;

	// SCKP and SWSP are both 1, which is the opposite of what SSI0 uses. That is
	// deliberate: the gate driver inverts, so inverting here puts the bit clock and
	// word select back the right way up at the socket.
	SSIF3.SSICR = (0u << 30)                      // CKS: AUDIO_X1, 22.5792 MHz
	              | (0u << 22)                    // CHNL: 1 channel per system word
	              | (0x1u << 19)                  // DWL: 16-bit data word
	              | (0x1u << 16)                  // SWL: 16-bit system word -> 32 bits/frame
	              | (1u << 15)                    // SCKD: bit clock master
	              | (1u << 14)                    // SWSD: word select master
	              | (1u << 13)                    // SCKP: inverted, cancels the driver
	              | (1u << 12)                    // SWSP: inverted, cancels the driver
	              | (0u << 8)                     // DEL: one-clock delay, standard I2S
	              | ((uint32_t)step.ckdv << 4);

	// TTRG as SSI0 uses it, TRANSMIT FIFO reset released, TIE set, RECEIVE FIFO HELD
	// IN RESET, RIE clear.
	//
	// Holding the receive side in reset matters. An earlier version copied ssi.c and
	// released both, then enabled TEN and REN together -- but ssi.c also runs an RX
	// DMA channel that drains the receive FIFO, and this does not. With REN set and
	// nothing emptying it, the receive FIFO fills within microseconds and overflows,
	// and that error halts the module. The clock divider carries on regardless, so
	// the symptom was a perfect bit clock with word select and data frozen.
	//
	// TIE matters just as much, and for a reason that is easy to get wrong. It reads
	// as "transmit interrupt enable", and this is a DMA-driven link with no interrupt
	// handler, so leaving it clear looks correct. It is not: the TXI request that TIE
	// gates is the very signal the DMAC uses as its peripheral transfer request. With
	// TIE clear the channel arms and then waits forever -- CHSTAT shows enabled and
	// active while the FIFO never fills, which is exactly what the hardware showed.
	//
	// It cannot cause a spurious CPU interrupt: SSI3's interrupt is never registered
	// with the INTC, so the request is masked there and only ever reaches the DMAC.
	SSIF3.SSIFCR = 0xA9u;

	initDMAWithLinkDescriptor(kI2sDmaChannel, i2sDmaLinkDescriptor,
	                          DMARS_FOR_SSI0_TX + kI2sSsiChannel * 4); // 0xED for SSI3 TX
	dmaChannelStart(kI2sDmaChannel);

	// Prime the transmit FIFO by hand as well. The DMA should fill it, but starting
	// an empty transmitter risks an immediate underflow, and these writes cost nothing.
	for (int32_t i = 0; i < 4; i++) {
		SSIF3.SSIFTDR.LONG = i2sBuffer[i];
	}

	SSIF3.SSICR |= 0b10u; // TEN only. This is a transmit-only link -- see SSIFCR above
}

/// Transmit FIFO fill level, SSIFSR bits 27:24. 0 means the FIFO is empty, 8 means
/// full. Anything in between means the DMA is filling it and the transmitter is
/// draining it -- which is the whole chain working.
uint32_t i2sFifoLevel() {
	return (SSIF3.SSIFSR >> 24) & 0xF;
}

/// The DMA channel's own state, squeezed into one digit so it fits the 7SEG.
///
///   +1  EN    the channel is enabled
///   +2  TACT  a transfer is actually in progress
///   +4  ER    the channel has faulted
///
/// So 3 is a healthy running channel, 1 is enabled but never triggered (the
/// peripheral request is not arriving), 0 is a channel that has finished and
/// stopped (the self-link failed), and anything with 4 in it is an error.
uint32_t i2sDmaState() {
	uint32_t s = DMACn(kI2sDmaChannel).CHSTAT_n;
	uint32_t out = 0;
	if (s & DMAC0_CHSTAT_n_EN) {
		out |= 1;
	}
	if (s & DMAC0_CHSTAT_n_TACT) {
		out |= 2;
	}
	if (s & DMAC0_CHSTAT_n_ER) {
		out |= 4;
	}
	return out;
}

} // namespace

/// One press, one step: PAT1, PAT2, TON1, TON2, then OFF and everything restored.
/// The two PAT steps are checkable with a multimeter alone; the TON steps are for
/// once a DAC is connected.
void i2sAdvance() {
	if (i2sStep >= 0) {
		i2sStop();
	}

	i2sStep++;

	if (i2sStep >= kI2sNumSteps) {
		i2sStep = -1;
		display->setText("OFF");
		return;
	}

	i2sStart(kI2sSteps[i2sStep]);

	// Let the DMA and the transmitter run for a moment, then report the transmit FIFO
	// level as the last character of the label -- "PAT13" means step PAT1, FIFO at 3.
	//
	// This turns the one question that a multimeter cannot answer into a number on
	// the display: 0 means nothing is filling the FIFO, 8 means nothing is emptying
	// it, and anything between means both ends are working.
	delayMS(20);

	char label[8];
	char const* name = kI2sSteps[i2sStep].label;
	int32_t i = 0;
	for (; name[i] != 0 && i < 2; i++) { // 2 chars, leaving room for both digits
		label[i] = name[i];
	}
	label[i++] = (char)('0' + i2sFifoLevel());
	label[i++] = (char)('0' + i2sDmaState());
	label[i] = 0;
	display->setText(label);
}

// ============================================================================
// Gate PWM falsification test  --  SHIFT + CV
// ----------------------------------------------------------------------------
// Gate PWM rests on the gate driver's ~380 ns turn-off delay being something
// firmware can undo. It cannot if the delay depends on anything firmware does not
// know. This asks two narrower questions, with a multimeter on gate 2: does the
// delay depend on how wide the pulse is, and does it depend on the gap before it?
//
// Each step loops one 32-bit frame on the data line through the I2S path proven
// on 2026-08-10, so the DC average is the fraction of the frame the socket was
// actually high. Every pattern sits inside the left 16-bit word and keeps its
// shape when that word is reversed, so the answer does not depend on which order
// SSI3 sends bits in -- which has never been confirmed.
//
//   A k  one pulse of k bits at 22.05 kHz: 1417 ns a bit, 45.35 us a frame
//   B k  one pulse of k bits at 44.1 kHz:   709 ns a bit, 22.68 us a frame
//        A k and B 2k are the same width after a different gap.
//   C g  two 4-bit pulses g bits apart, 22.05 kHz
//   D g  two 4-bit pulses g bits apart, 44.1 kHz
//        The commanded high time is the same at every g, so the reading should
//        not move with g. If it does, the delay depends on the gap before a pulse.
//
// The old plan paired A k with B 2k alone, but every gap in that pairing is at
// least 14 us, and a PWM carrier at 176.4 kHz has gaps of 0 to 5.7 us. C and D
// reach gaps of 1.4 us and 709 ns.
//
// HI and B 16 read above 2 V and come first, on the meter's 20 V range. B 16 is
// the 2026-08-10 PAT1 pattern and should read ~2.36 V again, which proves the
// firmware, the cable and the meter together. Everything from LO on stays under
// 2 V, where the meter reads in 1 mV steps -- ~9 ns at 22.05 kHz, ~4.5 ns at
// 44.1 kHz. The last two steps repeat earlier ones to show drift.
// ============================================================================

namespace {

constexpr uint8_t kPulseCkdv22k = 5; // /32, 705.6 kHz bit clock
constexpr uint8_t kPulseCkdv44k = 4; // /16, 1.4112 MHz bit clock

struct PulseStep {
	uint8_t ckdv;
	uint16_t left;  ///< in send order if SSI3 sends MSB first; symmetric either way
	uint16_t right; ///< zero except for HI
	char const* label;
};

/// One run of k ones at the start of the left word.
constexpr uint16_t pulseRun(int32_t k) {
	return (uint16_t)(0xFFFFu << (16 - k));
}

/// Two runs of four ones with g zeros between them.
constexpr uint16_t pulsePair(int32_t g) {
	return (uint16_t)(0xF000u | (0xF000u >> (4 + g)));
}

constexpr PulseStep kPulseSteps[] = {
    {kPulseCkdv22k, 0xFFFF, 0xFFFF, "HI"},   // 20 V range: true high, ~5.04 V
    {kPulseCkdv44k, 0xFFFF, 0x0000, "B 16"}, // 20 V range: the 2026-08-10 control, ~2.36 V
    {kPulseCkdv22k, 0x0000, 0x0000, "LO"},   // 2 V range from here on
    {kPulseCkdv22k, pulseRun(1), 0, "A 1"},    {kPulseCkdv22k, pulseRun(2), 0, "A 2"},
    {kPulseCkdv22k, pulseRun(3), 0, "A 3"},    {kPulseCkdv22k, pulseRun(4), 0, "A 4"},
    {kPulseCkdv22k, pulseRun(5), 0, "A 5"},    {kPulseCkdv22k, pulseRun(6), 0, "A 6"},
    {kPulseCkdv22k, pulseRun(8), 0, "A 8"},    {kPulseCkdv22k, pulseRun(10), 0, "A 10"},
    {kPulseCkdv22k, pulseRun(12), 0, "A 12"},  {kPulseCkdv44k, pulseRun(1), 0, "B 1"},
    {kPulseCkdv44k, pulseRun(2), 0, "B 2"},    {kPulseCkdv44k, pulseRun(3), 0, "B 3"},
    {kPulseCkdv44k, pulseRun(4), 0, "B 4"},    {kPulseCkdv44k, pulseRun(6), 0, "B 6"},
    {kPulseCkdv44k, pulseRun(8), 0, "B 8"},    {kPulseCkdv44k, pulseRun(10), 0, "B 10"},
    {kPulseCkdv44k, pulseRun(12), 0, "B 12"},  {kPulseCkdv22k, pulsePair(1), 0, "C 1"},
    {kPulseCkdv22k, pulsePair(2), 0, "C 2"},   {kPulseCkdv22k, pulsePair(3), 0, "C 3"},
    {kPulseCkdv22k, pulsePair(4), 0, "C 4"},   {kPulseCkdv22k, pulsePair(6), 0, "C 6"},
    {kPulseCkdv22k, pulsePair(8), 0, "C 8"},   {kPulseCkdv44k, pulsePair(1), 0, "D 1"},
    {kPulseCkdv44k, pulsePair(2), 0, "D 2"},   {kPulseCkdv44k, pulsePair(4), 0, "D 4"},
    {kPulseCkdv44k, pulsePair(8), 0, "D 8"},   {kPulseCkdv22k, pulseRun(4), 0, "A 4"}, // repeat
    {kPulseCkdv22k, pulsePair(1), 0, "C 1"},                                           // repeat
};
constexpr int32_t kPulseNumSteps = sizeof(kPulseSteps) / sizeof(kPulseSteps[0]);

/// -1 means stopped and the pins are back under the gate engine.
int32_t pulseStep = -1;

} // namespace

void pulseTestAdvance() {
	if (pulseStep >= 0) {
		i2sStop();
	}

	pulseStep++;

	if (pulseStep >= kPulseNumSteps) {
		pulseStep = -1;
		i2sPatternWord = i2sFrameWord(-1, 0);
		display->setText("OFF");
		return;
	}

	PulseStep const& step = kPulseSteps[pulseStep];
	i2sPatternWord = i2sFrameWord((int16_t)step.left, (int16_t)step.right);
	i2sStart(I2sStep{step.ckdv, false, step.label});
	display->setText(step.label);
}

/// Plays the phrase once through the given correction setting.
static void cvPlayPhrase(CvEqSetting const& eq) {
	const bool correct = (eq.scale != 0.0f);
	float lastIn = 0.0f;
	float lastOut = 0.0f;
	uint32_t phase = 0;

	for (uint32_t repeat = 0; repeat < kCvMusicRepeats; repeat++) {
		for (int32_t note : kCvMusicNotes) {
			const float freq = kCvMusicRootHz * powf(2.0f, (float)note / 12.0f);
			const uint32_t phaseIncrement = (uint32_t)((freq / (float)kCvTestSampleRate) * 4294967296.0f);
			float envelope = 1.0f;

			for (uint32_t i = 0; i < kCvMusicStepSamples; i++) {
				cvTestNextTick += kCvTestTicksPerSample;
				cvTestWaitUntil(cvTestNextTick);

				phase += phaseIncrement;
				const float saw = (float)phase * (2.0f / 4294967296.0f) - 1.0f;
				const float in = saw * envelope * kCvMusicAmplitude;
				envelope *= kCvMusicEnvDecay;

				float out = in;
				if (correct) {
					out = eq.scale * (in - kCvEqPole * lastIn) + eq.limit * lastOut;
					lastIn = in;
					lastOut = out;
				}

				int32_t sample = (int32_t)out;
				if (sample > kCvMusicClamp) {
					sample = kCvMusicClamp;
				}
				else if (sample < -kCvMusicClamp) {
					sample = -kCvMusicClamp;
				}
				cvEngine.sendVoltageOut(0, (uint16_t)((int32_t)kCvTestCentre + sample));
			}
		}
	}
}

/// Plays the same bassline out of CV 1 four times: uncorrected, then with three
/// increasing amounts of correction. Same level throughout, so only the tone
/// changes. Pick the one that sounds best.
void cvMusicTest() {
	cvTestBegin("CVMU");
	cvTestRamp(true);

	for (CvEqSetting const& eq : kCvEqSettings) {
		display->setText(eq.name);
		cvPlayPhrase(eq);
		cvTestEmitTone(0, 0, kCvTestSampleRate / 2); // half a second between passes
	}

	cvTestRamp(false);
	display->setText("DONE");
}

// ============================================================================
// CV audio outputs
// ----------------------------------------------------------------------------
// Streams two independent tracks out of the CV sockets while the main outputs
// carry the usual mix.
//
// The DAC's chip-select sits on P6_1, which is the SPI block's own SSL00 pin and
// is already configured as such at boot. That lets the transfer engine feed the
// DAC continuously with no processor involvement. Words alternate between the
// DAC's two channels, so one buffer serves both sockets.
//
// Each socket's analogue path is a single-pole low-pass with its corner near
// 700 Hz, so each channel gets the inverse applied before it leaves.
// ============================================================================

extern "C" void v7_dma_flush_range(uint32_t start, uint32_t end);

#define CV_STREAM_DMA_CHANNEL 5 // unassigned on every model

void cvStreamStart();
void cvStreamStop();

namespace {

/// Words in the streamed buffer. Alternating, so half belong to each socket.
/// The ring has to hold the target lead *plus* one whole burst, because the pump writes a
/// window's worth of frames in one go. At 512 words that was 256 frames against a 128-frame
/// lead and bursts of up to ~136 -- no margin at all, and a full-size engine window could
/// lap the reader. The engine doubles its window size under load, so the bursts get bigger
/// exactly when the machine is busiest. 2048 words is 1024 frames, which leaves the burst
/// six times the room it needs. Costs 8 KB of SDRAM and no latency: see kCvTargetLead.
constexpr uint32_t kCvStreamWords = 4096;
constexpr uint32_t kCvFramesPerChannel = kCvStreamWords / 2;
constexpr uint32_t kCvMaxWindow = 256;

/// How far ahead of the DMA read pointer the pump tries to stay, in frames. This *is* the
/// CV output's latency behind the main outputs, and it is also how late the engine may be
/// before the buffer runs dry.
///
/// This has to exceed the worst phase excursion the engine produces, or the buffer runs dry
/// and the DMA reads frames that were never written. Measured 2026-08-13: idle the lead
/// swings ~57 frames, loaded it swings ~650. Simulation against that jitter puts the
/// minimum lead near 100 frames with a 768-frame target, and underruns with 512.
///
/// **This is the CV output's latency behind the main outputs: 768 frames is ~16.4 ms.**
/// Fine for a send into a pedal, a mixer channel or a modular case, which is what these
/// sockets are for; summing a send back against the same source in the mains will
/// comb-filter. Lower it if the resync count stays at zero on real songs -- it is the one
/// constant here with a cost the player can hear.
constexpr int32_t kCvTargetLead = 768;


PLACE_SDRAM_DATA uint32_t cvStreamBuffer[kCvStreamWords] __attribute__((aligned(CACHE_LINE_SIZE)));
PLACE_SDRAM_DATA int32_t cvSourceMono[2][kCvMaxWindow] __attribute__((aligned(CACHE_LINE_SIZE)));
PLACE_SDRAM_DATA int32_t cvCaptureScratch[kCvMaxWindow * 2] __attribute__((aligned(CACHE_LINE_SIZE)));

const uint32_t cvStreamDmaLinkDescriptor[] __attribute__((aligned(CACHE_LINE_SIZE))) = {
    0b1101,                                                                             // Header
    (uint32_t)cvStreamBuffer,                                                           // Source
    (uint32_t)&RSPI(SPI_CHANNEL_CV).SPDR.LONG,                                          // Destination
    sizeof(cvStreamBuffer),                                                             // Transaction size
    0b10000001001000100010001000101000 | DMA_LVL_FOR_SSI | (CV_STREAM_DMA_CHANNEL & 7), // Config
    0,                                                                                  // Interval
    0,                                                                                  // Extension
    (uint32_t)cvStreamDmaLinkDescriptor                                                 // Next link: itself
};

bool cvStereoSplitGlobal = false;
bool cvStreamRunning = false;
uint32_t cvStreamWriteFrame = 0;
bool cvSourceValid[2] = {false, false};
float cvFeedLastIn[2] = {0.0f, 0.0f};
float cvFeedLastOut[2] = {0.0f, 0.0f};

/// Which Clips fed each socket. Accumulated while the outputs render, then
/// compared with the previous render's set: per-clip routing means the set can
/// change mid-performance, and carrying filter state across that change clicks.
uint32_t cvSourceSignatureAccum[2] = {0, 0};
uint32_t cvSourceSignatureLive[2] = {0, 0};
bool cvAnyRoutedAccum = false;
bool cvSplitAccum = false;
bool cvSplitLive = false;

/// Treble correction, expressed as the one number that actually decides it: how hard we
/// invert the socket's own 700 Hz low-pass.
///
/// The shelf is  out = scale*(in - pole*lastIn) + limit*lastOut.  At DC (z=1) its gain is
/// scale*(1-pole)/(1-limit); at Nyquist (z=-1) it is scale*(1+pole)/(1+limit). Their ratio
/// is the boost, and it depends on `limit` alone:
///
///     boost = K * (1-limit)/(1+limit),   K = (1+pole)/(1-pole)
///
/// so  limit = (K-boost)/(K+boost), and `scale` is then fixed by requiring unity gain at DC.
/// That unity-at-DC constraint is what makes this a tone control rather than a volume one:
/// the bass sits exactly where it would with no correction at all, and only the treble moves.
///
/// K itself is full inversion -- flat to 20 kHz -- and is unreachable, because the output
/// stage compresses above about a third of its swing and every dB of boost is a dB of
/// headroom spent. The chosen value is therefore a listening call, not a calculation.
///
/// **x6.87, chosen by ear 2026-08-15**, flat to roughly 4.8 kHz. It was picked from a 1-9
/// menu control built for that one session and removed again afterwards: a tone knob is a
/// question asked of a user who has no way to answer it, and the whole point of correcting
/// a known filter is that there is a right answer. If user feedback ever disagrees, the
/// control is a small revert away and this comment is the reason it went.
///
/// Supersedes the x3.125 tuned by ear on 2026-08-04. That earlier session tried x6 and
/// rejected it as harsh -- correctly at the time, because the resampler then in use put
/// frequency-modulation sidebands right where the boost lands (-25 dB by 5 kHz against
/// -51 dB at 260 Hz). The resampler rewrite dropped that floor by roughly 24 dB, and the
/// setting that had been unusable became the preferred one. **The verdict changed because
/// the signal path changed, not because the ear did.**
constexpr float kCvEqK = (1.0f + kCvEqPole) / (1.0f - kCvEqPole); // 20.03
constexpr float kCvEqBoost = 6.866f;
constexpr float kCvStreamEqLimit = (kCvEqK - kCvEqBoost) / (kCvEqK + kCvEqBoost);
constexpr float kCvStreamEqScale = (1.0f - kCvStreamEqLimit) / (1.0f - kCvEqPole);

constexpr float kCvFeedBaseScale = 0.098f;
/// Display 50 is x256, and every step below it is a fixed 1.2 dB rather than a fixed
/// multiplier, so the steps sound evenly spaced the whole way down and reach roughly 59 dB
/// down by 1. Display 0 is a true mute rather than the bottom of the taper.
///
/// The scale moved from 1-100 at 0.6 dB to 0-50 at 1.2 dB when this became a param. Coarser by
/// a factor of two, and taken deliberately: 0-50 is what every other param on the machine
/// displays, so a master recorded into an automation lane now reads the same numbers the menu
/// shows. The alternative kept the finer steps and made the two disagree.
constexpr float kCvLevelTopGain = 256.0f;
constexpr float kCvLevelDbPerStep = 1.2f;

float cvLevelToScale(int32_t display) {
	if (display <= 0) {
		return 0.0f;
	}
	if (display > kCvMasterDisplayMax) {
		display = kCvMasterDisplayMax;
	}
	const float decibels = (float)(display - kCvMasterDisplayMax) * kCvLevelDbPerStep;
	return kCvFeedBaseScale * kCvLevelTopGain * powf(10.0f, decibels / 20.0f);
}

float cvFeedScale[2] = {cvLevelToScale(40), cvLevelToScale(40)};
int32_t cvPrevSample[2] = {0, 0};

/// Output samples per input sample. SPBR=9 gives a 3.333 MHz bit clock, and the
/// note's measurement of ~35.5 bit-times per frame puts the stream at ~46.95 kHz
/// per socket against the engine's 44.1 kHz. That is only the starting estimate --
/// the loop below trims it to whatever the hardware actually does.
constexpr float kCvNominalRate = 46948.0f / 44100.0f;

/// How far the ratio may stray from nominal. Both clocks come off the same crystal, so the
/// true ratio is fixed and the only slack needed is for the estimate above being slightly
/// wrong -- the loop finds the real value within this band and holds it.
///
/// This used to be 0.90 to 1.25, which let a control loop transpose the audio by roughly a
/// musical third. Measured 2026-08-13: under load it sat *on* both of those clamps, which
/// was the "transposing" symptom. Nothing legitimate ever needs that range. If the resync
/// counter climbs steadily on a quiet song, this band is too narrow rather than too wide --
/// the true rate is then outside it and the loop cannot reach it.
/// Measured 2026-08-13 on an idle machine: the loop settles around 1064, which is the
/// nominal above -- so the estimate is good and the band only has to cover the residual.
/// +-0.3% is about 10 cents worst case, at the very edge of audible on a sustained tone,
/// against roughly 70 cents at the old +-2%.
constexpr float kCvRateSpan = 0.003f;
constexpr float kCvRateMin = kCvNominalRate * (1.0f - kCvRateSpan);
constexpr float kCvRateMax = kCvNominalRate * (1.0f + kCvRateSpan);

/// A rate trim can only fix a *rate* error. A phase error -- the buffer running empty or
/// overfull because the engine arrived late -- can only be nursed back over thousands of
/// windows, during which the integrator winds up and hits a clamp. Past this much error,
/// snap the write pointer back instead: one discontinuity, instantly recovered, rather than
/// continuous pitch movement. Measured 2026-08-13: without this the lead swung across the
/// entire ring, 0 to 1023.
/// Sized so that ordinary jitter never reaches it and only a real stall does: the lead may
/// wander between 128 and 1408 frames before this fires, inside a 2048-frame ring.
constexpr int32_t kCvResyncThreshold = 640;

/// The lead error is smoothed before it reaches the integrator, with a time constant of
/// roughly three seconds at the ~345 Hz window rate.
///
/// This is the fix. The engine renders in irregular windows, so the lead jitters by
/// hundreds of frames under load -- and that is *phase* jitter, which a rate trim cannot
/// correct and must not chase. Feeding the raw per-window error to the integrator made the
/// loop bang-bang between its clamps, measured 2026-08-13. Averaging leaves only genuine
/// rate drift, which is what a rate trim is actually for.
constexpr float kCvLeadAvgAlpha = 0.001f;
float cvLeadAvg = 0.0f;

/// Integrator, applied to the *averaged* error. Small enough that a full band traverse takes
/// seconds rather than the ten windows the old gain took. Measured 2026-08-13: at 1e-7 the
/// ratio swept the whole band in about a second, which is audible as a slow warble even
/// though the depth was only ~10 cents. Depth was fixed by the clamps; this fixes the speed.
constexpr float kCvRateTrackGain = 0.000000003f;

/// Proportional gain, on the SMOOTHED error.
///
/// A rate error integrates into lead, and this loop integrates lead error into rate: two
/// integrators in series, which is 180 degrees of phase lag and oscillates however small the
/// gain is made. Lowering the gain and adding a deadband only changed the period, which is
/// what every build on 2026-08-13 did. A double integrator needs damping, and that is what a
/// proportional term is.
///
/// There *was* a proportional term originally, fed the raw per-window error, which injected
/// jitter straight into the ratio -- so removing it was right, and putting it back on the
/// smoothed error is what it should always have been.
///
/// Sized so a 100-frame averaged error moves the ratio by ~0.03%, correcting that much lead
/// in about a second.
constexpr float kCvRateDampGain = 0.000003f;

// There is deliberately no proportional term any more. It applied the raw per-window error
// straight to the emitted rate, so a few hundred frames of jitter moved the ratio by ~0.8%
// -- about 14 cents -- injected from exactly the noise this loop is supposed to reject.

float cvRate = kCvNominalRate;
/// Resampling phase, carried across windows. See cvStreamPump.
float cvResamplePos = 0.0f;

constexpr uint16_t kCvCentre = 32768;
constexpr int32_t kCvClamp = 12000;

/// Command word the DAC expects: write-and-update, given channel, 16-bit value.
inline uint32_t cvWord(uint32_t channel, int32_t sample) {
	if (sample > kCvClamp) {
		sample = kCvClamp;
	}
	else if (sample < -kCvClamp) {
		sample = -kCvClamp;
	}
	const uint16_t voltage = (uint16_t)((int32_t)kCvCentre + sample);
	return ((uint32_t)(0b00110000 | (1u << channel)) << 24) | ((uint32_t)voltage << 8);
}

} // namespace

bool cvOutputsAvailable() {
	return !deluge::hid::display::have_oled_screen;
}

bool cvStreamIsRunning() {
	return cvStreamRunning;
}

namespace {

/// Adds one Clip's contribution to a socket. Several Clips can share a socket, so
/// the first one this window sets the buffer and the rest add into it.
/// `channel` picks what the socket gets: 0 for left, 1 for right, 2 for the mono sum.
///
/// The Clip's contribution is `post` minus `pre` -- the mix after it rendered minus the
/// snapshot taken before. That subtraction happens here, per sample, rather than in a
/// pass of its own beforehand: this loop already touches every sample it needs, so a
/// separate pass was a second walk over the same data for nothing.
///
/// The mono sum halves each side before adding, so a centred Clip comes out at the
/// level of one channel. A single channel is therefore taken at full scale, not
/// halved -- otherwise turning STEREO SPLIT on would drop a centred Clip by 6 dB.
///
/// `gainFrom` and `gainTo` are Q16 -- kCvSendGainUnity is unity, 0 is silence. The gain
/// ramps across the window rather than stepping at its edge: the window is only ~128
/// samples (~345 Hz), so a step per window is plainly audible as zipper noise while a
/// send is being ridden. At unity throughout there is no multiply at all, which keeps the
/// common case exactly as cheap as it was before sends existed -- this loop runs with all
/// interrupts disabled.
void cvAccumulateInto(uint32_t socket, uint32_t channel, const int32_t* post, const int32_t* pre, uint32_t numSamples,
                      int32_t gainFrom, int32_t gainTo) {
	const bool first = !cvSourceValid[socket];
	int32_t* const dest = cvSourceMono[socket];
	const bool unity = (gainFrom == kCvSendGainUnity) && (gainTo == kCvSendGainUnity);

	// Q16.16 accumulator so the per-sample step needs no divide inside the loop. 64-bit
	// because unity is 65536, and 65536 << 16 does not fit in a signed 32-bit.
	int64_t gain = (int64_t)gainFrom << 16;
	const int64_t gainStep = (((int64_t)(gainTo - gainFrom)) << 16) / (int32_t)numSamples;

	for (uint32_t i = 0; i < numSamples; i++) {
		int32_t sample;
		if (channel == 2) {
			// Shift each side before adding, exactly as the two-pass version did: the
			// difference was taken at full width and only halved here, so halving the two
			// differences separately keeps every rounding step where it already was.
			const int32_t diffL = post[i * 2] - pre[i * 2];
			const int32_t diffR = post[i * 2 + 1] - pre[i * 2 + 1];
			sample = (diffL >> 1) + (diffR >> 1);
		}
		else {
			sample = post[i * 2 + channel] - pre[i * 2 + channel];
		}
		if (!unity) {
			sample = (int32_t)(((int64_t)sample * (int32_t)(gain >> 16)) >> 16);
			gain += gainStep;
		}
		if (first) {
			dest[i] = sample;
		}
		else {
			dest[i] += sample;
		}
	}
	cvSourceValid[socket] = true;
}

} // namespace

/// Called from the per-output render loop with one Clip's isolated contribution.
/// With STEREO SPLIT off both sockets get the mono sum, so the same signal can go
/// to both; with it on CV1 gets left and CV2 gets right.
int32_t cvSendParamToGain(int32_t paramValue) {
	// Param space is the full signed range; the bottom is silence and the top is unity.
	const uint32_t position = (uint32_t)paramValue ^ 0x80000000u;
	const uint32_t linear = position >> 16; // 0 .. 65535
	if (linear >= 65535) {
		return kCvSendGainUnity;
	}
	// Unsigned deliberately: 65534 squared is 4.29e9, which overflows a signed 32-bit.
	return (int32_t)((linear * linear) >> 16);
}

/// `sendGain` is this Clip's two send amounts for this window, Q16. `lastSendGain` is the
/// same pair from the previous window, owned by the Clip and updated here -- it is what the
/// ramp starts from, so it has to persist between windows and be per Clip rather than per
/// socket, since several Clips can feed one socket at different amounts.
void cvStreamCapture(uint32_t sourceId, const int32_t* post, const int32_t* pre, uint32_t numSamples,
                     const int32_t* sendGain, int32_t* lastSendGain) {
	if (numSamples == 0 || numSamples > kCvMaxWindow) {
		return;
	}

	// Global now, not a Clip bit. The old per-Clip bit is still parsed from song files so
	// that a song saved before this change loads without complaint -- it just no longer
	// decides anything, because two cables cannot be a stereo pair for one clip and two
	// mono outs for another at the same time.
	const bool split = cvStereoSplitGlobal;
	// Mixed in rather than summed raw so that two Clips swapping sockets, or one
	// Clip changing its split mode, both register as a change.
	const uint32_t stamp = (sourceId ^ (split ? 0x5bd1e995u : 0u)) * 2654435761u;

	for (uint32_t socket = 0; socket < 2; socket++) {
		const int32_t gainTo = sendGain[socket];
		const int32_t gainFrom = lastSendGain[socket];
		lastSendGain[socket] = gainTo;

		// A send off at both ends of the window is simply not routed to this socket, and
		// must not stamp -- otherwise a send falling to zero never changes the socket's
		// signature and the correction filter is never reset.
		if (gainFrom == 0 && gainTo == 0) {
			continue;
		}

		cvAnyRoutedAccum = true;
		cvSplitAccum |= split;
		cvAccumulateInto(socket, split ? socket : 2, post, pre, numSamples, gainFrom, gainTo);
		cvSourceSignatureAccum[socket] += stamp + socket;
	}
}

/// Called once at the end of the render loop, when the full set of Clips feeding
/// each socket for this window is known. Resets the correction filter for any
/// socket whose sources changed, and starts or stops the stream to match.
void cvStreamRenderComplete() {
	for (uint32_t socket = 0; socket < 2; socket++) {
		if (cvSourceSignatureAccum[socket] != cvSourceSignatureLive[socket]) {
			cvSourceSignatureLive[socket] = cvSourceSignatureAccum[socket];
			cvFeedLastIn[socket] = 0.0f;
			cvFeedLastOut[socket] = 0.0f;
			cvPrevSample[socket] = 0;
		}
		cvSourceSignatureAccum[socket] = 0;
	}

	if (cvAnyRoutedAccum && !cvStreamRunning) {
		cvStreamStart();
	}
	else if (!cvAnyRoutedAccum && cvStreamRunning) {
		cvStreamStop();
	}
	cvAnyRoutedAccum = false;

	cvSplitLive = cvSplitAccum;
	cvSplitAccum = false;
}

bool cvStreamNeedsStereo() {
	return cvSplitLive;
}

bool cvGetStereoSplit() {
	return cvStereoSplitGlobal;
}

void cvSetStereoSplit(bool on) {
	cvStereoSplitGlobal = on;
}

int32_t cvMasterParamToDisplay(int32_t paramValue) {
	// Param space is the full signed range. Rounded rather than truncated so that the value
	// written back from a display number reads back as the same number -- truncating makes the
	// menu appear to drop a step every time you leave it and come back.
	const uint32_t position = (uint32_t)paramValue ^ 0x80000000u;
	return (int32_t)(((uint64_t)position * kCvMasterDisplayMax + 0x80000000ull) >> 32);
}

int32_t cvMasterDisplayToParam(int32_t display) {
	if (display <= 0) {
		return -2147483648;
	}
	if (display >= kCvMasterDisplayMax) {
		return 2147483647;
	}
	// Land in the middle of the display step's band, so a round trip through
	// cvMasterParamToDisplay returns what went in rather than sitting on a boundary. The
	// rounding term is half the *display* range, not half of param space -- getting that wrong
	// put steps 12 and 37 one out, which a simulation caught and no amount of listening would.
	const uint64_t position =
	    ((uint64_t)display * 4294967296ull + (uint64_t)(kCvMasterDisplayMax / 2)) / (uint64_t)kCvMasterDisplayMax;
	return (int32_t)((uint32_t)position ^ 0x80000000u);
}

uint8_t cvMasterKnobSocket = 0;

void cvSetMasterFromParams(int32_t cv1Value, int32_t cv2Value) {
	cvFeedScale[0] = cvLevelToScale(cvMasterParamToDisplay(cv1Value));
	// With the pair patched as one stereo destination there is one master, not two: the
	// sockets carry left and right of the same thing, so CV1's value drives both. This is the
	// same rule the sends follow, and it is why the second knob is idle when split is on.
	cvFeedScale[1] = cvLevelToScale(cvMasterParamToDisplay(cvStereoSplitGlobal ? cv1Value : cv2Value));
}

int32_t* cvStreamCaptureScratch() {
	return cvCaptureScratch;
}

/// Called once per audio window. Resamples both captured tracks to whatever rate
/// the transfer engine is running at, applies the correction, and interleaves
/// them into the buffer being streamed. Staying half a buffer ahead of the read
/// position is what keeps the two rates matched without knowing the ratio.
void cvStreamPump(uint32_t numSamples) {
	if (!cvStreamRunning || numSamples == 0 || numSamples > kCvMaxWindow) {
		cvSourceValid[0] = cvSourceValid[1] = false;
		return;
	}

	const uint32_t readFrame =
	    ((DMACn(CV_STREAM_DMA_CHANNEL).CRSA_n - (uint32_t)cvStreamBuffer) >> 3) & (kCvFramesPerChannel - 1);
	const uint32_t lead = (cvStreamWriteFrame - readFrame) & (kCvFramesPerChannel - 1);
	// Deliberately NOT half the ring: the lead sets latency, while the ring only has to be
	// big enough that a burst cannot lap the reader. Tying the two together would have made
	// every buffer enlargement cost delay for no reason.
	constexpr int32_t targetLead = kCvTargetLead;
	int32_t leadError = targetLead - (int32_t)lead;

	// Phase resync, for a genuine stall only. A rate trim cannot fix a phase error in any
	// useful time, so past the threshold the write pointer is simply put where it belongs.
	// Costs one discontinuity.
	//
	// The rate estimate is deliberately KEPT. Only the phase is broken here; the ratio is a
	// property of two crystals and is still correct, and throwing it away made the loop
	// re-acquire from nominal every time this fired.
	if (leadError > kCvResyncThreshold || leadError < -kCvResyncThreshold) {
		cvStreamWriteFrame = ((uint32_t)readFrame + (uint32_t)targetLead) & (kCvFramesPerChannel - 1);
		cvResamplePos = 0.0f;
		for (uint32_t socket = 0; socket < 2; socket++) {
			cvFeedLastIn[socket] = 0.0f;
			cvFeedLastOut[socket] = 0.0f;
			cvPrevSample[socket] = 0;
		}
		leadError = 0;
	}

	// Both clocks come off the same crystal, so the true ratio between them is a
	// constant -- there is nothing to chase, only something to converge on. The
	// integrator finds it and then stops moving; the proportional term only damps
	// acquisition, and is tiny so it does not move the ratio once locked.
	cvLeadAvg += ((float)leadError - cvLeadAvg) * kCvLeadAvgAlpha;

	// PI on the smoothed error. The integrator alone is a second integrator in series with
	// the one the buffer already provides, and oscillates; the proportional term damps it.
	cvRate += cvLeadAvg * kCvRateTrackGain;
	if (cvRate < kCvRateMin) {
		cvRate = kCvRateMin;
	}
	else if (cvRate > kCvRateMax) {
		cvRate = kCvRateMax;
	}
	float rateNow = cvRate + cvLeadAvg * kCvRateDampGain;
	if (rateNow < kCvRateMin) {
		rateNow = kCvRateMin;
	}
	else if (rateNow > kCvRateMax) {
		rateNow = kCvRateMax;
	}

	// Input samples consumed per output sample. Held constant and the phase carried
	// across windows, rather than restarting at zero each window and stretching that
	// window's input to fit a whole number of outputs.
	//
	// That difference is the whole point. Emitting a whole number of samples per
	// window forces the ratio to alternate between neighbouring integers -- about
	// 0.73% either way -- and a resampling ratio that alternates is frequency
	// modulation at the window rate. The sidebands it produces grow with frequency:
	// about -51 dB at 260 Hz, but only -25 dB by 5 kHz. That is why a sine sounded
	// clean while anything with strong harmonics sounded grainy. Carrying the phase
	// keeps the instantaneous ratio fixed and lets the sample count per window vary
	// instead, which costs nothing and takes the jitter to about 0.07%.
	const float step = 1.0f / rateNow;

	const float startPos = cvResamplePos;
	int32_t toEmit = 0;
	{
		float p = startPos;
		while (p < (float)numSamples && toEmit < (int32_t)kCvMaxWindow) {
			p += step;
			toEmit++;
		}
		if (toEmit < 1 || toEmit >= (int32_t)kCvMaxWindow) {
			toEmit = (toEmit < 1) ? 1 : (int32_t)kCvMaxWindow;
			cvResamplePos = 0.0f; // resync rather than carry a position we did not reach
		}
		else {
			cvResamplePos = p - (float)numSamples;
		}
	}

	for (uint32_t socket = 0; socket < 2; socket++) {
		// Folded into the scale rather than shifted off the integer first. Shifting
		// first quantised the signal to steps of `scale` DAC counts -- at the tuned
		// level that is a step of about 6 counts in a working range of 12000, so
		// roughly 11 bits of the converter's resolution instead of 13.5. Audible as
		// grit on quiet material, which is exactly where it hurts most.
		const float scale = cvFeedScale[socket] * (1.0f / 32768.0f);
		const int32_t* const source = cvSourceMono[socket];
		const bool valid = cvSourceValid[socket];
		float position = startPos;
		uint32_t frame = cvStreamWriteFrame;

		for (int32_t i = 0; i < toEmit; i++) {
			float in = 0.0f;
			if (valid) {
				uint32_t index = (uint32_t)position;
				if (index >= numSamples) {
					index = numSamples - 1;
				}
				// Interpolated rather than nearest-neighbour. 44.1 kHz into a stream
				// running near 47 kHz is a ratio no whole number of samples fits, so
				// picking the nearest one holds each value for an irregular length of
				// time. That irregularity is a modulation of the signal, and it lands
				// as inharmonic grit right across the band -- much louder than the
				// converter's own noise. The first output of a window interpolates
				// from the last sample of the previous one, hence cvPrevSample.
				const float frac = position - (float)index;
				const float a = (float)((index == 0) ? cvPrevSample[socket] : source[index - 1]);
				const float b = (float)source[index];
				in = (a + (b - a) * frac) * scale;
			}
			position += step;

			const float out =
			    kCvStreamEqScale * (in - kCvEqPole * cvFeedLastIn[socket]) + kCvStreamEqLimit * cvFeedLastOut[socket];
			cvFeedLastIn[socket] = in;
			cvFeedLastOut[socket] = out;

			cvStreamBuffer[frame * 2 + socket] = cvWord(socket, (int32_t)out);
			frame = (frame + 1) & (kCvFramesPerChannel - 1);
		}

		// Carried so the next window can interpolate across the join rather than
		// stepping. Cleared when a socket falls silent, so a stale sample can't
		// reappear when it comes back.
		cvPrevSample[socket] = valid ? source[numSamples - 1] : 0;
	}

	cvStreamWriteFrame = (cvStreamWriteFrame + (uint32_t)toEmit) & (kCvFramesPerChannel - 1);
	cvSourceValid[0] = cvSourceValid[1] = false;

	v7_dma_flush_range((uint32_t)cvStreamBuffer, (uint32_t)cvStreamBuffer + sizeof(cvStreamBuffer));
}

void cvStreamStart() {
	// The DAC shares this SPI channel with the display on OLED models, where boot
	// hands the chip-select to software and screen data is already using the bus.
	// Nothing here would work, so don't start.
	if (deluge::hid::display::have_oled_screen) {
		return;
	}

	for (uint32_t frame = 0; frame < kCvFramesPerChannel; frame++) {
		cvStreamBuffer[frame * 2] = cvWord(0, 0);
		cvStreamBuffer[frame * 2 + 1] = cvWord(1, 0);
	}
	v7_dma_flush_range((uint32_t)cvStreamBuffer, (uint32_t)cvStreamBuffer + sizeof(cvStreamBuffer));

	// Boot already sets 32-bit frames, master mode, transmit requests and the
	// hardware chip-select. Change only the clock -- twice as fast as for one
	// channel, since every sample now costs two frames.
	RSPI(SPI_CHANNEL_CV).SPCR &= ~(1 << 6);
	RSPI(SPI_CHANNEL_CV).SPBR = 9; // ~3.3 MHz -> ~47 kHz per socket
	RSPI(SPI_CHANNEL_CV).SPCR |= (1 << 1); // transmit only
	RSPI(SPI_CHANNEL_CV).SPCR |= (1 << 6);

	initDMAWithLinkDescriptor(CV_STREAM_DMA_CHANNEL, cvStreamDmaLinkDescriptor, DMARS_FOR_RSPI_TX);
	dmaChannelStart(CV_STREAM_DMA_CHANNEL);

	// Start already at the target lead rather than at zero. The buffer was just filled with
	// the centre value, so the DMA reads silence until the writer catches up -- and the loop
	// begins with no error to correct instead of a full-scale one.
	//
	// Starting at zero meant every stream start was an acquisition transient that drove the
	// ratio hard onto one clamp and overshot onto the other. That is inaudible in itself,
	// but it is also what made two rounds of diagnostic readings meaningless, because the
	// extremes recorded the transient rather than the running state.
	cvStreamWriteFrame = (uint32_t)kCvTargetLead & (kCvFramesPerChannel - 1);
	cvFeedLastIn[0] = cvFeedLastIn[1] = 0.0f;
	cvFeedLastOut[0] = cvFeedLastOut[1] = 0.0f;
	cvPrevSample[0] = cvPrevSample[1] = 0;
	cvRate = kCvNominalRate;
	cvLeadAvg = 0.0f;
	cvResamplePos = 0.0f;
	cvStreamRunning = true;
}

void cvStreamStop() {
	// Stop the transfer engine before touching the SPI block, or it keeps writing
	// into a register we are reconfiguring.
	DMACn(CV_STREAM_DMA_CHANNEL).CHCTRL_n |= DMAC0_CHCTRL_n_CLREN;

	// Put the SPI back exactly as boot left it, rather than clearing it: the CV
	// sockets go back to being note-voltage outputs when nothing is routed here,
	// and that path uses this same channel.
	RSPI(SPI_CHANNEL_CV).SPCR &= ~(1 << 6);  // disable while reconfiguring
	RSPI(SPI_CHANNEL_CV).SPBR = 1;           // boot's rate for the 30 MHz request
	RSPI(SPI_CHANNEL_CV).SPCR &= ~(1 << 1);  // full duplex again, not transmit-only
	RSPI(SPI_CHANNEL_CV).SPCR |= (1 << 6);   // re-enable

	cvStreamRunning = false;
}
