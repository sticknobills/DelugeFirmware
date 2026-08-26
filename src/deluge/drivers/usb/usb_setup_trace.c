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

#include "deluge/drivers/usb/usb_setup_trace.h"

/* Power of two so the wrap is a mask. Enumeration alone is a couple of dozen requests, and only the ones around a
 * stream start matter, so a full buffer drops the newest rather than stalling the interrupt. */
#define TRACE_SIZE 64u
#define TRACE_MASK (TRACE_SIZE - 1u)

static volatile UsbSetupTraceEntry entries[TRACE_SIZE];
static volatile uint16_t writeIndex = 0;
static volatile uint16_t readIndex = 0;
static volatile uint16_t sequence = 0;

volatile uint32_t usbBempNonZeroCount = 0;
volatile uint32_t usbBempAudioCount = 0;
volatile uint32_t usbBempBitsSeen = 0;
volatile uint32_t usbBempAudioStall = 0;
volatile uint32_t usbBempAudioDeferred = 0;
volatile uint32_t usbAudioWriteEnd[4] = {0, 0, 0, 0};
volatile uint32_t usbAudioForcedTerm = 0;
volatile uint32_t usbBrdyNonZeroCount = 0;
volatile uint32_t usbBrdyAudioCount = 0;

static void push(uint16_t type, uint16_t value, uint16_t index, uint16_t length, uint16_t isMark) {
	uint16_t next = (uint16_t)((writeIndex + 1u) & TRACE_MASK);
	sequence++;
	if (next == readIndex) {
		return; /* Full. The gap in sequence numbers is what tells the reader. */
	}
	entries[writeIndex].type = type;
	entries[writeIndex].value = value;
	entries[writeIndex].index = index;
	entries[writeIndex].length = length;
	entries[writeIndex].sequence = sequence;
	entries[writeIndex].isMark = isMark;
	writeIndex = next;
}

void usbSetupTraceRecord(uint16_t type, uint16_t value, uint16_t index, uint16_t length) {
	push(type, value, index, length, 0u);
}

void usbSetupTraceMark(uint16_t tag, uint16_t a, uint16_t b, uint16_t c) {
	push(tag, a, b, c, 1u);
}

int usbSetupTracePop(UsbSetupTraceEntry* out) {
	if (readIndex == writeIndex) {
		return 0;
	}
	out->type = entries[readIndex].type;
	out->value = entries[readIndex].value;
	out->index = entries[readIndex].index;
	out->length = entries[readIndex].length;
	out->sequence = entries[readIndex].sequence;
	out->isMark = entries[readIndex].isMark;
	readIndex = (uint16_t)((readIndex + 1u) & TRACE_MASK);
	return 1;
}
