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

#include <stdint.h>

/* Diagnostic only, for bringing up USB Audio Class. Records every control request the host sends so the reason a
 * host refuses to start the audio stream can be read off the wire rather than guessed at from the driver source.
 * Recording happens in the USB interrupt and must stay trivial; formatting and sending happen from the main loop. */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint16_t type; /* USBREQ: low byte bmRequestType, high byte bRequest */
	uint16_t value;
	uint16_t index;
	uint16_t length;
	uint16_t sequence; /* Wraps. Non-consecutive numbers on arrival mean entries were dropped. */
} UsbSetupTraceEntry;

/// Called from the USB interrupt for every control request. Drops the entry if the buffer is full.
void usbSetupTraceRecord(uint16_t type, uint16_t value, uint16_t index, uint16_t length);

/// Pops one entry. Returns 0 when empty.
int usbSetupTracePop(UsbSetupTraceEntry* out);

#ifdef __cplusplus
}
#endif
