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

/* Marker tags, for following the SET_INTERFACE path that a host stream start dies on. */
#define USB_TRACE_MARK_SETIF_PIPES_BEGIN 0x01u /* about to configure pipes for the new alternate setting */
#define USB_TRACE_MARK_SETIF_PIPES_END 0x02u   /* they all came back */
#define USB_TRACE_MARK_PIPE_INIT_BEGIN 0x03u   /* one pipe, with the PIPEMAXP and PIPECFG about to be written */
#define USB_TRACE_MARK_PIPE_INIT_END 0x04u     /* that pipe came back */

typedef struct {
	uint16_t type; /* Request: low byte bmRequestType, high byte bRequest. Marker: the tag. */
	uint16_t value;
	uint16_t index;
	uint16_t length;
	uint16_t sequence; /* Wraps. Non-consecutive numbers on arrival mean entries were dropped. */
	uint16_t isMark;   /* 1 for a code-path marker, 0 for a control request off the wire. */
} UsbSetupTraceEntry;

/// Called from the USB interrupt for every control request. Drops the entry if the buffer is full.
void usbSetupTraceRecord(uint16_t type, uint16_t value, uint16_t index, uint16_t length);

/// Records a point in the driver rather than a request off the wire, so a path that never returns can be told apart
/// from one that returns and achieves nothing. Same buffer, so markers interleave with requests in order.
void usbSetupTraceMark(uint16_t tag, uint16_t a, uint16_t b, uint16_t c);

/// Pops one entry. Returns 0 when empty.
int usbSetupTracePop(UsbSetupTraceEntry* out);

#ifdef __cplusplus
}
#endif
