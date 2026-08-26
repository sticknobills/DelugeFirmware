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

#pragma once

// Only PIPE1 and PIPE2 can carry isochronous transfers - see usb_pstd_chk_pipe_info() in
// r_usb_pdriver.c. PIPE2 and PIPE3 belong to USB MIDI, so PIPE1 is the only one available.
#define USB_CFG_PAUDIO_ISO_IN (USB_PIPE1)

// Channel count for the stream. The descriptors and the transmit path both derive their packet
// arithmetic from this, so it is the only place it is written down.
//
// Eleven is the ceiling at Full Speed, and it is the USB specification's rather than this
// chip's: one isochronous endpoint may carry at most 1023 bytes per 1 ms frame. 44.1 kHz does
// not divide into 1 ms frames, so a packet has to be sized for 45 audio frames rather than 44,
// which puts 11 channels at 990 bytes and 12 at 1080. Nothing local binds first - PIPEMAXP is
// an 11-bit field and pipe buffers reach 2048 bytes. Only PIPE1 and PIPE2 can carry isochronous
// transfers here and PIPE2 is USB MIDI, so a second endpoint cannot be added to widen this.
// High Speed is what lifts it, and that is a separate piece of work.
//
// DIAGNOSTIC, 2026-08-26: temporarily 2 rather than 11, taking the packet from 990 bytes to 180
// and changing nothing else - the buffer allocation below is deliberately left at 1024 so packet
// size is the only variable. Delivery sits at exactly one packet every two frames with the pipe
// confirmed correctly configured and double-buffered, and whether a fifth-size packet lifts that
// separates "a 990-byte packet cannot be refilled in time" from "only one packet is ever handed
// over at a time". Restore to 11u once the answer is in.
#define USB_CFG_PAUDIO_CHANNELS (2u)

// 11 x 16 bit is 22 bytes per audio frame, so the largest packet is 45 * 22 = 990 bytes. Pipe
// buffers are allocated in 64-byte units, so 1024 is the smallest that fits.
#define USB_CFG_PAUDIO_BUF_BYTES (1024u)

// First 64-byte buffer block. MIDI holds blocks 8-15 and 72-79; double buffering doubles the
// allocation, so 1024 bytes occupies 16-47, clear of both.
#define USB_CFG_PAUDIO_BUF_START (16u)
