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
#define USB_CFG_PAUDIO_CHANNELS (8u)

// 8 x 16 bit is 16 bytes per audio frame. 44.1 kHz gives 44 or 45 frames per 1 ms packet, so the
// largest packet is 45 * 16 = 720 bytes. Pipe buffers are allocated in 64-byte units, so 768 is
// the smallest that fits, and it is left at that size when the channel count is varied for a
// test so that only the packet size moves.
#define USB_CFG_PAUDIO_BUF_BYTES (768u)

// First 64-byte buffer block. MIDI holds blocks 8-15 and 72-79; double buffering doubles the
// allocation, so this occupies 16-39.
#define USB_CFG_PAUDIO_BUF_START (16u)
