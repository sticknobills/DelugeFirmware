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

// Let's just keep these two pipes as not the same as the send-pipe for USB MIDI hosting (PIPE1)
//
// PIPE4 rather than PIPE2 since the audio return exists: only PIPE1 and PIPE2 can carry isochronous
// transfers (usb_pstd_chk_pipe_info, r_usb_pdriver.c), and audio needs both - one each way. Bulk is
// legal on PIPE1 through PIPE5, so MIDI moves aside at no cost to itself. Nothing outside this file
// names a MIDI pipe by number; every use is through these two constants.
#define USB_CFG_PMIDI_BULK_IN (USB_PIPE4)
#define USB_CFG_PMIDI_BULK_OUT (USB_PIPE3)
