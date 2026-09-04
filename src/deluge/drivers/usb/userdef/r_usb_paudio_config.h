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

/* Direction naming, because USB's own is host-relative and reads backwards here.
 *
 *   ISO_IN  - the IN endpoint. Audio leaving the Deluge, arriving at the host. Stage A.
 *   ISO_OUT - the OUT endpoint. Audio leaving the host, arriving at the Deluge. Stage B.
 *
 * A host calls our outgoing stems its "inputs". The names below are the endpoint's, not the
 * user's. */

// Only PIPE1 and PIPE2 can carry isochronous transfers - see usb_pstd_chk_pipe_info() in
// r_usb_pdriver.c. Both are now spoken for, which is also why no feedback endpoint is possible
// at Full Speed: a feedback endpoint is itself isochronous and there is no third pipe for it.
#define USB_CFG_PAUDIO_ISO_IN (USB_PIPE1)

// Freed by moving USB MIDI's bulk-IN to PIPE4 - see r_usb_pmidi_config.h. Bulk may live on
// pipes 1-5, isochronous only on 1-2, so the pipe that can be moved is the one that moves.
#define USB_CFG_PAUDIO_ISO_OUT (USB_PIPE2)

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
// Eight, not eleven, since 2026-08-27, and the reason is packet arithmetic rather than the ceiling. A packet
// holds 1023 / (channels * 2) whole audio frames, so the transmit path must land 44100 / that many writes a
// second: 959/s at eleven channels, 700/s at eight. It measures 921/s under load. Eleven cannot keep up and
// eight has 30% to spare. Eleven remains the target; raising this back is one constant plus the write rate
// to sustain it.
#define USB_CFG_PAUDIO_CHANNELS (8u)

// 11 x 16 bit is 22 bytes per audio frame, so the largest packet is 45 * 22 = 990 bytes. Pipe
// buffers are allocated in 64-byte units, so 1024 is the smallest that fits.
#define USB_CFG_PAUDIO_BUF_BYTES (1024u)

// First 64-byte buffer block. MIDI holds blocks 8-15 and 72-79; double buffering doubles the
// allocation, so 1024 bytes occupies 16-47, clear of both.
#define USB_CFG_PAUDIO_BUF_START (16u)

/* ---- The return, host to device ---- */

// Two channels: Siphon sums its effect lanes and hands back one stereo pair, and a master return
// into the song's summing point needs no more. The receive path is written channel-addressable, so
// widening this is a constant plus a wider pipe buffer - the ceiling is six, above which the two
// directions no longer fit in a Full Speed frame together.
#define USB_CFG_PAUDIO_RX_CHANNELS (2u)

// 2 x 16 bit is 4 bytes per audio frame. 44.1 kHz does not divide into 1 ms frames, so a host may
// send 44 or 45 frames; 48 frames of headroom costs nothing and cannot be undersized by a host that
// rounds differently.
#define USB_CFG_PAUDIO_RX_MAX_FRAMES (48u)
#define USB_CFG_PAUDIO_RX_PACKET_BYTES (USB_CFG_PAUDIO_RX_CHANNELS * 2u * USB_CFG_PAUDIO_RX_MAX_FRAMES)

// Rounded up to the 64-byte unit pipe buffers are allocated in.
#define USB_CFG_PAUDIO_RX_BUF_BYTES (256u)

// Blocks 48-55 once double-buffered. Re-derived at the moment of use rather than taken from a note:
// the live allocation is MIDI at 8 and 72 (512 bytes each) and the outgoing audio pipe at 16 (1024
// bytes), so 48-71 is unclaimed under every reading of how double buffering consumes blocks. That
// caveat is deliberate - the existing map has an apparent overlap between MIDI's IN pipe and the
// outgoing audio pipe if double buffering doubles the block count, and it has run for weeks without
// trouble. Unresolved, recorded, and steered well clear of rather than relied upon.
#define USB_CFG_PAUDIO_RX_BUF_START (48u)

// Called from the peripheral interrupt handler's frame branch, 1000 times a second, to write the
// next audio packet off the host's own clock. Declared here because that handler already includes
// this file and is otherwise vendored C.
#ifdef __cplusplus
extern "C" {
#endif
void usbAudioStreamStartOfFrame(void);
#ifdef __cplusplus
}
#endif
