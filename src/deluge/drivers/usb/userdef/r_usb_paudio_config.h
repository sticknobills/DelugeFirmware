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

// USB audio (UAC1) as a peripheral: one stereo stream each way, alongside USB MIDI.
// "Out" and "in" are from the host's point of view, as in the USB spec - so the out stream is audio coming into the
// Deluge from the computer, and the in stream is audio the Deluge sends to the computer.

// Only pipes 1 and 2 can do isochronous transfers
#define USB_CFG_PAUDIO_ISO_OUT (USB_PIPE1)
#define USB_CFG_PAUDIO_ISO_IN (USB_PIPE2)

#define USB_PAUDIO_EP_OUT 3 // Endpoint numbers. MIDI has 1 (in) and 2 (out)
#define USB_PAUDIO_EP_IN 4

#define USB_PAUDIO_INTERFACE_CONTROL 0
#define USB_PAUDIO_INTERFACE_STREAM_OUT 1
#define USB_PAUDIO_INTERFACE_STREAM_IN 2
#define USB_PMIDI_INTERFACE 3
#define USB_PAUDIO_NUM_INTERFACES 4

#define USB_PAUDIO_SAMPLE_RATE 44100
#define USB_PAUDIO_NUM_CHANNELS 2
// 24-bit samples, each in a 4-byte subframe and MSB-aligned - which makes them the same as the Deluge's q31 samples,
// and keeps every packet a whole number of 32-bit words for the FIFO DMA
#define USB_PAUDIO_SUBFRAME_BYTES 4
#define USB_PAUDIO_BIT_RESOLUTION 24
#define USB_PAUDIO_BYTES_PER_FRAME (USB_PAUDIO_NUM_CHANNELS * USB_PAUDIO_SUBFRAME_BYTES)
// 44.1 frames per 1ms frame on average. We never send more than 46 (see framesForNextPacket()), and allow the host up
// to 48
#define USB_PAUDIO_MAX_FRAMES_PER_PACKET 48
#define USB_PAUDIO_MAX_PACKET_BYTES (USB_PAUDIO_MAX_FRAMES_PER_PACKET * USB_PAUDIO_BYTES_PER_FRAME)
// The out stream's adaptive alternate setting (for hosts without implicit feedback, like Windows) gets a slightly
// smaller max packet size - still plenty for a host sending 44 or 45 frames at its own rate. Hosts choosing between
// alternate settings of the same format by bandwidth then prefer it, if they can't use the asynchronous one
#define USB_PAUDIO_ADAPTIVE_MAX_FRAMES_PER_PACKET 46
#define USB_PAUDIO_ADAPTIVE_MAX_PACKET_BYTES (USB_PAUDIO_ADAPTIVE_MAX_FRAMES_PER_PACKET * USB_PAUDIO_BYTES_PER_FRAME)

// Which USB configuration the Deluge offers, chosen in SETTINGS > USB AUDIO > MODE and fixed until USB next starts
#define USB_PAUDIO_MODE_OFF 0        // USB MIDI only
#define USB_PAUDIO_MODE_STEREO 1     // USB MIDI, and one stereo stream each way
#define USB_PAUDIO_MODE_MULTITRACK 2 // USB MIDI, eight channels to the host and four from it
#define USB_PAUDIO_NUM_MODES 3

// Multitrack: the same endpoints, interfaces and clocking as stereo, with more channels at 16 bits.
//
// What limits it is the host's bandwidth reservation, made from each endpoint's wMaxPacketSize. Behind a USB 2.0 hub or
// a USB 2.0/3 host's built-in one - which every current Mac is - a Full Speed device's isochronous traffic is scheduled
// in 188-byte microframe slots, at most six in a frame. Both streams have to fit together, since implicit feedback
// opens them as a pair: 736 bytes (4 slots) and 368 (2) do. At the 48-frame maximum stereo uses, 768 (5) and 384 (3)
// don't, and macOS refuses to start either stream. So multitrack declares 46 frames each way, which is the most we
// ever send (framesForNextPacket()) - and so, under implicit feedback, the most a host sends us. At 24 bits the to-host
// stream alone wouldn't fit.
#define USB_PAUDIO_MT_CHANNELS_TO_HOST 8
#define USB_PAUDIO_MT_CHANNELS_FROM_HOST 4
#define USB_PAUDIO_MT_SUBFRAME_BYTES 2
#define USB_PAUDIO_MT_BIT_RESOLUTION 16
#define USB_PAUDIO_MT_MAX_FRAMES_PER_PACKET 46
#define USB_PAUDIO_MT_TO_HOST_BYTES_PER_FRAME (USB_PAUDIO_MT_CHANNELS_TO_HOST * USB_PAUDIO_MT_SUBFRAME_BYTES)
#define USB_PAUDIO_MT_FROM_HOST_BYTES_PER_FRAME (USB_PAUDIO_MT_CHANNELS_FROM_HOST * USB_PAUDIO_MT_SUBFRAME_BYTES)
#define USB_PAUDIO_MT_TO_HOST_MAX_PACKET_BYTES                                                                         \
	(USB_PAUDIO_MT_MAX_FRAMES_PER_PACKET * USB_PAUDIO_MT_TO_HOST_BYTES_PER_FRAME)
#define USB_PAUDIO_MT_FROM_HOST_MAX_PACKET_BYTES                                                                       \
	(USB_PAUDIO_MT_MAX_FRAMES_PER_PACKET * USB_PAUDIO_MT_FROM_HOST_BYTES_PER_FRAME)
// The to-host pipe's buffer, in whole 64-byte blocks: 736 bytes needs 12
#define USB_PAUDIO_MT_TO_HOST_BUFFER_BYTES (768)
#if USB_PAUDIO_MT_TO_HOST_BUFFER_BYTES < USB_PAUDIO_MT_TO_HOST_MAX_PACKET_BYTES
#error "the to-host pipe's buffer must hold its largest packet"
#endif
// Four 16-bit channels make the same 8-byte frame as two 24-bit ones, so the from-host stream's packet sizes, pipe
// buffer and DMA are the same in both modes
#if USB_PAUDIO_MT_FROM_HOST_BYTES_PER_FRAME != USB_PAUDIO_BYTES_PER_FRAME
#error "the from-host stream relies on both modes having the same frame size"
#endif
