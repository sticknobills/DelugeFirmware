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

// The largest packet the outgoing endpoint may carry, in audio frames.
//
// This is a *bandwidth reservation*, not a buffer size: a Full Speed frame reserves at most
// 1350 bytes for all timed audio traffic in both directions combined, and the host reserves
// against what an endpoint declares rather than what it sends. Declared at the endpoint's
// theoretical ceiling - 63 frames, 1008 bytes - the pair of directions came to about 1218 bytes
// with per-packet overhead, over ninety percent of the allowance before the host's own margin.
//
// 46 frames is 736 bytes, which takes the pair to about 955. The transmit path sends 44 or 45,
// so nominal traffic is unaffected; what it costs is the catch-up burst, which wanted 53 and is
// now clamped to 46, so recovery from a stall takes longer.
//
// One constant, because the descriptor and the packet builder must not disagree about this. They
// did: the descriptor declared 63 frames and the builder capped itself at 62.
#define USB_CFG_PAUDIO_MAX_FRAMES (46u)

// 11 x 16 bit is 22 bytes per audio frame, so the largest packet is 45 * 22 = 990 bytes. Pipe
// buffers are allocated in 64-byte units, so 1024 is the smallest that fits.
#define USB_CFG_PAUDIO_BUF_BYTES (1024u)

// First 64-byte buffer block.
//
// Block 24, not 16, since 2026-09-06, and the arithmetic is the whole reason. USB_BUF_SIZE(x) encodes
// x/64 blocks and double buffering doubles what a pipe consumes, so:
//
//   MIDI in    512 bytes at block 8  -> 8 blocks doubled  -> blocks 8-23
//   this pipe  1024 bytes at block 16 -> 16 blocks doubled -> blocks 16-47
//
// which put MIDI's second buffer plane wholly inside this pipe's first. MIDI only advances to its second
// plane when a packet arrives before the previous one has been read, which is why an occasional message was
// destroyed rather than every other one: a DAW's clock at 120 BPM lost three to six messages a second, the
// Deluge read the resulting double-length gaps as the tempo halving, and following an external clock while a
// host captured audio was unusable. Measured 2026-09-06: three to six packets a second arriving as four zero
// bytes, zero with the stream closed.
//
// The overlap was written down here as an apparent one and steered clear of rather than resolved. It was
// real. Nothing had ever measured MIDI arriving while audio streamed.
//
// 24 gives this pipe blocks 24-55, clear of MIDI at 8-23 and 72-87, with the return moved to 56-63 below.
#define USB_CFG_PAUDIO_BUF_START (24u)

/* ---- The return, host to device ---- */

// How many channels the return carries. The receive path and the descriptors both derive from this,
// so it is the only place it is written down.
//
// Two is the product shape: a master return into the song's summing point needs one stereo pair.
// One, four and six exist because the per-channel price of this direction has never been measured -
// the note asserted the cost was fixed per packet without ever testing it, and whether sixteen
// channels costs like two or like sixteen decides whether a per-track insert is possible at all.
// Four builds, four points, rather than a slope drawn through one.
//
// Six is the ceiling at Full Speed and it is the periodic-bandwidth budget rather than anything
// local: a frame reserves ~1350 bytes for all timed traffic in both directions, the outgoing
// endpoint declares 736, and six channels of return declare 540 - 1302 with per-packet overhead.
// Eight channels would declare 720 and put the pair over.
#define USB_CFG_PAUDIO_RX_CHANNELS (1u)

// The largest packet the return endpoint may carry, in audio frames.
//
// 45, not 48, since 2026-09-07, and this is a bandwidth reservation rather than a buffer size - the
// host reserves against what an endpoint declares, which is the same lesson the outgoing endpoint
// learned on 2026-09-05. 45 is what a 44.1 kHz host can actually send in a 1 ms frame; the three
// spare frames were free at two channels and cost 72 bytes of the frame's budget at six, which is
// the difference between the wide arms enumerating and not.
//
// A host that rounds differently and sends 46 has its packet dropped by the hardware. The check on
// that is the collection rate: the two-channel arm has to reproduce ~44,100 frames a second, and a
// host doing anything else shows in the packet-size buckets on the return's own report line before
// it shows anywhere else.
#define USB_CFG_PAUDIO_RX_MAX_FRAMES (45u)
#define USB_CFG_PAUDIO_RX_PACKET_BYTES (USB_CFG_PAUDIO_RX_CHANNELS * 2u * USB_CFG_PAUDIO_RX_MAX_FRAMES)

// Sized for six channels in every build, not for the channel count this one carries.
//
// 6 * 2 * 45 is 540 bytes, rounded up to the 64-byte unit pipe buffers are allocated in. Deliberately
// the same at one, two, four and six channels so the buffer map is identical across the four builds
// and the only thing that differs between them is the channel count. A per-arm buffer would move the
// pipe's block allocation between arms, which is a second variable in a measurement whose whole point
// is a slope.
#define USB_CFG_PAUDIO_RX_BUF_BYTES (576u)

// Blocks 88-105 once double-buffered: 576 bytes is 9 blocks, doubled is 18.
//
// Moved up from 56 on 2026-09-07, because the six-channel buffer above no longer fits between the
// outgoing pipe and USB MIDI's outgoing one. The full map, all four pipes with double buffering
// counted rather than assumed:
//
//   MIDI in    8-23    audio out  24-55    MIDI out  72-87    return  88-105
//
// 56-71 and 106-127 are spare. The block number is an 8-bit field valid from 4 to 127 (hardware
// manual p28-58), so 105 is comfortably inside it. Anything added here counts its own doubling and
// checks it against this list - the one place this was left as "apparent" cost a day's work and
// shipped a fault.
#define USB_CFG_PAUDIO_RX_BUF_START (88u)

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
