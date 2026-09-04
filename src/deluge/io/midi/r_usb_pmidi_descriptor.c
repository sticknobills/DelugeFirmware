/*
 * Copyright © 2018-2023 Synthstrom Audible Limited
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

/*
 * Portions of this code initially copied from the relevant sample code by Renesas.
 *
 * These global config definitions are used in USBConnection.c
 *
 * The connect functions in USBConnection.c are called around line 800 of deluge_main
 */

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/
#include "RZA1/usb/r_usb_basic/r_usb_basic_if.h"

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
#define USB_BCDNUM (0x0200u)  /* bcdUSB */
#define USB_RELEASE (0x0200u) /* Release Number */
#define USB_CONFIGNUM (1u)    /* Configuration number */
#define USB_DCPMAXP (64u)     /* DCP max packet size */

// Get VID from
// http://www.mcselec.com/index.php?page=shop.product_details&flypage=shop.flypage&product_id=92&category_id=20&option=com_phpshop&Itemid=1
#define USB_VENDORID (0x16D0)  /* Vendor ID */
#define USB_PRODUCTID (0x0CE2) /* Product ID */

// size of the USB configuration, including all interfaces
/* 9 - config description
 * 9 - interface description
 * 7 - MIDI streaming header
 * (6+9)*ncables - in and out jack per cable
 * (9+4+ncables)*2 - shared bulk endpoint, descriptors include list of cables
 * for easy c+p -> 9+9+7+((6+9)*ncables)+(9+4+ncables)*2
 */
#include "deluge/drivers/usb/userdef/r_usb_paudio_config.h"

#define NCABLES 3
#define USB_MIDI_CD_WTOTALLENGTH (9 + 7 + (15 * NCABLES) + (13 + NCABLES) * 2)

/* Audio class descriptors, added alongside USB MIDI rather than around it. USB MIDI is
 * formally an audio class subclass, so a conforming device would collect both streaming
 * interfaces under one AudioControl interface. The MIDI interface here predates that and is
 * left exactly as it is: rewriting a descriptor that has enumerated correctly for years, to
 * satisfy a rule no host enforces, risks a shipped feature for nothing.
 *
 * There are two AudioStreaming interfaces, one per direction, collected under a single
 * AudioControl interface. Each has its own terminal pair, because a terminal is one-directional:
 * audio out is "inside the Deluge" -> "the USB stream", and audio in is the reverse.
 *
 * 9  - AudioControl interface
 * 11 - class-specific AC header, three interfaces in its collection (8 + one byte each)
 * 12 - input terminal, out direction: the Deluge's own audio
 * 9  - output terminal, out direction: the USB stream
 * 12 - input terminal, in direction: the USB stream
 * 9  - output terminal, in direction: where returning audio lands
 *
 * and then, twice over - once per direction:
 * 9  - AudioStreaming interface, alt 0, no endpoint
 * 9  - AudioStreaming interface, alt 1
 * 7  - class-specific AS general
 * 11 - type I format, one discrete sample rate
 * 9  - isochronous endpoint
 * 7  - class-specific isochronous endpoint
 */
#define USB_AUDIO_AC_WTOTALLENGTH (11 + 12 + 9 + 12 + 9)
#define USB_AUDIO_AS_BLOCK_LENGTH (9 + 9 + 7 + 11 + 9 + 7)
#define USB_AUDIO_CD_WTOTALLENGTH (9 + USB_AUDIO_AC_WTOTALLENGTH + (2 * USB_AUDIO_AS_BLOCK_LENGTH))

// 9 for config descriptor. Add any additional config lengths here
#define TOTAL_CONFIG_LENGTH (9 + USB_MIDI_CD_WTOTALLENGTH + USB_AUDIO_CD_WTOTALLENGTH)
// Good summary ref on overall USB structure https://www.beyondlogic.org/usbnutshell/usb5.shtml

// USB midi defines
#define CS_INTERFACE 0x24
#define CS_ENDPOINT 0x25
#define MIDI_IN_JACK 0x02
#define MIDI_OUT_JACK 0x03

// USB audio class 1.0 defines - ref https://www.usb.org/sites/default/files/audio10.pdf
#define AUDIO_SUBCLASS_CONTROL 0x01
#define AUDIO_SUBCLASS_STREAMING 0x02
#define AUDIO_AC_HEADER 0x01
#define AUDIO_AC_INPUT_TERMINAL 0x02
#define AUDIO_AC_OUTPUT_TERMINAL 0x03
#define AUDIO_AS_GENERAL 0x01
#define AUDIO_AS_FORMAT_TYPE 0x02
#define AUDIO_EP_GENERAL 0x01
#define AUDIO_FORMAT_TYPE_I 0x01

// There is no terminal type for "the inside of a synth". 0x0201 is the one every host
// reliably presents as a capture device.
#define AUDIO_TERMINAL_INPUT_TYPE 0x0201
#define AUDIO_TERMINAL_USB_STREAMING 0x0101
#define AUDIO_IN_TERMINAL_ID 0x01
#define AUDIO_OUT_TERMINAL_ID 0x02

// The return's terminal pair. 0x0301 is "speaker" - there is no terminal type for "back into the
// song's mix", and speaker is the one every host reliably presents as a playback destination.
#define AUDIO_TERMINAL_OUTPUT_TYPE 0x0301
#define AUDIO_RX_IN_TERMINAL_ID 0x03
#define AUDIO_RX_OUT_TERMINAL_ID 0x04

// Interface numbers. MIDI is 0 and predates all of this.
#define MIDI_STREAMING_INTERFACE 0x00
#define AUDIO_CONTROL_INTERFACE 0x01
#define AUDIO_STREAM_OUT_INTERFACE 0x02
#define AUDIO_STREAM_IN_INTERFACE 0x03

#define AUDIO_RX_CHANNELS USB_CFG_PAUDIO_RX_CHANNELS
#define AUDIO_RX_MAX_PACKET_SIZE USB_CFG_PAUDIO_RX_PACKET_BYTES

// Isochronous, adaptive. The device cannot be asynchronous in this direction: a conforming
// asynchronous sink signals its rate back with a feedback endpoint, which is itself isochronous,
// and both isochronous-capable pipes are already carrying audio. Adaptive says the device absorbs
// the mismatch, which it does with an elastic buffer - see usb_audio_stream.cpp.
#define AUDIO_RX_EP_ATTRIBUTES 0x09

#define AUDIO_NUM_CHANNELS USB_CFG_PAUDIO_CHANNELS
#define AUDIO_SUBFRAME_BYTES 2
#define AUDIO_BIT_RESOLUTION 16
#define AUDIO_SAMPLE_RATE 44100

// The Deluge runs on its own clock, so the endpoint is asynchronous and the host adapts to it.
// A device-to-host stream signals its rate by varying packet size, so no feedback endpoint.
#define AUDIO_EP_ATTRIBUTES 0x05

// A Full Speed isochronous endpoint carries at most 1023 bytes per frame, so the endpoint is sized
// for as many whole audio frames as fit - 46 at eleven channels, not the 45 the sample rate implies.
// Break-even is 44100 / frames-per-packet writes a second, so every extra frame the packet can hold
// lowers the write rate the transmit path has to sustain: 980/s at 45 frames, 959/s at 46.
#define AUDIO_FRAME_BYTES (AUDIO_NUM_CHANNELS * AUDIO_SUBFRAME_BYTES)
#define AUDIO_ISO_LIMIT_BYTES 1023
#define AUDIO_MAX_PACKET_SIZE ((AUDIO_ISO_LIMIT_BYTES / AUDIO_FRAME_BYTES) * AUDIO_FRAME_BYTES)
/***********************************************************************************************************************
Exported global variables (to be accessed by other files)
***********************************************************************************************************************/
/* Standard Device Descriptor
 * Top level USB descriptor
 * Used by host to enumerate device - e.g. what is this
 * bytes 4-7 declares that the device is specified at the interface level
 * bytes 8-16 declare that it's a synthstrom deluge (presumably)
 * byte 17 says how many configurations are available - e.g. it could
 * offer both an FS and HS configuration and allow the host to choose,
 * but in this case we offer a single USB midi FS config
 */
uint8_t g_midi_device[USB_DD_BLENGTH + (USB_DD_BLENGTH % 2)] = {
    USB_DD_BLENGTH,                                        /*  0:bLength */
    USB_DT_DEVICE,                                         /*  1:bDescriptorType */
    (uint8_t)(USB_BCDNUM&(uint8_t)0xff),                   /*  2:bcdUSB_lo */
    (uint8_t)((uint8_t)(USB_BCDNUM >> 8) & (uint8_t)0xff), /*  3:bcdUSB_hi */
                                                           // Device to be specified at interface level
    0x00,                                                  /*  4:bDeviceClass */
    0x00,                                                  /*  5:bDeviceSubClass */
    0x00,                                                  /*  6:bDeviceProtocol */

    (uint8_t)USB_DCPMAXP,                                     /*  7:bMAXPacketSize(for DCP) */
    (uint8_t)(USB_VENDORID&(uint8_t)0xff),                    /*  8:idVendor_lo */
    (uint8_t)((uint8_t)(USB_VENDORID >> 8) & (uint8_t)0xff),  /*  9:idVendor_hi */
    (uint8_t)(USB_PRODUCTID&(uint8_t)0xff),                   /* 10:idProduct_lo */
    (uint8_t)((uint8_t)(USB_PRODUCTID >> 8) & (uint8_t)0xff), /* 11:idProduct_hi */
    (uint8_t)(USB_RELEASE&(uint8_t)0xff),                     /* 12:bcdDevice_lo */
    (uint8_t)((uint8_t)(USB_RELEASE >> 8) & (uint8_t)0xff),   /* 13:bcdDevice_hi */
    1,                                                        /* 14:iManufacturer */
    2,                                                        /* 15:iProduct */
    0,                                                        /* 16:iSerialNumber */
    USB_CONFIGNUM,                                            /* 17:bNumConfigurations */
};

/************************************************************
 * Configuration Or Other_Speed_Configuration Descriptor     *
 ************************************************************/

/* USB Configuration description - USB spec 9.6.3
 * 2nd level of USB declaration - defines power and number of interfaces
 * This config specifies that it is configuration 1 and has a single interface
 * To add usb audio we would add a second interface under this configuration
 */
uint8_t g_midi_configuration[TOTAL_CONFIG_LENGTH + (TOTAL_CONFIG_LENGTH % 2)] = {
    USB_CD_BLENGTH,                       /*  0:bLength */
    USB_DT_CONFIGURATION,                 /*  1:bDescriptorType */
    (uint8_t)(TOTAL_CONFIG_LENGTH % 256), /*  2:wTotalLength(L) */
    (uint8_t)(TOTAL_CONFIG_LENGTH / 256), /*  3:wTotalLength(H) */
    4,                                    /*  4:bNumInterfaces */
    1,                                    /*  5:bConfigurationValue */
    0,                                    /*  6:iConfiguration */
    (uint8_t)(USB_CF_RESERVED),           /*  7:bmAttributes */
    (uint8_t)(500 / 2),                   /*  8:bMaxPower (2mA unit) */

    /* Interface Descriptor
     * 3rd level of USB declarations. This declares a single USB midi interface with 2 endpoints (subclass of audio)
     */
    USB_ID_BLENGTH,   /*  0:bLength */
    USB_DT_INTERFACE, /*  1:bDescriptorType */
    0,                /*  2:bInterfaceNumber */
    0,                /*  3:bAlternateSetting */
    2,                /*  4:bNumEndpoints */
    USB_IFCLS_AUD,    /*  5:bInterfaceClass(AUD) */
    0x03,             /*  6:bInterfaceSubClass(MIDI) */
    0,                /*  7:bInterfaceProtocol */
    0,                /*  8:iInterface */

    /* Midi Streaming Interface descriptors
     * A level below interface, specific to USB midi
     * ref - https://www.usb.org/sites/default/files/midi10.pdf sect 6.1.2
     * to add ports/cables add more jacks here
     */
    // Header
    0x07,                        // header length
    CS_INTERFACE,                // bDescriptorType - CS interface
    0x01,                        // Subtype - Midi Streaming Header
    0x00, 0x01,                  // BCD revision (1.00)
    (uint8_t)(7 + 15 * NCABLES), // TotalLength - LSB of interface descriptors - 7+15*ncables
    0x00,                        // Interface descriptors MSB
    // MIDI_IN 1
    0x06,         // bLength
    CS_INTERFACE, // bDescriptorType
    MIDI_IN_JACK, // bDescriptorSubtype - MIDI_IN_JACK
    0x01,         // bJackType - EMBEDDED
    0x01,         // bJackID - 1
    0x00,         // iJack (unused)

    // MIDI_OUT 1
    0x09,          // bLength
    CS_INTERFACE,  // bDescriptorType - CS_I
    MIDI_OUT_JACK, // bDescriptorSubtype - MIDI_OUT_JACK
    0x01,          // bJackType - EMBEDDED
    0x02,          // bJackID - 2
    0x01,          // bNrInputPins (I can't find an explanation for what this means)
    0x01,          // BaSourceID (ditto here but I think it is asking which midi in jack is associated?)
    0x01,          // BaSourcePin (ditto)
    0x00,          // iJack (unused)

    // MIDI_IN 2
    0x06,         // bLength
    CS_INTERFACE, // bDescriptorType
    0x02,         // bDescriptorSubtype - MIDI_IN_JACK
    0x01,         // bJackType - EMBEDDED
    0x03,         // bJackID
    0x00,         // iJack (unused)

    // MIDI_OUT 2
    0x09,          // bLength
    CS_INTERFACE,  // bDescriptorType - CS_I
    MIDI_OUT_JACK, // bDescriptorSubtype - MIDI_OUT_JACK
    0x01,          // bJackType - EMBEDDED
    0x04,          // bJackID
    0x01,          // bNrInputPins (I can't find an explanation for what this means)
    0x02,          // BaSourceID (ditto here but I think it is asking which midi in jack is associated?)
    0x01,          // BaSourcePin (ditto)
    0x00,          // iJack (unused)

    // MIDI_IN 3
    0x06,         // bLength
    CS_INTERFACE, // bDescriptorType
    MIDI_IN_JACK, // bDescriptorSubtype - MIDI_IN_JACK
    0x01,         // bJackType - EMBEDDED
    0x05,         // bJackID
    0x00,         // iJack (unused)

    // MIDI_OUT 3
    0x09,          // bLength
    CS_INTERFACE,  // bDescriptorType - CS_I
    MIDI_OUT_JACK, // bDescriptorSubtype - MIDI_OUT_JACK
    0x01,          // bJackType - EMBEDDED
    0x06,          // bJackID
    0x01,          // bNrInputPins (I can't find an explanation for what this means)
    0x05,          // BaSourceID (ditto here but I think it is asking which midi in jack is associated?)
    0x01,          // BaSourcePin (ditto)
    0x00,          // iJack (unused)

    /* MidiStreaming Endpoint Descriptors - USBMidi spec 6.2.1
     * These endpoints are shared across all jacks
     */
    // 28 bytes for bulk endpoints
    // USB standard bulk out -
    0x09,                            // bLength
    0x05,                            // bDescriptorType = ENDPOINT
    (uint8_t)(USB_EP_OUT | USB_EP2), // bEndpointAddress. D7 direction, low 4 addr
    0x02,                            // bmAttributes (bulk)
    0x40, 0x00,                      // wMaxPacketSize
    0x00,                            // bInterval
    0x00,                            // bRefresh
    0x00,                            // bSynchAddress
                                     // midi class specific bulk out
    (uint8_t)(4 + NCABLES),          // bLength - 4+ncables
    0x25,                            // bDescriptorType - CS_ENDPOINT
    0x01,                            // bDescriptorSubType - MS_GENERAL
    0x03,                            // bNumEmbMidiJack - number of MIDI IN jacks
    0x01,                            // BaAssocJackID - ID of first associated jack
    0x03,
    0x05, // ID of last associated jack

    // USB standard bulk in - same fields as above, differences annotated
    0x09,                           // bLength
    0x05,                           // bDescriptor
    (uint8_t)(USB_EP_IN | USB_EP1), // different address
    0x02,                           // bmAttributes
    0x40, 0x00,                     // wMaxPacketSize
    0x00,                           // bInterval
    0x00,                           // bRefresh
    0x00,                           // bSynchAddress
                                    // midi specific bulk in
    (uint8_t)(4 + NCABLES),         // bLength - 4+ncables
    CS_ENDPOINT,                    // bDescriptorType - CS_ENDPOINT
    0x01,                           // bDescriptorSubtype
    0x03,                           // bNumEmbMidiJack - number of MIDI OUT jacks
    0x02,                           // BaAssocJackID - first associated jack
    0x04,
    0x06, // Last associated jack

    /* AudioControl Interface - USB audio 1.0 spec 4.3
     * Owns no endpoints. It exists to describe what the streaming interface below is
     * connected to: audio enters at a terminal inside the Deluge and leaves over USB.
     */
    0x09,                   // bLength
    USB_DT_INTERFACE,       // bDescriptorType
    0x01,                   // bInterfaceNumber - MIDI is 0
    0x00,                   // bAlternateSetting
    0x00,                   // bNumEndpoints
    USB_IFCLS_AUD,          // bInterfaceClass
    AUDIO_SUBCLASS_CONTROL, // bInterfaceSubClass
    0x00,                   // bInterfaceProtocol
    0x00,                   // iInterface

    /* Class-specific AC header. Collects every audio-class interface in this configuration - both
     * AudioStreaming interfaces below and USB MIDI, which is an audio-class interface too
     * (subclass MIDIStreaming) and has to belong to an AudioControl interface.
     *
     * MIDI was left out when this was written, because before the return there was no
     * AudioControl interface at all and MIDI had stood alone for years. Adding one and not
     * collecting MIDI into it left the configuration describing an audio-class interface that
     * belongs to nothing. */
    0x0B,                                       // bLength - 8 + one byte per interface collected
    CS_INTERFACE,                               // bDescriptorType
    AUDIO_AC_HEADER,                            // bDescriptorSubtype
    0x00, 0x01,                                 // bcdADC - 1.00
    (uint8_t)(USB_AUDIO_AC_WTOTALLENGTH % 256), // wTotalLength(L) - this header plus all four terminals
    (uint8_t)(USB_AUDIO_AC_WTOTALLENGTH / 256), // wTotalLength(H)
    0x03,                                       // bInCollection
    MIDI_STREAMING_INTERFACE,                   // baInterfaceNr(1) - USB MIDI
    AUDIO_STREAM_OUT_INTERFACE,                 // baInterfaceNr(2) - audio leaving the Deluge
    AUDIO_STREAM_IN_INTERFACE,                  // baInterfaceNr(3) - audio returning to it

    // Input terminal - where the audio comes from, as far as the host is concerned
    0x0C,                                       // bLength
    CS_INTERFACE,                               // bDescriptorType
    AUDIO_AC_INPUT_TERMINAL,                    // bDescriptorSubtype
    AUDIO_IN_TERMINAL_ID,                       // bTerminalID
    (uint8_t)(AUDIO_TERMINAL_INPUT_TYPE % 256), // wTerminalType(L)
    (uint8_t)(AUDIO_TERMINAL_INPUT_TYPE / 256), // wTerminalType(H)
    0x00,                                       // bAssocTerminal - none
    AUDIO_NUM_CHANNELS,                         // bNrChannels
    0x00, 0x00,                                 // wChannelConfig - discrete, no spatial positions
    0x00,                                       // iChannelNames
    0x00,                                       // iTerminal

    // Output terminal - the USB stream, fed by the input terminal above
    0x09,                                          // bLength
    CS_INTERFACE,                                  // bDescriptorType
    AUDIO_AC_OUTPUT_TERMINAL,                      // bDescriptorSubtype
    AUDIO_OUT_TERMINAL_ID,                         // bTerminalID
    (uint8_t)(AUDIO_TERMINAL_USB_STREAMING % 256), // wTerminalType(L)
    (uint8_t)(AUDIO_TERMINAL_USB_STREAMING / 256), // wTerminalType(H)
    0x00,                                          // bAssocTerminal - none
    AUDIO_IN_TERMINAL_ID,                          // bSourceID
    0x00,                                          // iTerminal

    /* The return's terminal pair, describing the other direction. A terminal is one-directional,
     * so the two streams cannot share one pair: here the USB stream is the source and the
     * destination is inside the Deluge, which is the mirror of the two above. */

    // Input terminal - the USB stream arriving from the host
    0x0C,                                          // bLength
    CS_INTERFACE,                                  // bDescriptorType
    AUDIO_AC_INPUT_TERMINAL,                       // bDescriptorSubtype
    AUDIO_RX_IN_TERMINAL_ID,                       // bTerminalID
    (uint8_t)(AUDIO_TERMINAL_USB_STREAMING % 256), // wTerminalType(L)
    (uint8_t)(AUDIO_TERMINAL_USB_STREAMING / 256), // wTerminalType(H)
    0x00,                                          // bAssocTerminal - none
    AUDIO_RX_CHANNELS,                             // bNrChannels
    0x03, 0x00,                                    // wChannelConfig - front left and front right
    0x00,                                          // iChannelNames
    0x00,                                          // iTerminal

    // Output terminal - where returning audio lands, as far as the host is concerned
    0x09,                                        // bLength
    CS_INTERFACE,                                // bDescriptorType
    AUDIO_AC_OUTPUT_TERMINAL,                    // bDescriptorSubtype
    AUDIO_RX_OUT_TERMINAL_ID,                    // bTerminalID
    (uint8_t)(AUDIO_TERMINAL_OUTPUT_TYPE % 256), // wTerminalType(L)
    (uint8_t)(AUDIO_TERMINAL_OUTPUT_TYPE / 256), // wTerminalType(H)
    0x00,                                        // bAssocTerminal - none
    AUDIO_RX_IN_TERMINAL_ID,                     // bSourceID
    0x00,                                        // iTerminal

    /* AudioStreaming Interface, alt 0 - USB audio 1.0 spec 4.5
     * Zero bandwidth. The host selects this when it is not streaming, which is what frees the
     * isochronous bandwidth for everything else on the bus.
     */
    0x09,                     // bLength
    USB_DT_INTERFACE,         // bDescriptorType
    0x02,                     // bInterfaceNumber
    0x00,                     // bAlternateSetting
    0x00,                     // bNumEndpoints
    USB_IFCLS_AUD,            // bInterfaceClass
    AUDIO_SUBCLASS_STREAMING, // bInterfaceSubClass
    0x00,                     // bInterfaceProtocol
    0x00,                     // iInterface

    // AudioStreaming Interface, alt 1 - the operational setting
    0x09,                     // bLength
    USB_DT_INTERFACE,         // bDescriptorType
    0x02,                     // bInterfaceNumber
    0x01,                     // bAlternateSetting
    0x01,                     // bNumEndpoints
    USB_IFCLS_AUD,            // bInterfaceClass
    AUDIO_SUBCLASS_STREAMING, // bInterfaceSubClass
    0x00,                     // bInterfaceProtocol
    0x00,                     // iInterface

    // Class-specific AS general
    0x07,                  // bLength
    CS_INTERFACE,          // bDescriptorType
    AUDIO_AS_GENERAL,      // bDescriptorSubtype
    AUDIO_OUT_TERMINAL_ID, // bTerminalLink
    0x01,                  // bDelay - one frame
    0x01, 0x00,            // wFormatTag - PCM

    // Type I format. One discrete sample rate, so bSamFreqType is 1 and one 24-bit rate follows.
    0x0B,                                        // bLength
    CS_INTERFACE,                                // bDescriptorType
    AUDIO_AS_FORMAT_TYPE,                        // bDescriptorSubtype
    AUDIO_FORMAT_TYPE_I,                         // bFormatType
    AUDIO_NUM_CHANNELS,                          // bNrChannels
    AUDIO_SUBFRAME_BYTES,                        // bSubframeSize
    AUDIO_BIT_RESOLUTION,                        // bBitResolution
    0x01,                                        // bSamFreqType - one discrete rate
    (uint8_t)(AUDIO_SAMPLE_RATE & 0xFF),         // tSamFreq(L)
    (uint8_t)((AUDIO_SAMPLE_RATE >> 8) & 0xFF),  // tSamFreq
    (uint8_t)((AUDIO_SAMPLE_RATE >> 16) & 0xFF), // tSamFreq(H) - rates are 24-bit

    /* Isochronous endpoint. Third endpoint in this configuration, so it takes the third row of
     * g_usb_pstd_eptbl, which is PIPE1 - the only isochronous-capable pipe MIDI leaves free.
     * The endpoint number below is a label and does not have to match the pipe number.
     */
    0x09,                                   // bLength - audio endpoints carry two extra bytes
    USB_DT_ENDPOINT,                        // bDescriptorType
    (uint8_t)(USB_EP_IN | USB_EP3),         // bEndpointAddress - MIDI uses EP1 IN and EP2 OUT
    AUDIO_EP_ATTRIBUTES,                    // bmAttributes - isochronous, asynchronous
    (uint8_t)(AUDIO_MAX_PACKET_SIZE % 256), // wMaxPacketSize(L)
    (uint8_t)(AUDIO_MAX_PACKET_SIZE / 256), // wMaxPacketSize(H)
    0x01,                                   // bInterval - every frame; full speed requires 1
    0x00,                                   // bRefresh
    0x00,                                   // bSynchAddress - no feedback endpoint

    // Class-specific isochronous endpoint. Nothing is host-settable, so no controls are claimed.
    0x07,             // bLength
    CS_ENDPOINT,      // bDescriptorType
    AUDIO_EP_GENERAL, // bDescriptorSubtype
    0x00,             // bmAttributes - sampling frequency is not host-settable
    0x00,             // bLockDelayUnits
    0x00, 0x00,       // wLockDelay

    /* ================= The return: host to device ================= *
     * Same shape as the interface above, mirrored. Its endpoint is the fourth in this
     * configuration, so it takes the fourth row of g_usb_pstd_eptbl, which is PIPE2 - freed by
     * moving USB MIDI's bulk-IN to PIPE4. */

    /* AudioStreaming Interface, alt 0 - zero bandwidth, selected when the host is not sending.
     * Without this the host would hold isochronous bandwidth open permanently in both directions. */
    0x09,                      // bLength
    USB_DT_INTERFACE,          // bDescriptorType
    AUDIO_STREAM_IN_INTERFACE, // bInterfaceNumber
    0x00,                      // bAlternateSetting
    0x00,                      // bNumEndpoints
    USB_IFCLS_AUD,             // bInterfaceClass
    AUDIO_SUBCLASS_STREAMING,  // bInterfaceSubClass
    0x00,                      // bInterfaceProtocol
    0x00,                      // iInterface

    // AudioStreaming Interface, alt 1 - the operational setting
    0x09,                      // bLength
    USB_DT_INTERFACE,          // bDescriptorType
    AUDIO_STREAM_IN_INTERFACE, // bInterfaceNumber
    0x01,                      // bAlternateSetting
    0x01,                      // bNumEndpoints
    USB_IFCLS_AUD,             // bInterfaceClass
    AUDIO_SUBCLASS_STREAMING,  // bInterfaceSubClass
    0x00,                      // bInterfaceProtocol
    0x00,                      // iInterface

    // Class-specific AS general. Linked to the terminal the USB stream enters at.
    0x07,                    // bLength
    CS_INTERFACE,            // bDescriptorType
    AUDIO_AS_GENERAL,        // bDescriptorSubtype
    AUDIO_RX_IN_TERMINAL_ID, // bTerminalLink
    0x01,                    // bDelay - one frame
    0x01, 0x00,              // wFormatTag - PCM

    // Type I format. One discrete sample rate, matching the outgoing direction.
    0x0B,                                        // bLength
    CS_INTERFACE,                                // bDescriptorType
    AUDIO_AS_FORMAT_TYPE,                        // bDescriptorSubtype
    AUDIO_FORMAT_TYPE_I,                         // bFormatType
    AUDIO_RX_CHANNELS,                           // bNrChannels
    AUDIO_SUBFRAME_BYTES,                        // bSubframeSize
    AUDIO_BIT_RESOLUTION,                        // bBitResolution
    0x01,                                        // bSamFreqType - one discrete rate
    (uint8_t)(AUDIO_SAMPLE_RATE & 0xFF),         // tSamFreq(L)
    (uint8_t)((AUDIO_SAMPLE_RATE >> 8) & 0xFF),  // tSamFreq
    (uint8_t)((AUDIO_SAMPLE_RATE >> 16) & 0xFF), // tSamFreq(H) - rates are 24-bit

    /* Isochronous OUT endpoint. The endpoint number is a label and need not match the pipe. */
    0x09,                                      // bLength - audio endpoints carry two extra bytes
    USB_DT_ENDPOINT,                           // bDescriptorType
    (uint8_t)(USB_EP_OUT | USB_EP4),           // bEndpointAddress - MIDI has EP1 IN and EP2 OUT, audio out has EP3 IN
    AUDIO_RX_EP_ATTRIBUTES,                    // bmAttributes - isochronous, adaptive
    (uint8_t)(AUDIO_RX_MAX_PACKET_SIZE % 256), // wMaxPacketSize(L)
    (uint8_t)(AUDIO_RX_MAX_PACKET_SIZE / 256), // wMaxPacketSize(H)
    0x01,                                      // bInterval - every frame; full speed requires 1
    0x00,                                      // bRefresh
    0x00,                                      // bSynchAddress - no feedback endpoint exists to name

    // Class-specific isochronous endpoint.
    0x07,             // bLength
    CS_ENDPOINT,      // bDescriptorType
    AUDIO_EP_GENERAL, // bDescriptorSubtype
    0x00,             // bmAttributes - sampling frequency is not host-settable
    0x00,             // bLockDelayUnits
    0x00, 0x00,       // wLockDelay
};

/*************************************
 *    String Descriptor               *
 *************************************/
uint8_t g_midi_string0[] = {
    /* UNICODE 0x0409 English (United States) */
    4,             /*  0:bLength */
    USB_DT_STRING, /*  1:bDescriptorType */
    0x09, 0x04     /*  2:wLANGID[0] */
};

uint8_t g_midi_string1[] = {
    38,            /*  0:bLength */
    USB_DT_STRING, /*  1:bDescriptorType */
    'S',
    0x00, /*  2:bString */
    'y',
    0x00,
    'n',
    0x00,
    't',
    0x00,
    'h',
    0x00,
    's',
    0x00,
    't',
    0x00,
    'r',
    0x00,
    'o',
    0x00,
    'm',
    0x00,
    ' ',
    0x00,
    'A',
    0x00,
    'u',
    0x00,
    'd',
    0x00,
    'i',
    0x00,
    'b',
    0x00,
    'l',
    0x00,
    'e',
    0x00,
};

uint8_t g_midi_string2[] = {
    14,            /*  0:bLength */
    USB_DT_STRING, /*  1:bDescriptorType */
    'D',
    0x00, /*  2:bString */
    'e',
    0x00,
    'l',
    0x00,
    'u',
    0x00,
    'g',
    0x00,
    'e',
    0x00,
};

uint8_t g_midi_string3[] = {
    8,             /*  0:bLength */
    USB_DT_STRING, /*  1:bDescriptorType */
    'O',
    0x00, /*  2:bString */
    'U',
    0x00,
    'T',
    0x00,
};

uint8_t g_midi_string4[] = {
    6,             /*  0:bLength */
    USB_DT_STRING, /*  1:bDescriptorType */
    'I',
    0x00, /*  2:bString */
    'N',
    0x00,
};

uint8_t* g_midi_string_table[] = {g_midi_string0, g_midi_string1, g_midi_string2, g_midi_string3, g_midi_string4};

/***********************************************************************************************************************
Private global variables and functions
***********************************************************************************************************************/
