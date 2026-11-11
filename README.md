# Deluge USB Audio — multitrack

Multitrack USB audio for the Deluge: send each track to its own channel in your DAW, and send up to four channels back. No driver needed.

The current release (2.0) is built on the community firmware's own USB audio, which already gives the Deluge a stereo USB audio interface. It adds a third mode, multitrack, alongside the community nightly build's MIDI-only and stereo modes.

## Features

- **Three modes**, in `SETTINGS > USB AUDIO > MODE` (7SEG: `USBA > MODE`):
  - `OFF`: USB MIDI only.
  - `STEREO` (the default): upstream's stereo USB audio, unchanged. 2 channels each way, 24-bit.
  - `MULTITRACK`: 8 channels to the computer and 4 back, 16-bit, 44.1 kHz, alongside USB MIDI.
- **Per track**, in multitrack mode: send to any of 8 mono channels or 4 stereo pairs, and optionally take the track out of the Deluge's own outputs.
- **Audio back from the computer**: channels 1-2 work as in stereo mode. Channels 3-4 are a second audio clip source, `USB INPUT 3-4`.
- **Delay compensation**: holds the Deluge's own tracks back 0-100 ms, so a track sent through an effect on the computer comes back in time.
- Multitrack's memory is only set aside when the Deluge starts in multitrack mode.

## Quick start (multitrack)

1. Go to `SETTINGS > USB AUDIO > MODE` and choose `MULTITRACK` (7SEG: `MULT`). Restart the Deluge.
2. Connect it to your computer. It appears as an audio device with 8 inputs and 4 outputs. Set your DAW to 44.1 kHz.
3. Send a track out:
   - **Synth or audio clip:** open the clip, press `SELECT`, go to `USB AUDIO` (7SEG: `USBA`), and turn on the channels you want.
   - **Kit:** turn on `AFFECT ENTIRE` first, then press `SELECT` and go to `USB AUDIO`. A kit is routed as a whole. The menu of a single drum has no USB routing.
   - Turn off `MAIN OUTPUTS` (`MAIN`) to hear the track only through USB.
4. Record those channels in your DAW. A new song sends nothing until you route a track.

To play the computer's channels 3-4 on the Deluge, set an audio clip's source to `USB INPUT 3-4` (`SHIFT` + `SELECT` > `AUDIO SOURCE`). Then set the clip to `LOOPER` mode: hold its pad in song view and turn `SELECT`. New audio clips start in `PLAYER` mode, which doesn't play its input.

The computer's channels 1-2 are mixed straight into the Deluge's outputs by default. Turn this off with `SETTINGS > USB AUDIO > INPUT`.

## Documentation

📖 [USB Audio guide](website/src/content/docs/features/usb_audio.mdx): every mode, setting and limitation, including how to send a track through an effect and back.

## Changes from previous releases

- 1.0's USB audio is replaced by the community firmware's, with multitrack added on top. Clocking now follows upstream's design.
- The settings moved from `SETTINGS > USB` to `SETTINGS > USB AUDIO`.
- `RTN` and `RLVL` are gone. Channels 1-2 use the community firmware's `INPUT` setting, which mixes them in after the song's effects. Channels 3-4 are heard through an audio clip.
- An audio clip takes a pair as its source: `USB INPUT` (1-2) or `USB INPUT 3-4`. Single return channels are gone; a 1.0 song set to one loads as the pair it belongs to.
- Track routing saved in release 1.0 songs loads unchanged.
- Delay compensation no longer delays the audio coming back from the computer.

## Compatibility

Tested on a 7-segment Deluge with macOS. Not yet tested on OLED Deluges, Windows or iPad. Windows' built-in driver may not handle more than two channels. Reports welcome.

While connected as a USB audio device, the Deluge cannot host a USB MIDI controller. MIDI to and from the computer still works.

## Installation

⚠️ Experimental feature! Install at your own risk.

### For users (flash only)

1. **Back up your SD card**: copy the entire card to your computer first.
2. Download `USB-Audio-2.0.bin` from [Releases](https://github.com/sticknobills/DelugeFirmware/releases/tag/USB-Audio-2.0).
3. Copy the `.bin` file to the root of your SD card (no subfolders), and delete any other `.bin` there.
4. Insert the card into your Deluge and power off.
5. Hold `SHIFT` while powering on.
6. Wait about a minute for the flash to complete. Do not interrupt it or power off.
7. Your songs and samples are unaffected by the flash.

To revert: put an official Deluge firmware `.bin` back and repeat.

### For developers (build from source)

Clone this repository and follow the [upstream build instructions](https://github.com/SynthstromAudible/DelugeFirmware/blob/main/docs/CONTRIBUTING.md).

```bash
git clone https://github.com/sticknobills/DelugeFirmware.git
cd DelugeFirmware
git checkout usb-multitrack-audio
# Then follow upstream build steps
```

## About this fork

This repo is a downstream fork of [SynthstromAudible/DelugeFirmware](https://github.com/SynthstromAudible/DelugeFirmware). USB Audio 2.0 is built on upstream `main` at 8766cce2 (2026-11-10), and is maintained on `usb-multitrack-audio`. USB Audio 1.0 stays available from its [release](https://github.com/sticknobills/DelugeFirmware/releases/tag/USB-Audio-1.0).

## Bugs

This is an experimental feature in development by one person with a limited testing environment. [Please report any bugs!](https://github.com/sticknobills/DelugeFirmware/issues/new/choose)

## License

GPL-3.0 (from Deluge community firmware)
