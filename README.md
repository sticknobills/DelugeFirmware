# Deluge CV Audio Output Firmware (7-segment only)

Send any track's audio out of the Deluge's two CV sockets instead of the main outputs. Each track goes to one place: the main outputs, CV1, CV2, or both CV sockets as a stereo pair.

## Features

- One **CV Audio** setting per track: `MAIN`, `CV1`, `CV2` or `CVSTEREO`
- `CV1` and `CV2` carry the track in mono; `CVSTEREO` carries it in stereo (CV1 left, CV2 right), placed by the track's pan
- A track on a CV output leaves the main mix completely, including its reverb
- The setting belongs to the track: it stays when you load another preset, change the track type, or turn the clip into an audio clip
- Works on synth, kit, sample and audio tracks

## Documentation

📖 See the [CV Audio Output feature guide](docs/CV_AUDIO_OUTPUT.md) for controls, example setups, audio quality and known limitations.

## Compatibility

**⚠️ 7-segment Deluge only.**
On OLED models the CV converter and the screen share one data bus, so CV audio cannot work there. See the [feature guide](docs/CV_AUDIO_OUTPUT.md) for details.

Built on **community Release 1.3.0**.

## Installation

⚠️ Experimental feature. Install at your own risk.

### For users (flash only)

1. **Back up your SD card**: copy the entire card to your computer first
2. Download `CV-Audio-Output-2.0.bin` from [Releases](https://github.com/sticknobills/DelugeFirmware/releases)
3. Copy the `.bin` file to the root of your SD card (no subfolders), and remove any other `.bin` from the root
4. Insert the card into your Deluge and power off
5. Hold SHIFT while powering on
6. Wait about 1 minute for the flash to complete; do not interrupt or power off
7. Verify: SHIFT + SELECT → Firmware Version
8. Your songs and samples are not affected by the flash

To go back, put an official firmware `.bin` on the card and repeat.

### For developers (build from source)

```bash
git clone https://github.com/sticknobills/DelugeFirmware.git
cd DelugeFirmware
git checkout cv-audio-output
./dbt build release
```

The result is `build/Release/deluge.bin`. See the [upstream build instructions](https://github.com/SynthstromAudible/DelugeFirmware/blob/main/docs/CONTRIBUTING.md) for details.

## Upgrading from AUX Sends 1.0.x

CV Audio Output replaces AUX Sends. Send levels, the `MAIN` switch and the AUX master level are gone.

Songs saved with AUX Sends load onto the nearest output:

- A track with a send to one socket goes to `CV1` or `CV2`
- A track with sends to both sockets, or with the old stereo split on, goes to `CVSTEREO`
- A track that was in the main mix *and* sent to CV now goes to CV only
- Send levels are not kept; use the track's volume instead

## Known limitations

- **No send levels and no parallel copy.** A track goes to one output at a time. To feed a pedal and keep the dry track in the mix, route the track to CV and use the pedal's own mix control.
- **The CV output level is fixed.** A track's level on CV is its own volume. The song's master volume and master compressor do not affect the CV outputs.
- **`CVSTEREO` makes the Deluge mix in stereo.** With only the left main output connected and no headphones, the left output then carries only the left half of the mix.
- **Classic CV/Gate note output is blocked on both CV sockets** while any track is routed to CV. Not tested with real CV/Gate gear.
- **Kits** are routed as a whole, from KIT GLOBAL FX with AFFECT ENTIRE on. Single drums cannot be routed on their own.

## About this fork

A downstream fork of [SynthstromAudible/DelugeFirmware](https://github.com/SynthstromAudible/DelugeFirmware). CV Audio Output is maintained on the `cv-audio-output` branch, and each release names the community firmware it is built on.

## Bugs

An experimental feature developed by one person with a limited test setup. [Please report any bugs!](https://github.com/sticknobills/DelugeFirmware/issues/new/choose)

## License

GPL-3.0 (from the Deluge community firmware)
