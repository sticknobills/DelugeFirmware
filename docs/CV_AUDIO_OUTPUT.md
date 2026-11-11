# CV Audio Output: feature guide

Send any track's audio out of the Deluge's two CV sockets instead of the main outputs. 7-segment Deluge only.

## The CV Audio setting

Every track has one **CV Audio** setting (`CVAU` on the display). It decides where the track's audio goes:

| Option | Where the track goes | Does pan matter? |
|---|---|---|
| `MAIN` (default) | The main outputs, as in stock firmware | Yes, as normal |
| `CV1` | CV1 only, in mono | No |
| `CV2` | CV2 only, in mono | No |
| `CVSTEREO` | CV1 (left) and CV2 (right) as a stereo pair | Yes: it places the track between CV1 and CV2 |

Turning the select knob changes the output immediately, like every other Deluge menu.

A track on a CV output is removed from the main mix completely. Its reverb is removed too: a CV track never feeds the Deluge's reverb, so no reverb tail from it reaches the main outputs. The track's own reverb setting is not changed.

### Where to find it

- **Synth tracks:** the sound menu → `CVAU`
- **Kits:** KIT GLOBAL FX with **AFFECT ENTIRE on** → `CVAU`. The whole kit is routed together.
- **Sample and audio tracks:** the audio clip menu → `CVAU`

### The setting belongs to the track

The CV Audio setting is saved with the clip, not with the sound. It stays when you:

- load another synth preset or kit
- change the track type (synth, kit, MIDI, CV)
- turn the clip into an audio clip

A copy of a clip keeps its setting. A new, empty clip starts on `MAIN`.

## Signal flow

```
                  ┌──────── MAIN ──────────▶ main outputs (+ reverb)
                  │
track ─▶ pan ─────┼──────── CV1 ───────────▶ CV1 socket (mono)
                  │
                  ├──────── CV2 ───────────▶ CV2 socket (mono)
                  │
                  └──────── CVSTEREO ──┬───▶ CV1 socket (left)
                                       └───▶ CV2 socket (right)
```

Each track takes exactly one of these paths. Several tracks can share a socket; they are mixed together there.

## Levels

- **A track's level on CV is its own volume.** Balance several tracks on one socket with their track volumes.
- **The CV output level is fixed.** There is no master level for the CV sockets.
- **The song's master volume and master compressor affect only the main outputs.** CV tracks are taken out before them.

## Example setups

### Two separate mono outputs

1. Set your bass track to `CV1`.
2. Set your keys track to `CV2`.
3. CV1 now carries only the bass and CV2 only the keys. Neither plays from the main outputs.

### A stereo bus out

1. Set one or more tracks to `CVSTEREO`.
2. Connect the CV sockets to a stereo input with a channel-splitting TRS cable (see Wiring below).
3. The tracks arrive in stereo, placed by their pan.

### A kit to its own output

1. Open KIT GLOBAL FX with AFFECT ENTIRE on.
2. Set `CVAU` to `CV1`, `CV2` or `CVSTEREO`.

## Wiring

CV1 and CV2 share one 3.5 mm stereo socket: CV1 on the tip (left), CV2 on the ring (right).

Use a cable or adapter that **splits** the two channels, such as a TRS-to-two-TS cable. Do **not** use a "headphone splitter" that connects two stereo sockets in parallel: it ties CV1 and CV2 together.

## Audio quality

- Bit depth: 16-bit converter
- Streaming rate: about 47 kHz per channel
- Frequency response: flat within ±3 dB to 4.7 kHz, −3 dB at about 6 kHz, −6 dB at 10 kHz, after a fixed treble correction for the sockets' own filter
- Noise: worst-case spur −60 dB, median −86 dB (main outputs for reference: −79 / −92 dB)
- Distortion: −50 to −53 dB (main outputs: −80 to −91 dB)
- Latency: about 16.4 ms behind the main outputs
- Loud signals start to compress above about a third of the output's range

These figures were measured on AUX Sends 1.0, which uses the same audio path.

## Known limitations

### Hardware

- **7-segment only.** On OLED models the CV converter and the screen share one data bus. Every screen update would interrupt the audio for 17–25 ms. No firmware-only fix exists.

### Firmware

- **One output per track.** There are no send levels and no parallel copy. To add an effect to a track while keeping it dry in the mix, route the track to CV and use the effect's own mix control.
- **`CVSTEREO` makes the Deluge mix in stereo.** The stock Deluge mixes in mono unless headphones or the right main output are connected, and pan does nothing in mono. While a track is on `CVSTEREO`, the Deluge mixes in stereo anyway. With only the left main output connected, that output then carries only the left half of the mix.
- **Classic CV/Gate note output is blocked on both CV sockets** while any track is routed to CV, because both use the same converter. Not tested with real CV/Gate gear.
- **Single drums cannot be routed on their own.** A kit is routed as a whole.

## Upgrading from AUX Sends 1.0.x

- Send levels, the `MAIN` switch and the AUX master (SETTINGS → AUX and the gold knob) are removed.
- Songs saved with AUX Sends load onto the nearest output: one socket → `CV1` or `CV2`; both sockets, or the old stereo split on → `CVSTEREO`.
- A track that was in the main mix and sent to CV now goes to CV only.
- Send levels are not kept; set the level with the track's volume.

## Binary size

About +33 KB (+1.9%) against the official community 1.3.0 firmware.

---

**⚠️ Experimental feature. Install and use at your own risk.**
