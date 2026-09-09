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

#include "io/midi/usb_audio_sysex.h"
#include "OSLikeStuff/scheduler_api.h"
#include "gui/views/session_view.h"
#include "io/midi/midi_device.h"
#include "model/clip/clip.h"
#include "model/output.h"
#include "model/song/song.h"
#include "model/usb_route.h"
#include "processing/engines/usb_audio_stream.h"
#include "storage/smsysex.h"
#include "storage/storage_manager.h"
#include "util/d_string.h"
#include <cstring>
#include <version.h>

extern "C" {
extern uint8_t currentlyAccessingCard;
}

namespace deluge::io::midi::usb_audio {

namespace {

using deluge::processing::engines::USBAudioStream;

constexpr uint32_t kNumChannels = USBAudioStream::kStemChannels;

/// Tracks per "tracks" reply. A host pages with "from"; the cap keeps one message inside what the MIDI send buffer
/// swallows in a single pass, the same reason the directory listing has one.
constexpr uint32_t kMaxTracksPerMessage = 8;

/// Longest track name put on the wire. Long enough for anything the instrument displays.
constexpr uint32_t kMaxNameChars = 30;

constexpr double kAliveInterval = 1.0;

/// A subscription is a lease, not a standing arrangement: it lapses this many seconds after the host last said
/// anything, and any request renews it.
///
/// Three things fall out of that one rule. A host that is unplugged, crashes or simply stops caring costs nothing
/// afterwards. The instrument stops talking into a cable with nothing on the end of it. And the menu line below
/// can say honestly whether anything is attached, which a flag set once and never cleared cannot.
constexpr double kSubscriptionLease = 10.0;

/// How long a host that has said nothing still counts as attached, for the menu line.
constexpr double kHostTimeout = kSubscriptionLease;

/// Outcomes. One code for every failure, so a host waits on exactly one thing and never has to parse prose.
enum Result : uint8_t {
	kOk = 0,
	kNoSourceHeld = 1,
	kNoFreeChannel = 2,
	kBadWidth = 3,
	kNoSuchTrack = 4,
	kBadChannel = 5,
	kTrackHasNoAudio = 6,
	kNoSong = 7,
	kUnknownRequest = 8,
};

/// The one host receiving pushes. One slot rather than a list: the pushes describe the state of the instrument,
/// and a second host that wants it can ask for it.
MIDICable* subscriber = nullptr;
double subscriptionExpiry = 0.0;

String attachedHostName;
uint32_t attachedHostVersion = 0;
double lastHeardFrom = -1000.0;
double lastAliveSent = 0.0;

uint32_t lastMapSignature = 0;
uint32_t lastSongSignature = 0;
int32_t lastHeldTrack = -1;

/// Copies a name into a buffer as printable ASCII, dropping anything else.
///
/// Two reasons, and both are hard requirements rather than tidiness: SysEx carries seven-bit bytes only, and this
/// JSON writer does not escape, so a quote or a backslash in a name would produce a message no host can parse.
void copySafeName(char* dest, const char* src) {
	uint32_t out = 0;
	if (src != nullptr) {
		for (uint32_t i = 0; src[i] != 0 && out < kMaxNameChars; ++i) {
			const uint8_t c = (uint8_t)src[i];
			if (c >= 0x20 && c < 0x7F && c != '"' && c != '\\') {
				dest[out++] = (char)c;
			}
		}
	}
	dest[out] = 0;
}

uint32_t hashStep(uint32_t hash, uint32_t value) {
	return (hash ^ value) * 16777619u;
}

uint32_t hashString(uint32_t hash, const char* text) {
	if (text != nullptr) {
		for (uint32_t i = 0; text[i] != 0; ++i) {
			hash = hashStep(hash, (uint8_t)text[i]);
		}
	}
	return hash;
}

/// Whether this track renders audio of its own. A MIDI or CV track has none to route, which is why it has no
/// routing menu on the instrument either.
bool trackCarriesAudio(Output* output) {
	return output != nullptr && output->type != OutputType::MIDI_OUT && output->type != OutputType::CV;
}

Output* outputAtIndex(int32_t index) {
	if (currentSong == nullptr || index < 0) {
		return nullptr;
	}
	int32_t i = 0;
	for (Output* output = currentSong->firstOutput; output != nullptr; output = output->next, ++i) {
		if (i == index) {
			return output;
		}
	}
	return nullptr;
}

int32_t indexOfOutput(Output* wanted) {
	if (currentSong == nullptr) {
		return -1;
	}
	int32_t i = 0;
	for (Output* output = currentSong->firstOutput; output != nullptr; output = output->next, ++i) {
		if (output == wanted) {
			return i;
		}
	}
	return -1;
}

int32_t countOutputs() {
	int32_t n = 0;
	if (currentSong != nullptr) {
		for (Output* output = currentSong->firstOutput; output != nullptr; output = output->next) {
			++n;
		}
	}
	return n;
}

/// channel is 1-based.
uint16_t monoBitFor(uint32_t channel) {
	return (uint16_t)(UsbRoute::CH1 << (channel - 1));
}

/// lower is the lower channel of the pair, 1-based and odd.
uint16_t pairBitFor(uint32_t lower) {
	return (uint16_t)(UsbRoute::PAIR12 << ((lower - 1) / 2));
}

/// The lower channel of the pair this channel belongs to.
uint32_t pairLowerOf(uint32_t channel) {
	return ((channel - 1) & ~1u) + 1;
}

/// Which track is being offered right now: a clip pad held down in song view names its track.
///
/// The one place the instrument says "the user is pointing at this". A host that has no such gesture ignores it;
/// one that does can light up what would accept it.
int32_t heldTrackIndex() {
	const uint8_t y = sessionView.selectedClipYDisplay;
	if (y == 255) {
		return -1;
	}
	Clip* clip = sessionView.getClipOnScreen(y);
	if (clip == nullptr || clip->output == nullptr) {
		return -1;
	}
	return indexOfOutput(clip->output);
}

/// What a channel currently carries.
struct ChannelState {
	uint32_t contributors = 0;
	bool stereo = false;
	Output* first = nullptr;
};

ChannelState stateOfChannel(uint32_t channel) {
	ChannelState state;
	if (currentSong == nullptr) {
		return state;
	}
	const uint16_t mono = monoBitFor(channel);
	const uint16_t pair = pairBitFor(pairLowerOf(channel));
	for (Output* output = currentSong->firstOutput; output != nullptr; output = output->next) {
		if (!trackCarriesAudio(output)) {
			continue;
		}
		const uint16_t route = output->usbRouting;
		if ((route & (mono | pair)) == 0) {
			continue;
		}
		if ((route & pair) != 0) {
			state.stereo = true;
		}
		if (state.first == nullptr) {
			state.first = output;
		}
		state.contributors++;
	}
	return state;
}

uint32_t mapSignature() {
	uint32_t hash = 2166136261u;
	if (currentSong == nullptr) {
		return hash;
	}
	for (Output* output = currentSong->firstOutput; output != nullptr; output = output->next) {
		hash = hashStep(hash, output->usbRouting);
		hash = hashStep(hash, (uint32_t)output->type);
		hash = hashString(hash, output->name.get());
	}
	return hash;
}

uint32_t songSignature() {
	uint32_t hash = hashStep(2166136261u, (uint32_t)(uintptr_t)currentSong);
	if (currentSong != nullptr) {
		hash = hashString(hash, currentSong->name.get());
	}
	return hash;
}

void writeChannelArray(JsonSerializer& writer) {
	char nameBuffer[kMaxNameChars + 1];
	writer.writeArrayStart("ch", true, false);
	for (uint32_t channel = 1; channel <= kNumChannels; ++channel) {
		const ChannelState state = stateOfChannel(channel);
		writer.writeOpeningTag(nullptr, true);
		writer.writeAttribute("n", (int32_t)channel);
		writer.writeAttribute("used", state.contributors > 0 ? 1 : 0);
		writer.writeAttribute("sum", (int32_t)state.contributors);
		// 0 mono, 1 the left of a pair, 2 the right of a pair.
		int32_t mode = 0;
		if (state.stereo) {
			mode = (channel == pairLowerOf(channel)) ? 1 : 2;
		}
		writer.writeAttribute("mode", mode);
		writer.writeAttribute("track", indexOfOutput(state.first));
		copySafeName(nameBuffer, state.first != nullptr ? state.first->name.get() : "");
		writer.writeAttribute("name", nameBuffer);
		writer.writeAttribute("type", state.first != nullptr ? outputTypeToString(state.first->type) : "");
		// Whether the track feeding this channel still reaches the Deluge's own sockets. A host showing a stem
		// wants to know whether it is also still in the room.
		writer.writeAttribute("main", (state.first != nullptr && state.first->isInMainMix()) ? 1 : 0);
		writer.closeTag();
	}
	writer.writeArrayEnding("ch", true, false);
}

/// Song, held source and the eight channels. Every state message carries the whole of this: a dropped message
/// costs one stale display until the next one, never a permanently wrong picture.
void writeMapBody(JsonSerializer& writer) {
	char nameBuffer[kMaxNameChars + 1];
	copySafeName(nameBuffer, currentSong != nullptr ? currentSong->name.get() : "");
	writer.writeAttribute("song", nameBuffer);

	const int32_t held = heldTrackIndex();
	writer.writeAttribute("held", held);
	Output* heldOutput = outputAtIndex(held);
	copySafeName(nameBuffer, heldOutput != nullptr ? heldOutput->name.get() : "");
	writer.writeAttribute("heldName", nameBuffer);
	// The width the held track is routed at today, not a limit on what may be asked for: 0 unrouted, 1 mono,
	// 2 a stereo pair.
	int32_t heldWidth = 0;
	if (heldOutput != nullptr) {
		if ((heldOutput->usbRouting & UsbRoute::PAIR_MASK) != 0) {
			heldWidth = 2;
		}
		else if ((heldOutput->usbRouting & UsbRoute::MONO_MASK) != 0) {
			heldWidth = 1;
		}
	}
	writer.writeAttribute("heldW", heldWidth);
	writeChannelArray(writer);
}

void writeInfoBody(JsonSerializer& writer) {
	char nameBuffer[kMaxNameChars + 1];
	writer.writeAttribute("dev", "deluge");
	writer.writeAttribute("fw", kFirmwareVersionString);
	writer.writeAttribute("ver", (int32_t)kVocabularyVersion);
	writer.writeAttribute("chOut", (int32_t)kNumChannels);
	writer.writeAttribute("chIn", (int32_t)USBAudioStream::numReturnChannels());
	// Whether a host has the audio stream open. A control channel that works while this reads zero means the
	// cable is fine and the host has simply not selected the interface.
	writer.writeAttribute("open", USBAudioStream::stemsWanted() ? 1 : 0);
	writer.writeAttribute("sub", subscriber != nullptr ? 1 : 0);
	writer.writeAttribute("tracks", countOutputs());
	copySafeName(nameBuffer, currentSong != nullptr ? currentSong->name.get() : "");
	writer.writeAttribute("song", nameBuffer);
}

void sendPush(const char* what) {
	if (subscriber == nullptr) {
		return;
	}
	// A whole channel map is a few hundred bytes. Sending one into a send buffer that cannot hold it would put
	// half a message on the wire, which is worse than sending nothing: the next push carries the same state.
	if (subscriber->sendBufferSpace() < 1024) {
		return;
	}
	JsonSerializer& writer = smSysex::sharedWriter();
	smSysex::startDirect(writer);
	writer.writeOpeningTag("^usbAudio", false, true);
	writer.writeAttribute("push", what);
	if (strcmp(what, "alive") == 0) {
		writer.writeAttribute("ver", (int32_t)kVocabularyVersion);
		writer.writeAttribute("open", USBAudioStream::stemsWanted() ? 1 : 0);
	}
	else {
		writeMapBody(writer);
	}
	writer.closeTag(true);
	smSysex::sendMsg(*subscriber, writer);
}

/// Clears one channel, and the pair it belongs to, from every track.
void clearChannelEverywhere(uint32_t channel) {
	const uint16_t mono = monoBitFor(channel);
	const uint16_t pair = pairBitFor(pairLowerOf(channel));
	for (Output* output = currentSong->firstOutput; output != nullptr; output = output->next) {
		output->usbRouting &= (uint16_t)~(mono | pair);
	}
}

/// The request as read off the wire. Absent numbers stay at -1 so a missing field is told from a zero.
struct Request {
	String verb;
	String name;
	int32_t track = -1;
	int32_t channel = -1;
	int32_t width = -1;
	int32_t on = -1;
	int32_t add = -1;
	int32_t from = -1;
	int32_t tag = -1;
	int32_t version = -1;
};

void readRequest(JsonDeserializer& reader, Request& request) {
	char const* tagName;
	reader.match('{');
	while (*(tagName = reader.readNextTagOrAttributeName())) {
		if (!strcmp(tagName, "req")) {
			reader.readTagOrAttributeValueString(&request.verb);
		}
		else if (!strcmp(tagName, "name")) {
			reader.readTagOrAttributeValueString(&request.name);
		}
		else if (!strcmp(tagName, "track")) {
			request.track = reader.readTagOrAttributeValueInt();
		}
		else if (!strcmp(tagName, "ch")) {
			request.channel = reader.readTagOrAttributeValueInt();
		}
		else if (!strcmp(tagName, "width")) {
			request.width = reader.readTagOrAttributeValueInt();
		}
		else if (!strcmp(tagName, "on")) {
			request.on = reader.readTagOrAttributeValueInt();
		}
		else if (!strcmp(tagName, "add")) {
			request.add = reader.readTagOrAttributeValueInt();
		}
		else if (!strcmp(tagName, "from")) {
			request.from = reader.readTagOrAttributeValueInt();
		}
		else if (!strcmp(tagName, "tag")) {
			request.tag = reader.readTagOrAttributeValueInt();
		}
		else if (!strcmp(tagName, "ver")) {
			request.version = reader.readTagOrAttributeValueInt();
		}
		else {
			// An unknown key is ignored rather than refused: the two ends of this conversation will not be
			// updated together.
			reader.exitTag();
		}
	}
	reader.match('}');
}

/// Finds the track a routing request names, and says why it cannot be used.
Result resolveTrack(const Request& request, Output** outputOut) {
	if (currentSong == nullptr) {
		return kNoSong;
	}
	Output* output = outputAtIndex(request.track);
	if (output == nullptr) {
		return kNoSuchTrack;
	}
	// The name is an optional guard: a host acting on a list it read before the song changed underneath it gets a
	// refusal rather than a wrong track.
	if (request.name.get()[0] != 0) {
		char safe[kMaxNameChars + 1];
		copySafeName(safe, output->name.get());
		if (strcmp(safe, request.name.get()) != 0) {
			return kNoSuchTrack;
		}
	}
	if (!trackCarriesAudio(output)) {
		return kTrackHasNoAudio;
	}
	*outputOut = output;
	return kOk;
}

/// Applies one destination to a track. channel is 1-based; a stereo pair is named by its lower channel.
Result applyRouting(Output* output, int32_t channel, int32_t width, bool add) {
	if (width != 1 && width != 2) {
		return kBadWidth;
	}
	if (channel < 1 || channel > (int32_t)kNumChannels) {
		return kBadChannel;
	}
	if (width == 2 && (uint32_t)channel != pairLowerOf((uint32_t)channel)) {
		// A pair is named by its lower channel. Refusing rather than rounding keeps a host's own mistake visible.
		return kBadChannel;
	}
	if (!add) {
		output->usbRouting &= (uint16_t)~UsbRoute::ANY_USB;
	}
	output->usbRouting |= (width == 1) ? monoBitFor((uint32_t)channel) : pairBitFor((uint32_t)channel);
	return kOk;
}

/// The lowest channel, or pair, nothing is using.
int32_t findFreeChannel(int32_t width) {
	if (width == 1) {
		for (uint32_t channel = 1; channel <= kNumChannels; ++channel) {
			if (stateOfChannel(channel).contributors == 0) {
				return (int32_t)channel;
			}
		}
	}
	else if (width == 2) {
		for (uint32_t lower = 1; lower < kNumChannels; lower += 2) {
			if (stateOfChannel(lower).contributors == 0 && stateOfChannel(lower + 1).contributors == 0) {
				return (int32_t)lower;
			}
		}
	}
	return -1;
}

} // namespace

const char* hostName() {
	const bool present = (subscriber != nullptr) || (getSystemTime() - lastHeardFrom < kHostTimeout);
	return present ? attachedHostName.get() : "";
}

uint32_t hostVocabularyVersion() {
	return (hostName()[0] != 0) ? attachedHostVersion : 0;
}

void handleRequest(MIDICable& cable, JsonDeserializer& reader) {
	Request request;
	readRequest(reader, request);
	lastHeardFrom = getSystemTime();
	// Any word from the subscribed host renews its lease.
	if (subscriber == &cable) {
		subscriptionExpiry = lastHeardFrom + kSubscriptionLease;
	}

	const char* verb = request.verb.get();
	Result result = kOk;
	int32_t resultChannel = -1;
	int32_t resultWidth = -1;

	if (!strcmp(verb, "hello")) {
		char safe[kMaxNameChars + 1];
		copySafeName(safe, request.name.get());
		attachedHostName.set(safe);
		attachedHostVersion = (request.version > 0) ? (uint32_t)request.version : 0;
	}
	else if (!strcmp(verb, "sub")) {
		if (request.on == 0) {
			if (subscriber == &cable) {
				subscriber = nullptr;
			}
		}
		else {
			// A renewal is not a fresh subscription. Only a host that was not already subscribed gets the state
			// pushed at it unasked; renewing is how a subscribed host stays subscribed, and treating the two the
			// same re-announced the whole song on every renewal.
			if (subscriber != &cable) {
				subscriber = &cable;
				// Force the next pass to send the state rather than waiting for it to change.
				lastMapSignature = 0;
				lastSongSignature = 0;
				lastHeldTrack = -2;
			}
			subscriptionExpiry = lastHeardFrom + kSubscriptionLease;
		}
	}
	else if (!strcmp(verb, "set")) {
		Output* output = nullptr;
		result = resolveTrack(request, &output);
		if (result == kOk) {
			result = applyRouting(output, request.channel, request.width, request.add == 1);
			if (result == kOk) {
				resultChannel = request.channel;
				resultWidth = request.width;
			}
		}
	}
	else if (!strcmp(verb, "clear")) {
		if (currentSong == nullptr) {
			result = kNoSong;
		}
		else if (request.channel >= 1 && request.channel <= (int32_t)kNumChannels) {
			clearChannelEverywhere((uint32_t)request.channel);
			resultChannel = request.channel;
		}
		else if (request.track >= 0) {
			Output* output = nullptr;
			result = resolveTrack(request, &output);
			if (result == kOk) {
				output->usbRouting &= (uint16_t)~UsbRoute::ANY_USB;
			}
		}
		else {
			result = kBadChannel;
		}
	}
	else if (!strcmp(verb, "main")) {
		Output* output = nullptr;
		result = resolveTrack(request, &output);
		if (result == kOk) {
			if (request.on == 0) {
				output->usbRouting &= (uint16_t)~UsbRoute::MAIN;
			}
			else {
				output->usbRouting |= UsbRoute::MAIN;
			}
		}
	}
	else if (!strcmp(verb, "claim")) {
		// The gesture: the user holds a track on the instrument, the host asks for somewhere to put it. The host
		// never names a channel - allocation stays with the device that owns the channel numbers.
		const int32_t width = (request.width == 1 || request.width == 2) ? request.width : 2;
		Output* output = outputAtIndex(heldTrackIndex());
		if (currentSong == nullptr) {
			result = kNoSong;
		}
		else if (output == nullptr) {
			result = kNoSourceHeld;
		}
		else if (!trackCarriesAudio(output)) {
			result = kTrackHasNoAudio;
		}
		else {
			const int32_t channel = findFreeChannel(width);
			if (channel < 0) {
				result = kNoFreeChannel;
			}
			else {
				result = applyRouting(output, channel, width, true);
				if (result == kOk) {
					resultChannel = channel;
					resultWidth = width;
				}
			}
		}
	}
	else if (strcmp(verb, "info") != 0 && strcmp(verb, "map") != 0 && strcmp(verb, "tracks") != 0) {
		// An unknown verb still gets an answer, carrying this device's version, so a host built against a later
		// vocabulary learns what it is talking to instead of timing out.
		result = kUnknownRequest;
	}

	JsonSerializer& writer = smSysex::sharedWriter();
	smSysex::startReply(writer, reader);
	writer.writeOpeningTag("^usbAudio", false, true);
	writer.writeAttribute("req", verb);
	writer.writeAttribute("err", (int32_t)result);
	if (request.tag >= 0) {
		writer.writeAttribute("tag", request.tag);
	}
	if (!strcmp(verb, "sub")) {
		// Told rather than guessed: the host renews before this many seconds have passed.
		writer.writeAttribute("lease", (int32_t)kSubscriptionLease);
	}
	if (resultChannel >= 0) {
		writer.writeAttribute("ch", resultChannel);
	}
	if (resultWidth >= 0) {
		writer.writeAttribute("width", resultWidth);
	}

	if (!strcmp(verb, "map")) {
		writeMapBody(writer);
	}
	else if (!strcmp(verb, "tracks")) {
		char nameBuffer[kMaxNameChars + 1];
		const int32_t total = countOutputs();
		int32_t from = (request.from > 0) ? request.from : 0;
		writer.writeAttribute("from", from);
		writer.writeAttribute("total", total);
		writer.writeArrayStart("tr", true, false);
		int32_t index = 0;
		uint32_t written = 0;
		for (Output* output = (currentSong != nullptr) ? currentSong->firstOutput : nullptr;
		     output != nullptr && written < kMaxTracksPerMessage; output = output->next, ++index) {
			if (index < from) {
				continue;
			}
			writer.writeOpeningTag(nullptr, true);
			writer.writeAttribute("i", index);
			copySafeName(nameBuffer, output->name.get());
			writer.writeAttribute("name", nameBuffer);
			writer.writeAttribute("type", outputTypeToString(output->type));
			writer.writeAttribute("audio", trackCarriesAudio(output) ? 1 : 0);
			// The routing bits themselves, so a host can show a track sent to several places without asking
			// about each channel in turn.
			writer.writeAttribute("route", (int32_t)output->usbRouting);
			writer.writeAttribute("main", output->isInMainMix() ? 1 : 0);
			writer.closeTag();
			written++;
		}
		writer.writeArrayEnding("tr", true, false);
		writer.writeAttribute("n", (int32_t)written);
	}
	else {
		writeInfoBody(writer);
	}

	writer.closeTag(true);
	smSysex::sendMsg(cable, writer);
}

void routine() {
	// Nothing is broadcast at a host that did not ask for it. A machine with no subscriber pays one comparison.
	if (subscriber == nullptr || currentSong == nullptr) {
		return;
	}
	if (getSystemTime() > subscriptionExpiry) {
		subscriber = nullptr;
		return;
	}
	// The song's structures move while the card is being read. Reading them here would be reading them mid-change.
	if (currentlyAccessingCard != 0) {
		return;
	}

	const double now = getSystemTime();

	// One push per pass, most significant first, so no pass walks the song more than it has to.
	const uint32_t songSig = songSignature();
	if (songSig != lastSongSignature) {
		lastSongSignature = songSig;
		lastMapSignature = mapSignature();
		lastHeldTrack = heldTrackIndex();
		lastAliveSent = now;
		sendPush("song");
		return;
	}

	const uint32_t mapSig = mapSignature();
	if (mapSig != lastMapSignature) {
		lastMapSignature = mapSig;
		lastHeldTrack = heldTrackIndex();
		lastAliveSent = now;
		sendPush("map");
		return;
	}

	const int32_t held = heldTrackIndex();
	if (held != lastHeldTrack) {
		lastHeldTrack = held;
		lastAliveSent = now;
		sendPush("held");
		return;
	}

	if (now - lastAliveSent >= kAliveInterval) {
		lastAliveSent = now;
		sendPush("alive");
	}
}

} // namespace deluge::io::midi::usb_audio
