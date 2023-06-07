#include "VoiceManager.h"

VoiceManager voiceManager;

void VoiceManager::NoteOnOff(uint8_t midiNote, bool on)
{
	if (on) {
		// Locate next available note in each channel
		Channel::Voice* voiceA;
		for (auto& v: channel[channelA].voice) {
			if (v.start == 0) {
				voiceA = &v;
				break;
			}
		}
		voiceA->midiNote = midiNote;
		voiceA->start = SysTickVal;
		voiceA->envelope.noteOn = true;
	} else {
		for (auto& v: channel[channelA].voice) {
			if (v.midiNote == midiNote) {
				v.start = 0;
				v.envelope.noteOn = false;
			}
		}
	}
}

void VoiceManager::calcEnvelopes()
{
	for (auto& v: channel[channelA].voice) {
		v.envelope.calcEnvelope(channel[channelA].adsr);
	}
}

