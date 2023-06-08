#include "VoiceManager.h"

VoiceManager voiceManager;

void VoiceManager::NoteOnOff(uint8_t midiNote, bool on)
{
	if (on) {
		// Locate next available note in each channel
		for (auto& chn: channel) {
			Channel::Voice* chnVoice;
			for (auto& v: chn.voice) {
				if (v.start == 0) {
					chnVoice = &v;
					break;
				}
			}
			chnVoice->midiNote = midiNote;
			chnVoice->start = SysTickVal;
			chnVoice->envelope.noteOn = true;
			if (chn.index == channelNo::channelA) {
				gates[chnVoice->index].GateOn();
			}
		}

	} else {
		for (auto& chn: channel) {
			for (auto& v: chn.voice) {
				if (v.midiNote == midiNote) {
					v.start = 0;
					v.envelope.noteOn = false;
					gates[v.index].GateOff();
				}
			}
		}
	}
}

void VoiceManager::calcEnvelopes()
{
	for (auto& chn: channel) {
		for (auto& v: chn.voice) {
			v.envelope.calcEnvelope(chn.adsr);
		}
	}
}

