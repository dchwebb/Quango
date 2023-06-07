#include "VoiceManager.h"

VoiceManager voiceManager;

VoiceManager::VoiceManager() {
	channel[channelA].voice[0].envelope.envDAC = &(DAC1->DHR12R1);
	channel[channelA].voice[1].envelope.envDAC = &(DAC3->DHR12R1);
	channel[channelA].voice[2].envelope.envDAC = &(DAC2->DHR12R1);
	channel[channelA].voice[3].envelope.envDAC = &(DAC1->DHR12R2);

	channel[channelB].voice[0].envelope.envDAC = &(DAC4->DHR12R2);
	channel[channelB].voice[1].envelope.envDAC = &(DAC3->DHR12R2);
	channel[channelB].voice[2].envelope.envDAC = &(DAC4->DHR12R1);

}



