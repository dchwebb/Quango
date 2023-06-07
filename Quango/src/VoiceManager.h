#pragma once

#include "initialisation.h"
#include "Envelope.h"



class VoiceManager
{
public:
	enum channelNo {channelA = 0, channelB = 1};

	VoiceManager();
	void SetEnv(channelNo channel, uint8_t voice, uint16_t value);

	struct Channel {
		Channel() {};
		//Channel(volatile uint32_t* dac) : envelope{dac} {};

		ADSR* adsr;
		uint16_t* level;

		struct Voice {
			Voice() {};
			Voice(volatile uint32_t* dac) : envelope{dac} {};
			Envelope envelope;
			uint8_t midiNote;
		};

		Voice voice[4] = {
				{&(DAC1->DHR12R1)},
				{&(DAC3->DHR12R1)},
				{&(DAC2->DHR12R1)},
				{&(DAC1->DHR12R2)},

		};


	};

	Channel channel[2];
};

extern VoiceManager voiceManager;
