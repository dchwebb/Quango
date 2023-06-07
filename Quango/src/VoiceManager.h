#pragma once

#include "initialisation.h"
#include "Envelope.h"



class VoiceManager
{
public:
	enum channelNo {channelA = 0, channelB = 1};

	//VoiceManager();
	void SetEnv(channelNo channel, uint8_t voice, uint16_t value);

	struct Channel {
		ADSR* adsr;
		uint16_t* level;

		struct Voice {
			Voice() {};
			Voice(volatile uint32_t* dac, volatile uint32_t* led) : envelope{dac, led} {};
			Envelope envelope;
			uint8_t midiNote;
		} voice[4];

		Channel(Voice v1, Voice v2, Voice v3, Voice v4) : voice{v1, v2, v3, v4} {};
	};

	// Initialise each channel's voices with the appropriate pointer to the internal DAC and LED PWM setting
	Channel channel[2] = {
		{
			{&DAC1->DHR12R1, &TIM3->CCR1},
			{&DAC3->DHR12R1, &TIM3->CCR2},
			{&DAC2->DHR12R1, &TIM3->CCR3},
			{&DAC1->DHR12R2, &TIM3->CCR4},
		},
		{
			{&DAC4->DHR12R2, &TIM4->CCR1},
			{&DAC3->DHR12R2, &TIM4->CCR2},
			{&DAC4->DHR12R1, &TIM4->CCR3},
			{nullptr, &(TIM4->CCR4)},				// B4 is handled with an external DAC
		}
	};
};

extern VoiceManager voiceManager;
