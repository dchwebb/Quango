#pragma once

#include "initialisation.h"
#include "Envelope.h"



class VoiceManager
{
public:
	enum channelNo {channelA = 0, channelB = 1};
	void NoteOnOff(uint8_t midiNote, bool on);
	void calcEnvelopes();

	struct Channel {
		volatile ADSR* adsr;		// pointer to ADC values for ADSR pots
		volatile uint16_t* level;	// pointer to ADC value for level slider

		struct Voice {
			Voice() {};
			Voice(volatile uint32_t* dac, volatile uint32_t* led) : envelope{dac, led} {};
			Envelope envelope;
			uint8_t midiNote = 0;
			uint32_t start = 0;			// time that note was started
		} voice[4];

		Channel(volatile ADSR* adsr, volatile uint16_t* level, Voice v1, Voice v2, Voice v3, Voice v4)
		 : adsr{adsr}, level{level}, voice{v1, v2, v3, v4} {};

	};

	// Initialise each channel's voices with pointers to the internal DAC and LED PWM setting
	Channel channel[2] = {
		{
			&adc.EnvA,
			&adc.ChannelALevel,
			{&DAC1->DHR12R1, &TIM3->CCR1},
			{&DAC3->DHR12R1, &TIM3->CCR2},
			{&DAC2->DHR12R1, &TIM3->CCR3},
			{&DAC1->DHR12R2, &TIM3->CCR4},
		},
		{
			&adc.EnvB,
			&adc.ChannelBLevel,
			{&DAC4->DHR12R2, &TIM4->CCR1},
			{&DAC3->DHR12R2, &TIM4->CCR2},
			{&DAC4->DHR12R1, &TIM4->CCR3},
			{nullptr, &(TIM4->CCR4)},				// B4 is handled with an external DAC
		}
	};
};

extern VoiceManager voiceManager;
