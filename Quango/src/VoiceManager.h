#pragma once

#include "initialisation.h"
#include "Envelope.h"



class VoiceManager
{
public:
	enum channelNo {channelA = 0, channelB = 1};
	void NoteOnOff(uint8_t midiNote, bool on);
	void CalcEnvelopes();
	void RetriggerGates();
	void Pitchbend(uint16_t pitch);

	float pitchbend = 0.0f;
	static constexpr float pitchbendSemitones = 12.0f;

	struct Channel {
		channelNo index;
		uint32_t counter = 0;		// Used to determine oldest note for note stealing
		volatile ADSR* adsr;		// pointer to ADC values for ADSR pots
		volatile uint16_t* level;	// pointer to ADC value for level slider

		struct Voice {
			Voice(uint8_t index, volatile uint32_t* dac, volatile uint32_t* led) : index(index), envelope{dac, led} {};

			uint8_t index = 0;
			Envelope envelope;
			//volatile uint32_t* pitchDAC;
			uint8_t midiNote = 0;
			uint32_t startTime = 0;		// time that note was started (for note stealing)

			void SetPitch(channelNo chn);
			void SetPitch(channelNo chn, uint16_t dacOutput);
		} voice[4];

		Channel(channelNo chn, volatile ADSR* adsr, volatile uint16_t* level, Voice v1, Voice v2, Voice v3, Voice v4)
		 : index{chn}, adsr{adsr}, level{level}, voice{v1, v2, v3, v4} {};

	};

	// Initialise each channel's voices with pointers to the internal DAC and LED PWM setting
	Channel channel[2] = {
		{
			channelA,
			&adc.EnvA,
			&adc.ChannelALevel,
			{0, &DAC1->DHR12R1, &TIM3->CCR1},
			{1, &DAC3->DHR12R1, &TIM3->CCR2},
			{2, &DAC2->DHR12R1, &TIM3->CCR3},
			{3, &DAC1->DHR12R2, &TIM3->CCR4},
		},
		{
			channelB,
			&adc.EnvB,
			&adc.ChannelBLevel,
			{0, &DAC4->DHR12R2, &TIM4->CCR1},
			{1, &DAC3->DHR12R2, &TIM4->CCR2},
			{2, &DAC4->DHR12R1, &TIM4->CCR3},
			{3, nullptr,        &(TIM4->CCR4)},				// B4 is handled with an external DAC
		}
	};

	struct Gate {
		volatile uint32_t* gateODR;
		uint32_t gatePin;
		uint32_t gateRetrigger;		// Used to apply a gap in the gate signal when note stealing

		void GateOn()	{ *gateODR |= gatePin; }
		void GateOff()	{ *gateODR &= ~gatePin; }
	};

	Gate gates[4] = {
		{&GPIOD->ODR, GPIO_ODR_OD2},
		{&GPIOD->ODR, GPIO_ODR_OD3},
		{&GPIOD->ODR, GPIO_ODR_OD4},
		{&GPIOD->ODR, GPIO_ODR_OD5}
	};


};

extern VoiceManager voiceManager;
