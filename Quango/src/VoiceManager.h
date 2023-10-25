#pragma once

#include "initialisation.h"
#include "Envelope.h"


class VoiceManager
{
public:
	static constexpr uint32_t octaves = 7;
	static constexpr uint32_t lowestNote = 24;								// Lowest midi note available C0 (DAC outputs 0v)
	static constexpr uint32_t highestNote = lowestNote + (12 * octaves);	// Highest midi note available 108 = C7 (DAC outputs 7.13v)

	enum channelNo {channelA = 0, channelB = 1};

	void Init();
	void NoteOnOff(const uint8_t midiNote, const bool on);
	void CalcEnvelopes();
	void RetriggerGates();
	void ProcessMidi();
	void Pitchbend(const uint16_t pitch);

	float pitchbend = 0.0f;
	static constexpr float pitchbendSemitones = 12.0f;
	uint8_t monoVoice = 0;				// Can be set in config to output just one voice (eg for calibrating)

	struct Channel {
		channelNo index;
		uint32_t counter = 0;			// Used to determine oldest note for note stealing
		volatile ADSR* adsr;			// pointer to ADC values for ADSR pots

		struct Voice {
			Voice(uint8_t index, volatile uint32_t* dac, volatile uint32_t* led) : index(index), envelope{dac, led} {};

			uint8_t index = 0;
			Envelope envelope;
			uint8_t midiNote = 0;
			uint32_t startTime = 0;		// time that note was started (for note stealing)

			void SetPitch(const channelNo chn);
			void SetPitch(const channelNo chn, const uint16_t dacOutput);
		} voice[4];

		Channel(channelNo chn, volatile ADSR* adsr, Voice v1, Voice v2, Voice v3, Voice v4)
		 : index{chn}, adsr{adsr}, voice{v1, v2, v3, v4} {};

	};

	// Initialise each channel's voices with pointers to the internal DAC and LED PWM setting
	Channel channel[2] = {
		{
			channelA,
			&adc.EnvA,
			{0, &DAC1->DHR12R1, &TIM3->CCR1},
			{1, &DAC3->DHR12R1, &TIM3->CCR2},
			{2, &DAC2->DHR12R1, &TIM3->CCR3},
			{3, &DAC1->DHR12R2, &TIM3->CCR4},
		},
		{
			channelB,
			&adc.EnvB,
			{0, &DAC4->DHR12R2, &TIM4->CCR1},
			{1, &DAC3->DHR12R2, &TIM4->CCR2},
			{2, &DAC4->DHR12R1, &TIM4->CCR3},
			{3, nullptr,        &(TIM4->CCR4)},				// B4 is handled with an external DAC
		}
	};

	struct Gate {
		uint8_t index;
		volatile uint32_t* gateODR;
		uint32_t gatePin;
		uint32_t gateRetrigger;		// Used to apply a gap in the gate signal when note stealing
		bool gateOn;

		void GateOn()	{ *gateODR |= gatePin;  gateOn = true; }
		void GateOff()	{ *gateODR &= ~gatePin; gateOn = false; }
	};

	Gate gates[4] = {
		{0, &GPIOD->ODR, GPIO_ODR_OD2},
		{1, &GPIOD->ODR, GPIO_ODR_OD3},
		{2, &GPIOD->ODR, GPIO_ODR_OD4},
		{3, &GPIOD->ODR, GPIO_ODR_OD5}
	};

	// Midi queue - as midi notes can interrupt envelope generation, queue them up here and process before envelope
	static constexpr uint32_t midiQueueSize = 20;
	bool pitchbendUpdated = false;
	struct {
		uint8_t noteVal;
		bool on;
	} midiQueue[midiQueueSize];
	uint32_t midiQueueRead = 0;			// circular buffer read/write heads
	uint32_t midiQueueWrite = 0;

};

extern VoiceManager voiceManager;
