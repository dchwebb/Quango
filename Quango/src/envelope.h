#pragma once

#include "initialisation.h"


struct ADSR {
	uint16_t attack;
	uint16_t decay;
	uint16_t sustain;
	uint16_t release;
};



//extern volatile ADSR adsr;

struct Envelope {
public:
	Envelope(volatile uint32_t* envDAC, volatile uint32_t* envLED)
	 : envDAC{envDAC}, envLED{envLED} {}

	Envelope() {}

	void calcEnvelope();							// Sets the DAC level for envelope

	volatile uint32_t* envDAC;
	volatile uint32_t* envLED;
private:
	float CordicExp(float x);

	const float     timeStep = 1.0f / SAMPLERATE;	// one time unit - corresponding to sample time

	float           attack = 800.0f;				// Store the ADSR values based on the pot values (mainly for debugging)
	float           sustain = 4095.0f;
	float           currentLevel = 0.0f;			// The current level of the envelope (held as a float for accuracy of calulculation)

	enum class      gateStates {off, attack, decay, sustain, release};
	gateStates      gateState = gateStates::off;

	ADSR adsr;										// Hold the channel ADSR

	void SetEnvelope(uint16_t value);


};

