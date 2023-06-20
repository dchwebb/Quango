#pragma once

#include "initialisation.h"
#include <cmath>

struct Envelope {
public:
	enum class      gateStates {off, attack, decay, sustain, release};

	Envelope(volatile uint32_t* envDAC, volatile uint32_t* envLED)
	 : envDAC{envDAC}, envLED{envLED} {}

	Envelope() {}

	void calcEnvelope(volatile ADSR* adsr);			// Sets the DAC level for envelope
	void SetEnvelope(const uint32_t value);

	volatile uint32_t* envDAC;
	volatile uint32_t* envLED;
	gateStates gateState = gateStates::off;

private:
	float CordicExp(float x);

	const float     timeStep = 1.0f / SAMPLERATE;	// one time unit - corresponding to sample time

	float           attack = 800.0f;				// Store the ADSR values based on the pot values (mainly for debugging)
	float           sustain = 4095.0f;
	float           currentLevel = 0.0f;			// The current level of the envelope (held as a float for accuracy of calulculation)

public:

};

