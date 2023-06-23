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

	static constexpr float timeStep = 1.0f / SAMPLERATE;	// one time unit - corresponding to sample time

	float currentLevel = 0.0f;			// The current level of the envelope (held as a float for accuracy of calulculation)

public:

};

