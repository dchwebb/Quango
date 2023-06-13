#pragma once

#include "initialisation.h"
#include <numbers>
#include <cmath>

class Calib;


class FFT {
	friend class Calib;

public:
	void Capture();
	void Activate();

	static constexpr uint32_t fftSamples = 1024;
	static constexpr uint32_t sinLUTSize = 1024;
	static constexpr uint32_t timerDefault = 10000;		// Default speed of sample capture (start fairly slow) sample rate is 85Mhz / clockDivider

	// FFT working variables
	float fftBuffer[2][fftSamples];						// holds raw samples captured in interrupt for FFT analysis

private:

	float cosBuffer[fftSamples];						// Stores working cosine part of FFT calculation
	uint16_t fftErrors = 0;

	void CalcFFT(float* sinBuffer, uint32_t samples);
	float HarmonicFreq(const float harmonicNumber);

public:
	constexpr auto CreateSinLUT()		// constexpr function to generate LUT in Flash
	{
		std::array<float, sinLUTSize> array {};
		for (uint32_t s = 0; s < sinLUTSize; ++s){
			array[s] = std::sin(s * 2.0f * std::numbers::pi / sinLUTSize);
		}
		return array;
	}

};


extern FFT fft;
