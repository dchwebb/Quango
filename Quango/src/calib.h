#pragma once

#include "initialisation.h"
#include "fft.h"
#include "VoiceManager.h"
#include "CDCHandler.h"

class Calib {
	friend class CDCHandler;
public:
	enum tunerMode {FFT, ZeroCrossing};

	void Capture();									// Called from timer interrupt
	void Activate(bool startTimer);					// Tuning mode started
	void CalcFreq();								// Processes samples once collected
	bool CheckStart();								// check if calibration button is pressed
	uint32_t SerialiseConfig(uint8_t** buff);
	uint32_t StoreConfig(uint8_t* buff);
	void ClearOffsets(VoiceManager::channelNo chn, bool animate = false);

	float calibOffsets[2][4][7];					// Calibration offsets for channel | voice | octave
	bool running = false;
	tunerMode mode = FFT;

private:

	float FreqFromPos(const uint16_t pos);
	void Multiplexer(VoiceManager::channelNo chn, uint8_t voice);
	void End();

	uint32_t bufferPos = 0;							// Capture buffer position
	float currFreq = 0.0f;							// estimate of current frequency to optimise timer interval
	uint32_t lastValid = 0;							// Store time we last saw a good signal to show 'No signal' as appropriate
	bool convBlink = false;							// Show a flashing cursor each time a conversion has finished

	// Calibration state machine settings
	static constexpr uint8_t calibNoteStart = 33;	// Start with MIDI note 33 which should be A1
	VoiceManager::channelNo calibchannel;			// channel being calibrated
	uint8_t calibVoice;								// voice 0-3 being calibrated
	uint8_t calibNote;								// current MIDI note being calibrated
	uint8_t calibOctave;							// octave being calibrated
	uint8_t calibCount;								// number of calibration passes per voice to be averaged
	uint8_t calibOffset;							// offset octave of frequency found from expected octave (eg A0 vs A1)
	float calibFrequencies[3];						// each octave tuning pass averages three measurements

	uint32_t calibStart = 0;						// For timing calibratrion
	uint32_t calibTime;								// store calibration time in ms
	uint32_t calibErrors = 0;

	// Calibration button handling
	struct {
		uint32_t channel;
		volatile uint32_t* btnIDR;					// Address of calibration button GPIO IDR register
		uint32_t btnPin;					// Value to test for button down
		uint32_t upCount = 0;						// Used to debounce calibration button and check for long presses
		uint32_t downCount = 0;
		bool longPress = false;						// Calibration button long press mode
	} calibBtn[2]  = {
			{0, &GPIOB->IDR, GPIO_IDR_ID10},
			{1, &GPIOB->IDR, GPIO_IDR_ID11}
	};

	// Phase Adjusted FFT settings
	int8_t sampleRateAdj = 0;						// Used to make small adjustments to fft sample rate to avoid phase errors
	uint32_t magThreshold = 8000;					// Threshold at which a bin is significant enough to count as fundamental

	// Zero crossing settings
	static constexpr uint32_t zeroCrossRate = 100;	// sample rate is 85Mhz / clockDivider
	std::array<uint32_t, 6> zeroCrossings;			// Holds the timestamps in samples of each upward zero crossing
	bool overZero = false;							// Stores current direction of waveform
	uint32_t timer  = 0;

};

extern Calib calib;

