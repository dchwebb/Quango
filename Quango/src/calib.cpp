#include "calib.h"
#include "configManager.h"
#include <cstring>

Calib calib;


void Calib::Capture()
{
	bool samplesReady = false;
	if (mode == FFT) {
		auto floatBuffer = (float* const)&(fft.fftBuffer);
		static constexpr uint32_t halfNumSamples = FFT::fftSamples / 2;

		// Check whether to add sample to buffer 1, both or buffer 2 to create two buffers of 1024 samples with 512 samples overlapping
		if (bufferPos < FFT::fftSamples) {
			floatBuffer[bufferPos] = 2047.0f - static_cast<float>(adc.PitchDetect);
		}
		if (bufferPos >= halfNumSamples) {
			floatBuffer[bufferPos + halfNumSamples] = 2047.0f - static_cast<float>(adc.PitchDetect);
		}

		if (++bufferPos == FFT::fftSamples + halfNumSamples) {
			samplesReady = true;
		}
	} else {
		++timer;

		if (overZero && static_cast<int>(adc.PitchDetect) < 2047 - 100) {
			overZero = false;
		}
		if (!overZero && adc.PitchDetect > 2047) {
			overZero = true;
			zeroCrossings[bufferPos] = timer;
			if (++bufferPos == zeroCrossings.size()) {
				samplesReady = true;
			}
		}

		// If no signal found abort and display no signal message
		if (timer > 200000) {
			samplesReady = true;
		}
	}

	if (samplesReady) {
		TIM5->CR1 &= ~TIM_CR1_CEN;			// Disable the sample acquisiton timer
		CalcFreq();
	}
}


void Calib::End()
{
	for (auto& btn : calibBtn) {
		btn.downCount = 0;
		btn.upCount = 0;
	}
	TIM5->CR1 &= ~TIM_CR1_CEN;				// Disable the sample acquisiton timer
	running = false;

	// If sounding mute the last played note (may be set to 4 if successfully completed - mute voice 3)
	voiceManager.channel[calibchannel].voice[calibVoice == 4 ? 3 : calibVoice].envelope.SetEnvelope(0);
}


bool Calib::CheckStart()
{
	// Check for long/short calibration button press (PB10 - channel A, PB11 - channel B)
	bool started = false;
	for (auto& btn : calibBtn) {
		if ((*btn.btnIDR & btn.btnPin) == 0) {					// Check for button down
			if (++btn.downCount > 15000 && !btn.longPress && !running) {	// Check for long press
				// Switch on all LEDs to show user in long count mode
				btn.longPress = true;
				for (auto& voice : voiceManager.channel[btn.channel].voice) {
					*voice.envelope.envLED = 0xFFF;
				}
			}
			btn.upCount = 0;
		} else if (btn.downCount) {								// Previously pressed down
			++btn.upCount;										// Check button has been released long enough to ensure not a bounce
		}
	}

	if (calibBtn[0].upCount > 5 || calibBtn[1].upCount > 5) {
		if (running) {
			End();
		} else {
			calibchannel = calibBtn[0].downCount ? VoiceManager::channelA : VoiceManager::channelB;		// Prioritise button one if both pressed

			if (calibBtn[calibchannel].longPress) {
				ClearOffsets(calibchannel, true, true);			// Blank previous calibration offsets, play animation and save
			} else {
				started = true;
			}

			for (auto& btn : calibBtn) {						// Clear button counters
				btn.downCount = 0;
				btn.upCount = 0;
				btn.longPress = false;
			}
		}
	}


	if (started) {
		running = true;
		calibStart = SysTickVal;

		// set Envelopes to silent
		for (auto& c : voiceManager.channel) {
			for (auto& v : c.voice) {
				v.envelope.SetEnvelope(0);
				v.envelope.gateState = Envelope::gateStates::off;
			}
		}
		// Ensure gates are off
		for (auto& g : voiceManager.gates) {
			g.GateOff();
		}

		// Initialise calibration state machine information
		calibVoice = 0;
		calibNote = calibNoteStart;
		calibOctave = 0;
		calibCount = 0;;
		currFreq = 27.5f;					// Guess the frequency
		calibErrors = 0;

		ClearOffsets(calibchannel, false, false);			// Blank previous calibration offsets
		Activate(true);						// start tuner
	}
	return running;
}


void Calib::ClearOffsets(VoiceManager::channelNo chn, bool animate, bool saveConfig)
{
	for (uint8_t v = 0; v < 4; ++v) {
		for (uint8_t o = 0; o < VoiceManager::octaves; ++o) {
			calibOffsets[chn][v][o] = 0.0f;
		}
	}
	if (saveConfig) {
		config.SaveConfig();				// Only save config when cleared from button - no point if calibrating
	}

	// Clear LEDs
	if (animate) {
		int32_t dimTimer[4] = {10240, 8192, 6144, 4096};
		while (dimTimer[0] > 1) {
			for (auto& voice : voiceManager.channel[calibchannel].voice) {
				*voice.envelope.envLED = std::max(std::min(--dimTimer[voice.index], 4095L), 0L);
				volatile uint32_t delay = 0;
				while (delay < 100) {
					++delay;
				}
			}
		}
	}
}


void Calib::Multiplexer(VoiceManager::channelNo chn, uint8_t voice)
{
	// Configure multiplexer for correct voice; Multiplexer pins PA1, PA3, PF2
	uint8_t bits = voice + (4 * chn);
	GPIOA->ODR &= ~GPIO_ODR_OD1;
	GPIOA->ODR &= ~GPIO_ODR_OD3;
	GPIOF->ODR &= ~GPIO_ODR_OD2;
	if (bits & 0b001) {
		GPIOA->ODR |= GPIO_ODR_OD1;
	}
	if (bits & 0b010) {
		GPIOA->ODR |= GPIO_ODR_OD3;
	}
	if (bits & 0b100) {
		GPIOF->ODR |= GPIO_ODR_OD2;
	}
}



void Calib::Activate(bool startTimer)
{
	// Set pitch of oscillator
	voiceManager.channel[calibchannel].voice[calibVoice].midiNote = calibNote;
	voiceManager.channel[calibchannel].voice[calibVoice].SetPitch(calibchannel);

	// Make calibration audible
	if (calibVoice > 0) {
		voiceManager.channel[calibchannel].voice[calibVoice - 1].envelope.SetEnvelope(0);		// Silence previously calibrated voice
	}
	voiceManager.channel[calibchannel].voice[calibVoice].envelope.SetEnvelope(
			calibchannel ==	VoiceManager::channelA ? adc.EnvA.level : adc.EnvB.level);			// Set calibration volume from mixer slider

	convBlink = !convBlink;
	*(voiceManager.channel[calibchannel].voice[calibVoice].envelope.envLED) = convBlink ? 0xFFF : 0;


	// Set input audio multiplexer to correct channel and voice
	Multiplexer(calibchannel, calibVoice);

	if (mode == FFT) {
		if (currFreq > 800.0f) {
			TIM5->ARR = (FFT::timerDefault / 2) + sampleRateAdj;
		} else if (currFreq < 50.0f) {
			TIM5->ARR = (FFT::timerDefault * 2) + sampleRateAdj;
		} else {
			TIM5->ARR = FFT::timerDefault + sampleRateAdj;
		}
	} else {
		// Get current value of ADC (assume channel A for now)
		const uint32_t currVal = adc.PitchDetect;
		overZero = currVal > 2047;
		timer = 0;
		TIM5->ARR = zeroCrossRate;
	}

	bufferPos = 0;

	if (startTimer) {
		TIM5->CR1 |= TIM_CR1_CEN;
	}
}


void Calib::CalcFreq()
{
	float frequency = 0.0f;
	const uint32_t start = SysTickVal;

	if (mode == FFT) {
		// Phase adjusted FFT: FFT on two overlapping buffers; where the 2nd half of buffer 1 is the 1st part of buffer 2
		// Calculate the fundamental bin looking for a magnitude over a threshold, then adjust by the phase difference of the two FFTs

		static constexpr uint32_t fftBinCount = FFT::fftSamples / 2;

		fft.CalcFFT(fft.fftBuffer[0], FFT::fftSamples);			// Carry out FFT on first buffer

		// Find first significant harmonic
		uint32_t fundMag = 0;
		uint32_t maxMag = 0;
		uint32_t maxBin = 0;
		bool localMax = false;				// True once magnitude of bin is large enough to count as fundamental

		// Locate maximum hypoteneuse
		for (uint32_t i = 1; i < fftBinCount; ++i) {
			const float hypotenuse = std::hypot(fft.fftBuffer[0][i], fft.cosBuffer[i]);
			if (hypotenuse > maxMag) {
				maxMag = hypotenuse;
			}
		}

		// Locate first hypoteuse that is large enough relative to the maximum to count as fundamental
		for (uint32_t i = 1; i < fftBinCount; ++i) {
			const float hypotenuse = std::hypot(fft.fftBuffer[0][i], fft.cosBuffer[i]);
			if (localMax) {
				if (hypotenuse > fundMag) {
					fundMag = hypotenuse;
					maxBin = i;
				} else {
					break;
				}
			} else if (hypotenuse > magThreshold && hypotenuse > static_cast<float>(maxMag) * 0.5f) {
				localMax = true;
				fundMag = hypotenuse;
				maxBin = i;
			}
		}

		if (maxBin) {

			const float phase0 = atan(fft.cosBuffer[maxBin] / fft.fftBuffer[0][maxBin]);

			fft.CalcFFT(fft.fftBuffer[1], FFT::fftSamples);				// Carry out FFT on buffer 2 (overwrites cosine results from first FFT)

			const float phase1 = atan(fft.cosBuffer[maxBin] / fft.fftBuffer[1][maxBin]);
			float phaseAdj = (phase0 - phase1) / M_PI;			// normalise phase adjustment

			// handle phase wrapping
			if (phaseAdj < -0.5f) {
				phaseAdj += 1.0f;
			}
			if (phaseAdj > 0.5f) {
				phaseAdj -= 1.0f;
			}

			// When a signal is almost exactly between two bins the first and second FFT can disagree
			// Use the direction of the phase adjustment to correct
			const uint32_t hyp11 = std::hypot(fft.fftBuffer[1][maxBin + 0], fft.cosBuffer[maxBin + 0]);
			const uint32_t hyp12 = std::hypot(fft.fftBuffer[1][maxBin + 1], fft.cosBuffer[maxBin + 1]);
			if (hyp12 > hyp11 && phaseAdj < 0.0f) {		// Correct for situations where each FFT disagrees about the fundamental bin
				++maxBin;
			}

			frequency = fft.HarmonicFreq(static_cast<float>(maxBin) + phaseAdj);

			// Possibly due to rounding errors at around 50% phase adjustments the cycle rate can be out by a cycle - abort and shift sampling rate
			if (phaseAdj > 0.47f || phaseAdj < -0.47f) {
				if (std::fabs(frequency - currFreq) / currFreq > 0.05f) {
					sampleRateAdj += currFreq < 60 ? 32 : 2;
					frequency = 0.0f;
				}
			}
		}

	} else {
		// Zero crossing mode
		if (bufferPos == zeroCrossings.size()) {					// Check that a signal was found

			float diff = zeroCrossings[1] - zeroCrossings[0];		// Get first time difference between zero crossings
			uint32_t stride = 1;									// Allow pitch detection where multiple zero crossings in cycle
			uint32_t noMatch = 0;									// When enough failed matches increase stride
			uint32_t matchCount = 0;

			uint32_t i;
			for (i = 1; i < zeroCrossings.size() - stride; ++i) {
				float tempDiff = zeroCrossings[i + stride] - zeroCrossings[i];

				if (tempDiff - diff < 0.05f * diff) {
					diff = (diff + tempDiff) / 2.0f;				// Apply some damping to average out differences
					++matchCount;
				} else {
					++noMatch;
					if (noMatch > 3 && stride < 10) {				// After three failures increase stride length and restart loop
						++stride;
						diff = zeroCrossings[stride] - zeroCrossings[0];
						i = 0;
						matchCount = 0;
						noMatch = 0;
					}
				}
			}

			if (matchCount > 2) {
				frequency = FreqFromPos(diff);
			}
		}
	}

	if (frequency > 0.0f) {

		calibFrequencies[calibCount++] = frequency;

		if (calibCount == 3) {

			// normalise the frequency differences - below gives a value of 1/12 for a one note difference
			float diff1 = std::abs(log2(calibFrequencies[0] / calibFrequencies[1]));
			float diff2 = std::abs(log2(calibFrequencies[1] / calibFrequencies[2]));

			// All frequencies are within half a note of each other
			if (diff1 < 0.5f && diff2 < 0.5f) {
				// Get average frequency
				currFreq = (calibFrequencies[0] + calibFrequencies[1] + calibFrequencies[2]) / 3.0f;

				// Formula to get musical note from frequency is (ln(freq) - ln(16.3516)) / ln(2 ^ (1/12))
				// Where 16.35 is frequency of low C and return value is semi-tones from low C
				constexpr float numRecip = 1.0f / log(pow(2.0f, 1.0f / 12.0f));		// Store reciprocal to avoid division
				constexpr float logBase = log(16.35160f);
				float currNote = (log(frequency) - logBase) * numRecip;

				// If first note establish nearest A so that offset can be applied to other notes
				if (calibVoice == 0 && calibNote == calibNoteStart) {
					calibOffset = 0;
					while (currNote + calibOffset < calibNoteStart - VoiceManager::octaves && calibOffset < 72) {
						calibOffset += 12;
					}
				}
				float noteDiff = (calibNote - calibOffset) - currNote;
				calibOffsets[calibchannel][calibVoice][calibOctave] = noteDiff;
			}

			// Move to next octave
			calibNote += 12;
			++calibOctave;
			calibCount = 0;
			currFreq *= 2;					// Set the next frequency so the timer rate can be optimised

			if (calibNote > VoiceManager::highestNote) {
				if (++calibVoice < 4) {
					calibNote = calibNoteStart;
					calibOctave = 0;
					currFreq /= 32;			// reset predicted frequency dividing by 2^5
				}
			}
		}


		lastValid = SysTickVal;

	} else {
		++calibErrors;
	}

	// Check if calibration complete
	if (calibVoice == 4) {
		calibTime = SysTickVal - calibStart;
		config.SaveConfig();
		InterpolateOffsets();
		End();
	} else {
		Activate(true);
	}

}


float Calib::FreqFromPos(const uint16_t pos)
{
	// returns frequency of signal based on number of samples wide the signal is versus the sampling rate
	return static_cast<float>(SystemCoreClock) / (pos * (TIM5->PSC + 1) * (TIM5->ARR + 1));
}


void Calib::InterpolateOffsets()
{
	// Populate interpolated calibration data from octaves
	for (uint32_t channel = 0; channel < 2; ++channel) {
		for (uint32_t voice = 0; voice < 4; ++voice) {
			for (uint32_t note = 0; note < Calib::midiNoteCount; ++note) {
				// Raw calibration from  A0 (33) to A6 (117)
				// Midi note will be     C0 (24) to C7 (108)
				// Interpolated calib position = midiNote - 24
				const uint32_t midiNote = note + VoiceManager::lowestNote;
				const float calibOctave = static_cast<float>(midiNote - Calib::calibNoteStart) / 12.0f;

				if (midiNote <= Calib::calibNoteStart) {
					calib.interpolatedOffsets[channel][voice][note] = calib.calibOffsets[channel][voice][0];
				} else {
					const uint32_t octaveLow = std::floor(calibOctave);
					const uint32_t octaveHigh = std::min(octaveLow + 1, VoiceManager::octaves - 1);
					const float calibLow = calib.calibOffsets[channel][voice][octaveLow];
					const float calibHigh = calib.calibOffsets[channel][voice][octaveHigh];
					calib.interpolatedOffsets[channel][voice][note] = std::lerp(calibLow, calibHigh, calibOctave - octaveLow);
				}
			}
		}
	}


}


//float Tuner::FreqFromMidiNote(const uint8_t note)
//{
//	return 16.35160 * std::pow(2.0f, note / 12.0f);
//}


