#include "VoiceManager.h"
#include "Calib.h"
#include <limits>
#include <cmath>

VoiceManager voiceManager;

void VoiceManager::Init()
{
	// Mute vhannel b voice 4 external DAC which starts on
	channel[channelB].voice[3].envelope.SetEnvelope(0);
}


void VoiceManager::NoteOnOff(const uint8_t midiNote, const bool on)
{
	// Queue midi notes for playback on envelope trigger
	midiQueue[midiQueueWrite].noteVal = midiNote;
	midiQueue[midiQueueWrite].on = on;
	if (++midiQueueWrite >= midiQueueSize) {
		midiQueueWrite = 0;
	}
}


void VoiceManager::ProcessMidi()
{
	while (midiQueueRead != midiQueueWrite)	{
		auto& note = midiQueue[midiQueueRead];
		if (++midiQueueRead >= midiQueueSize) {
			midiQueueRead = 0;
		}

		uint8_t chnVoice = 255;

		if (note.on) {
			if (monoVoice) {
				chnVoice = monoVoice - 1;			// Output only on one voice eg for calibrating

			} else {
				// Locate next available note in each channel
				for (auto& v: channel[channelA].voice) {
					if (v.envelope.gateState == Envelope::gateStates::off) {
						chnVoice = v.index;
						break;
					}
				}

				// Voice stealing: overwrite oldest note playing
				if (chnVoice == 255) {
					uint32_t oldestStart = std::numeric_limits<uint32_t>::max();
					for (auto& v: channel[channelA].voice) {
						if (v.startTime < oldestStart) {
							oldestStart = v.startTime;
							chnVoice = v.index;
						}
					}
				}
			}

			for (auto& chn: channel) {
				chn.voice[chnVoice].midiNote = note.noteVal;
				chn.voice[chnVoice].startTime = ++chn.counter;
				chn.voice[chnVoice].envelope.gateState = Envelope::gateStates::attack;
				chn.voice[chnVoice].SetPitch(chn.index);
			}

			// Check if gate status has changed
			if (!gates[chnVoice].gateOn) {
				gates[chnVoice].GateOn();
			} else {
				gates[chnVoice].gateRetrigger = 5;			// trigger a countdown before resetting gate
				gates[chnVoice].GateOff();
			}

		} else {

			// Note Off
			for (auto& chn: channel) {
				for (auto& v: chn.voice) {
					if (v.midiNote == note.noteVal) {
						v.startTime = 0;
						v.envelope.gateState = Envelope::gateStates::release;
					}
				}
			}

			// Check if gate status has changed
			for (auto& gate : gates) {
				auto& voiceA = channel[channelA].voice[gate.index].envelope.gateState;
				auto& voiceB = channel[channelB].voice[gate.index].envelope.gateState;
				if ((voiceA == Envelope::gateStates::release || voiceA == Envelope::gateStates::off) &&
						(voiceB == Envelope::gateStates::release || voiceB == Envelope::gateStates::off)) {
					gate.GateOff();
				}
			}
		}
	}
}


void VoiceManager::CalcEnvelopes()
{
	ProcessMidi();
	for (auto& chn: channel) {
		for (auto& v: chn.voice) {
			v.envelope.calcEnvelope(chn.adsr);
		}
	}
}


void VoiceManager::RetriggerGates()
{
	// Check if any gates are being paused when voice stealing
	for (auto& g: gates) {
		if (g.gateRetrigger > 0) {
			if (--g.gateRetrigger == 0) {
				g.GateOn();
			}
		}
	}
}


void VoiceManager::Channel::Voice::SetPitch(const channelNo chn)
{
	uint32_t oct = 0;
	if (chn == channelB) {
		// Octave Up
		if ((GPIOD->IDR & GPIO_IDR_ID0) != 0) {
			oct = 12;
		}
		if ((GPIOC->IDR & GPIO_IDR_ID12) != 0) {
			oct = -12;
		}
	}
	float note = std::clamp(static_cast<float>(midiNote + oct) + voiceManager.pitchbend, (float)lowestNote, (float)highestNote);		// limit C1 to C8

	const uint8_t calibOffset = std::round(note - lowestNote);
	note = note + calib.interpolatedOffsets[chn][index][calibOffset];

	// CV DAC outputs 5v which is amplified to 7.13v. This gives a range of 7.13 * 12 = 85.56 notes
	uint16_t dacOutput = std::min((uint32_t)(65536.0f * (note - (float)lowestNote) / 85.56f), 0xFFFFUL);
	SetPitch(chn, dacOutput);
}


void VoiceManager::Channel::Voice::SetPitch(const channelNo chn, const uint16_t dacOutput)
{
	// AD5676 command structure:
	// CCCC AAAA DDDDDDD DDDDDDDD
	// Command 0011 is update DAC channel.
	// Channel selected with bottom three address bits
	// Eg Update channel 5: 0011 0101 DDDDDDDD DDDDDDDD

	GPIOA->ODR &= ~GPIO_ODR_OD15;		// NSS low

	static volatile uint8_t& spi8Bit = (uint8_t&)(SPI1->DR);		// Pitch DAC cast to 8 bit value


	// Data must be written as bytes as sending a 32bit word will trigger a 16 bit send
	spi8Bit = (uint8_t)(0b00110000 | (index + (4 * chn)));
	spi8Bit = (uint8_t)(dacOutput >> 8);
	spi8Bit = (uint8_t)(dacOutput & 0xFF);

	while ((SPI1->SR & SPI_SR_BSY) != 0 || (SPI1->SR & SPI_SR_FTLVL) != 0) {};

	GPIOA->ODR |= GPIO_ODR_OD15;		// NSS high
}


void VoiceManager::Pitchbend(const uint16_t pitch)
{
	// Raw pitchbend data is 0-16384 centered at 8192
	pitchbend = (static_cast<float>(pitch - 8192) / 8192.0f) * pitchbendSemitones;
	for (auto& chn: channel) {
		for (auto& v: chn.voice) {
			if (v.envelope.gateState != Envelope::gateStates::off) {
				v.SetPitch(chn.index);
			}
		}
	}
}

