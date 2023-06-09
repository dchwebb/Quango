#include "VoiceManager.h"
#include <limits>

VoiceManager voiceManager;

void VoiceManager::NoteOnOff(uint8_t midiNote, bool on)
{
	if (on) {
		// Locate next available note in each channel
		for (auto& chn: channel) {
			Channel::Voice* chnVoice = nullptr;
			bool noteStealing = false;
			for (auto& v: chn.voice) {
				if (v.envelope.gateState == Envelope::gateStates::off) {
					chnVoice = &v;
					break;
				}
			}

			// Voice stealing: overwrite oldest note playing
			if (chnVoice == nullptr) {
				noteStealing = true;
				uint32_t oldestStart = -1;
				for (auto& v: chn.voice) {
					if (v.startTime < oldestStart) {
						oldestStart = v.startTime;
						chnVoice = &v;
					}
				}
			}

			chnVoice->midiNote = midiNote;
			chnVoice->startTime = ++chn.counter;
			chnVoice->envelope.gateState = Envelope::gateStates::attack;
			chnVoice->SetPitch(chn.index);
			if (chn.index == channelNo::channelA) {
				if (noteStealing) {
					gates[chnVoice->index].gateRetrigger = 5;			// trigger a time before resetting gate
					gates[chnVoice->index].GateOff();
				} else {
					gates[chnVoice->index].GateOn();
				}
			}
		}

	} else {

		// Note Off
		for (auto& chn: channel) {
			for (auto& v: chn.voice) {
				if (v.midiNote == midiNote) {
					v.startTime = 0;
					v.envelope.gateState = Envelope::gateStates::release;
					gates[v.index].GateOff();
				}
			}
		}
	}
}

void VoiceManager::CalcEnvelopes()
{
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

void VoiceManager::Channel::Voice::SetPitch(channelNo chn)
{
	/*
	 * AD5676 command structure:
	 * CCCC AAAA DDDDDDD DDDDDDDD
	 * Command 0011 is update DAC channel.
	 * Channel selected with bottom three address bits
	 * Eg Update channel 5: 0011 0101 DDDDDDDD DDDDDDDD
	*/
	uint16_t dacOutput = 0xFFFF * (float)(std::min(std::max((float)midiNote + voiceManager.pitchbend, 24.0f), 96.0f) - 24) / 72;		// limit C1 to C7

	GPIOA->ODR &= ~GPIO_ODR_OD15;		// NSS low

	// Data must be written as bytes as sending a 32bit word will trigger a 16 bit send
	auto spi8Bit = reinterpret_cast<volatile uint8_t*>(&SPI1->DR);
	*spi8Bit = (uint8_t)(0b00110000 | (index + (4 * chn)));
	*spi8Bit = (uint8_t)(dacOutput >> 8);
	*spi8Bit = (uint8_t)(dacOutput & 0xFF);

	while ((SPI1->SR & SPI_SR_BSY) != 0 || (SPI1->SR & SPI_SR_FTLVL) != 0) {};

	GPIOA->ODR |= GPIO_ODR_OD15;		// NSS high
}
