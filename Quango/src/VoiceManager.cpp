#include "VoiceManager.h"

VoiceManager voiceManager;

void VoiceManager::NoteOnOff(uint8_t midiNote, bool on)
{
	if (on) {
		// Locate next available note in each channel
		for (auto& chn: channel) {
			Channel::Voice* chnVoice;
			for (auto& v: chn.voice) {
				if (v.start == 0) {
					chnVoice = &v;
					break;
				}
			}
			if (chnVoice != nullptr) {				// FIXME - not necessary once note stealing implemented
				chnVoice->midiNote = midiNote;
				chnVoice->start = ++chn.counter;
				chnVoice->envelope.noteOn = true;
				//chnVoice->SetPitch();
				if (chn.index == channelNo::channelA) {
					gates[chnVoice->index].GateOn();
				}
			}
		}

	} else {

		// Note Off
		for (auto& chn: channel) {
			for (auto& v: chn.voice) {
				if (v.midiNote == midiNote) {
					v.start = 0;
					v.envelope.noteOn = false;
					gates[v.index].GateOff();
				}
			}
		}
	}
}

void VoiceManager::calcEnvelopes()
{
	for (auto& chn: channel) {
		for (auto& v: chn.voice) {
			v.envelope.calcEnvelope(chn.adsr);
		}
	}
}


void VoiceManager::Channel::Voice::SetPitch()
{
	/*
	 * AD5676 command structure:
	 * CCCC AAAA DDDDDDD DDDDDDDD
	 * Command 0011 is update DAC channel.
	 * Channel selected with bottom three address bits
	 * Eg Update channel 5: 0011 0101 DDDDDDDD DDDDDDDD
	*/

	uint16_t dacOutput = 0xFFFF * (float)(std::min(std::max((float)currentNote + channelNotes[channel - 1].pitchbend, 24.0f), 96.0f) - 24) / 72;		// limit C1 to C7

	uint8_t* spi8Bit = (uint8_t*)(&SPI1->DR);
	//uint16_t* spi16Bit = (uint16_t*)(&SPI1->DR);
	GPIOA->ODR &= ~GPIO_ODR_OD15;

	// Output test signal
	*spi8Bit = (uint8_t)0b00110000 | index;
	*spi8Bit = (uint8_t)(dacOutput >> 8);
	*spi8Bit = (uint8_t)(dacOutput & 0xFF);

	while ((SPI1->SR & SPI_SR_BSY) != 0 || (SPI1->SR & SPI_SR_FTLVL) != 0) {};

	GPIOA->ODR |= GPIO_ODR_OD15;
}
