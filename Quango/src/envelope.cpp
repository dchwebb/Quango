#include "envelope.h"
#include <cmath>
#include <cstring>

// Create LED brightness look up table as constexpr so will be stored in flash
constexpr auto CreateLEDLUT()		// create exponential brightness curve to better reflect perceived loudness
{
	std::array<uint16_t, 4096> array {};
	for (uint32_t i = 0; i < 4096; ++i){
		array[i] = (uint16_t)(4096.0f * std::pow((float)i / 4096.0f, 3.0f));
	}
	return array;
}
constexpr std::array<uint16_t, 4096> ledBrightness = CreateLEDLUT();


void Envelope::SetEnvelope(const uint32_t value) {

	if (envDAC != nullptr) {
		*envDAC = value;
	} else {
		// Channel B voice 4 is the only one controlled by an external DAC (MCP48) on SPI2
		// MCP48 commmand structure:
		// Address 0x00: Volatile DAC Wiper Register 0
		// Command: 0x00: Write data
		// Data: 12 bit
		// 24 bit message Structure: AAAAA CC X XXXX DDDDDDDDDDDD

		GPIOD->ODR &= ~GPIO_ODR_OD15;					// SPI2 NSS - Low

		// Data must be written as bytes as sending a 32bit word will trigger a 16 bit send
		volatile uint8_t* spi8Bit = (uint8_t*)(&SPI2->DR);

		*spi8Bit = uint8_t{0};
		*spi8Bit = (uint8_t)(value >> 8);
		*spi8Bit = (uint8_t)(value & 0xFF);

		while ((SPI2->SR & SPI_SR_BSY) != 0 || (SPI2->SR & SPI_SR_FTLVL) != 0) {};

		GPIOD->ODR |= GPIO_ODR_OD15;					// SPI2 NSS - High
	}
	*envLED = ledBrightness[value];
}


void Envelope::calcEnvelope(volatile ADSR* adsr)
{
	float level = currentLevel;

	static constexpr float reciprocal4096 = 1.0f / 4096.0f;		// Store reciprocal to avoid divisions

	switch (gateState) {
	case gateStates::off:
		break;

	case gateStates::attack: {
		// fullRange = value of fully charged capacitor; comparitor value is 4096 where cap is charged enough to trigger decay phase
		static constexpr float fullRange = 5000.0f;

		// scales attack pot to allow more range at low end of pot, exponentially longer times at upper end
		const float maxDurationMult = 0.9f * 0.578;			// 0.578 allows duration to be set in seconds

		// RC value - attackScale represents R component; maxDurationMult represents capacitor size (Reduce rc for a steeper curve)
		float rc = std::pow(adsr->attack * reciprocal4096, 3.0f) * maxDurationMult;		// Using a^3 for fast approximation for measured charging rate (^2.9)

		if (rc != 0.0f) {
			/*
			 * Long hand calculations:
			 * Capacitor charging equation: Vc = Vs(1 - e ^ -t/RC)
			 * 1. Invert capacitor equation to calculate current 'time' based on y/voltage value
			 * float ln = std::log(1.0f - (currentLevel / fullRange));
			 * float xPos = -rc * ln;
			 * float newXPos = xPos + timeStep;		// Add timeStep (based on sample rate) to current X position
			 *
			 * 2. Calculate exponential of time for capacitor charging equation
			 * float exponent = -newXPos / rc;
			 * float newYPos = 1.0f - std::exp(exponent);
			 * currentLevel = newYPos * fullRange;
			 */

			level = fullRange - (fullRange - level) * CordicExp(-timeStep / rc);

		} else {
			level = fullRange;
		}

		if (level >= 4095.0f) {
			level = 4095.0f;
			gateState = gateStates::decay;
		}

		break;

	}

	case gateStates::decay: {
		// scales decay pot to allow more range at low end of pot, exponentially longer times at upper end
		const float maxDurationMult = 5.28f * 0.227f;		// to scale maximum delay time

		// RC value - decayScale represents R component; maxDurationMult represents capacitor size
		float rc = std::pow(static_cast<float>(adsr->decay) * reciprocal4096, 2.0f) * maxDurationMult;		// Use x^2 as approximation for measured x^2.4

		if (rc != 0.0f && level > adsr->sustain) {
			/*
			 * Long hand calculations:
			 * Capacitor discharge equation: Vc = Vo * e ^ -t/RC
			 * 1. Invert capacitor discharge equation to calculate current 'time' based on y/voltage
			 * float yHeight = 4096.0f - sustain;		// Height of decay curve
			 * float xPos = -rc * std::log((currentLevel - sustain) / yHeight);
			 * float newXPos = xPos + timeStep;
			 *
			 * 2. Calculate exponential of time for capacitor discharging equation
			 * float exponent = -newXPos / rc;
			 * float newYPos = std::exp(exponent);		// Capacitor discharging equation
			 * currentLevel = (newYPos * yHeight) + sustain;
			 */

			level = adsr->sustain + (level - adsr->sustain) * CordicExp(-timeStep / rc);

		} else {
			level = 0.0f;
		}

		if (level <= adsr->sustain + 1.5f) {				// add a little extra to avoid getting stuck in infinitely small decrease
			level = adsr->sustain;
			gateState = gateStates::sustain;
		}

		break;
	}

	case gateStates::sustain:
		level = adsr->sustain;
		break;

	case gateStates::release:
		if (level > 0.0f) {
			if (adsr->release < 4080) { 					// If release is set to max, hold the note indefinitely for drones/calibration etc
				const float maxDurationMult = 1.15f;		// to scale maximum delay time

				// RC value - decayScale represents R component; maxDurationMult represents capacitor size
				float rc = std::pow(static_cast<float>(adsr->release) * reciprocal4096, 2.0f) * maxDurationMult;

				if (rc != 0.0f && level > 1.0f) {
					/*
					 * Long hand calculations:
					 * float xPos = -rc * std::log(currentLevel / 4096.0f);
					 * float newXPos = xPos + timeStep;
					 * float newYPos = std::exp(-newXPos / rc);
					 * currentLevel = newYPos * 4096.0f;
					 */

					level = level * CordicExp(-timeStep / rc);
				} else {
					level = 0.0f;
				}
			}
		} else {
			gateState = gateStates::off;
		}
		break;
	}

	//if (currentLevel != level || envDAC == nullptr) {			// FIXME - hack to test envB4 DAC
	if (currentLevel != level) {
		currentLevel = level;
		SetEnvelope(static_cast<uint32_t>(currentLevel * static_cast<float>(adsr->level) * reciprocal4096));
	}
}


float Envelope::CordicExp(float x)
{
	// use CORDIC sinh function and generate e^x = sinh(x) + cosh(x)
	CORDIC->CSR = (6 << CORDIC_CSR_FUNC_Pos) | 		// 0: Cos, 1: Sin, 2: Phase, 3: Modulus, 4: Arctan, 5: cosh, 6: sinh, 7: Arctanh, 8: ln, 9: Square Root
			CORDIC_CSR_SCALE_0 |					// Must be 1 for sinh
			CORDIC_CSR_NRES |						// 2 Results as we need both sinh and cosh
			(6 << CORDIC_CSR_PRECISION_Pos);		// Set precision to 6 (gives 6 * 4 = 24 iterations in 6 clock cycles)

	// convert float to q1_31 format scaling x by 1/2 at the same time
	int q31;
	if (x < -1.118f) {
		q31 = (int)((x + 1.0f) * 1073741824.0f);	// as range of x is limited to -1.118 to +1.118 reduce exponent by e^-1 (note that only values from around -1.75 to 0 used in this mechanism)
	} else {
		q31 = (int)(x * 1073741824.0f);
	}

	//volatile float etest = std::exp(x);

	CORDIC->WDATA = q31;

	// convert values back to floats scaling by * 2 at the same time
	float sinh = (float)((int)CORDIC->RDATA) / 1073741824.0f;	// command will block until RDATA is ready - no need to poll RRDY flag
	float cosh = (float)((int)CORDIC->RDATA) / 1073741824.0f;
	float res = sinh + cosh;
	if (x < -1.118f) {
		return res * 0.3678794411714f;				// multiply by e^-1 to correct range offset
	} else {
		return res;
	}
}

