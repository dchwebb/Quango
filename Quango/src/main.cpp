#include "calib.h"
#include "initialisation.h"
#include "usb.h"
#include "SerialHandler.h"
#include "MidiHandler.h"
#include "VoiceManager.h"

volatile uint32_t SysTickVal;

volatile ADCValues adc;



int16_t vCalibOffset = 2047;
float vCalibScale = 1.0f;
uint16_t calibZeroPos = 2047;
bool outputUSBDebug = false;
extern "C" {
#include "interrupts.h"
}


inline uint16_t CalcZeroSize() {			// returns ADC size that corresponds to 0v
	return (4096 - vCalibOffset) / vCalibScale;
}

extern uint32_t SystemCoreClock;
int main(void)
{
	SystemInit();						// Activates floating point coprocessor and resets clock
	InitSystemClock();					// Configure the clock and PLL
	SystemCoreClockUpdate();			// Update SystemCoreClock (system clock frequency) derived from settings of oscillators, prescalers and PLL
	InitSysTick();
	InitDAC();							// FIXME - calibrate
	InitIO();
	InitPWMTimer();
	InitADC1(&adc.PitchDetect, 2);
	InitADC3(reinterpret_cast<volatile uint16_t*>(&adc.EnvA.attack), 4);
	InitADC4(&adc.EnvB.level, 5);
	InitMidiUART();
	InitSPI2();
	InitSPI1();
	InitEnvTimer();
	InitTunerTimer();
	InitCordic();
	usb.InitUSB();

	calibZeroPos = CalcZeroSize();
	while (1) {
		usb.cdc.ProcessCommand();		// Check for incoming USB serial commands

#if (USB_DEBUG)
		if (outputUSBDebug) {
			outputUSBDebug = false;
			usb.OutputDebug();
		}
#endif
	}
}

