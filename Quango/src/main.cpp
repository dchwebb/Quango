#include "initialisation.h"
#include "usb.h"
#include "SerialHandler.h"
#include "MidiHandler.h"
#include "VoiceManager.h"
#include "Tuner.h"

volatile uint32_t SysTickVal;

volatile ADCValues adc;
USBHandler usb;
SerialHandler serial(usb);
MidiHandler midi;

int16_t vCalibOffset = 2047;
float vCalibScale = 1.0f;
uint16_t calibZeroPos = 2047;

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
	InitADC3(reinterpret_cast<volatile uint16_t*>(&adc.EnvA), 4);
	InitADC4(&adc.ChannelBLevel, 5);
	InitMidiUART();
	InitSPI2();
	InitSPI1();
	InitEnvTimer();
	InitTunerTimer();
	InitCordic();
	usb.InitUSB();

	calibZeroPos = CalcZeroSize();
	while (1) {
		serial.Command();				// Check for incoming CDC commands
	}
}

