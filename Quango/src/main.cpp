#include "initialisation.h"
#include "usb.h"
#include "SerialHandler.h"
#include "MidiHandler.h"
#include "VoiceManager.h"

volatile uint32_t SysTickVal;

volatile ADCValues adc;
USBHandler usb;
SerialHandler serial(usb);
MidiHandler midi;

extern "C" {
#include "interrupts.h"
}


extern uint32_t SystemCoreClock;
int main(void)
{
	SystemInit();							// Activates floating point coprocessor and resets clock
	SystemClock_Config();					// Configure the clock and PLL
	SystemCoreClockUpdate();				// Update SystemCoreClock (system clock frequency) derived from settings of oscillators, prescalers and PLL
	InitSysTick();
	InitDAC();			// FIXME - calibrate
	InitIO();
	InitPWMTimer();
	InitADC1(&adc.PitchDetect, 2);
	InitADC3(reinterpret_cast<volatile uint16_t*>(&adc.EnvA), 4);
	InitADC4(&adc.ChannelBLevel, 5);
	InitMidiUART();
	InitSPI2();
	InitSPI1();

	InitEnvTimer();
	InitCordic();

	usb.InitUSB();

	while (1) {
		serial.Command();			// Check for incoming CDC commands
	}
}

