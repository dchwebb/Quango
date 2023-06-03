#include "initialisation.h"


volatile uint32_t SysTickVal;

volatile struct ADCValues {
	uint16_t PitchDetect;
	uint16_t ChannelALevel;
	ADSR EnvA;
	uint16_t ChannelBLevel;
	ADSR EnvB;
} adc;


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
//	InitEnvTimer();
	InitPWMTimer();
	InitADC1(&adc.PitchDetect, 2);
	InitADC3(reinterpret_cast<volatile uint16_t*>(&adc.EnvA), 4);
	InitADC4(&adc.ChannelBLevel, 5);
//	InitUart();
//	InitCordic();
//
//	usb.InitUSB();

	while (1) {
//		serial.Command();			// Check for incoming CDC commands
	}
}

