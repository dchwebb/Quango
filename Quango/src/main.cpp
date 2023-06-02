#include "initialisation.h"


volatile uint32_t SysTickVal;
volatile ADCValues ADC_array;


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
	InitDAC();
//	InitIO();
//	InitEnvTimer();
	InitADC1(&ADC_array.PitchDetect, 1);
	//InitADC1(reinterpret_cast<volatile uint16_t*>(&ADC_array.PitchDetect), 1);
//	InitUart();
//	InitCordic();
//
//	usb.InitUSB();

	while (1) {
//		serial.Command();			// Check for incoming CDC commands
	}
}

