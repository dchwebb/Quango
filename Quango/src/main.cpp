#include "calib.h"
#include "initialisation.h"
#include "usb.h"
#include "MidiHandler.h"
#include "VoiceManager.h"
#include "configManager.h"

volatile uint32_t SysTickVal;
volatile ADCValues adc;


/* TODO:
- interpolation on calibration offsets
*/


extern "C" {
#include "interrupts.h"
}


extern uint32_t SystemCoreClock;
int main(void)
{
	SystemInit();						// Activates floating point coprocessor and resets clock
	InitSystemClock();					// Configure the clock and PLL
	SystemCoreClockUpdate();			// Update SystemCoreClock (system clock frequency) derived from settings of oscillators, prescalers and PLL
	InitHardware();
	voiceManager.Init();				// Initialises external DACs
	usb.InitUSB();
	configManager.RestoreConfig();

	while (1) {
		usb.cdc.ProcessCommand();		// Check for incoming USB serial commands
	}
}

