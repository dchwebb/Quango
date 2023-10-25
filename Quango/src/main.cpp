#include "calib.h"
#include "initialisation.h"
#include "usb.h"
#include "VoiceManager.h"
#include "configManager.h"

volatile uint32_t SysTickVal;
volatile ADCValues adc;


Config config{&calib.configSaver};		// Initialise configuration to handle saving and restoring calibration settings


extern "C" {
#include "interrupts.h"
}


extern uint32_t SystemCoreClock;
int main(void)
{
	SystemInit();						// Activates floating point coprocessor and resets clock
	InitClocks();						// Configure the clock and PLL
	InitHardware();
	voiceManager.Init();				// Initialises external DACs
	usb.InitUSB();
	config.RestoreConfig();

	while (1) {
		usb.cdc.ProcessCommand();		// Check for incoming USB serial commands
	}
}

