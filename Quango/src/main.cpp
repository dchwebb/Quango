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


/*
 * AD5676 command structure:
 * CCCC AAAA DDDDDDD DDDDDDDD
 * Command 0011 is update DAC channel.
 * Channel selected with bottom three address bits
 * Eg Update channel 5: 0011 0101 DDDDDDDD DDDDDDDD

 * MCP48 commmand structure:
 * Address 0x00: Volatile DAC Wiper Register 0
 * Command: 0x00: Write data
 * Data: 12 bit
 * 24 bit message Structure: AAAAA CC X XXXX DDDDDDDDDDDD
 * So basically send a 24 bit word with the 12 bit data (0x0 - 0xFFF) right aligned
 */

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

