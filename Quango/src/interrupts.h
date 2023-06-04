void SysTick_Handler(void)
{
	SysTickVal++;
}

uint16_t triOut = 0;

void TIM2_IRQHandler()
{
	TIM2->SR &= ~TIM_SR_UIF;
//	envelopes.calcEnvelopes();

	// Output test signal
	SPI1->DR = triOut;
	if (++triOut > 4095) {
		triOut = 0;
	}
}

uint32_t debugUartOR = 0;
uint32_t midiTest[255];
uint8_t midiCounter = 0;

void UART4_IRQHandler(void) {
	// MIDI Decoder
	if (UART4->ISR & USART_ISR_RXNE_RXFNE) {
		//usb.midi.serialHandler(UART8->RDR); 			// accessing DR automatically resets the receive flag
		midiTest[midiCounter++] = UART4->RDR;
	} else {
		// Overrun
		UART4->ICR |= USART_ICR_ORECF;					// The ORE bit is reset by setting the ORECF bit in the USART_ICR register
		++debugUartOR;
	}
}

void USB_LP_IRQHandler() {
	usb.USBInterruptHandler();
}

void NMI_Handler(void) {}

void HardFault_Handler(void) {
	while (1) {}
}

void MemManage_Handler(void) {
	while (1) {}
}

void BusFault_Handler(void) {
	while (1) {}
}

void UsageFault_Handler(void) {
	while (1) {}
}

void SVC_Handler(void) {}

void DebugMon_Handler(void) {}

void PendSV_Handler(void) {}

