void SysTick_Handler(void)
{
	SysTickVal++;
}


void TIM2_IRQHandler()
{
	TIM2->SR &= ~TIM_SR_UIF;
	tuner.CheckStart();							// Check if calibration buttons are pressed to activate tuning
	voiceManager.CalcEnvelopes();
	voiceManager.RetriggerGates();				// Check if any gates paused during note stealing need to be reactivated
}


// Main sample capture
void TIM5_IRQHandler(void)
{
	TIM5->SR &= ~TIM_SR_UIF;					// clear UIF flag
	tuner.Capture();
}


uint32_t debugUartOR = 0;
void UART4_IRQHandler(void)
{
	// MIDI Decoder
	if (UART4->ISR & USART_ISR_RXNE_RXFNE) {
		midi.serialHandler(UART4->RDR);
	} else {
		// Overrun
		UART4->ICR |= USART_ICR_ORECF;			// The ORE bit is reset by setting the ORECF bit in the USART_ICR register
		++debugUartOR;
	}
}


void USB_LP_IRQHandler()
{
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

