#include "uartHandler.h"
#include "usb.h"

#if (USB_DEBUG)
UART uart;
#endif

UART::UART() {
	// Debug UART pin: PC12 = UART5_TX

	RCC->APB1ENR1 |= RCC_APB1ENR1_UART5EN;			// USART5 clock enable
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;			// GPIO C clock

	GPIOC->MODER &= ~GPIO_MODER_MODE12_0;			// Set alternate function on PC12
	GPIOC->AFR[1] |= 5 << GPIO_AFRH_AFSEL12_Pos;	// Alternate function on PC12 for UART5_TX is AF5

	// Calculations depended on oversampling mode set in CR1 OVER8. Default = 0: Oversampling by 16
	UART5->BRR |= SystemCoreClock / 230400;			// clk / desired_baud
	UART5->CR1 &= ~USART_CR1_M;						// 0: 1 Start bit, 8 Data bits, n Stop bit; 1: 1 Start bit, 9 Data bits, n Stop bit
	UART5->CR1 |= USART_CR1_RE;						// Receive enable
	UART5->CR1 |= USART_CR1_TE;						// Transmitter enable

	// Set up interrupts
	UART5->CR1 |= USART_CR1_RXNEIE;
	NVIC_SetPriority(UART5_IRQn, 6);				// Lower is higher priority
	NVIC_EnableIRQ(UART5_IRQn);

	UART5->CR1 |= USART_CR1_UE;						// UART Enable
}


void UART::SendChar(char c) {
	while ((UART5->ISR & USART_ISR_TXE_TXFNF) == 0);
	UART5->TDR = c;
}

void UART::SendString(const char* s) {
	char c = s[0];
	uint8_t i = 0;
	while (c) {
		while ((UART5->ISR & USART_ISR_TXE_TXFNF) == 0);
		UART5->TDR = c;
		c = s[++i];
	}
}

void UART::SendString(const std::string& s) {
	for (char c : s) {
		while ((UART5->ISR & USART_ISR_TXE_TXFNF) == 0);
		UART5->TDR = c;
	}
}


void UART::ProcessCommand()
{
	if (!commandReady) {
		return;
	}
	std::string_view cmd {command};

#if (USB_DEBUG)
	if (cmd.compare("printdebug\n") == 0) {
		usb.OutputDebug();

	} else if (cmd.compare("debugon\n") == 0) {
		extern volatile bool debugStart;
		debugStart = true;
		SendString("Debug activated\r\n");
	} else {
		SendString("Unrecognised command\r\n");
	}
#endif
	commandReady = false;
}


extern "C" {

// USART Decoder
void UART5_IRQHandler() {
#if (USB_DEBUG)
	if (!uart.commandReady) {
		const uint32_t recData = UART5->RDR;					// Note that 32 bits must be read to clear the receive flag
		uart.command[uart.cmdPos] = (char)recData; 				// accessing RDR automatically resets the receive flag
		if (uart.command[uart.cmdPos] == 10) {
			uart.commandReady = true;
			uart.cmdPos = 0;
		} else {
			uart.cmdPos++;
		}
	}
#endif
}
}
