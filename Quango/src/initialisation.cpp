#include "initialisation.h"

// 12MHz / 3(M) * 85(N) / 2(R) = 170MHz
#define PLL_M 0b010		// 0010: PLLM = 3
#define PLL_N 85
#define PLL_R 0			//  00: PLLR = 2, 01: PLLR = 4, 10: PLLR = 6, 11: PLLR = 8



void InitSystemClock(void) {
	// See page 236 for clock configuration
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;		// SYSCFG + COMP + VREFBUF + OPAMP clock enable
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;		// Enable Power Control clock
	PWR->CR5 &= ~PWR_CR5_R1MODE;				// Select the Range 1 boost mode

	RCC->CR |= RCC_CR_HSEON;					// HSE ON
	while ((RCC->CR & RCC_CR_HSERDY) == 0);		// Wait till HSE is ready

	// Configure PLL
	RCC->PLLCFGR = (PLL_M << RCC_PLLCFGR_PLLM_Pos) | (PLL_N << RCC_PLLCFGR_PLLN_Pos) | (PLL_R << RCC_PLLCFGR_PLLR_Pos) | (RCC_PLLCFGR_PLLSRC_HSE);
	RCC->CR |= RCC_CR_PLLON;					// Enable the main PLL
	RCC->PLLCFGR = RCC_PLLCFGR_PLLREN;			// Enable PLL R (drives AHB clock)
	while ((RCC->CR & RCC_CR_PLLRDY) == 0);		// Wait till the main PLL is ready

	// Configure Flash prefetch and wait state. NB STM32G473 is a category 3 device
	FLASH->ACR |= FLASH_ACR_LATENCY_4WS | FLASH_ACR_PRFTEN;
	FLASH->ACR &= ~FLASH_ACR_LATENCY_1WS;

	// The system clock must be divided by 2 using the AHB prescaler before switching to a higher system frequency.
	RCC->CFGR |= RCC_CFGR_HPRE_DIV2;			// HCLK = SYSCLK / 2
	RCC->CFGR |= RCC_CFGR_SW_PLL;				// Select the main PLL as system clock source

	// Wait till the main PLL is used as system clock source
	while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL);

	// Reset the AHB clock (previously divided by 2) and set APB clocks
	RCC->CFGR &= ~RCC_CFGR_HPRE_Msk;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;			// PCLK1 = HCLK / 1 (APB1)
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;			// PCLK2 = HCLK / 1 (APB2)
}


void InitSysTick()
{
	SysTick_Config(SystemCoreClock / SYSTICK);		// gives 1ms
	NVIC_SetPriority(SysTick_IRQn, 0);
}


void InitDAC()
{
	// Configure 4 DAC outputs PA4 and PA5 are regular DAC1 buffered outputs; PA2 and PB1 are DAC3 via OpAmp1 and OpAmp3 (Manual p.782)

	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;			// Enable GPIO Clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;			// Enable GPIO Clock
	RCC->AHB2ENR |= RCC_AHB2ENR_DAC1EN | RCC_AHB2ENR_DAC2EN | RCC_AHB2ENR_DAC3EN | RCC_AHB2ENR_DAC4EN;				// Enable DAC Clock

	DAC1->MCR &= ~DAC_MCR_MODE1_Msk;				// Set to normal mode: DAC1 channel1 is connected to external pin with Buffer enabled
	DAC1->CR |= DAC_CR_EN1;							// Enable DAC using PA4 (DAC_OUT1)

	DAC1->MCR &= ~DAC_MCR_MODE2_Msk;				// Set to normal mode: DAC1 channel2 is connected to external pin with Buffer enabled
	DAC1->CR |= DAC_CR_EN2;							// Enable DAC using PA5 (DAC_OUT2)

	DAC2->MCR &= ~DAC_MCR_MODE1_Msk;				// Set to normal mode: DAC2 channel1 is connected to external pin with Buffer enabled
	DAC2->CR |= DAC_CR_EN1;							// Enable DAC using PA6 (DAC_OUT1)

	// output triggered with DAC->DHR12R1 = x;

	// Opamp for DAC3 Channel 1: Follower configuration mode - output on PA2
	DAC3->MCR |= DAC_MCR_MODE1_0 | DAC_MCR_MODE1_1;	// 011: DAC channel1 is connected to on chip peripherals with Buffer disabled
	DAC3->CR |= DAC_CR_EN1;							// Enable DAC

	OPAMP1->CSR |= OPAMP_CSR_VMSEL;					// 11: Opamp_out connected to OPAMPx_VINM input
	OPAMP1->CSR |= OPAMP_CSR_VPSEL;					// 11: DAC3_CH1  connected to OPAMP1 VINP input
	OPAMP1->CSR |= OPAMP_CSR_OPAMPxEN;				// Enable OpAmp: voltage on pin OPAMPx_VINP is buffered to pin OPAMPx_VOUT (PA2)

	// Opamp for DAC3 Channel 2: Follower configuration mode - output on PB1
	DAC3->MCR |= DAC_MCR_MODE2_0 | DAC_MCR_MODE2_1;	// 011: DAC channel2 is connected to on chip peripherals with Buffer disabled
	DAC3->CR |= DAC_CR_EN2;							// Enable DAC

	OPAMP3->CSR |= OPAMP_CSR_VMSEL;					// 11: Opamp_out connected to OPAMPx_VINM input
	OPAMP3->CSR |= OPAMP_CSR_VPSEL;					// 11: DAC3_CH2  connected to OPAMP1 VINP input
	OPAMP3->CSR |= OPAMP_CSR_OPAMPxEN;				// Enable OpAmp: voltage on pin OPAMPx_VINP is buffered to pin OPAMPx_VOUT (PB1)

	// Opamp for DAC4 Channel 1: Follower configuration mode - output on PB12
	DAC4->MCR |= DAC_MCR_MODE1_0 | DAC_MCR_MODE1_1;	// 011: DAC channel1 is connected to on chip peripherals with Buffer disabled
	DAC4->CR |= DAC_CR_EN1;							// Enable DAC

	OPAMP4->CSR |= OPAMP_CSR_VMSEL;					// 11: Opamp_out connected to OPAMPx_VINM input
	OPAMP4->CSR |= OPAMP_CSR_VPSEL;					// 11: DAC4_CH1  connected to OPAMP1 VINP input
	OPAMP4->CSR |= OPAMP_CSR_OPAMPxEN;				// Enable OpAmp: voltage on pin OPAMPx_VINP is buffered to pin OPAMPx_VOUT (PB12)

	// Opamp for DAC4 Channel 2: Follower configuration mode - output on PA8
	DAC4->MCR |= DAC_MCR_MODE2_0 | DAC_MCR_MODE2_1;	// 011: DAC channel2 is connected to on chip peripherals with Buffer disabled
	DAC4->CR |= DAC_CR_EN2;							// Enable DAC

	OPAMP5->CSR |= OPAMP_CSR_VMSEL;					// 11: Opamp_out connected to OPAMPx_VINM input
	OPAMP5->CSR |= OPAMP_CSR_VPSEL;					// 11: DAC4_CH2  connected to OPAMP1 VINP input
	OPAMP5->CSR |= OPAMP_CSR_OPAMPxEN;				// Enable OpAmp: voltage on pin OPAMPx_VINP is buffered to pin OPAMPx_VOUT (PA8)

}


void InitIO()
{
	// MODER 00: Input mode, 01: General purpose output mode, 10: Alternate function mode, 11: Analog mode (reset state)
	// PUPDR 01: Pull-up; 10: Pull-down

	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;			// GPIO A clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;			// GPIO B clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;			// GPIO C clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;			// GPIO D clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOFEN;			// GPIO F clock

	//	Init Multiplexer pins PA1, PA3, PF2
	GPIOA->MODER &= ~GPIO_MODER_MODER1_1;			// PA1: Multiplexer A0 out
	GPIOA->MODER &= ~GPIO_MODER_MODER3_1;			// PA3: Multiplexer A1 out
	GPIOF->MODER &= ~GPIO_MODER_MODER2_1;			// PF2: Multiplexer A2 out

	// Init Gate outputs PD2 - PD5
	GPIOD->MODER &= ~GPIO_MODER_MODER2_1;			// PD2: Gate1 Out
	GPIOD->MODER &= ~GPIO_MODER_MODER3_1;			// PD3: Gate2 Out
	GPIOD->MODER &= ~GPIO_MODER_MODER4_1;			// PD4: Gate3 Out
	GPIOD->MODER &= ~GPIO_MODER_MODER5_1;			// PD5: Gate4 Out

	// Calibration buttons input
	GPIOB->MODER &= ~GPIO_MODER_MODER10;			// PB10: channel A
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD10_0;			// Activate pull-up
	GPIOB->MODER &= ~GPIO_MODER_MODER11;			// PB11: channel B
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD11_0;			// Activate pull-up

	// Debug pins PD0, PC12
	GPIOD->MODER &= ~GPIO_MODER_MODER0_1;			// PD0: Test pin 1
	GPIOC->MODER &= ~GPIO_MODER_MODER12_1;			// PC12: Test pin 2

}


void InitSPI1()
{
	// Controls AD5676 8 channel DAC
	// PB3: SPI1_SCK; PB4: SPI1_MISO; PB5: SPI1_MOSI; PA15: SPI1_NSS (AF5)
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;			// SPI1 clock enable
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;			// GPIO clocks

	// PB3: SPI1_SCK
	GPIOB->MODER  &= ~GPIO_MODER_MODE3_0;			// 10: Alternate function mode
	GPIOB->AFR[0] |= 5 << GPIO_AFRL_AFSEL3_Pos;		// Alternate Function 5 (SPI1)

	// PB5: SPI1_MOSI
	GPIOB->MODER  &= ~GPIO_MODER_MODE5_0;			// 10: Alternate function mode
	GPIOB->AFR[0] |= 5 << GPIO_AFRL_AFSEL5_Pos;		// Alternate Function 5 (SPI1)

	// PA15: SPI1_NSS (uses GPIO rather than hardware NSS which doesn't work with 24 bit data)
	GPIOA->MODER |= GPIO_MODER_MODE15_0;			// 01: Output mode
	GPIOA->MODER &= ~GPIO_MODER_MODE15_1;			// 01: Output mode

	// Configure SPI - baud rate tested working at /4 (42MHz) but run at /8 for now
	SPI1->CR1 |= SPI_CR1_MSTR;						// Master mode
	SPI1->CR1 |= SPI_CR1_BR_1;						// Baud rate (170Mhz/x): 000: /2; 001: /4; *010: /8; 011: /16; 100: /32; 101: /64
	SPI1->CR1 |= SPI_CR1_SSI;						// Internal slave select
	SPI1->CR1 |= SPI_CR1_SSM;						// Software NSS management
	SPI1->CR2 |= 0b111 << SPI_CR2_DS_Pos;			// Data Size: 0b111 = 8 bit

	SPI1->CR1 |= SPI_CR1_SPE;						// Enable SPI
}


void InitSPI2()
{
	// Controls MCP48CMB21 single channel DAC for channel B envelope 4
	// PA10: SPI2_MISO; PB13: SPI2_SCK; PB15: SPI2_MOSI; PD15: SPI2_NSS
	RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN;			// SPI2 clock enable
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIODEN;			// GPIO clocks

	// PB13: SPI2_SCK
	GPIOB->MODER  &= ~GPIO_MODER_MODE13_0;			// 10: Alternate function mode
	GPIOB->AFR[1] |= 5 << GPIO_AFRH_AFSEL13_Pos;	// Alternate Function 5 (SPI2)

	// PB15: SPI2_MOSI
	GPIOB->MODER  &= ~GPIO_MODER_MODE15_0;			// 10: Alternate function mode
	GPIOB->AFR[1] |= 5 << GPIO_AFRH_AFSEL15_Pos;	// Alternate Function 5 (SPI2)

	// PD15: SPI2_NSS
	GPIOD->MODER  &= ~GPIO_MODER_MODE15_1;			// 01: Output mode

	// Configure SPI
	SPI2->CR1 |= SPI_CR1_MSTR;						// Master mode
	SPI2->CR1 |= SPI_CR1_SSI;						// Internal slave select
	SPI2->CR1 |= SPI_CR1_BR_1;						// Baud rate (170Mhz/x): 000: /2; 001: /4; *010: /8; 011: /16; 100: /32; 101: /64
	SPI2->CR1 |= SPI_CR1_SSM;						// Software NSS management
	SPI2->CR2 |= 0b111 << SPI_CR2_DS_Pos;			// Data Size: 0b1011 = 12-bit; 0b111 = 8 bit

	SPI2->CR1 |= SPI_CR1_SPE;						// Enable SPI

	GPIOD->ODR &= ~GPIO_ODR_OD15;					// SPI2 NSS - this can be left low
}


void InitMidiUART()
{
	// PC11: UART4_RX (AF5) or USART3_RX (AF7)

	RCC->APB1ENR1 |= RCC_APB1ENR1_UART4EN;			// UART4 clock enable
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;			// GPIO port C

	GPIOC->MODER &= ~GPIO_MODER_MODE11_0;			// Set alternate function on PE0
	GPIOC->AFR[1] |= 5 << GPIO_AFRH_AFSEL11_Pos;	// Alternate function on PC11 for USART4_RX is AF5

	// By default clock source is muxed to peripheral clock 1 which is system clock
	// Calculations depended on oversampling mode set in CR1 OVER8. Default = 0: Oversampling by 16

	UART4->BRR = SystemCoreClock / 31250;			// clk / desired_baud
	UART4->CR1 &= ~USART_CR1_M;						// 0: 1 Start bit, 8 Data bits, n Stop bit; 	1: 1 Start bit, 9 Data bits, n Stop bit
	UART4->CR1 |= USART_CR1_RE;						// Receive enable
	UART4->CR2 |= USART_CR2_RXINV;					// Invert UART receive to allow use of inverting buffer

	// Set up interrupts
	UART4->CR1 |= USART_CR1_RXNEIE;
	NVIC_SetPriority(UART4_IRQn, 1);				// Lower is higher priority
	NVIC_EnableIRQ(UART4_IRQn);

	UART4->CR1 |= USART_CR1_UE;						// UART Enable
}


void InitPWMTimer()
{
	// TIM3: Channel 1: PE2 (AF2)
	// 		 Channel 2: PE3 (AF2)
	// 		 Channel 3: PE4 (AF2)
	// 		 Channel 4: PE5 (AF2)
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;			// Enable GPIO Clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;

	// Enable channels 1 - 4 PWM output pins on PE2-5
	GPIOE->MODER &= ~(GPIO_MODER_MODE2_0 | GPIO_MODER_MODE3_0 | GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0);
	GPIOE->AFR[0] |= GPIO_AFRL_AFSEL2_1 | GPIO_AFRL_AFSEL3_1 | GPIO_AFRL_AFSEL4_1 | GPIO_AFRL_AFSEL5_1;			// AF2

	TIM3->CCMR1 |= TIM_CCMR1_OC1PE;					// Output compare 1 preload enable
	TIM3->CCMR1 |= TIM_CCMR1_OC2PE;					// Output compare 2 preload enable
	TIM3->CCMR2 |= TIM_CCMR2_OC3PE;					// Output compare 3 preload enable
	TIM3->CCMR2 |= TIM_CCMR2_OC4PE;					// Output compare 4 preload enable

	TIM3->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);	// 0110: PWM mode 1 - In upcounting, channel 1 active if TIMx_CNT<TIMx_CCR1
	TIM3->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);	// 0110: PWM mode 1 - In upcounting, channel 2 active if TIMx_CNT<TIMx_CCR2
	TIM3->CCMR2 |= (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2);	// 0110: PWM mode 1 - In upcounting, channel 3 active if TIMx_CNT<TIMx_CCR3
	TIM3->CCMR2 |= (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2);	// 0110: PWM mode 1 - In upcounting, channel 3 active if TIMx_CNT<TIMx_CCR3

	TIM3->CCR1 = 0;									// Initialise PWM level to 0
	TIM3->CCR2 = 0;
	TIM3->CCR3 = 0;
	TIM3->CCR4 = 0;

	// Timing calculations: Clock = 170MHz / (PSC + 1) = 21.25m counts per second
	// ARR = number of counts per PWM tick = 4095
	// 21.25m / ARR ~= 5.2kHz of PWM square wave with 4095 levels of output

	TIM3->ARR = 4095;								// Total number of PWM ticks
	TIM3->PSC = 7;									// Should give ~5.2kHz
	TIM3->CR1 |= TIM_CR1_ARPE;						// 1: TIMx_ARR register is buffered
	TIM3->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);		// Capture mode enabled / OC1 signal is output on the corresponding output pin
	TIM3->EGR |= TIM_EGR_UG;						// 1: Re-initialize the counter and generates an update of the registers

	TIM3->CR1 |= TIM_CR1_CEN;						// Enable counter

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TIM4: PD12 TIM4_CH1 (AF2)
	// 		 PD13 TIM4_CH2 (AF2)
	// 		 PD14 TIM4_CH3 (AF2)
	// 		 PB9  TIM4_CH4 (AF2)
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;			// Enable GPIO Clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;			// Enable GPIO Clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;

	// Enable channels 1 - 3 PWM output pins on PD12-14
	GPIOD->MODER &= ~(GPIO_MODER_MODE12_0 | GPIO_MODER_MODE13_0 | GPIO_MODER_MODE14_0);
	GPIOD->AFR[1] |= GPIO_AFRH_AFSEL12_1 | GPIO_AFRH_AFSEL13_1 | GPIO_AFRH_AFSEL14_1;			// AF2

	// Enable channel 4 PWM output pin on PB9
	GPIOB->MODER &= ~GPIO_MODER_MODE9_0;
	GPIOB->AFR[1] |= GPIO_AFRH_AFSEL9_1;			// AF2

	TIM4->CCMR1 |= TIM_CCMR1_OC1PE;					// Output compare 1 preload enable
	TIM4->CCMR1 |= TIM_CCMR1_OC2PE;					// Output compare 2 preload enable
	TIM4->CCMR2 |= TIM_CCMR2_OC3PE;					// Output compare 3 preload enable
	TIM4->CCMR2 |= TIM_CCMR2_OC4PE;					// Output compare 4 preload enable

	TIM4->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);	// 0110: PWM mode 1 - In upcounting, channel 1 active if TIMx_CNT<TIMx_CCR1
	TIM4->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);	// 0110: PWM mode 1 - In upcounting, channel 2 active if TIMx_CNT<TIMx_CCR2
	TIM4->CCMR2 |= (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2);	// 0110: PWM mode 1 - In upcounting, channel 3 active if TIMx_CNT<TIMx_CCR3
	TIM4->CCMR2 |= (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2);	// 0110: PWM mode 1 - In upcounting, channel 3 active if TIMx_CNT<TIMx_CCR3

	TIM4->CCR1 = 0;									// Initialise PWM level to 0
	TIM4->CCR2 = 0;
	TIM4->CCR3 = 0;
	TIM4->CCR4 = 0;

	// Timing calculations: Clock = 170MHz / (PSC + 1) = 21.25m counts per second
	// ARR = number of counts per PWM tick = 2047
	// 21.25m / ARR ~= 5.2kHz of PWM square wave with 2047 levels of output

	TIM4->ARR = 4095;								// Total number of PWM ticks
	TIM4->PSC = 7;									// Should give ~5.2kHz
	TIM4->CR1 |= TIM_CR1_ARPE;						// 1: TIMx_ARR register is buffered
	TIM4->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);		// Capture mode enabled / OC1 signal is output on the corresponding output pin
	TIM4->EGR |= TIM_EGR_UG;						// 1: Re-initialize the counter and generates an update of the registers

	TIM4->CR1 |= TIM_CR1_CEN;						// Enable counter
}



//	Setup Timer 2 on an interrupt to trigger sample output
void InitEnvTimer() {
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;			// Enable Timer 2
	TIM2->PSC = 16;									// Set prescaler
	TIM2->ARR = 500; 								// Set auto reload register - 170Mhz / (PSC + 1) / ARR = ~20kHz

	TIM2->DIER |= TIM_DIER_UIE;						// DMA/interrupt enable register
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 2);					// Lower is higher priority

	TIM2->CR1 |= TIM_CR1_CEN;
	TIM2->EGR |= TIM_EGR_UG;						//  Re-initializes counter and generates update of registers
}


//	Setup Timer 5 on an interrupt to trigger tuner sample capture
void InitTunerTimer()
{
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;			// Enable Timer 5
	TIM5->PSC = 1;									// Set prescaler
	TIM5->ARR = 500; 								// Set auto reload register - 170Mhz / (PSC + 1) / ARR = ~10kHz

	TIM5->DIER |= TIM_DIER_UIE;						// DMA/interrupt enable register
	NVIC_EnableIRQ(TIM5_IRQn);
	NVIC_SetPriority(TIM5_IRQn, 0);					// Lower is higher priority
}


void InitAdcPins(ADC_TypeDef* ADC_No, std::initializer_list<uint8_t> channels) {
	uint8_t sequence = 1;

	for (auto channel: channels) {
		// Set conversion sequence to order ADC channels are passed to this function
		if (sequence < 5) {
			ADC_No->SQR1 |= channel << ((sequence) * 6);
		} else if (sequence < 10) {
			ADC_No->SQR2 |= channel << ((sequence - 5) * 6);
		} else if (sequence < 15) {
			ADC_No->SQR3 |= channel << ((sequence - 10) * 6);
		} else {
			ADC_No->SQR4 |= channel << ((sequence - 15) * 6);
		}

		// 000: 3 cycles, 001: 15 cycles, 010: 28 cycles, 011: 56 cycles, 100: 84 cycles, 101: 112 cycles, 110: 144 cycles, 111: 480 cycles
		if (channel < 10)
			ADC_No->SMPR1 |= 0b010 << (3 * channel);
		else
			ADC_No->SMPR2 |= 0b010 << (3 * (channel - 10));

		sequence++;
	}
}


/*--------------------------------------------------------------------------------------------
Configure ADC Channels to be converted:
0	PE7 ADC3_IN4		Env1 Attack
1	PE8 ADC345_IN6		Env1 Decay
2	PE9 ADC3_IN2		Env1 Sustain
3	PE10 ADC345_IN14	Env1 Release

4	PE11 ADC345_IN15	Env2 Attack
5	PE12 ADC345_IN16	Env2 Decay
6	PE14 ADC4_IN1		Env2 Sustain
7	PE15 ADC4_IN2		Env2 Release

8	PC0 ADC12_IN6		Pitch Detect Audio
*/

void InitADC1(volatile uint16_t* buffer, uint16_t channels)
{
	// Initialize Clocks
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;
	RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
	RCC->CCIPR |= RCC_CCIPR_ADC12SEL_1;				// 00: no clock, 01: PLL P clk clock, *10: System clock

	DMA1_Channel1->CCR &= ~DMA_CCR_EN;
	DMA1_Channel1->CCR |= DMA_CCR_CIRC;				// Circular mode to keep refilling buffer
	DMA1_Channel1->CCR |= DMA_CCR_MINC;				// Memory in increment mode
	DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0;			// Peripheral size: 8 bit; 01 = 16 bit; 10 = 32 bit
	DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0;			// Memory size: 8 bit; 01 = 16 bit; 10 = 32 bit
	DMA1_Channel1->CCR |= DMA_CCR_PL_0;				// Priority: 00 = low; 01 = Medium; 10 = High; 11 = Very High

	DMA1->IFCR = 0x3F << DMA_IFCR_CGIF1_Pos;		// clear all five interrupts for this stream

	DMAMUX1_Channel0->CCR |= 5; 					// DMA request MUX input 5 = ADC1 (See p.427)
	DMAMUX1_ChannelStatus->CFR |= DMAMUX_CFR_CSOF0; // Channel 1 Clear synchronization overrun event flag

	ADC1->CR &= ~ADC_CR_DEEPPWD;					// Deep power down: 0: ADC not in deep-power down	1: ADC in deep-power-down (default reset state)
	ADC1->CR |= ADC_CR_ADVREGEN;					// Enable ADC internal voltage regulator

	// Wait until voltage regulator settled
	volatile uint32_t wait_loop_index = (SystemCoreClock / (100000UL * 2UL));
	while (wait_loop_index != 0UL) {
		wait_loop_index--;
	}
	while ((ADC1->CR & ADC_CR_ADVREGEN) != ADC_CR_ADVREGEN) {}

	ADC12_COMMON->CCR |= ADC_CCR_CKMODE;			// adc_hclk/4 (Synchronous clock mode)
	ADC1->CFGR |= ADC_CFGR_CONT;					// 1: Continuous conversion mode for regular conversions
	ADC1->CFGR |= ADC_CFGR_OVRMOD;					// Overrun Mode 1: ADC_DR register is overwritten with the last conversion result when an overrun is detected.
	ADC1->CFGR |= ADC_CFGR_DMACFG;					// 0: DMA One Shot Mode selected, 1: DMA Circular Mode selected
	ADC1->CFGR |= ADC_CFGR_DMAEN;					// Enable ADC DMA

	// For scan mode: set number of channels to be converted
	ADC1->SQR1 |= (channels - 1);

	// Start calibration
	ADC1->CR &= ~ADC_CR_ADCALDIF;					// Calibration in single ended mode
	ADC1->CR |= ADC_CR_ADCAL;
	while ((ADC1->CR & ADC_CR_ADCAL) == ADC_CR_ADCAL) {};


	/*
	Configure ADC Channels to be converted:
	PC0 ADC12_IN6		Pitch Detect Audio
	PC2 ADC12_IN8		ChannelALevel
	*/

	InitAdcPins(ADC1, {6, 8});

	// Enable ADC
	ADC1->CR |= ADC_CR_ADEN;
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) {}

	DMAMUX1_ChannelStatus->CFR |= DMAMUX_CFR_CSOF0; // Channel 1 Clear synchronization overrun event flag
	DMA1->IFCR = 0x3F << DMA_IFCR_CGIF1_Pos;		// clear all five interrupts for this stream

	DMA1_Channel1->CNDTR |= channels;				// Number of data items to transfer (ie size of ADC buffer)
	DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR));	// Configure the peripheral data register address 0x40022040
	DMA1_Channel1->CMAR = (uint32_t)(buffer);		// Configure the memory address (note that M1AR is used for double-buffer mode) 0x24000040

	DMA1_Channel1->CCR |= DMA_CCR_EN;				// Enable DMA and wait
	wait_loop_index = (SystemCoreClock / (100000UL * 2UL));
	while (wait_loop_index != 0UL) {
		wait_loop_index--;
	}

	ADC1->CR |= ADC_CR_ADSTART;						// Start ADC
}




void InitADC3(volatile uint16_t* buffer, uint16_t channels)
{
	// Initialize Clocks
	//RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	//RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;
	RCC->AHB2ENR |= RCC_AHB2ENR_ADC345EN;
	RCC->CCIPR |= RCC_CCIPR_ADC345SEL_1;			// 00: no clock, 01: PLL P clk clock, *10: System clock

	DMA1_Channel2->CCR &= ~DMA_CCR_EN;
	DMA1_Channel2->CCR |= DMA_CCR_CIRC;				// Circular mode to keep refilling buffer
	DMA1_Channel2->CCR |= DMA_CCR_MINC;				// Memory in increment mode
	DMA1_Channel2->CCR |= DMA_CCR_PSIZE_0;			// Peripheral size: 8 bit; 01 = 16 bit; 10 = 32 bit
	DMA1_Channel2->CCR |= DMA_CCR_MSIZE_0;			// Memory size: 8 bit; 01 = 16 bit; 10 = 32 bit
	DMA1_Channel2->CCR |= DMA_CCR_PL_0;				// Priority: 00 = low; 01 = Medium; 10 = High; 11 = Very High

	DMA1->IFCR = 0x3F << DMA_IFCR_CGIF2_Pos;		// clear all five interrupts for this stream

	DMAMUX1_Channel1->CCR |= 37; 					// DMA request MUX input 37 = ADC3 (See p.426)
	DMAMUX1_ChannelStatus->CFR |= DMAMUX_CFR_CSOF1; // Channel 2 Clear synchronization overrun event flag

	ADC3->CR &= ~ADC_CR_DEEPPWD;					// Deep power down: 0: ADC not in deep-power down	1: ADC in deep-power-down (default reset state)
	ADC3->CR |= ADC_CR_ADVREGEN;					// Enable ADC internal voltage regulator

	// Wait until voltage regulator settled
	volatile uint32_t wait_loop_index = (SystemCoreClock / (100000UL * 2UL));
	while (wait_loop_index != 0UL) {
		wait_loop_index--;
	}
	while ((ADC3->CR & ADC_CR_ADVREGEN) != ADC_CR_ADVREGEN) {}

	ADC345_COMMON->CCR |= ADC_CCR_CKMODE;			// adc_hclk/4 (Synchronous clock mode)
	ADC3->CFGR |= ADC_CFGR_CONT;					// 1: Continuous conversion mode for regular conversions
	ADC3->CFGR |= ADC_CFGR_OVRMOD;					// Overrun Mode 1: ADC_DR register is overwritten with the last conversion result when an overrun is detected.
	ADC3->CFGR |= ADC_CFGR_DMACFG;					// 0: DMA One Shot Mode selected, 1: DMA Circular Mode selected
	ADC3->CFGR |= ADC_CFGR_DMAEN;					// Enable ADC DMA

	// For scan mode: set number of channels to be converted
	ADC3->SQR1 |= (channels - 1);

	// Start calibration
	ADC3->CR &= ~ADC_CR_ADCALDIF;					// Calibration in single ended mode
	ADC3->CR |= ADC_CR_ADCAL;
	while ((ADC3->CR & ADC_CR_ADCAL) == ADC_CR_ADCAL) {};


	/*--------------------------------------------------------------------------------------------
	Configure ADC Channels to be converted:
	PE7 ADC3_IN4		Env1 Attack
	PE8 ADC345_IN6		Env1 Decay
	PE9 ADC3_IN2		Env1 Sustain
	PE10 ADC345_IN14	Env1 Release
	*/

	InitAdcPins(ADC3, {4, 6, 2, 14});

	// Enable ADC
	ADC3->CR |= ADC_CR_ADEN;
	while ((ADC3->ISR & ADC_ISR_ADRDY) == 0) {}

	DMAMUX1_ChannelStatus->CFR |= DMAMUX_CFR_CSOF1; // Channel 2 Clear synchronization overrun event flag
	DMA1->IFCR = 0x3F << DMA_IFCR_CGIF2_Pos;		// clear all five interrupts for this stream

	DMA1_Channel2->CNDTR |= channels;				// Number of data items to transfer (ie size of ADC buffer)
	DMA1_Channel2->CPAR = (uint32_t)(&(ADC3->DR));	// Configure the peripheral data register address 0x40022040
	DMA1_Channel2->CMAR = (uint32_t)(buffer);		// Configure the memory address (note that M1AR is used for double-buffer mode) 0x24000040

	DMA1_Channel2->CCR |= DMA_CCR_EN;				// Enable DMA and wait
	wait_loop_index = (SystemCoreClock / (100000UL * 2UL));
	while (wait_loop_index != 0UL) {
		wait_loop_index--;
	}

	ADC3->CR |= ADC_CR_ADSTART;						// Start ADC
}


void InitADC4(volatile uint16_t* buffer, uint16_t channels)
{
	// Initialize Clocks
	//RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	//RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;
	//RCC->AHB2ENR |= RCC_AHB2ENR_ADC345EN;
	//RCC->CCIPR |= RCC_CCIPR_ADC345SEL_1;			// 00: no clock, 01: PLL P clk clock, *10: System clock

	DMA1_Channel3->CCR &= ~DMA_CCR_EN;
	DMA1_Channel3->CCR |= DMA_CCR_CIRC;				// Circular mode to keep refilling buffer
	DMA1_Channel3->CCR |= DMA_CCR_MINC;				// Memory in increment mode
	DMA1_Channel3->CCR |= DMA_CCR_PSIZE_0;			// Peripheral size: 8 bit; 01 = 16 bit; 10 = 32 bit
	DMA1_Channel3->CCR |= DMA_CCR_MSIZE_0;			// Memory size: 8 bit; 01 = 16 bit; 10 = 32 bit
	DMA1_Channel3->CCR |= DMA_CCR_PL_0;				// Priority: 00 = low; 01 = Medium; 10 = High; 11 = Very High

	DMA1->IFCR = 0x3F << DMA_IFCR_CGIF3_Pos;		// clear all five interrupts for this stream

	DMAMUX1_Channel2->CCR |= 38; 					// DMA request MUX input 38 = ADC4 (See p.426)
	DMAMUX1_ChannelStatus->CFR |= DMAMUX_CFR_CSOF2; // Channel 2 Clear synchronization overrun event flag

	ADC4->CR &= ~ADC_CR_DEEPPWD;					// Deep power down: 0: ADC not in deep-power down	1: ADC in deep-power-down (default reset state)
	ADC4->CR |= ADC_CR_ADVREGEN;					// Enable ADC internal voltage regulator

	// Wait until voltage regulator settled
	volatile uint32_t wait_loop_index = (SystemCoreClock / (100000UL * 2UL));
	while (wait_loop_index != 0UL) {
		wait_loop_index--;
	}
	while ((ADC4->CR & ADC_CR_ADVREGEN) != ADC_CR_ADVREGEN) {}

	//ADC345_COMMON->CCR |= ADC_CCR_CKMODE;			// adc_hclk/4 (Synchronous clock mode)
	ADC4->CFGR |= ADC_CFGR_CONT;					// 1: Continuous conversion mode for regular conversions
	ADC4->CFGR |= ADC_CFGR_OVRMOD;					// Overrun Mode 1: ADC_DR register is overwritten with the last conversion result when an overrun is detected.
	ADC4->CFGR |= ADC_CFGR_DMACFG;					// 0: DMA One Shot Mode selected, 1: DMA Circular Mode selected
	ADC4->CFGR |= ADC_CFGR_DMAEN;					// Enable ADC DMA

	// For scan mode: set number of channels to be converted
	ADC4->SQR1 |= (channels - 1);

	// Start calibration
	ADC4->CR &= ~ADC_CR_ADCALDIF;					// Calibration in single ended mode
	ADC4->CR |= ADC_CR_ADCAL;
	while ((ADC4->CR & ADC_CR_ADCAL) == ADC_CR_ADCAL) {};


	/*--------------------------------------------------------------------------------------------
	Configure ADC Channels to be converted:
	PD9 ADC4_IN13 		ChannelBLevel
	PE11 ADC345_IN15	Env2 Attack
	PE12 ADC345_IN16	Env2 Decay
	PE14 ADC4_IN1		Env2 Sustain
	PE15 ADC4_IN2		Env2 Release
	*/

	InitAdcPins(ADC4, {13, 15, 16, 1, 2});

	// Enable ADC
	ADC4->CR |= ADC_CR_ADEN;
	while ((ADC4->ISR & ADC_ISR_ADRDY) == 0) {}

	DMAMUX1_ChannelStatus->CFR |= DMAMUX_CFR_CSOF2; // Channel 3 Clear synchronization overrun event flag
	DMA1->IFCR = 0x3F << DMA_IFCR_CGIF3_Pos;		// clear all five interrupts for this stream

	DMA1_Channel3->CNDTR |= channels;				// Number of data items to transfer (ie size of ADC buffer)
	DMA1_Channel3->CPAR = (uint32_t)(&(ADC4->DR));	// Configure the peripheral data register address 0x40022040
	DMA1_Channel3->CMAR = (uint32_t)(buffer);		// Configure the memory address (note that M1AR is used for double-buffer mode) 0x24000040

	DMA1_Channel3->CCR |= DMA_CCR_EN;				// Enable DMA and wait
	wait_loop_index = (SystemCoreClock / (100000UL * 2UL));
	while (wait_loop_index != 0UL) {
		wait_loop_index--;
	}

	ADC4->CR |= ADC_CR_ADSTART;						// Start ADC
}


void InitCordic()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
}


/*
//	Setup Timer 9 to count clock cycles for coverage profiling
void InitCoverageTimer() {
	RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;				// Enable Timer
	TIM9->PSC = 100;
	TIM9->ARR = 65535;

	TIM9->DIER |= TIM_DIER_UIE;						// DMA/interrupt enable register
	NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
	NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 2);		// Lower is higher priority

}



*/



