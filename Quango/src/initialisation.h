#pragma once

#include "stm32g4xx.h"
#include <algorithm>

extern volatile uint32_t SysTickVal;


struct ADSR {
	uint16_t attack;
	uint16_t decay;
	uint16_t sustain;
	uint16_t release;
};

struct ADCValues {
	uint16_t PitchDetect;
	uint16_t ChannelALevel;
	ADSR EnvA;
	uint16_t ChannelBLevel;
	ADSR EnvB;
};

extern volatile ADCValues adc;

#define SYSTICK 1000						// 1ms
#define SAMPLERATE 48000.0f
//#define ADC_BUFFER_LENGTH 9

#define DEBUG_ON  GPIOB->ODR |= GPIO_ODR_OD9;
#define DEBUG_OFF GPIOB->ODR &= ~GPIO_ODR_OD9;

void SystemClock_Config();
void InitSysTick();
void InitDAC();
void InitIO();
void InitEnvTimer();
void InitADC1(volatile uint16_t* buffer, uint16_t channels);
void InitADC3(volatile uint16_t* buffer, uint16_t channels);
void InitADC4(volatile uint16_t* buffer, uint16_t channels);
void InitUart();
void InitCordic();
void InitPWMTimer();
void InitMidiUART();
void InitSPI2();
void InitSPI1();
/*
void InitCoverageTimer();
void InitDebounceTimer();
*/
