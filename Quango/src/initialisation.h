#pragma once

#include "stm32g4xx.h"
#include <algorithm>
#include <Array>

extern volatile uint32_t SysTickVal;


struct ADSR {
	uint16_t level;
	uint16_t attack;
	uint16_t decay;
	uint16_t sustain;
	uint16_t release;
};

struct ADCValues {
	uint16_t PitchDetect;
	ADSR EnvA;
	ADSR EnvB;
};

extern volatile ADCValues adc;
extern uint16_t calibZeroPos;

#define SYSTICK 1000						// 1ms
#define SAMPLERATE 48000.0f

#define DEBUG_ON1  GPIOD->ODR |= GPIO_ODR_OD0;		// Test pin 1 on
#define DEBUG_OFF1 GPIOD->ODR &= ~GPIO_ODR_OD0;		// Test pin 1 off
#define DEBUG_ON2  GPIOC->ODR |= GPIO_ODR_OD12;		// Test pin 2 on
#define DEBUG_OFF2 GPIOC->ODR &= ~GPIO_ODR_OD12;	// Test pin 2 off


void InitSystemClock();
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
void InitTunerTimer();

/*
void InitCoverageTimer();
*/
