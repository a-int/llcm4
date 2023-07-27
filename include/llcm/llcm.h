//#include <cstdint>
#pragma once
#include <stdbool.h>
#include "stm32f4xx.h"

void setBitN(GPIO_TypeDef* gpiox, const uint8_t bitN);
// void resetBitN(GPIO_TypeDef* gpiox, const uint8_t bitN);
void toggleBitN(GPIO_TypeDef* gpiox, const uint8_t bitN); /*toogle the bit N in GPIO X a*/
bool stateBitN(GPIO_TypeDef* gpiox, uint8_t bitN);
void setupSW();  //Serial Wire setup. NOTE unused pins set as analog

void init_clock_100();             /*initialize System clock to work at 100 MHz*/
void init_SysTick(uint32_t ticks); /* initialize SysTick to work at 1ms */
