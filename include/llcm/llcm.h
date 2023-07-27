#pragma once
#include "stm32f4xx.h"
#include <stdbool.h>

extern volatile uint32_t delay_ms_time;

void setupSW(); //Serial Wire setup. NOTE unused pins set as analog
void init_clock_100(); /*initialize System clock to work at 100 MHz*/
void init_SysTick(uint32_t ticks); /* initialize SysTick to work at 1ms */
void delay_ms(uint32_t time); /*form a delay according to SysTick setup*/
