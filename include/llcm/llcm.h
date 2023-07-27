#pragma once
#include "stm32f4xx.h"
#include <stdbool.h>

void setupSW(); //Serial Wire setup. NOTE unused pins set as analog
void init_clock_100(); /*initialize System clock to work at 100 MHz*/
void init_SysTick(uint32_t ticks); /* initialize SysTick to work at 1ms */
