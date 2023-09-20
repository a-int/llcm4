#pragma once
#include "stm32f4xx.h"
#include <stdbool.h>

extern volatile uint32_t delay_ms_time;
typedef enum { pllp_div2, pllp_div4, pllp_div6, pllp_div8 } ePLLP_arg;
typedef enum { APB_div1 = 0, APB_div2 = 0b100, APB_div4, APB_div8, APB_div16 } APB_divider;
typedef enum { AHB_div1 = 0, AHB_div2 = 0b1000, AHB_div4, AHB_div8, AHB_div16, \
              AHB_div64, AHB_div128, AHB_div256, AHB_div512 } AHB_divider;

void initSW(); //Serial Wire setup. NOTE unused pins set as analog
void init_clock_HSE(uint32_t M_arg, uint32_t N_arg, ePLLP_arg P_arg, AHB_divider AHB, APB_divider APB1, APB_divider APB2); /*initialize System clock to work at 100 MHz*/
void init_SysTick(uint32_t ticks); /* initialize SysTick to work at 1ms */
void delay_ms(uint32_t time); /*form a delay according to SysTick setup*/
