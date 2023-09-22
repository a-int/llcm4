/**
*  @file llcm.h
*/
#pragma once
#include <stdbool.h>
#include "stm32f4xx.h"

extern volatile uint32_t delay_ms_time;
/** 
* llcm.h
* The enumeration used for quick selection of PLL P argument
* @warning RCC_PLLPCFGR_PLLP_POS must be used to write proper value to register
*/
typedef enum {
  pllp_div2,  ///< P = 2. Used to write 0b00 to RCC->PLLCFGR register to PLLP position
  pllp_div4,  ///< P = 4. Used to write 0b01 to RCC->PLLCFGR register to PLLP position
  pllp_div6,  ///< P = 6. Used to write 0b10 to RCC->PLLCFGR register to PLLP position
  pllp_div8,  ///< P = 8. Used to write 0b11 to RCC->PLLCFGR register to PLLP position
} ePLLP_arg;

/** 
* llcm.h
* The enumeration used for quick selection of APB1/APB2 divider
* @warning RCC_CFGR_PREP1_POS or RCC_CFGR_PREP2_POS must be used to write proper value to register
*/
typedef enum {
  APB_div1 = 0,      ///< APB = 1.  Used to write 0b000 to RCC->CFGR register to appropriate PREP position
  APB_div2 = 0b100,  ///< APB = 2.  Used to write 0b100 to RCC->CFGR register to appropriate PREP position
  APB_div4,          ///< APB = 4.  Used to write 0b101 to RCC->CFGR register to appropriate PREP position
  APB_div8,          ///< APB = 8.  Used to write 0b110 to RCC->CFGR register to appropriate PREP position
  APB_div16          ///< APB = 16. Used to write 0b111 to RCC->CFGR register to appropriate PREP position
} APB_divider;

/** 
* llcm.h
* The enumeration used for quick selection of AHB divider
* @warning RCC_CFGR_HPRE_POS must be used to write proper value to register
*/
typedef enum {
  AHB_div1 = 0,       ///< AHB = 1. Writes 0b0000 to RCC->CFGR register to HPRE position
  AHB_div2 = 0b1000,  ///< AHB = 2. Writes 0b1000 to RCC->CFGR register to HPRE position
  AHB_div4,           ///< AHB = 4. Writes 0b1001 to RCC->CFGR register to HPRE position
  AHB_div8,           ///< AHB = 8. Writes 0b1010 to RCC->CFGR register to HPRE position
  AHB_div16,          ///< AHB = 16. Writes 0b1011 to RCC->CFGR register to HPRE position
  AHB_div64,          ///< AHB = 64. Writes 0b1100 to RCC->CFGR register to HPRE position
  AHB_div128,         ///< AHB = 128. Writes 0b1101 to RCC->CFGRR register to HPRE position
  AHB_div256,         ///< AHB = 256. Writes 0b1110 to RCC->CFGRR register to HPRE position
  AHB_div512          ///< AHB = 512. Writes 0b1111 to RCC->CFGRR register to HPRE position
} AHB_divider;

/**
* llcm.h
* @code{C}
* void initSW(void);
* @endcode
* The function used for quick pins setup to select Serial Wire debug.
* Sets PA15(JTDI), PB3(JTDO), PB4(NJRST) as analog to minimize power consuption unless it is used
*/
void initSW(void);  //Serial Wire setup. NOTE unused pins set as analog

/**
* llcm.h
* @code{c}
void init_clock_HSE(uint32_t M_arg, uint32_t N_arg, ePLLP_arg P_arg, AHB_divider AHB, APB_divider APB1, APB_divider APB2);
* @endcode
* The functions is used to set up SystemCoreClock speed.
* @warning M_arg must be between 2 and 63. M_arg equal to 0 or 1 is prohibited
* @warning N_arg must be between 2 and 510. M_arg equal to 0, 1, 433 or 511 is prohibited
* @param M_arg the value for PLL M argument. 1st argument of PLL that divides the value of HSE clock. 
* @param N_arg the value for PLL N argument. 2nd argument of PLL that mupltyplies the HSE clock after M division. 
* @param P_arg the value for PLL P argument. 3d argmunet of of PLL that divides the value of HSE clock after N multiplication. #ePLLP_arg may be used to quickly select divider
* @param AHB the value for AHB bus division that is used to specify clock speed on all peripherals buses. #AHB_divider may be used to quickly select divider
* @param APB1 the value specifies the bus clock speed for APB1 bus. #APB_divider may be used to quickly select divider
* @param APB2 the value speicifies the bus clock speed for APB2 bus #APB_divider may be used to quickly select divider
*/
void init_clock_HSE(uint32_t M_arg, uint32_t N_arg, ePLLP_arg P_arg, AHB_divider AHB, APB_divider APB1,
                    APB_divider APB2); /*initialize System clock to work at 100 MHz*/

/**
* llcm.h
* @code{C}
* void init_SysTick(uint32_t ticks)
* @endcode
* The function is used to initialize SysTick.
* Example usage:
* @code{C}
* int main(void){
*  init_clock_HSE(25, 200, pllp_div2, AHB_div1, APB_div2, APB_div1); // initialize clock to work at 100 MHz (HSE = 25 MHz)
*  init_SysTick(1000); // initialize SysTick to work in milliseconds
*  while(1){}
*  return 0;
* }
* @endcode
* @param ticks the number of ticks interpreted as a period.
*/
void init_SysTick(uint32_t ticks);     /* initialize SysTick to work at 1ms */

/**
* llcm.h
* @code{c}
* void delay_ms(uint32_t time);
* @endcode
* The function snippet used to create delay function working in milliseconds.
* @warning The SysTick must be initialized before usage.
* @param time the number of milliseconds to wait for
*/
void delay_ms(uint32_t time);          /*form a delay according to SysTick setup*/
