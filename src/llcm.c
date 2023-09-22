#include <llcm.h>
#include <stdint.h>
#include "stm32f411xe.h"

/*initialize System clock to work at 100 MHz*/
void init_clock_HSE(uint32_t M_arg, uint32_t N_arg, ePLLP_arg P_arg, AHB_divider AHB, APB_divider APB1, APB_divider APB2) {
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
  SET_BIT(FLASH->ACR, (FLASH_ACR_LATENCY_3WS | FLASH_ACR_DCEN | FLASH_ACR_ICEN));

  SET_BIT(RCC->CR, RCC_CR_HSEON);  // turn on HSE
  while (READ_BIT(RCC->CR, RCC_CR_HSERDY) == RESET)
    ;

  CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);              //turn off bypass
  SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC_HSE);  //select PLL inputr as HSE
  SET_BIT(RCC->CR, RCC_CR_CSSON);                 // turn on CSS

  MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLM, (M_arg << RCC_PLLCFGR_PLLM_Pos));
  MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLN, (N_arg << RCC_PLLCFGR_PLLN_Pos));
  MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLP, (P_arg << RCC_PLLCFGR_PLLP_Pos));

  MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, (AHB << RCC_CFGR_HPRE_Pos)); //set AHB prescaler
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, (APB1 << RCC_CFGR_PPRE1_Pos)); //set APB1 prescaler
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, (APB2 << RCC_CFGR_PPRE2_Pos)); //set APB2 prescaler

  SET_BIT(RCC->CR, RCC_CR_PLLON);
  while ((READ_BIT(RCC->CR, RCC_CR_PLLRDY)) == RESET)
    ;

  SET_BIT(RCC->CFGR, RCC_CFGR_SW_PLL);  // select PLL in SW
  while ((READ_BIT(RCC->CFGR, RCC_CFGR_SWS_PLL)) == RESET)
    ;

  SystemCoreClockUpdate();  //update global variable for system clock
}

/* initialize SysTick*/
void init_SysTick(uint32_t ticks) {
  MODIFY_REG(SysTick->LOAD, SysTick_LOAD_RELOAD_Msk, (SystemCoreClock / ticks - 1));
  MODIFY_REG(SysTick->VAL, SysTick_VAL_CURRENT_Msk, (SystemCoreClock / ticks - 1));
  SET_BIT(SysTick->CTRL, (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk));
}

//Serial Wire setup. NOTE unused pins set as analog
void initSW() {
  /* Set PA15(JTDI), PB3(JTDO), PB4(NJRST) as analog to minimize power consuption unless it used*/
  CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE15);
  SET_BIT(GPIOA->MODER, GPIO_MODER_MODE15);

  CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODE3);
  SET_BIT(GPIOB->MODER, GPIO_MODER_MODE3);

  CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODE4);
  SET_BIT(GPIOB->MODER, GPIO_MODER_MODE4);
}

void delay_ms(uint32_t time) {
  delay_ms_time = time;
  while (delay_ms_time > 0)
    ;
}
