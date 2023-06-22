#include <stm32f411xe.h>
#include <llcm.hpp>

void cmsis::setBitN(GPIO_TypeDef* gpiox, const uint8_t bitN) {
  gpiox->ODR |= (1 << bitN);
}

void cmsis::resetBitN(GPIO_TypeDef* gpiox, const uint8_t bitN) {
  gpiox->ODR &= ~(1 << bitN);  // BR = [16;31]
}

void cmsis::toggleBitN(GPIO_TypeDef* gpiox, const uint8_t bitN) {
  gpiox->ODR ^= 1 << bitN;
}

bool cmsis::stateBitN(GPIO_TypeDef* gpiox, const uint8_t bitN) {
  return gpiox->IDR & (1 << bitN);
}

/*initialize System clock to work at 100 MHz*/
void cmsis::init_clock_100() {
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
  SET_BIT(FLASH->ACR,
          (FLASH_ACR_LATENCY_3WS | FLASH_ACR_DCEN | FLASH_ACR_ICEN));

  SET_BIT(RCC->CR, RCC_CR_HSEON);  // turn on HSE
  while (READ_BIT(RCC->CR, RCC_CR_HSERDY) == RESET)
    ;

  CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);              //turn off bypass
  SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC_HSE);  //select PLL inputr as HSE
  SET_BIT(RCC->CR, RCC_CR_CSSON);                 // turn on CSS

  MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLM, (25UL << RCC_PLLCFGR_PLLM_Pos));
  MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLN, (200UL << RCC_PLLCFGR_PLLN_Pos));
  CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLP);  // set P = 2

  // CLEAR_BIT(RCC->CFGR, RCC_CFGR_HPRE);     // set AHB = 1
  SET_BIT(RCC->CFGR, RCC_CFGR_PPRE1_DIV2);  // set APB1 = 2
  // CLEAR_BIT(RCC->CFGR, RCC_CFGR_PPRE2);    // set APB2 = 1

  SET_BIT(RCC->CR, RCC_CR_PLLON);
  while ((READ_BIT(RCC->CR, RCC_CR_PLLRDY)) == RESET)
    ;

  SET_BIT(RCC->CFGR, RCC_CFGR_SW_PLL);  // select PLL in SW
  while ((READ_BIT(RCC->CFGR, RCC_CFGR_SWS_PLL)) == RESET)
    ;

  SystemCoreClockUpdate();  //update global variable for system clock
}

/* initialize SysTick*/
void cmsis::init_SysTick(uint32_t ticks) {
  MODIFY_REG(SysTick->LOAD, SysTick_LOAD_RELOAD_Msk,
             (SystemCoreClock / ticks - 1));
  MODIFY_REG(SysTick->VAL, SysTick_VAL_CURRENT_Msk,
             (SystemCoreClock / ticks - 1));
  SET_BIT(SysTick->CTRL, (SysTick_CTRL_CLKSOURCE_Msk |
                          SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk));
}
