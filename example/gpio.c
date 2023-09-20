#include <llcm.h>

void init_GPIO();  // initialize GPIO ports
volatile uint32_t delay_ms_time = 0;

int main() {
  initSW();           // setup pins for debug work
  init_clock_HSE(25, 200, pllp_div2, AHB_div1, APB_div2, APB_div1);    // initialize clock speed as 100 MHz
  init_SysTick(1000);  // initialize SysTic to work at ms
  init_GPIO();
  while (1) {
    if (!READ_BIT(GPIOA->IDR, 1 << 0)) {
      while (!READ_BIT(GPIOA->IDR, 1 << 0))
        ;
      GPIOC->ODR ^= 1 << 13;
      delay_ms(100);
    }
  }
}

void init_GPIO() {
  //set up for GPIOC13 (LED)
  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);      //enable GPIOC
  SET_BIT(GPIOC->MODER, GPIO_MODER_MODE13_0);      // set as output
  CLEAR_BIT(GPIOC->OTYPER, GPIO_OTYPER_OT13_Msk);  //select push-pull mode
  CLEAR_BIT(GPIOC->PUPDR, GPIO_PUPDR_PUPD13_Msk);  //select no push no pull mode

  // set up for GPIOA0 (BUTTON)
  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);     //enable GPIOA
  CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE0_Msk);  // set as input
  CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT0);      //select push-pull mode
  SET_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPD0_0);      //select no push no pull mode
}

void SysTick_Handler(void) {
  if (delay_ms_time > 0)
    --delay_ms_time;
}
