#include "stm32f411xe.h"
#include "stm32f4xx.h"
#include <llcm.hpp>

volatile uint32_t delay_ms_time = 0;
void delay_ms(uint32_t time);
void gpio_setup();
void exti_setup();

int main(){
  cmsis::init_clock_100(); // initialize MPU to work at 100 MHz
  cmsis::init_SysTick(1000); // set SysTick to work at ms
  gpio_setup(); // set up gpio ports for exti
  exti_setup(); // set up exti
  while (1){
    delay_ms(200);
  }
  return 0;
}

void gpio_setup(){
  // set PA0(Button) as input pull-up
  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN); //enable GPIOA
  CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE0_Msk); // set as input
  CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT0); //select push-pull mode
  SET_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPD0_0); //select no push no pull mode

  //set up PC13(led) as output
  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN); //enable GPIOC
  SET_BIT(GPIOC->MODER, GPIO_MODER_MODE13_0); // set as output
  CLEAR_BIT(GPIOC->OTYPER, GPIO_OTYPER_OT13_Msk); //select push-pull mode
  CLEAR_BIT(GPIOC->PUPDR, GPIO_PUPDR_PUPD13_Msk); //select no push no pull mode
}

void exti_setup(){
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN); // enable the System Configuration 
  MODIFY_REG(SYSCFG->EXTICR[0], SYSCFG_EXTICR1_EXTI0_Msk, SYSCFG_EXTICR1_EXTI0_PA); // select the PA0 as source of EXTI0
  SET_BIT(EXTI->IMR, EXTI_IMR_MR0); //enable IRQ for MR0 (EXTI0)
  CLEAR_BIT(EXTI->RTSR, EXTI_RTSR_TR0); // disable the trigger on rising
  SET_BIT(EXTI->FTSR, EXTI_FTSR_TR0); // enable the trigger on falling
  NVIC_EnableIRQ(EXTI0_IRQn); // enable the EXTI0 IRQ
}

extern "C"{
  void SysTick_Handler(void){
    if(delay_ms_time > 0) --delay_ms_time;
  }

  void EXTI0_IRQHandler(void){
    cmsis::toggleBitN(GPIOC, 13); // switch the state of led
    SET_BIT(EXTI->PR, EXTI_PR_PR0); // clear the flag
  }
}

void delay_ms(uint32_t time){
  delay_ms_time = time;
  while(delay_ms_time > 0 );
}
