#include <llcm.hpp>
#include "stm32f411xe.h"
#include "stm32f4xx.h"

volatile uint32_t DELAY_MS_TIME = 0;
void init_GPIO(); // initialize GPIO ports
void delay_ms(uint32_t time_ms);

int main(){  
  cmsis::setupSW(); // setup pins for debug work
  cmsis::init_clock_100(); // initialize clock speed as 100 MHz
  cmsis::init_SysTick(1000); // initialize SysTic to work at ms
  init_GPIO();
  while(1){
    if(!cmsis::stateBitN(GPIOA, 0)){
      while(!cmsis::stateBitN(GPIOA, 0));
      cmsis::toggleBitN(GPIOC, 13);
      delay_ms(100);
    }
  }
}

void init_GPIO(){
  //set up for GPIOC13 (LED)
  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN); //enable GPIOC
  SET_BIT(GPIOC->MODER, GPIO_MODER_MODE13_0); // set as output
  CLEAR_BIT(GPIOC->OTYPER, GPIO_OTYPER_OT13_Msk); //select push-pull mode
  CLEAR_BIT(GPIOC->PUPDR, GPIO_PUPDR_PUPD13_Msk); //select no push no pull mode
  
  // set up for GPIOA0 (BUTTON)
  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN); //enable GPIOA
  CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE0_Msk); // set as input
  CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT0); //select push-pull mode
  SET_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPD0_0); //select no push no pull mode
}

void delay_ms(uint32_t time_ms){
  DELAY_MS_TIME = time_ms;
  while(DELAY_MS_TIME);
}

extern "C"{
  void SysTick_Handler(void){
    if(DELAY_MS_TIME > 0 ) --DELAY_MS_TIME;
  }
}