#include <llcm.h>
#include <usart.h>

volatile uint32_t delay_ms_time = 0;

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

int main() {
  initSW();           // setup pins for Serial Wire debug
  init_clock_100();    // initialize MPU to work at 100 MHz
  init_SysTick(1000);  // initialize SysTick to work at ms
  init_usart2_115200();
  init_GPIO();
  while (1) {}
}

void SysTick_Handler(void) {
  if (delay_ms_time > 0) {
    --delay_ms_time;
  }
}

void USART2_IRQHandler(void) {
  if (READ_BIT(USART2->SR, USART_SR_RXNE)) {  //if data may be read
    if ('F' == USART2->DR) {
      GPIOC->ODR ^= 1 << 13;
      const char* msg = "Hello world!\r\n";
      usart_send((uint8_t*)msg, strlen(msg));
    }
  }
  if (READ_BIT(USART2->SR, USART_SR_IDLE)) {
    USART2->DR;  // reset IDLE flag
      const char* msg = "IDLE LINE DETECTED\r\n";
      usart_send((uint8_t*)msg, strlen(msg));
  }
}
