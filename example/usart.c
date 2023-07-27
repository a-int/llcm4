#include <llcm.h>
#include "stm32f411xe.h"
#include "stm32f4xx.h"

void usart2_init();
void usart_sendByte(uint8_t data);
void usart_send(void* data, uint32_t size);
void delay_ms(uint32_t time);
volatile uint32_t delay_ms_time = 0;

int main() {
  setupSW();           // setup pins for Serial Wire debug
  init_clock_100();    // initialize MPU to work at 100 MHz
  init_SysTick(1000);  // initialize SysTick to work at ms
  usart2_init();
  while (1) {}
}

void usart2_init() {
  //GPIO registers set up
  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);   // enable GPIOA
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);  //enable usart module

  GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3);                  // clear default mode bits
  SET_BIT(GPIOA->MODER, GPIO_MODER_MODE2_1);                               // set PA2(TX) as alternative function
  MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFSEL2, 7 << GPIO_AFRL_AFSEL2_Pos);  // select Alternative function 7 (USART TX)
  SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR2);                         // select High speed for PA2

  SET_BIT(GPIOA->MODER, GPIO_MODER_MODE3_1);                               // set PA3(RX) as alternative function
  MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFSEL3, 7 << GPIO_AFRL_AFSEL3_Pos);  // select Alternative function 7 (USART RX)
  SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR3);                         // select High speed for PA3

  //Usart registers set up
  CLEAR_BIT(USART2->CR1, USART_CR1_M_Msk);  // use 8 bits (1 bit for Pairty check)
  CLEAR_BIT(USART2->CR1, USART_CR1_PCE);    // disable paitry bit

  //BAUD = 115200 ==> DIV = 50*10^6/16BAUD = 27.1267361111
  // Fraction = 0.1267361111*16 = 2.0277777776 ~ 2 (nearest)
  // Mantissa = 27
  MODIFY_REG(USART2->BRR, USART_BRR_DIV_Fraction, (2 << USART_BRR_DIV_Fraction_Pos));
  MODIFY_REG(USART2->BRR, USART_BRR_DIV_Mantissa, (27 << USART_BRR_DIV_Mantissa_Pos));

  SET_BIT(USART2->CR1, USART_CR1_RXNEIE);   //enable IRQ for ready to read
  CLEAR_BIT(USART2->CR1, USART_CR1_TXEIE);  //disable IRQ for completing the  transimition

  SET_BIT(USART2->CR1, USART_CR1_TE);  // enable transmitter
  SET_BIT(USART2->CR1, USART_CR1_RE);  // enable receiver
  SET_BIT(USART2->CR1, USART_CR1_UE);  // enable usart

  NVIC_EnableIRQ(USART2_IRQn);  // turn on IRQs for USART 2
}

void usart_sendByte(uint8_t data) {
  while (!(USART2->SR & USART_SR_TC))
    ;
  USART2->DR = data;
}

void usart_send(void* data, uint32_t size) {
  uint8_t* byte = (uint8_t*)data;
  while (size--) {
    usart_sendByte(*byte++);
  }
}

void delay_ms(uint32_t time) {
  delay_ms_time = time;
  while (delay_ms_time > 0)
    ;
}

void SysTick_Handler(void) {
  if (delay_ms_time > 0) {
    --delay_ms_time;
  }
}

void USART2_IRQHandler(void) {
  if (READ_BIT(USART2->SR, USART_SR_RXNE)) {  //if data may be read
    if ('F' == USART2->DR) {
      const char* msg = "Hello world!\n";
      usart_send((char*)msg, 13);
    }
  }
  if (READ_BIT(USART2->SR, USART_SR_IDLE)) {
    USART2->DR;  // reset IDLE flag
  }
}
