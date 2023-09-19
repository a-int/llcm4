#include <usart.h>
#include "stm32f411xe.h"

void __base_init_usart2_115200() {
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

  //BAUD = 115200 ==> DIV = PCLK1/(16*BAUD) = 27.1267361111
  // Fraction = 0.1267361111*16 = 2.0277777776 ~ 2 (nearest)
  // Mantissa = 27
  MODIFY_REG(USART2->BRR, USART_BRR_DIV_Fraction, (2 << USART_BRR_DIV_Fraction_Pos));
  MODIFY_REG(USART2->BRR, USART_BRR_DIV_Mantissa, (27 << USART_BRR_DIV_Mantissa_Pos));

  SET_BIT(USART2->CR1, USART_CR1_TE);  // enable transmitter
  SET_BIT(USART2->CR1, USART_CR1_RE);  // enable receiver
  SET_BIT(USART2->CR1, USART_CR1_UE);  // enable usart
}

void init_usart2_115200() {
  __base_init_usart2_115200();

  SET_BIT(USART2->CR1, USART_CR1_RXNEIE);   //enable IRQ for ready to read
  SET_BIT(USART2->CR1, USART_SR_IDLE);      //enable IRQ if IDLE line detected
  CLEAR_BIT(USART2->CR1, USART_CR1_TXEIE);  //disable IRQ for completing the transimition
  NVIC_EnableIRQ(USART2_IRQn);              // turn on IRQs for USART 2
}

void __base_init_usart1_115200() {
  //GPIO registers set up
  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);   // enable GPIOA
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);  //enable usart module

  GPIOA->MODER &= ~(GPIO_MODER_MODE9 | GPIO_MODER_MODE10);                  // clear default mode bits
  SET_BIT(GPIOA->MODER, GPIO_MODER_MODE9_1);                               // set PA9(TX) as alternative function
  MODIFY_REG(GPIOA->AFR[1], GPIO_AFRH_AFSEL9, 7 << GPIO_AFRH_AFSEL9_Pos);  // select Alternative function 7 (USART TX)
  SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR9);                         // select High speed for PA9

  SET_BIT(GPIOA->MODER, GPIO_MODER_MODE10_1);                               // set PA3(RX) as alternative function
  MODIFY_REG(GPIOA->AFR[1], GPIO_AFRH_AFSEL10, 7 << GPIO_AFRH_AFSEL10_Pos);  // select Alternative function 7 (USART RX)
  SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR10);                         // select High speed for PA10

  //Usart registers set up
  CLEAR_BIT(USART1->CR1, USART_CR1_M);  // use 8 bits (1 bit for Pairty check)
  CLEAR_BIT(USART1->CR1, USART_CR1_PCE);    // disable paitry bit

  //BAUD = 115200 ==> DIV = PCLK2/(16*BAUD) = 54,2534722222
  // Fraction = 0,2534722222*16 = 4,0555555552 ~ 4 (nearest)
  // Mantissa = 54
  MODIFY_REG(USART1->BRR, USART_BRR_DIV_Fraction, (4 << USART_BRR_DIV_Fraction_Pos));
  MODIFY_REG(USART1->BRR, USART_BRR_DIV_Mantissa, (54 << USART_BRR_DIV_Mantissa_Pos));

  SET_BIT(USART1->CR1, USART_CR1_TE);  // enable transmitter
  SET_BIT(USART1->CR1, USART_CR1_RE);  // enable receiver
  SET_BIT(USART1->CR1, USART_CR1_UE);  // enable usart
}

void init_usart1_115200() {
  __base_init_usart1_115200();

  SET_BIT(USART1->CR1, USART_CR1_RXNEIE);   //enable IRQ for ready to read
  SET_BIT(USART1->CR1, USART_SR_IDLE);      //enable IRQ if IDLE line detected
  CLEAR_BIT(USART1->CR1, USART_CR1_TXEIE);  //disable IRQ for completing the transimition
  NVIC_EnableIRQ(USART1_IRQn);              // turn on IRQs for USART 2
}

void init_usart2_dma_115200() {
  __base_init_usart2_115200();

  SET_BIT(USART2->CR3, USART_CR3_DMAR);  //select DMA for reception
}

void usart_sendByte(uint8_t data, USART_TypeDef* USARTx) {
  while (!(USARTx->SR & USART_SR_TC)) {}  //what while cannot transmit
  USARTx->DR = data;
}

void usart_send(uint8_t* data, uint32_t size, USART_TypeDef* USARTx) {
  while (size--) {
    usart_sendByte(*data++, USARTx);
  }
}

void usart_sendString(const char* str, USART_TypeDef* USARTx) {
  usart_send((uint8_t*)str, strlen(str), USARTx);
}

uint32_t strlen(const char* str){
  uint32_t lenght = 0;
  while(str[lenght] != '\0' && lenght != UINT32_MAX){
    ++lenght;
  }
  return lenght;
}

inline uint8_t isIDLE(USART_TypeDef* USARTx){
  return READ_BIT(USARTx->SR, USART_SR_IDLE);
}

inline uint8_t isRXNE(USART_TypeDef* USARTx){
  return READ_BIT(USARTx->SR, USART_SR_RXNE);
}
