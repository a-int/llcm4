#pragma once
#include "stm32f4xx.h"
#include <stdint.h>

double __calculateDiv(USART_TypeDef* USARTx, uint32_t baudRate);
void __base_init_usart(USART_TypeDef* USARTx, uint32_t baudRate);
void __usart2_gpio_setup(void);
void __usart1_gpio_setup(void);
void init_usart2(uint32_t baudRate); // initialize usart2 at 115200 Baudrate
void init_usart2_dma(uint32_t baudRate); // initialize usart2 at 115200 Baudrate
void init_usart1(uint32_t baudRate); // initialize usart1 at 115200 Baudrate
void usart_sendByte(uint8_t data, USART_TypeDef* USARTx);
void usart_send(uint8_t* data, uint32_t size, USART_TypeDef* USARTx);
void usart_sendString(const char* data, USART_TypeDef* USARTx);
uint32_t strlen(const char* str);
extern uint8_t isIDLE(USART_TypeDef* USARTx);
extern uint8_t isRXNE(USART_TypeDef* USARTx);
