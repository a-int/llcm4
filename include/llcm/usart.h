#pragma once
#include "stm32f4xx.h"
#include <stdint.h>

typedef enum {eBaudRate9600 = 9600, eBaudRate19200 = 19200, eBaudRate115200 = 115200} eUSARTBaudRate;

double __calculateDiv(USART_TypeDef* USARTx, eUSARTBaudRate baudRate);

void __base_init_usart(USART_TypeDef* USARTx, eUSARTBaudRate baudRate);
void __usart2_gpio_setup(void);
void __usart1_gpio_setup(void);

void init_usart(USART_TypeDef* USARTx, eUSARTBaudRate baudRate); // initialize USARTx selected baud rate
void init_usart2_dma(eUSARTBaudRate baudRate); // initialize usart2 at 115200 Baudrate

void usart_sendByte(uint8_t data, USART_TypeDef* USARTx);
void usart_send(uint8_t* data, uint32_t size, USART_TypeDef* USARTx);
void usart_sendString(const char* data, USART_TypeDef* USARTx);
uint32_t strlen(const char* str);

extern uint8_t isIDLE(USART_TypeDef* USARTx);
extern uint8_t isRXNE(USART_TypeDef* USARTx);

extern void usartxRXNEIEup(USART_TypeDef* USARTx);
extern void usartxRXNEIEdown(USART_TypeDef* USARTx);
extern void usartxIDLEIEup(USART_TypeDef* USARTx);
extern void usartxIDLEIEdown(USART_TypeDef* USARTx);
extern void usartxTXEIEup(USART_TypeDef* USARTx);
extern void usartxTXEIEdown(USART_TypeDef* USARTx);