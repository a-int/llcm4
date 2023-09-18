#pragma once
#include "stm32f4xx.h"
#include <stdint.h>

void __base_init_usart2_115200(); // initialize usart2 at 115200 Baudrate
void init_usart2_115200(); // initialize usart2 at 115200 Baudrate
void init_usart2_dma_115200(); // initialize usart2 at 115200 Baudrate
void usart_sendByte(uint8_t data);
void usart_send(uint8_t* data, uint32_t size);
void usart_sendString(const char* data);
uint32_t strlen(const char* str);
extern uint8_t isIDLE();
extern uint8_t isRXNE();
