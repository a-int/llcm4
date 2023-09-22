/**
*  @file usart.h
*  The collection of functions and constant for simple USART usage with CMSIS API 
*/
#pragma once
#include "stm32f4xx.h"
#include <stdint.h>

/**
* usart.h
* The enumeration to quickly select baud rate
*/
typedef enum {
    eBaudRate9600 = 9600, ///< Baud rate for 9600 bit/s
    eBaudRate19200 = 19200, ///< Baud rate for 19200 bit/s 
    eBaudRate115200 = 115200 ///< Baud rate for 115200 bit/s
} eUSARTBaudRate;

/**
* usart.h
* The function to initialize USART using specific baud rate and USART module
* @code{c}
* void init_usart(USART_TypeDef* USARTx, eUSARTBaudRate baudRate);
* @endcode
* Example usage:
* @code{c}
* int main(void){
*  *** any other initialization ***
*  init_usart(USART1, eBaudRate115200); // initialize USART1 to work at baud rate 115200
*  usart_sendString("The usart is initialized\r\n"); // send string to receiver
*  return 0; 
* }
* @endcode
* @param USARTx is the USART to initialize
* @param baudRate is the desired baud rate which may be selected using #eUSARTBaudRate enumeration
*/
void init_usart(USART_TypeDef* USARTx, eUSARTBaudRate baudRate);

/**
* usart.h
* The function to initialize USART using specific baud rate and USART module in DMA mode
* Pretty the same as usart_init() but enables interrupt specific for DMA mode
* @code{c}
* void init_usart2_dma(eUSARTBaudRate baudRate);
* @endcode
* Example usage:
* @code{c}
* int main(void){
*  *** any other initialization ***
*  init_usart2_dma(eBaudRate115200); // initialize USART2 to work at baud rate 115200 in DMA mode
*  return 0; 
* }
* @endcode
* @param baudRate is the desired baud rate which may be selected using #eUSARTBaudRate enumeration
*/
void init_usart2_dma(eUSARTBaudRate baudRate);

/**
* usart.h
* @code{c}
* void usart_sendByte(uint8_t data, USART_TypeDef* USARTx)
* @endcode
* Sends the provided byte via selected USART module. Used to build more complex sending functions.
* Example usage:
* @code{c}
* #define USARTx USART1
*
* int main(void){
*    ***any other initialization***
*    init_usart(USARTx, eBaudRate9600); // initialize USART module to work at baud rate 9600
*    usart_sendByte('F', USARTx); // send the byte to the receiver
*    return 0;
* }
* @endcode 
* @param byte the data to be sent via USART
* @param USARTx the desired usart module used to send the byte (Be sure the USART module is initialized before using send functions).
*/
void usart_sendByte(uint8_t byte, USART_TypeDef* USARTx);

/**
* usart.h
* @code{c}
* void usart_send(uint8_t* data, uint32_t size, USART_TypeDef* USARTx);
* @endcode
* The function used to send the data of predetermined size via selected USART module.
* (Be sure the the selected USART module is succesfully initialized before usage).
* Example usage:
* @code{C}
* #define USART USART1
* #define USART_BAUD_RATE eBaudRate9600
* 
* int main() {
*   *** any other initialization ***
*   init_usart(USART, USART_BAUD_RATE); // initialize the selected USART module
*   while (1) {}
*   return 0;
* }
* 
* void SysTick_Handler(void){} // required for proper USART work
* 
* void USART1_IRQHandler(void) {
*   if (isRXNE(USART)) {  //if data may be read
*     if ('F' == USART->DR) { // check if the received data conforms the condition
*       const char* msg = "Hello world!\r\n"; // the message to be sent
*       usart_send((uint8_t*)msg, strlen(msg), USART); // send the message
*     }
*   }
*   if (isIDLE(USART)) {
*       USART->DR;  // reset IDLE flag
*       const char* msg = "IDLE LINE DETECTED\r\n"; // the message to be sent
*       usart_send((uint8_t*)msg, strlen(msg), USART); // send the message
*   }
* }
* @endcode
* @param data the data to be sent via selected USART module
* @param size the number of bytes in the data to be sent
* @param USARTx the desired USART module used to send data
*/
void usart_send(uint8_t* data, uint32_t size, USART_TypeDef* USARTx);

/**
* usart.h
* @code{c}
* void usart_sendString(const char* data, USART_TypeDef* USARTx);
* @endcode
* The function used to send strings.
* May be used as simple alternative to printf to debug code.
* Example code:
* @code{c}
* #define USART USART1
* #define USART_BAUD_RATE eBaudRate9600
* 
* int main() {
*   *** any other initialization ***
*   init_usart(USART, USART_BAUD_RATE); // initialize the selected USART module
*   while (1) {}
*   return 0;
* }
* 
* void SysTick_Handler(void){} // required for proper USART work
* 
* void USART1_IRQHandler(void) {
*   if (isRXNE(USART)) {  //if data may be read
*     if ('F' == USART->DR) { // check if the received data conforms the condition
*       const char* msg = "Hello world!\r\n"; // the message to be sent
*       usart_sendString(msg);
*     }
*   }
*   if (isIDLE(USART)) {
*       USART->DR;  // reset IDLE flag
*       const char* msg = "IDLE LINE DETECTED\r\n"; // the message to be sent
*       usart_sendString(msg);
*   }
* }
* @endcode
* @param data the string to be sent
* @param USARTx the desired USART module used to send string
*/
void usart_sendString(const char* data, USART_TypeDef* USARTx);

/**
* usart.h
* @code{c}
* uint32_t strlen(const char* str);
* @endcode
* The function used to calculate the size of the string.
* @param str string the size is to be calculate for
* @return uint32_t the number of bytes in the string. UINT32_MAX if the string is too large
*/
uint32_t strlen(const char* str);

/**
* usart.h
* @code{c}
* extern uint8_t isIDLE(USART_TypeDef* USARTx);
* @endcode
* The function used to determine quickly whether the IDLE flag is set or not

* @code{c}
* #define USART USART1
* #define USART_BAUD_RATE eBaudRate9600
*  
* int main() {
*   *** any other initialization ***
*   init_usart(USART, USART_BAUD_RATE); // initialize the selected USART module
*   while (1) {}
*   return 0;
* }
*
* void USART1_IRQHandler(void) {
*   if (isRXNE(USART)) {}
*   if (isIDLE(USART)) {
*       USART->DR;  // reset IDLE flag
*       const char* msg = "IDLE LINE DETECTED\r\n"; // the message to be sent
*       usart_sendString(msg);
*   }
* }
*
* void SysTick_Handler(void){} // required for proper USART work
* @endcode
* @param USARTx the usart to check the flag for
* @return true if the flag is set
*/
extern uint8_t isIDLE(USART_TypeDef* USARTx);

/**
* usart.h
* @code{c}
* extern uint8_t isRXNE(USART_TypeDef* USARTx);
* @endcode
* The function used to determine quickly whether the RXNE flag is set or not
* @code{c}
* #define USART USART1
* #define USART_BAUD_RATE eBaudRate9600
*  
* int main() {
*   *** any other initialization ***
*   init_usart(USART, USART_BAUD_RATE); // initialize the selected USART module
*   while (1) {}
*   return 0;
* }
*  
* void USART1_IRQHandler(void) {
*   if (isRXNE(USART)) {  //if data may be read
*     if ('F' == USART->DR) { // check if the received data conforms the condition
*       const char* msg = "Hello world!\r\n"; // the message to be sent
*       usart_sendString(msg);
*     }
*   }
*   if (isIDLE(USART)) {
*       USART->DR;  // reset IDLE flag
*   }
* }
*
* void SysTick_Handler(void){} // required for proper USART work
* @endcode
* @param USARTx the usart to check the flag for
* @return true if the flag is set
*/
extern uint8_t isRXNE(USART_TypeDef* USARTx);

/**
* usart.h
* The function used to quickly enable RX Not Empty Interrupt
* @code{c}
* extern void usartxRXNEIEup(USART_TypeDef* USARTx);
* @endcode
* @param USARTx the USART module the interrupt is to be enabled
*/
extern void usartxRXNEIEup(USART_TypeDef* USARTx);

/**
* usart.h
* The function used to quickly disable RX Not Empty Interrupt
* @code{c}
* extern void usartxRXNEIEdown(USART_TypeDef* USARTx);
* @endcode
* @param USARTx the USART module the interrupt is to be disabled
*/
extern void usartxRXNEIEdown(USART_TypeDef* USARTx);

/**
* usart.h
* The function used to quickly enable IDLE Interrupt
* @code{c}
* extern void usartxIDLEIEup(USART_TypeDef* USARTx);
* @endcode
* @param USARTx the USART module the interrupt is to be enabled
*/
extern void usartxIDLEIEup(USART_TypeDef* USARTx);

/**
* usart.h
* The function used to quickly disable IDLE Interrupt
* @code{c}
* extern void usartxIDLEIEdown(USART_TypeDef* USARTx);
* @endcode
* @param USARTx the USART module the interrupt is to be disabled
*/
extern void usartxIDLEIEdown(USART_TypeDef* USARTx);

/**
* usart.h
* The function used to quickly enable TX Empty Interrupt
* @code{c}
* extern void usartxTXEIEup(USART_TypeDef* USARTx);
* @endcode
* @param USARTx the USART module the interrupt is to be enabled
*/
extern void usartxTXEIEup(USART_TypeDef* USARTx);

/**
* usart.h
* The function used to quickly disable TX Empty Interrupt
* @code{c}
* extern void usartxTXEIEdown(USART_TypeDef* USARTx);
* @endcode
* @param USARTx the USART module the interrupt is to be disabled
*/
extern void usartxTXEIEdown(USART_TypeDef* USARTx);