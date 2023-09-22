/**
* @file flash.h
*/
#pragma once
#include <stdint.h>

/**
* flash.h
* @code{C}
* void FLASH_unlock(void);
* @endcode
* The function is used to unlock internal FLASH memory if it is locked
*/
void FLASH_unlock(void);                                     //unlock the FLASH

/**
* flash.h
* @code{C}
* void FLASH_lock(void);
* @endcode
* The function is used to lock internal FLASH memory after it is unlocked
*/
void FLASH_lock(void);                                       //lock the FLASH

/**
* flash.h
* @code{C}
* void FLASH_clear_section(uint32_t sectorN);
* @endcode
* The function is used to clear the specific FLASH section
* @param sectorN the sector to be cleared.\n
*  sector 1 is 0x0800_0000 - 0x0800_3FFF 16 Kib \n 
*  sector 2 is 0x0800_4000 - 0x0800_7FFF 16 Kib \n
*  sector 3 is 0x0800_8000 - 0x0800_BFFF 16 Kib \n
*  sector 4 is 0x0800_C000 - 0x0800_FFFF 16 Kib \n
*  sector 5 is 0x0801_0000 - 0x0801_FFFF 64 Kib \n
*  sector 6 is 0x0802_0000 - 0x0803_FFFF 128 Kib \n
*  sector 7 is 0x0804_0000 - 0x0805_FFFF 128 Kib \n
*  sector 7 is 0x0806_0000 - 0x0807_FFFF 128 Kib \n
*/
void FLASH_clear_section(uint32_t sectorN);                         // clear the firmware data

/**
* flash.h
* @code{C}
* void FLASH_write(uint32_t* src, uint32_t* dest, uint32_t size);
* @endcode
* The function is used to write desired number of words from source to destination(FLASH address).
* Example usage:
* @code{C}
* #define USART_RX_DATA_MAX (32*1024)
* #define USARTx USART1
* typedef struct {
*   uint8_t rx_data[USART_RX_DATA_MAX];
*   uint32_t rx_data_counter;
* } USART;
* 
* USART huart;
* 
* int main() {
*   *** initialization routine ***
*   while (1) {
*     if(isButtonPressed()){
*       Enter_Firmware();
*     }
*   }
* }
* * 
* void USART1_IRQHandler(void) {
*   if (isRXNE(USARTx)) { //if data may be read
*     huart.rx_data[huart.rx_data_counter++] = (uint8_t)USARTx->DR;  // copy recieved data
*   }
*   if (isIDLE(USARTx)) {
*     USARTx->DR;  // reset IDLE flag
*     FLASH_unlock();
*     FLASH_clear_section(1); //firmware at sector 1 : 0x0800_4000  -  0x0800_4fff
*     FLASH_write((uint32_t*)huart.rx_data, (uint32_t*)_sfirmware, huart.rx_data_counter / 4); //number of words is 4 times less than number of received  bytes
*     FLASH_lock();
*   }
* }
* void SysTick_Handler(void) {}
* @endcode
* @param src the pointer to source to read from
* @param dest the address to save to
* @param size the number of words to save 
* @warning The word is equal to 4 bytes due to 32 bit architecture
*/
void FLASH_write(uint32_t* src, uint32_t* dest, uint32_t size);  // write the firmware data
