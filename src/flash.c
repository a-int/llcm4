#include <flash.h>
#include <stm32f4xx.h>

void FLASH_unlock() {
  WRITE_REG(FLASH->KEYR, 0x45670123);  // enter the 1st ket to unlock the FLASH
  WRITE_REG(FLASH->KEYR, 0xCDEF89AB);  // enter the 2d ket to unlock the FLASH
}

void FLASH_lock() {
  FLASH->CR |= FLASH_CR_LOCK;
}

void FLASH_clear_section(uint32_t sectorN) {
  MODIFY_REG(FLASH->CR, FLASH_CR_PSIZE, FLASH_CR_PSIZE_1);             //word mode for erasing
  MODIFY_REG(FLASH->CR, FLASH_CR_SNB, (sectorN << FLASH_CR_SNB_Pos));  //the sector to be erased
  SET_BIT(FLASH->CR, FLASH_CR_SER);                                    //sector erase mode
  SET_BIT(FLASH->CR, FLASH_CR_STRT);                                   // initiate the clearing of the selected section
}

void FLASH_write(uint32_t* src, uint32_t* dest, uint32_t size) {
  while ((READ_BIT(FLASH->SR, FLASH_SR_BSY))) {}            // whait while Busy
  SET_BIT(FLASH->CR, FLASH_CR_PG);                          //programming mode
  MODIFY_REG(FLASH->CR, FLASH_CR_PSIZE, FLASH_CR_PSIZE_1);  //select word mode for erasing

  for(uint32_t i = 0; i < size; ++i){
    dest[i] = src[i];
  }
  
  while ((READ_BIT(FLASH->SR, FLASH_SR_BSY))) {}  // whait while Busy
  CLEAR_BIT(FLASH->CR, FLASH_CR_PG);              // turn off programming mode
}