#pragma once
#include <stdint.h>
void FLASH_unlock(void);                                     //unlock the FLASH
void FLASH_lock(void);                                       //lock the FLASH
void FLASH_clear_section(uint32_t sectorN);                         // clear the firmware data
void FLASH_write(uint32_t* src, uint32_t* dest, uint32_t size);  // write the firmware data
