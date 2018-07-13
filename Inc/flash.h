#ifndef __FLASH_H__
#define __FLASH_H__

#include <stm32f1xx_hal.h>
#include <stm32f1xx_hal_flash.h>
#include <stdint.h>
#include <stdlib.h>


void Flash_ErasePage(uint32_t bank, uint8_t* pageAddress);
void Flash_Write(uint8_t* flashAddress, const void* data, size_t length);

#endif
