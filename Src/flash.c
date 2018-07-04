#include <stm32f1xx_hal.h>
#include <stm32f1xx_hal_flash.h>

void Flash_ErasePage(uint32_t bank, uint8_t* pageAddress) {
	FLASH_EraseInitTypeDef erase;
	uint32_t error;

	__disable_irq();
	HAL_FLASH_Unlock();
	FLASH->SR = FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR;

	erase.TypeErase = FLASH_TYPEERASE_PAGES;
	erase.Banks = bank;
	erase.PageAddress = (uint32_t) pageAddress;
	erase.NbPages = 1;
	HAL_FLASHEx_Erase(&erase, &error);
	__enable_irq();

	HAL_FLASH_Lock();
}

void Flash_Write(uint8_t* flashAddress, const void* data, size_t length) {
	const size_t wordsToFlush = (length + 3) / 4;

	__disable_irq();
	HAL_FLASH_Unlock();
	FLASH->SR = FLASH_SR_EOP | FLASH_FLAG_PGERR;

	const uint32_t* words = (uint32_t*) data;

	for (size_t i = 0; i < wordsToFlush; i++) {
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)flashAddress, *words++);
		flashAddress += 4;
	}

	__enable_irq();
	HAL_FLASH_Lock();
}
