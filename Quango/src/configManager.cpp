#include "configManager.h"
#include <cmath>
#include <cstring>
#include <cstdio>


bool Config::SaveConfig()
{
	// Write config settings to Flash memory
	scheduleSave = false;
	bool result = true;

	if (currentSettingsOffset == -1) {					// First set in RestoreConfig
		currentSettingsOffset = 0;
	} else {
		currentSettingsOffset += settingsSize;
		if (currentSettingsOffset > flashPageSize - settingsSize) {
			currentSettingsOffset = 0;
		}
	}
	uint32_t* flashPos = flashConfigAddr + currentSettingsOffset / 4;

	// Check if flash needs erasing
	bool eraseFlash = false;
	for (uint32_t i = 0; i < settingsSize / 4; ++i) {
		if (flashPos[i] != 0xFFFFFFFF) {
			eraseFlash = true;
			currentSettingsOffset = 0;					// Reset offset of current settings to beginning of page
			flashPos = flashConfigAddr;
			break;
		}
	}

	uint8_t configBuffer[settingsSize];					// Will hold all the data to be written to the
	memcpy(configBuffer, ConfigHeader, 4);

	// Add individual config settings to buffer after header
	uint32_t configPos = 4;
	for (auto& saver : configSavers) {
		memcpy(&configBuffer[configPos], saver->settingsAddress, saver->settingsSize);
		configPos += saver->settingsSize;
	}

	__disable_irq();									// Disable Interrupts
	FlashUnlock();										// Unlock Flash memory for writing
	FLASH->SR = flashAllErrors;							// Clear error flags in Status Register

	if (eraseFlash) {
		FlashErasePage(flashConfigPage - 1);
	}
	result = FlashProgram(flashPos, reinterpret_cast<uint32_t*>(&configBuffer), settingsSize);

	FlashLock();						// Lock Flash
	__enable_irq(); 					// Enable Interrupts

	printf(result ? "Config Saved\r\n" : "Error saving config\r\n");
	return result;
}


void Config::RestoreConfig()
{
	// Locate latest (active) config block
	uint32_t pos = 0;
	while (pos < flashPageSize - settingsSize) {
		if (*(flashConfigAddr + pos / 4) == *(uint32_t*)ConfigHeader) {
			currentSettingsOffset = pos;
			pos += settingsSize;
		} else {
			break;			// Either reached the end of the page or found the latest valid config block
		}
	}

	if (currentSettingsOffset >= 0) {
		const uint8_t* flashConfig = reinterpret_cast<uint8_t*>(flashConfigAddr) + currentSettingsOffset;
		uint32_t configPos = sizeof(ConfigHeader);		// Position in buffer to retrieve settings from

		// Restore settings
		for (auto saver : configSavers) {
			memcpy(saver->settingsAddress, &flashConfig[configPos], saver->settingsSize);
			if (saver->validateSettings != nullptr) {
				saver->validateSettings();
			}
			configPos += saver->settingsSize;
		}
	}
}


void Config::EraseConfig()
{
	__disable_irq();									// Disable Interrupts
	FlashUnlock();										// Unlock Flash memory for writing
	FLASH->SR = flashAllErrors;							// Clear error flags in Status Register

	FlashErasePage(flashConfigPage - 1);

	FlashLock();										// Lock Flash
	__enable_irq();
	printf("Config Erased\r\n");
	}


void Config::ScheduleSave()
{
	// called whenever a config setting is changed to schedule a save after waiting to see if any more changes are being made
	scheduleSave = true;
	saveBooked = SysTickVal;
}


void Config::FlashUnlock()
{
	// Unlock the FLASH control register access
	if ((FLASH->CR & FLASH_CR_LOCK) != 0)  {
		FLASH->KEYR = 0x45670123U;					// These magic numbers unlock the flash for programming
		FLASH->KEYR = 0xCDEF89ABU;
	}
}


void Config::FlashLock()
{
	FLASH->CR |= FLASH_CR_LOCK;							// Lock the FLASH Registers access
}


void Config::FlashErasePage(uint8_t page)
{
	FLASH->CR &= ~FLASH_CR_PNB_Msk;
	FLASH->CR |= page << FLASH_CR_PNB_Pos;
	FLASH->CR |= FLASH_CR_PER;
	FLASH->CR |= FLASH_CR_STRT;
	FlashWaitForLastOperation();
	FLASH->CR &= ~FLASH_CR_PER;		// Unless this bit is cleared programming flash later throws a Programming Sequence error
}


bool Config::FlashWaitForLastOperation()
{
	if (FLASH->SR & flashAllErrors) {						// If any error occurred abort
		FLASH->SR = flashAllErrors;							// Clear all errors
		return false;
	}

	while ((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY) {

	}

	if ((FLASH->SR & FLASH_SR_EOP) == FLASH_SR_EOP) {		// Check End of Operation flag
		FLASH->SR = FLASH_SR_EOP;							// Clear FLASH End of Operation pending bit
	}

	return true;
}


bool Config::FlashProgram(uint32_t* dest_addr, uint32_t* src_addr, size_t size)
{
	// - requires that 64 bit words are written in rows of 32 bits
	if (FlashWaitForLastOperation()) {
		FLASH->CR |= FLASH_CR_PG;

		__ISB();
		__DSB();

		// Each write block is 64 bits
		for (uint16_t b = 0; b < std::ceil(static_cast<float>(size) / 8); ++b) {
			for (uint8_t i = 0; i < 2; ++i) {
				*dest_addr = *src_addr;
				++dest_addr;
				++src_addr;
			}

			if (!FlashWaitForLastOperation()) {
				FLASH->CR &= ~FLASH_CR_PG;				// Clear programming flag
				return false;
			}
		}

		__ISB();
		__DSB();

		FLASH->CR &= ~FLASH_CR_PG;						// Clear programming flag
	}
	return true;
}


