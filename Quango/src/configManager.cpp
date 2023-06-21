#include "configManager.h"
#include <cmath>
#include <cstring>
#include "calib.h"


Config configManager;

// called whenever a config setting is changed to schedule a save after waiting to see if any more changes are being made
void Config::ScheduleSave()
{
	scheduleSave = true;
	saveBooked = SysTickVal;
}


// Write config settings to Flash memory
bool Config::SaveConfig(bool eraseOnly)
{
	scheduleSave = false;
	bool result = true;

	uint32_t cfgSize = SetConfig();

	__disable_irq();					// Disable Interrupts
	FlashUnlock();						// Unlock Flash memory for writing
	FLASH->SR = FLASH_ALL_ERRORS;		// Clear error flags in Status Register

	// Check if flash needs erasing
	for (uint32_t i = 0; i < BufferSize / 4; ++i) {
		if (flashConfigAddr[i] != 0xFFFFFFFF) {
			FlashErasePage(flashConfigPage - 1);			// Erase page
			break;
		}
	}

	if (!eraseOnly) {
		result = FlashProgram(flashConfigAddr, reinterpret_cast<uint32_t*>(&configBuffer), cfgSize);
	}

	FlashLock();						// Lock Flash
	__enable_irq(); 					// Enable Interrupts

	return result;
}


uint32_t Config::SetConfig()
{
	// Serialise config values into buffer
	memset(configBuffer, 0xF, sizeof(configBuffer));				// Clear buffer
	strncpy(reinterpret_cast<char*>(configBuffer), "CFG", 4);		// Header
	configBuffer[4] = configVersion;
	uint32_t configPos = 8;											// Position in buffer to store data
	uint32_t configSize = 0;										// Holds the size of each config buffer

	uint8_t* cfgBuffer = nullptr;

	// Envelope settings
	configSize = calib.SerialiseConfig(&cfgBuffer);
	memcpy(&configBuffer[configPos], cfgBuffer, configSize);
	configPos += configSize;

	// Footer
	strncpy(reinterpret_cast<char*>(&configBuffer[configPos]), "END", 4);
	configPos += 4;
	return configPos;
}


// Restore configuration settings from flash memory
void Config::RestoreConfig()
{
	uint8_t* flashConfig = reinterpret_cast<uint8_t*>(flashConfigAddr);

	// Check for config start and version number
	if (strcmp((char*)flashConfig, "CFG") == 0 && flashConfig[4] == configVersion) {
		uint32_t configPos = 8;											// Position in buffer to store data

		// Envelope Settings
		configPos += calib.StoreConfig(&flashConfig[configPos]);
	}
}


// Unlock the FLASH control register access
void Config::FlashUnlock()
{
	if ((FLASH->CR & FLASH_CR_LOCK) != 0)  {
		FLASH->KEYR = 0x45670123U;					// These magic numbers unlock the flash for programming
		FLASH->KEYR = 0xCDEF89ABU;
	}
}


// Lock the FLASH Registers access
void Config::FlashLock()
{
	FLASH->CR |= FLASH_CR_LOCK;
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
	if (FLASH->SR & FLASH_ALL_ERRORS) {						// If any error occurred abort
		FLASH->SR = FLASH_ALL_ERRORS;						// Clear all errors
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


