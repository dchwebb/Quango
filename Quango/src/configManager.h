#pragma once

#include "initialisation.h"


// Struct added to classes that need settings saved
struct ConfigSaver {
	void* settingsAddress;
	uint32_t settingsSize;
	void (*validateSettings)(void);		// function pointer to method that will validate config settings when restored
};


class Config {
public:
	static constexpr uint32_t configVersion = 1;
	
	// STM32G473 category 3 device 256k Flash in 128 pages of 2048k (though memory browser indicates part actually has 512k??)
	static constexpr uint32_t flashConfigPage = 60;
	static constexpr uint32_t flashPageSize = 2048;
	uint32_t* const flashConfigAddr = reinterpret_cast<uint32_t* const>(FLASH_BASE + flashPageSize * (flashConfigPage - 1));

	bool scheduleSave = false;
	uint32_t saveBooked = false;

	// Constructor taking multiple config savers: Get total config block size from each saver
	Config(std::initializer_list<ConfigSaver*> initList) : configSavers(initList) {
		for (auto saver : configSavers) {
			settingsSize += saver->settingsSize;
		}
		// Ensure config size (plus 4 byte header) is aligned to 8 byte boundary
		settingsSize = AlignTo8Bytes(settingsSize + sizeof(ConfigHeader));
	}

	void ScheduleSave();				// called whenever a config setting is changed to schedule a save after waiting to see if any more changes are being made
	bool SaveConfig();
	void EraseConfig();					// Erase flash page containing config
	void RestoreConfig();				// gets config from Flash, checks and updates settings accordingly

private:
	static constexpr uint32_t flashAllErrors = FLASH_SR_OPERR  | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR | FLASH_SR_RDERR  | FLASH_SR_OPTVERR;

	const std::vector<ConfigSaver*> configSavers;
	uint32_t settingsSize = 0;			// Size of all settings from each config saver module + size of config header

	const char ConfigHeader[4] = {'C', 'F', 'G', configVersion};
	int32_t currentSettingsOffset = -1;	// Offset within flash page to block containing active/latest settings

	void FlashUnlock();
	void FlashLock();
	void FlashErasePage(uint8_t Sector);
	bool FlashWaitForLastOperation();
	bool FlashProgram(uint32_t* dest_addr, uint32_t* src_addr, size_t size);

	static const inline uint32_t AlignTo8Bytes(uint32_t val) {
		val += 7;
		val >>= 3;
		val <<= 3;
		return val;
	}
};

extern Config config;
