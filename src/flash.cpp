#include <Arduino.h>
#include <SPI.h>
#include <cassert>

#include "flash.hpp"
#include "config.hpp"
#include "log.hpp"

#if defined(FLIGHT_FLASH_MOSI_PIN) || defined(FLIGHT_FLASH_MISO_PIN) || defined(FLIGHT_FLASH_SCK_PIN)
SPIClass FLASH_SPI(FLIGHT_FLASH_MOSI_PIN, FLIGHT_FLASH_MISO_PIN, FLIGHT_FLASH_SCK_PIN);
#else
#define FLASH_SPI SPI
#endif

#define W25Q16_DEVICE_ID 0x14

// 18Mhz is max spopported by STM32 when SYSCLK is 72MHz.
// Need to use FAST_READ instruction if above 50MHz
static const SPISettings spi_settings(18'000'000, MSBFIRST, SPI_MODE0);

enum class FlashInstruction : uint8_t {
	PAGE_PROGRAM = 0x02,
	READ_DATA = 0x03,
	READ_STATUS_REGISTER_1 = 0x05,
	WRITE_ENABLE = 0x06,
	BLOCK_ERASE_32KB = 0x52,
	RELEASE_POWER_DOWN_DEVICE_ID = 0xAB,
};

#define FLIGHT_FLASH_BUSY_MASK 0x01
#define FLIGHT_FLASH_WEL_MASK  0x02

static uint32_t flash_status_1();

static void spi_begin()
{
	FLASH_SPI.beginTransaction(spi_settings);
	digitalWrite(PIN_FLASH_CS, LOW);
}

static void spi_end()
{
	digitalWrite(PIN_FLASH_CS, HIGH);
	FLASH_SPI.endTransaction();
}

void flash_setup()
{
	pinMode(PIN_FLASH_CS, OUTPUT);
	digitalWrite(PIN_FLASH_CS, HIGH);
	FLASH_SPI.begin();

	spi_begin();
	FLASH_SPI.transfer((uint8_t)FlashInstruction::RELEASE_POWER_DOWN_DEVICE_ID);
	for (size_t i = 0; i < 3; ++i) {
		FLASH_SPI.transfer(0);
	}
	uint8_t device_id = FLASH_SPI.transfer(0);
	spi_end();

	if (device_id != W25Q16_DEVICE_ID) {
		Serial.println(F("Failed to set up flash!"));
		while (true) { delay(1); }
	} else {
		Serial.println(F("Flash detected."));
	}
}

void flash_write_enable()
{
	assert(!flash_busy());

	spi_begin();
	FLASH_SPI.transfer((uint8_t)FlashInstruction::WRITE_ENABLE);
	spi_end();

	assert(flash_status_1() & FLIGHT_FLASH_WEL_MASK);
}

static uint32_t flash_status_1()
{
	spi_begin();
	FLASH_SPI.transfer((uint8_t)FlashInstruction::READ_STATUS_REGISTER_1);
	uint8_t status = FLASH_SPI.transfer(0);
	spi_end();
	return status;
}

bool flash_busy()
{
	return flash_status_1() & FLIGHT_FLASH_BUSY_MASK;
}

void flash_write(size_t page_addr, uint8_t page[FLIGHT_FLASH_PAGE_SIZE])
{
	flash_write_enable();
	spi_begin();
	FLASH_SPI.transfer((uint8_t)FlashInstruction::PAGE_PROGRAM);

	FLASH_SPI.transfer((page_addr >> 8) & 0xFF);
	FLASH_SPI.transfer(page_addr & 0xFF);
	FLASH_SPI.transfer(0x00);

	for (size_t i = 0; i < FLIGHT_FLASH_PAGE_SIZE; ++i) {
		FLASH_SPI.transfer(page[i]);
	}

	spi_end();

	assert(flash_busy());

#ifndef NDEBUG
	// Validate write
	while (flash_busy()) { delayMicroseconds(1); }

	spi_begin();
	FLASH_SPI.transfer((uint8_t)FlashInstruction::READ_DATA);

	FLASH_SPI.transfer((page_addr >> 8) & 0xFF);
	FLASH_SPI.transfer(page_addr & 0xFF);
	FLASH_SPI.transfer(0x00);

	for (size_t i = 0; i < FLIGHT_FLASH_PAGE_SIZE; ++i) {
		assert(page[i] == FLASH_SPI.transfer(0x00));
	}

	spi_end();
#endif
}

void flash_read(size_t page_addr, uint8_t page[FLIGHT_FLASH_PAGE_SIZE])
{
	assert(!flash_busy());

	spi_begin();
	FLASH_SPI.transfer((uint8_t)FlashInstruction::READ_DATA);

	FLASH_SPI.transfer((page_addr >> 8) & 0xFF);
	FLASH_SPI.transfer(page_addr & 0xFF);
	FLASH_SPI.transfer(0x00);

	for (size_t i = 0; i < FLIGHT_FLASH_PAGE_SIZE; ++i) {
		page[i] = FLASH_SPI.transfer(0x00);
	}

	spi_end();
}

void flash_erase(size_t page_addr)
{
	flash_write_enable();
	spi_begin();
	FLASH_SPI.transfer((uint8_t)FlashInstruction::BLOCK_ERASE_32KB);

	FLASH_SPI.transfer((page_addr >> 8) & 0xFF);
	FLASH_SPI.transfer(page_addr & 0xFF);
	FLASH_SPI.transfer(0x00);
	spi_end();

	assert(flash_busy());
}
