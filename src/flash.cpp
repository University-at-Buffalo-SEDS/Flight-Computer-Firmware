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

static const SPISettings spi_settings(133'000'000, MSBFIRST, SPI_MODE0);

enum class FlashInstruction : uint8_t {
	PAGE_PROGRAM = 0x02,
	READ_STATUS_REGISTER_1 = 0x05,
	WRITE_ENABLE = 0x06,
	FAST_READ = 0x0B,
	BLOCK_ERASE_32KB = 0x52,
};

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
}

void flash_write_enable()
{
	spi_begin();
	FLASH_SPI.transfer((uint8_t)FlashInstruction::WRITE_ENABLE);
	spi_end();
}

bool flash_busy()
{
	spi_begin();
	FLASH_SPI.transfer((uint8_t)FlashInstruction::READ_STATUS_REGISTER_1);
	bool busy = (bool)(SPI.transfer(0) & 0x01);
	spi_end();
	return busy;
}

void flash_write(size_t page_addr, uint8_t page[FLIGHT_FLASH_PAGE_SIZE])
{
	flash_write_enable();
	spi_begin();
	FLASH_SPI.transfer((uint8_t)FlashInstruction::PAGE_PROGRAM);

	FLASH_SPI.transfer(page_addr & 0xFF00);
	FLASH_SPI.transfer(page_addr & 0xFF);
	FLASH_SPI.transfer(0x00);

	for (size_t i = 0; i < FLIGHT_FLASH_PAGE_SIZE; ++i) {
		FLASH_SPI.transfer(page[i]);
	}

	spi_end();

#ifndef NDEBUG
	// Validate write
	spi_begin();
	FLASH_SPI.transfer((uint8_t)FlashInstruction::FAST_READ);

	FLASH_SPI.transfer(page_addr & 0xFF00);
	FLASH_SPI.transfer(page_addr & 0xFF);
	FLASH_SPI.transfer(0x00);
	FLASH_SPI.transfer(0x00);

	for (size_t i = 0; i < FLIGHT_FLASH_PAGE_SIZE; ++i) {
		assert(page[i] == FLASH_SPI.transfer(0x00));
	}

	spi_end();
#endif
}

void flash_read(size_t page_addr, uint8_t page[FLIGHT_FLASH_PAGE_SIZE])
{
	flash_write_enable();
	spi_begin();
	FLASH_SPI.transfer((uint8_t)FlashInstruction::FAST_READ);

	FLASH_SPI.transfer(page_addr & 0xFF00);
	FLASH_SPI.transfer(page_addr & 0xFF);
	FLASH_SPI.transfer(0x00);
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

	FLASH_SPI.transfer(page_addr & 0xFF00);
	FLASH_SPI.transfer(page_addr & 0xFF);
	FLASH_SPI.transfer(0x00);

	spi_end();
}
