#include <Arduino.h>
#include <SPI.h>
#include <cassert>

#include "flash.hpp"
#include "config.hpp"
#include "log.hpp"

static const SPISettings spi_settings(100'000'000, MSBFIRST, SPI_MODE0);

static void spi_begin()
{
	SPI.beginTransaction(spi_settings);
	digitalWrite(PIN_FLASH_CS, LOW);
}

static void spi_end()
{
	digitalWrite(PIN_FLASH_CS, HIGH);
	SPI.endTransaction();
}

void flash_setup()
{	pinMode(PIN_FLASH_CS, OUTPUT);
	digitalWrite(PIN_FLASH_CS, HIGH);
}

void flash_write_enable()
{
	spi_begin();
	SPI.transfer(0x06);
	spi_end();
}

bool flash_busy()
{
	spi_begin();
	SPI.transfer(0x05);
	bool busy = SPI.transfer(0) & 0x01;
	spi_end();
	return busy;
}

void flash_write(size_t page_addr, uint8_t page[FLIGHT_FLASH_PAGE_SIZE])
{
	flash_write_enable();
	spi_begin();
	SPI.transfer(0x02);

	SPI.transfer(page_addr & 0xFF00);
	SPI.transfer(page_addr & 0xFF);
	SPI.transfer(0x00);

	for (size_t i = 0; i < FLIGHT_FLASH_PAGE_SIZE; ++i) {
		SPI.transfer(page[i]);
	}

	spi_end();

	// Validate write
	spi_begin();
	SPI.transfer(0x0B);

	SPI.transfer(page_addr & 0xFF00);
	SPI.transfer(page_addr & 0xFF);
	SPI.transfer(0x00);
	SPI.transfer(0x00);

	for (size_t i = 0; i < FLIGHT_FLASH_PAGE_SIZE; ++i) {
		assert(page[i] == SPI.transfer(0x00));
	}

	spi_end();
}

void flash_read(size_t page_addr, uint8_t page[FLIGHT_FLASH_PAGE_SIZE])
{
	flash_write_enable();
	spi_begin();
	SPI.transfer(0x0B);

	SPI.transfer(page_addr & 0xFF00);
	SPI.transfer(page_addr & 0xFF);
	SPI.transfer(0x00);
	SPI.transfer(0x00);

	for (size_t i = 0; i < FLIGHT_FLASH_PAGE_SIZE; ++i) {
		page[i] = SPI.transfer(0x00);
	}

	spi_end();
}

void flash_erase(size_t page_addr)
{
	flash_write_enable();
	spi_begin();
	SPI.transfer(0x52);

	SPI.transfer(page_addr & 0xFF00);
	SPI.transfer(page_addr & 0xFF);
	SPI.transfer(0x00);

	spi_end();
}