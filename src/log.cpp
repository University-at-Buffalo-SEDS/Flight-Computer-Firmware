#include <Arduino.h>
#include <EEPROM.h>

#include "log.hpp"
#include "config.hpp"
#include "util.hpp"
#include "flash.hpp"
#include "scheduler.hpp"

#define LOG_BUF_SIZE (PRELOG_MS / KALMAN_PERIOD)
#define LOG_WRITE_BUF_SIZE (sizeof(LogMessage) * LOG_BUF_SIZE)

void log_step();
void log_print_msg(const LogMessage &msg);

RingBuffer<LogMessage, LOG_BUF_SIZE> log_buf;
RingBuffer<uint8_t, LOG_WRITE_BUF_SIZE> write_buf;

static bool write_enabled = false;
static size_t current_page = 0;
static size_t written_pages = 0;
static uint8_t flight_num;
static bool current_block_erased = false;

void log_setup()
{
	flash_setup();

	flight_num = EEPROM.read(EEPROM_FLIGHT);
	current_page = FLIGHT_FLASH_FLIGHT_SIZE * flight_num;

	scheduler_add(TaskId::LogFlush, Task(log_step, 100'000L, 30'000L, 250'000L));
}

void log_start()
{
	write_enabled = true;
	// Flight started, advance to next flight.
	EEPROM.write(EEPROM_FLIGHT, wrapping_add(flight_num, 1, FLIGHT_FLASH_FLIGHTS));
	// Run one step to move records from the log buffer to the write buffer
	// and start the first erase operation.
	log_step();
}

void log_stop()
{
	write_enabled = false;
}

void log_step()
{
	// Don't do anything if we haven't started flight yet
	// or if we've run out of storage space.
	if (!write_enabled || written_pages > FLIGHT_FLASH_FLIGHT_SIZE) {
		return;
	}

	// Move messages from log buffer to write buffer
	LogMessage temp;
	while (log_buf.used() > 0 && write_buf.available() >= sizeof(LogMessage)) {
		log_buf.pop(&temp);
		write_buf.push(reinterpret_cast<uint8_t*>(&temp), sizeof(temp), false);
	}

	// Write messages from write buffer
	uint8_t page[FLASH_PAGE_SIZE];
	while (write_buf.pop(page, FLASH_PAGE_SIZE)) {
		if (written_pages > FLIGHT_FLASH_FLIGHT_SIZE || flash_busy()) {
			break;
		}

		if (!current_block_erased && current_page % FLIGHT_FLASH_PAGES_PER_BLOCK == 0) {
			// Current page is in the next block.
			// Erase it first.
			flash_erase(current_page);
			current_block_erased = true;
			break;
		}
		// Clear erased flag
		current_block_erased = false;

		flash_write(current_page, page);

		++current_page;
		++written_pages;
	}
}

void log_add(const LogMessage &data)
{
	if (!log_buf.push(data, true) && write_enabled) {
		Serial.println("Log buffer overflow!");
	}
}

void log_print()
{
	if (write_enabled) {
		Serial.println("Cannot read while in flight!");
		return;
	}

	uint8_t first_flight = EEPROM.read(EEPROM_FLIGHT);
	uint8_t page[FLASH_PAGE_SIZE];
	LogMessage msg;
	uint32_t last_time = 0;

	for (size_t flight_i = 0; flight_i < FLIGHT_FLASH_FLIGHTS; ++flight_i) {
		RingBuffer<uint8_t, LOG_WRITE_BUF_SIZE> read_buf;
		uint8_t flight = wrapping_add(first_flight, flight_i, FLIGHT_FLASH_FLIGHTS);
		size_t flight_addr = FLIGHT_FLASH_FLIGHT_SIZE * flight;
		bool flight_done = false;

		for (size_t page_i = 0; page_i < FLIGHT_FLASH_FLIGHT_SIZE; ++page_i) {
			flash_read(flight_addr + page_i, page);

			if (!read_buf.push(page, FLASH_PAGE_SIZE, false)) {
				Serial.println("Read buffer error.");
				break;
			}

			while (read_buf.pop(reinterpret_cast<uint8_t*>(&msg), sizeof(LogMessage))) {
				// Flight ends if difference between timestamps is too large,
				// there is no difference, or the checksum doesn't match.
				if (page_i > 0 && (msg.time_ms - last_time > 1000 ||
						msg.time_ms == last_time ||
						struct_checksum(msg) != msg.checksum)) {
					flight_done = true;
					break;
				}

				log_print_msg(msg);

				last_time = msg.time_ms;
			}

			if (flight_done) {
				break;
			}
		}
		Serial.println("---");
	}
}

void log_print_msg(const LogMessage &msg)
{
	Serial.print(msg.time_ms);
	Serial.print(',');
	Serial.print(msg.state.pos);
	Serial.print(',');
	Serial.print(msg.state.rate);
	Serial.print(',');
	Serial.print(msg.state.accel);
	Serial.print(',');
	Serial.print(msg.temp);
	Serial.print(',');
	Serial.print(msg.altitude);
	Serial.print(',');
	Serial.print(msg.accel_x);
	Serial.print(',');
	Serial.print(msg.accel_y);
	Serial.print(',');
	Serial.print(msg.accel_z);
	Serial.print(',');
	Serial.print(msg.lat);
	Serial.print(',');
	Serial.print(msg.lon);
	Serial.print(',');
	Serial.print(msg.gps_alt);
	Serial.print(',');
	Serial.print(msg.batt_v);
	Serial.println();
}
