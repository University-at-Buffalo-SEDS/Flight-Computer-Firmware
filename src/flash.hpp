#pragma once

#define FLASH_PAGE_SIZE (256)
#define FLASH_BLOCK_SIZE (32768)
#define FLASH_PAGES_PER_BLOCK (FLASH_BLOCK_SIZE / FLASH_PAGE_SIZE)
#define FLASH_PAGE_COUNT (1<<21 / FLASH_PAGE_SIZE)
#define FLASH_FLIGHTS (4)
#define FLASH_FLIGHT_SIZE (FLASH_PAGE_COUNT / FLASH_FLIGHTS)

void flash_setup();
void flash_erase(size_t start_page);
void flash_write(size_t page_addr, uint8_t page[FLASH_PAGE_SIZE]);
void flash_read(size_t page_addr, uint8_t page[FLASH_PAGE_SIZE]);
bool flash_busy();