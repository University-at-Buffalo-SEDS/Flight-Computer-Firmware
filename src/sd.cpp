#include "config.hpp"
#if SD_LOG
#include "sd.hpp"
#include "util.hpp"
#include "scheduler.hpp"
#include <SdFat.h>

#define SD_PRELOG_MS 2000
#define SD_PCBUF_SIZE (SD_PRELOG_MS / KALMAN_PERIOD)

void sd_flush();

static SdFatSdio sd;
static FatFile data_log_file;
static PrintFile messages_log_file;

static bool commit_enabled = false;
// Ring buffer to hold samples until commit is enabled so that samples from launch are preserved.
static SdDataRecord precommit_buf[SD_PCBUF_SIZE];
// Index where next record should be written
static uint16_t pcbuf_idx = 0;

void sd_setup()
{
	if (!sd.begin()) {
		sd.initErrorHalt("SdFatSdio begin() failed!");
	}
	sd.chvol();
	char *fn_data = strdup("Data0000.log");
	char *fn_msgs = strdup("Msgs0000.log");
	for (uint16_t i = 0; true; ++i) {
		if (i > 9999) {
			sd.errorHalt("Failed to find free file name!");
		}
		uint16_t j = i;
		fn_data[7] = fn_msgs[7] = (j % 10) + '0'; j /= 10;
		fn_data[6] = fn_msgs[6] = (j % 10) + '0'; j /= 10;
		fn_data[5] = fn_msgs[5] = (j % 10) + '0'; j /= 10;
		fn_data[4] = fn_msgs[4] = (j % 10) + '0';
		if (!sd.exists(fn_data) && !sd.exists(fn_msgs)) {
			break;
		}
	}
	if (!data_log_file.open(fn_data, O_WRITE | O_AT_END | O_CREAT)) {
		sd.errorHalt("Failed to open data log!");
	}
	if (!messages_log_file.open(fn_msgs, O_WRITE | O_AT_END | O_CREAT)) {
		sd.errorHalt("Failed to open message log!");
	}

	// Flush every 4s when we have enough free time, with a maximum delay of 6s.
	scheduler_add(TaskId::SdFlush, Task(sd_flush, 4'000'000L, 30'000L, 6'000'000L));
}

void sd_flush()
{
	if (!commit_enabled) {
		return;
	}

	uint32_t start = micros();
	data_log_file.sync();
	Serial.print(F("SD Flush Data: "));
	Serial.print(delta(start, micros()));

	start = micros();
	messages_log_file.sync();
	Serial.print(F(", SD Flush Messages: "));
	Serial.println(delta(start, micros()));
}


static void pcbuf_increment()
{
	++pcbuf_idx;
	if (pcbuf_idx >= SD_PCBUF_SIZE) {
		pcbuf_idx = 0;
	}
}

void sd_log(const SdDataRecord &data)
{
	if (commit_enabled) {
		data_log_file.write((char*)(&data), sizeof(data));
	} else {
		precommit_buf[pcbuf_idx] = data;
		pcbuf_increment();
	}
}

void sd_commit(bool enable)
{
	if (commit_enabled == enable) return;
	
	if (enable) {
		// Copy data from the ring buffer to the file buffer.
		// Data will be flushed on the next scheduled run of the flush task.
		data_log_file.write((char*)(&precommit_buf[pcbuf_idx]),
				sizeof(*precommit_buf) * (SD_PCBUF_SIZE - pcbuf_idx));
		if (pcbuf_idx > 0) {
			data_log_file.write((char*)precommit_buf,
				sizeof(*precommit_buf) * pcbuf_idx);
		}
	} else {
		sd_flush();
	}
	commit_enabled = enable;
}

Print *sd_messages() { return &messages_log_file; }
#endif




#if 0

#define FILE_BLOCK_COUNT 1024  // 512kiB
#define BUFFER_BLOCK_COUNT 8
#define BLOCK_RECORD_COUNT (512 / sizeof(SdDataRecord))

struct SdDataBlock {
	SdDataRecord records[BLOCK_RECORD_COUNT];
	uint8_t _pad[512 - (BLOCK_RECORD_COUNT * sizeof(SdDataRecord))];
};

static_assert(sizeof(SdDataBlock) == 512, "SdDataBlock is not the size of a block");

static SdFatSdio sd;
static FatFile data_log_file;
static PrintFile messages_log_file;

SdDataBlock blocks[BUFFER_BLOCK_COUNT];
PtrStack<SdDataBlock, BUFFER_BLOCK_COUNT> emptyStack;
PtrQueue<SdDataBlock, BUFFER_BLOCK_COUNT> blockQueue;
SdDataBlock *curBlock = nullptr;
uint8_t curBlockIdx = 0;
uint32_t blockNum = 0;
uint32_t beginBlockAddr, endBlockAddr, curBlockAddr;

void sd_setup()
{
	if (!sd.begin()) {
		sd.initErrorHalt("SdFatSdio begin() failed!");
	}
	sd.chvol();
	char *fn_data = strdup("Data0000.log");
	char *fn_msgs = strdup("Msgs0000.log");
	for (uint16_t i = 0; true; ++i) {
		if (i > 9999) {
			sd.errorHalt("Failed to find free file name!");
		}
		uint16_t j = i;
		fn_data[7] = fn_msgs[7] = (j % 10) + '0'; j /= 10;
		fn_data[6] = fn_msgs[6] = (j % 10) + '0'; j /= 10;
		fn_data[5] = fn_msgs[5] = (j % 10) + '0'; j /= 10;
		fn_data[4] = fn_msgs[4] = (j % 10) + '0';
		if (!sd.exists(fn_data) && !sd.exists(fn_msgs)) {
			break;
		}
	}
	if (!messages_log_file.open(fn_msgs, O_WRITE | O_AT_END | O_CREAT)) {
		sd.errorHalt("Failed to open message log!");
	}

	if (!data_log_file.createContiguous(fn_data, 512 * FILE_BLOCK_COUNT)) {
		sd.errorHalt("createContiguous failed");
	}
	if (!data_log_file.contiguousRange(&beginBlockAddr, &endBlockAddr)) {
		sd.errorHalt("contiguousRange failed");
	}
	curBlockAddr = beginBlockAddr;
	
	sd_erase_data_log(beginBlockAddr, endBlockAddr);
	sd_init_buffers();

	// Flush every 1 second
	scheduler_add(TaskId::SdFlush, Task(sd_flush, 1'000'000L));
}

void sd_flush()
{
	if (sd.card()->isBusy()) {
		return;
	}
	// Get address of block to write.
	SdDataBlock *block = blockQueue.pop();
	if (block == nullptr) {
		return;
	}
	// Write block to SD.
	if (!sd.card()->writeData((uint8_t*)block)) {
		sd.errorHalt("Write data failed");
	}
	emptyStack.push(block);
	blockNum++;
	if (blockNum % FILE_BLOCK_COUNT == 0) {
		// We've reached the end, start back at the beginning.
		if (!sd.card()->writeStop()) {
			sd.errorHalt("writeStop failed");
		}
		if (!sd.card()->writeStart(data_log_file.firstBlock())) {
			error("writeStart failed");
		}
	}

	uint32_t start = micros();
	data_log_file.flush();
	Serial.print(F("SDFD: "));
	Serial.print(delta(start, micros()));

	start = micros();
	messages_log_file.flush();
	Serial.print(F(", SDFM: "));
	Serial.println(delta(start, micros()));
}

void sd_log(const SdDataRecord &data)
{
	if (curBlock == nullptr && !emptyStack.empty()) {
		curBlock = emptyStack.pop();
	}
	if (curBlock == nullptr) {
		sd.errorHalt("Overrun abort");
	} else {
		curBlock->records[curBlockIdx++] = data;
		if (curBlockIdx >= records_per_block) {
			fullQueue.push(curBlock);
			curBlock = nullptr;
			curBlockIdx = 0;
		}
	}

	data_log_file.write((char*)(&data), sizeof(data));
}

void sd_log(const char *msg)
{
	// TODO: Support PROGMEM strings?
	messages_log_file.write(msg, strlen(msg));
}

static void sd_erase_data_log(uint32_t beginBlock, uint32_t endBlock)
{
	// Max number of blocks to erase per erase call
	constexpr uint32_t ERASE_SIZE = 262'144L;

	uint32_t beginErase = beginBlock;
	uint32_t endErase;
	while (beginErase < endBlock) {
		endErase = beginErase + ERASE_SIZE;
		if (endErase > endBlock) {
			endErase = endBlock;
		}
		if (!sd.card()->erase(beginErase, endErase)) {
			sd.errorHalt("SD Log file erase failed");
		}
		beginErase = endErase + 1;
	}
}

static void sd_init_buffers()
{
	// Use SdFat's internal buffer.
	SdDataBlock *sdfat_buf = (SdDataBlock*)sd.vol()->cacheClear();
	if (sdfat_buf == nullptr) {
		sd.errorHalt("cacheClear failed");
	}
	emptyStack.push(sdfat_buf);
	// Put rest of buffers on the empty stack.
	for (int i = 0; i < BUFFER_BLOCK_COUNT - 1; i++) {
		emptyStack.push(&blocks[i]);
	}

	// Zero out the buffers so that the pad is always all zeros.
	memset(sdfat_buf, 0, sizeof(sdfat_buf));
	memset(blocks, 0, sizeof(blocks));
	static_assert(sizeof(blocks) == (sizeof(SdDataBlock) * BUFFER_BLOCK_COUNT)
}

{
	// Truncate file if recording stopped early.
	if (blockNum != FILE_BLOCK_COUNT) {
		Serial.println(F("Truncating file"));
		if (!data_log_file.truncate(512L * blockNum)) {
			sd.errorHalt("Can't truncate file");
		}
	}
}
#endif
