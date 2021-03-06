#pragma once

#include <stdint.h>

typedef void (*SchedCb)();

enum class TaskId : uint8_t {
	// The order of these entries represents a sort of priority system,
	// in that the first elements will always be checked first.
	Deployment,  // This should be first because the Kalman filter is fairly sensitive to delays.
	Accel,
	GpsRead,
	Gps,
	Baro,
	PacketSync,
	SdFlush,
	Print,
	Blink,
	Count,
};

class Task {
public:
	Task(SchedCb cb, uint32_t period_us, uint32_t time_requirement_us = 0, uint32_t max_delta_us = 0xFFFFFFFFL) :
		callback(cb), last_run_us(0), period_us(period_us),
		time_requirement_us(time_requirement_us), max_delta_us(max_delta_us) {}
	Task() : callback(nullptr) {}

	SchedCb callback;
	uint32_t last_run_us, period_us, time_requirement_us, max_delta_us;
};

void scheduler_add(TaskId tid, Task task);

// Runs scheduled tasks.  Returns the micros time when the next task is scheduled.
uint32_t scheduler();
