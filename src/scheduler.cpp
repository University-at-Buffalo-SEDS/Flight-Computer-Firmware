#include "scheduler.hpp"
#include "util.hpp"

#include <Arduino.h>

static Task tasks[(int)TaskId::Count];

void scheduler_add(TaskId tid, const Task &task)
{
	tasks[(uint8_t)tid] = task;
}

static void run_task(uint32_t sched_start, uint32_t time_since_last_run, Task *task, uint8_t task_id) {
	uint32_t task_start = micros();
	task->callback();
	uint32_t task_time = delta(task_start, micros());

	// Warn if any individual task takes too long.
	uint32_t max_time = (task->time_requirement_us != 0) ? task->time_requirement_us : 200;
	if (task_time > max_time) {
		Serial.print(F("Slow task "));
		Serial.print(task_id);
		Serial.print(": ");
		Serial.println(task_time);
	}

	// Print a warning if we start running more than 1ms behind.
	// Ignore this on the first run though.
	if (time_since_last_run > task->period_us + 1000 && task->last_run_us > 0) {
		Serial.print(F("Scheduling overrun: "));
		Serial.print(task_id);
		Serial.print(':');
		Serial.println(time_since_last_run - task->period_us);
	}

	task->last_run_us = sched_start;
}

uint32_t schedule()
{
	uint32_t sched_start = micros();
	// Time that caller should wait after sched_start before calling this again.
	uint32_t wait_time = UINT32_MAX;

	for (size_t i = 0; i < (size_t)TaskId::Count; ++i) {
		Task &task = tasks[i];
		if (task.callback == nullptr) {
			continue;
		}

		uint32_t time_since_last_run = delta(task.last_run_us, sched_start);
		// If it is time to run this task.
		if (time_since_last_run >= task.period_us) {
			run_task(sched_start, time_since_last_run, &task, i);
		}

		uint32_t task_next_run_us = task.last_run_us + task.period_us;
		uint32_t task_wait_time = delta(sched_start, task_next_run_us);
		// If this task will have to be run before wait_time, then update
		// wait_time.
		if (task_wait_time < wait_time) {
			wait_time = task_wait_time;
		}
	}

	uint32_t used_time = delta(sched_start, micros());
	if (used_time < wait_time) {
		return wait_time - used_time;
	}
	return 0;
}
