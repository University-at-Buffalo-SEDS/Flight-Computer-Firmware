#include <Arduino.h>
#include "util.hpp"
#include "scheduler.hpp"

static Task tasks[(int)TaskId::Count];

void scheduler_add(TaskId tid, const Task &task)
{
	tasks[(uint8_t)tid] = task;
}

static void run_task(uint32_t sched_start, uint32_t time_since_last_run, Task &task, uint8_t task_id) {
	uint32_t task_start = micros();
	task.callback();
	uint32_t task_time = delta(task_start, micros());
	// Warn if any individual task takes too long.
	uint32_t max_time = (task.time_requirement_us != 0) ? task.time_requirement_us : 200;
	if (task_time > max_time) {
		Serial.print(F("Slow task "));
		Serial.print(task_id);
		Serial.print(": ");
		Serial.println(task_time);
	}
	task.last_run_us = sched_start;
	// Print a warning if we start running more than 1ms behind.
	// Ignore this on the first run though.
	if (time_since_last_run > task.period_us + 1000 && task.last_run_us > 0) {
		Serial.print(F("Scheduling overrun: "));
		Serial.print(task_id);
		Serial.print(':');
		Serial.println(time_since_last_run - task.period_us);
	}
}

uint32_t scheduler()
{
	uint32_t sched_start = micros();
	uint32_t next_task = sched_start - 1;

	// Calculate the time at which we will next have to run a task: next_task.
	// This is used to figure out how much free time we have.
	for (uint8_t i = 0; i < (uint8_t)TaskId::Count; ++i) {
		Task &task = tasks[i];
		if (task.callback == nullptr) {
			continue;
		}
		uint32_t time_since_last_run = delta(task.last_run_us, sched_start);
		if (time_since_last_run >= task.period_us) {
			// We are going to run commands in this iteration, we have no free time.
			next_task = sched_start;
			break;
		}
		uint32_t next_run = task.last_run_us + task.period_us;
		// If this task will have to be run before next_task, then update next_task.
		if (delta(sched_start, next_run) < delta(sched_start, next_task)) {
			next_task = next_run;
		}
	}

	for (uint8_t i = 0; i < (uint8_t)TaskId::Count; ++i) {
		Task &task = tasks[i];
		if (task.callback == nullptr) {
			continue;
		}
		uint32_t time_since_last_run = delta(task.last_run_us, sched_start);
		// If it is time to run this task.
		if (time_since_last_run >= task.period_us) {
			uint32_t free_time = sdelta(micros(), next_task);
			// Don't run the task if we don't have enough free time left to meet the time requirement.
			if (time_since_last_run > task.max_delta_us || task.time_requirement_us < free_time) {
				run_task(sched_start, time_since_last_run, task, i);
			}
		}
	}

	return next_task;
}
