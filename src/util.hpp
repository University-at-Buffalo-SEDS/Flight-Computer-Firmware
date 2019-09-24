#pragma once

#include <limits.h>
#include <stdint.h>
#include <math.h>
#include "config.hpp"
#include "kalman.hpp"

enum class FlightPhase : uint8_t {
	Startup, // Doing initial setup
	Idle, // On launchpad, not launched.
	Launched,  // Launched, still ascending.
	DescendingWithDrogue, // Reached apogee, descending to main deployment altitude.
	DescendingWithMain,  // Main deployed, waiting to land or already landed.
	Landed,
};

inline static uint32_t delta(uint32_t old, uint32_t current)
{
	if (old > current) {
		return (UINT32_MAX - old) + current + 1;
	}
	return current - old;
}

inline static int32_t sdelta(uint32_t old, uint32_t current)
{
	return (int32_t)delta(old, current);
}

struct DelayedEstState {
	DelayedEstState() : old_old_est(NAN), old_est(NAN), est(0), count(0) {}
	kfloat_t old_old_est, old_est, est;
	uint8_t count;
};

// Calculates an average of the readings passed to it, with a delay.
// Used to calculate an estumate of the local gravity and altitude above sea level.
void calc_delayed_est(DelayedEstState &state, kfloat_t reading);

#if 0
template <typename T, uint8_t S>
class PtrStack {
	T *arr[S];
	uint8_t top;
public:
	PtrStack() : top(0) {}
	bool push(T *v) {
		if (top == S) return false;
		arr[top++] = v;
		return true;
	}
	T *pop() {
		if (top == 0) return nullptr;
		return arr[--top];
	}
};


template <typename T, uint8_t S>
class PtrQueue {
	T *arr[S];
	uint8_t head, tail, size;
	uint8_t next(uint8_t i) { return i < (S - 1) ? i + 1 : 0; }
public:
	PtrQueue() : head(0), tail(0), size(0) {}
	bool push(T *v) {
		if (size >= S) return false;
		arr[head] = v;
		head = next(head);
		++size;
		return true;
	}
	T pop() {
		if (size <= 0) return nullptr;
		T *v = arr[tail];
		tail = next(tail);
		--size;
		return v;
	}
	bool empty() { return head == tail; }
};
#endif

#if DEBUG
#define DEBUG_SECTION(x) x
#else
#define DEBUG_SECTION(x)
#endif
