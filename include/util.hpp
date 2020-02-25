#pragma once

#include <climits>
#include <cstdint>
#include <cstddef>
#include <cmath>
#include "config.hpp"
#include "kalman.hpp"

#if DEBUG
#define DEBUG_SECTION(x) x
#else
#define DEBUG_SECTION(x)
#endif

#define ROUND_UP_ALIGN(type, len) (((len) + alignof(type) - 1) & ~(alignof(type) - 1))

uint8_t calc_checksum(const uint8_t *data, size_t len);

template<typename S>
inline static uint8_t struct_checksum(const S &val)
{
	constexpr size_t size_if_checksum_last = ROUND_UP_ALIGN(S,
			offsetof(S, S::checksum) + sizeof(S::checksum));
	static_assert(sizeof(S) == size_if_checksum_last,
			"Checksum must be last field in struct!");
	static_assert(sizeof(S::checksum) == 1, "Checksums must be 8 bits.");

	// Don't include checksum field in calculation of checksum
	return calc_checksum(reinterpret_cast<const uint8_t *>(&val), offsetof(S, S::checksum));
}

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

inline static size_t wrapping_add(size_t val, size_t increment, size_t max) {
	return (val + increment) % max;
}

template <typename T, unsigned int Cap>
class RingBuffer {
	size_t head, tail, size;
	T buf[Cap];

public:
	RingBuffer() : head(0), tail(0), size(0), buf() {}

	size_t available() { return Cap - size; }
	size_t used() { return size; }

	// Adds a single element to the end of the ring buffer.
	// Returns whether there was enough space to add the
	// new element without overwriting old elements.
	bool push(const T &val, bool overwrite);

	// Pops a single element from the ring buffer and sets val to its value.
	// Returns whether there was a value to pop.
	bool pop(T *val);

	// Pushes an array of data into the ring buffer.
	// If overwrite is false and there is not enough space to push
	// all of the items, this will return false and do nothing.
	// Otherwise, returns whether there was enough space to add the
	// items without overwriting old data.
	bool push(const T *data, size_t count, bool overwrite);

	// Pops an array of data from the ring buffer.
	// If the array does not have count items to pop this
	// immediately returns false and does nothing.
	// Otherwise, this copies count elements into data and returns true.
	bool pop(T *data, size_t count);
};

template <typename T, unsigned int Cap>
bool RingBuffer<T, Cap>::push(const T &val, bool overwrite)
{
	if (size == Cap && !overwrite) {
		return false;
	}

	buf[tail] = val;

	tail = wrapping_add(tail, 1, Cap);

	if (size < Cap) {
		++size;
		return true;
	}

	head = wrapping_add(head, 1, Cap);
	return false;
}

template <typename T, unsigned int Cap>
bool RingBuffer<T, Cap>::pop(T *val) {
	if (size == 0) {
		return false;
	}
	*val = buf[head];
	--size;
	head = wrapping_add(head, 1, Cap);
	return true;
}

template <typename T, unsigned int Cap>
bool RingBuffer<T, Cap>::push(const T *data, size_t count, bool overwrite) {
	if (size + count > Cap && !overwrite) {
		return false;
	}
	if (count > Cap) {
		data += count - Cap;
		count = Cap;
	}
	if (tail + count > Cap) {
		// Range wraps around, copy the start of the data from
		// tail to the limit of the buffer, then copy the rest
		// after the start of the buffer.
		size_t count_to_tail = Cap - tail;
		size_t remainder = count - count_to_tail;
		memcpy(buf + tail, data, sizeof(T) * count_to_tail);
		memcpy(buf, data + count_to_tail, sizeof(T) * remainder);
	} else {
		// We have enough space after the tail before the limit of the buffer.
		// Simply copy the data to the tail.
		memcpy(buf + tail, data, sizeof(T) * count);
	}
	tail = wrapping_add(tail, count, Cap);
	if (size + count > Cap) {
		// We've advanced past the head, set it to the tail
		head = tail;
	}
	size += count;
	if (size > Cap) {
		size = Cap;
		return false;
	}
	return true;
}

template <typename T, unsigned int Cap>
bool RingBuffer<T, Cap>::pop(T *data, size_t count) {
	if (size < count) {
		return false;
	}
	if (head + count > Cap) {
		size_t count_from_head = Cap - head;
		size_t remainder = count - count_from_head;
		memcpy(data, buf + head, sizeof(T) * count_from_head);
		memcpy(data + count_from_head, buf, sizeof(T) * remainder);
	} else {
		memcpy(data, buf + head, sizeof(T) * count);
	}
	size -= count;
	head = wrapping_add(head, count, Cap);
	return true;
}
