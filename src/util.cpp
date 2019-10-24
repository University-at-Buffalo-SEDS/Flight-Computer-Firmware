#include "util.hpp"

void calc_delayed_est(DelayedEstState &state, kfloat_t reading)
{
	if (state.count > DELAYED_EST_SAMPLES) {
		state.old_old_est = state.old_est;
		state.old_est = state.est;
		state.count = 1;
	}
	// Compute a running average
	state.est *= (float)state.count / (state.count + 1);
	state.est += reading / (state.count + 1);
	state.count += 1;
}

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
	} else {
		head = wrapping_add(head, 1, Cap);
		return false;
	}
}

template <typename T, unsigned int Cap>
bool RingBuffer<T, Cap>::pop(T *val) {
	if (size == 0) {
		return false;
	}
	*val = buf[head];
	--size;
	head = wrapping_add(head, 1, Cap);
}

template <typename T, unsigned int Cap>
bool RingBuffer<T, Cap>::push(const T *data, size_t count, bool overwrite) {
	if (size + count > Cap && !overwrite) {
		return false;
	}
	if (count > Cap) {
		data += Cap - count;
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
		// We've advanced past the head, push it past the tail
		head = wrapping_add(tail, 1, Cap);
	}
	size += count;
	if (size > Cap) {
		size = Cap;
		return false;
	} else {
		return true;
	}
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
}
