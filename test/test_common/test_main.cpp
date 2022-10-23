#include <unity.h>
#include <cstring>
#include <cstdio>

#include "kalman.hpp"
#include "util.hpp"

#define TEST_ASSERT_FLOAT_WITHIN_REL(factor, expected, actual) TEST_ASSERT_FLOAT_WITHIN((factor) * (expected), expected, actual)

void test_ring_buffer() {
	RingBuffer<char, 4> buf;
	char temp[4];

	// Check initial state
	TEST_ASSERT_EQUAL(buf.used(), 0);
	TEST_ASSERT_EQUAL(buf.available(), 4);

	// Check single push
	TEST_ASSERT(buf.push('a', false));
	TEST_ASSERT_EQUAL(buf.used(), 1);
	TEST_ASSERT_EQUAL(buf.available(), 3);

	// Test multiple push
	TEST_ASSERT(buf.push("bc", 2, false));
	TEST_ASSERT_EQUAL(buf.used(), 3);
	TEST_ASSERT_EQUAL(buf.available(), 1);

	// Test full
	TEST_ASSERT(buf.push('d', false));
	TEST_ASSERT_EQUAL(buf.used(), 4);
	TEST_ASSERT_EQUAL(buf.available(), 0);

	// Test overflow
	TEST_ASSERT(!buf.push("ef", 2, false));
	TEST_ASSERT(!buf.push("gh", 2, true));
	TEST_ASSERT_EQUAL(buf.used(), 4);
	TEST_ASSERT_EQUAL(buf.available(), 0);

	// Test pop results after overflow
	TEST_ASSERT(!buf.pop(temp, 5));
	TEST_ASSERT(buf.pop(temp, 4));
	TEST_ASSERT_EQUAL(0, memcmp(temp, "cdgh", 4));
	TEST_ASSERT_EQUAL(buf.used(), 0);
	TEST_ASSERT_EQUAL(buf.available(), 4);

	// Test push after poping empty
	TEST_ASSERT(buf.push("ij", 2, false));
	TEST_ASSERT(!buf.pop(temp, 3));
	TEST_ASSERT(buf.pop(temp, 2));
	TEST_ASSERT_EQUAL(0, memcmp(temp, "ij", 2));

	// Test push with array larger than size
	TEST_ASSERT(buf.push("klmno", 5, true));
	TEST_ASSERT(!buf.pop(temp, 5));
	TEST_ASSERT(buf.pop(temp, 4));
	TEST_ASSERT_EQUAL(0, memcmp(temp, "lmno", 4));

	// Test single push and pop to overflow
	TEST_ASSERT(buf.push('p', false));
	TEST_ASSERT(buf.push('q', false));
	TEST_ASSERT(buf.push('r', false));
	TEST_ASSERT(buf.push('s', false));
	TEST_ASSERT(!buf.push('t', false));
	TEST_ASSERT(buf.pop(temp));
	TEST_ASSERT(buf.pop(temp + 1));
	TEST_ASSERT(buf.pop(temp + 2));
	TEST_ASSERT(buf.pop(temp + 3));
	TEST_ASSERT(!buf.pop(temp + 1));
	TEST_ASSERT_EQUAL(0, memcmp(temp, "pqrs", 4));
}

void test_checksum() {
	struct {
		uint32_t val;
		uint8_t checksum;
	} test;
	test.val = 0x42ca52c9;
	test.checksum = struct_checksum(test);

	TEST_ASSERT_EQUAL_HEX8(0x58, test.checksum);
	// Make sure it doesn't change with a different value in the checksum field
	TEST_ASSERT_EQUAL_HEX8(test.checksum, struct_checksum(test));
}

void test_kalman() {
	float eps = 1e-5;
	KalmanFilter kf(0.1f, 1.0f, 1.0f, 0.2f);

	kf.step(1, 2);
	TEST_ASSERT_FLOAT_WITHIN_REL(eps, 2.275105, (float)kf.pos());
	TEST_ASSERT_FLOAT_WITHIN_REL(eps, 0.250143, (float)kf.rate());
	TEST_ASSERT_FLOAT_WITHIN_REL(eps, 0.224111, (float)kf.accel());

	kf.step(2, 1);
	TEST_ASSERT_FLOAT_WITHIN_REL(eps, 2.213849, (float)kf.pos());
	TEST_ASSERT_FLOAT_WITHIN_REL(eps, 0.317454, (float)kf.rate());
	TEST_ASSERT_FLOAT_WITHIN_REL(eps, 0.533552, (float)kf.accel());

	kf.step(16, 10000);
	TEST_ASSERT_FLOAT_WITHIN_REL(eps, 1277.477295, (float)kf.pos());
	TEST_ASSERT_FLOAT_WITHIN_REL(eps, 912.007812, (float)kf.rate());
	TEST_ASSERT_FLOAT_WITHIN_REL(eps, 280.687927, (float)kf.accel());

	kf.step(0, 0);
	TEST_ASSERT_FLOAT_WITHIN_REL(eps, 1369.784180, (float)kf.pos());
	TEST_ASSERT_FLOAT_WITHIN_REL(eps, 939.835571, (float)kf.rate());
	TEST_ASSERT_FLOAT_WITHIN_REL(eps, 280.535583, (float)kf.accel());

	for (int i = 0; i < 32; ++i) {
		kf.step(kfloat_t(i), kfloat_t(i));
	}
	kf.step(0, 0);

	TEST_ASSERT_FLOAT_WITHIN_REL(eps, 78.035400, (float)kf.pos());
	TEST_ASSERT_FLOAT_WITHIN_REL(eps, 36.407646, (float)kf.rate());
	TEST_ASSERT_FLOAT_WITHIN_REL(eps, 8.332301, (float)kf.accel());
}

int main() {
	UNITY_BEGIN();

	RUN_TEST(test_ring_buffer);
	RUN_TEST(test_checksum);
	RUN_TEST(test_kalman);

	UNITY_END();
	return 0;
}