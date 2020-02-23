#include <unity.h>
#include <cstring>
#include <cstdio>
#include "util.hpp"

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

	TEST_ASSERT_EQUAL_HEX8(0b00010011, test.checksum);
	// Make sure it doesn't change with a different value in the checksum field
	TEST_ASSERT_EQUAL_HEX8(test.checksum, struct_checksum(test));
}

int main() {
	UNITY_BEGIN();

	RUN_TEST(test_ring_buffer);
	RUN_TEST(test_checksum);

	UNITY_END();
	return 0;
}