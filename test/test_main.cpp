#include <unity.h>
#include <cstring>
#include <cstdio>
#include "../src/util.hpp"

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
	TEST_ASSERT(memcmp(temp, "cdgh", 4) == 0);
	TEST_ASSERT_EQUAL(buf.used(), 0);
	TEST_ASSERT_EQUAL(buf.available(), 4);

	// Test push after poping empty
	TEST_ASSERT(buf.push("ij", 2, false));
	TEST_ASSERT(!buf.pop(temp, 3));
	TEST_ASSERT(buf.pop(temp, 2));
	TEST_ASSERT(memcmp(temp, "ij", 2) == 0);

	// Test push with array larger than size
	TEST_ASSERT(buf.push("klmno", 5, true));
	TEST_ASSERT(!buf.pop(temp, 5));
	TEST_ASSERT(buf.pop(temp, 4));
	TEST_ASSERT(memcmp(temp, "lmno", 4) == 0);

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
	TEST_ASSERT(memcmp(temp, "pqrs", 4) == 0);
}

int main() {
	UNITY_BEGIN();

	RUN_TEST(test_ring_buffer);

	UNITY_END();
	return 0;
}