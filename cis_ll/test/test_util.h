/*
 * Minimal assertions for the host tests: they count errors and carry on, so one run shows all the mismatches.
 */
#pragma once

#include <stdio.h>
#include <string.h>

static int g_checks;
static int g_fails;

#define CHECK(c) do { \
	g_checks++; \
	if (!(c)) { \
		g_fails++; \
		fprintf(stderr, "%s:%d: CHECK(%s)\n", __FILE__, __LINE__, #c); \
	} \
} while (0)

#define CHECK_EQ(a, b) do { \
	unsigned long long _a = (unsigned long long)(a), _b = (unsigned long long)(b); \
	g_checks++; \
	if (_a != _b) { \
		g_fails++; \
		fprintf(stderr, "%s:%d: %s == %s (0x%llx != 0x%llx)\n", __FILE__, __LINE__, #a, #b, _a, _b); \
	} \
} while (0)

static inline void test_dump(const char *what, const unsigned char *p, unsigned n)
{
	fprintf(stderr, "  %s:", what);
	for (unsigned i = 0; i < n; i++)
		fprintf(stderr, " %02X", p[i]);
	fprintf(stderr, "\n");
}

#define CHECK_MEM(a, b, n) do { \
	g_checks++; \
	if (memcmp((a), (b), (n)) != 0) { \
		g_fails++; \
		fprintf(stderr, "%s:%d: memcmp(%s, %s, %u)\n", __FILE__, __LINE__, #a, #b, (unsigned)(n)); \
		test_dump("got ", (const unsigned char *)(a), (unsigned)(n)); \
		test_dump("want", (const unsigned char *)(b), (unsigned)(n)); \
	} \
} while (0)

#define TEST_DONE(name) do { \
	printf("%s: %d checks, %d failures\n", (name), g_checks, g_fails); \
	return g_fails ? 1 : 0; \
} while (0)
