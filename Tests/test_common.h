/**
 * @file    test_common.h
 * @brief   Minimal host-side unit test helpers (no external framework).
 *
 * Usage:
 *   static void test_foo(void) { CHECK(x == 1); CHECK_NEAR(v, 1.0f, 1e-3f); }
 *   int main(void) { RUN_TEST(test_foo); return TEST_REPORT(); }
 */
#ifndef TEST_COMMON_H
#define TEST_COMMON_H

#include <math.h>
#include <stdio.h>

static int g_test_failures = 0;
static int g_test_checks   = 0;
static int g_test_count    = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        g_test_checks++;                                                       \
        if (!(cond)) {                                                         \
            g_test_failures++;                                                 \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);           \
        }                                                                      \
    } while (0)

#define CHECK_NEAR(actual, expected, tol)                                      \
    do {                                                                       \
        double a_ = (double)(actual), e_ = (double)(expected);                 \
        g_test_checks++;                                                       \
        if (fabs(a_ - e_) > (double)(tol)) {                                   \
            g_test_failures++;                                                 \
            printf("  FAIL %s:%d: %s = %f, expected %f (tol %g)\n",            \
                   __FILE__, __LINE__, #actual, a_, e_, (double)(tol));        \
        }                                                                      \
    } while (0)

#define RUN_TEST(fn)                                                           \
    do {                                                                       \
        int before_ = g_test_failures;                                         \
        g_test_count++;                                                        \
        fn();                                                                  \
        printf("%s %s\n", (g_test_failures == before_) ? "[ OK ]" : "[FAIL]", #fn); \
    } while (0)

#define TEST_REPORT()                                                          \
    (printf("\n%d tests, %d checks, %d failures\n",                            \
            g_test_count, g_test_checks, g_test_failures),                     \
     g_test_failures == 0 ? 0 : 1)

#endif /* TEST_COMMON_H */
