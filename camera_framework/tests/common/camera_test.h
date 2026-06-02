#ifndef __CAMERA_TEST_H
#define __CAMERA_TEST_H

#include <rtthread.h>

typedef struct
{
    const char *name;
    void (*run)(void);
} camera_test_case_t;

void camera_test_fail(const char *file,
                      int line,
                      const char *expr,
                      long actual,
                      long expected);
void camera_test_finish_case(void);
rt_bool_t camera_test_case_failed(void);
int camera_test_run_suite(const char *suite_name,
                          const camera_test_case_t *cases,
                          rt_size_t num_cases);

#define CAMERA_TEST_ASSERT_TRUE(expr) \
    do \
    { \
        if (!(expr)) \
        { \
            camera_test_fail(__FILE__, __LINE__, #expr, 0, 1); \
            return; \
        } \
    } while (0)

#define CAMERA_TEST_ASSERT_EQ(actual, expected) \
    do \
    { \
        long _actual = (long)(actual); \
        long _expected = (long)(expected); \
        if (_actual != _expected) \
        { \
            camera_test_fail(__FILE__, __LINE__, #actual " == " #expected, _actual, _expected); \
            return; \
        } \
    } while (0)

#define CAMERA_TEST_ASSERT_NOT_NULL(ptr) \
    CAMERA_TEST_ASSERT_TRUE((ptr) != RT_NULL)

#endif /* __CAMERA_TEST_H */
