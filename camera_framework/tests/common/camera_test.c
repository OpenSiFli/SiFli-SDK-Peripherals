#include "camera_test.h"

static rt_bool_t s_camera_test_case_failed = RT_FALSE;

void camera_test_fail(const char *file,
                      int line,
                      const char *expr,
                      long actual,
                      long expected)
{
    s_camera_test_case_failed = RT_TRUE;
    rt_kprintf("[TEST][FAIL] %s:%d %s (actual=%ld expected=%ld)\n",
               file,
               line,
               expr,
               actual,
               expected);
}

void camera_test_finish_case(void)
{
    s_camera_test_case_failed = RT_FALSE;
}

rt_bool_t camera_test_case_failed(void)
{
    return s_camera_test_case_failed;
}

int camera_test_run_suite(const char *suite_name,
                          const camera_test_case_t *cases,
                          rt_size_t num_cases)
{
    rt_size_t i;
    int failed = 0;

    rt_kprintf("[TEST] suite=%s cases=%u\n",
               suite_name,
               (unsigned int)num_cases);

    for (i = 0; i < num_cases; i++)
    {
        camera_test_finish_case();
        rt_kprintf("[TEST] RUN  %s\n", cases[i].name);
        cases[i].run();
        if (camera_test_case_failed())
        {
            failed++;
            rt_kprintf("[TEST] FAIL %s\n", cases[i].name);
        }
        else
        {
            rt_kprintf("[TEST] PASS %s\n", cases[i].name);
        }
    }

    rt_kprintf("[TEST] summary suite=%s passed=%d failed=%d\n",
               suite_name,
               (int)(num_cases - failed),
               failed);

    return failed == 0 ? 0 : -RT_ERROR;
}
