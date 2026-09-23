/* SPDX-License-Identifier: MIT */
/*
 * Strong override of the SDK weak acpu_run_task(). A JPEG decode can exceed
 * the SDK's default one-second synchronous timeout.
 */
#include <rtconfig.h>
#include <board.h>
#include <rtthread.h>

#include <acpu_ctrl.h>
#include <acpu_ctrl_private.h>
#include <bf0_mbox_common.h>
#include <ipc_queue.h>

#define CAMERA_SW_ACPU_TASK_TIMEOUT_MS 5000U

void *acpu_run_task(uint8_t task_name, void *param, uint32_t param_size,
                    uint8_t *error_code)
{
    rt_err_t err;
    char task_name_str[RT_NAME_MAX];
    rt_sem_t sem;
    size_t wr_size;
    size_t msg_size;
    acpu_ctrl_ipc_msg_t msg;
    acpu_ctrl_ipc_msg_t *message = &msg;

    SCB_CleanInvalidateDCache();
#ifdef RT_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
#endif
    rt_snprintf(task_name_str, sizeof(task_name_str), "tsk_%d", task_name);
    sem = rt_sem_create(task_name_str, 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(sem);

    msg.is_rsp = 0;
    msg.task_id = task_name;
    msg.task_param_size = param_size;
    msg.task_param = param;
    msg.sema = sem;
    msg.ret_error_code = 0;
    msg.ret_value = 0;
    msg_size = sizeof(message);
    wr_size = ipc_queue_write(sys_get_ha_ipc_queue(), &message, msg_size,
                              1000);
    RT_ASSERT(wr_size == msg_size);

    err = rt_sem_take(sem, rt_tick_from_millisecond(CAMERA_SW_ACPU_TASK_TIMEOUT_MS));
    RT_ASSERT(err == RT_EOK);
    rt_sem_delete(sem);
    RT_ASSERT(msg.is_rsp == 1);
    RT_ASSERT(task_name == msg.task_id);
    if (msg.ret_error_code == (uint8_t)ACPU_ERR_ASSERT)
    {
        rt_kprintf("acpu assert: %s\n", msg.ret_value);
        RT_ASSERT(0);
    }
    if (error_code != RT_NULL)
        *error_code = msg.ret_error_code;
#ifdef RT_USING_PM
    rt_pm_release(PM_SLEEP_MODE_IDLE);
#endif
    return (void *)msg.ret_value;
}
