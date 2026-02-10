/**
 * @file    sccb.c
 * @brief   SCCB (Serial Camera Control Bus) communication layer
 *
 * This module implements the SCCB protocol over I2C for
 * reading and writing OV2640 sensor registers.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026
 */

#include "sccb.h"

#define DBG_TAG "sccb"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static struct rt_i2c_bus_device *i2c_bus = RT_NULL;

rt_err_t sccb_init(const char *i2c_bus_name)
{
    i2c_bus = rt_i2c_bus_device_find(i2c_bus_name);
    if (i2c_bus == RT_NULL)
    {
        LOG_E("SCCB: I2C bus %s not found!", i2c_bus_name);
        return -RT_ERROR;
    }
    rt_err_t ret = rt_device_open((rt_device_t)i2c_bus, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
    {
        LOG_E("Failed to open I2C bus: %d", ret);
        return ret;
    }
    
    struct rt_i2c_configuration configuration =
    {
        .mode = 0,
        .addr = 0,
        .timeout = SCCB_TIMEOUT_MS, //Waiting for timeout period (ms)
        .max_hz = SCCB_MAX_HZ, //I2C rate (hz)
    };
    // config I2C parameter
    return rt_i2c_configure(i2c_bus, &configuration);
}

void sccb_deinit(void)
{
    if (i2c_bus != RT_NULL)
    {
        rt_device_close((rt_device_t)i2c_bus);
        i2c_bus = RT_NULL;
        LOG_I("SCCB deinitialized");
    }
}

int sccb_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    rt_size_t res = 0;
    uint8_t buf[2];
    buf[0] = reg_addr;
    buf[1] = data;
    res = rt_i2c_master_send(i2c_bus, dev_addr, RT_I2C_WR , buf, 2);
    if(res<2) return -RT_ERROR;
    return RT_EOK;
}

uint8_t sccb_read(uint8_t dev_addr, uint8_t reg_addr)
{
    rt_size_t res = 0;
    uint8_t data = 0;
    res = rt_i2c_master_send(i2c_bus, dev_addr, RT_I2C_WR , &reg_addr, 1);
    if(res<1)
    {
        LOG_W("SCCB: failed to send register 0x%02X to device 0x%02X", reg_addr, dev_addr);
        return 0;
    }
    res = rt_i2c_master_recv(i2c_bus, dev_addr, RT_I2C_RD , &data, 1);
    if(res<1)
    {
        LOG_W("SCCB: failed to read from device 0x%02X register 0x%02X", dev_addr, reg_addr);
        return 0;
    }
    return data;
}