/**
 * @file    sccb.c
 * @brief   SCCB (Serial Camera Control Bus) communication layer
 *
 * This module implements the SCCB protocol over I2C for
 * reading and writing camera sensor registers.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026
 */

#include "sccb.h"
#include <string.h>

#define DBG_TAG "sccb"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static struct rt_i2c_bus_device *i2c_bus = RT_NULL;

#define SCCB_MAX_REG_ADDR_LEN 4
#define SCCB_MAX_TRANSFER_DATA_LEN 32

/** @brief Configure SCCB pinmux for selected I2C bus. */
static rt_err_t sccb_configure_pins(const char *i2c_bus_name)
{
    if (i2c_bus_name == RT_NULL)
    {
        LOG_E("SCCB: I2C bus name is null");
        return -RT_EINVAL;
    }

    if (strcmp(i2c_bus_name, "i2c1") == 0)
    {
        HAL_PIN_Set(SCCB_SCL_PIN, I2C1_SCL, PIN_PULLUP, 1);
        HAL_PIN_Set(SCCB_SDA_PIN, I2C1_SDA, PIN_PULLUP, 1);
        return RT_EOK;
    }

    if (strcmp(i2c_bus_name, "i2c2") == 0)
    {
        HAL_PIN_Set(SCCB_SCL_PIN, I2C2_SCL, PIN_PULLUP, 1);
        HAL_PIN_Set(SCCB_SDA_PIN, I2C2_SDA, PIN_PULLUP, 1);
        return RT_EOK;
    }

    LOG_E("SCCB: unsupported I2C bus name %s", i2c_bus_name);
    return -RT_EINVAL;
}

/** @brief Init SCCB I2C transport. */
rt_err_t sccb_init(const sccb_config_t *config)
{
    rt_err_t ret;

    if (config == RT_NULL || config->bus_name == RT_NULL)
    {
        LOG_E("SCCB: invalid configuration");
        return -RT_EINVAL;
    }

    ret = sccb_configure_pins(config->bus_name);
    if (ret != RT_EOK)
    {
        return ret;
    }

    i2c_bus = rt_i2c_bus_device_find(config->bus_name);
    if (i2c_bus == RT_NULL)
    {
        LOG_E("SCCB: I2C bus %s not found!", config->bus_name);
        return -RT_ERROR;
    }
    ret = rt_device_open((rt_device_t)i2c_bus, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
    {
        LOG_E("Failed to open I2C bus: %d", ret);
        return ret;
    }
    
    struct rt_i2c_configuration configuration =
    {
        .mode = 0,
        .addr = 0,
        .timeout = config->timeout_ms,
        .max_hz = config->max_hz,
    };

    return rt_i2c_configure(i2c_bus, &configuration);
}

/**
 * @brief Deinitialize SCCB transport and close the I2C device.
 */
void sccb_deinit(void)
{
    if (i2c_bus != RT_NULL)
    {
        rt_device_close((rt_device_t)i2c_bus);
        i2c_bus = RT_NULL;
        LOG_I("SCCB deinitialized");
    }
}

/** @brief Write register-address + payload bytes. */
int sccb_write_bytes(uint8_t dev_addr,
                     const uint8_t *reg_addr,
                     rt_size_t reg_addr_len,
                     const uint8_t *data,
                     rt_size_t data_len)
{
    rt_size_t res = 0;
    uint8_t tx_buf[SCCB_MAX_REG_ADDR_LEN + SCCB_MAX_TRANSFER_DATA_LEN];

    if (i2c_bus == RT_NULL)
    {
        LOG_E("SCCB: I2C bus is not initialized");
        return -RT_ERROR;
    }

    if ((reg_addr == RT_NULL) || (reg_addr_len == 0) ||
        (data == RT_NULL) || (data_len == 0))
    {
        return -RT_EINVAL;
    }

    if ((reg_addr_len > SCCB_MAX_REG_ADDR_LEN) ||
        (data_len > SCCB_MAX_TRANSFER_DATA_LEN))
    {
        LOG_E("SCCB: write size too large (addr=%u, data=%u)",
              (unsigned int)reg_addr_len,
              (unsigned int)data_len);
        return -RT_EINVAL;
    }

    rt_memcpy(tx_buf, reg_addr, reg_addr_len);
    rt_memcpy(tx_buf + reg_addr_len, data, data_len);

    res = rt_i2c_master_send(i2c_bus,
                             dev_addr,
                             RT_I2C_WR,
                             tx_buf,
                             reg_addr_len + data_len);
    if (res < (reg_addr_len + data_len))
    {
        LOG_W("SCCB: write failed to device 0x%02X", dev_addr);
        return -RT_ERROR;
    }

    return RT_EOK;
}

/** @brief Read payload bytes from sensor registers. */
int sccb_read_bytes(uint8_t dev_addr,
                    const uint8_t *reg_addr,
                    rt_size_t reg_addr_len,
                    uint8_t *data,
                    rt_size_t data_len)
{
    rt_size_t res = 0;

    if (i2c_bus == RT_NULL)
    {
        LOG_E("SCCB: I2C bus is not initialized");
        return -RT_ERROR;
    }

    if ((reg_addr == RT_NULL) || (reg_addr_len == 0) ||
        (data == RT_NULL) || (data_len == 0))
    {
        return -RT_EINVAL;
    }

    if (reg_addr_len > SCCB_MAX_REG_ADDR_LEN)
    {
        LOG_E("SCCB: register address too large (%u)", (unsigned int)reg_addr_len);
        return -RT_EINVAL;
    }

    res = rt_i2c_master_send(i2c_bus, dev_addr, RT_I2C_WR, reg_addr, reg_addr_len);
    if (res < reg_addr_len)
    {
        LOG_W("SCCB: failed to send register address to device 0x%02X", dev_addr);
        return -RT_ERROR;
    }

    res = rt_i2c_master_recv(i2c_bus, dev_addr, RT_I2C_RD, data, data_len);
    if (res < data_len)
    {
        LOG_W("SCCB: failed to read %u byte(s) from device 0x%02X",
              (unsigned int)data_len,
              dev_addr);
        return -RT_ERROR;
    }

    return RT_EOK;
}

/**
 * @brief Write one 8-bit sensor register.
 */
int sccb_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    return sccb_write_bytes(dev_addr, &reg_addr, 1, &data, 1);
}

/** @brief Read one 8-bit register (0 on failure). */
uint8_t sccb_read(uint8_t dev_addr, uint8_t reg_addr)
{
    uint8_t data = 0;
    if (sccb_read_bytes(dev_addr, &reg_addr, 1, &data, 1) != RT_EOK)
    {
        return 0;
    }

    return data;
}
