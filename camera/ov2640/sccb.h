/**
 * @file    sccb.h
 * @brief   SCCB (Serial Camera Control Bus) communication interface
 *
 * This header defines the SCCB initialization, read/write API
 * and configurable parameters for I2C-based sensor register access.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026
 */

#ifndef SCCB_H_
#define SCCB_H_

#include <stdint.h>
#include <rtthread.h>
#include "drivers/i2c.h"
#include "bf0_hal.h"

#ifdef PKG_USING_OV2640
#include <rtconfig.h>
#endif

#ifndef OV2640_SCCB_TIMEOUT_MS
#define SCCB_TIMEOUT_MS 1000
#else
#define SCCB_TIMEOUT_MS OV2640_SCCB_TIMEOUT_MS
#endif

#ifndef OV2640_SCCB_MAX_HZ
#define SCCB_MAX_HZ 100000
#else
#define SCCB_MAX_HZ OV2640_SCCB_MAX_HZ
#endif

#ifndef OV2640_SCCB_I2C_BUS_NAME
#define SCCB_USE_IIC "i2c1"
#else
#define SCCB_USE_IIC OV2640_SCCB_I2C_BUS_NAME
#endif

rt_err_t sccb_init(const char *i2c_bus_name);
void sccb_deinit(void);
int sccb_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
uint8_t sccb_read(uint8_t dev_addr, uint8_t reg_addr);

#endif // SCCB_H_