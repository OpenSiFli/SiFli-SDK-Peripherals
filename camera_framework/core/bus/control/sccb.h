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

#include <rtconfig.h>

/* The camera I2C pads are owned by the board: its bsp_pinmux.c muxes them
 * together with the DVP/SPI pads. This layer only opens the bus by name, and
 * the name is the board's own CAMERA_I2C_DEVICE_NAME (the same symbol the SDK
 * boards already declare), never a framework menu option. */
#if !defined(CAMERA_I2C_DEVICE_NAME)
#error "board must name the camera I2C device as CAMERA_I2C_DEVICE_NAME"
#endif

#define SCCB_I2C_BUS_NAME CAMERA_I2C_DEVICE_NAME

typedef struct
{
	const char *bus_name;
	uint32_t timeout_ms;
	uint32_t max_hz;
} sccb_config_t;

/** @brief Init SCCB I2C transport. */
rt_err_t sccb_init(const sccb_config_t *config);

/**
 * @brief Deinitialize SCCB transport and close the I2C device.
 */
void sccb_deinit(void);

/** @brief Write register-address + payload bytes. */
int sccb_write_bytes(uint8_t dev_addr,
					 const uint8_t *reg_addr,
					 rt_size_t reg_addr_len,
					 const uint8_t *data,
					 rt_size_t data_len);

/** @brief Read payload bytes from sensor registers. */
int sccb_read_bytes(uint8_t dev_addr,
					const uint8_t *reg_addr,
					rt_size_t reg_addr_len,
					uint8_t *data,
					rt_size_t data_len);

/**
 * @brief Write one 8-bit sensor register.
 */
int sccb_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);

/** @brief Read one 8-bit register (0 on failure). */
uint8_t sccb_read(uint8_t dev_addr, uint8_t reg_addr);

#endif // SCCB_H_
