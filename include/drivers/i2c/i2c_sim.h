/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Backend API for simulated I2C
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_I2C_I2C_SIM_H_
#define ZEPHYR_INCLUDE_DRIVERS_I2C_I2C_SIM_H_

#include <zephyr/types.h>
#include <drivers/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Simulated I2C backend API
 * @defgroup i2c_sim Simulated I2C
 * @ingroup i2c_interface
 * @{
 *
 * Behaviour of virtual I2C is application-defined. As-such, each
 * application may
 *
 * - define a Device Tree overlay file to indicate the number of I2C
 *   controllers as well as the number of pins for each controller
 * - register a callback with the I2C controller using
 *   @ref i2c_add_callback to simulate "wiring"
 * - asynchronously call @ref i2c_sim_input_set and / or
 *   @ref i2c_sim_input_set_masked in order to simulate I2C events
 *
 * An example of an appropriate Device Tree overlay file is in
 * tests/drivers/i2c/i2c_api/boards/native_posix_64.overlay.
 *
 * An example of registering a callback to simulate "wiring" as well as
 * an example of calling @ref i2c_sim_input_set is in the file
 * tests/drivers/i2c/i2c_api/src/main.c .
 */

/**
 * @brief A callback for simulated I2C I/O
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param msgs Array of messages to transfer.
 * @param num_msgs Number of messages to transfer.
 * @param addr Address of the I2C target device.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 *
 * @see @ref i2c_transfer
 */
typedef int (*i2c_sim_callback_t)(struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs, uint16_t addr);

/**
 * @brief Register a callback for simulated I2C I/O 
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param addr The i2c peripheral address for which a callback should be registered
 * @param cb The function to call when traffic is available for @p dev / @p addr
 * @return 0 on success.
 * @return -ENOMEM if insufficient memory is available
 * @return -EALREADY if insufficient memory is available
 */
int i2c_sim_callback_register(struct device *dev, uint16_t addr, i2c_sim_callback_t cb);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_I2C_I2C_SIM_H_ */
