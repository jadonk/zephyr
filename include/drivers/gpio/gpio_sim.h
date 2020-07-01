/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Backend API for simulated GPIO
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_SIM_H_
#define ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_SIM_H_

#include <zephyr/types.h>
#include <drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Simulated GPIO backend API
 * @defgroup gpio_sim Simulated GPIO
 * @ingroup gpio_interface
 * @{
 *
 * Behaviour of virtual GPIO is application-defined. As-such, each
 * application may
 *
 * - define a Device Tree overlay file to indicate the number of GPIO
 *   controllers as well as the number of pins for each controller
 * - register a callback with the GPIO controller using
 *   @ref gpio_add_callback to simulate "wiring"
 * - asynchronously call @ref gpio_sim_input_set and / or
 *   @ref gpio_sim_input_set_masked in order to simulate GPIO events
 *
 * An example of an appropriate Device Tree overlay file is in
 * tests/drivers/gpio/gpio_basic_api/boards/native_posix_64.overlay.
 *
 * An example of registering a callback to simulate "wiring" as well as
 * an example of calling @ref gpio_sim_input_set is in the file
 * tests/drivers/gpio/gpio_basic_api/src/main.c .
 */

/**
 * @brief Modify the values of one or more simulated GPIO input @p pins
 *
 * @param port The simulated GPIO port
 * @param pins The mask of pins that have changed
 * @param values New values to assign to @p pins
 *
 * @return 0 on success
 * @return -EINVAL if an invalid argument is provided
 */
int gpio_sim_input_set_masked(struct device *port, gpio_port_pins_t pins,
			      gpio_port_value_t values);

/**
 * @brief Modify the value of one simulated GPIO input @p pin
 *
 * @param port The simulated GPIO port
 * @param pin The pin to modify
 * @param value New values to assign to @p pin
 *
 * @return 0 on success
 * @return -EINVAL if an invalid argument is provided
 */
static inline int gpio_sim_input_set(struct device *port, gpio_pin_t pin,
				     int value)
{
	return gpio_sim_input_set_masked(port, 1 << pin, (!!value) << pin);
}

/**
 * @brief Read the value of one or more simulated GPIO output @p pins
 *
 * @param port The simulated GPIO port
 * @param pins The mask of pins that have changed
 * @param values A pointer to where the value of @p pins will be stored
 *
 * @return 0 on success
 * @return -EINVAL if an invalid argument is provided
 */
int gpio_sim_output_get_masked(struct device *port, gpio_port_pins_t pins,
			       gpio_port_value_t *values);

/**
 * @brief Read the value of one simulated GPIO output @p pin
 *
 * @param port The simulated GPIO port
 * @param pin The pin to read
 *
 * @return 0 or 1 on success
 * @return -EINVAL if an invalid argument is provided
 */
static inline int gpio_sim_output_get(struct device *port, gpio_pin_t pin)
{
	int ret;
	gpio_port_value_t values;

	ret = gpio_sim_output_get_masked(port, 1 << pin, &values);
	if (ret == 0) {
		ret = !!(values >> pin);
	}

	return ret;
}

/**
 * @brief Get @p flags for a given simulated GPIO @p pin
 *
 * For more information on available flags, see @ref gpio_interface.
 *
 * @param port The simulated GPIO port
 * @param pin The pin to retriev @p flags for
 * @param flags a pointer to where the flags for @p pin will be stored
 *
 * @return 0 on success
 * @return -EINVAL if an invalid argument is provided
 */
int gpio_sim_flags_get(struct device *port, gpio_pin_t pin, uint32_t *flags);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_SIM_H_ */
