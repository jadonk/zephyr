/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_sim_gpio

#include <device.h>
#include <drivers/gpio.h>
#include <drivers/gpio/gpio_sim.h>
#include <errno.h>
#include <zephyr.h>

#include "gpio_utils.h"

#define LOG_LEVEL CONFIG_GPIO_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(gpio_sim);

#define GPIO_SIM_INT_BITMASK							\
	(GPIO_INT_DISABLE | GPIO_INT_DEBOUNCE | GPIO_INT_ENABLE |		\
	 GPIO_INT_LEVELS_LOGICAL | GPIO_INT_EDGE | GPIO_INT_LOW_0 |		\
	 GPIO_INT_HIGH_1)

/*
 * GPIO config / data
 */

/**
 * @brief Simulated GPIO controller configuration data
 *
 * This structure contains all of the state for a given simulated GPIO
 * controller as well as all of the pins associated with it.
 *
 * The @a flags member is a pointer to an array which is @a num_pins in size.
 *
 * @a num_pins must be in the range [1, @ref GPIO_MAX_PINS_PER_PORT].
 *
 * Pin direction as well as other pin properties are set using
 * specific bits in @a flags. For more details, see @ref gpio_interface.
 *
 * Changes are synchronized using @ref gpio_sim_data.mu.
 */
struct gpio_sim_config {
	/** Common @ref gpio_driver_config */
	struct gpio_driver_config common;
	/** Pointer to an array of flags is @a num_pins in size */
	uint32_t *const flags;
	/** Number of pins available in the given GPIO controller instance */
	const gpio_pin_t num_pins;
	/** Input or output values for each pin */
	gpio_port_value_t vals;
	/** Interrupt status for each pin */
	gpio_port_pins_t interrupts;
};

/**
 * @brief Simulated GPIO controller data
 *
 * This structure contains data structures used by a simulated GPIO
 * controller.
 *
 * If the application wishes to specify a "wiring" for the simulated
 * GPIO, then a @ref gpio_callback_handler_t should be registered using
 * @ref gpio_add_callback.
 *
 * Changes are to @ref gpio_sim_data and @ref gpio_sim_config are
 * synchronized using @a mu.
 */
struct gpio_sim_data {
	/** Common @ref gpio_driver_data */
	struct gpio_driver_data common;
	/** Pointer to the @erf gpio_sim_config for this controller */
	struct gpio_sim_config *const config;
	/** Mutex to synchronize accesses to driver data and config */
	struct k_mutex mu;
	/** Singly-linked list of callbacks associated with the controller */
	sys_slist_t callbacks;
};

/**
 * @brief Obtain a mask of pins that match the provided @p flags
 *
 * Use this function to see which pins match the current GPIO configuration.
 *
 * The caller must ensure that @ref gpio_sim_data.mu is locked.
 *
 * @param config The simulated GPIO configuration
 * @param flags The flags to match
 */
static gpio_port_pins_t
gpio_sim_get_mask_of_pins_with_flags(struct gpio_sim_config *config,
				     uint32_t flags)
{
	size_t i;
	gpio_port_pins_t mask;

	for (mask = 0, i = 0; i < config->num_pins; ++i) {
		if (config->flags[i] & flags) {
			mask |= BIT(i);
		}
	}

	return mask;
}

/**
 * @brief Obtain a mask of pins that are configured as @ref GPIO_INPUT
 *
 * The caller must ensure that @ref gpio_sim_data.mu is locked.
 *
 * @param config The simulated GPIO configuration
 */
static gpio_port_pins_t gpio_sim_input_mask(struct gpio_sim_config *config)
{
	return gpio_sim_get_mask_of_pins_with_flags(config, GPIO_INPUT);
}

/**
 * @brief Obtain a mask of pins that are configured as @ref GPIO_OUTPUT
 *
 * The caller must ensure that @ref gpio_sim_data.mu is locked.
 *
 * @param config The simulated GPIO configuration
 */
static gpio_port_pins_t gpio_sim_output_mask(struct gpio_sim_config *config)
{
	return gpio_sim_get_mask_of_pins_with_flags(config, GPIO_OUTPUT);
}

/*
 * GPIO backend API (for setting input pin values)
 */

/**
 * @brief Trigger possible interrupt events after an input pin has changed
 *
 * For more information, see @ref gpio_interface.
 *
 * The caller must ensure that @ref gpio_sim_data.mu is locked.
 *
 * @param port The simulated GPIO port
 * @param mask The mask of pins that have changed
 * @param prev_values Previous pin values
 * @param values Current pin values
 */
static void gpio_sim_pend_interrupt(struct device *port, gpio_port_pins_t mask,
				    gpio_port_value_t prev_values,
				    gpio_port_value_t values)
{
	size_t i;
	bool prev_bit;
	bool bit;
	struct gpio_sim_data *drv_data =
		(struct gpio_sim_data *)port->driver_data;
	struct gpio_sim_config *config = drv_data->config;
	gpio_port_pins_t interrupts = 0;

	for (i = 0; mask && i < config->num_pins;
	     ++i, mask >>= 1, prev_values >>= 1, values >>= 1) {
		if (!(mask & 1)) {
			continue;
		}

		prev_bit = prev_values & 1;
		bit = values & 1;

		switch (config->flags[i] & GPIO_SIM_INT_BITMASK) {
		case GPIO_INT_EDGE_RISING:
			if (!prev_bit && bit) {
				interrupts |= BIT(i);
			}
			break;
		case GPIO_INT_EDGE_FALLING:
			if (prev_bit && !bit) {
				interrupts |= BIT(i);
			}
			break;
		case GPIO_INT_EDGE_BOTH:
			if (prev_bit != bit) {
				interrupts |= BIT(i);
			}
			break;
		case GPIO_INT_LEVEL_LOW:
			if (!bit) {
				interrupts |= BIT(i);
			}
			break;
		case GPIO_INT_LEVEL_HIGH:
			if (bit) {
				interrupts |= BIT(i);
			}
			break;
		case 0:
		case GPIO_INT_DISABLE:
			break;
		default:
			LOG_DBG("unhandled case %u",
				config->flags[i] & GPIO_SIM_INT_BITMASK);
			break;
		}
	}

	config->interrupts |= interrupts;

	if (interrupts) {
		gpio_fire_callbacks(&drv_data->callbacks, port, interrupts);
	}
}

/* documented in drivers/gpio/gpio_sim.h */
int gpio_sim_input_set_masked(struct device *port, gpio_port_pins_t mask,
			      gpio_port_value_t values)
{
	int ret;
	struct gpio_sim_data *drv_data =
		(struct gpio_sim_data *)port->driver_data;
	struct gpio_sim_config *config = drv_data->config;
	gpio_port_pins_t input_mask;
	gpio_port_pins_t prev_values;

	if (mask == 0) {
		ret = 0;
		goto out;
	}

	k_mutex_lock(&drv_data->mu, K_FOREVER);

	if (~config->common.port_pin_mask & mask) {
		ret = -EINVAL;
		goto unlock;
	}

	if (values & ~mask) {
		ret = -EINVAL;
		goto unlock;
	}

	input_mask = gpio_sim_input_mask(config);
	if (~input_mask & mask) {
		ret = -EINVAL;
		goto unlock;
	}

	prev_values = config->vals;
	config->vals &= ~mask;
	config->vals |= values;
	values = config->vals;
	gpio_sim_pend_interrupt(port, mask, prev_values, values);
	ret = 0;

unlock:
	k_mutex_unlock(&drv_data->mu);

out:
	return ret;
}

/* documented in drivers/gpio/gpio_sim.h */
int gpio_sim_output_get_masked(struct device *port, gpio_port_pins_t mask,
			       gpio_port_value_t *values)
{
	int ret;
	struct gpio_sim_data *drv_data =
		(struct gpio_sim_data *)port->driver_data;
	struct gpio_sim_config *config = drv_data->config;
	gpio_port_pins_t output_mask;

	if (mask == 0) {
		ret = 0;
		goto out;
	}

	k_mutex_lock(&drv_data->mu, K_FOREVER);

	if (~config->common.port_pin_mask & mask) {
		ret = -EINVAL;
		goto unlock;
	}

	output_mask = gpio_sim_output_mask(config);
	if (~output_mask & mask) {
		ret = -EINVAL;
		goto unlock;
	}

	*values = config->vals & mask;
	ret = 0;

unlock:
	k_mutex_unlock(&drv_data->mu);

out:
	return ret;
}

/* documented in drivers/gpio/gpio_sim.h */
int gpio_sim_flags_get(struct device *port, gpio_pin_t pin, uint32_t *flags)
{
	int ret;
	struct gpio_sim_data *drv_data =
		(struct gpio_sim_data *)port->driver_data;
	struct gpio_sim_config *config = drv_data->config;

	if (flags == NULL) {
		ret = -EINVAL;
		goto out;
	}

	k_mutex_lock(&drv_data->mu, K_FOREVER);

	if (pin >= config->num_pins) {
		ret = -EINVAL;
		goto unlock;
	}

	*flags = config->flags[pin];
	ret = 0;

unlock:
	k_mutex_unlock(&drv_data->mu);

out:
	return ret;
}

/*
 * GPIO Driver API
 *
 * API is documented at drivers/gpio.h
 */

static int gpio_sim_pin_configure(struct device *port, gpio_pin_t pin,
				  gpio_flags_t flags)
{
	int ret;
	struct gpio_sim_data *drv_data =
		(struct gpio_sim_data *)port->driver_data;
	struct gpio_sim_config *config = drv_data->config;

	if ((config->common.port_pin_mask & BIT(pin)) == 0) {
		ret = -EINVAL;
		goto out;
	}

	k_mutex_lock(&drv_data->mu, K_FOREVER);

	config->flags[pin] = flags;

	if (flags & GPIO_OUTPUT) {
		if (flags & GPIO_OUTPUT_INIT_LOW) {
			config->vals &= ~BIT(pin);
		} else {
			config->vals |= BIT(pin);
		}
	}

	ret = 0;

	k_mutex_unlock(&drv_data->mu);

	gpio_fire_callbacks(&drv_data->callbacks, port, BIT(pin));

out:
	return ret;
}

static int gpio_sim_port_get_raw(struct device *port, gpio_port_value_t *values)
{
	gpio_port_pins_t input_mask;
	struct gpio_sim_data *drv_data =
		(struct gpio_sim_data *)port->driver_data;
	struct gpio_sim_config *config = drv_data->config;

	if (values == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&drv_data->mu, K_FOREVER);

	input_mask = gpio_sim_input_mask(config);
	*values = config->vals & input_mask;

	k_mutex_unlock(&drv_data->mu);

	return 0;
}

static int gpio_sim_port_set_masked_raw(struct device *port,
					gpio_port_pins_t mask,
					gpio_port_value_t values)
{
	gpio_port_pins_t output_mask;
	gpio_port_pins_t prev_values;
	struct gpio_sim_data *drv_data =
		(struct gpio_sim_data *)port->driver_data;
	struct gpio_sim_config *config = drv_data->config;

	k_mutex_lock(&drv_data->mu, K_FOREVER);

	prev_values = config->vals;

	output_mask = gpio_sim_output_mask(config);
	/* silently correct mask (otherwise CI fails) */
	mask &= output_mask;
	/* silently correct values (otherwise CI fails) */
	prev_values &= output_mask;
	values &= output_mask;

	config->vals &= ~mask;
	config->vals |= values;

	k_mutex_unlock(&drv_data->mu);

	if (prev_values ^ values) {
		gpio_fire_callbacks(&drv_data->callbacks, port, mask);
	}

	return 0;
}

static int gpio_sim_port_set_bits_raw(struct device *port,
				      gpio_port_pins_t pins)
{
	int ret;
	gpio_port_pins_t output_mask;
	struct gpio_sim_data *drv_data =
		(struct gpio_sim_data *)port->driver_data;
	struct gpio_sim_config *config = drv_data->config;

	k_mutex_lock(&drv_data->mu, K_FOREVER);

	output_mask = gpio_sim_output_mask(config);
	if (pins & ~output_mask) {
		LOG_ERR("attempt to set pins not configured for output: %08x",
			pins & ~output_mask);
		ret = -EINVAL;
		goto unlock;
	}

	config->vals |= pins;
	ret = 0;

unlock:
	k_mutex_unlock(&drv_data->mu);

	if (ret == 0) {
		gpio_fire_callbacks(&drv_data->callbacks, port, pins);
	}

	return ret;
}

static int gpio_sim_port_clear_bits_raw(struct device *port,
					gpio_port_pins_t pins)
{
	int ret;
	gpio_port_pins_t output_mask;
	struct gpio_sim_data *drv_data =
		(struct gpio_sim_data *)port->driver_data;
	struct gpio_sim_config *config = drv_data->config;

	k_mutex_lock(&drv_data->mu, K_FOREVER);

	output_mask = gpio_sim_output_mask(config);
	if (pins & ~output_mask) {
		LOG_ERR("attempt to clear pins not configured for output: %08x",
			pins & ~output_mask);
		ret = -EINVAL;
		goto unlock;
	}

	config->vals &= ~pins;
	ret = 0;

unlock:
	k_mutex_unlock(&drv_data->mu);

	if (ret == 0) {
		gpio_fire_callbacks(&drv_data->callbacks, port, pins);
	}

	return ret;
}

static int gpio_sim_port_toggle_bits(struct device *port, gpio_port_pins_t pins)
{
	int ret;
	gpio_port_pins_t output_mask;
	struct gpio_sim_data *drv_data =
		(struct gpio_sim_data *)port->driver_data;
	struct gpio_sim_config *config = drv_data->config;

	k_mutex_lock(&drv_data->mu, K_FOREVER);

	output_mask = gpio_sim_output_mask(config);
	if (pins & ~output_mask) {
		LOG_ERR("attempt to toggle pins not configured for output: %08x",
			pins & ~output_mask);
		ret = -EINVAL;
		goto unlock;
	}

	config->vals ^= pins;
	ret = 0;

unlock:
	k_mutex_unlock(&drv_data->mu);

	if (ret == 0) {
		gpio_fire_callbacks(&drv_data->callbacks, port, pins);
	}

	return ret;
}

static int gpio_sim_pin_interrupt_configure(struct device *port, gpio_pin_t pin,
					    enum gpio_int_mode mode,
					    enum gpio_int_trig trig)
{
	int ret;
	struct gpio_sim_data *drv_data =
		(struct gpio_sim_data *)port->driver_data;
	struct gpio_sim_config *config = drv_data->config;

	k_mutex_lock(&drv_data->mu, K_FOREVER);

	if (pin >= config->num_pins) {
		ret = -EINVAL;
		goto unlock;
	}

	if (mode != GPIO_INT_MODE_DISABLED) {
		switch (trig) {
		case GPIO_INT_TRIG_LOW:
		case GPIO_INT_TRIG_HIGH:
		case GPIO_INT_TRIG_BOTH:
			break;
		default:
			ret = -EINVAL;
			goto unlock;
		}
	}

	switch (mode) {
	case GPIO_INT_MODE_DISABLED:
		config->flags[pin] &= ~GPIO_SIM_INT_BITMASK;
		config->flags[pin] |= GPIO_INT_DISABLE;
		break;

	case GPIO_INT_MODE_LEVEL:
	case GPIO_INT_MODE_EDGE:
		config->flags[pin] &= ~GPIO_SIM_INT_BITMASK;
		config->flags[pin] |= (mode | trig);
		break;

	default:
		ret = -EINVAL;
		goto unlock;
	}

	ret = 0;

unlock:
	k_mutex_unlock(&drv_data->mu);

	return ret;
}

static int gpio_sim_manage_callback(struct device *port,
				    struct gpio_callback *cb, bool set)
{
	struct gpio_sim_data *drv_data =
		(struct gpio_sim_data *)port->driver_data;

	return gpio_manage_callback(&drv_data->callbacks, cb, set);
}

static uint32_t gpio_sim_get_pending_int(struct device *dev)
{
	struct gpio_sim_data *drv_data =
		(struct gpio_sim_data *)dev->driver_data;
	struct gpio_sim_config *config = drv_data->config;

	return config->interrupts;
}

static const struct gpio_driver_api gpio_sim_driver = {
	.pin_configure = gpio_sim_pin_configure,
	.port_get_raw = gpio_sim_port_get_raw,
	.port_set_masked_raw = gpio_sim_port_set_masked_raw,
	.port_set_bits_raw = gpio_sim_port_set_bits_raw,
	.port_clear_bits_raw = gpio_sim_port_clear_bits_raw,
	.port_toggle_bits = gpio_sim_port_toggle_bits,
	.pin_interrupt_configure = gpio_sim_pin_interrupt_configure,
	.manage_callback = gpio_sim_manage_callback,
	.get_pending_int = gpio_sim_get_pending_int,
};

static int gpio_sim_init(struct device *dev)
{
	struct gpio_sim_data *drv_data =
		(struct gpio_sim_data *)dev->driver_data;

	sys_slist_init(&drv_data->callbacks);

	return k_mutex_init(&drv_data->mu);
}

/*
 * Device Initialization
 */

#define DEFINE_GPIO_SIM(_num)							\
										\
	static uint32_t gpio_sim_flags_##_num[DT_INST_PROP(_num, ngpios)];	\
										\
	static struct gpio_sim_config gpio_sim_config_##_num = {		\
		.common = {							\
			.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(_num),	\
		},								\
		.flags = gpio_sim_flags_##_num,					\
		.num_pins = DT_INST_PROP(_num, ngpios),				\
	};									\
										\
	static struct gpio_sim_data gpio_sim_data_##_num = {			\
		.config = &gpio_sim_config_##_num,				\
	};									\
										\
	DEVICE_AND_API_INIT(gpio_sim_##_num, DT_INST_LABEL(_num),		\
			    gpio_sim_init, &gpio_sim_data_##_num,		\
			    &gpio_sim_config_##_num, POST_KERNEL,		\
			    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,			\
			    &gpio_sim_driver)

DT_INST_FOREACH_STATUS_OKAY(DEFINE_GPIO_SIM);
