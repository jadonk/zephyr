#include <device.h>
#include <drivers/gpio.h>

#include <greybus/greybus.h>
#include <greybus-utils/platform.h>

const unsigned green_led_pin = DT_ALIAS_LED0_GPIOS_PIN;
static bool green_led_direction = 1; // make it an input by default

static uint8_t gb_gpio_cc13x2r_line_count(void) {
	return 1;
}
static int gb_gpio_cc13x2r_activate(uint8_t which) {
	return 0;
}
static int gb_gpio_cc13x2r_deactivate(uint8_t which) {
	return 0;
}
static int gb_gpio_cc13x2r_get_direction(uint8_t which) {

	if (0 != which) {
		return -ENODEV;
	}

	return green_led_direction;
}
static int gb_gpio_cc13x2r_direction_in(uint8_t which) {

	if (0 != which) {
		return -ENODEV;
	}

	struct device *green_led_dev;
	green_led_dev = device_get_binding(DT_ALIAS_LED0_GPIOS_CONTROLLER);
	gpio_pin_configure(green_led_dev, green_led_pin, GPIO_DIR_IN);

	return 0;
}
static int gb_gpio_cc13x2r_direction_out(uint8_t which, uint8_t val) {

	if (0 != which) {
		return -ENODEV;
	}

	struct device *green_led_dev;
	green_led_dev = device_get_binding(DT_ALIAS_LED0_GPIOS_CONTROLLER);
	gpio_pin_configure(green_led_dev, green_led_pin, GPIO_DIR_OUT);
	gpio_pin_write(green_led_dev, green_led_pin, val);

	return 0;
}
static int gb_gpio_cc13x2r_get_value(uint8_t which) {

	if (0 != which) {
		return -ENODEV;
	}

	struct device *green_led_dev;
	green_led_dev = device_get_binding(DT_ALIAS_LED0_GPIOS_CONTROLLER);
	u32_t value;
	gpio_pin_read(green_led_dev, green_led_pin, &value);
	return value;
}
static int gb_gpio_cc13x2r_set_value(uint8_t which, uint8_t val) {

	if (0 != which) {
		return -ENODEV;
	}

	struct device *green_led_dev;
	green_led_dev = device_get_binding(DT_ALIAS_LED0_GPIOS_CONTROLLER);
	gpio_pin_write(green_led_dev, green_led_pin, val);

	return 0;
}
static int gb_gpio_cc13x2r_set_debounce(uint8_t which, uint16_t usec) {
	return -ENOSYS;
}
static int gb_gpio_cc13x2r_irq_mask(uint8_t which) {
	return -ENOSYS;
}
static int gb_gpio_cc13x2r_irq_unmask(uint8_t which) {
	return -ENOSYS;
}
static int gb_gpio_cc13x2r_irq_type(uint8_t which, uint8_t type) {
	return -ENOSYS;
}

const struct gb_gpio_platform_driver gb_gpio_cc13x2r = {
	.line_count = gb_gpio_cc13x2r_line_count,
	.activate = gb_gpio_cc13x2r_activate,
	.deactivate = gb_gpio_cc13x2r_deactivate,
	.get_direction = gb_gpio_cc13x2r_get_direction,
	.direction_in = gb_gpio_cc13x2r_direction_in,
	.direction_out = gb_gpio_cc13x2r_direction_out,
	.get_value = gb_gpio_cc13x2r_get_value,
	.set_value = gb_gpio_cc13x2r_set_value,
	.set_debounce = gb_gpio_cc13x2r_set_debounce,
	.irq_mask = gb_gpio_cc13x2r_irq_mask,
	.irq_unmask = gb_gpio_cc13x2r_irq_unmask,
	.irq_type = gb_gpio_cc13x2r_irq_type,
};
