# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(
	control-gpb.c
	greybus-core.c
)

zephyr_library_sources_ifdef(CONFIG_GB_AUDIO          audio.c)
zephyr_library_sources_ifdef(CONFIG_GB_CAMERA         camera.c)
zephyr_library_sources_ifdef(CONFIG_GB_GPIO           gpio.c)
zephyr_library_sources_ifdef(CONFIG_GB_DEBUG          greybus-debug.c)
zephyr_library_sources_ifdef(CONFIG_GB_HID            hid.c)
zephyr_library_sources_ifdef(CONFIG_GB_I2C            i2c.c)
zephyr_library_sources_ifdef(CONFIG_GB_LIGHTS         lights.c)
zephyr_library_sources_ifdef(CONFIG_GB_LOOPBACK       loopback.c)
zephyr_library_sources_ifdef(CONFIG_GB_POWER_SUPPLY   power_supply.c)
zephyr_library_sources_ifdef(CONFIG_GB_PWM            pwm-protocol.c)
zephyr_library_sources_ifdef(CONFIG_GB_SDIO           sdio.c)
zephyr_library_sources_ifdef(CONFIG_GB_SPI            spi.c)
zephyr_library_sources_ifdef(CONFIG_GB_UART           uart.c)
zephyr_library_sources_ifdef(CONFIG_GB_USB            usb.c)
zephyr_library_sources_ifdef(CONFIG_GB_VIBRATOR       vibrator.c)
