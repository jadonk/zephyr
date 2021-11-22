/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 * Copyright (c) 2020 Jason Kridner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/led.h>
#include <drivers/sensor.h>
#include <drivers/adc.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <net/net_ip.h>
#include <net/socket.h>
#include <sys/util.h>
#include <random/rand32.h>
#include <devicetree.h>
#include <devicetree/io-channels.h>

#include <math.h>

#define LOG_LEVEL LOG_LEVEL_INF
#include <logging/log.h>
LOG_MODULE_REGISTER(sensortest);

#define BLINK_MS 500

#define ADC_NUM_CHANNELS	DT_PROP_LEN(DT_PATH(zephyr_user), io_channels)
#define ADC_NODE		DT_PHANDLE(DT_PATH(zephyr_user), io_channels)
#define ADC_REFERENCE		ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME_DEFAULT
#define ADC_REF_MV		49152  /* 4.096V * 12x divider * 1000mV/V */

#define MAX_STR_LEN 200
static char outstr[MAX_STR_LEN];

enum api {
	BUTTON_API,
	SENSOR_API,
	ADC_API,
};

enum edev {
	BUTTON,
	LIGHT,
	ACCEL,
	HUMIDITY,
	ENVIRONMENT,
	AIRQUALITY,
	PARTICULATE,
	ADC_0,
	ADC_1,
	ADC_2,
	NUM_DEVICES,
};

struct led_work {
	uint8_t active_led;
	struct k_work_delayable dwork;
};

static void sensor_work_handler(struct k_work *work);

static const char *device_labels[NUM_DEVICES] = {
	[BUTTON] = DT_LABEL(DT_ALIAS(sw0)),
	[LIGHT] = "LIGHT",
	[ACCEL] = "ACCEL",
	[HUMIDITY] = "HUMIDITY",
	[ENVIRONMENT] = "ENVIRONMENT",
	[AIRQUALITY] = "AIRQUALITY",
	[PARTICULATE] = "PARTICULATE",
	[ADC_0] = "ADC_0",
	[ADC_1] = "ADC_1",
	[ADC_2] = "ADC_2",
};

static const char *device_names[NUM_DEVICES] = {
	[BUTTON] = DT_GPIO_LABEL(DT_ALIAS(sw0), gpios),
	[LIGHT] = "OPT3001-LIGHT",
	[ACCEL] = "LIS2DE12-ACCEL",
	[HUMIDITY] = "HDC2010-HUMIDITY",
	[ENVIRONMENT] = "BME680-ENVIRONMENT",
	[AIRQUALITY] = "SGP30-AIRQUALITY",
	[PARTICULATE] = "HM3301-PARTICULATE",
	[ADC_0] = "ADS1115-ADC0",
	[ADC_1] = "ADS1115-ADC1",
	[ADC_2] = "ADS1115-ADC2",
};

static const uint8_t device_pins[NUM_DEVICES] = {
	[BUTTON] = DT_GPIO_PIN(DT_ALIAS(sw0), gpios),
};

static const enum api apis[NUM_DEVICES] = {
	BUTTON_API,
	SENSOR_API, /* LIGHT */
	SENSOR_API, /* ACCEL */
	SENSOR_API, /* HUMIDITY */
	SENSOR_API, /* ENVIRONMENT */
	SENSOR_API, /* AIRQUALITY */
	SENSOR_API, /* PARTICULATE */
	ADC_API, /* ADC_0 */
	ADC_API, /* ADC_1 */
	ADC_API, /* ADC_2 */
};

static struct device *devices[NUM_DEVICES];

static struct k_work_delayable adc_dwork;
static struct led_work led_work;
K_WORK_DEFINE(sensor_work, sensor_work_handler);
static struct gpio_callback button_callback;

static struct sockaddr_in6 addr;
static int fd = -1;

/* Set TIMED_SENSOR_READ to 0 to disable */
#define TIMED_SENSOR_READ 60
static int sensor_read_count = TIMED_SENSOR_READ;

static void led_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	/*
	int r;
	uint8_t prev_led;

	LOG_DBG("%s(): active_led: %u", __func__, led_work.active_led);

	if (led_work.active_led == LED_SUBG) {
		prev_led = LED_24G;
	} else {
		prev_led = LED_SUBG;
	}

	r = gpio_pin_set(devices[prev_led], device_pins[prev_led], 0);
	__ASSERT(r == 0, "failed to turn off led %u: %d", prev_led, r);

	r = gpio_pin_set(devices[led_work.active_led],
			 device_pins[led_work.active_led], 1);
	__ASSERT(r == 0, "failed to turn on led %u: %d", led_work.active_led,
		 r);

	if (led_work.active_led == LED_SUBG) {
		led_work.active_led = LED_24G;
	} else {
		led_work.active_led = LED_SUBG;
	}

	r = k_work_schedule(&led_work.dwork, K_MSEC(BLINK_MS));
	__ASSERT(r == 0, "k_work_schedule() failed for LED %u work: %d",
		 led_work.active_led, r);
	*/

	if (sensor_read_count > 0) {
		sensor_read_count--;
		if (sensor_read_count <= 0) {
			sensor_read_count = TIMED_SENSOR_READ;
			if(TIMED_SENSOR_READ > 0)
				sensor_read_count +=
				       	sys_rand32_get() % TIMED_SENSOR_READ;
			k_work_submit(&sensor_work);
		}
	}
}

static void print_sensor_value(size_t idx, const char *chan,
			       struct sensor_value *val)
{
	LOG_INF("%s: %s%d,%d", device_labels[idx], chan, val->val1, val->val2);

	sprintf(outstr+strlen(outstr), "%d%c:", idx, chan[0]);
	sprintf(outstr+strlen(outstr), "%d", val->val1);
	if (val->val2 != 0) {
		sprintf(outstr+strlen(outstr), ".%02d;", abs(val->val2) / 10000);
	} else {
		sprintf(outstr+strlen(outstr), ";");
	}
}

static void send_sensor_value()
{
	if ((fd >= 0) && (strlen(outstr) > 0)) {
		//LOG_INF("%s", log_strdup(outstr));
		sendto(fd, outstr, strlen(outstr), 0,
			(const struct sockaddr *) &addr,
			sizeof(addr));
	}

	outstr[0] = '\0';
}

/* Sampling frequency: 200Hz, 5ms
 * Sample window: 16 samples, 80ms, 12.5Hz
 * Data window: 32 RMS samples, 2.5s of data
 */
uint16_t ain0_buffer[32];

struct adc_channel_cfg ain0_channel_cfg = {
	.gain = ADC_GAIN_1,
	.reference = ADC_REF_INTERNAL,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = DT_IO_CHANNELS_INPUT_BY_IDX(DT_PATH(zephyr_user), 0),
	.differential = 0,
};

const struct adc_sequence_options ain0_seq_options = {
	.interval_us = 1000,
	.extra_samplings = 0,
	//.extra_samplings = 31,
	.callback = NULL,
	.user_data = NULL,
};

struct adc_sequence sequence0 = {
	.options = &ain0_seq_options,
	.channels = 1,
	.buffer = ain0_buffer,
	.buffer_size = sizeof(ain0_buffer),
	.resolution = 16,
	.oversampling = 0,
	.calibrate = false,
};

static void adc_work_handler(struct k_work *work)
{
	int err;
	const struct device * dev = devices[ADC_0];

	err = adc_read(dev, &sequence0);
	if (err != 0) {
		LOG_ERR("ADC reading failed with error %d.", err);
		return;
	}

	LOG_INF("ADC reading:");
	for (uint8_t i = 0; i < 2; i+=2) {
		int32_t raw_value = ain0_buffer[i/2];
		int32_t mv_value = raw_value;
		adc_raw_to_millivolts(ADC_REF_MV, ADC_GAIN_1,
			16, &mv_value);
		LOG_INF(" %d: %d = %d mV", i/2, raw_value, mv_value);
	}

	err = k_work_schedule(&adc_dwork, K_MSEC(2500));
	__ASSERT(err == 0, "k_work_schedule() failed for adc_dwork: %d", r);

	return;

	/*
	uint8_t count = 0;
	struct sensor_value val[ADC_SAMPLING_CNT_MAX];
	struct sensor_value val_rms = {0};
	uint64_t sum_square = 0;
	int r = 0;
	uint8_t valid_data_cnt = 0;

	// Sampling
	for(count = 0; count < ADC_SAMPLING_CNT_MAX; count++)
	{
		sensor_sample_fetch(devices[ADC_0]);

		sensor_channel_get(devices[ADC_0], SENSOR_CHAN_VOLTAGE, &val[count]);

		//	valid_data_cnt
		if(val[count].val1 != 0xffff)
		{
			valid_data_cnt++;
			sum_square += val[count].val1 * val[count].val1;
		}
	}

	//data_handler
	val_rms.val1 = sqrt(sum_square/valid_data_cnt);

	print_sensor_value(ADC_0, "ADC_0: ", &val_rms);
	send_sensor_value();

	r = k_work_schedule(&adc_dwork, K_MSEC(2500));
	__ASSERT(r == 0, "k_work_schedule() failed for adc_dwork: %d", r);
	*/
}

static void sensor_work_handler(struct k_work *work)
{
	struct sensor_value val;
	struct sensor_value th[2] = {{0,0}, {0,0}}; /* calibration data for SGP30 */

	outstr[0] = '\0';

	for (size_t i = 0; i < NUM_DEVICES; ++i) {
		if (apis[i] != SENSOR_API) {
			continue;
		}

		if (devices[i] == NULL) {
			continue;
		}

		sensor_sample_fetch(devices[i]);

		if (i == LIGHT) {
			sensor_channel_get(devices[i], SENSOR_CHAN_LIGHT, &val);
			print_sensor_value(i, "l: ", &val);
			send_sensor_value();
			continue;
		}

		if (i == ACCEL) {
			sensor_channel_get(devices[i], SENSOR_CHAN_ACCEL_X,
					   &val);
			print_sensor_value(i, "x: ", &val);
			sensor_channel_get(devices[i], SENSOR_CHAN_ACCEL_Y,
					   &val);
			print_sensor_value(i, "y: ", &val);
			sensor_channel_get(devices[i], SENSOR_CHAN_ACCEL_Z,
					   &val);
			print_sensor_value(i, "z: ", &val);
			send_sensor_value();
			continue;
		}

		if (i == HUMIDITY) {
			sensor_channel_get(devices[i], SENSOR_CHAN_HUMIDITY,
					   &val);
			print_sensor_value(i, "h: ", &val);
			sensor_channel_get(devices[i], SENSOR_CHAN_AMBIENT_TEMP,
					   &val);
			print_sensor_value(i, "t: ", &val);
			send_sensor_value();
			continue;
		}

		if (i == ENVIRONMENT) {
			sensor_channel_get(devices[i], SENSOR_CHAN_AMBIENT_TEMP, &val);
			th[0].val1 = val.val1;
			th[0].val2 = val.val2;
			print_sensor_value(i, "t: ", &val);
			sensor_channel_get(devices[i], SENSOR_CHAN_PRESS, &val);
			print_sensor_value(i, "p: ", &val);
			sensor_channel_get(devices[i], SENSOR_CHAN_HUMIDITY, &val);
			th[1].val1 = val.val1;
			th[1].val2 = val.val2;
			print_sensor_value(i, "h: ", &val);
			sensor_channel_get(devices[i], SENSOR_CHAN_GAS_RES, &val);
			print_sensor_value(i, "g: ", &val);
			send_sensor_value();
			continue;
		}
		
		if (i == AIRQUALITY) {
			sensor_channel_get(devices[i], SENSOR_CHAN_VOC, &val);
			print_sensor_value(i, "v: ", &val);
			sensor_channel_get(devices[i], SENSOR_CHAN_CO2, &val);
			print_sensor_value(i, "c: ", &val);
			if(th[0].val1 != 0) {
				sensor_attr_set(devices[i], SENSOR_CHAN_VOC,
					SENSOR_ATTR_CALIB_TARGET, &th[0]);
			}
			send_sensor_value();
			continue;
		}

		if (i == PARTICULATE) {
			sensor_channel_get(devices[i], SENSOR_CHAN_PM_1_0, &val);
			print_sensor_value(i, "1: ", &val);
			sensor_channel_get(devices[i], SENSOR_CHAN_PM_2_5, &val);
			print_sensor_value(i, "2: ", &val);
			sensor_channel_get(devices[i], SENSOR_CHAN_PM_10, &val);
			print_sensor_value(i, "X: ", &val);
			send_sensor_value();
			continue;
		}

		if(i == ADC_0) {
			sensor_channel_get(devices[i], SENSOR_CHAN_VOLTAGE, &val);
			print_sensor_value(i, "ADC_0: ", &val);
			send_sensor_value();
			continue;
		};

		if(i == ADC_1) {
			sensor_channel_get(devices[i], SENSOR_CHAN_VOLTAGE, &val);
			print_sensor_value(i, "ADC_1: ", &val);
			send_sensor_value();
			continue;
		};

		if(i == ADC_2) {
			sensor_channel_get(devices[i], SENSOR_CHAN_VOLTAGE, &val);
			print_sensor_value(i, "ADC_2: ", &val);
			send_sensor_value();
			continue;
		};
	}
}

static void button_handler(struct device *port, struct gpio_callback *cb,
			   gpio_port_pins_t pins)
{
	ARG_UNUSED(port);

	gpio_pin_t pin = device_pins[BUTTON];
	gpio_port_pins_t mask = BIT(pin);

	if ((mask & cb->pin_mask) != 0) {
		if ((mask & pins) != 0) {
			/* BEL (7) triggers BEEP on MSP430 */
			LOG_INF("%c%s event", 7, device_labels[BUTTON]);
			/* print sensor readings */
			k_work_submit(&sensor_work);
		}
	}
}

void main(void)
{
	int r;
	outstr[0] = '\0';

	/* Set RF_SW */
	const struct device *rf_sw_dev;
	rf_sw_dev = device_get_binding(DT_GPIO_LABEL(DT_NODELABEL(rf_sw), gpios));
	r = gpio_pin_configure(rf_sw_dev, DT_GPIO_PIN_BY_IDX(DT_NODELABEL(rf_sw), gpios, 0),
		GPIO_OUTPUT_LOW);  /* Disable SubG +20dB */
	r = gpio_pin_configure(rf_sw_dev, DT_GPIO_PIN_BY_IDX(DT_NODELABEL(rf_sw), gpios, 1),
		GPIO_OUTPUT_HIGH); /* Enable SubG TX/RX 0dB */
	r = gpio_pin_configure(rf_sw_dev, DT_GPIO_PIN_BY_IDX(DT_NODELABEL(rf_sw), gpios, 2),
		GPIO_OUTPUT_LOW);  /* Disable 2.4GHz TX/RX */

	fd = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
	if (fd < 0) {
		LOG_ERR("failed to open socket");
	} else {
		memset(&addr, 0, sizeof(struct sockaddr_in6));
		addr.sin6_family = AF_INET6;
		addr.sin6_port = htons(9999);
		inet_pton(AF_INET6, "ff02::1", &addr.sin6_addr);
	}

	for (size_t i = 0; i < NUM_DEVICES; ++i) {
		LOG_INF("opening device %s", device_labels[i]);
		devices[i] =
			(struct device *)device_get_binding(device_names[i]);
		if (devices[i] == NULL) {
			LOG_ERR("failed to open device %s", device_labels[i]);
			continue;
		}

		/* per-device setup */
		switch (apis[i]) {
		/*
		case LED_API:
			r = gpio_pin_configure(devices[i], device_pins[i],
					       GPIO_OUTPUT_LOW);
			__ASSERT(r == 0,
				 "gpio_pin_configure(%s, %u, %x) failed: %d",
				 device_labels[i], device_pins[i],
				 GPIO_OUTPUT_LOW, r);
			break;
		*/
		case BUTTON_API:
			r = gpio_pin_configure(
				devices[i], device_pins[i],
				(GPIO_INPUT |
				 DT_GPIO_FLAGS(DT_ALIAS(sw0), gpios)));
			__ASSERT(r == 0,
				 "gpio_pin_configure(%s, %u, %x) failed: %d",
				 device_labels[i], device_pins[i],
				 (GPIO_INPUT |
				  DT_GPIO_FLAGS(DT_ALIAS(sw0), gpios)),
				 r);
			r = gpio_pin_interrupt_configure(
				devices[i], device_pins[i],
				GPIO_INT_EDGE_TO_ACTIVE);
			__ASSERT(
				r == 0,
				"gpio_pin_interrupt_configure(%s, %u, %x) failed: %d",
				device_labels[i], device_pins[i],
				GPIO_INT_EDGE_TO_ACTIVE, r);
			break;

		case SENSOR_API:
			break;

		default:
			break;
		}
	}

	/* setup timer-driven LED event */
	k_work_init_delayable(&led_work.dwork, led_work_handler);
	//led_work.active_led = LED_SUBG;
	r = k_work_schedule(&led_work.dwork, K_MSEC(BLINK_MS));
	__ASSERT(r == 0, "k_work_schedule() failed for LED %u work: %d",
		 LED_SUBG, r);

	if(devices[ADC_0])
	{
		LOG_INF("setting up device %s", device_labels[ADC_0]);

		adc_channel_setup(devices[ADC_0], &ain0_channel_cfg);

		/* setup timer-driven ADC event */
		k_work_init_delayable(&adc_dwork, adc_work_handler);
		r = k_work_schedule(&adc_dwork, K_MSEC(2500));
		__ASSERT(r == 0, "k_work_schedule() failed for adc_dwork: %d", r);
	}

	/* setup input-driven button event */
	gpio_init_callback(
		&button_callback, (gpio_callback_handler_t)button_handler,
		BIT(device_pins[BUTTON]));
	r = gpio_add_callback(devices[BUTTON], &button_callback);
	__ASSERT(r == 0, "gpio_add_callback() failed: %d", r);

	for (;;) {
		k_sleep(K_MSEC(1000));
	}
}

