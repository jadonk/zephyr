/*
 * Copyright (c) 2021 Jackychen
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_ads1115

#include <drivers/i2c.h>
#include <drivers/gpio.h>
#include <init.h>
#include <kernel.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <drivers/sensor.h>
#include <net/net_ip.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(ads1115, LOG_LEVEL_INF);

#include <device.h>
#include <zephyr/types.h>

#define ADS1115_REG_CONVERSION	0x00
#define ADS1115_REG_CONFIG		0x01
#define ADS1115_REG_LO_TH		0x02
#define ADS1115_REG_HI_TH		0x03

#define ADS1115_AIN_INDEX_MAX	3

struct ads1115_data {
	struct k_timer *		timer;
	struct k_work 			sample_worker;
	const struct device *		i2c_master;
	uint16_t 			i2c_slave_addr;
	uint16_t 			ain_value[4];
	uint16_t			ch_index;
	uint8_t				continuous_mode;
	uint8_t				device_index;
	const struct gpio_dt_spec	int_gpio;
	struct gpio_callback		gpio_cb;
	struct k_sem			data_sem;
};

static int ads1115_init(const struct device *dev);
static int ads1115_sample_fetch(const struct device *dev,enum sensor_channel chan);
static int ads1115_channel_get(const struct device *dev,enum sensor_channel chan,struct sensor_value *val);
static int ads1115_attr_set(const struct device *dev,enum sensor_channel chan,enum sensor_attribute attr,const struct sensor_value *val);

static const struct sensor_driver_api ads1115_api_funcs = {
	.sample_fetch = ads1115_sample_fetch,
	.channel_get = ads1115_channel_get,
	.attr_set = ads1115_attr_set,
};

static void ads1115_gpio_callback(const struct device *dev,
				  struct gpio_callback *cb, uint32_t pins)
{
	struct ads1115_data *p_ads1115_data = CONTAINER_OF(cb, struct ads1115_data, gpio_cb);

	ARG_UNUSED(pins);

	gpio_pin_interrupt_configure_dt(&p_ads1115_data->int_gpio, GPIO_INT_DISABLE);
	k_sem_give(&p_ads1115_data->data_sem);
}

static int ads1115_reg_write(struct ads1115_data *p_data, uint8_t reg, uint16_t *p_val)
{
	int err = 0;

	uint8_t wr_buff[3] = {0};
	uint16_t wr_value = *p_val;

	wr_buff[0] = reg;
	wr_buff[1] = (wr_value >> 8) & 0xff;
	wr_buff[2] = wr_value & 0xff;

	err = i2c_write(p_data->i2c_master, wr_buff, 3, p_data->i2c_slave_addr);

	if (err < 0) {
		LOG_ERR("0x%02x ads1115_reg_write reg 0x%02x error %d",p_data->i2c_slave_addr,reg,err);
		return err;
	}

	return 0;
}

static int ads1115_reg_read(struct ads1115_data *p_data, uint8_t reg, uint16_t *p_val)
{
	int err = 0;

	uint8_t wr_buff[1] = {0};
	uint8_t rd_buff[2] = {0};

	wr_buff[0] = reg;

	err = i2c_write_read(p_data->i2c_master, p_data->i2c_slave_addr, wr_buff, 1, rd_buff, 2);
	if (err < 0) {
		LOG_ERR("0x%02x ads1115_reg_read reg 0x%02x error %d",p_data->i2c_slave_addr,reg,err);
		return err;
	}

	*p_val = (uint16_t)rd_buff[0] << 8;
	*p_val |= rd_buff[1];

	return 0;
}

/*
static int ads1115_ainx_chan_change(struct ads1115_data *p_data, uint8_t ainx_index)
{
	int err = 0;
	uint16_t	config = 0;
	uint16_t    wr_value = 0;

	if(ainx_index > ADS1115_AIN_INDEX_MAX)
	{
		LOG_ERR("unvalid  ainx_index %d",ainx_index);
		return -EINVAL;
	}

	err = ads1115_reg_read(p_data, ADS1115_REG_CONFIG, &config);
	if(err < 0)
	{
		return -EINVAL;
	}

	config &= 0x8fff;
	wr_value = 0x04 + ainx_index;
	wr_value <<= 12;
	wr_value |= config;
	err = ads1115_reg_write(p_data, ADS1115_REG_CONFIG, &wr_value);
	if(err < 0)
	{
		return -EINVAL;
	}

	return 0;
}

static int ads1115_once_conversion_start(struct ads1115_data *p_data)
{
	int err = 0;
	uint16_t	config = 0;
	uint16_t    wr_value = 0;

	err = ads1115_reg_read(p_data, ADS1115_REG_CONFIG, &config);
	if(err < 0)
	{
		return -EINVAL;
	}

	wr_value = config | 0x8000;

	err = ads1115_reg_write(p_data, ADS1115_REG_CONFIG, &wr_value);
	if(err < 0)
	{
		return -EINVAL;
	}

	return 0;
}
*/

static int ads1115_chan_change_with_start_conversion(struct ads1115_data *p_data,uint8_t ainx_index)
{
	int err = 0;
	uint16_t	config = 0;
	uint16_t    wr_value = 0;

	if(ainx_index > ADS1115_AIN_INDEX_MAX)
	{
		LOG_ERR("unvalid  ainx_index %d",ainx_index);
		return -EINVAL;
	}

	err = ads1115_reg_read(p_data, ADS1115_REG_CONFIG, &config);
	if(err < 0)
	{
		return -EINVAL;
	}

	config &= 0x8fff;
	wr_value = 0x04 + ainx_index;
	wr_value <<= 12;
	wr_value |= config;

	wr_value |= 0x8000;

	err = ads1115_reg_write(p_data, ADS1115_REG_CONFIG, &wr_value);
	if(err < 0)
	{
		return -EINVAL;
	}

	return 0;
}


static int ads1115_once_conversion_status_get(struct ads1115_data *p_data,uint16_t *p_status)
{
	int err = 0;

	err = ads1115_reg_read(p_data, ADS1115_REG_CONFIG, p_status);
	if(err < 0)
	{
		return -EINVAL;
	}

	*p_status &= 0x8000;

	return 0;
}

static int ads1115_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct ads1115_data *p_ads1115_data = dev->data;

	switch (chan)
	{
		case SENSOR_CHAN_VOLTAGE:
#if 0
			/* 0.25mV / LSB - 5x divider, 4.096V full-scale @ 16 bit */
			val->val1 = p_ads1115_data->ain_value[0]/0x10;
			val->val2 = (p_ads1115_data->ain_value[0]&0x8000)<<31 |
				(p_ads1115_data->ain_value[0]&0xF;
#else
			val->val1 = p_ads1115_data->ain_value[0];
			val->val2 = 0;
#endif
			break;

		default:
			return -EINVAL;
			break;
	}

	return 0;
}

static int ads1115_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
   	struct ads1115_data *p_ads1115_data = dev->data;
	uint8_t i = 0;
	uint16_t status = 0;
	uint8_t rty = 0;

	if(p_ads1115_data->continuous_mode)
	{
		if(p_ads1115_data->int_gpio.port != NULL) {
			gpio_pin_interrupt_configure_dt(&p_ads1115_data->int_gpio,
						GPIO_INT_EDGE_TO_ACTIVE);

			k_sem_take(&p_ads1115_data->data_sem, K_FOREVER);

			ads1115_reg_read(p_ads1115_data, ADS1115_REG_CONVERSION, &p_ads1115_data->ain_value[i]);
			LOG_DBG("salver_addr 0x%02x get ain%d value 0x%04x",p_ads1115_data->i2c_slave_addr,i,p_ads1115_data->ain_value[i]);
		}
	}
	else
	{
		if(p_ads1115_data->ch_index > ADS1115_AIN_INDEX_MAX)
		{
			LOG_WRN("Invalid sampling channle. Please set a valid sampling channle");
		}
		else
		{
			ads1115_chan_change_with_start_conversion(p_ads1115_data, p_ads1115_data->ch_index);
			do
			{
				k_msleep(1);
				ads1115_once_conversion_status_get(p_ads1115_data , &status);
				LOG_DBG("try %d to get conversion status is 0x%04x",rty,status);
			}while( (!status) && (rty++ < 3) );

			rty = 0;
			if(status)
			{
				ads1115_reg_read(p_ads1115_data, ADS1115_REG_CONVERSION, &p_ads1115_data->ain_value[i]);
				LOG_DBG("salver_addr 0x%02x get ain%d value 0x%04x",p_ads1115_data->i2c_slave_addr,i,p_ads1115_data->ain_value[i]);
			}
			else
			{
				p_ads1115_data->ain_value[i] = 0;
				LOG_WRN("salver_addr 0x%02x Can't get ain%d",p_ads1115_data->i2c_slave_addr,i);
			}
		}
	}
   	return 0;
}

static int ads1115_attr_set(const struct device * dev,
				enum sensor_channel chan,
				enum sensor_attribute attr,
				const struct sensor_value * p_val)
{
	struct ads1115_data *p_ads1115_data = dev->data;

	if(attr == SENSOR_ATTR_FULL_SCALE)
	{
		//full channel
		if(p_val->val1)
		{
			p_ads1115_data->continuous_mode = true;
		}
		//single channel
		else
		{
			p_ads1115_data->continuous_mode = false;
			p_ads1115_data->ch_index = (uint16_t)chan;
		}
	}
	else
	{
		LOG_WRN("Invalid attribute to set");
	}

	return 0;
}

static int ads1115_init(const struct device *dev)
{
	int err = 0;
	uint16_t	config = 0;
	uint16_t    wr_value = 0;

	struct ads1115_data *p_ads1115_data = dev->data;

	p_ads1115_data->i2c_master = device_get_binding(DT_INST_BUS_LABEL(0));
	if(!p_ads1115_data->i2c_master)
	{
		LOG_ERR("I2C master not found: %s",DT_INST_BUS_LABEL(0));
		return -EINVAL;
	}

	/* Try 100ms delay at the start */
	k_msleep(100);

	err = ads1115_reg_read(p_ads1115_data, ADS1115_REG_CONFIG, &config);
	if(err < 0)
	{
		LOG_ERR("ads1115_init err!");
		return -EINVAL;
	}

#if 0
	wr_value =
		0 << 15			/* OS = No conversion start */
		| 0x4 << 12		/* MUX = 100b: AINP = AIN0 and AINN = GND */
	        | 1 << 9		/* FSR = 001b: 4.096V */
	        | 0 << 8		/* MODE = 0: Continuous-conversion mode */
	        | 0x5 << 5		/* DR = 101b: 250 samples per second */
	        | 0 << 4		/* COMP_MODE = 0: Traditional comparator */
	        | 1 << 3		/* COMP_POL = 1: Active high polarity */
	        | 0 << 2		/* COMP_MODE = 0: Traditional comparator */
	        | 3 << 0;		/* COMP_QUE = 11b: Disable comparator */
	err = ads1115_reg_write(p_ads1115_data, ADS1115_REG_CONFIG, &wr_value);
#endif
	/* FSR = 6.144 */
	config &= 0xf11f;
	wr_value = 0;
	wr_value <<= 9;
	config |= wr_value;
	/* DR = 860 SPS */
	config &= 0xff1f;
	wr_value = 7;
	wr_value <<= 5;
	config |= wr_value;
	/* Set the sampling channel */
	config &= 0x8fff;
	wr_value = 0x04 + p_ads1115_data->ch_index;
	wr_value <<= 12;
	config |= wr_value;

	/* if device in continuous_mode */
	if(p_ads1115_data->continuous_mode)
	{
		if(p_ads1115_data->int_gpio.port != NULL) {
			k_sem_init(&p_ads1115_data->data_sem, 0, K_SEM_MAX_LIMIT);

			/* setup data ready gpio interrupt */
			gpio_pin_configure_dt(&p_ads1115_data->int_gpio, GPIO_INPUT);

			gpio_init_callback(&p_ads1115_data->gpio_cb,
				ads1115_gpio_callback,
				p_ads1115_data->int_gpio.pin);

			if (gpio_add_callback(p_ads1115_data->int_gpio.port, &p_ads1115_data->gpio_cb) < 0) {
				LOG_DBG("Failed to set GPIO callback");
				return -EIO;
			}

			gpio_pin_interrupt_configure_dt(&p_ads1115_data->int_gpio,
						GPIO_INT_EDGE_TO_ACTIVE);

			/* Continuous-conversion mode */
			config &= 0xfeff;
			/* Assert after four conversions */
			config &= 0xfffc;
			config |= 0x02;
		}
	}

	err = ads1115_reg_write(p_ads1115_data, ADS1115_REG_CONFIG, &config);
	if(err < 0)
	{
		LOG_ERR("ads1115_init err!");
		return -EINVAL;
	}
	ads1115_reg_read(p_ads1115_data, ADS1115_REG_CONFIG, &config);
	LOG_INF("ads1115_init 0x%04x finsh , config reg is 0x%04x,chan_index = %d",p_ads1115_data->i2c_slave_addr,	\
				config,p_ads1115_data->ch_index);
	return err;
}

#define ADS1115_DEV(inst) DT_INST(inst, DT_DRV_COMPAT)
#define ADS1115_PROP(inst, prop) DT_PROP(DT_INST(inst, DT_DRV_COMPAT), prop)

#define DEFINE_ADS1115(inst)						\
									\
static struct ads1115_data m_ads1115_data_##inst = {			\
	.i2c_slave_addr = DT_INST_REG_ADDR(inst),			\
	.continuous_mode = ADS1115_PROP(inst,continuous_mode),		\
	.ch_index = ADS1115_PROP(inst,sampling_channel),		\
	.int_gpio = GPIO_DT_SPEC_GET_OR(ADS1115_DEV(inst), int_gpios, {.port=NULL}),	\
};									\
									\
DEVICE_DT_INST_DEFINE(inst,						\
		ads1115_init,						\
		device_pm_control_nop,					\
		&m_ads1115_data_##inst,					\
		NULL,							\
		POST_KERNEL,						\
		CONFIG_SENSOR_INIT_PRIORITY,				\
		&ads1115_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_ADS1115)
