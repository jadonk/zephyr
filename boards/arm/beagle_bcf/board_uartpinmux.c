/*
 * Copyright (c) 2021 Vaishnav, BeagleBoard.org Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Implements mechanism to swap UART pins according to operating mode                               
 */
                                                 
#include <zephyr.h>
#include <device.h>
#include <init.h>
#include <kernel.h>
#include <errno.h>
#include <devicetree.h>
#include <drivers/uart.h>
#include <shell/shell.h>
#include <driverlib/ioc.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/util.h>

#define LOG_LEVEL LOG_LEVEL_INF
#include <logging/log.h>
LOG_MODULE_REGISTER(board_uartpinmux);


#define UARTPINMUX_MODE_MIKROBUS 0x0A
#define UARTPINMUX_MODE_MSP430 0x0B
#define UARTPINMUX_MODE_MSP430_LOCK 0x10

#define BCF_I2CSLAVE_IDLE 0x01
#define BCF_I2CSLAVE_UARTMUX_CHANGE 0x02
#define BCF_I2CSLAVE_ENUMERATION_PROFILE 0x03

#define MSP430_SLAVE_ADDR 0x04
#define MSP430_I2CSLAVE_SET_UARTMIKROBUS_REG 0x08
#define MSP430_I2CSLAVE_SET_UARTMSP430_REG 0x09
#define MSP430_I2CSLAVE_GET_UARTMUX_REG 0x0C
#define MSP430_I2CSLAVE_STATUS_REG 0x0D

#define IOC_PORT_MCU_UART0_TX 0x00000010
#define IOC_PORT_MCU_UART0_RX 0x0000000F

#define MIKROBUS0_UART0_PIN_RX 13
#define MIKROBUS0_UART0_PIN_TX 12

static struct gpio_callback i2cslave_callback;
const struct device *i2cmaster;
static void i2cslave_work_handler(struct k_work *work);
K_WORK_DEFINE(i2cslave_work, i2cslave_work_handler);

static int uartpinmuxget()
{    
    int ret;
    uint8_t rdbuf[1];

    ret = i2c_reg_read_byte(i2cmaster, MSP430_SLAVE_ADDR, MSP430_I2CSLAVE_GET_UARTMUX_REG, rdbuf);
    return rdbuf[0];
}

static int uartpinmuxset(uint8_t mode)
{
    int ret;

    if(mode == UARTPINMUX_MODE_MIKROBUS){
        ret = i2c_reg_write_byte(i2cmaster, MSP430_SLAVE_ADDR, MSP430_I2CSLAVE_SET_UARTMIKROBUS_REG, 0);
        k_sleep(K_MSEC(100));
        while((uartpinmuxget() & 0x0F) != UARTPINMUX_MODE_MIKROBUS);
        IOCPortConfigureSet(MIKROBUS0_UART0_PIN_RX, IOC_PORT_MCU_UART0_RX, IOC_STD_INPUT);
        IOCPortConfigureSet(MIKROBUS0_UART0_PIN_TX, IOC_PORT_MCU_UART0_TX, IOC_STD_OUTPUT);
    }
    else if(mode == UARTPINMUX_MODE_MSP430){
        IOCPortConfigureSet(MIKROBUS0_UART0_PIN_RX, IOC_PORT_MCU_UART0_TX, IOC_STD_OUTPUT);
        IOCPortConfigureSet(MIKROBUS0_UART0_PIN_TX, IOC_PORT_MCU_UART0_RX, IOC_STD_INPUT);
        k_sleep(K_MSEC(100));
        ret = i2c_reg_write_byte(i2cmaster, MSP430_SLAVE_ADDR, MSP430_I2CSLAVE_SET_UARTMSP430_REG, 0);
        while((uartpinmuxget() & 0x0F) != UARTPINMUX_MODE_MSP430);
    }
    return ret;
}

static int cmd_uartpinmuxget(const struct shell *shell, size_t argc,
			      char **argv)
{
    int uartpinmux_mode = uartpinmuxget();

    if(uartpinmux_mode == UARTPINMUX_MODE_MIKROBUS){
    	shell_print(shell, "Current UART pinmux setting : mikrobus");
    }
    else if((uartpinmux_mode & 0x0F) == UARTPINMUX_MODE_MSP430){
        shell_print(shell, "Current UART pinmux setting : msp430, lock=%d", (uartpinmux_mode & UARTPINMUX_MODE_MSP430_LOCK) >> 4);
    }
    else{
        shell_print(shell, "Invalid UART pinmux seting");
    }
    
	return 0;
}


static int cmd_uartpinmuxset(const struct shell *shell, size_t argc,
			      char **argv)
{
    int uartpinmux_mode = 0;
    int currentpinmux_mode = uartpinmuxget();

    uartpinmux_mode = strcmp(argv[1], "mikrobus") ? 
                      (strcmp(argv[1], "msp430") ? 0 : UARTPINMUX_MODE_MSP430)
                      : UARTPINMUX_MODE_MIKROBUS;

    if(uartpinmux_mode){
        if(currentpinmux_mode == uartpinmux_mode){
            shell_print(shell, "current UART pinmux mode is same as requested mode : %s", argv[1]);
            return 0;
        }
        if(currentpinmux_mode & UARTPINMUX_MODE_MSP430_LOCK){
            shell_print(shell, "UART pinmux mode locked by MSP430 (active USB-UART)");
            return -EINVAL;
        }     
    	shell_print(shell, "setting UART pinmux setting: %s", argv[1]);
        k_sleep(K_MSEC(100));
        uartpinmuxset(uartpinmux_mode);
    }
    else{
        shell_print(shell, "invalid UART pinmux setting: %s", argv[1]);
        return -EINVAL;
    } 
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_uartpinmux,
	SHELL_CMD_ARG(set, NULL,
		  "Set UART Pinmux setting to desired mode [msp430(default)|mikrobus]",
		  cmd_uartpinmuxset, 2, 0),
    SHELL_CMD_ARG(get, NULL,
		  "Get current UART Pinmux setting, returns: [msp430|mikrobus]",
		  cmd_uartpinmuxget, 1, 0),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(uartpinmux, &sub_uartpinmux, "UART Pinmux setting", NULL);

static void i2cslave_work_handler(struct k_work *work) {        
    int ret;
    uint8_t rdbuf[1];
    int currentpinmux_mode;

    ret = i2c_reg_read_byte(i2cmaster, MSP430_SLAVE_ADDR, MSP430_I2CSLAVE_STATUS_REG, rdbuf);
    switch (rdbuf[0])
    {
    case BCF_I2CSLAVE_UARTMUX_CHANGE:
        currentpinmux_mode = uartpinmuxget(); 
        if(currentpinmux_mode & UARTPINMUX_MODE_MSP430_LOCK){
            LOG_INF("UART MUX lock requested by MSP430");
            if((currentpinmux_mode & 0x0F) == UARTPINMUX_MODE_MIKROBUS){
                LOG_INF("Setting UART pinmux setting : msp430");
                uartpinmuxset(UARTPINMUX_MODE_MSP430);
            }
        }
    #if CONFIG_BCF_UARTMUX_STATE_MIKROBUS
        else {
            LOG_INF("UART MUX unlocked by MSP430");
            if((currentpinmux_mode & 0x0F) == UARTPINMUX_MODE_MSP430){
                LOG_INF("Setting UART pinmux setting : mikrobus");
                uartpinmuxset(UARTPINMUX_MODE_MIKROBUS);
            }
        }
    #endif                
        break;
    default:
        break;
    }    
}
static void i2cslave_handler(struct device *port, struct gpio_callback *cb,
			   gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	gpio_pin_t pin = DT_GPIO_PIN(DT_ALIAS(sw0), gpios);
	gpio_port_pins_t mask = BIT(pin);

	if ((mask & cb->pin_mask) != 0) {
		if ((mask & pins) != 0) {
            k_work_submit(&i2cslave_work);       			
		}
	}
}

static int board_uartmux_init(const struct device *dev)
{
    int r;
    int currentpinmux_mode;

	ARG_UNUSED(dev);
    const struct device *bootgpio = device_get_binding("GPIO_0");
    gpio_port_pins_t bootpin = DT_GPIO_PIN(DT_ALIAS(sw0), gpios);
    i2cmaster = device_get_binding("I2C_0");

    k_sleep(K_MSEC(1000));
    r = gpio_pin_configure(
				bootgpio, bootpin,
				(GPIO_INPUT |
				 DT_GPIO_FLAGS(DT_ALIAS(sw0), gpios)));
    __ASSERT(r == 0,
            "gpio_pin_configure(%s, %u, %x) failed: %d",
            DT_LABEL(DT_ALIAS(sw0)), bootpin,
            (GPIO_INPUT |
            DT_GPIO_FLAGS(DT_ALIAS(sw0), gpios)),
            r);
    r = gpio_pin_interrupt_configure(
                    bootgpio, bootpin,
                    GPIO_INT_EDGE_TO_ACTIVE);
    __ASSERT(
    r == 0,
    "gpio_pin_interrupt_configure(%s, %u, %x) failed: %d",
    DT_LABEL(DT_ALIAS(sw0)), bootpin,
    GPIO_INT_EDGE_TO_ACTIVE, r);

    gpio_init_callback(
		&i2cslave_callback, (gpio_callback_handler_t)i2cslave_handler,
		BIT(bootpin));
	r = gpio_add_callback(bootgpio, &i2cslave_callback);
	__ASSERT(r == 0, "gpio_add_callback() failed: %d", r);

    #if CONFIG_BCF_UARTMUX_STATE_MIKROBUS
        currentpinmux_mode = uartpinmuxget();
        if(currentpinmux_mode & UARTPINMUX_MODE_MSP430_LOCK){
                LOG_ERR("UART pinmux mode locked by MSP430 (active USB-UART)");
                return -EINVAL;
        }
        uartpinmuxset(UARTPINMUX_MODE_MIKROBUS);
    #endif
    
    return 0;
}
SYS_INIT(board_uartmux_init, POST_KERNEL, CONFIG_BOARD_UARTMUX_INIT_PRIO);

