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
#include <errno.h>
#include <drivers/uart.h>
#include <shell/shell.h>
#include <driverlib/ioc.h>
#include <drivers/i2c.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define UARTPINMUX_MODE_MIKROBUS 0x0A
#define UARTPINMUX_MODE_MSP430 0x0B

#define MSP430_SLAVE_ADDR 0x04
#define MSP430_I2CSLAVE_SET_UARTMIKROBUS_REG 0x08
#define MSP430_I2CSLAVE_SET_UARTMSP430_REG 0x09
#define MSP430_I2CSLAVE_GET_UARTMUX_REG 0x0C

#define IOC_PORT_MCU_UART0_TX 0x00000010
#define IOC_PORT_MCU_UART0_RX 0x0000000F

#define MIKROBUS0_UART0_PIN_RX 13
#define MIKROBUS0_UART0_PIN_TX 12

static int uartpinmuxget()
{
    const struct device *i2cmaster = device_get_binding("I2C_0");
    int ret;
    uint8_t rdbuf[1];

    ret = i2c_reg_read_byte(i2cmaster, MSP430_SLAVE_ADDR, MSP430_I2CSLAVE_GET_UARTMUX_REG, rdbuf);
    return rdbuf[0];
}

static int uartpinmuxset(uint8_t mode)
{
    const struct device *i2cmaster = device_get_binding("I2C_0");
    int ret;

    if(mode == UARTPINMUX_MODE_MIKROBUS){
        ret = i2c_reg_write_byte(i2cmaster, MSP430_SLAVE_ADDR, MSP430_I2CSLAVE_SET_UARTMIKROBUS_REG, 0);
        k_sleep(K_MSEC(100));
        while(uartpinmuxget() != UARTPINMUX_MODE_MIKROBUS);
        IOCPortConfigureSet(MIKROBUS0_UART0_PIN_RX, IOC_PORT_MCU_UART0_RX, IOC_STD_INPUT);
        IOCPortConfigureSet(MIKROBUS0_UART0_PIN_TX, IOC_PORT_MCU_UART0_TX, IOC_STD_OUTPUT);
    }
    else if(mode == UARTPINMUX_MODE_MSP430){
        IOCPortConfigureSet(MIKROBUS0_UART0_PIN_RX, IOC_PORT_MCU_UART0_TX, IOC_STD_OUTPUT);
        IOCPortConfigureSet(MIKROBUS0_UART0_PIN_TX, IOC_PORT_MCU_UART0_RX, IOC_STD_INPUT);
        k_sleep(K_MSEC(100));
        ret = i2c_reg_write_byte(i2cmaster, MSP430_SLAVE_ADDR, MSP430_I2CSLAVE_SET_UARTMSP430_REG, 0);
        while(uartpinmuxget() != UARTPINMUX_MODE_MSP430);
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
    else if(uartpinmux_mode == UARTPINMUX_MODE_MSP430){
        shell_print(shell, "Current UART pinmux setting : msp430");
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
