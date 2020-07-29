/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef CONFIG_I2C_SIM

#include <drivers/i2c.h>
#include <drivers/i2c/i2c_sim.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include <zephyr.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(test_i2c_sim, CONFIG_I2C_LOG_LEVEL);

#include "test-greybus-i2c.h"
#include "../../../drivers/sensor/hmc5883l/hmc5883l.h"

enum {
    HMC5883L_REG_CFG_A = HMC5883L_REG_CONFIG_A,
    HMC5883L_REG_CFG_B,
    HMC5883L_REG_MODE_ = HMC5883L_REG_MODE,
    HMC5883L_REG_X_MSB = HMC5883L_REG_DATA_START,
    HMC5883L_REG_X_LSB,
    HMC5883L_REG_Z_MSB,
    HMC5883L_REG_Z_LSB,
    HMC5883L_REG_Y_MSB,
    HMC5883L_REG_Y_LSB,
    HMC5883L_REG_STATUS,
    HMC5883L_REG_ID_A = HMC5883L_REG_CHIP_ID,
    HMC5883L_REG_ID_B,
    HMC5883L_REG_ID_C,
    HMC5883L_REG_NUM,
    HMC5883L_REG_PTR = HMC5883L_REG_NUM,
};

#define HMC5883L_REG_INIT { \
    [HMC5883L_REG_CHIP_ID + 0] = HMC5883L_CHIP_ID_A, \
    [HMC5883L_REG_CHIP_ID + 1] = HMC5883L_CHIP_ID_B, \
    [HMC5883L_REG_CHIP_ID + 2] = HMC5883L_CHIP_ID_C, \
}

static uint8_t hmc5883l_reg[HMC5883L_REG_NUM + 1] = HMC5883L_REG_INIT;

static char *to_string(uint8_t *data, size_t len)
{
    static char buf[64];
    char *p = buf;

    memset(buf, '\0', sizeof(buf));
    for(size_t i = 0; i < len && p < buf + sizeof(buf) - 4; ++i) {
        sprintf(p, "%02x", data[i]);
        p += 2;
        if (i < len - 1) {
            *p++ = ',';
            *p++ = ' ';
        }
    }

    return buf;
}

static int i2c_sim_hmc_callback(struct device *dev,
				 struct i2c_msg *msgs,
				 uint8_t num_msgs,
				 uint16_t addr)
{
    ARG_UNUSED(dev);
    uint8_t j;
    uint8_t ptr;
    uint8_t len;

    for(uint8_t i = 0; i < num_msgs; ++i) {

        ptr = hmc5883l_reg[HMC5883L_REG_PTR];
        //LOG_DBG("%s(): ptr: %u", __func__, ptr);

        char *dump = log_strdup(to_string(msgs[i].buf, msgs[i].len)); 
        LOG_DBG("%s(): %c: addr: %04x: flags: %02x len: %u buf: [%s]",
            __func__, (msgs[i].flags & I2C_MSG_READ) ? 'R': 'W',
            addr, msgs[i].flags, msgs[i].len,
            dump
        );

        switch(msgs[i].len) {
        case 0:
            __ASSERT(msgs[i].len > 0, "unsupported operation length");
            break;
        case 1:
            ptr = msgs[i].buf[0];
            break;
        default:
            len = msgs[i].len;
            if (msgs[i].flags & I2C_MSG_READ) {
                for(j = 0; ptr < HMC5883L_REG_NUM && len > 0; ++ptr, --len, ++j) {
                    msgs[i].buf[j] = hmc5883l_reg[ptr];
                }
            } else {
                __ASSERT((len % 2) == 0, "msgs[%u].len (%u) is not a multiple of two", i, msgs[i].len);
                for(j = 0; len > 0; len -= 2, j += 2) {
                    ptr = msgs[i].buf[j];
                    ptr %= HMC5883L_REG_NUM;
                    if (0 <= ptr && ptr <= 2) {
                        hmc5883l_reg[ptr] = msgs[i].buf[j + 1];
                    }
                }
            }
            break;
        }
        //LOG_DBG("%s(): ptr: %u", __func__, ptr);
        hmc5883l_reg[HMC5883L_REG_PTR] = ptr;
    }

    return 0;
}

void i2c_sim_setup(void)
{
    int r;
    struct device *dev;

    dev = device_get_binding(I2C_DEV_NAME);
    __ASSERT(dev != NULL, "failed to get binding for " I2C_DEV_NAME);

    r = i2c_sim_callback_register(dev, 0x1e, i2c_sim_hmc_callback);
    __ASSERT(r == 0, "failed to register i2c_sim callback: %d", r);
}

#endif /* CONFIG_I2C_SIM */
