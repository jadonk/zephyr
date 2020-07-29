/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __TEST_I2C_H__
#define __TEST_I2C_H__

#include <devicetree.h>

#if DT_NODE_HAS_STATUS(DT_ALIAS(i2c_0), okay)
#define I2C_DEV_NAME	DT_LABEL(DT_ALIAS(i2c_0))
#elif DT_NODE_HAS_STATUS(DT_ALIAS(i2c_1), okay)
#define I2C_DEV_NAME	DT_LABEL(DT_ALIAS(i2c_1))
#elif DT_NODE_HAS_STATUS(DT_ALIAS(i2c_2), okay)
#define I2C_DEV_NAME	DT_LABEL(DT_ALIAS(i2c_2))
#else
#error "Please set the correct I2C device"
#endif

#endif /* __TEST_I2C_H__ */
