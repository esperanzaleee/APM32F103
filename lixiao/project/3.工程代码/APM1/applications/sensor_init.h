/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-08-03     Esperanza       the first version
 */
#ifndef APPLICATIONS_SENSOR_INIT_H_
#define APPLICATIONS_SENSOR_INIT_H_

#include "sensor_asair_aht10.h"

#define AHT10_I2C_BUS  "i2c1"

typedef struct
{
    float AHT10_temp_val;
    float AHT10_humi_val;
} Temp_Humi_val;

#endif /* APPLICATIONS_SENSOR_INIT_H_ */
