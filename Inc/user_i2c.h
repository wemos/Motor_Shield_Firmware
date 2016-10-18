#ifndef __USER_I2C_H
#define __USER_I2C_H

#include "stm32f0xx_hal.h"

uint8_t user_i2c_proc(uint8_t i2c_data[4], uint8_t * pdata_out);

#endif

