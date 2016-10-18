#ifndef __TB6612_H
#define __TB6612_H

#include "stm32f0xx_hal.h"

extern void Set_Freq(uint32_t freq);
extern void Set_PWM(uint8_t pwm_ch, uint16_t Pulse);
extern void Set_TB6612_Dir(uint8_t motor, uint8_t dir);
extern void Set_PWM_IO(uint8_t ch, uint8_t mode);

extern uint32_t TB6612_FREQ;
extern uint16_t TB6612_PWM_VAL_A;
extern uint16_t TB6612_PWM_VAL_B;

#endif

