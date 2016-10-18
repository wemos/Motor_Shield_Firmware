#include "stm32f0xx_hal.h"
#include "tb6612.h"

/*
total 4bytes

|0.5byte CMD| 3.5byte Parm|

CMD								| 	parm
0x0X  set freq  	|  uint32  freq
0x10  set motorA  |  uint8 dir  uint16 pwm
0x11  set motorB  |  uint8 dir  uint16 pwm


return 

'OK'  - OK
'ER'  - Error

*/

uint8_t user_i2c_proc(uint8_t i2c_data[4], uint8_t * pdata_out)
{
	uint8_t cmd;
	uint32_t freq=0;
	uint8_t ch;
	uint8_t dir;
	uint16_t pluse;
	cmd=i2c_data[0]>>4;
	
	switch(cmd)
	{
		case 0:
			freq=(uint32_t)(i2c_data[0]&0x0f)<<24|(uint32_t)i2c_data[1]<<16|(uint32_t)i2c_data[2]<<8|(uint32_t)i2c_data[3];
			Set_Freq(freq);

			pdata_out[0]='o';
			pdata_out[1]='k';
			break;
		case 1:

			ch=i2c_data[0]&0x01;
			dir=i2c_data[1];
			pluse=(uint16_t)i2c_data[2]<<8|(uint16_t)i2c_data[3];
		
			Set_TB6612_Dir(ch,dir);
			
			if(dir!=0x03)
				Set_PWM(ch,pluse);		
			
			pdata_out[0]='o';
			pdata_out[1]='k';
			break;
		default:
			
			break;
	}
	
	return 0;
}

