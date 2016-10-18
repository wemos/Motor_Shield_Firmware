#include "stm32f0xx_hal.h"

extern TIM_HandleTypeDef htim3;
extern void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
extern void Error_Handler(void);

uint32_t TB6612_FREQ=0;
uint16_t TB6612_PWM_VAL_A=0;
uint16_t TB6612_PWM_VAL_B=0;

void Set_PWM_IO(uint8_t ch, uint8_t mode);

void Set_Freq(uint32_t freq)
{
  
	/*
	freq:
	Max:80,000Hz(80kHz)
	Min:1Hz
	*/
	
	TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;


	
  htim3.Instance = TIM3;
	
	if(freq>80000)
		freq=80000;

	TB6612_FREQ=freq;
	
	if(freq<20)
	{
		htim3.Init.Prescaler = 749;
	}
	else if(freq<1000)
	{
		htim3.Init.Prescaler = 47;

	}
	else

	{
		htim3.Init.Prescaler = 0;
		
	}
	htim3.Init.Period=48000000/(htim3.Init.Prescaler+1)/freq;
	
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
	
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = htim3.Init.Period*TB6612_PWM_VAL_A/10000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.Pulse = htim3.Init.Period*TB6612_PWM_VAL_B/10000;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  //HAL_TIM_MspPostInit(&htim3);
}


/**/
void Set_PWM(uint8_t pwm_ch, uint16_t Pulse)
{
	uint32_t CH;
	uint32_t pwm_val;
	if(Pulse>10000)
		pwm_val=10000;
	else
		pwm_val=Pulse;
	
	if(pwm_ch==0)
	{
		CH=TIM_CHANNEL_1;
		TB6612_PWM_VAL_A=pwm_val;
	}
	else
	{
		CH=TIM_CHANNEL_2;
		TB6612_PWM_VAL_B=pwm_val;
	}
	
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = htim3.Init.Period * pwm_val / 10000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, CH) != HAL_OK)
  {
    Error_Handler();
  }
	
	if (HAL_TIM_PWM_Stop(&htim3, CH) != HAL_OK)
	{

    Error_Handler();
  }
	
	if (HAL_TIM_PWM_Start(&htim3, CH) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }

}


/*
motor:
0 - Motor A
1 - Motor B

dir:
0x00 - short brake
0x01 - CCW
0x02 - CW
0x03 - Stop
0x04 - Standby

*/
void Set_TB6612_Dir(uint8_t motor, uint8_t dir)
{
	switch(dir)
	{
		case 0x00:
			HAL_GPIO_WritePin(STBY_GPIO_Port,STBY_Pin,GPIO_PIN_SET);
			if(motor==0)
			{			
				HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_SET);
				Set_PWM_IO(0,1);
			}	
			else
			{
				HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_SET);
				Set_PWM_IO(1,1);
			}				
		break;
			
		case 0x01:
			HAL_GPIO_WritePin(STBY_GPIO_Port,STBY_Pin,GPIO_PIN_SET);
			if(motor==0)
			{			
				HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_SET);
				Set_PWM_IO(0,2);
			}	
			else
			{
				HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_SET);
				Set_PWM_IO(1,2);
			}				
		break;
			
		case 0x02:
			HAL_GPIO_WritePin(STBY_GPIO_Port,STBY_Pin,GPIO_PIN_SET);
			if(motor==0)
			{			
				HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_RESET);
				Set_PWM_IO(0,2);
			}	
			else
			{
				HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_RESET);
				Set_PWM_IO(1,2);
			}				
		break;

		case 0x03:
			HAL_GPIO_WritePin(STBY_GPIO_Port,STBY_Pin,GPIO_PIN_SET);
			if(motor==0)
			{			
				HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_RESET);
				Set_PWM_IO(0,1);//IO H
			}	
			else
			{
				HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_RESET);
				Set_PWM_IO(1,1);
			}				
		break;
			
		case 0x04:
			HAL_GPIO_WritePin(STBY_GPIO_Port,STBY_Pin,GPIO_PIN_RESET);	
			Set_PWM_IO(2,1);
		break;
		
		default:
		
		break;
	}
}

/*
ch:
0 - PWMA
1 - PWMB
2 - PWMA&PWMB

mode:
0 - output L
1 - output H
2 - PWM

*/
void Set_PWM_IO(uint8_t ch, uint8_t mode)
{
		GPIO_InitTypeDef GPIO_InitStruct;

		switch(ch)
		{
			case 0:
				GPIO_InitStruct.Pin=PWMA_Pin;
				break;
			
			case 1:
				GPIO_InitStruct.Pin=PWMB_Pin;
				break;
			
			case 2:
				GPIO_InitStruct.Pin = PWMA_Pin|PWMB_Pin;
				break;
		}
    /**TIM3 GPIO Configuration    
    PA6     ------> TIM3_CH1
    PA7     ------> TIM3_CH2 
    */
    
		if(mode<2)
		{
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
			if(mode==0)
				HAL_GPIO_WritePin(GPIOA,GPIO_InitStruct.Pin,GPIO_PIN_RESET);
			else
				HAL_GPIO_WritePin(GPIOA,GPIO_InitStruct.Pin,GPIO_PIN_SET);
		}
		else
		{
				GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
				GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);			
		} 
    
}
