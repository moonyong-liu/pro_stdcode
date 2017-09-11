/**
  ******************************************************************************
  * @file    gpio.c
  * @author  LHw
  * @version V1.0
  * @date    2017/06/08
  * @brief   config gpio
  ******************************************************************************
  * @attention
  ******************************************************************************
  */
#include "stdcode.h"
/**
  * @brief  gpioCfg
	* @param  Pin # GPIO_Pin_x
	* @param	Mode # GPIO_Mode_Out_PP etc.
	* @param 	Speed	#	GPIO_Speed_50MHz etc.
	* @param	sta	#	Initial RESET OR SET
  * @retval SUCCESS/ERROR
  */
u8 gpioCfg(	GPIO_TypeDef* GPIOx,uint16_t Pin,GPIOMode_TypeDef Mode,
						GPIOSpeed_TypeDef Speed,FlagStatus sta)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = Pin;	
	GPIO_InitStructure.GPIO_Mode = Mode;       
	GPIO_InitStructure.GPIO_Speed = Speed;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
	
	if( sta == SET ){
		GPIO_SetBits(GPIOx, Pin);
	}
	else if( sta == RESET ){
		GPIO_ResetBits(GPIOx, Pin);
	}
	else;	
	
	return SUCCESS;
}
