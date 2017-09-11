/**
  ******************************************************************************
  * @file    extiline.c
  * @author  LHw
  * @version V1.0
  * @date    2017/07/14
  * @brief   exti init. and config, standard function code
  ******************************************************************************
  * @attention
  ******************************************************************************
  * @updateRecord
	*| who: lhw | when: 20170717 | content:
	*1.create update record
	*2.function declaration moved to stdcode.h
	*/

#include "stdcode.h"
						
/* Public functions ---------------------------------------------------------*/

/* Private functions --------------------------------------------------------*/

/**
  * @brief  
	* @param  Pin # GPIO_Pin_x
	* @param	GMode	#	GPIO_Mode_IPD etc.
	*	@param	PortSource	#	GPIO_PortSourceGPIO(A..G)
	* @param	PinSource	#	GPIO_PinSource(0..15)
	* @param	Line	#	EXTI_Line(0..19)
	*	@param	EMode	#	EXTI_Mode_Interrupt etc.
	* @param 	Trigger	#	EXTI_Trigger_Falling
	* @param	sta #	ENABLE/DISABLE
  * @retval SUCCESS/ERROR
  */
u8 extiCfg(	GPIO_TypeDef* GPIOx, uint16_t Pin, GPIOMode_TypeDef GMode,
						uint8_t PortSource, uint8_t PinSource, uint32_t Line, 
						EXTIMode_TypeDef EMode,	EXTITrigger_TypeDef Trigger, 
						FunctionalState sta)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	EXTI_InitTypeDef EXTI_InitStructure;
		
	GPIO_InitStructure.GPIO_Pin 	= Pin;       
	GPIO_InitStructure.GPIO_Mode 	= GMode;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(PortSource, PinSource); 
	
	EXTI_InitStructure.EXTI_Line 		= Line;
	EXTI_InitStructure.EXTI_Mode 		= EMode;
	EXTI_InitStructure.EXTI_Trigger = Trigger;
	EXTI_InitStructure.EXTI_LineCmd = sta;	
	EXTI_Init(&EXTI_InitStructure); 

	return SUCCESS;
}





