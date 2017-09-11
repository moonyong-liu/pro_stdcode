/**
  ******************************************************************************
  * @file    nvicfg.c
  * @author  LHw
  * @version V1.0
  * @date    2017/06/01
  * @brief   NVIC config, standard function code
  ******************************************************************************
  * @attention
  ******************************************************************************
  * @updateRecord
	*| who: lhw | when: 20170717 | content:
	*1.create update record
	*2.function declaration moved to stdcode.h
	*/
	
#include "stdcode.h"

/**
  * @brief  NVIC config func.
	* @param  IRQn 		# USART3_IRQn(etc.)
	* @param	Prio		#	Preemption priority.
	* @param	SubPrio	#	sub priority.
  * @retval SUCCESS/ERROR
  */

u8 nviconfig(	uint8_t 	 IRQn, uint8_t         Prio, 
							uint8_t SubPrio, FunctionalState sta )
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel 										= IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= Prio; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 				= SubPrio;
	NVIC_InitStructure.NVIC_IRQChannelCmd 								= sta;
	NVIC_Init(&NVIC_InitStructure);
	
	return SUCCESS;
}







