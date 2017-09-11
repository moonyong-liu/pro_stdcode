/**
  *****************************************************************************
  * @file    timer.c
  * @author  LHw
  * @version V1.0
  * @date    2017/06/01
  * @brief   TIMER config and init. standard function code
  *****************************************************************************
  * @attention STM32 APB1 clk,if APB1 prescaler=1  multiple 1 else multiple 2.
  *****************************************************************************
	* @updateRecord
	*| who: lhw | when: 20170717 | content:
	*1.create update record
	*2.function declaration moved to stdcode.h
  */
	

#include "stdcode.h"


/* Global Variables ---------------------------------------------------------*/
/* Public functions ---------------------------------------------------------*/
/* Private functions --------------------------------------------------------*/

/**
  * @brief  General timer config function; private call
						T = (Period+1)/(HCLK/(Pres+1)/Div)
	* @param  Div # 		TIM_CKD_DIV1
	* @param	CMode: 		TIM_CounterMode_Up etc.
	* @param	Flag: 		TIM_FLAG_Update etc.
	* @param	ITsource: TIM_IT_Update etc.
	* @param	sta: 			ENABLE/DISABLE
  * @retval SUCCESS/ERROR
  */
u8 genTimCfg( TIM_TypeDef *TIMx, uint16_t Period, 
							uint16_t 		 Pres, uint16_t Div, 
							uint16_t 		CMode, uint16_t Flag,
							uint16_t ITsource, FunctionalState sta)
{
	
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    
    TIM_DeInit(TIMx);
    TIM_TimeBaseStructure.TIM_Period        = Period;	 
    TIM_TimeBaseStructure.TIM_Prescaler     = Pres;		 
    TIM_TimeBaseStructure.TIM_ClockDivision = Div; 		 
    TIM_TimeBaseStructure.TIM_CounterMode   = CMode;   
    TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIMx, Flag);							    				 
    TIM_ITConfig(TIMx,ITsource,sta);
    TIM_Cmd(TIMx, sta);
	
		return SUCCESS;
}

/**
  * @brief  General timer Capture/compare config function; private call
	* @param  Channel: 	TIM_Channel_1 etc.
	* @param  Polarity: TIM_ICPolarity_Rising etc.
	* @param  Selec:		TIM_ICSelection_DirectTI etc.
	* @param  Pres: 		TIM_ICPSC_DIV1 etc.
	* @param  Filter:		a number between 0x0 and 0xF
	* @param  GPIO_Pin: GPIO_Pin_x 
	* @param  ITsource:	TIM_IT_Update etc.
	* @param  sta: 			ENABLE/DISABLE
	* @param  GPIO_Mode: GPIO_Mode_IPD
	* @param  GPIO_Remap# GPIO_FullRemap_TIM1 (etc.)
  * @retval None
  */
u8 genTimCaMpCfg(	TIM_TypeDef 	 *TIMx, uint16_t Channel,
									uint16_t 		Polarity, uint16_t Selec,
									uint16_t 				Pres, uint16_t Filter,
									GPIO_TypeDef 	*GPIOx, uint16_t GPIO_Pin,
									uint16_t 		ITsource, GPIOMode_TypeDef GPIO_Mode,
									uint32_t  GPIO_Remap,  FunctionalState sta)
{
		TIM_ICInitTypeDef  TIM_ICInitStructure;  
    GPIO_InitTypeDef GPIO_InitStructure;  

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode;
    GPIO_Init(GPIOx, &GPIO_InitStructure);  
    GPIO_ResetBits(GPIOx, GPIO_Pin);
		GPIO_PinRemapConfig(GPIO_Remap,sta);
	
    TIM_ICInitStructure.TIM_Channel 		= Channel;
    TIM_ICInitStructure.TIM_ICPolarity	= Polarity;
    TIM_ICInitStructure.TIM_ICSelection = Selec;
    TIM_ICInitStructure.TIM_ICPrescaler = Pres; 
    TIM_ICInitStructure.TIM_ICFilter 		= Filter;
    TIM_ICInit(TIMx, &TIM_ICInitStructure);  
 
    TIM_ITConfig(TIMx,ITsource, sta);        
		TIM_Cmd(TIMx, sta);
	
		return SUCCESS;
}

/**
  * @brief  General timer Capture/compare config function; private call
	* @param 	Channel: 			DMA1_Channel1 etc.
	* @param 	PeriInc: 			DMA_PeripheralInc_Enable etc.
	* @param 	MemInc:	 			DMA_MemoryInc_Enable etc.
	* @param	PeriDataSize:	DMA_PeripheralDataSize_Byte etc.
	* @param	MemDataSize:	DMA_MemoryDataSize_Byte etc.
	* @param	Mode:					DMA_Mode_Circular etc.
	* @param	Prio:					DMA_Priority_High etc.
	* @param	M2M:					DMA_M2M_Disable etc.
	* @param	DMAIT:				DMA_IT_TC etc.
	* @param  Request: 			TIM_DMA_Update etc.
	* @param 	sta: 					ENABLE/DISABLE
  * @retval None
  */
u8 genTimCaMpDmaCfg(	DMA_Channel_TypeDef* Channel, TIM_TypeDef* TIMx,
																uint32_t PeriAddr, uint16_t Request,
																uint32_t MemAddr, uint32_t DIR,
																uint32_t Buffsize, uint32_t PeriInc,
																uint32_t MemInc, uint32_t PeriDataSize,
																uint32_t MemDataSize, uint32_t Mode,
																uint32_t Prio, uint32_t M2M,
																uint32_t DMAIT, FunctionalState sta )
{
	
		DMA_InitTypeDef DMA_InitStructure;

		DMA_DeInit(Channel);  
		DMA_InitStructure.DMA_PeripheralBaseAddr 	= (uint32_t)PeriAddr;
		DMA_InitStructure.DMA_MemoryBaseAddr 			= (uint32_t)MemAddr;
		DMA_InitStructure.DMA_DIR 								= DMA_DIR_PeripheralDST;
		DMA_InitStructure.DMA_BufferSize				 	= Buffsize;
		DMA_InitStructure.DMA_PeripheralInc 			= PeriInc;
		DMA_InitStructure.DMA_MemoryInc 					= MemInc;
		DMA_InitStructure.DMA_PeripheralDataSize 	= PeriDataSize;
		DMA_InitStructure.DMA_MemoryDataSize 			= MemDataSize;
		DMA_InitStructure.DMA_Mode 								= Mode;
		DMA_InitStructure.DMA_Priority 						= Prio;
		DMA_InitStructure.DMA_M2M 								= M2M;
		DMA_Init(Channel, &DMA_InitStructure);
		
		DMA_ITConfig(Channel, DMAIT, sta);
		/* Enable TIMx DMA request */
		TIM_DMACmd(TIMx, Request, sta);	
	
		DMA_Cmd(Channel, sta);
		
		return SUCCESS; 
}


/*
*---------------------------- IRQ FUNC. ---------------------------------------
*/




