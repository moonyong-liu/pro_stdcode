/**
  ******************************************************************************
  * @file    delay.c
  * @author  LHw
  * @version V1.0
  * @date    2017/06/01
  * @brief   system delay
  ******************************************************************************
  * @attention
	* @updateRecord
	*| who: lhw | when: 20170717 | content:
	*1.create update record
	*2.function declaration moved to stdcode.h
  ******************************************************************************
  */

#include "stdcode.h"

static u8  fac_us=0;//us
static u16 fac_ms=0;//ms

/**
  * @brief  system delay 
  * @param  SYSCLK : HCLK(MHz)
  * @retval None
  */
void delay_init( u8 SYSCLK ) 
{
	SysTick->CTRL &= 0xfffffffb;
	fac_us         = SYSCLK/8; 
	fac_ms         = (u16)fac_us*1000; 
}


/**
  * @brief  delay Millisecond
	* @param  nms : count (MAX =) 
  * @retval None
  */
void delay_ms( u16 nms ) 
{ 
	u32 temp; 

	SysTick->LOAD=(u32)nms*fac_ms;
	SysTick->VAL =0x00; 
	SysTick->CTRL=0x01;

	do 
	{ 
		temp=SysTick->CTRL; 
	} 
	while(temp&0x01&&!(temp&(1<<16)));
	SysTick->CTRL=0x00; 
	SysTick->VAL =0X00; 
}


/**
  * @brief  delay Microsecond
	* @param  nus : count (MAX = )
  * @retval None
  */
void delay_us( u32 nus )
{
	u32 temp;
	SysTick->LOAD=nus*fac_us;
	SysTick->VAL =0x00; 
	SysTick->CTRL=0x01;
	do 
	{ 
		temp=SysTick->CTRL; 
	}
	while(temp&0x01&&!(temp&(1<<16)));
	SysTick->CTRL=0x00; 
	SysTick->VAL =0X00; 
}

