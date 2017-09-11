/**
  *****************************************************************************
  * @file    wwdg.c
  * @author  LHw
  * @version V1.0
  * @date    2017/06/07
  * @brief   window watch dog, standard function code
  *****************************************************************************
  * @attention
  *****************************************************************************
  *| who: lhw | when: 20170714 | content:
	*1.create update record
	*2.move function declaration to stdcode.h
	*/

#include "stdcode.h"

u8 Feed_Dog;
/**
  * @brief  Watch dog from 4096*8*(0x7f-0x7f+1)/36(910us) 
											  to 4096*8*(0x7f-0x40+1)/36(58.25ms)
	* @param  None
  * @retval None
	* @note # tWWDG = 1/tPCLK1 * 4096 * 2^WDGTB[1:0] * (T[5:0] + 1) (ms)
					# PCLK1 = 36MHz
					# Prescaler WDGTB Min timeout value Max timeout value
						1 				0 		113 us 						7.28 ms
						2 				1 		227 us 						14.56 ms
						4 				2 		455 us 						29.12 ms
						8 				3 		910 us 						58.25 ms
  */
void initWWDG(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
	
	WWDG_SetPrescaler(WWDG_Prescaler_8);
	WWDG_SetWindowValue(0x7f);
	WWDG_Enable(0x7f);
	WWDG_ClearFlag();
	//WWDG_EnableIT();      //(EWI).
}

/**
  * @brief  iWatch dog from   40K/256=156HZ(6.4ms) 
	* @param  None
  * @retval None
	* @note # 5s/6.4MS=781
  */
void initIWDG(void)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_256);     
    IWDG_SetReload(781);
    IWDG_ReloadCounter();
    IWDG_Enable();
}

/**
  * @brief  Early Wakeup interrupt(EWI)
	* @param  None
  * @retval None
  */
void WWDG_IRQHandler(void)
{
// Do something like make log before RESET sys.
	WWDG_ClearFlag();

}
