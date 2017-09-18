/**
  ******************************************************************************
  * @file    usart.c
  * @author  LHw
  * @version V1.0
  * @date    2017/06/01
  * @brief   usart init and config standard function code
  ******************************************************************************
  * @attention
	* @updateRecord
	# who: lhw | when: 20170914 | content:
	1. add usartMaintain_VB(),usart_send_VB(),eia485Maintain_VB()
	2. modi usart_send(),usartMaintain();
	# who: lhw | when: 20170913 | content:
	1. modi usart_send() u8* SrcBuf -> const u8* SrcBuf
	# who: lhw | when: 20170825 | content:
	1. add eia485Maintain()
	# who: lhw | when: 20170731 | content:
	1. add usart1 function 
	2. usart3Maintain() -> usartMaintain()
	3. move private declaration to this file(source stdcode.h)
	4. modi usart3_send() -> usart_send, move usart3 buffer to app file
	5. modi usart3_printf() -> usart_printf() move to app file
	*| who: lhw | when: 20170714 | content:
	*1.create update record
	*2.move function declaration to stdcode.h
  ******************************************************************************
  */
#include "stdcode.h"


/* PUBLIC  DECLARATION ------------------------------------------------------*/
/* PRIVATE DECLARATION ------------------------------------------------------*/
/* Global Variables ---------------------------------------------------------*/
/* PUBLIC  DEFINITION -------------------------------------------------------*/

/**
  * @brief  USART send data func.
  * @brief  TarBuf    # target buffer,usually is the DMABuf
	* @param 	SrcBuf    # source buffer
	* @param	Len       # data length
  * @param  indexTxcb # tx buffer index
  * @param  txBuflen  # TX Max buffer length 
  * @retval SUCCESS/ERROR
  * @note 	public use
  */
u8 usart_send( u8* TarBuf, const u8* SrcBuf, u16 Len, u16* indexTxcb, u16 txBuflen )
{
	u16 i=0;
	u16 tmplen;
	tmplen = Len;
	assert_param(IS_U3TX_DATA_LEN_LEGAL(Len));
	while(tmplen--){
		if( *indexTxcb >= txBuflen ){
			*indexTxcb = RESET;           // loop filling
			//*indexTxcb = txBuflen-1;      // throw away
		}
		TarBuf[*indexTxcb] = SrcBuf[i++];
		(*indexTxcb)++;
	}
	return SUCCESS;
}

u8 usart_send_VB( _pBufPointer pBuff, const u8* SrcBuf, u16 Len )
{
	u16 i=0;
	u16 tmplen;
	tmplen = Len;
	assert_param(IS_UXTX_DATA_LEN_LEGAL(Len,pBuff->TxBufLen));
	while(tmplen--){
		if( *pBuff->TxIndex >= pBuff->TxBufLen ){
			*pBuff->TxIndex = RESET;           // loop filling
			//*pBuff->TxIndex = pBuff->TxBufLen-1;      // throw away
		}
		pBuff->TxDmaBuf[*pBuff->TxIndex] = SrcBuf[i++];
		(*pBuff->TxIndex)++;
	}
	return SUCCESS;
}

/**
  * @brief  usart init. function
	* @param  GPIO_Pin_Rx/Tx: GPIO_Pin_x
	* @param  ITdefine: 			USART_IT_PE etc.
	* @param  Flag: 					USART_FLAG_CTS etc.
	* @param  sta: 						ENABLE/DISABLE
  * @retval None
  */
u8 usartCfg( GPIO_TypeDef* GPIOx, USART_TypeDef* USARTx,
						uint16_t GPIO_Pin_Rx, uint16_t GPIO_Pin_Tx,
						uint32_t BaudRate, uint16_t ITdefine,
						uint16_t Flag, FunctionalState sta)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin 		= GPIO_Pin_Rx;
	GPIO_InitStructure.GPIO_Mode	 	= GPIO_Mode_IN_FLOATING;           
	GPIO_Init(GPIOx, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin		 	= GPIO_Pin_Tx;
	GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
 
	USART_InitStructure.USART_BaudRate            = BaudRate;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;        
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No; 
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USARTx, &USART_InitStructure);                           

	USART_ITConfig( USARTx, ITdefine, ENABLE );
	USART_Cmd( USARTx, sta );     
	USART_ClearFlag( USARTx, Flag );
	
	return SUCCESS;
}

/**
  * @brief  usart dma config ; private call
	* @param 	Channel: 			DMA1_Channel1 etc.
	* @param 	PeriInc: 			DMA_peripheral_incremented_mode
	* @param 	DIR: 					DMA_DIR_PeripheralDST
	* @param 	MemInc:	 			DMA_MemoryInc_Enable etc.
	* @param	PeriDataSize:	DMA_PeripheralDataSize_Byte etc.
	* @param	MemDataSize:	DMA_MemoryDataSize_Byte etc.
	* @param	Mode:					DMA_Mode_Circular etc.
	* @param	Prio:					DMA_Priority_High etc.
	* @param	M2M:					DMA_M2M_Disable etc.
	* @param	DMAIT:				DMA_IT_TC etc.
	* @param  Request: 			USART_DMAReq_Tx etc.
	* @param 	sta: 					ENABLE/DISABLE
  * @retval None
  */
u8 usartDmaCfg(	DMA_Channel_TypeDef* Channel, USART_TypeDef* USARTx,
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
		DMA_InitStructure.DMA_DIR 								= DIR;
		DMA_InitStructure.DMA_BufferSize				 	= Buffsize;
		DMA_InitStructure.DMA_PeripheralInc 			= PeriInc;
		DMA_InitStructure.DMA_MemoryInc 					= MemInc;
		DMA_InitStructure.DMA_PeripheralDataSize 	= PeriDataSize;
		DMA_InitStructure.DMA_MemoryDataSize 			= MemDataSize;
		DMA_InitStructure.DMA_Mode 								= Mode;
		DMA_InitStructure.DMA_Priority 						= Prio;
		DMA_InitStructure.DMA_M2M 								= M2M;
		DMA_Init(Channel, &DMA_InitStructure);

		DMA_ITConfig(Channel, DMAIT, ENABLE);

		USART_DMACmd(USARTx, Request, ENABLE);	

		DMA_Cmd(Channel, sta);

		return SUCCESS;
}


/**
  * @brief	USARTx data maintain.
	* @param 	DMAy_Cx# DMA channel
	* @param 	Len    # Data length
  * @retval SUCCESS/ERROR
  */
u8 usartMaintain(DMA_Channel_TypeDef* DMAy_Cx_Tx, DMA_Channel_TypeDef* DMAy_Cx_Rx,
													uint16_t* TxLen, FlagStatus* DMATxFinFlag)
{
	if( DMA_GetCurrDataCounter(DMAy_Cx_Rx) != U3RXBUFLEN ){
		return ERROR;
	}
	if(*DMATxFinFlag == SET){
		*DMATxFinFlag = RESET;
		DMA_Cmd(DMAy_Cx_Tx, DISABLE);
		DMA_SetCurrDataCounter(DMAy_Cx_Tx,*TxLen);
		*TxLen = RESET;
		
		DMA_Cmd(DMAy_Cx_Tx, ENABLE);	
	}
	else{
		return ERROR;
	}
	return SUCCESS;
}
u8 usartMaintain_VB(_pBufPointer pBuff)
{
	if( DMA_GetCurrDataCounter(pBuff->RxDmaCH) != pBuff->RxDmaLen ){
		return ERROR;
	}
	if(*pBuff->DMATxFlg == SET){
		*pBuff->DMATxFlg = RESET;
		DMA_Cmd(pBuff->TxDmaCH, DISABLE);
		DMA_SetCurrDataCounter(pBuff->TxDmaCH,*pBuff->TxIndex);
		*pBuff->TxIndex = RESET;		
		DMA_Cmd(pBuff->TxDmaCH, ENABLE);	
	}
	else{
		return ERROR;
	}
	return SUCCESS;
}

/**
  * @brief	USARTx-EIA485 data maintain.
	* @param 	DMAy_Cx# DMA channel
	* @param 	Len    # Data length
  * @retval SUCCESS/ERROR
  */
u8 eia485Maintain(DMA_Channel_TypeDef* DMAy_Cx_Tx, DMA_Channel_TypeDef* DMAy_Cx_Rx,
													uint16_t* TxLen, FlagStatus* DMATxFinFlag, _pDlyfun dlySwiRT)
{
	if( DMA_GetCurrDataCounter(DMAy_Cx_Rx) != U3RXBUFLEN ){
		return ERROR;
	}
	if(*DMATxFinFlag == SET){
		*DMATxFinFlag = RESET;
		DMA_Cmd(DMAy_Cx_Tx, DISABLE);
		DMA_SetCurrDataCounter(DMAy_Cx_Tx,*TxLen);		
		
		DMA_Cmd(DMAy_Cx_Tx, ENABLE);	
		dlySwiRT(*TxLen);
		*TxLen = RESET;
	}
	else{
		return ERROR;
	}
	return SUCCESS;
}

u8 eia485Maintain_VB(_pBufPointer pBuff, _pDlyfun dlySwiRT)
{
	if( DMA_GetCurrDataCounter(pBuff->RxDmaCH) != pBuff->RxDmaLen ){
		return ERROR;
	}
	if(*pBuff->DMATxFlg == SET){
		*pBuff->DMATxFlg = RESET;
		DMA_Cmd(pBuff->TxDmaCH, DISABLE);
		DMA_SetCurrDataCounter(pBuff->TxDmaCH,*pBuff->TxIndex);
		*pBuff->TxIndex = RESET;		
		DMA_Cmd(pBuff->TxDmaCH, ENABLE);	
	}
	else{
		return ERROR;
	}
	return SUCCESS;
}

/**
  * @brief	Clear USART REG. SR DR
	* @param 	USARTx # USARTx
  * @retval SUCCESS/ERROR
  */
u8 clearUsartSRDR(USART_TypeDef* USARTx)
{
	u16 tmp = RESET;
	tmp = tmp; 						// Shielding warning
	tmp = USARTx->SR;
	tmp = USARTx->DR;
	
	return SUCCESS;
}
/* PRIVATE DEFINITION -------------------------------------------------------*/
