/**
  *****************************************************************************
  * @file    axisfun.c
  * @author  LHw
  * @version V1.0
  * @date    2017/07/24
  * @brief   about all axis function, count, speed, director...
  *****************************************************************************
  * @attention
	* @updateRecord
	1. add saveFreqAxesCap(),axesFreqDetc()
	# who: lhw | when: 20170824 | content:
	1. del speedDetc(),JzFlag,timNvalues,speedDetcStep1
	2. modi saveAxesCap_Vb() -> saveAxesCap()
	# who: lhw | when: 20170818 | content:
	1. modi axesDetc(),add Detection axis success
	# who: lhw | when: 20170817 | content:
	1. add speedDetcStep1_Vb(),speedDetc_Vb(),saveAxesCap_Vb()
	# who: lhw | when: 20170815 | content:
	1.add crc calculate 
	# who: lhw | when: 20170724 | content:
	1.create update record
  *****************************************************************************
  */
/**
  * @brief  Speed detection master func. main() call
	* @param  None
  * @retval SUCCESS/ERROR
  */
#include "stdcode.h"

#define CRCPOLYMSB 0x31 //MSB x8+x5+x4+1
#define CRCPOLYLSB 0x8c //LSB x8+x5+x4+1

/* crc x8+x5+x4+1 0x8c */
static const u8 crcLSBbuff[]={
	0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41, 
	0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 
	0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 
	0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 
	0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 
	0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a, 
	0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 
	0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 
	0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 
	0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 
	0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 
	0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 
	0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 
	0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 
	0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8, 
	0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
};
/* crc x8+x5+x4+1 0x31 */
/*
static const u8 crc_table_msb[]={
	0x00, 0x31, 0x62, 0x53, 0xc4, 0xf5, 0xa6, 0x97, 0xb9, 0x88, 0xdb, 0xea, 0x7d, 0x4c, 0x1f, 0x2e, 
	0x43, 0x72, 0x21, 0x10, 0x87, 0xb6, 0xe5, 0xd4, 0xfa, 0xcb, 0x98, 0xa9, 0x3e, 0x0f, 0x5c, 0x6d, 
	0x86, 0xb7, 0xe4, 0xd5, 0x42, 0x73, 0x20, 0x11, 0x3f, 0x0e, 0x5d, 0x6c, 0xfb, 0xca, 0x99, 0xa8, 
	0xc5, 0xf4, 0xa7, 0x96, 0x01, 0x30, 0x63, 0x52, 0x7c, 0x4d, 0x1e, 0x2f, 0xb8, 0x89, 0xda, 0xeb, 
	0x3d, 0x0c, 0x5f, 0x6e, 0xf9, 0xc8, 0x9b, 0xaa, 0x84, 0xb5, 0xe6, 0xd7, 0x40, 0x71, 0x22, 0x13, 
	0x7e, 0x4f, 0x1c, 0x2d, 0xba, 0x8b, 0xd8, 0xe9, 0xc7, 0xf6, 0xa5, 0x94, 0x03, 0x32, 0x61, 0x50, 
	0xbb, 0x8a, 0xd9, 0xe8, 0x7f, 0x4e, 0x1d, 0x2c, 0x02, 0x33, 0x60, 0x51, 0xc6, 0xf7, 0xa4, 0x95, 
	0xf8, 0xc9, 0x9a, 0xab, 0x3c, 0x0d, 0x5e, 0x6f, 0x41, 0x70, 0x23, 0x12, 0x85, 0xb4, 0xe7, 0xd6, 
	0x7a, 0x4b, 0x18, 0x29, 0xbe, 0x8f, 0xdc, 0xed, 0xc3, 0xf2, 0xa1, 0x90, 0x07, 0x36, 0x65, 0x54, 
	0x39, 0x08, 0x5b, 0x6a, 0xfd, 0xcc, 0x9f, 0xae, 0x80, 0xb1, 0xe2, 0xd3, 0x44, 0x75, 0x26, 0x17, 
	0xfc, 0xcd, 0x9e, 0xaf, 0x38, 0x09, 0x5a, 0x6b, 0x45, 0x74, 0x27, 0x16, 0x81, 0xb0, 0xe3, 0xd2, 
	0xbf, 0x8e, 0xdd, 0xec, 0x7b, 0x4a, 0x19, 0x28, 0x06, 0x37, 0x64, 0x55, 0xc2, 0xf3, 0xa0, 0x91, 
	0x47, 0x76, 0x25, 0x14, 0x83, 0xb2, 0xe1, 0xd0, 0xfe, 0xcf, 0x9c, 0xad, 0x3a, 0x0b, 0x58, 0x69, 
	0x04, 0x35, 0x66, 0x57, 0xc0, 0xf1, 0xa2, 0x93, 0xbd, 0x8c, 0xdf, 0xee, 0x79, 0x48, 0x1b, 0x2a, 
	0xc1, 0xf0, 0xa3, 0x92, 0x05, 0x34, 0x67, 0x56, 0x78, 0x49, 0x1a, 0x2b, 0xbc, 0x8d, 0xde, 0xef, 
	0x82, 0xb3, 0xe0, 0xd1, 0x46, 0x77, 0x24, 0x15, 0x3b, 0x0a, 0x59, 0x68, 0xff, 0xce, 0x9d, 0xac
};
*/
/* Global Variables ---------------------------------------------------------*/
/* PRIVATE DEFINITION -------------------------------------------------------*/
static u8   axesDetc(_TimrValus* values,_JZflag* axesFlag);
static u8 axesFreqDetc(_TimrValus* values, _JZflag* axesFlag);
static void axesDetcStep1(_TimrValus* values, u8 magnetic);

/* PUBLIC DECLARATION -------------------------------------------------------*/

/* PUBLIC DEFINITION --------------------------------------------------------*/
/**
  * @brief  Check sum func. a1+a2...+aN = sum(8bit)
	* @param  *srcdata 	# source data address
	* @param 	len				#	length of data
  * @retval sum				# result
  */
u8 checkSum(u8* srcdata, u16 len)
{
	u8 sum = RESET;
	while(len--){
		sum += *srcdata++;
	}
	return sum;
}

/**
  * @brief  reCheck sum func. a1+a2...+(aN-1) = aN(sum) (8bit)
	* @param  *srcdata 	# source data address
	* @param 	len				#	length of data
  * @retval sum				# result
  */
u8 reCheckSum(u8* srcdata, u8 len)
{
	u8 sum = RESET;
	while(--len){
		sum += *srcdata++;
	}
	if( sum == *srcdata ){
		return SUCCESS;
	}
	else{
		return ERROR;
	}
}

/**
  * @brief  calculate CRC-8 LSB
  * @param  value # need calc value
  * @retval crc		#
  */
u8 calcCRC8Lsb(unsigned char value)
{
  unsigned char i, crc;
  crc = 0x00;           // default value
  crc ^= value;

  for (i=8; i>0; --i){
    if (crc & 0x01){
      crc = (crc >> 1) ^ CRCPOLYLSB;
    }
    else{
      crc = (crc >> 1);
    }
  }
  return crc;
}

/**
  * @brief  calculate by buffer CRC-8 LSB
  * @param  value # need calc value
  * @retval crc	  #
  */
u8 calcCRC8BufLsb(u8 *ptr, u8 len)
{
  unsigned char crc=0x00;  //default value
  while(len--){
    crc = crcLSBbuff[crc ^ *ptr++];
  }
  return (crc);
}

/**
  * @brief  RE-calculate by buffer CRC-8 LSB
  * @param  value # need calc value
  * @retval crc	  #
  */
u8 reCalcCRC8BufLsb(u8 *ptr, u8 len)
{
  unsigned char crc=0x00;  //default value
  while(len--){
    crc = crcLSBbuff[crc ^ *ptr++];
  }
  if( crc == *ptr ){
		return SUCCESS;
  }
  else{
		return ERROR;
  }
}

/**
  * @brief  Saving Axes(capture mode) information, 
						sign the axes detect successful or not
	* @param  *data	   # applicatioin call _Package*
  * @param  sysENtim # exit axsi detec system delay callback func
  * @param  timNval  # two channel A B, select one to save infomation
  * @param  axesFlag # about axes infomation
  * @param  timAVal  # one timer(capture magnetic trigger) for count axes
	* @param  timBVal  # aother timer(capture magnetic trigger) for count axes
  * @retval SUCCESS/ERROR
  */
u8 saveAxesCap( _pClrTimFlgAxisNs sysENtim,_CheckGroup* timNval,_JZflag* axesFlag,
								 _TimrValus* timAVal, _TimrValus* timBVal )
{
	/* check input group */
	if( timNval->MutexSign == RESET ){
			/* check enter */
		if( timAVal != NULL && timAVal->Pair != RESET ){
			/* Enter speed detection mode */
			sysENtim();
			axesDetc(timAVal,axesFlag);
			/* add count axis information at here */
			//axisInfo();
			/* set first activate input */
			timNval->pTimVal = timAVal;
			/* mutex sign */
			timNval->MutexSign = SET;
		}
		else if( timBVal != NULL && timBVal->Pair != RESET ){
			/* Enter speed detection mode */
			sysENtim();
			axesDetc(timBVal,axesFlag);
			/* add count axis information at here */
			//axisInfo();
			/* set first activate input */
			timNval->pTimVal = timBVal;
			/* mutex sign */
			timNval->MutexSign = SET;
		}
	}
	else{
			/* check enter */
		if( timNval->pTimVal->Pair != RESET ){
			/* Enter speed detection mode */
			sysENtim();
			axesDetc(timNval->pTimVal,axesFlag);
			/* add count axis information at here */
			//axisInfo();
		}
		/* According Lock status collection write data */
		if( timNval->pTimVal->DetecFlag ){
			return timNval->pTimVal->DetecFlag;
		}
		else{			
		}
	}
	return ERROR;
}

/**
  * @brief  Saving Axes(capture mode) information, 
						sign the axes detect successful or not
	* @param  *data	   # applicatioin call _Package*
  * @param  sysENtim # exit axsi detec system delay callback func
  * @param  timNval  # two channel A B, select one to save infomation
  * @param  axesFlag # about axes infomation
  * @param  timAVal  # one timer(capture magnetic trigger) for count axes
	* @param  timBVal  # aother timer(capture magnetic trigger) for count axes
  * @retval SUCCESS/ERROR
  */
u8 saveFreqAxesCap( _pClrTimFlgAxisNs sysENtim,_CheckGroup* timNval,
										_JZflag* axesAFlag,_JZflag* axesBFlag,
										_TimrValus* timAVal, _TimrValus* timBVal )
{
	/* check enter */
	if( timAVal->Pair != RESET ){
		/* Enter speed detection mode */
		sysENtim();
		axesFreqDetc(timAVal,axesAFlag);
	}
	if( timBVal->Pair != RESET ){
		/* Enter speed detection mode */
		sysENtim();
		axesFreqDetc(timBVal,axesBFlag);
	}
	/* According Lock status collection write data */
	if( timAVal->DetecFlag ){
		timNval->MutexSign |= DETECFLG_AXES;
	}
	if( timBVal->DetecFlag ){
		timNval->MutexSign |= DETECFLG_SUCC;
	}
	return timNval->MutexSign;
}

/**
  * @brief  count Axes quaitily,Axis feature,Axis proportion
	* @param  pPack # send package
  * @param  
  * @retval SUCCESS/ERROR
  */
u8 countAxesProperty(_Package* pPack, _TimrValus* values, _JZflag* flag)
{
	u16 time_axle1,speed_detect;
	u32 axle_len0,axle_len1;
	
	time_axle1 = values->IntervalsAB + values->IntervalsBA;
	
	if((flag->LastAxesInterTim==0)&&(time_axle1>0)){	//短轴 第一次轴
		flag->LastAxesInterTim = time_axle1;
		flag->LastMagnInterTim = values->Intervals;
		pPack->JzkPack.AxisFea = 0x0C;
		pPack->JzkPack.AxisPro = 0;
		return ERROR;
	}	

	if(flag->LastMagnInterTim == 0){
		axle_len0 = flag->LastAxesInterTim;
		axle_len1 = time_axle1; 
		
	}else{
		speed_detect = MAGNATICINTER/flag->LastMagnInterTim;	//上次值
		axle_len0 = flag->LastAxesInterTim * speed_detect;

		speed_detect = MAGNATICINTER/values->Intervals;			//当前值
		axle_len1 = time_axle1 * speed_detect; 
	}

	flag->LastAxesInterTim = time_axle1; 					//备份数据
	flag->LastMagnInterTim = values->Intervals;

	if(axle_len1>axle_len0)
	{
		if(13 >= axle_len1/(axle_len0/10))
		{
			pPack->JzkPack.AxisFea = pPack->JzkPack.AxisFea+1;
			pPack->JzkPack.AxisPro = axle_len1/(axle_len0/10);	
		}			
		else
		{
			if(1==pPack->JzkPack.AxisFea/10)	
			{
				pPack->JzkPack.AxisFea = 21;
				pPack->JzkPack.AxisPro = axle_len1/(axle_len0/10);
			}
			else	
			{
				pPack->JzkPack.AxisFea = 11;
				pPack->JzkPack.AxisPro = axle_len1/(axle_len0/10);		
			}
		}  
		if(0 == axle_len0) 
		{
			pPack->JzkPack.AxisFea = 12;
			pPack->JzkPack.AxisPro = axle_len1/(axle_len0/10);			
		}
	}
	else
	{
		if(13 >= axle_len0/(axle_len1/10))
		{
			pPack->JzkPack.AxisFea =pPack->JzkPack.AxisFea+1;
			pPack->JzkPack.AxisPro = axle_len0/(axle_len1/10);
		}
		else
		{
			pPack->JzkPack.AxisFea = pPack->JzkPack.AxisFea+1;
			pPack->JzkPack.AxisPro = axle_len0/(axle_len1/10);			
		}
	}
	if(1 == pPack->JzkPack.AxisFea/10)	flag->AxisFeaflag = pPack->JzkPack.AxisFea%10;
	if(2 == pPack->JzkPack.AxisFea/10)	
	{
		if(flag->AxisFeaflag < pPack->JzkPack.AxisFea%10)	pPack->JzkPack.AxisFea = 11;			
	}	
	return SUCCESS;
}	


/* PRIVATE DEFINITION -------------------------------------------------------*/
/**
  * @brief  axesDetc detcetion algorithm
	* @param  values # timer about struction
  * @retval SUCCESS/ERROR
  */
static u8 axesDetc(_TimrValus* values, _JZflag* axesFlag)
{
	/* Capture magnetica steel A activation */
	if( values->Pair == MAGNETICA ){
		if( (values->Lock != MAGNETICB) && (values->Lock > RESET) ){      // if detect Failed
			values->Counter = RESET;
			axesFlag->MissMagnA++;              // Miss magnetic A 
		}
		/* Lock, gettim, counter+1, clear */
		axesDetcStep1(values,MAGNETICA);
	}
	/* Capture magnetica steel B activation */
	else if( values->Pair == MAGNETICB ){
		if( (values->Lock != MAGNETICA) && (values->Lock > RESET)){			// if detect Failed
			values->Counter = RESET;
			axesFlag->MissMagnB++;							// Miss magnetic B
		}
		/* Lock, gettim, counter+1, clear */
		axesDetcStep1(values,MAGNETICB);
	}
	else{		
	}
	/* Detection speed+dir SUCCESS */
	if( values->Counter >= DETCSUCCESS ){
		/* shortAB langBA DIR=A */
		if(values->IntervalsAB < values->IntervalsBA){
			values->Dirc = 'A';
			values->Intervals = values->IntervalsAB;
		}
		/* shortBA langAB DIR=B */
		else if( values->IntervalsBA < values->IntervalsAB ){
			values->Dirc = 'B';
			values->Intervals = values->IntervalsBA;
		}
		else{
			values->Intervals = 0xffff;           // MAX intervals
		}
		values->DetecFlag |= DETECFLG_SUCC;
		values->Counter = SET;  // counter from 1 begin
	}
	/* Detection axis success */
	/* just detect AB or BA magnetic interval time */
	else if( values->Counter >= DETECAXES ){
		values->Intervals = values->IntervalsAB ? values->IntervalsAB :
												values->IntervalsBA ? values->IntervalsBA :
												RESET;		
		/* Statistics Axes quantity */
		axesFlag->CountAxis++;
		values->DetecFlag |= DETECFLG_AXES;
	}
	/* Detectioin lost axes */
	/* Single axis speed detec */
	else if(0){
		
	}
	return SUCCESS;
}

/**
  * @brief  axesDetc detcetion algorithm
	* @param  values # timer about struction
  * @retval SUCCESS/ERROR
  */
static u8 axesFreqDetc(_TimrValus* values, _JZflag* axesFlag)
{
	/* Capture magnetica steel A activation */
	if( values->Pair == MAGNETICA ){
		if( (values->Lock != MAGNETICA) && (values->Lock > RESET) ){      // if detect Failed
			values->Counter = RESET;
			axesFlag->MissMagnA++;              // Miss magnetic A 
		}
		/* Lock, gettim, counter+1, clear */
		axesDetcStep1(values,MAGNETICA);
	}
	else{		
	}
	/* Detection speed+dir SUCCESS */
	if( values->Counter >= DETCSUCCESS ){
		/* shortAB langBA DIR=A */
		if(values->IntervalsAB < values->IntervalsBA){
			values->Dirc = 'A';
			values->Intervals = values->IntervalsAB;
		}
		/* shortBA langAB DIR=B */
		else if( values->IntervalsBA < values->IntervalsAB ){
			values->Dirc = 'B';
			values->Intervals = values->IntervalsBA;
		}
		else{
			values->Intervals = 0xffff;           // MAX intervals
		}
		values->DetecFlag |= DETECFLG_SUCC;
		values->Counter = SET;  // counter from 1 begin
	}
	/* Detection axis success */
	/* just detect AB or BA magnetic interval time */
	else if( values->Counter >= DETECAXES ){
		values->Intervals = values->IntervalsAB ? values->IntervalsAB :
												values->IntervalsBA ? values->IntervalsBA :
												RESET;		
		/* Statistics Axes quantity */
		axesFlag->CountAxis++;
		values->DetecFlag |= DETECFLG_AXES;
	}
	/* Single axis speed detec */
	else if( values->Counter >= DETECAXIS ){
		values->Intervals = values->IntervalsBA;
		axesFlag->CountAxis++;
		values->DetecFlag |= DETECFLG_AXIS;
		values->Counter = RESET;
	}
	/* Detectioin lost axes */	
	else if(0){
		
	}
	return SUCCESS;
}


/**
  * @brief  axesDetcStep1
	* @param  values #  timer values
  * @param  magnetic # MAGNETICA OR MAGNETICB
  * @retval SUCCESS/ERROR
  */

static void axesDetcStep1(_TimrValus* values, u8 magnetic)
{
	u16 tmpinter=RESET;
	tmpinter = tmpinter;										// Shielding warning
	/* getTim, counter+1, clear */
	
	/* first detect */
	if(values->Lock == RESET){
		values->InterCarry = RESET;
		tmpinter = RESET;
	}
	else{
		tmpinter  = TIM_GetCounter(values->TIMx);
		tmpinter |= (values->InterCarry) ? 0xffff : 0;
	}
	/* Lock  */
	values->Lock = magnetic;

	if( magnetic == MAGNETICA ){
		values->IntervalsBA =  tmpinter;
	}
	else if( magnetic == MAGNETICB ){
		values->IntervalsAB =  tmpinter;
	}
	
	values->Counter++;
	TIM_SetCounter(values->TIMx,RESET);	
	values->Pair = RESET;
	values->InterCarry = RESET;	
}


