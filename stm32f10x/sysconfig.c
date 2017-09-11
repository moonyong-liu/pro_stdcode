/**
  *****************************************************************************
  * @file    sysconfig.c
  * @author  LHw
  * @version V1.0
  * @date    2017/07/31
  * @brief   system config file
  *****************************************************************************
  * @attention
	* @updateRecord
	# who: lhw | when: 20170731 | content:
	1.create update record
  *****************************************************************************
  */
#include "stdcode.h"

_SYSConfig sysProfile;

void initSysProfile(void)
{
	sysProfile.HumbeinMode = RESET;
}


