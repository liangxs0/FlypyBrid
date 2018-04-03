/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * File name: jpstm32_beep.h
 * Author: Duke An
 * Version: 0.1
 * Date: 2014-06-17
 * Description: 蜂鸣器功能定义
 * Copyright @ 2013 by Briup Technology,Inc. All rights Reserved.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "jpstm32_common.h"
#include "jpstm32_gpio.h"
#include "jpstm32_delay.h"
 
 
#ifndef __JPSTM32_BEEP_H__
#define __JPSTM32_BEEP_H__

#define	BEEP_ON()	PGxOut(14) = 1;
#define	BEEP_OFF()	PGxOut(14) = 0;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	beep_init
 * Description: 初始化蜂鸣器
 * Input: NULL
 * Output: NULL
 * Return: 成功返回0 否则返回非0
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern u8 beep_init(void);
 
 
#endif

