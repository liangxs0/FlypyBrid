/*************************************************************
 * File name: jpstm32_sysclk.h
 * Author: Duke An
 * Version: 0.1
 * Date: 20140617
 * Description: 系统时钟操作头文件
 * Copyright @ 2013 by Briup Technology,Inc. All rights Reserved.
 *************************************************************/

#ifndef __JPSTM32_SYSCLK_H__
#define __JPSTM32_SYSCLK_H__

#include "jpstm32_common.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	clk_init
 * Description: 初始化系统时钟
 * Input: 
 *	>PLL, 倍频
 * Output: NULL
 * Return: NULL
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
 extern void clk_init(u8 pll);

#endif
