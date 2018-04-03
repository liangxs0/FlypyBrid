/*************************************************************
 * File name: jpstm32_sysclk.h
 * Author: Duke An
 * Version: 0.1
 * Date: 20140617
 * Description: ϵͳʱ�Ӳ���ͷ�ļ�
 * Copyright @ 2013 by Briup Technology,Inc. All rights Reserved.
 *************************************************************/

#ifndef __JPSTM32_SYSCLK_H__
#define __JPSTM32_SYSCLK_H__

#include "jpstm32_common.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	clk_init
 * Description: ��ʼ��ϵͳʱ��
 * Input: 
 *	>PLL, ��Ƶ
 * Output: NULL
 * Return: NULL
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
 extern void clk_init(u8 pll);

#endif
