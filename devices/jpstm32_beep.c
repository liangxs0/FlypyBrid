/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * File name: jpstm32_beep.c
 * Author: Duke An
 * Version: 0.1
 * Date: 2014-06-17
 * Description: 蜂鸣器
 * Copyright @ 2013 by Briup Technology,Inc. All rights Reserved.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "jpstm32_beep.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	beep_init
 * Description: 初始化蜂鸣器
 * Input: NULL
 * Output: NULL
 * Return: 成功返回0 否则返回非0
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern u8 beep_init(void)
{
	//时能PORTG时钟
	_BD(RCC->APB2ENR, 8) = 1;
	//PORTG 14 推挽输出 默认低电平
	config_gpio(PORTG, O_GPPP|OMODE_T50MHz, PinE, PULL_DOWN);
	PGxOut(14) = 0;
	return 0;
}
