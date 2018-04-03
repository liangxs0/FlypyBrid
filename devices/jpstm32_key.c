/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * File name: jpstm32_key.c
 * Author: Duke An
 * Version: 0.1
 * Date: 2014-06-17
 * Description: 五向按键功能实现
 * Copyright @ 2013 by Briup Technology,Inc. All rights Reserved.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "jpstm32_key.h"
#include "jpstm32_gpio.h"
#include "jpstm32_delay.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	key_init
 * Description: 初始化五向按键
 * Input: NULL
 * Output: NULL
 * Return: NULL
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern void key_init(void)
{
	/* * * * * * * * * * * * * * * * * 
	 * PG.6	-->key center 
	 * PG.7	-->key down 
	 * PG.9	-->key left 
	 * PC.4	-->key right 
	 * PG.11-->key up
	 * * * * * * * * * * * * * * * * */

	//PORTC 时钟时能
	_BD(RCC->APB2ENR, 4) = 1;
	//PORTG 时钟时能
	_BD(RCC->APB2ENR, 8) = 1;
	
	//配置按键引脚功能
	config_gpio(PORTC, I_PULLUD|OMODE_RES, Pin4, PULL_UP);
	config_gpio(PORTG, I_PULLUD|OMODE_RES, Pin6|Pin7|Pin9|PinB, PULL_UP);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	key_init
 * Description: 扫描五向按键
 * Input: NULL
 * Output: NULL
 * Return: 如有按键按下返回对应键值 否则返回KEY_NOKEY
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern KEY_VAL key_scan(void)
{
	if((VAL_0==PGxIn(0x06))
		||(VAL_0==PGxIn(0x07))
		||(VAL_0==PGxIn(0x09))
		||(VAL_0==PGxIn(0x0B))
		||(VAL_0==PCxIn(0x04)))
	{
		//软件消抖
		_delay_ms(10);

		if(VAL_0==PGxIn(0x06)) 
		{
			return KEY_CENTER;
		}
		else if (VAL_0==PGxIn(0x07))
		{
			return KEY_DOWN;
		} 
		else if (VAL_0==PGxIn(0x09))
		{
			return KEY_LEFT;
		} 
		else if (VAL_0==PGxIn(0x0b))
		{
			return KEY_UP;
		} 
		else if (VAL_0==PCxIn(0x04))
		{
			return KEY_RIGHT;
		}
	}
	return KEY_NOKEY;
}




