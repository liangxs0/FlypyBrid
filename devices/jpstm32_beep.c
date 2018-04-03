/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * File name: jpstm32_beep.c
 * Author: Duke An
 * Version: 0.1
 * Date: 2014-06-17
 * Description: ������
 * Copyright @ 2013 by Briup Technology,Inc. All rights Reserved.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "jpstm32_beep.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	beep_init
 * Description: ��ʼ��������
 * Input: NULL
 * Output: NULL
 * Return: �ɹ�����0 ���򷵻ط�0
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern u8 beep_init(void)
{
	//ʱ��PORTGʱ��
	_BD(RCC->APB2ENR, 8) = 1;
	//PORTG 14 ������� Ĭ�ϵ͵�ƽ
	config_gpio(PORTG, O_GPPP|OMODE_T50MHz, PinE, PULL_DOWN);
	PGxOut(14) = 0;
	return 0;
}
