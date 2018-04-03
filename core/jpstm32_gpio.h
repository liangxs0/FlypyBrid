/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * File name: jpstm32_gpio.h
 * Author: Duke An
 * Version: 0.1
 * Date: 2014-06-17
 * Description: ϵͳͨ��I/O�ӿڶ���
 * Copyright @ 2013 by Briup Technology,Inc. All rights Reserved.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef __JPSTM32_GPIO_H__
#define __JPSTM32_GPIO_H__

#include "jpstm32_common.h"

//����״ֵ̬
typedef enum{
	VAL_0=0, 	/*����ֵΪ�͵�ƽ �߼�0*/
	VAL_1		/*����ֵΪ�ߵ�ƽ �߼�1*/
} PIN_VAL; 
//����������״̬
typedef enum{
	PULL_UP,	/*����������*/
	PULL_DOWN	/*����������*/
} PULL_STAT;

//IO���ܶ���
#define		O_GPPP		0x00	//0B0000	//ͨ���������
#define		O_GPOD		0x04	//0B0100	//ͨ�ÿ�©���
#define		O_AFPP		0x08	//0B1000	//�����������
#define		O_AFOD		0x0c	//0b1100	//���ÿ�©���
#define		I_ANA		0x00	//0B0000	//ģ������
#define		I_FLO		0x04	//0B0100	//��������
#define		I_PULLUD	0x08	//0B1000	//��������������

//���ģʽ����
#define		OMODE_RES			0x00	//0B00	//����
#define		OMODE_T10MHz		0x01	//0B01	//���10MHz���
#define		OMODE_T02MHz		0x02	//0B10	//���2MHz���
#define		OMODE_T50MHz		0x03	//0B11	//���50MHz���

//I/O�ӿڵ�ַ����
#define		GPIO_BASE		(APB2PERIPH_BASE+0x0800)
#define		PORTA		0x0000
#define		PORTB		0x0400
#define		PORTC		0x0800
#define		PORTD		0x0c00
#define		PORTE		0x1000
#define		PORTF		0x1400
#define		PORTG		0x1800

//���Ŷ���
#define		Pin0		1<<0
#define		Pin1		1<<1
#define		Pin2		1<<2
#define		Pin3		1<<3
#define		Pin4		1<<4
#define		Pin5		1<<5
#define		Pin6		1<<6
#define		Pin7		1<<7
#define		Pin8		1<<8
#define		Pin9		1<<9
#define		PinA		1<<10
#define		PinB		1<<11
#define		PinC		1<<12
#define		PinD		1<<13
#define		PinE		1<<14
#define		PinF		1<<15

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	set_gpio_port
 * Description: ����GPIO�˿�
 * Input: 
 *	>PORTx,	�˿����� x=(A B C ...)
 *	>MODE, ���ģʽ����, �����趨�˿����Ƶ�� 
 *	>PINx, ��Ҫ���õ����� Pin0~PinF ���Խ��л����
 *	>PULL_STAT, ���������Ƿ�����
 * Output: NULL
 * Return: NULL
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void config_gpio(u32 PORTx, u32 MODE, u32 PINxs, PULL_STAT stat);
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	set_PINxs
 * Description: ����ĳһI/O�ڵ�ĳ(��)�����ŵ�ֵ
 * Input: 
 *	>PORTx,	�˿����� x=(A B C ...)
 *	>PINx, ��Ҫ���õ����� Pin0~PinF ���Խ��л����
 *	>val, Ҫ���õ�ֵ
 * Output: NULL
 * Return: NULL
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void set_PINxs(u32	PORTx, u32 PINxs, PIN_VAL val);

#endif


