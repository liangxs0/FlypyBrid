/***********************************
* FileName:		briupKeyboard.c
* CreateDate:	2017-8-25
* Author:		Dennis Chen
* Description:	ʵ�ְ�����ʼ�������ͻ�ȡ������ֵ
* Update:		--------
************************************/

#include "briupKeyboard.h"
#include "briupNVIC.h"
#include "briupBasePeriph.h"
#include "briupDelay.h"
#include "briupTIM2.h"

/******************
* FunctionName:	briupKeyInit(void);
* Desciption:	��ʼ�����򰴼���IO����
* Input:		NULL
* Output:		NULL
******************/
void briupKeyInit(void)
{
	//����GPIOC/GPIOG��ʱ��
	//RCC->APB2ENR = RCC->APB2ENR | ( 1<<4 | 1<<8);
	RCC->APB2ENR |= 1<<4 | 1<<8;
	
	//���ð�����Ӧ��IO���ŵ�ģʽ
		//�����λֵ
	//GPIOG Pin.6/7/9/11  ---  GPIOC Pin.4
	GPIOG->CRL &= 0x00ffffff;	//�����Ӧ��������
	GPIOG->CRH &= 0xffff0f0f;
	
	//��������������Ϊ����(��/��������)
	GPIOG->CRL |= 0x88000000;
	GPIOG->CRH |= 0x00008080;
	GPIOG->ODR |= 1<<6 | 1<<7 | 1<<9 | 1<<11;
	
	GPIOC->CRL &= ~(0xf<<16);
	GPIOC->CRL |= 0x08<<16;
	
	GPIOC->ODR |= 1<<4;
	/*
	//select���ж�����
	//����GPIOӳ��ӿڣ�ȷ����Ӧ���ź��ⲿ�ж��ߵ�ӳ��
	briupExNVICInit( GPIO_G, 6, 0x02);	//SELECT
	briupExNVICInit( GPIO_G, 7, 0x02);	//DOWN
	briupExNVICInit( GPIO_G, 9, 0x02);	//LEFT
	//�����ⲿ�ж���6�����ȼ����ж�Դ
	briupNVICPriorityConfig( 3, 1, EXTI9_5_IRQn);
	
	//up���ж�����
	briupExNVICInit( GPIO_G, 11, 0x02);
	briupNVICPriorityConfig( 3, 1, EXTI15_10_IRQn);
	
	briupExNVICInit( GPIO_C, 4, 0x02);	//RIGHT
	briupNVICPriorityConfig( 3, 1, EXTI4_IRQn);*/
}

/******************
* FunctionName:	getKeyValue(void);
* Desciption:	��ⰴ���������ذ�����ֵ
* Input:		NULL
* Output:		KEY_VALUE ���ذ�����Ӧ��ֵ
******************/
KEY_VALUE getKeyValue(void)
{
	KEY_VALUE key = KEY_NONE;
	
	if(!(GPIOG->IDR & KEY_SEL))	//ѡ���
	{
		_delay_ms(4);
		if(!(GPIOG->IDR & KEY_SEL))
			key |= (1<<4);
	}
		
	if(!(GPIOG->IDR & KEY_UP))	//��
	{
		_delay_ms(4);
		if(!(GPIOG->IDR & KEY_UP))
			key |= (1<<0);
	}
	if(!(GPIOG->IDR & KEY_DOW))	//��
	{
		_delay_ms(4);
		if(!(GPIOG->IDR & KEY_DOW))
			key |= (1<<1);
	}
	if(!(GPIOG->IDR & KEY_LFT))	//��
	{
		_delay_ms(4);
		if(!(GPIOG->IDR & KEY_LFT))
			key |= (1<<2);
	}
	if(!(GPIOC->IDR & KEY_RGT))	//��
	{
		_delay_ms(4);
		if(!(GPIOG->IDR & KEY_RGT))
			key |= (1<<3);
	}
	
	return key;
}

//select�жϷ�����
void EXTI9_5_IRQHandler()
{
	//�˿���������Ĵ����ж�
	if( !(GPIOG->IDR & 1<<6))	//SEL
	{
		//�����Ӧ�ⲿ�ж�
		EXTI->PR |= 1<<6;
	}
	if( !(GPIOG->IDR & 1<<7))	//DOWN
	{
		EXTI->PR |= 1<<7;
	}
	if( !(GPIOG->IDR & 1<<9))	//LEFT
	{
		EXTI->PR |= 1<<9;
	}
}

//up���жϷ�����
void EXTI15_10_IRQHandler()
{
	EXTI->PR |= 1<<11;
}

//RIGHT
void EXTI4_IRQHandler()
{
	EXTI->PR |= 1<<4;
}




