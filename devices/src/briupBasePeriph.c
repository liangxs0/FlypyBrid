/***********************************
* FileName:		briupBasePeriph.c
* CreateDate:	2017-8-24
* Author:		Dennis Chen
* Description:	�Ի�������ӿڵ�ʵ���ļ�
* Update:		--------
***********************************/
#include "briupBasePeriph.h"


/******************
* FunctionName:	basePeriphInit();
* Desciption:	��ʼ���������躯���ӿ�
* Input:		NULL
* Output:		NULL
******************/
void basePeriphInit(void)
{
	//����GPIOC��ʱ��
	RCC->APB2ENR |= 1<<4 | 1<<8;
	
	//����GPIOC Pin.0/1/2����Ϊ�������50MHz
	GPIOC->CRL &= ~0xffff;
	GPIOC->CRL |= 0x3333;
	
	GPIOG->CRH &= 0xf00fffff;
	GPIOG->CRH |= 0x33<<20;
	
	resetLed( LED_ALL);
	resetBeep();
	resetRelay(0x03);
}

/******************
* FunctionName:	setLed(u8 ledStat);
* Desciption:	����ledStat������ӦLED
* Input:		ledStat  �õ�3λ��Ӧ3��LED�Ƶ�״̬
* Output:		NULL
******************/
void setLed(u8 ledStat)
{
	//ʹGPIOC Pin.0/1/2����͵�ƽ
	//ODR	BSRR	BRR
	GPIOC->ODR &= ~ledStat;
}

//����ledStatϨ���ӦLED��
void resetLed(u8 ledStat)
{
	GPIOC->ODR |= ledStat;
}

void setBeep()
{
	GPIOG->ODR |= 1<<14;
}
void resetBeep()
{
	GPIOG->ODR &= ~(1<<14);
}

void setRelay(u8 choose)
{
	if( choose & 0x01)
		GPIOG->ODR |= 1<<13;
	if( choose & 0x02)
		GPIOC->ODR |= 1<<3;
}
void resetRelay(u8 choose)
{
	if( choose & 0x01)
		GPIOG->ODR &= ~(1<<13);
	if( choose & 0x02)
		GPIOC->ODR &= ~(1<<3);
}
