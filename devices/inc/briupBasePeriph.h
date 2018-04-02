/***********************************
* FileName:		briupBasePeriph.h
* CreateDate:	2017-8-24
* Author:		Dennis Chen
* Description:	�����������������ʼ���ӿ�
				����غ궨�塢�����ӿ�
* Update:		--------
***********************************/
#ifndef __BASEPERIPH_H__
#define __BASEPERIPH_H__

#include <stm32f10x.h>

//������ɫ λ����
#define	LED_R	(1<<2)
#define	LED_G	(1<<1)
#define	LED_B	(1<<0)
#define	LED_ALL	(0x07)
//�����ɫ
#define LED_Y	( LED_R | LED_G )
#define LED_P	( LED_R | LED_B )
#define LED_C	( LED_B | LED_G )
#define LED_W	( LED_ALL )

/******************
* FunctionName:	basePeriphInit();
* Desciption:	��ʼ���������躯���ӿ�
* Input:		NULL
* Output:		NULL
******************/
void basePeriphInit(void);

/******************
* FunctionName:	setLed(u8 ledStat);
* Desciption:	����ledStat������ӦLED
* Input:		ledStat  �õ�3λ��Ӧ3��LED�Ƶ�״̬
* Output:		NULL
******************/
void setLed(u8 ledStat);
//����ledStatϨ���ӦLED��
void resetLed(u8 ledStat);


void setBeep(void);
void resetBeep(void);

void setRelay(u8 choose);
void resetRelay(u8 choose);

#endif

