/***********************************
* FileName:		briupDelay.h
* CreateDate:	2017-8-24
* Author:		Dennis Chen
* Description:	������ʱ�ĳ�ʼ���ӿ�
				����us/ms/s������ʱ�Ľӿ�
* Update:		--------
***********************************/
#ifndef __DELAY_H__
#define __DELAY_H__

#include <stm32f10x.h>

/******************
* FunctionName:	_delay_init( u8 SYSCLK)
* Desciption:	��ʼ��ϵͳʱ�ӵĶ�ʱ��
* Input:		SYSCLK	ϵͳʱ��Ƶ��(MHz)
* Output:		NULL
******************/
void _delay_init( u8 SYSCLK);

//����ϵͳ�δ�ʱ��
void _delay_os_start( void );

//��us��ʱ
void _delay_us( u32 us);
//��ms��ʱ
void _delay_ms( u32 ms);
//��s��ʱ
void _delay_s( double s);

#endif
