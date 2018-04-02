/***********************************
* FileName:		briupUsart.h
* CreateDate:	2017-8-25
* Author:		Dennis Chen
* Description:	���ڳ�ʼ���ӿڡ����ݷ��͡����սӿ�
				������
* Update:		--------
***********************************/

#ifndef __USART_H__
#define __USART_H__

#include <stm32f10x.h>
#include <stdio.h>

/******************
* FunctionName:	briupUsart1Init(u32 bound);
* Desciption:	��ʼ���������躯���ӿ�
* Input:		u32 bound	��Ҫ���õĴ��ڲ�����
* Output:		NULL
******************/
void briupUsart1Init( u32 bound);
//ͨ����������λ������һ�ֽ�����
void usart1PutChar( u8 ch);
//ͨ����������λ������һ���ַ���
void usart1PutStr( u8 * str);
//ͨ�����ڽ���һ�ֽ�����
u8 usart1GetChar( void);

#endif
