/***********************************
* FileName:		briupKeyboard.h
* CreateDate:	2017-8-25
* Author:		Dennis Chen
* Description:	��������������ʼ���ӿں���
				�밴����ֵ���ӿں���
* Update:		--------
***********************************/
#ifndef __KEYBOARD_H__
#define __KEYBOARD_H__

#include <stm32f10x.h>

//��  ��  ��  ��  ��
//6   11   7   9   4	bit
//ö�����Ͷ���
typedef  enum{
	KEY_NONE = 0,
	KEY_RGT = 1<<4,
	KEY_SEL = 1<<6,
	KEY_DOW = 1<<7,
	KEY_LFT = 1<<9,
	KEY_UP = 1<<11
}KEY_VALUE;

/******************
* FunctionName:	briupKeyInit(void);
* Desciption:	��ʼ�����򰴼���IO����
* Input:		NULL
* Output:		NULL
******************/
void briupKeyInit(void);

/******************
* FunctionName:	getKeyValue(void);
* Desciption:	��ⰴ���������ذ�����ֵ
* Input:		NULL
* Output:		KEY_VALUE ���ذ�����Ӧ��ֵ
******************/
KEY_VALUE getKeyValue(void);

#endif
