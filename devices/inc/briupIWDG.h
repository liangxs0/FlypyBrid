//briupIWDG.h
//�������Ź��ӿ������ļ�
#ifndef __IWDG_H__
#define	__IWDG_H__

#include "stm32f10x.h"

//�������Ź���ʼ���ӿ�
//	pr  - Ԥ��Ƶϵ��
//	rlr - ��װ��ֵ
void briupIWDGInit(u16 pr, u16 rlr);

//ִ��һ��ι������
void briupIWDGFeed(void);
//�������Ź����Գ���
void briupIWDGTest(void);

#endif

