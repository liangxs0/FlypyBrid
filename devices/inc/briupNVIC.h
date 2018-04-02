
#ifndef __NVIC_H__
#define __NVIC_H__

#include <stm32f10x.h>

typedef enum{
	GPIO_A,
	GPIO_B,
	GPIO_C,
	GPIO_D,
	GPIO_E,
	GPIO_F,
	GPIO_G
}GPIO_x;

//NVIC�жϷ����ʼ�� !!ע�⣡�ú���ֻ��ִ��һ�Σ�
void briupNVICGroupInit(u8 group);

//NVIC�������ȼ�����
void briupNVICPriorityConfig( u8 preemptionPriority,\
							u8 subPriority, u8 channel);

//GPIO�ⲿ�ж���������
//						�˿�		λ		����
void briupExNVICInit( u8 GPIOx, u8 Bitx, u8 TRIM);

#endif
