
#include "briupNVIC.h"

static volatile u8 NVIC_Group;	//��������

//NVIC�жϷ����ʼ�� !!ע�⣡�ú���ֻ��ִ��һ�Σ�
void briupNVICGroupInit(u8 group)
{
	//SCB->AIRCR
	//System Control Block	ϵͳ���ƿ�(��)
	u32 temp;
	NVIC_Group = group;
	//ͨ��STM32��Ż�ȡCM3�ں��жϷ����
	group = ~group & 0x07;
	
	temp = SCB->AIRCR;
	temp &= 0x0000f8ff;
	temp |= group << 8;
	temp |= 0x05fa0000;
	SCB->AIRCR = temp;
}

//NVIC�������ȼ�����
//channel -> �ж�Դ  stm32f10x.h �ж����
void briupNVICPriorityConfig( u8 preemptionPriority,\
							u8 subPriority, u8 channel)
{
	//NVIC->IP�Ĵ�����
	//�����жϷ���ָ��4λ��Чλ����ռ���ȼ�λ����...
	u8 tmp;
	//ָ����Чλ������ʱ����
	u8 tmpptr, tmpsub;
	tmpptr = 0x04 - NVIC_Group;
	tmpsub = 0x0f >> NVIC_Group;
	
	tmp = preemptionPriority << tmpptr;
	tmp |= subPriority & tmpsub;
	tmp <<= 4;
	NVIC->IP[channel] = tmp;
	NVIC->ISER[channel>>0x05] |= 1 << (channel & 0x1f);
}

//GPIO�ⲿ�ж���������
//						�˿�		λ		����
void briupExNVICInit( u8 GPIOx, u8 Bitx, u8 TRIM)
{
	//����AFIO->EXTICR�Ĵ������λ��
	u8 exAddr, exOffset;
	exAddr = Bitx/4;
	exOffset = (Bitx%4)*4;
	//����AFIOʱ��
	RCC->APB2ENR |= 0x01;
	//�������IO����ӳ�䵽��ӦEXTI����
	AFIO->EXTICR[exAddr] |= GPIOx << exOffset;
	//����IO���Ŷ�Ӧ�ⲿ�ж����ϵ��ж�����λ
	EXTI->IMR |= 1<<Bitx;
	EXTI->EMR |= 1<<Bitx;
	
	if(TRIM & 0x01)	//�����ش���
		EXTI->RTSR |= 1<<Bitx;
	if(TRIM & 0x02)	//�½��ش���
		EXTI->FTSR |= 1<<Bitx;
}

