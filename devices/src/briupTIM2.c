
#include "briupTIM2.h"

//��ʱ��2��ʼ���ӿ�
//���������Ԥ��Ƶϵ��
void briupTIM2Init(u16 psc)
{
	RCC->APB1ENR |= 0x01;
	
	TIM2->PSC = psc-1;
	//ʹ��ʱ��Ϊ���¼���
	TIM2->CR1 |= 1<<4;
	//ʹ�ܶ�ʱ�������ж�
	TIM2->DIER |= 0x01;
	
	//ע�ᶨʱ��2�ж�����
	briupNVICPriorityConfig( 2,1, TIM2_IRQn);
}

//������ʱ������
void briupTIM2Start( u16 arr)
{
	TIM2->ARR = arr;
	TIM2->CR1 |= 0x01;
}

//�رն�ʱ������
void briupTIM2Stop(void)
{
	TIM2->CR1 &= ~0x01;
}

//��д�жϷ�����
static volatile u8 BeepFlag = 0;
void TIM2_IRQHandler()
{
	if(BeepFlag)
	{
		setBeep();
		BeepFlag = 0;
	}else
	{
		resetBeep();
		BeepFlag = 1;
	}
	//����ʱ��2�ĸ����ж�
	TIM2->SR &= ~0x01;
}

