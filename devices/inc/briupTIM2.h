
#ifndef __TIM2_H__
#define __TIM2_H__

#include <stm32f10x.h>
#include "briupUsart.h"
#include "briupBasePeriph.h"
#include "briupDelay.h"
#include "briupNVIC.h"

//��ʱ��2��ʼ���ӿ�
//���������Ԥ��Ƶϵ��
void briupTIM2Init(u16 psc);

//������ʱ������
void briupTIM2Start( u16 arr);
//�رն�ʱ������
void briupTIM2Stop(void);

#endif
