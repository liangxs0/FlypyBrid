#ifndef __TIMER_H
#define __TIMER_H
#include "jpstm32_common.h"

//ͨ���ı�TIM3->CCR2��ֵ���ı�ռ�ձȣ��Ӷ�����LED0������
#define LED0_PWM_VAL TIM3->CCR2    
//TIM4 CH1��ΪPWM DAC�����ͨ�� 
#define PWM_DAC_VAL  TIM4->CCR1 

//////////////////////LCD����PWMռ�ձ�����////////////////////////////////////////
#define LCD_BLPWM_VAL   TIM8->CCR2 	//LCD PWM��������

void TIM3_Int_Init(u16 arr,u16 psc);
void TIM3_PWM_Init(u16 arr,u16 psc);
void TIM5_Cap_Init(u16 arr,u16 psc);
void TIM4_PWM_Init(u16 arr,u16 psc);
void TIM6_Int_Init(u16 arr,u16 psc);
void LCD_PWM_Init(void);

#endif



