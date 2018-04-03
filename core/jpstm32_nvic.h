/*************************************************************
 * File name: jpstm32_nvic.h
 * Author: Duke An
 * Version: 0.1
 * Date: 20140617
 * Description: ϵͳ�ж�����
 * Copyright @ 2013 by Briup Technology,Inc. All rights Reserved.
 *************************************************************/

#ifndef __JPSTM32_NVIC_H__
#define __JPSTM32_NVIC_H__

#include "jpstm32_common.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	select_nvic_vector
 * Description: ����ϵͳNVIC������Ĵ洢λ��
 * Input: 
 *	>NVIC_VectTab, ������洢λ��
 *  >Offset, ƫ����
 * Output: NULL
 * Return: NULL
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern void select_nvic_vector(u32 NVIC_VectTab, u32 Offset);

extern bool nvic_set_enable(u8 Channel);
extern bool nvic_clear_enable(u8 Channel);
extern bool nvic_set_pending(u8 Channel);
extern bool nvic_clear_pending(u8 Channel);
extern bool nvic_get_active(u8 Channel);
extern bool nvic_set_group(u8 group);
extern bool nvic_set_priority(u8 Channel, u8 PreemptionPriority, u8 SubPriority);
extern vu32 nvic_get_priority(void);

extern bool register_nvic(u8 nvic_Group, u8 nvic_PreemptionPriority, u8 nvic_SubPriority, u8 nvic_Channel);


#endif
