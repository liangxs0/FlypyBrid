/*************************************************************
 * File name: jpstm32_nvic.c
 * Author: Duke An
 * Version: 0.1
 * Date: 20140617
 * Description: ϵͳ�ж�����
 * Copyright @ 2013 by Briup Technology,Inc. All rights Reserved.
 *************************************************************/

#include "jpstm32_nvic.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	select_nvic_vector
 * Description: ����ϵͳNVIC�������Ĵ洢λ��
 * Input: 
 *	>NVIC_VectTab, �������洢λ��
 *  >Offset, ƫ����
 * Output: NULL
 * Return: NULL
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void select_nvic_vector(u32 NVIC_VectTab, u32 Offset)
{
	//����NVIC��������ƫ�ƼĴ���
	//���ڱ�ʶ����������CODE��������RAM��
	SCB->VTOR = NVIC_VectTab|(Offset & (u32)0x1FFFFF80);
}

//ʹ���ж�
bool nvic_set_enable(u8 Channel)
{
    int temp = 0;
    if(IS_NVIC_IRQ_CHANNEL(Channel) == FALSE)//�Ƿ���ڴ��ж�
    {
        return FALSE;
    }
    
    //����ʹ��λ
    NVIC->ISER[Channel/32] |= ENABLE << (Channel%32);
    
    //����Ƿ����óɹ�
    temp = NVIC->ISER[Channel/32];
    temp = temp & (ENABLE << (Channel%32));
    if( (NVIC->ISER[Channel/32] & (ENABLE << (Channel%32)) ) != FALSE )
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

//�����ж�
bool nvic_clear_enable(u8 Channel)
{
    if(IS_NVIC_IRQ_CHANNEL(Channel) == FALSE)//�Ƿ���ڴ��ж�
    {
        return FALSE;
    }
    
    //���ó���λ д1����
    NVIC->ICER[Channel/32] |= ENABLE << (Channel%32);
    
    //����Ƿ����óɹ�
    if( (NVIC->ICER[Channel/32] & (ENABLE << (Channel%32)) ) == DISABLE )
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

//�����ж�
bool nvic_set_pending(u8 Channel)
{
    if(IS_NVIC_IRQ_CHANNEL(Channel) == FALSE)//�Ƿ���ڴ��ж�
    {
        return FALSE;
    }
    
    //��������λ
    NVIC->ISPR[Channel/32] = ENABLE << (Channel%32);
    
    //����Ƿ����óɹ�
    if( (NVIC->ISPR[Channel/32]&(ENABLE<<(Channel%32))) == ENABLE )
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}
//�жϽ���
bool nvic_clear_pending(u8 Channel)
{
    if(IS_NVIC_IRQ_CHANNEL(Channel) == FALSE)//�Ƿ���ڴ��ж�
    {
        return FALSE;
    }
    
    //��������λ
    NVIC->ICPR[Channel/32] = ENABLE << (Channel%32);
    
    //����Ƿ����óɹ�
    if( (NVIC->ICPR[Channel/32]&(ENABLE<<(Channel%32)) ) == DISABLE )
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

//��ȡ��ǰ�жϻ״̬
bool nvic_get_active(u8 Channel)
{
    if(IS_NVIC_IRQ_CHANNEL(Channel) == FALSE)//�Ƿ���ڴ��ж�
    {
        return FALSE;
    }
    
    return( (NVIC->IABR[Channel/32] & (ENABLE<<(Channel%32))) ? TRUE:FALSE);
}

bool nvic_set_group(u8 group)
{
    u32 temp = 0;
    temp = SCB->AIRCR;  //��ȡ��ǰ������
	temp&=0X0000F8FF; //�����ǰ����
    if(group > 4)
    {
        return FALSE;
    }
    temp |= 0x05FA0000;
    temp |= (group+3)<<8;
    SCB->AIRCR = temp;
    if( (((SCB->AIRCR >> 8)&0x07)-3) == group )
    {
        return TRUE;
    }
    return FALSE;
}

bool nvic_set_priority(u8 Channel, u8 PreemptionPriority, u8 SubPriority)
{
    u8 group = 0;
    u8 priority = 0;
    group = (((SCB->AIRCR >> 8)&0x07)-3);
    switch (group)
    {
        case 0:
            //������ռ���ȼ�������16��    �ҵ���Ӧ��
            priority = (PreemptionPriority & 0xf);
            break;
        case 1:
            priority = ( (PreemptionPriority & 0x7)<<1 )|(SubPriority & 0x1);
            break;
        case 2:
            priority = ( (PreemptionPriority & 0x3)<<2 )|(SubPriority & 0x3);
            break;
        case 3:
            priority = ( (PreemptionPriority & 0x1)<<3 )|(SubPriority & 0x7);
            break;
    }    
    NVIC->IPR[Channel>>2] &= ~( (0xf<<4) << ((Channel%4)<<3) );
    NVIC->IPR[Channel>>2] |= ((priority<<4) << ((Channel%4)<<3) );
    return TRUE;
}
vu32 nvic_get_priority(void)
{
    return FALSE;
}

bool unregister_nvic(void)
{
    return FALSE;
}


/**************************************************************
ע���ж����ȼ�
NVIC_PreemptionPriority:��ռ���ȼ�
NVIC_SubPriority       :��Ӧ���ȼ�
NVIC_Channel           :�ж�ͨ����
NVIC_Group             :�жϷ��� 0~4
ע�����ȼ����ܳ����趨����ķ�Χ!�������δ����Ĵ���
�黮��:
��0:0λ��ռ���ȼ�,4λ��Ӧ���ȼ�
��1:1λ��ռ���ȼ�,3λ��Ӧ���ȼ�
��2:2λ��ռ���ȼ�,2λ��Ӧ���ȼ�
��3:3λ��ռ���ȼ�,1λ��Ӧ���ȼ�
��4:4λ��ռ���ȼ�,0λ��Ӧ���ȼ�
NVIC_SubPriority��NVIC_PreemptionPriority��ԭ����,��ֵԽС,Խ����
**************************************************************/
bool register_nvic(u8 nvic_Group, u8 nvic_PreemptionPriority, u8 nvic_SubPriority, u8 nvic_Channel)
{
    bool ret = TRUE;
    ret = nvic_set_group(nvic_Group);
    ret = nvic_set_priority(nvic_Channel, nvic_PreemptionPriority, nvic_SubPriority);
    //ret = nvic_set_enable(nvic_Channel);
    return ret;
}

