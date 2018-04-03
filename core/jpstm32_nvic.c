/*************************************************************
 * File name: jpstm32_nvic.c
 * Author: Duke An
 * Version: 0.1
 * Date: 20140617
 * Description: 系统中断设置
 * Copyright @ 2013 by Briup Technology,Inc. All rights Reserved.
 *************************************************************/

#include "jpstm32_nvic.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	select_nvic_vector
 * Description: 设置系统NVIC向量表的存储位置
 * Input: 
 *	>NVIC_VectTab, 向量表存储位置
 *  >Offset, 偏移量
 * Output: NULL
 * Return: NULL
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void select_nvic_vector(u32 NVIC_VectTab, u32 Offset)
{
	//设置NVIC的向量表偏移寄存器
	//用于标识向量表是在CODE区还是在RAM区
	SCB->VTOR = NVIC_VectTab|(Offset & (u32)0x1FFFFF80);
}

//使能中断
bool nvic_set_enable(u8 Channel)
{
    int temp = 0;
    if(IS_NVIC_IRQ_CHANNEL(Channel) == FALSE)//是否存在此中断
    {
        return FALSE;
    }
    
    //设置使能位
    NVIC->ISER[Channel/32] |= ENABLE << (Channel%32);
    
    //检查是否设置成功
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

//除能中断
bool nvic_clear_enable(u8 Channel)
{
    if(IS_NVIC_IRQ_CHANNEL(Channel) == FALSE)//是否存在此中断
    {
        return FALSE;
    }
    
    //设置除能位 写1除能
    NVIC->ICER[Channel/32] |= ENABLE << (Channel%32);
    
    //检查是否设置成功
    if( (NVIC->ICER[Channel/32] & (ENABLE << (Channel%32)) ) == DISABLE )
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

//悬起中断
bool nvic_set_pending(u8 Channel)
{
    if(IS_NVIC_IRQ_CHANNEL(Channel) == FALSE)//是否存在此中断
    {
        return FALSE;
    }
    
    //设置悬起位
    NVIC->ISPR[Channel/32] = ENABLE << (Channel%32);
    
    //检查是否设置成功
    if( (NVIC->ISPR[Channel/32]&(ENABLE<<(Channel%32))) == ENABLE )
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}
//中断解悬
bool nvic_clear_pending(u8 Channel)
{
    if(IS_NVIC_IRQ_CHANNEL(Channel) == FALSE)//是否存在此中断
    {
        return FALSE;
    }
    
    //设置悬起位
    NVIC->ICPR[Channel/32] = ENABLE << (Channel%32);
    
    //检查是否设置成功
    if( (NVIC->ICPR[Channel/32]&(ENABLE<<(Channel%32)) ) == DISABLE )
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

//读取当前中断活动状态
bool nvic_get_active(u8 Channel)
{
    if(IS_NVIC_IRQ_CHANNEL(Channel) == FALSE)//是否存在此中断
    {
        return FALSE;
    }
    
    return( (NVIC->IABR[Channel/32] & (ENABLE<<(Channel%32))) ? TRUE:FALSE);
}

bool nvic_set_group(u8 group)
{
    u32 temp = 0;
    temp = SCB->AIRCR;  //读取先前的设置
	temp&=0X0000F8FF; //清空先前分组
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
            //限制抢占优先级不超过16个    找到对应域
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
注册中断优先级
NVIC_PreemptionPriority:抢占优先级
NVIC_SubPriority       :响应优先级
NVIC_Channel           :中断通道号
NVIC_Group             :中断分组 0~4
注意优先级不能超过设定的组的范围!否则会有未定义的错误
组划分:
组0:0位抢占优先级,4位响应优先级
组1:1位抢占优先级,3位响应优先级
组2:2位抢占优先级,2位响应优先级
组3:3位抢占优先级,1位响应优先级
组4:4位抢占优先级,0位响应优先级
NVIC_SubPriority和NVIC_PreemptionPriority的原则是,数值越小,越优先
**************************************************************/
bool register_nvic(u8 nvic_Group, u8 nvic_PreemptionPriority, u8 nvic_SubPriority, u8 nvic_Channel)
{
    bool ret = TRUE;
    ret = nvic_set_group(nvic_Group);
    ret = nvic_set_priority(nvic_Channel, nvic_PreemptionPriority, nvic_SubPriority);
    //ret = nvic_set_enable(nvic_Channel);
    return ret;
}


