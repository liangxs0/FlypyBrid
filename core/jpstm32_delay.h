/*************************************************************
 * File name: jpstm32_delay.h
 * Author: Duke An
 * Version: 0.1
 * Date: 20140617
 * Description: 系统延时相关定义
 * Copyright @ 2013 by Briup Technology,Inc. All rights Reserved.
 *************************************************************/
#ifndef __JPSTM32_DELAY_H__
#define	__JPSTM32_DELAY_H__

#include "jpstm32_common.h"

extern void _delay_init(u8 SYSCLK);
extern void _delay_ms(u16 ms);
extern void _delay_us(u32 us);

#endif
