/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * File name: jpstm32_usart.c
 * Author: Duke An
 * Version: 0.1
 * Date: 2014-06-17
 * Description: USART灯操作实现
 * Copyright @ 2013 by Briup Technology,Inc. All rights Reserved.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifndef __JPSTM32_USART_H__
#define __JPSTM32_USART_H__

#include "jpstm32_common.h"
#include "jpstm32_nvic.h"
#include "stdio.h"



#define USART_REC_LEN  		256  	//定义最大接收字节数 256
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
#define RX_MASK						0x00FF
//接收缓冲区状态标记
#define	RX_BUF_F					0x01
#define RX_BUF_E					0x02
typedef struct {
	s8	rx_buf[USART_REC_LEN];
	u16	rx_seek;
	u16	rx_size;
	u8	rx_stat;
}USARTypDef;
	  	
//如果想串口中断接收，请不要注释以下宏定义
void usart1_init(u32 pclk2,u32 bound);
//获取一个字节的数据
s8	usart1_getc(void);
//获取一个定长的字符串
u8 usart1_gets(s8* buf, u16 len);


#endif
