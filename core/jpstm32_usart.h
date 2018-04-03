/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * File name: jpstm32_usart.c
 * Author: Duke An
 * Version: 0.1
 * Date: 2014-06-17
 * Description: USART�Ʋ���ʵ��
 * Copyright @ 2013 by Briup Technology,Inc. All rights Reserved.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifndef __JPSTM32_USART_H__
#define __JPSTM32_USART_H__

#include "jpstm32_common.h"
#include "jpstm32_nvic.h"
#include "stdio.h"



#define USART_REC_LEN  		256  	//�����������ֽ��� 256
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
#define RX_MASK						0x00FF
//���ջ�����״̬���
#define	RX_BUF_F					0x01
#define RX_BUF_E					0x02
typedef struct {
	s8	rx_buf[USART_REC_LEN];
	u16	rx_seek;
	u16	rx_size;
	u8	rx_stat;
}USARTypDef;
	  	
//����봮���жϽ��գ��벻Ҫע�����º궨��
void usart1_init(u32 pclk2,u32 bound);
//��ȡһ���ֽڵ�����
s8	usart1_getc(void);
//��ȡһ���������ַ���
u8 usart1_gets(s8* buf, u16 len);


#endif
