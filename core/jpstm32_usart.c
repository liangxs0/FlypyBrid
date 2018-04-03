/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * File name: jpstm32_usart.c
 * Author: Duke An
 * Version: 0.1
 * Date: 2014-06-17
 * Description: USART;�Ʋ���ʵ��
 * Copyright @ 2013 by Briup Technology,Inc. All rights Reserved.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
 
#include "jpstm32_usart.h"

USARTypDef	usart1 = {0};

//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef�� d in stdio.h. */ 
FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif 

#if EN_USART1_RX   //���ʹ���˽���
 
void USART1_IRQHandler(void)
{
	u8 res;
	#ifdef OS_CRITICAL_METHOD
		OSIntEnter();
	#endif
	if(USART1->SR & (1<<5))/*�ж��Ƿ���ܵ�����*/
	{	
		res = USART1->DR;
		if(!(usart1.rx_stat&RX_BUF_F))
		{
				usart1.rx_buf[usart1.rx_size] = res;
				usart1.rx_size++;
				usart1.rx_size = usart1.rx_size&RX_MASK;
				if(usart1.rx_size == usart1.rx_seek)
				{
					usart1.rx_stat |= RX_BUF_F;
				}
				usart1.rx_stat &= ~RX_BUF_E;
		}
	}
	//printf("%s\r\n", usart1.rx_buf);
	#ifdef OS_CRITICAL_METHOD
		OSIntExit();
	#endif
} 

//��ȡһ���ֽڵ�����
s8	usart1_getc(void)
{
	s8	res;
	if(!(usart1.rx_stat&RX_BUF_E))
	{
		res = usart1.rx_buf[usart1.rx_seek];
		usart1.rx_seek++;
		usart1.rx_seek = usart1.rx_seek&RX_MASK;
		if(usart1.rx_size == usart1.rx_seek)
		{
			usart1.rx_stat |= RX_BUF_E;
		}
		usart1.rx_stat &= ~RX_BUF_F;
	}
	else 
	{
		res = -1;
	}
	return res;
}

//��ȡһ���������ַ���
u8 usart1_gets(s8* buf, u16 len)
{
	u16 i = 0;
	s8 res;
	while(len>1)
	{
		res = usart1_getc();
		if(res!=-1)
		{
			buf[i] = res;
			len--;
			i++;
		}
	}
	buf[i] = 0;
	return 0;
}

#endif										 
//��ʼ��IO ����1
//pclk2:PCLK2ʱ��Ƶ��(Mhz)
//bound:������
void usart1_init(u32 pclk2,u32 bound)
{
	float temp;
	u16 mantissa,fraction;
	temp = (float)(pclk2 * 1000000)/(bound * 16);
	mantissa = temp;
	fraction = (temp - mantissa)*16;
	mantissa = (mantissa<<4) + fraction;
	//PORTA ʱ��ʱ��
	_BD(RCC->APB2ENR, 2) = 1;
	//USART1 ʱ��ʱ��
	_BD(RCC->APB2ENR, 14) = 1;
	GPIOA->CRH &= 0xfffff00f;
	GPIOA->CRH |= 0x000008B0;
	//��λUSART1, ����1����0
	_BD(RCC->APB2RSTR, 14) = 1;
	_BD(RCC->APB2RSTR, 14) = 0;
	
	USART1->BRR = mantissa;
	USART1->CR1 |= 0x200C;
	
#if EN_USART1_RX
	//���ջ�������ʼ��
	usart1.rx_seek = 0;
	usart1.rx_size = 0;
	usart1.rx_stat |= RX_BUF_E;
	
	//USART1�����ն˴���
	_BD(USART1->CR1, 8) = 1;		//��żУ�����жϿ���
	_BD(USART1->CR1, 5) = 1;		//����Ҫ��RXNIE����UASRT_SR��RXNE(�������ˣ��������ݼĴ����ǿ�),��������Ӧ�ж�
	register_nvic(0, 10, 4, USART1_IRQChannel);
	nvic_set_enable(USART1_IRQChannel);
#endif
}
