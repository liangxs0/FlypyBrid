/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * File name: jpstm32_usart.c
 * Author: Duke An
 * Version: 0.1
 * Date: 2014-06-17
 * Description: USART;灯操作实现
 * Copyright @ 2013 by Briup Technology,Inc. All rights Reserved.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
 
#include "jpstm32_usart.h"

USARTypDef	usart1 = {0};

//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif 

#if EN_USART1_RX   //如果使能了接收
 
void USART1_IRQHandler(void)
{
	u8 res;
	#ifdef OS_CRITICAL_METHOD
		OSIntEnter();
	#endif
	if(USART1->SR & (1<<5))/*判断是否接受到数据*/
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

//获取一个字节的数据
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

//获取一个定长的字符串
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
//初始化IO 串口1
//pclk2:PCLK2时钟频率(Mhz)
//bound:波特率
void usart1_init(u32 pclk2,u32 bound)
{
	float temp;
	u16 mantissa,fraction;
	temp = (float)(pclk2 * 1000000)/(bound * 16);
	mantissa = temp;
	fraction = (temp - mantissa)*16;
	mantissa = (mantissa<<4) + fraction;
	//PORTA 时钟时能
	_BD(RCC->APB2ENR, 2) = 1;
	//USART1 时钟时能
	_BD(RCC->APB2ENR, 14) = 1;
	GPIOA->CRH &= 0xfffff00f;
	GPIOA->CRH |= 0x000008B0;
	//复位USART1, 先置1再清0
	_BD(RCC->APB2RSTR, 14) = 1;
	_BD(RCC->APB2RSTR, 14) = 0;
	
	USART1->BRR = mantissa;
	USART1->CR1 |= 0x200C;
	
#if EN_USART1_RX
	//接收缓冲区初始化
	usart1.rx_seek = 0;
	usart1.rx_size = 0;
	usart1.rx_stat |= RX_BUF_E;
	
	//USART1接收终端处理
	_BD(USART1->CR1, 8) = 1;		//奇偶校验终中断开启
	_BD(USART1->CR1, 5) = 1;		//很重要，RXNIE，当UASRT_SR的RXNE(有数据了，即读数据寄存器非空),将发生对应中断
	register_nvic(0, 10, 4, USART1_IRQChannel);
	nvic_set_enable(USART1_IRQChannel);
#endif
}
