#include "dma.h"
#include "jpstm32_usart.h"
#include "jpstm32_lcd.h"
#include "jpstm32_delay.h"

extern u16 FlappyBird_Frame[480][320];

u32 DMA1_MEM_LEN = (u32)(480*320/4);//保持DMA每次数据传送的长度
vu32 DMA2_LCD_LEN = 153600;

//配置DMA
void DMA_Init(void)
{
	//开启DMA1时钟
	RCC->AHBENR |= 1<<1;
	//等待时钟稳定
	_delay_ms(5);
	
	//设置DMA1的外设地址与存储器地址
	DMA2_Channel5->CPAR = (u32)FlappyBird_Frame;
	DMA2_Channel5->CMAR =(u32)0X6C000800;
	
	//复位
	DMA2_Channel5->CCR = 0x00000000;
	
	/* * * * * * 
	*从存储器读
	*普通模式
	*外设地址非增量模式
	*存储器增量模式
	*外设数据宽度为16位
	*存储器数据宽度为16位
	*中等优先级
	*非存储器到存储器模式
	* * * * * * */
	DMA2_Channel5->CCR |= 0<<4;
	DMA2_Channel5->CCR |= 0<<5;
	DMA2_Channel5->CCR |= 1<<6;
	DMA2_Channel5->CCR |= 0<<7;
	DMA2_Channel5->CCR |= 1<<8;
	DMA2_Channel5->CCR |= 1<<10;
	DMA2_Channel5->CCR |= 1<<12;
	DMA2_Channel5->CCR |= 1<<14;
	
}


void DMA_LCD(void)
{
	DMA2_Channel5->CCR &= ~(1<<0);
	DMA2_Channel5->CPAR = (u32)FlappyBird_Frame[160];
	DMA2_Channel5->CNDTR = 153600;
	printf("| LEN = %ld|\r\n",DMA2_LCD_LEN);
	DMA2_Channel5->CCR |= 1<<0;
	printf("| DMA2_Channel5 = %ld|\r\n",DMA2_Channel5->CNDTR);
	while(DMA2_Channel5->CNDTR != 0)
		printf("| DMA2_Channel5 = %ld|\r\n",DMA2_Channel5->CNDTR);
}

//使能DMA
void DMA_Enable(void)
{
	//第一次DMA传输
	//关闭DMA传输
	DMA2_Channel5->CCR &= ~(1<<0);
	//清除上次的传输完成标记
//	DMA2->IFCR = 1<<17;
	//改变存储器地址
	DMA2_Channel5->CPAR = (u32)FlappyBird_Frame[0];
	//指定传输长度
	DMA2_Channel5->CNDTR = DMA1_MEM_LEN;
	//开启DMA传输
	DMA2_Channel5->CCR |= 1<<0;
//	_delay_ms(1);
	//等待
	while(DMA2_Channel5->CNDTR != 0);
		/*
	{
		if(DMA2->ISR & 1<< 18)
			printf("DMA_ONE going helf! 1/2!    ");
		if(DMA2->ISR & 1<<19)
			printf("DMA_ONE going error!    ");
		if(DMA2->ISR & 1<<17)
		{
			printf("DMA_ONE going end!    ");
			break;
		}
	}*/
	
	//while((DMA2->ISR & 1<<17) == 0);
	
	//第二次DMA传输
	//清除上次的传输完成标记
//	DMA2->IFCR = 1<<17;
	DMA2_Channel5->CPAR = (u32)FlappyBird_Frame[120];
	DMA2_Channel5->CCR &= ~(1<<0);
	DMA2_Channel5->CNDTR = DMA1_MEM_LEN;
	DMA2_Channel5->CCR |= 1<<0;
//	_delay_ms(1);
	//等待
	while(DMA2_Channel5->CNDTR != 0);
	
	DMA2_Channel5->CPAR = (u32)FlappyBird_Frame[240];
	DMA2_Channel5->CCR &= ~(1<<0);
	DMA2_Channel5->CNDTR = DMA1_MEM_LEN;
	DMA2_Channel5->CCR |= 1<<0;
//	_delay_ms(1);
	//等待
	while(DMA2_Channel5->CNDTR != 0);
	
	DMA2_Channel5->CPAR = (u32)FlappyBird_Frame[360];
	DMA2_Channel5->CCR &= ~(1<<0);
	DMA2_Channel5->CNDTR = DMA1_MEM_LEN;
	DMA2_Channel5->CCR |= 1<<0;
//	_delay_ms(1);
	//等待
	while(DMA2_Channel5->CNDTR != 0);
	
	/*
	{
		if(DMA2->ISR & 1<< 18)
			printf("DMA_TWO going helf! 1/2!    ");
		if(DMA2->ISR & 1<<19)
			printf("DMA_TWO going error!    ");
		if(DMA2->ISR & 1<<17)
		{
			printf("DMA_TWO going end!    ");
			break;
		}
	}*/
	//while((DMA2->ISR & 1<<17) == 0);
	
}

