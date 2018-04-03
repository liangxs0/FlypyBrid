#include "dma.h"
#include "jpstm32_usart.h"
#include "jpstm32_lcd.h"
#include "jpstm32_delay.h"

extern u16 FlappyBird_Frame[480][320];

u32 DMA1_MEM_LEN = (u32)(480*320/4);//����DMAÿ�����ݴ��͵ĳ���
vu32 DMA2_LCD_LEN = 153600;

//����DMA
void DMA_Init(void)
{
	//����DMA1ʱ��
	RCC->AHBENR |= 1<<1;
	//�ȴ�ʱ���ȶ�
	_delay_ms(5);
	
	//����DMA1�������ַ��洢����ַ
	DMA2_Channel5->CPAR = (u32)FlappyBird_Frame;
	DMA2_Channel5->CMAR =(u32)0X6C000800;
	
	//��λ
	DMA2_Channel5->CCR = 0x00000000;
	
	/* * * * * * 
	*�Ӵ洢����
	*��ͨģʽ
	*�����ַ������ģʽ
	*�洢������ģʽ
	*�������ݿ��Ϊ16λ
	*�洢�����ݿ��Ϊ16λ
	*�е����ȼ�
	*�Ǵ洢�����洢��ģʽ
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

//ʹ��DMA
void DMA_Enable(void)
{
	//��һ��DMA����
	//�ر�DMA����
	DMA2_Channel5->CCR &= ~(1<<0);
	//����ϴεĴ�����ɱ��
//	DMA2->IFCR = 1<<17;
	//�ı�洢����ַ
	DMA2_Channel5->CPAR = (u32)FlappyBird_Frame[0];
	//ָ�����䳤��
	DMA2_Channel5->CNDTR = DMA1_MEM_LEN;
	//����DMA����
	DMA2_Channel5->CCR |= 1<<0;
//	_delay_ms(1);
	//�ȴ�
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
	
	//�ڶ���DMA����
	//����ϴεĴ�����ɱ��
//	DMA2->IFCR = 1<<17;
	DMA2_Channel5->CPAR = (u32)FlappyBird_Frame[120];
	DMA2_Channel5->CCR &= ~(1<<0);
	DMA2_Channel5->CNDTR = DMA1_MEM_LEN;
	DMA2_Channel5->CCR |= 1<<0;
//	_delay_ms(1);
	//�ȴ�
	while(DMA2_Channel5->CNDTR != 0);
	
	DMA2_Channel5->CPAR = (u32)FlappyBird_Frame[240];
	DMA2_Channel5->CCR &= ~(1<<0);
	DMA2_Channel5->CNDTR = DMA1_MEM_LEN;
	DMA2_Channel5->CCR |= 1<<0;
//	_delay_ms(1);
	//�ȴ�
	while(DMA2_Channel5->CNDTR != 0);
	
	DMA2_Channel5->CPAR = (u32)FlappyBird_Frame[360];
	DMA2_Channel5->CCR &= ~(1<<0);
	DMA2_Channel5->CNDTR = DMA1_MEM_LEN;
	DMA2_Channel5->CCR |= 1<<0;
//	_delay_ms(1);
	//�ȴ�
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

