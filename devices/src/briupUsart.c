/***********************************
* FileName:		briupUsart.c
* CreateDate:	2017-8-25
* Author:		Dennis Chen
* Description:	���ڳ�ʼ���ӿڡ����ݷ��͡����սӿ�
				��ʵ��
* Update:		--------
***********************************/

#include "briupUsart.h"
#include "briupNVIC.h"

//���ڿ�����ر�printf����ض�����
#if 1

struct __FILE{
	int handle;
};

FILE __stdout;
_sys_exit( int x)
{
	x = x;
}

//fputc����ض������
//ch Ҫ���͵��ַ�
//FILE * f	stdout ��׼�����
int fputc( int ch, FILE * f)
{
	while( !(USART1->SR & 1<<6));
	USART1->DR = ch;
	return ch;
}

#endif

/******************
* FunctionName:	briupUsart1Init(u32 bound);
* Desciption:	��ʼ���������躯���ӿ�
* Input:		u32 bound	��Ҫ���õĴ��ڲ�����
* Output:		NULL
******************/
void briupUsart1Init( u32 bound)
{
	//============�ָ���==================
	//		���㲨���ʼĴ�����ֵ
	double temp = 72000000.0/(16.0*bound);
	u16 mantissa = (u16)temp;
	u16 fraction = (temp - mantissa)*16;
	mantissa = (mantissa<<4) + fraction;
	//====================================
	//		IO�˿�  �Ĵ������ò���
	//ʹ��GPIOA/USART1ʱ�ӿ���
	RCC->APB2ENR |= 1<<2 | 1<<14;
	
	GPIOA->CRH &= 0xfffff00f;
	//Tx -> PA9  ���츴�����
	GPIOA->CRH |= 0x0b<<4;
	
	//Rx -> PA10 ��������
	GPIOA->CRH |= 0x08<<8;
	GPIOA->ODR |= 1<<10;
	//=============================
	//		USART1	�Ĵ�������
	USART1->BRR = mantissa;
	USART1->CR1 = 1<<13 | 1<<3 | 1<<2;
	//=============================
	//		USART1�ж�����
	USART1->CR1 |= 1<<5;	//ʹ�� ���ջ������ǿ��ж�
	briupNVICPriorityConfig( 3, 3, USART1_IRQn);
}

//ͨ����������λ������һ�ֽ�����
void usart1PutChar( u8 ch)
{
	//ͨ��״̬λ���(ǰһ��)�Ƿ������
	while( !(USART1->SR & (1<<6)));
	//�������͵����ݷ��봮�����ݼĴ���
	USART1->DR = ch;
}

//ͨ����������λ������һ���ַ���
void usart1PutStr( u8 * str)
{
	u8 * temp = str;
	while( *temp != 0 && *temp != '\n')
	{
		usart1PutChar(*temp);
		temp ++;
	}
	usart1PutChar('\n');
}

//ͨ�����ڽ���һ�ֽ�����
u8 usart1GetChar( void)
{
	//�жϵ�ǰ���ݼĴ������Ƿ���δ��������
	while( !(USART1->SR & 1<<5));
	//���У��򷵻����ݼĴ����е�ֵ����ȡ���ݼĴ�����
	return USART1->DR;
}

//���ڽ����жϴ�����
void USART1_IRQHandler()
{
	//�ж��Ƿ� �ǽ��ջ������ǿ��������ж�
	if( USART1->SR & (1<<5))	//�����ݼĴ����ǿ�
	{
		printf("From Computer:%c\n",usart1GetChar());
	}
}


