/*************************************************************
 * File name: jpstm32_sysclk.c
 * Author: Duke An
 * Version: 0.1
 * Date: 20140617
 * Description: ϵͳʱ�Ӳ���Դ�ļ�
 * Copyright @ 2013 by Briup Technology,Inc. All rights Reserved.
 *************************************************************/
#include "jpstm32_nvic.h"
#include "jpstm32_sysclk.h"


//ʱ�ӼĴ�����ʼ��
static void rcc_reset(void)
{
	RCC->APB1RSTR	= 0x00000000;
	RCC->APB2RSTR = 0x00000000;
	//˯��ģʽ�����SRAMʱ��ʹ�ܣ������ر�
	RCC->AHBENR 	= 0x00000014;
	//����ʱ�ӹر�
	RCC->APB2ENR 	= 0x00000000;
	RCC->APB1ENR 	= 0x00000000;
	//ʹ���ڲ�����ʱ��
	RCC->CR 	|= 0x00000001;
	//��λSW[1:0],HPRE[3:0],PPRE1[2:0],PPRE2[2:0]
	//ADCPRE[1:0],MCO[2:0]	
	RCC->CFGR &= 0xF8FF0000;
	//��λHSEON��CSSON��PLLON
	RCC->CR 	&= 0xFEF6FFFF;
	//��λHSEBYP
	RCC->CR 	&= 0xFFFBFFFF;	 
	//��λPLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE 
	RCC->CFGR &= 0xFF80FFFF;   
	RCC->CIR 	= 0x00000000; 
	//�����쳣������
	select_nvic_vector(NVIC_VectTab_FLASH, 0x0);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	clk_init
 * Description: ��ʼ��ϵͳʱ��
 * Input: 
 *	>PLL, ��Ƶ
 * Output: NULL
 * Return: NULL
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern void clk_init(u8 PLL)
{
	u8	temp = 0;
	rcc_reset();//��λʱ�Ӳ������쳣������
	RCC->CR |= 0x00010000;//�ⲿ����ʱ��ʹ��
	while(!(RCC->CR>>17));//�ȴ��ⲿʱ�Ӿ���
	//HCLK 2��Ƶ
	RCC->CFGR = 0x00000400;
	PLL -= 2;
	//PLL 9��Ƶ���-->72MHz
	RCC->CFGR 	|= PLL<<18;
	//HSEʱ����ΪPLL���
	RCC->CFGR 	|= 1<<16;
	
	FLASH->ACR 	|= 0x32;
	//������Ƶ
	RCC->CR 		|= 0x01000000;
	//�ȴ�PLL����
	while(!(RCC->CR>>25));
	//����PLLΪ��ǰϵͳʱ��
	RCC->CFGR |=0x00000002;
	while(temp != 0x02){//�ȴ�PLL���óɹ�
		temp = RCC->CFGR>>2;
		temp &= 0x03;
	}	
}
