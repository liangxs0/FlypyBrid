/*************************************************************
 * File name: jpstm32_sysclk.c
 * Author: Duke An
 * Version: 0.1
 * Date: 20140617
 * Description: 系统时钟操作源文件
 * Copyright @ 2013 by Briup Technology,Inc. All rights Reserved.
 *************************************************************/
#include "jpstm32_nvic.h"
#include "jpstm32_sysclk.h"


//时钟寄存器初始化
static void rcc_reset(void)
{
	RCC->APB1RSTR	= 0x00000000;
	RCC->APB2RSTR = 0x00000000;
	//睡眠模式闪存和SRAM时钟使能，其他关闭
	RCC->AHBENR 	= 0x00000014;
	//外设时钟关闭
	RCC->APB2ENR 	= 0x00000000;
	RCC->APB1ENR 	= 0x00000000;
	//使能内部高速时钟
	RCC->CR 	|= 0x00000001;
	//复位SW[1:0],HPRE[3:0],PPRE1[2:0],PPRE2[2:0]
	//ADCPRE[1:0],MCO[2:0]	
	RCC->CFGR &= 0xF8FF0000;
	//复位HSEON，CSSON，PLLON
	RCC->CR 	&= 0xFEF6FFFF;
	//复位HSEBYP
	RCC->CR 	&= 0xFFFBFFFF;	 
	//复位PLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE 
	RCC->CFGR &= 0xFF80FFFF;   
	RCC->CIR 	= 0x00000000; 
	//配置异常向量表
	select_nvic_vector(NVIC_VectTab_FLASH, 0x0);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	clk_init
 * Description: 初始化系统时钟
 * Input: 
 *	>PLL, 倍频
 * Output: NULL
 * Return: NULL
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern void clk_init(u8 PLL)
{
	u8	temp = 0;
	rcc_reset();//复位时钟并配置异常向量表
	RCC->CR |= 0x00010000;//外部高速时钟使能
	while(!(RCC->CR>>17));//等待外部时钟就绪
	//HCLK 2分频
	RCC->CFGR = 0x00000400;
	PLL -= 2;
	//PLL 9倍频输出-->72MHz
	RCC->CFGR 	|= PLL<<18;
	//HSE时钟作为PLL输出
	RCC->CFGR 	|= 1<<16;
	
	FLASH->ACR 	|= 0x32;
	//开启倍频
	RCC->CR 		|= 0x01000000;
	//等待PLL锁定
	while(!(RCC->CR>>25));
	//设置PLL为当前系统时钟
	RCC->CFGR |=0x00000002;
	while(temp != 0x02){//等待PLL设置成功
		temp = RCC->CFGR>>2;
		temp &= 0x03;
	}	
}
