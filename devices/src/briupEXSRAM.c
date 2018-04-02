
#include "common.h"

#define FSMC_BASE ((u32)(0x68000000))

//FSMC��ʼ���ⲿSRAM
void FSMC_SRAM_init(void)
{
	RCC->AHBENR |= 1<<8;		//FSMCʱ��ʹ��
	/*
 ��ַ�ߣ�GPIOF��0~5 . 12~15����GPIOG��0~5����GPIOD��11.12.13�� | �ܹ�19����ַ��
	* �����ߣ�GPIOE��7~15����GPIOD��0.1.8.9.10.14.15��		| �ܹ�16��������
	* �����ߣ�GPIOE��0.1����GPIOD��4.5����GPIOG��10��
	* ��Ҫ���õ�IO�ڣ�
	* GPIOD��0/1/4/5/8-15��	12
	* GPIOE��0/1/7-15��			11
	* GPIOF��0-5/12-15��		10
	* GPIOG��0-5/10��				7
	*/
	RCC->APB2ENR |= 1<<5 | 1<<6 | 1<<7 | 1<<8;
	
	//������IO����Ϊ�����������������ǰ���Ĵ�������
	GPIOD->CRH &= 0x00000000;
	GPIOD->CRH |= 0xBBBBBBBB;
	GPIOD->CRL &= 0xFF00FF00;
	GPIOD->CRL |= 0x00BB00BB;
	
	GPIOE->CRH &= 0x00000000;
	GPIOE->CRH |= 0xBBBBBBBB;
	GPIOE->CRL &= 0x0FFFFF00;
	GPIOE->CRL |= 0xB00000BB;
	
	GPIOF->CRH &= 0x0000FFFF;
	GPIOF->CRH |= 0xBBBB0000;
	GPIOF->CRL &= 0xFF000000;
	GPIOF->CRL |= 0x00BBBBBB;
	
	GPIOG->CRH &= 0xFFFFF0FF;
	GPIOG->CRH |= 0x00000B00;
	GPIOG->CRL &= 0xFF000000;
	GPIOG->CRL |= 0x00BBBBBB;
	
	//�Ĵ�������
	//Bank1����NE1~4��ÿ���˿ڶ���һ��BCR+TCR�����Թ���8���Ĵ���
	//ʹ�õ���NE3������ӦBTCR[4]��BTCR[5]
	FSMC_Bank1->BTCR[4] = 0x00000000;
	FSMC_Bank1->BTCR[5] = 0x00000000;
	FSMC_Bank1E->BWTR[4] = 0x00000000;
	
	
	//����BCR�Ĵ���	ʹ���첽ģʽ��ģʽA����д����һ��ʱ��Ĵ�����
	//BTCR[ż��]��BCR�Ĵ�����BTCR[����]��BTR�Ĵ���
	FSMC_Bank1->BTCR[4] |= 1<< 14;
	FSMC_Bank1->BTCR[4] |= 1<< 12;			//�洢��дʹ��
	FSMC_Bank1->BTCR[4] |= 1<< 4;				//�洢�����ݿ��Ϊ16bit
	FSMC_Bank1->BTCR[4] |= 0<< 2;
	
	/*
	FSMC_Bank1->BTCR[4] |= 1<<12;	//�洢�� д ʹ��
	FSMC_Bank1->BTCR[4] |= 1<<14;	//�� д ʹ�ò�ͬʱ��
	FSMC_Bank1->BTCR[4] |= 1<<4;	//�洢���ݿ��Ϊ16λ
	//д ����
	FSMC_Bank1E->BWTR[4] |= 0<<28;	//MODE A
	FSMC_Bank1E->BWTR[4] |= 0<<0;	//��ַ����ʱ��Ϊ1*HCLK
	*/
	//����BTR�Ĵ���
	FSMC_Bank1->BTCR[5] |= 0<<28;
	FSMC_Bank1->BTCR[5] |= 0x2<<8;			//���ݱ���ʱ��(DATAST) Ϊ4��HCLK 4/72M = 55ns
	FSMC_Bank1->BTCR[5] |= 1<<0;				//��ַ����ʱ��ADDSETΪ1��HCLK
	//����дʱ��Ĵ���
	FSMC_Bank1E->BWTR[4] |= 3<<8;
	//ʹ��BANK1����3
	FSMC_Bank1->BTCR[4] |= 1<<0;
}

