#include "fsmc.h"
#include "jpstm32_usart.h"

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
/*
void FSMC_SRAM_WriteBuffer(u8* pBuffer,u32 WriteAddr,u32 n)
{
	while(n != 0)
	{
		*(vu8*)(FSMC_BASE+WriteAddr) = *pBuffer;
		pBuffer++;
		WriteAddr+=2;
		n--;
	}
}

void FSMC_SRAM_ReadBuffer(u8* pBuffer,u32 ReadAddr,u32 n)
{
	while(n != 0)
	{
		*pBuffer = *(vu8*)(FSMC_BASE+ReadAddr);
		pBuffer++;
		ReadAddr+=2;
		n--;
	}
}
*/
/*
#include "fsmc.h"
#include "jpstm32_usart.h"
#define Bank1_SRAM3_ADDR  ((u32)(0x68000000))
void  FSMC_SRAM_Init(void)
{
		RCC->AHBENR |= 1<<8;
		RCC->APB2ENR |= (1<<5) | (1<<6)  | (1<<7)  | (1<<8);  //D\E\F\G___Enable
		
		//PORTD Push-pull output multiplexing
		GPIOD->CRH &= 0x00000000;					
		GPIOD->CRH |= 0xbbbbbbbb;			//D(0,1,13,14,15)  A(16,17,18)
		GPIOD->CRL &= 0xff00ff00;			//D(2,3)
		GPIOD->CRL |= 0x00bb00bb;
		
		//PORTE Push-pull  output multiplexing
		GPIOE->CRH &= 0x00000000;
		GPIOE->CRH |= 0xbbbbbbbb;		//D(5,6,7,8,9,10,11,12)
		GPIOE->CRL &= 0x0fffff00;		//D(4)
		GPIOE->CRL |= 0xb00000bb;		//����FSMC_NBL0��1
		
		//PORTF  Push-pull output multiplexing
		GPIOF->CRH &= 0x0000ffff;			//A(6,7,8,9)
		GPIOF->CRH |= 0xbbbb0000;
		GPIOF->CRL &= 0xff000000;		//A(0,1,2,3,4,5)
		GPIOF->CRL |= 0x00bbbbbb;
	
		//PORTG  Push-pull output multiplexing
		GPIOG->CRH &= 0xfffff0ff;			//FSMC_NE3_Enable
		GPIOG->CRH |= 0x00000b00;
		GPIOG->CRL &= 0xff000000;			//A(10,11,12,13,14,15)
		GPIOG->CRL |= 0x00bbbbbb;
		
		
		//�Ĵ�����0
		FSMC_Bank1->BTCR[4] = 0x00000000;
		FSMC_Bank1->BTCR[5] = 0x00000000;
		FSMC_Bank1E->BWTR[4] = 0x00000000;
		
		FSMC_Bank1->BTCR[4] |= 1<<12;
		FSMC_Bank1->BTCR[4] |= 1<<4;
		
		FSMC_Bank1->BTCR[5] |= 0xF<<8;
		FSMC_Bank1->BTCR[5] |= 0<<4;
		FSMC_Bank1->BTCR[5] |= 1<<0;
		
		FSMC_Bank1E->BWTR[4] = 0xffffffff;
		FSMC_Bank1->BTCR[4] |= 1<<0;		//ʹ��Bnak1_Sector3
}
//��ָ����ַ(WriteAddr+Bank1_SRAM3_ADDR)��ʼ,����д��n���ֽ�.
//pBuffer:�ֽ�ָ��
//WriteAddr:Ҫд��ĵ�ַ
//NumHalfwordToWrite:Ҫд����ֽ���
void  FSMC_SRAM_WriteBuffer(u8 *pBuffer,u32 WriteAddr,u32 NumHalfwordToWrite)
{
	while(NumHalfwordToWrite--)
	{
		*(vu8*)(Bank1_SRAM3_ADDR+WriteAddr) = *pBuffer;
		WriteAddr +=2;
		pBuffer++;
	}
}
	
//��ָ����ַ(WriteAddr+Bank1_SRAM3_ADDR)��ʼ,��������n���ֽ�.
//pBuffer:�ֽ�ָ��
//ReadAddr:Ҫ��������ʼ��ַ
//NumHalfwordToRead:Ҫ�������ֽ���	
void  FSMC_SRAM_ReadBuffer(u8 *pBuffer,u32 ReadAddr,u32 NumHalfwordToRead)
{
	while(NumHalfwordToRead--)
	{
			*pBuffer++ = *(vu8*)(Bank1_SRAM3_ADDR+ReadAddr);
			ReadAddr+=2;
	}
}
//���Ժ���
//��ָ����ַд��1���ֽ�
//addr:��ַ
//data:Ҫд�������
void fsmc_sram_test_write(u32 addr,u8 data)
{
	FSMC_SRAM_WriteBuffer(&data,addr,1);
}
//��ȡ1���ֽ�
//addr:Ҫ��ȡ�ĵ�ַ
//����ֵ:��ȡ��������
u8 fsmc_sram_test_read(u32 addr)
{
	u8 data;
	FSMC_SRAM_ReadBuffer(&data,addr,1);
	return data;
}

*/
