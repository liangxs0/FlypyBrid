#include "fsmc.h"
#include "jpstm32_usart.h"

#define FSMC_BASE ((u32)(0x68000000))

//FSMC初始化外部SRAM
void FSMC_SRAM_init(void)
{
	RCC->AHBENR |= 1<<8;		//FSMC时钟使能
	/*
 地址线：GPIOF（0~5 . 12~15）、GPIOG（0~5）、GPIOD（11.12.13） | 总共19跟地址线
	* 数据线：GPIOE（7~15）、GPIOD（0.1.8.9.10.14.15）		| 总共16跟数据线
	* 配置线：GPIOE（0.1）、GPIOD（4.5）、GPIOG（10）
	* 需要配置的IO口：
	* GPIOD（0/1/4/5/8-15）	12
	* GPIOE（0/1/7-15）			11
	* GPIOF（0-5/12-15）		10
	* GPIOG（0-5/10）				7
	*/
	RCC->APB2ENR |= 1<<5 | 1<<6 | 1<<7 | 1<<8;
	
	//将各个IO口置为复用推挽输出，配置前将寄存器清零
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
	
	//寄存器清零
	//Bank1共有NE1~4，每个端口都有一个BCR+TCR，所以共有8个寄存器
	//使用的是NE3，即对应BTCR[4]、BTCR[5]
	FSMC_Bank1->BTCR[4] = 0x00000000;
	FSMC_Bank1->BTCR[5] = 0x00000000;
	FSMC_Bank1E->BWTR[4] = 0x00000000;
	
	
	//操作BCR寄存器	使用异步模式，模式A（读写共用一个时序寄存器）
	//BTCR[偶数]：BCR寄存器；BTCR[奇数]：BTR寄存器
	FSMC_Bank1->BTCR[4] |= 1<< 14;
	FSMC_Bank1->BTCR[4] |= 1<< 12;			//存储器写使能
	FSMC_Bank1->BTCR[4] |= 1<< 4;				//存储器数据宽度为16bit
	FSMC_Bank1->BTCR[4] |= 0<< 2;
	
	/*
	FSMC_Bank1->BTCR[4] |= 1<<12;	//存储器 写 使能
	FSMC_Bank1->BTCR[4] |= 1<<14;	//读 写 使用不同时序
	FSMC_Bank1->BTCR[4] |= 1<<4;	//存储数据宽度为16位
	//写 控制
	FSMC_Bank1E->BWTR[4] |= 0<<28;	//MODE A
	FSMC_Bank1E->BWTR[4] |= 0<<0;	//地址建立时间为1*HCLK
	*/
	//操作BTR寄存器
	FSMC_Bank1->BTCR[5] |= 0<<28;
	FSMC_Bank1->BTCR[5] |= 0x2<<8;			//数据保存时间(DATAST) 为4个HCLK 4/72M = 55ns
	FSMC_Bank1->BTCR[5] |= 1<<0;				//地址建立时间ADDSET为1个HCLK
	//闪存写时序寄存器
	FSMC_Bank1E->BWTR[4] |= 3<<8;
	//使能BANK1区域3
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
		GPIOE->CRL |= 0xb00000bb;		//启用FSMC_NBL0、1
		
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
		
		
		//寄存器清0
		FSMC_Bank1->BTCR[4] = 0x00000000;
		FSMC_Bank1->BTCR[5] = 0x00000000;
		FSMC_Bank1E->BWTR[4] = 0x00000000;
		
		FSMC_Bank1->BTCR[4] |= 1<<12;
		FSMC_Bank1->BTCR[4] |= 1<<4;
		
		FSMC_Bank1->BTCR[5] |= 0xF<<8;
		FSMC_Bank1->BTCR[5] |= 0<<4;
		FSMC_Bank1->BTCR[5] |= 1<<0;
		
		FSMC_Bank1E->BWTR[4] = 0xffffffff;
		FSMC_Bank1->BTCR[4] |= 1<<0;		//使能Bnak1_Sector3
}
//在指定地址(WriteAddr+Bank1_SRAM3_ADDR)开始,连续写入n个字节.
//pBuffer:字节指针
//WriteAddr:要写入的地址
//NumHalfwordToWrite:要写入的字节数
void  FSMC_SRAM_WriteBuffer(u8 *pBuffer,u32 WriteAddr,u32 NumHalfwordToWrite)
{
	while(NumHalfwordToWrite--)
	{
		*(vu8*)(Bank1_SRAM3_ADDR+WriteAddr) = *pBuffer;
		WriteAddr +=2;
		pBuffer++;
	}
}
	
//在指定地址(WriteAddr+Bank1_SRAM3_ADDR)开始,连续读出n个字节.
//pBuffer:字节指针
//ReadAddr:要读出的起始地址
//NumHalfwordToRead:要读出的字节数	
void  FSMC_SRAM_ReadBuffer(u8 *pBuffer,u32 ReadAddr,u32 NumHalfwordToRead)
{
	while(NumHalfwordToRead--)
	{
			*pBuffer++ = *(vu8*)(Bank1_SRAM3_ADDR+ReadAddr);
			ReadAddr+=2;
	}
}
//测试函数
//在指定地址写入1个字节
//addr:地址
//data:要写入的数据
void fsmc_sram_test_write(u32 addr,u8 data)
{
	FSMC_SRAM_WriteBuffer(&data,addr,1);
}
//读取1个字节
//addr:要读取的地址
//返回值:读取到的数据
u8 fsmc_sram_test_read(u32 addr)
{
	u8 data;
	FSMC_SRAM_ReadBuffer(&data,addr,1);
	return data;
}

*/
