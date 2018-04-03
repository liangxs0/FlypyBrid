#ifndef __FSMC_H__
#define __FSMC_H__


#include "jpstm32_gpio.h"

/*void  FSMC_SRAM_Init(void);
void  FSMC_SRAM_WriteBuffer(u8 *pBuffer,u32 WriteAddr,u32 NumHalfwordToWrite);
void  FSMC_SRAM_ReadBuffer(u8 *pBuffer,u32 ReadAddr,u32 NumHalfwordToWrite);

void fsmc_sram_test_write(u32 addr,u8 data);
u8 fsmc_sram_test_read(u32 addr);
*/


//FSMC初始化外部SRAM
void FSMC_SRAM_init(void);
/*
void FSMC_SRAM_WriteBuffer(u8* pBuffer,u32 WriteAddr,u32 NumHalfwordToWrite);

void FSMC_SRAM_ReadBuffer(u8* pBuffer,u32 ReadAddr,u32 NumHalfwordToRead);
*/
#endif
