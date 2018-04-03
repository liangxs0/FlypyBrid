#ifndef __MYIIC_H_
#define __MYIIC_H_
#include "jpstm32_common.h"


#if 1
//I/O��������
#define SDA_IN()			{GPIOB->CRL &= 0x0fffffff;GPIOB->CRL |= 0x80000000;}
#define SDA_OUT()			{GPIOB->CRL &= 0x0fffffff;GPIOB->CRL |= 0x30000000;}

//IO��������	 
#define IIC_SCL    PBout(6) //SCL
#define IIC_SDA    PBout(7) //SDA	 
#define READ_SDA   PBin(7)  //����SDA 
#endif
#if 0
//I/O��������
#define SDA_IN()			{GPIOC->CRL &= 0xfff0ffff;GPIOC->CRL |= 0x00080000;}
#define SDA_OUT()			{GPIOC->CRL &= 0xfff0ffff;GPIOC->CRL |= 0x00030000;}

//IO��������	 
#define IIC_SCL    PCout(3) //SCL
#define IIC_SDA    PCout(4) //SDA	 
#define READ_SDA   PCin(4)  //����SDA 
#endif
//IIC���в�������
void IIC_Init(void);                 //��ʼ��IIC��IO��				 
void IIC_Start(void);								//����IIC��ʼ�ź�
void IIC_Stop(void);	  						//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);					//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 							//IIC�ȴ�ACK�ź�
void IIC_Ack(void);									//IIC����ACK�ź�
void IIC_NAck(void);								//IIC������ACK�ź�

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  

#endif