//briupVS1003.h
//���������Ͷ���VS1003���������ӿ�

#ifndef __VS1003_H__
#define __VS1003_H__

#include "common.h"

//PC13 -> DREQ		PE6	->	XRESET
//PF6  -> XDCS		PF7 ->	XCS
#define	VS_DREQ		PCxIn(13)
#define	VS_RESET	PExOut(6)
#define	VS_XDCS		PFxOut(6)
#define	VS_XCS		PFxOut(7)

//VS1003 SPI �������� ������/���٣�
void	briupVsSpeedLow(void);
void	briupVsSpeedHigh(void);

//VS1003 ��ȡ����ӿ�
//	Input:	addr	VS1003�Ĵ����ĵ�ַ
u16	briupVsReadReg( u8	addr);

//VS1003 д����ӿ�
//	Inupt:	addr	VS1003�Ĵ����ĵ�ַ
//			data	Ҫд��Ĵ�����16λ����
void briupVsWriteReg( u8 addr, u16 data);

//VS1003 ��RAM�ӿ�
u16 briupVsReadRam( u16	addr);
//VS1003 дRAM�ӿ�
void briupVsWriteRam( u16 addr, u16 data);

//VS1003 д��Ҫ�������������
void briupVsWriteData( u8 data);

//VS1003Ӳ����λ�������λ�ӿ�
u8 briupVsHardReset(void);
void briupVsSoftReset(void);

//VS1003��ʼ���ӿ�
u8 briupVs1003Init(void);
//VS1003���Ҳ��Խӿ�
void briupVs1003Test(void);

//================================================
//			������ؽӿڴ���
//	��������

void briupVs1003VolSet( u16 vol);

//��ȡ������ʱ��
//	Input: 	�ļ���С
//	Output:	��ǰ������ʱ��
u16	briupVsPlayTime( u32 fsize);

//��ȡ��ǰ����ʱ�䣬����ǰ����ʱ��
u16	briupVsPlayTimeNow(void);

//�����и�
void briupVsRestart(void);

#endif

