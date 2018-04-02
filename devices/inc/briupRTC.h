//briupRTC.h
//����RTC�Ľӿڼ���ض���
#ifndef __RTC_H__
#define __RTC_H__

#include "stm32f10x.h"

//��׼ʱ��ĺ궨��
//��Ҫ������RTCʱ���޸ĳɶ�Ӧ����ʱ��
#define	RTC_BASE_YEAR	2017
#define	RTC_BASE_MONTH	  12
#define	RTC_BASE_DAY	  1
#define	RTC_BASE_WEEK	  5
#define	RTC_BASE_HOUR	  17
#define	RTC_BASE_MINUTE	  20
#define	RTC_BASE_SECOND	  0

//�Զ���RTCʱ�������
typedef	struct rtcTimeDef{
	u16	year;
	u16	month;
	u16	day;
	u16 week;
	u16 hour;
	u16	minute;
	u16 second;
	u32	rtcClock;	//RTC�������е�ֵ
}RTCTimeDef;

//����RTC�Ĵ���ʱ����Ҫȷ��RTC_CRL  RTOFFλΪ1
#define	RTC_WAIT_BUSY()		while( !(RTC->CRL & (1<<5)))

//RTC��ʼ�����ýӿ�
u8	briupRTCInit(void);

//RTCϵͳʱ���ȡ����
u8	briupRTCGetClock(RTCTimeDef * time);

//RTC���Ժ���
void briupRTCTest(void);

#endif

