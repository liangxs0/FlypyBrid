
#include "common.h"

//RTCʱ�ӷ�Ƶϵ��	32767
#define	RTC_CLK_DIV	0x7FF0

static u8 monthCalc[2][14] = {
	{0,31,28,31,30,31,30,31,31,30,31,30,31,0},
	{0,31,29,31,30,31,30,31,31,30,31,30,31,0}
};

u8 calcLeapYear(u16 year);

//RTC��ʼ�����ýӿ�
u8	briupRTCInit(void)
{
	u8	temp;
	RCC->APB1ENR |= 0x03 << 27;
	PWR->CR	|= 1<<8;		//����д��RTC�ͺ󱸼Ĵ���
	if( BKP->DR25 != 0x0125)
	{
		RCC->BDCR |= 1<<16;		//��λ������
		RCC->BDCR &= ~(1<<16);	//���ܸ�λ����
		RCC->BDCR |= 0x01;		//����LSEʱ��
		//�ȴ�ʱ���ȶ�
		temp = 0;
		while( !(RCC->BDCR & 0x02))
		{
			temp ++;
			_delay_ms(50);
			if( temp >= 100)
				return 1;	//ʱ�ӿ���ʧ��
		}
		RCC->BDCR |= 1<<8 | 1<<15;
		
		RTC_WAIT_BUSY();
		RTC->CRH = 0;
		RTC_WAIT_BUSY();
		RTC->CRL = (1<<4);	//��������λ
		//Ԥ��Ƶ������
		RTC->PRLH = RTC_CLK_DIV >> 15;
		RTC->PRLL = RTC_CLK_DIV & 0xffff;
		//��λRTC��������ֵ
		RTC->CNTH = 0;
		RTC->CNTL = 0;
		
		RTC_WAIT_BUSY();
		//�ر�RTC��������λ
		RTC->CRL &= ~(1<<4);
		RTC_WAIT_BUSY();
		BKP->DR25 = 0x0125;
	}
	return 0;
}

//RTCϵͳʱ���ȡ����
u8	briupRTCGetClock(RTCTimeDef * time)
{
	u32	temp;	//��ȡRTC��������ֵ
	u8	leapYear;
	if( time == (void *)0 )
		return 1;
	memset( time, 0, sizeof( RTCTimeDef));
	
	temp = RTC->CNTH;
	temp <<= 16;
	temp |= RTC->CNTL;
	
	
	
	//��ȡϵͳʱ�������
	time->second = temp % 60 + RTC_BASE_SECOND;
	if( time->second >= 60)
	{
		time->minute ++;
		time->second -= 60;
	}
	temp /= 60;
	
	//��ȡϵͳʱ��ķ�����
	time->minute += temp % 60 + RTC_BASE_MINUTE;
	if( time->minute >= 60)
	{
		time->hour ++;
		time->minute -=	 60;
	}
	temp /= 60;
	
	//��ȡϵͳʱ���Сʱ��
	time->hour += temp % 24 + RTC_BASE_HOUR;
	if( time->hour >= 24)
	{
		time->day ++;
		time->hour -= 24;
	}
	temp /= 24;
	
	time->day += RTC_BASE_DAY + temp;
	time->week = RTC_BASE_WEEK + (time->day-1) % 7;
	time->week = time->week <= 7 ? time->week : time->week - 7;
	time->month += RTC_BASE_MONTH;
	time->year += RTC_BASE_YEAR;
	leapYear = calcLeapYear(time->year);
	while( (leapYear == 0 && time->day > 365) || (leapYear == 1 && time->day > 366))
	{
		switch( leapYear)
		{
			case 0:
				time->day -= 365;
				break;
			case 1:
				time->day -= 366;
				break;
		}
		time->year ++;
		leapYear = calcLeapYear(time->year);
	}
	while( time->day > monthCalc[leapYear][time->month])
	{
		time->day -= monthCalc[leapYear][time->month];
		time->month ++;
		if( time->month > 12)
		{
			time->year ++;
			time->month = 1;
		}
		leapYear = calcLeapYear( time->year);
	}
	
	return 0;
}

//RTC���Ժ���
void briupRTCTest(void)
{
	RTCTimeDef time;
	while(1)
	{
		briupRTCGetClock( &time);
		printf("%4d-%02d-%02d %2d:%2d:%2d ����%d\n", \
			time.year, time.month, time.day,\
			time.hour, time.minute, time.second, time.week);
		_delay_ms(1000);
	}
}

u8 calcLeapYear(u16 year)
{
	return (year % 400 == 0 || (year % 4 == 0&&year % 100 != 0))? 1 : 0;
}

