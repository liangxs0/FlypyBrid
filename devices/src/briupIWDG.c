
#include "common.h"

//�������Ź���ʼ���ӿ�
//	pr  - Ԥ��Ƶϵ��
//	rlr - ��װ��ֵ
// �������Ź�������ʱ��ΪLSI��Ĭ��40KHz��������ȷ
void briupIWDGInit(u16 pr, u16 rlr)
{
	//1.����������LSIʱ��
	RCC->CSR |= 0x01;
	while( !(RCC->CSR & 0x02));
	
	//2.����PR/RLR��д����
	IWDG->KR = 0x5555;
	
	IWDG->PR = pr;
	IWDG->RLR = rlr;
	//ʹ��IWDG������
	IWDG->KR = 0xCCCC;
	IWDG->KR = 0xAAAA;
}

//ִ��һ��ι������
void briupIWDGFeed(void)
{
	IWDG->KR = 0xAAAA;
}

void briupIWDGTest(void)
{
	u8 key = 0;
	resetLed( LED_ALL);
	_delay_ms(1000);
	setLed( LED_G);
	while(1)
	{
		key = getKeyValue();
		if( key == KEY_SEL)
			briupIWDGFeed();
	}
}

