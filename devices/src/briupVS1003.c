
#include "common.h"


//VS1003 SPI �������� ������/���٣�
void	briupVsSpeedLow(void)
{
	briupSpiSpeedSet( SPI_CLK_DIV_32);
}
void	briupVsSpeedHigh(void)
{
	briupSpiSpeedSet( SPI_CLK_DIV_8);
}

//VS1003 ��ȡ����ӿ�
//	Input:	addr	VS1003�Ĵ����ĵ�ַ
u16	briupVsReadReg( u8	addr)
{
	u16	temp = 0;
	//�ж�VS1003���ڷ�æµ״̬
	while( VS_DREQ == 0);
	briupVsSpeedLow();
	VS_XDCS = 1;
	VS_XCS = 0;
	briupSpiWR( 0x03);
	briupSpiWR( addr);
	temp = briupSpiWR( 0xff);
	temp <<= 8;
	temp |= briupSpiWR( 0xff);
	VS_XCS = 1;
	briupVsSpeedHigh();
	return temp;
}

//VS1003 д����ӿ�
//	Inupt:	addr	VS1003�Ĵ����ĵ�ַ
//			data	Ҫд��Ĵ�����16λ����
void briupVsWriteReg( u8 addr, u16 data)
{
	while( VS_DREQ == 0);
	briupVsSpeedLow();
	VS_XDCS = 1;
	VS_XCS = 0;
	briupSpiWR( 0x02);
	briupSpiWR( addr);
	briupSpiWR( data >> 8);
	briupSpiWR( data & 0xff);
	VS_XCS = 1;
	briupVsSpeedHigh();
	
}

//VS1003 ��RAM�ӿ�
u16	briupVsReadRam( u16	addr)
{
	u16 temp = 0;
	//������Ҫ��ȡ��RAM�Ļ���ַ
	briupVsWriteReg( 0x07, addr);
	//ͨ��0x06�Ĵ�����RAM�е����ݶ�ȡ����
	temp = briupVsReadReg( 0x06);
	return temp;
}

//VS1003 дRAM�ӿ�
void briupVsWriteRam( u16 addr, u16 data)
{
	briupVsWriteReg( 0x07, addr);
	briupVsWriteReg( 0x06, data);
}

//VS1003 д��Ҫ�������������
void briupVsWriteData( u8 data)
{
	//�ȴ�VS1003���ڷ�æµ״̬
	while( VS_DREQ == 0);
	VS_XCS = 1;
	VS_XDCS = 0;
	briupSpiWR( data);
	VS_XDCS = 1;
}

//VS1003Ӳ����λ�������λ�ӿ�
u8 briupVsHardReset(void)
{
	u8	count = 0;
	VS_RESET = 0;	//��ʼ��λ����
	_delay_ms(20);
	VS_XDCS = 1;	//������ݽӿ�Ƭѡ
	VS_XCS = 1;		//������ƽӿ�Ƭѡ
	VS_RESET = 1;	//������λ����
	while( VS_DREQ == 0 && count < 100)
	{
		count ++;
		_delay_us(50);
	}
	_delay_ms(20);
	if( count >= 100)
		return 1;
	else
		return 0;
}

void briupVsSoftReset(void)
{
	u8	count = 0;	//��ʱ�˳�
	while( VS_DREQ == 0);
	briupSpiWR( 0xff);
	//���MODE�Ĵ����е�ֵ�����ڳ�ʼֵ0x0800
	//����������λ����
	while( briupVsReadReg(0x00) != 0x0800)
	{
		//ִ��һ�������λ����
		briupVsWriteReg( 0x00, 0x0804);
		_delay_ms(2);
		//�����λ�ɹ������ȡ0x00�ķ���ֵΪ0x0800
		//������ɹ��������ԣ�ֱ����λ50��֮�󣬳�ʱ�˳�
		if( ++count > 50)
			break;
	}
	while( VS_DREQ == 0);
	count = 0;
	//�����λ֮���轫�ڲ�ʱ�ӽ���3��Ƶ
	while( briupVsReadReg( 0x03) != 0x9800)
	{
		briupVsWriteReg( 0x03, 0x9800);
		count ++;
		if( count > 50)
			break;
	}
	_delay_ms(20);
}

//VS1003��ʼ���ӿ�
u8 briupVs1003Init(void)
{
	//��ʼ��PC13/PE6/PF6/PF7��GPIO����
	RCC->APB2ENR |= 1<<4 | 1<<6 | 1<<7;
	
	GPIOC->CRH &= 0xff0fffff;
	GPIOC->CRH |= 0x00800000;
	GPIOC->ODR |= 1<<13;
	
	GPIOE->CRL &= 0xf0ffffff;
	GPIOE->CRL |= 0x03000000;
	GPIOE->ODR |= 1<<6;
	
	GPIOF->CRL &= 0x00ffffff;
	GPIOF->CRL |= 0x33000000;
	GPIOF->ODR |= 3<<6;
	
	briupSpiInit();

	briupVsHardReset();		//Ӳ����λ
	briupVsSoftReset();		//�����λ
	//����/������ǿֵ
	briupVsWriteReg( 0x02, 0x7A7A);
	
	if( briupVsReadReg( 0x01) & (0x03<<4))
		return 0;
	else
		return 1;
	
}
//VS1003���Ҳ��Խӿ�
void briupVs1003Test(void)
{
	u16 temp, i;
	while( VS_DREQ == 0);
	temp = briupVsReadReg(0x00);
	temp |= 1<<5;	//SDI_TESTS��λ
	briupVsWriteReg( 0x00, temp);
	for( i = 0;i < 40; i++)
	{
		while( VS_DREQ == 0);
		VS_XDCS = 0;	//���ʹ������ݽӿ�Ƭѡ
		briupVsWriteData( 0x53);
		briupVsWriteData( 0xEF);
		briupVsWriteData( 0x6E);
		briupVsWriteData( i);
		briupVsWriteData( 0);
		briupVsWriteData( 0);
		briupVsWriteData( 0);
		briupVsWriteData( 0);
		_delay_ms(50);
		briupVsWriteData( 0x45);
		briupVsWriteData( 0x78);
		briupVsWriteData( 0x69);
		briupVsWriteData( 0x74);
		briupVsWriteData( 0);
		briupVsWriteData( 0);
		briupVsWriteData( 0);
		briupVsWriteData( 0);
		VS_XDCS = 1;
		_delay_ms(50);
	}
}
static u16 VOLUME_VAL[16] = {
	0xfefe, 0xfefe, 0xd0d0, 0xb0b0, 0x9090, 0x8080,
	0x7070, 0x6060, 0x5050, 0x4040, 0x3030, 0x2838,
	0x2020,	0x1010, 0x0000, 0x0000
};
void briupVs1003VolSet( u16 vol)
{
	vol &= 0xf;
	briupVsWriteReg( 0x0b, VOLUME_VAL[vol]);
}


//��ȡ������ʱ��
//	Input: 	�ļ���С
//	Output:	��ǰ������ʱ��
static	u16	VS_KBITS[2][16] = 
{
	{0,8,16,24,32,40,48,56,64,80,96,112,128,144,160,0}, 
	{0,32,40,48,56,64,80,96,112,128,160,192,224,256,320,0}
};
u16	briupVsPlayTime( u32 fsize)
{
	u16	HDAT0, HDAT1;
	HDAT1 = briupVsReadReg( 0x09);
	HDAT0 = briupVsReadReg( 0x08);
	
	HDAT1 >>= 3;	//��ID[0:1]�ƶ������λ
	HDAT1 &= 0x03;	//ֻ��ȡID����������������
	
	if( HDAT1 == 3)	HDAT1 = 1;	//���ݲ�ͬID��ѡ�ò�ͬ�ı�����
	else			HDAT1 = 0;	//���ж�������0/1/2��������ͬ
	
	HDAT0 >>= 12;
	//��ʱ,VS_KBITS[HDAT1][HDAT0]����ÿ���Ӵ���λ�ĸ���(Kbit)
	return fsize / (VS_KBITS[HDAT1][HDAT0] * 125);
}

//��ȡ��ǰ����ʱ�䣬����ǰ����ʱ��
u16	briupVsPlayTimeNow(void)
{
	u16 temp;
	temp = briupVsReadReg( 0x04);
	return temp;
}


//�����и�
void briupVsRestart(void)
{
	u16 temp;
	u16 i, j;
	temp=briupVsReadReg(0x00);	//��ȡSPI_MODE������
	temp|=1<<3;					//����SM_CANCELλ
	temp|=1<<2;					//����SM_LAYER12λ,������MP1,MP2
	briupVsWriteReg(0x00,temp);	//����ȡ����ǰ����ָ��
	for(i=0;i<2048;i += 32)			//����2048��0,�ڼ��ȡSM_CANCELλ.���Ϊ0,���ʾ�Ѿ�ȡ���˵�ǰ����
	{
		for( j = 0;j < 32;j++)
		{
			while( VS_DREQ == 0);
			VS_XDCS = 0;
			briupVsWriteData( 0);
			VS_XDCS = 1;
		}
		temp=briupVsReadReg(0x00);	//��ȡSPI_MODE������
 		if((temp&(1<<3))==0)break;	//�ɹ�ȡ����
	}
	if(i<2048)//SM_CANCEL����
	{
		temp=briupVsReadRam(0X1E06)&0xff;//��ȡ����ֽ�
		for(i=0;i<2052;i += 32)
		{
			for( j = 0;j < 32;j++)
			{
				while( VS_DREQ == 0);
				VS_XDCS = 0;
				briupVsWriteData( 0);
				VS_XDCS = 1;
			}
		}   	
	}else briupVsSoftReset();  	//SM_CANCEL���ɹ�,�����,��Ҫ��λ 	  
	temp=briupVsReadReg(0x08); 
    temp+=briupVsReadReg(0x09);
	if(temp)					//��λ,����û�гɹ�ȡ��,��ɱ���,Ӳ��λ
	{
		briupVsHardReset();		   	//Ӳ��λ
		briupVsSoftReset();  		//��λ 
	} 
}

