#include "timer.h"
#include "led.h"
#include "jpstm32_usart.h"

//��ʱ��3�жϷ������	 
void TIM3_IRQHandler(void)
{ 		    		  			    
	if ((TIM3->SR & 0x0001)) //���ָ����TIM�жϷ������:TIM �ж�Դ 
	{
		TIM3->SR &= (~0x0001);  //���TIMx���жϴ�����λ:TIM �ж�Դ 
		LED_G=!LED_G;
	}
}
//ͨ�ö�ʱ��3�жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ��3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR |= 1<<1; //ʱ��ʹ��

	TIM3->ARR = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM3->PSC =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM3->CR1 &= ~(1<<8); //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM3->CR1 &= ~(1<<4) ;  //TIM���ϼ���ģʽ
	TIM3->DIER |= 0x0041;
	
	register_nvic(2,0,3,TIM3_IRQChannel);
	//��ռ���ȼ�0��
	//�����ȼ�3��
	//IRQͨ����ʹ��
	//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
	//ʹ��TIMx����				
	TIM3->CR1 |= 1<<0;
	
}
//TIM3 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM3_PWM_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR |= 1<<1; //ʱ��ʹ��
 	RCC->APB2ENR |= 1<<0 | 1<<3;
	//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Timer3������ӳ��  TIM3_CH2->PB5 0x001A0800
	GPIOB->CRL &= 0xff0fffff;
	GPIOB->CRL |= 0x00b00000;//���ø�����Ϊ�����������,���TIM3 CH2��PWM���岨��//TIM_CH2 //�����������
	//GPIO_WriteBit(GPIOA, GPIO_Pin_7,Bit_SET); // PA7����	
	RCC->APB2ENR|=1<<0;     //��������ʱ��	   
	AFIO->MAPR&=0XFFFFF3FF; //���MAPR��[11:10]
	AFIO->MAPR|=1<<11;      //������ӳ��,TIM3_CH2->PB5
	

	TIM3->ARR = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 80K
	TIM3->PSC =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ����Ƶ
	TIM3->CR1 &= ~(1<<8); //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM3->CR1 &= ~(1<<4) ;  //TIM���ϼ���ģʽ
	 
	TIM3->CCMR1 |= 7<<12; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
	TIM3->CCMR1 |= 1<<11; //�Ƚ����ʹ��
	TIM3->CCR2 = 0; //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM3->CCER |= (0<<4); //�������:TIM����Ƚϼ��Ը�
	
	TIM3->CR1 |= 0x0080;
	TIM3->CR1 |= 0x0001;//ʹ��TIMx����											  
}
//��ʱ��5ͨ��1���벶������
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM5_Cap_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR |= 0x00000008;
	RCC->APB2ENR |= 0x00000004;			//ʹ��GPIO�����AFIO���ù���ģ��ʱ��ʹ��

//*********************************************************************
	GPIOA->BRR = 1<<0;//PA0 ���֮ǰ����
	GPIOA->CRL |= 0;//PA0 ����
	GPIOA->BRR = 1<<0;//PA0 ����
//*********************************************************************
	//�趨�������Զ���װֵ ���10ms���  //Ԥ��Ƶ��,1M�ļ���Ƶ��,1us��1. ����ʱ�ӷָ�:TDTS = Tck_tim//TIM���ϼ���ģʽ
	TIM5->ARR = arr;
	TIM5->PSC = psc;
	TIM5->CR1 &= ~(1<<8);
	TIM5->CR1 &= ~(1<<4);
	
	TIM5->CCMR1 |= 1<<0;
	TIM5->CCER &= ~(1<<1);
	TIM5->CCMR1 &= ~(0x3<<2);
	TIM5->CCMR1 &= ~(0xf<<4);

  //ʹ�ܶ�ʱ��5
	TIM5->CR1 |= 1<<0;
  TIM5->DMAR |= 1<<0 | 1<<1;//��������ж� ,����CC1IE�����ж�	


	//TIM3�ж�
	//��ռ���ȼ�0��
	//�����ȼ�3��
	//IRQͨ����ʹ��
	register_nvic(2,2,0,TIM5_IRQChannel);
	   
}

//����״̬
//[7]:0,û�гɹ��Ĳ���;1,�ɹ�����һ��.
//[6]:0,��û���񵽸ߵ�ƽ;1,�Ѿ����񵽸ߵ�ƽ��.
//[5:0]:����ߵ�ƽ������Ĵ���
u8  TIM5CH1_CAPTURE_STA=0;	//���벶��״̬		    				
u16	TIM5CH1_CAPTURE_VAL;	//���벶��ֵ
//��ʱ��5�жϷ������	 
void TIM5_IRQHandler(void)
{ 		    
	if((TIM5CH1_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
	{	  
		if ((TIM5->SR & 1) != 0 && (TIM5->DIER & 1) != 0)
		{
			if(TIM5CH1_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM5CH1_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					TIM5CH1_CAPTURE_STA|=0X80;//��ǳɹ�������һ��
					TIM5CH1_CAPTURE_VAL=0XFFFF;
				}else TIM5CH1_CAPTURE_STA++;
			}	 
		}
	if ((TIM5->SR & 1<<1) != 0 && (TIM5->DIER & 1<<1) != 0)//����1���������¼�
		{	
			if(TIM5CH1_CAPTURE_STA&0X40)		//����һ���½��� 		
			{	  			
				TIM5CH1_CAPTURE_STA|=0X80;		//��ǳɹ�����һ��������
				TIM5CH1_CAPTURE_VAL= TIM5->CCR1;
		   	TIM5->CCER &= ~(1<<1); //CC1P=0 ����Ϊ�����ز���
			}else  								//��δ��ʼ,��һ�β���������
			{
				TIM5CH1_CAPTURE_STA=0;			//���
				TIM5CH1_CAPTURE_VAL=0;
	 			TIM5->CNT = 0;
				TIM5CH1_CAPTURE_STA|=0X40;		//��ǲ�����������
		   	TIM5->CCER |= 1<<1;		//CC1P=1 ����Ϊ�½��ز���
			}
		}	     	    					   
 	}
	TIM5->SR = ~(1<<0 | 1<<1);
}
//TIM4 CH1 PWM������� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM4_PWM_Init(u16 arr,u16 psc)
{

	RCC->APB1ENR |= 1<<2;
	RCC->APB2ENR |= 1<<3;
 
   //���ø�����Ϊ�����������,���TIM4 CH1��PWM���岨��
	GPIOB->CRL |= 0x2<<26;
	GPIOB->CRL |= 0x3<<24;
	
	TIM4->ARR = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 80K
	TIM4->PSC =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ����Ƶ
	TIM4->CR1 &= ~(1<<8);//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM4->CR1 &= ~(1<<4);//TIM���ϼ���ģʽ
	
	TIM4->CCMR1 |= 0x7<<4;//CH1 PWM2ģʽ
	TIM4->CCMR1 |= 1<<3;//�Ƚ����ʹ��
	TIM4->CCR1 = 0;//���ô�װ�벶��ȽϼĴ���������ֵ
	TIM4->CCER |= 1<<1;//OC1 �͵�ƽ��Ч
	TIM4->CCER |= 1<<0;//ʹ��TIMx����
	
}
u8 ov_frame; 	//ͳ��֡��
//��ʱ��6�жϷ������	 
void TIM6_IRQHandler(void)
{
	if (TIM6->SR & 1<<0) //���ָ����TIM�жϷ������:TIM �ж�Դ 
	{
			LED_R=!LED_R;
			printf("frame:%dfps\r\n",ov_frame);	//��ӡ֡��
			ov_frame=0;	
	} 
	TIM6->SR &= ~(1<<0);  //���TIMx���жϴ�����λ:TIM �ж�Դ  	    
}
//������ʱ��6�жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ��3!
void TIM6_Int_Init(u16 arr,u16 psc)
{
	//ʱ��ʹ��
	RCC->APB1ENR |= 1<<4;

	TIM6->ARR = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM6->PSC =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM6->CR1 &= ~(1<<8);//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM6->CR1 &= ~(1<<4);//TIM���ϼ���ģʽ

	TIM6->DIER |= 1<<0;//ʹ�ܶ�ʱ��6���´����ж�
 
	TIM6->CR1 |= 1<<0; //ʹ��TIMx����
 	
  //TIM3�ж�
	//��ռ���ȼ�0��
	//�����ȼ�3��
	//IRQͨ����ʹ��
	register_nvic(2,1,3,TIM5_IRQChannel);
}

//LCD ����PWM�����ʼ��
//��ʼ���������Ϊ:PWM����Ƶ��Ϊ80Khz
//ͨ��LCD_BLPWM_VAL����LCD��������.
//0,�;250,�.
void LCD_PWM_Init(void)
{		 					 
	//�˲������ֶ��޸�IO������
	RCC->APB2ENR|=1<<13; 	//TIM8ʱ��ʹ��    
	RCC->APB2ENR|=1<<3;    	//ʹ��PORTBʱ��	 			 
	  	
	GPIOB->CRL&=0XFFFFFFF0;	//PB0���
	GPIOB->CRL|=0X0000000B;	//���ù������
	GPIOB->ODR|=1<<0;		//PB0����	 

	TIM8->ARR=110;			//�趨�������Զ���װֵΪ110.Ƶ��Ϊ654Khz 
	TIM8->PSC=0;			//Ԥ��Ƶ������Ƶ
	
	TIM8->CCMR1|=7<<12; 	//CH2 PWM2ģʽ		 
	TIM8->CCMR1|=1<<11; 	//CH2Ԥװ��ʹ��	   

	TIM8->CCER|=1<<6;   	//OC2�������ʹ��	   
 	TIM8->CCER|=1<<7;   	//OC2N�͵�ƽ��Ч	   
	TIM8->BDTR|=1<<15;   	//MOE�����ʹ��	   

	TIM8->CR1=0x0080;   	//ARPEʹ�� 
	TIM8->CR1|=0x01;    	//ʹ�ܶ�ʱ��8 										  
} 

