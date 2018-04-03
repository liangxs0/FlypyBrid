#include "timer.h"
#include "led.h"
#include "jpstm32_usart.h"

//定时器3中断服务程序	 
void TIM3_IRQHandler(void)
{ 		    		  			    
	if ((TIM3->SR & 0x0001)) //检查指定的TIM中断发生与否:TIM 中断源 
	{
		TIM3->SR &= (~0x0001);  //清除TIMx的中断待处理位:TIM 中断源 
		LED_G=!LED_G;
	}
}
//通用定时器3中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR |= 1<<1; //时钟使能

	TIM3->ARR = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM3->PSC =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM3->CR1 &= ~(1<<8); //设置时钟分割:TDTS = Tck_tim
	TIM3->CR1 &= ~(1<<4) ;  //TIM向上计数模式
	TIM3->DIER |= 0x0041;
	
	register_nvic(2,0,3,TIM3_IRQChannel);
	//先占优先级0级
	//从优先级3级
	//IRQ通道被使能
	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
	//使能TIMx外设				
	TIM3->CR1 |= 1<<0;
	
}
//TIM3 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM3_PWM_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR |= 1<<1; //时钟使能
 	RCC->APB2ENR |= 1<<0 | 1<<3;
	//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Timer3部分重映射  TIM3_CH2->PB5 0x001A0800
	GPIOB->CRL &= 0xff0fffff;
	GPIOB->CRL |= 0x00b00000;//设置该引脚为复用输出功能,输出TIM3 CH2的PWM脉冲波形//TIM_CH2 //复用推挽输出
	//GPIO_WriteBit(GPIOA, GPIO_Pin_7,Bit_SET); // PA7上拉	
	RCC->APB2ENR|=1<<0;     //开启辅助时钟	   
	AFIO->MAPR&=0XFFFFF3FF; //清除MAPR的[11:10]
	AFIO->MAPR|=1<<11;      //部分重映像,TIM3_CH2->PB5
	

	TIM3->ARR = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 80K
	TIM3->PSC =psc; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM3->CR1 &= ~(1<<8); //设置时钟分割:TDTS = Tck_tim
	TIM3->CR1 &= ~(1<<4) ;  //TIM向上计数模式
	 
	TIM3->CCMR1 |= 7<<12; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM3->CCMR1 |= 1<<11; //比较输出使能
	TIM3->CCR2 = 0; //设置待装入捕获比较寄存器的脉冲值
	TIM3->CCER |= (0<<4); //输出极性:TIM输出比较极性高
	
	TIM3->CR1 |= 0x0080;
	TIM3->CR1 |= 0x0001;//使能TIMx外设											  
}
//定时器5通道1输入捕获配置
//arr：自动重装值
//psc：时钟预分频数
void TIM5_Cap_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR |= 0x00000008;
	RCC->APB2ENR |= 0x00000004;			//使能GPIO外设和AFIO复用功能模块时钟使能

//*********************************************************************
	GPIOA->BRR = 1<<0;//PA0 清除之前设置
	GPIOA->CRL |= 0;//PA0 输入
	GPIOA->BRR = 1<<0;//PA0 下拉
//*********************************************************************
	//设定计数器自动重装值 最大10ms溢出  //预分频器,1M的计数频率,1us加1. 设置时钟分割:TDTS = Tck_tim//TIM向上计数模式
	TIM5->ARR = arr;
	TIM5->PSC = psc;
	TIM5->CR1 &= ~(1<<8);
	TIM5->CR1 &= ~(1<<4);
	
	TIM5->CCMR1 |= 1<<0;
	TIM5->CCER &= ~(1<<1);
	TIM5->CCMR1 &= ~(0x3<<2);
	TIM5->CCMR1 &= ~(0xf<<4);

  //使能定时器5
	TIM5->CR1 |= 1<<0;
  TIM5->DMAR |= 1<<0 | 1<<1;//允许更新中断 ,允许CC1IE捕获中断	


	//TIM3中断
	//先占优先级0级
	//从优先级3级
	//IRQ通道被使能
	register_nvic(2,2,0,TIM5_IRQChannel);
	   
}

//捕获状态
//[7]:0,没有成功的捕获;1,成功捕获到一次.
//[6]:0,还没捕获到高电平;1,已经捕获到高电平了.
//[5:0]:捕获高电平后溢出的次数
u8  TIM5CH1_CAPTURE_STA=0;	//输入捕获状态		    				
u16	TIM5CH1_CAPTURE_VAL;	//输入捕获值
//定时器5中断服务程序	 
void TIM5_IRQHandler(void)
{ 		    
	if((TIM5CH1_CAPTURE_STA&0X80)==0)//还未成功捕获	
	{	  
		if ((TIM5->SR & 1) != 0 && (TIM5->DIER & 1) != 0)
		{
			if(TIM5CH1_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((TIM5CH1_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					TIM5CH1_CAPTURE_STA|=0X80;//标记成功捕获了一次
					TIM5CH1_CAPTURE_VAL=0XFFFF;
				}else TIM5CH1_CAPTURE_STA++;
			}	 
		}
	if ((TIM5->SR & 1<<1) != 0 && (TIM5->DIER & 1<<1) != 0)//捕获1发生捕获事件
		{	
			if(TIM5CH1_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
			{	  			
				TIM5CH1_CAPTURE_STA|=0X80;		//标记成功捕获到一次上升沿
				TIM5CH1_CAPTURE_VAL= TIM5->CCR1;
		   	TIM5->CCER &= ~(1<<1); //CC1P=0 设置为上升沿捕获
			}else  								//还未开始,第一次捕获上升沿
			{
				TIM5CH1_CAPTURE_STA=0;			//清空
				TIM5CH1_CAPTURE_VAL=0;
	 			TIM5->CNT = 0;
				TIM5CH1_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
		   	TIM5->CCER |= 1<<1;		//CC1P=1 设置为下降沿捕获
			}
		}	     	    					   
 	}
	TIM5->SR = ~(1<<0 | 1<<1);
}
//TIM4 CH1 PWM输出设置 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM4_PWM_Init(u16 arr,u16 psc)
{

	RCC->APB1ENR |= 1<<2;
	RCC->APB2ENR |= 1<<3;
 
   //设置该引脚为复用输出功能,输出TIM4 CH1的PWM脉冲波形
	GPIOB->CRL |= 0x2<<26;
	GPIOB->CRL |= 0x3<<24;
	
	TIM4->ARR = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 80K
	TIM4->PSC =psc; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM4->CR1 &= ~(1<<8);//设置时钟分割:TDTS = Tck_tim
	TIM4->CR1 &= ~(1<<4);//TIM向上计数模式
	
	TIM4->CCMR1 |= 0x7<<4;//CH1 PWM2模式
	TIM4->CCMR1 |= 1<<3;//比较输出使能
	TIM4->CCR1 = 0;//设置待装入捕获比较寄存器的脉冲值
	TIM4->CCER |= 1<<1;//OC1 低电平有效
	TIM4->CCER |= 1<<0;//使能TIMx外设
	
}
u8 ov_frame; 	//统计帧数
//定时器6中断服务程序	 
void TIM6_IRQHandler(void)
{
	if (TIM6->SR & 1<<0) //检查指定的TIM中断发生与否:TIM 中断源 
	{
			LED_R=!LED_R;
			printf("frame:%dfps\r\n",ov_frame);	//打印帧率
			ov_frame=0;	
	} 
	TIM6->SR &= ~(1<<0);  //清除TIMx的中断待处理位:TIM 中断源  	    
}
//基本定时器6中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器3!
void TIM6_Int_Init(u16 arr,u16 psc)
{
	//时钟使能
	RCC->APB1ENR |= 1<<4;

	TIM6->ARR = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM6->PSC =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM6->CR1 &= ~(1<<8);//设置时钟分割:TDTS = Tck_tim
	TIM6->CR1 &= ~(1<<4);//TIM向上计数模式

	TIM6->DIER |= 1<<0;//使能定时器6更新触发中断
 
	TIM6->CR1 |= 1<<0; //使能TIMx外设
 	
  //TIM3中断
	//先占优先级0级
	//从优先级3级
	//IRQ通道被使能
	register_nvic(2,1,3,TIM5_IRQChannel);
}

//LCD 背光PWM输出初始化
//初始化背光参数为:PWM控制频率为80Khz
//通过LCD_BLPWM_VAL设置LCD背光亮度.
//0,最暗;250,最暗.
void LCD_PWM_Init(void)
{		 					 
	//此部分需手动修改IO口设置
	RCC->APB2ENR|=1<<13; 	//TIM8时钟使能    
	RCC->APB2ENR|=1<<3;    	//使能PORTB时钟	 			 
	  	
	GPIOB->CRL&=0XFFFFFFF0;	//PB0输出
	GPIOB->CRL|=0X0000000B;	//复用功能输出
	GPIOB->ODR|=1<<0;		//PB0上拉	 

	TIM8->ARR=110;			//设定计数器自动重装值为110.频率为654Khz 
	TIM8->PSC=0;			//预分频器不分频
	
	TIM8->CCMR1|=7<<12; 	//CH2 PWM2模式		 
	TIM8->CCMR1|=1<<11; 	//CH2预装载使能	   

	TIM8->CCER|=1<<6;   	//OC2互补输出使能	   
 	TIM8->CCER|=1<<7;   	//OC2N低电平有效	   
	TIM8->BDTR|=1<<15;   	//MOE主输出使能	   

	TIM8->CR1=0x0080;   	//ARPE使能 
	TIM8->CR1|=0x01;    	//使能定时器8 										  
} 

