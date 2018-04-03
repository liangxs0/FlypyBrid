/*************************************************************
 * File name: jpstm32_delay.c
 * Author: Duke An
 * Version: 0.1
 * Date: 20140617
 * Description: 系统延时相关实现
 * Copyright @ 2013 by Briup Technology,Inc. All rights Reserved.
 *************************************************************/
#include "jpstm32_delay.h"

//us延时倍乘数			   
static u8  fac_us=0;
//ms延时倍乘数,在ucos下,代表每个节拍的ms数
static u16 fac_ms=0;

//如果OS_CRITICAL_METHOD定义了,说明使用ucosII了.
#ifdef OS_CRITICAL_METHOD 	

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	SysTick_Handler
 * Description: systick 中断服务程序
 * Input: NULL
 * Output: NULL
 * Return: NULL
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void SysTick_Handler(void)
{				   
	OSIntEnter();		//进入中断
	OSTimeTick();       //调用ucos的时钟服务程序               
	OSIntExit();        //触发任务切换软中断
}
#endif

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	_delay_init
 * Description: 
 *	>延时功能初始化,当使用ucos的时候,
 *	>此函数会初始化ucos的时钟节拍
 * Input: 
 *	>SYSCLK 系统时钟频率,此刻指的是倍频之后的频率,即主频
 *	 当前系统中应传入72 表示72MHz
 * Output: NULL
 * Return: NULL
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void _delay_init(u8 SYSCLK)
{

	//如果使用了ucosII
#ifdef OS_CRITICAL_METHOD 	
	u32 reload;
#endif
	//SYSTICK使用外部时钟源	 
	SysTick->CTRL&=~(1<<2);	
	//不论是否使用ucos,fac_us都需要使用
	fac_us=SYSCLK/8;		

	//如果使用了ucosII
#ifdef OS_CRITICAL_METHOD 	
	//每秒钟的计数次数 单位为K	   
	reload=SYSCLK/8;		
	//根据OS_TICKS_PER_SEC设定溢出时间
	//reload为24位寄存器,最大值:16777216,在72M下,约合1.86s左右	
	reload*=1000000/OS_TICKS_PER_SEC;
	//代表ucos可以延时的最少单位	   
	fac_ms=1000/OS_TICKS_PER_SEC;
	//开启SYSTICK中断
	SysTick->CTRL |= 1<<1;
	//每1/OS_TICKS_PER_SEC秒中断一次	
	SysTick->LOAD=reload;
	//开启SYSTICK    
	SysTick->CTRL |= 1<<0;
#else
	//非ucos下,代表每个ms需要的systick时钟数   
	fac_ms=(u16)fac_us*1000;
#endif
}								    

//如果使用了ucosII
#ifdef OS_CRITICAL_METHOD
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	_delay_us
 * Description: 
 *	>微秒级延时程序
 * Input: 
 *	>延时的微秒数
 * Output: NULL
 * Return: NULL
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void _delay_us(u32 nus)
{		
	u32 ticks;
	u32 told,tnow,tcnt=0;
	u32 reload=SysTick->LOAD;	//LOAD的值	    	 
	ticks=nus*fac_us; 			//需要的节拍数	  		 
	tcnt=0;
	OSSchedLock();				//阻止ucos调度，防止打断us延时
	told=SysTick->VAL;        	//刚进入时的计数器值
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			//这里注意一下SYSTICK是一个递减的计数器就可以了.
			if(tnow<told)
			{
				tcnt+=told-tnow;
			} 
			else 
			{
				tcnt+=reload-tnow+told;	    
			}
			told=tnow;
			//时间超过/等于要延迟的时间,则退出.
			if(tcnt>=ticks)
			{
				break;
			}
		}  
	}
	OSSchedUnlock();			//开启ucos调度 									    
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	_delay_ms
 * Description: 
 *	>毫秒级延时程序
 * Input: 
 *	>延时的毫秒数
 * Output: NULL
 * Return: NULL
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void _delay_ms(u16 nms)
{	
	//如果os已经在跑了	    
	if(OSRunning==TRUE)
	{		  
		//延时的时间大于ucos的最少时间周期 
		if(nms>=fac_ms)
		{
			//ucos延时
			OSTimeDly(nms/fac_ms);
		}
		//ucos已经无法提供这么小的延时了,采用普通方式延时    
		nms%=fac_ms;
	}
	_delay_us((u32)(nms*1000));	//普通方式延时 
}

#else//不用ucos时

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	_delay_us
 * Description: 
 *	>微秒级延时程序
 * Input: 
 *	>延时的微秒数
 * Output: NULL
 * Return: NULL
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void _delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; //时间加载	  		 
	SysTick->VAL=0x00;        //清空计数器
	SysTick->CTRL=0x01 ;      //开始倒数 	 
	do
	{
		temp=SysTick->CTRL;
	} while((temp&0x01)&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	 
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	_delay_ms
 * Description: 
 *	>毫秒级延时程序
 * Input: 
 *	>延时的毫秒数 
 *	>注意nms的范围 SysTick->LOAD为24位寄存器
 *	>所以,最大延时为: nms<=0xffffff*8*1000/SYSCLK 
 *	>SYSCLK单位为Hz,nms单位为ms 对72M条件下,nms<=1864 
 * Output: NULL
 * Return: NULL
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void _delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;           //清空计数器
	SysTick->CTRL=0x01 ;          //开始倒数  
	do
	{
		temp=SysTick->CTRL;
	} while((temp&0x01)&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	  	    
} 
#endif


