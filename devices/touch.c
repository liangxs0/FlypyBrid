/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * File name: jpstm32_touch.c
 * Author: Duke An
 * Version: 0.1
 * Date: 2014-06-17
 * Description: 触摸屏驱动实现文件
 * Copyright @ 2013 by Briup Technology,Inc. All rights Reserved.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "touch.h"
#include "jpstm32_lcd.h"
#include "jpstm32_delay.h"
#include "stdio.h"
#include "math.h"

#define	LCD_WIDTH		480
#define	LCD_HEIGHT	320

TouchTpyDef	t_pad = {
	tp_init,
	tp_scan,
	tp_adjust,
	0, 0, 0, 0,
	0,
	-0.130680, -0.085418, 508, 332,
	P_DIR_HOR,
	0x90, 0xd0
};

//绘制工具面板
static void drawToolPad(void);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	tp_spi_write_byte
 * Description: 向触摸屏写入一个字节
 * Input: NULL
 * Output: NULL
 * Return: NULL
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void tp_spiWbyte(u8 byte)
{
	u8 count=0;   
	for(count=0;count<8;count++)  
	{ 	  
		if(byte&0x80)
		{
			TP_IN = 1;
		}
		else
		{
			TP_IN = 0;
		}			   
		byte<<=1;    
		TP_CLK=0; 	 
		TP_CLK=1;		//上升沿有效	        
	}		
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	tp_readAD
 * Description: 从触摸屏读取一个字节
 * Input: NULL
 * Output: NULL
 * Return: NULL
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
u16 tp_readAD(u8 cmd)
{
	u8 count=0; 	  
	u16 Num=0; 
	TP_CLK=0;		//先拉低时钟 	 
	TP_IN=0; 	//拉低数据线
	TP_CS=0; 		//选中触摸屏IC
	tp_spiWbyte(cmd);//发送命令字
	_delay_us(6);//ADS7846的转换时间最长为6us
	TP_CLK=0; 	     	    
	_delay_us(1);    	   
	TP_CLK=1;		//给1个时钟，清除BUSY	    	    
	TP_CLK=0; 	     	    
	for(count=0;count<16;count++)//读出16位数据,只有高12位有效 
	{ 				  
		Num<<=1; 	 
		TP_CLK=0;	//下降沿有效  	    	   
		TP_CLK=1;
		if(TP_OUT)Num++; 		 
	}  	
	Num>>=4;   	//只有高12位有效.
	TP_CS=1;		//释放片选	 
	return(Num);  
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	tp_readXorY
 * Description: 带滤波的读取x或y的值
 * Input: 获取x或y值的命令
 * Output: NULL
 * Return: 返回读到的值
 * Others: 连续读取N次对这些数据进行排序，去掉最大和最小值
 *				 剩下的数据区平均值
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define READ_TIMES 5 	//读取次数
#define LOST_VAL 1	  	//丢弃值
u16 tp_readXorY(u8 cmd)
{
	u16 i, j;
	u16 buf[READ_TIMES];
	u16 sum=0;
	u16 temp;

	for(i=0;i<READ_TIMES;i++)
	{
		buf[i]=tp_readAD(cmd);		 		    
	}
	for(i=0;i<READ_TIMES-1; i++)//排序
	{
		for(j=i+1;j<READ_TIMES;j++)
		{
			if(buf[i]>buf[j])//升序排列
			{
				temp=buf[i];
				buf[i]=buf[j];
				buf[j]=temp;
			}
		}
	}	  
	sum=0;
	for(i=LOST_VAL;i<READ_TIMES-LOST_VAL;i++)
	{
		sum+=buf[i];
	}

	temp=sum/(READ_TIMES-2*LOST_VAL);

	return temp;  
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	tp_readXandY
 * Description: 读取x与y的值
 * Input: 获取保存x y值的变量
 * Output: NULL
 * Return: 返回0成功 否则失败
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
u8 tp_readXandY(u16* x, u16* y)
{
	u16 xtemp,ytemp;			 	 		  
	xtemp=tp_readXorY(t_pad.cmdRdx);
	ytemp=tp_readXorY(t_pad.cmdRdy);	  												   
	*x=xtemp;
	*y=ytemp;
	return 0;//读数成功
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	tp_readXandY2
 * Description: 二次滤波 读取x与y的值
 * Input: 获取保存x y值的变量
 * Output: NULL
 * Return: 返回0成功 否则失败
 * Others: 连续2次读取触摸屏IC,且这两次的偏差不能超过
 *  			 ERR_RANGE,满足条件,则认为读数正确,否则读数错误
 *		     该函数能大大提高准确度
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define	ERR_RANGE	50	//误差范围
u8 tp_readXandY2(u16* x, u16* y)
{
	u16 x1,y1;
	u16 x2,y2;
	u8 flag;    
	flag=tp_readXandY(&x1,&y1);  	
	if(flag)
	{
		return(1);
	}
	flag=tp_readXandY(&x2,&y2);	   
	if(flag)
	{
		return(2);   
	}
	if(((x2<=x1&&x1<x2+ERR_RANGE)||(x1<=x2&&x2<x1+ERR_RANGE))//前后两次采样在+-50内
			&&((y2<=y1&&y1<y2+ERR_RANGE)||(y1<=y2&&y2<y1+ERR_RANGE)))
	{
		*x=(x1+x2)/2;
		*y=(y1+y2)/2;
		return 0;
	}
	else
	{
		return 3;
	}			
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	tp_drawAdjustPoint
 * Description: 绘制屏幕校准点
 * Input: 绘制坐标， 颜色
 * Output: NULL
 * Return: NULL
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void tp_drawAdjustPoint(u16 x, u16 y, u16 color)
{
	LcdPen	pen;

	pen.color = color;
	pen.brush = P_SET_BRUSH;
	lcdDrawRect(x-1, y-1, 3, 3, &pen);
	lcdDrawLine(x-15, y, x+15, y, &pen);
	lcdDrawLine(x, y-15, x, y+15, &pen);
	pen.brush = P_SET_NOBRUSH;
	lcdDrawRect(x-5, y-5, 11, 11, &pen);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	tp_scan(u8 tp)
 * Description: 绘制屏幕校准点
 * Input:  tp: 1 屏幕坐标，0 物理坐标
 * Output: NULL
 * Return: 当前屏幕状态 0 没有按下 1 有按下
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
u8 tp_scan(u8 tp)
{
	if(TP_PEN==0)//有按键按下
	{
		if(tp)
		{
			tp_readXandY2(&t_pad.currX, &t_pad.currY);//读取物理坐标
		}
		else if(!tp_readXandY2(&t_pad.currX, &t_pad.currY))//读取屏幕坐标
		{
			//将结果转换为屏幕坐标
			t_pad.currX = t_pad.xfac*t_pad.currX+t_pad.xoff;
			t_pad.currY = 320 - (t_pad.yfac*t_pad.currY+t_pad.yoff);
		} 
		
		if((t_pad.penStat&TP_PRES_DOWN)==0)//之前没有被按下
		{			
			//按键按下 
			t_pad.penStat = TP_PRES_DOWN|TP_CATH_PRES;
			//记录第一次按下时的坐标 
			t_pad.lastX = t_pad.currX;
			t_pad.lastY = t_pad.currY;

		}
	}
	else
	{
		if(t_pad.penStat&TP_PRES_DOWN)//之前是被按下的
		{
			t_pad.penStat&=0x7F;//标记按键松开	
		}
		else//之前就没有被按下
		{
			t_pad.lastX = 0;
			t_pad.lastY = 0;
			t_pad.currX = 0xFFFF;
			t_pad.currY = 0xFFFF;
		}	    
	}
	return t_pad.penStat&TP_PRES_DOWN;//返回当前的触屏状态
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	tp_init
 * Description: 初始化触摸屏
 * Input:  NULL
 * Output: NULL
 * Return: 成功返回0 否则返回非0
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
u8 tp_init(void)
{

	//所以上拉之前,必须使能时钟.才能实现真正的上拉输出
	RCC->APB2ENR|=1<<3;    //PB时钟使能	   
	RCC->APB2ENR|=1<<7;    //PF时钟使能	   

	GPIOB->CRL&=0XFFFFF00F;//PB1  2
	GPIOB->CRL|=0X00000330; 
	GPIOB->ODR|=3<<1;      //PB1 2 推挽输出 	 
	GPIOF->CRH&=0XFFFFF000;
	GPIOF->CRH|=0X00000838;
	GPIOF->ODR|=7<<8;      //PF8,9,10 全部上拉	  
	tp_readXandY(&t_pad.currX, &t_pad.currY);//第一次读取初始化	

	lcdClear(WHITE);	
	if(t_pad.xfac>-0.000001&&t_pad.xfac<0.000001){
		if(tp_adjust()==0)
		{
			return 0;
		}
		else 
		{
			return 1;
		}
	}
	return 0;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	tp_adjust
 * Description: 屏幕校准
 * Input:  NULL
 * Output: NULL
 * Return: 成功返回0 否则返回非0
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
u8 tp_adjust(void)
{
	u16 pos_temp[4][2];//坐标缓存值
	u8  cnt=0;	
	u16 d1,d2;
	u32 tem1,tem2;
	float fac; 	
	u16 outtime=0;
				
	LcdPen	pen;
	pen.color = RED;
	pen.p_size = P_S16CN;
	
	cnt=0;
	
	lcdClear(WHITE);//清屏   

	tp_drawAdjustPoint(20, 20, RED);
	
	lcdDrawStr(100, 120, (u8*)"请依次点击屏幕上的校准点进行屏幕校准!", &pen);

	t_pad.penStat = 0;//消除触发信号 
	t_pad.xfac = 0;//xfac用来标记是否校准过,所以校准之前必须清掉!以免错误	 

	while(1)//如果连续10秒钟没有按下,则自动退出
	{
		//扫描物理坐标
		t_pad.scan(1);
		if((t_pad.penStat&0xc0)==TP_CATH_PRES)//按键按下了一次(此时按键松开了.)
		{	
			outtime=0;		
			t_pad.penStat&=~(1<<6);//标记按键已经被处理过了.

			pos_temp[cnt][0]=t_pad.currX;
			pos_temp[cnt][1]=t_pad.currY;
			cnt++;	  
			switch(cnt)
			{			   
				case 1:						 
					//清除点1 
					tp_drawAdjustPoint(20,20,WHITE);
					//画点2
					tp_drawAdjustPoint(LCD_WIDTH-20,20,RED);
					break;
				case 2:
					//清除点2 
					tp_drawAdjustPoint(LCD_WIDTH-20,20,WHITE);
					//画点3
					tp_drawAdjustPoint(20,LCD_HEIGHT-20,RED);
					break;
				case 3:
					//清除点3 
					tp_drawAdjustPoint(20,LCD_HEIGHT-20,WHITE);
					//画点4
					tp_drawAdjustPoint(LCD_WIDTH-20,LCD_HEIGHT-20,RED);
					break;
				case 4:	 //全部四个点已经得到
					//对边相等
					tem1=abs(pos_temp[0][0]-pos_temp[1][0]);//x1-x2
					tem2=abs(pos_temp[0][1]-pos_temp[1][1]);//y1-y2
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//得到1,2的距离

					tem1=abs(pos_temp[2][0]-pos_temp[3][0]);//x3-x4
					tem2=abs(pos_temp[2][1]-pos_temp[3][1]);//y3-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//得到3,4的距离
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05||d1==0||d2==0)//不合格
					{
						cnt=0;
						tp_drawAdjustPoint(LCD_WIDTH-20,LCD_HEIGHT-20,WHITE);	//清除点4
						tp_drawAdjustPoint(20,20,RED);//画点1
						tp_printAdjustInfo(pos_temp, fac*100);
						continue;
					}
					tem1=abs(pos_temp[0][0]-pos_temp[2][0]);//x1-x3
					tem2=abs(pos_temp[0][1]-pos_temp[2][1]);//y1-y3
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//得到1,3的距离

					tem1=abs(pos_temp[1][0]-pos_temp[3][0]);//x2-x4
					tem2=abs(pos_temp[1][1]-pos_temp[3][1]);//y2-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//得到2,4的距离
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05)//不合格
					{
						cnt=0;
						tp_drawAdjustPoint(460,300,WHITE);	//清除点4
						tp_drawAdjustPoint(20,20,RED);//画点1
						tp_printAdjustInfo(pos_temp, fac*100);
						//tp_showAdjustInfo(pos_temp[0][0],pos_temp[0][1],pos_temp[1][0],pos_temp[1][1],pos_temp[2][0],pos_temp[2][1],pos_temp[3][0],pos_temp[3][1],fac*100);//显示数据   
						continue;
					}//正确了

					//对角线相等
					tem1=abs(pos_temp[1][0]-pos_temp[2][0]);//x1-x3
					tem2=abs(pos_temp[1][1]-pos_temp[2][1]);//y1-y3
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//得到1,4的距离

					tem1=abs(pos_temp[0][0]-pos_temp[3][0]);//x2-x4
					tem2=abs(pos_temp[0][1]-pos_temp[3][1]);//y2-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//得到2,3的距离
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05)//不合格
					{
						cnt=0;
						tp_drawAdjustPoint(460,300,WHITE);	//清除点4
						tp_drawAdjustPoint(20,20,RED);//画点1
						tp_printAdjustInfo(pos_temp, fac*100);
						//tp_showAdjustInfo(pos_temp[0][0],pos_temp[0][1],pos_temp[1][0],pos_temp[1][1],pos_temp[2][0],pos_temp[2][1],pos_temp[3][0],pos_temp[3][1],fac*100);//显示数据   
						continue;
					}//正确了
					//计算结果
					t_pad.xfac=(float)(LCD_WIDTH-40)/(pos_temp[1][0]-pos_temp[0][0]);//得到xfac		 
					t_pad.xoff=(LCD_WIDTH-t_pad.xfac*(pos_temp[1][0]+pos_temp[0][0]))/2;//得到xoff

					t_pad.yfac=(float)(LCD_HEIGHT-40)/(pos_temp[2][1]-pos_temp[0][1]);//得到yfac
					t_pad.yoff=(LCD_HEIGHT-t_pad.yfac*(pos_temp[2][1]+pos_temp[0][1]))/2;//得到yoff  
					if(abs(t_pad.xfac)>2||abs(t_pad.yfac)>2)//触屏和预设的相反了.
					{
						cnt=0;
						lcdClear(WHITE);
						lcdDrawStr(100,120, (u8*)"未完成校验，请重新操作！", &pen);
						tp_drawAdjustPoint(LCD_WIDTH-20,LCD_HEIGHT-20,WHITE);	//清除点4
						tp_drawAdjustPoint(20,20,RED);								//画点1

						t_pad.direct = (t_pad.direct==P_DIR_HOR)? P_DIR_VER:P_DIR_HOR;//修改触屏类型.
						if(t_pad.direct)//X,Y方向与屏幕相反
						{
							t_pad.cmdRdx = 0x90;
							t_pad.cmdRdy = 0xD0; 
						}else				   //X,Y方向与屏幕相同
						{
							t_pad.cmdRdx = 0xD0;
							t_pad.cmdRdy = 0x90; 
						}			    
						continue;
					}		
					lcdClear(WHITE);//清屏
					lcdDrawStr(100,120, (u8*)"屏幕校验完成!", &pen);
#if	TP_DEBIG_ENABLE
					printf("xfac: %f, xoff: %d\r\n", t_pad.xfac, t_pad.xoff);
					printf("yfac: %f, yoff: %d\r\n", t_pad.yfac, t_pad.yoff);
#endif				
					_delay_ms(1000);  
					lcdClear(WHITE);//清屏   
					return 0;//校正完成				 
			}
		}
		_delay_ms(10);
		outtime++;
		if(outtime>1000)
		{
			lcdClear(WHITE);
			lcdDrawStr(100, 120, (u8*)"屏幕校验操作超时！", &pen);
			_delay_ms(1000);
			lcdClear(WHITE);
			break;
		} 
	}
	return 0;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	tp_showAdjustInfo
 * Description: 打印校准信息
 * Input:  NULL
 * Output: NULL
 * Return: 成功返回0 否则返回非0
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
u8 tp_printAdjustInfo(u16 pos[][2], u16 fac)
{
	printf("x1 = %d, y1 = %d\r\n", pos[0][0], pos[0][1]);
	printf("x2 = %d, y2 = %d\r\n", pos[1][0], pos[1][1]);
	printf("x3 = %d, y3 = %d\r\n", pos[2][0], pos[2][1]);
	printf("x4 = %d, y4 = %d\r\n", pos[3][0], pos[3][1]);
	printf("fac = %d\r\n", fac);
	return 0;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	tp_screenTrack
 * Description: 屏幕操作跟踪
 * Input:  NULL
 * Output: NULL
 * Return: 
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void tp_screenTrack(void)
{
	u16 x, y;
	LcdPen pen;
	pen.color = RED;
	pen.p_size = P_S16CN;
	pen.brush = P_SET_BRUSH;
	pen.direct = P_DIR_HOR;
	
	//绘制调色板
	drawToolPad();
	
	while(1)
	{
		if(t_pad.scan(1))
		{
			x = t_pad.currX*t_pad.xfac+t_pad.xoff;
			y = t_pad.currY*t_pad.yfac+t_pad.yoff;

			if(x<LCD_WIDTH&&y<LCD_HEIGHT)
			{
				if(y>16)
				{
					lcdDrawBigPoint(x, y, &pen);
				}
				else
				{
					if(x>(LCD_WIDTH-32))
					{
						pen.color = WHITE;
						lcdDrawRect(0, 17, 480, 310, &pen);
					}
					else if(x<(1*32))
					{
						pen.color = RED;
					}
					else if(x>(1*32)&&x<(2*32))
					{
						pen.color = BLUE;
					}
					else if(x>(2*32)&&x<(3*32))
					{
						pen.color = GREEN;
					}
					else if(x>(3*32)&&x<(4*32))
					{
						pen.color = BLACK;
					}
					else if(x>(4*32)&&x<(5*32))
					{
						pen.color = YELLOW;
					}
					else if(x>(5*32)&&x<(6*32))
					{
						pen.color = GRED;
					}
					else if(x>(6*32)&&x<(7*32))
					{
						pen.color = WHITE;
					}
				}
			}		
		}
		else
		{
			_delay_ms(10);
		}
	}
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * 内部静态函数
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//绘制工具面板
void drawToolPad(void)
{
	LcdPen pen;
	pen.color = RED;
	pen.p_size = P_S16CN;
	pen.brush = P_SET_BRUSH;
	pen.direct = P_DIR_HOR;
	
	pen.color = RED;
	lcdDrawRect(32*0, 0, 32, 16, &pen);
	pen.color = BLUE;
	lcdDrawRect(32*1, 0, 32, 16, &pen);
	pen.color = GREEN;
	lcdDrawRect(32*2, 0, 32, 16, &pen);
	pen.color = BLACK;
	lcdDrawRect(32*3, 0, 32, 16, &pen);
	pen.color = YELLOW;
	lcdDrawRect(32*4, 0, 32, 16, &pen);
	pen.color = GRED;
	lcdDrawRect(32*5, 0, 32, 16, &pen);
	pen.color = WHITE;
	lcdDrawRect(32*6, 0, 32, 16, &pen);
	pen.color = BLUE;
	lcdDrawStr(LCD_WIDTH-32, 0, (u8*)"清屏", &pen);	
}


