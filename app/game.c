#include "game.h"
#include "flappybird_background.c"
#include "flappybird_logo.c"
#include "flappybird_button_play.c"
#include "flappybird_button_stop.c"
#include "flappybird_bird_u.c"
#include "flappybird_bird_m.c"
#include "flappybird_bird_d.c"
#include "flappybird_tag.c"
#include "flappybird_getstart.c"
#include "flappybird_column_bottom.c"
#include "flappybird_column_middle.c"
#include "flappybird_gameover.c"
#include "flappybird_score_board.c"

u16 FlappyBird_Frame[480][320] __attribute__((at(0x68000000)));
u16 FlappyBird_Bird_Buff[18][26] __attribute__((at(0X6804C000)));
u16 FlappyBird_column_Buff[3][359][39] __attribute__((at(0X6804F000))) ;
u16 FlappyBird_column_Buff_BG[3][359][39] __attribute__((at(0X6806D000)));

//==================================================================================================================

u8 hide[3] = {0,0,0};
static u16 l_x = 0, l_y = 0;
static u16 l_x_c[3] = {0},l_y_c[3] = {0};
u8 ud_mode = 0,bird_mode = 'u';
int bird_height = 200,bird_wide = 57;
u16 score_lock = 0;
u32 score = 0;
int difficult = 0;

//**********************************************************
// 帧操作区 
//**********************************************************/

//在帧上画一个点
void FlappyBird_Frame_DrawPoint( u16 x, u16 y, u16 color )
{
	FlappyBird_Frame[y][x] = color;
}

//在帧上填充颜色
void FlappyBird_Frame_Fill( u16 x, u16 y, u16 xx, u16 yy, u16 color )
{
	u16 x_t,y_t;
	for( y_t = y; y_t <= yy; y_t++ )
	{
		for( x_t = x; x_t <= xx; x_t++)
		{
			FlappyBird_Frame_DrawPoint(x_t,y_t,color);
		}
	}
}
//清除帧
void FlappyBird_Frame_Clear( void )
{
	u16 x_t,y_t;
	for( y_t = 0; y_t < 480; y_t++ )
	{
		for( x_t = 0; x_t < 320; x_t++)
		{
			FlappyBird_Frame_DrawPoint(x_t,y_t,0xEED3);		//0xEED3
		}
	}
}

//在帧上画线
//x1,y1:起点坐标
//x2,y2:终点坐标  
void FlappyBird_Frame_DrawLine( u16 x1, u16 y1, u16 x2, u16 y2 , u16 color )
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 
	delta_x=x2-x1; //计算坐标增量 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1; //设置单步方向 
	else if(delta_x==0)incx=0;//垂直线 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;//水平线 
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )//画线输出 
	{  
		FlappyBird_Frame_DrawPoint(uRow,uCol,color);//画点 
		xerr+=delta_x ; 
		yerr+=delta_y ; 
		if(xerr>distance) 
		{ 
			xerr-=distance; 
			uRow+=incx;
		} 
		if(yerr>distance) 
		{ 
			yerr-=distance; 
			uCol+=incy; 
		} 
	}  
} 


void FlappyBird_DrawGreenBar( u16 x, u16 y, u16 color )
{
	u8 i;
	color = ((color&0x1f)<<11) + (color&0x07e0) + ((color&0xf800)>>11);
	FlappyBird_Frame_Fill(x,y,320,y+16,color);
	for( i = 0; i < 16; i++)
		FlappyBird_DrawParallelogram(x+i*20,y+16,0x35EE);
	
}

void FlappyBird_DrawGreenBar_Play( u16 x, u16 y,u8 Speed )
{
	u8 i;
	u16 temp,xx,yy;
	for( i = 0; i < Speed ; i++ )
	{
		for( yy = y; yy < y + 16; yy++ )
		{
			for( xx = x; xx < 319 ; xx++)
			{
				if(x == xx)
				{
					temp = FlappyBird_Frame[yy][xx];
				}
				FlappyBird_Frame[yy][xx] = FlappyBird_Frame[yy][xx+1];
			}
			FlappyBird_Frame[yy][xx] = temp;
		}
	}
}


void FlappyBird_DrawParallelogram( u16 x, u16 y, u16 color )
{
	u8 i;
	for( i = 0; i < 10; i++ )
		FlappyBird_Frame_DrawLine(x+i,y-1,x+10+i,y-16,color);
}


void FlappyBird_Frame_ShowChar(u16 x,u16 y,u8 num,u8 size,u8 mode,u16 color)
{  							  
    u8 temp,t1,t;
	u16 y0=y;
	u16 colortemp=color;      			     
	//设置窗口		   
	num=num-' ';//得到偏移后的值
	if(!mode) //非叠加方式
	{
	    for(t=0;t<size;t++)
	    {   
			if(size==12)temp=asc2_1206[num][t];  //调用1206字体
			else temp=asc2_1608[num][t];		 //调用1608字体 	                          
	        for(t1=0;t1<8;t1++)
			{			    
		        if(temp&0x80)POINT_COLOR=colortemp;
				else POINT_COLOR=BACK_COLOR;
				FlappyBird_Frame_DrawPoint(x,y,POINT_COLOR);
				temp<<=1;
				y++;
				if(x>=Lcd_Dev.width){POINT_COLOR=colortemp;return;}//超区域了
				if((y-y0)==size)
				{
					y=y0;
					x++;
					if(x>=Lcd_Dev.width){POINT_COLOR=colortemp;return;}//超区域了
					break;
				}
			}  	 
	    }    
	}else//叠加方式
	{
	    for(t=0;t<size;t++)
	    {   
			if(size==12)temp=asc2_1206[num][t];  //调用1206字体
			else temp=asc2_1608[num][t];		 //调用1608字体
	        for(t1=0;t1<8;t1++)
			{			    
		        if(temp&0x80)FlappyBird_Frame_DrawPoint(x,y,POINT_COLOR);
				temp<<=1;
				y++;
				if(x>=Lcd_Dev.height){POINT_COLOR=colortemp;return;}//超区域了
				if((y-y0)==size)
				{
					y=y0;
					x++;
					if(x>=Lcd_Dev.width){POINT_COLOR=colortemp;return;}//超区域了
					break;
				}
			}  	 
	    }     
	}
	POINT_COLOR=colortemp;	    	   	 	  
}

void FlappyBird_Frame_ShowString(u16 x,u16 y,u16 width,u16 height,u8 size,u8 *p,u16 color)
{         
	u8 x0=x;
	width+=x;
	height+=y;
    while((*p<='~')&&(*p>=' '))//判断是不是非法字符!
    {       
        if(x>=width){x=x0;y+=size;}
        if(y>=height)break;//退出
        FlappyBird_Frame_ShowChar(x,y,*p,size,0,color);
        x+=size/2;
        p++;
    }  
}

void FlappyBird_Frame_DrawBGPic( u16 x, u16 y )
{
	u16 color,xx = x,yy = y;
	
	for( yy = y; yy < 80 + y; yy++ )
	{
		for( xx = x; xx < 320; xx++ )
		{
			color = gImage_flappybird_background[((yy-y)*320+xx)*2+1];
			color <<= 8;
			color = color + gImage_flappybird_background[((yy-y)*320+xx)*2];
			color = ((color&0x1f)<<11) + (color&0x07e0) + ((color&0xf800)>>11);
			FlappyBird_Frame_DrawPoint(xx,yy,color);
		}
	}
}

void FlappyBird_Frame_DrawLogo( u16 x, u16 y )
{
		u16 color,xx = x,yy = y;
	
	for( yy = y; yy < 48 + y; yy++ )
	{
		for( xx = x; xx < 178 + x; xx++ )
		{
			color = gImage_flappybird_Logo[((yy-y)*178+(xx-x))*2+1];
			color <<= 8;
			color = color + gImage_flappybird_Logo[((yy-y)*178+(xx-x))*2];
			color = ((color&0x1f)<<11) + (color&0x07e0) + ((color&0xf800)>>11);
			FlappyBird_Frame_DrawPoint(xx,yy,color);
		}
	}
	
}

void FlappyBird_Frame_DrawButton( u16 x, u16 y , u8 mode )
{
		u16 color,xx = x,yy = y;
		int R,G,B;
		if(0 == mode)
		{
			for( yy = y; yy < 58 + y; yy++ )
			{
				for( xx = x; xx < 104 + x; xx++ )
				{
					color = gImage_flappybird_button_play[((yy-y)*104+(xx-x))*2+1];
					color <<= 8;
					color = color + gImage_flappybird_button_play[((yy-y)*104+(xx-x))*2];
					R = color & 0xf800;
					R >>= 11;
					G = color & 0x07e0;
					G >>= 5;
					B = color & 0x001f;
					if((B - R) > 7)
					{
						if((G - B) > 10 && (G - R) > 10 && (B - R) < 10)
						{
							color = ((color&0x1f)<<11) + (color&0x07e0) + ((color&0xf800)>>11);
							FlappyBird_Frame_DrawPoint(xx,yy,color);
						}
					}
					else
					{
						color = ((color&0x1f)<<11) + (color&0x07e0) + ((color&0xf800)>>11);
						FlappyBird_Frame_DrawPoint(xx,yy,color);
					}
				}
			}
		}
		else
		{
			for( yy = y; yy < 58 + y; yy++ )
			{
				for( xx = x; xx < 104 + x; xx++ )
				{
					color = gImage_flappybird_button_stop[((yy-y)*104+(xx-x))*2+1];
					color <<= 8;
					color = color + gImage_flappybird_button_stop[((yy-y)*104+(xx-x))*2];
					R = color & 0xf800;
					R >>= 11;
					G = color & 0x07e0;
					G >>= 5;
					B = color & 0x001f;
					if((B - R) > 7)
						;
					else
					{
						color = ((color&0x1f)<<11) + (color&0x07e0) + ((color&0xf800)>>11);
						FlappyBird_Frame_DrawPoint(xx,yy,color);
					}
				}
			}
		}
}

void FlappyBird_Frame_DrawBird( u16 x, u16 y , u8 mode )
{
	u16 color,xx = x,yy = y;
	int R,G,B;
	switch(mode)
	{
		case 'u':
		{
			for( yy = y; yy < 18 + y; yy++ )
			{
				for( xx = x; xx < 26 + x; xx++ )
				{
					color = gImage_flappybird_bird_u[((yy-y)*26+(xx-x))*2+1];
					color <<= 8;
					color = color + gImage_flappybird_bird_u[((yy-y)*26+(xx-x))*2];
					R = color & 0xf800;
					R >>= 11;
					G = color & 0x07e0;
					G >>= 5;
					B = color & 0x001f;
					if((B - R) > 7)
						;
					else
					{
						color = ((color&0x1f)<<11) + (color&0x07e0) + ((color&0xf800)>>11);
						FlappyBird_Frame_DrawPoint(xx,yy,color);
					}
				}
			}
		}
		break;
		case 'm':
		{
			for( yy = y; yy < 18 + y; yy++ )
			{
				for( xx = x; xx < 26 + x; xx++ )
				{
					color = gImage_flappybird_bird_m[((yy-y)*26+(xx-x))*2+1];
					color <<= 8;
					color = color + gImage_flappybird_bird_m[((yy-y)*26+(xx-x))*2];
					R = color & 0xf800;
					R >>= 11;
					G = color & 0x07e0;
					G >>= 5;
					B = color & 0x001f;
					if((B - R) > 7)
						;
					else
					{
						color = ((color&0x1f)<<11) + (color&0x07e0) + ((color&0xf800)>>11);
						FlappyBird_Frame_DrawPoint(xx,yy,color);
					}
				}
			}
		}
		break;
		case 'd':
		{
			for( yy = y; yy < 18 + y; yy++ )
			{
				for( xx = x; xx < 26 + x; xx++ )
				{
					color = gImage_flappybird_bird_d[((yy-y)*26+(xx-x))*2+1];
					color <<= 8;
					color = color + gImage_flappybird_bird_d[((yy-y)*26+(xx-x))*2];
					R = color & 0xf800;
					R >>= 11;
					G = color & 0x07e0;
					G >>= 5;
					B = color & 0x001f;
					if((B - R) > 7)
						;
					else
					{
						color = ((color&0x1f)<<11) + (color&0x07e0) + ((color&0xf800)>>11);
						FlappyBird_Frame_DrawPoint(xx,yy,color);
					}
				}
			}
		}
		break;
		case 'f':
		{
			for( yy = y; yy < 26 + y; yy++ )
			{
				for( xx = x; xx < 18 + x; xx++ )
				{
					color = gImage_flappybird_bird_m[((17-(xx-x))*26+(yy-y))*2+1];
					color <<= 8;
					color = color + gImage_flappybird_bird_m[((17-(xx-x))*26+(yy-y))*2];
					R = color & 0xf800;
					R >>= 11;
					G = color & 0x07e0;
					G >>= 5;
					B = color & 0x001f;
					if((B - R) > 7)
						;
					else
					{
						color = ((color&0x1f)<<11) + (color&0x07e0) + ((color&0xf800)>>11);
						FlappyBird_Frame_DrawPoint(xx,yy,color);
						
					}
				}
			}
		}
		break;
		default:
			break;
	}
}

void FlappyBird_Frame_DrawBird_Play( u16 x, u16 y, u8 mode )
{
	u16 xx,yy;
	if( 0 != l_x || 0 != l_y )
	{
		for( yy = l_y; yy < l_y+18; yy++ )
		{
			for( xx = l_x; xx < l_x+26; xx++ )
			{
				FlappyBird_Frame_DrawPoint(xx,yy,FlappyBird_Bird_Buff[yy-l_y][xx-l_x]);
			}
		}
	}
	for( yy = y; yy < y+18; yy++ )
	{
		for( xx = x; xx < x+26; xx++ )
		{
			FlappyBird_Bird_Buff[yy-y][xx-x] = FlappyBird_Frame[yy][xx];
		}
	}
	FlappyBird_Frame_DrawBird(x,y,mode);
	l_x = x;
	l_y = y;
}

//void FlappyBird_Frame_DrawBird_test( u16 x, u16 y , u8 ang )////旋转函数测试不成功,矩阵操作
//{
//	int xx,yy;
//	int xx_t,yy_t;
//	double temp;
//	u16 color;
//	int R,G,B;
//			for( yy = y; yy < 18 + y; yy++ )
//			{
//				for( xx = x; xx < 26 + x; xx++ )
//				{
//					color = gImage_flappybird_bird_m[((yy-y)*26+(xx-x))*2+1];
//					color <<= 8;
//					color = color + gImage_flappybird_bird_m[((yy-y)*26+(xx-x))*2];
//					R = color & 0xf800;
//					R >>= 11;
//					G = color & 0x07e0;
//					G >>= 5;
//					B = color & 0x001f;
//					if((B - R) > 7)
//						;
//					else
//					{
//						temp = (double)(xx-x)*cos((double)(ang)) + (double)(yy-y)*sin((double)(ang));
//						xx_t = (int)(temp)+xx;
//						temp = (double)(yy-y)*cos((double)(ang)) - (double)(xx-x)*sin((double)(ang));
//						yy_t = (int)(temp)+yy;
//						FlappyBird_Frame_DrawPoint(xx_t,yy_t,color);
//					}
//				}
//			}
//}
u16 FlappyBird_Frame_MixColor(u16 color_1,u16 color_2,u8 percent)//进行帧混色操作,0-100混色等级
{
	u16 R_1,R_2;
	u16 G_1,G_2;
	u16 B_1,B_2;
	u16 R_t,G_t,B_t;
	if( percent > 100 )
	{
		;
	}
	else
	{
		R_1 = color_1 & 0xf800;
		R_1 >>= 11;
		G_1 = color_1 & 0x07e0;
		G_1 >>= 5;
		B_1 = color_1 & 0x001f;
		
		R_2 = color_2 & 0xf800;
		R_2 >>= 11;
		G_2 = color_2 & 0x07e0;
		G_2 >>= 5;
		B_2 = color_2 & 0x001f;
		
		R_t = R_1*percent/100 + R_2*(100-percent)/100;
		G_t = G_1*percent/100 + G_2*(100-percent)/100;
		B_t = B_1*percent/100 + B_2*(100-percent)/100;
		
		return LCD_DecToRGB((u8)R_t,(u8)G_t,(u8)B_t);
	}
	return 0xfe00;
}

u16 FlappyBird_Frame_IntensityControl(u16 color,u8 percent,u8 mode) //调整亮度,0模式是调暗,1模式是调亮
{
	switch(mode)
	{
		case 0:
			return FlappyBird_Frame_MixColor(color,0x0000,percent);
		case 1:
			return FlappyBird_Frame_MixColor(color,0xffff,percent);
		default:
			return 0xfe00;
	}
	
}

void FlappyBird_Frame_AllIntensityControl_1( u8 percent, u8 mode )
{
	u16 x,y;
	for( y = 0; y < 320; y++ )
	{
		for( x = 0; x < 240; x++ )
		{
			FlappyBird_Frame[y][x] = FlappyBird_Frame_IntensityControl(FlappyBird_Frame[y][x],percent,mode);
		}
	}
}

void FlappyBird_Frame_AllIntensityControl( u8 percent, u8 mode )
{
	LCD_BLPWM_VAL= percent*2;
}

void FlappyBird_Frame_DrawTag( u16 x, u16 y  )
{
		u16 color,xx = x,yy = y;
		int R,G,B;
		for( yy = y; yy < 98 + y; yy++ )
		{
			for( xx = x; xx < 114 + x; xx++ )
			{
				color = gImage_flappybird_tag[((yy-y)*114+(xx-x))*2+1];
				color <<= 8;
				color = color + gImage_flappybird_tag[((yy-y)*114+(xx-x))*2];
				R = color & 0xf800;
				R >>= 11;
				G = color & 0x07e0;
				G >>= 5;
				B = color & 0x001f;
				if((B - R) > 7)
					;
				else
				{
					color = ((color&0x1f)<<11) + (color&0x07e0) + ((color&0xf800)>>11);
					FlappyBird_Frame_DrawPoint(xx,yy,color);
				}
			}
		}
}

void FlappyBird_Frame_DrawGetStart( u16 x, u16 y )
{
		u16 color,xx = x,yy = y;
		int R,G,B;
		for( yy = y; yy < 50 + y; yy++ )
		{
			for( xx = x; xx < 184 + x; xx++ )
			{
				color = gImage_flappybird_getstart[((yy-y)*184+(xx-x))*2+1];
				color <<= 8;
				color = color + gImage_flappybird_getstart[((yy-y)*184+(xx-x))*2];
				R = color & 0xf800;
				R >>= 11;
				G = color & 0x07e0;
				G >>= 5;
				B = color & 0x001f;
				if((B - R) > 7)
					;
				else
				{
					color = ((color&0x1f)<<11) + (color&0x07e0) + ((color&0xf800)>>11);
					FlappyBird_Frame_DrawPoint(xx,yy,color);
				}
			}
		}
}

void FlappyBird_Frame_DrawColumn_bottom( u16 x, u16 y, u8 mode, u8 column )
{
	
		u16 color,xx = x,yy = y;
		int R,G,B;
		if( 0 == mode )
		{
			for( yy = y; yy < 21 + y; yy++ )
			{
				for( xx = x; xx < 39 + x; xx++ )
				{
					color = gImage_flappybird_column_bottom[((yy-y)*39+(xx-x))*2+1];
					color <<= 8;
					color = color + gImage_flappybird_column_bottom[((yy-y)*39+(xx-x))*2];
					R = color & 0xf800;
					R >>= 11;
					G = color & 0x07e0;
					G >>= 5;
					B = color & 0x001f;
					if((B - R) > 7)
						;
					else
						FlappyBird_column_Buff[column][yy][xx] = color;
				}
			}
		}
		else
		{
			for( yy = y + 20; yy >= y; yy-- )
			{
				for( xx = x; xx < 39 + x; xx++ )
				{
					color = gImage_flappybird_column_bottom[((yy-y)*39+(xx-x))*2+1];
					color <<= 8;
					color = color + gImage_flappybird_column_bottom[((yy-y)*39+(xx-x))*2];
					R = color & 0xf800;
					R >>= 11;
					G = color & 0x07e0;
					G >>= 5;
					B = color & 0x001f;
					if((B - R) > 7)
						;
					else
						FlappyBird_column_Buff[column][2*y+20-yy][xx] = color;
				}
			}
		}
}

void FlappyBird_Frame_DrawColumn_middle( u16 x, u16 y, u8 column )
{
	u16 color,xx = x,yy = y;
	int R,G,B;
	for( yy = y; yy < y + 7; yy++ )
	{
		for( xx = x; xx < 35 + x; xx++ )
		{
			color = gImage_flappybird_column_middle[((yy-y)*35+(xx-x))*2+1];
			color <<= 8;
			color = color + gImage_flappybird_column_middle[((yy-y)*35+(xx-x))*2];
			R = color & 0xf800;
			R >>= 11;
			G = color & 0x07e0;
			G >>= 5;
			B = color & 0x001f;
			if((B - R) > 7)
				;
			else
				FlappyBird_column_Buff[column][yy][xx] = color;
		}
	}
}

void FlappyBird_Frame_DrawColumn( int x, u16 y, u8 level, u8 column )//level指上层Column_middle个数,范围是0-21
{
	u16 i,yy;
	int xx;
	for( i = 0; i < level; i++ )
	{
		FlappyBird_Frame_DrawColumn_middle(2,i*7,column);
	}
	
	FlappyBird_Frame_DrawColumn_bottom(0,i*7,0,column);
	
	FlappyBird_Frame_DrawColumn_bottom(0,i*7+94,1,column);
	
	for( i = 0; i < 36-level; i++ )
	{
		FlappyBird_Frame_DrawColumn_middle(2,115+i*7+level*7,column);
	}
	if(x > -42 && x < 0)
	{
		for( yy = y; yy < 359 + y; yy++ )
		{
			for( xx = -x; xx < 39; xx++ )
			{
				if(FlappyBird_column_Buff[column][yy-y][xx]!=0xfe)
				{
					FlappyBird_Frame_DrawPoint(xx+x,yy,FlappyBird_column_Buff[column][yy-y][xx]);
				}
			}
		}
	}
	else
	{
		if(x > 281 && x < 319)
		{
			for( yy = y; yy < 359 + y; yy++ )
			{
				for( xx = x; xx < 320; xx++ )
				{
					if(FlappyBird_column_Buff[column][yy-y][xx-x]!=0xfe)
						FlappyBird_Frame_DrawPoint(xx,yy,FlappyBird_column_Buff[column][yy-y][xx-x]);
				}
			}
		}
		else if(x < 281 && x > 0)
		{
			for( yy = y; yy < 359 + y; yy++ )
			{
				for( xx = x; xx < 39 + x; xx++ )
				{
					FlappyBird_Frame_DrawPoint(xx,yy,FlappyBird_column_Buff[column][yy-y][xx-x]);
				}
			}
		}
	}

}

void FlappyBird_Frame_DrawColumn_Play( int x, u16 y, u8 level, u8 column )
{
	u16 xx,yy;
	if( hide[column] )
	{
		for( yy = l_y_c[column]; yy < l_y_c[column] + 359; yy++ )
		{
			for( xx = l_x_c[column]; xx < l_x_c[column] + 39; xx++ )
			{
				if( yy >= level*7+21 && yy <= level*7+85 )
				{
					;
				}
				else
					FlappyBird_Frame_DrawPoint(xx,yy,FlappyBird_column_Buff_BG[column][yy-l_y_c[column]][xx-l_x_c[column]]);
			}
		}
	}
	if( x > 281 )
	{
		for( yy = y; yy < y+359; yy++ )
		{
			for( xx = 281; xx < 320; xx++ )
			{
				FlappyBird_column_Buff_BG[column][yy-y][xx-281] = FlappyBird_Frame[yy][xx];
				FlappyBird_column_Buff[column][yy-y][xx-281] = 0xfe;
			}
		}
	}
	else
	{
		if( x < 0 )
		{
			for( yy = y; yy < y+359; yy++ )
			{
				for( xx = 0; xx < 39; xx++ )
				{
					FlappyBird_column_Buff_BG[column][yy-y][xx] = FlappyBird_Frame[yy][xx];
					FlappyBird_column_Buff[column][yy-y][xx] = 0xfe;
				}
			}
		}
		else
		{
			for( yy = y; yy < y+359; yy++ )
			{
				for( xx = x; xx < x+39; xx++ )
				{
					FlappyBird_column_Buff_BG[column][yy-y][xx-x] = FlappyBird_Frame[yy][xx];
					FlappyBird_column_Buff[column][yy-y][xx-x] = FlappyBird_Frame[yy][xx];
				}
			}
		}
	}
	FlappyBird_Frame_DrawColumn(x,y,level,column);
	if( x < 0 )
	{
		l_x_c[column] = 0;
	}
	else
	{
		if( x > 281 )
		{
			l_x_c[column] = 281;
		}
		else
		{
			l_x_c[column] = x;
		}
	}
	l_y_c[column] = y;
	hide[column] = 1;
}

void FlappyBird_Frame_DrawScoreBoard( u16 x, u16 y )
{
		u16 color,xx = x,yy = y;
		int R,G,B;
		for( yy = y; yy < 114 + y; yy++ )
		{
			for( xx = x; xx < 226 + x; xx++ )
			{
				color = gImage_flappybird_score_board[((yy-y)*226+(xx-x))*2+1];
				color <<= 8;
				color = color + gImage_flappybird_score_board[((yy-y)*226+(xx-x))*2];
				R = color & 0xf800;
				R >>= 11;
				G = color & 0x07e0;
				G >>= 5;
				B = color & 0x001f;
				if((B - R) > 7)
					;
				else
				{
					color = ((color&0x1f)<<11) + (color&0x07e0) + ((color&0xf800)>>11);
					FlappyBird_Frame_DrawPoint(xx,yy,color);
				}
			}
		}
}

void FlappyBird_Frame_DrawGameOver( u16 x, u16 y )
{
		u16 color,xx = x,yy = y;
		int R,G,B;
		for( yy = y; yy < 42 + y; yy++ )
		{
			for( xx = x; xx < 192 + x; xx++ )
			{
				color = gImage_flappybird_gameover[((yy-y)*192+(xx-x))*2+1];
				color <<= 8;
				color = color + gImage_flappybird_gameover[((yy-y)*192+(xx-x))*2];
				R = color & 0xf800;
				R >>= 11;
				G = color & 0x07e0;
				G >>= 5;
				B = color & 0x001f;
				if((B - R) > 7)
					;
				else
				{
					color = ((color&0x1f)<<11) + (color&0x07e0) + ((color&0xf800)>>11);
					FlappyBird_Frame_DrawPoint(xx,yy,color);
				}
			}
		}
}

void FlappyBird_Frame_DrawColumn_Play_times( int x, u16 y, u8 level, u8 column, u8 times )
{
	u8 i;
	
	for( i =0; i < times; i++ )
	{
		FlappyBird_Frame_DrawColumn_Play(x-i,y,level,column);
	}
}

void FlappyBird_Stage_Start( void )
{
	u8 x;
	u8 Start_Flag = 1;
	u8 Key;
	bird_height = 200;
	FlappyBird_Frame_Clear();
	FlappyBird_DrawGreenBar(0,360,0x5F33);
	FlappyBird_Frame_Fill(0,358,319,358,0x39E8);
	FlappyBird_Frame_Fill(0,359,319,359,0xB7BD);
	FlappyBird_Frame_Fill(0,360,319,360,0x8FFC);
	FlappyBird_Frame_Fill(0,373,319,373,0x1BEB);
	FlappyBird_Frame_Fill(0,374,319,374,0x4D59);
	FlappyBird_Frame_Fill(0,375,319,375,0x4D59);
	FlappyBird_Frame_Fill(0,376,319,480,0x96BB);
	FlappyBird_Frame_DrawBGPic(0,280);
	FlappyBird_Frame_DrawLogo(69,93);
	FlappyBird_Frame_DrawButton(42,300,0);
	FlappyBird_Frame_DrawButton(180,300,1);
	Font_Num_DrawNum_Multi(150,30,difficult,0);
	FlappyBird_DrawFrame();
	BACK_COLOR = 0x96BB;
	FlappyBird_Frame_ShowString(90,376,200,16,16,"(c)www.briup.com 2015",WHITE);
	while(Start_Flag)
	{
		Key=key_scan();
		if( Key == KEY_UP )
		{
			difficult++;
			if( difficult > 6 )
			{
				difficult = 6;
			}
			Font_Num_DrawNum_Multi(150,30,difficult,0);
		}
		if( Key == KEY_DOWN )
		{
			difficult--;
			if( difficult < 0 )
			{
				difficult = 0;
			}
			Font_Num_DrawNum_Multi(150,30,difficult,0);
		}
		t_pad.scan(0);
		if(t_pad.penStat&TP_PRES_DOWN)			//触摸屏被按下
		{
			lcdDisplayOn();
		 	if(t_pad.currY<Lcd_Dev.width&&t_pad.currX<Lcd_Dev.height)
			{
					if(t_pad.currY >= 42 && t_pad.currY <= 146 && t_pad.currX <= 358  && t_pad.currX >= 300)
					{
						FlappyBird_Frame_DrawBGPic(0,280);
						FlappyBird_Frame_DrawButton(43,302,0);
						FlappyBird_Frame_DrawButton(180,300,1);
						FlappyBird_DrawFrame();
						while(t_pad.penStat&TP_PRES_DOWN)
						{
							t_pad.scan(0);
							FlappyBird_DrawGreenBar_Play(0,360,5);
							FlappyBird_Frame_DrawBird_Play(106,bird_height,bird_mode);
							FlappyBird_DrawFrame();
							if( x%3 == 0 )
								switch(bird_height)
								{
									case 200: 
										bird_height = 201;
										bird_mode = 'd';
										ud_mode = 0;
										break;
									case 201: 
										if( 0 == ud_mode )
										{
											bird_height = 202;
											bird_mode = 'm';
										}
										else
										{
											bird_height = 200;
											bird_mode = 'u';
										}
										break;
									case 202: 
										if( 0 == ud_mode )
										{
											bird_height = 203;
											bird_mode = 'u';
										}
										else
										{
											bird_height = 201;
											bird_mode = 'd';
										}
										break;
									case 203: 
										if( 0 == ud_mode )
										{
											bird_height = 204;
											bird_mode = 'd';
										}
										else
										{
											bird_height = 202;
											bird_mode = 'm';
										}
										break;
									case 204: 
										bird_height = 203;
										bird_mode = 'u';
										ud_mode = 1;
										break;
								}
								x++;
								if(x == 100)
								{
									x = 0;
								}
						}
						FlappyBird_Frame_DrawBGPic(0,280);
						FlappyBird_Frame_DrawButton(42,300,0);
						FlappyBird_Frame_DrawButton(180,300,1);
						FlappyBird_Frame_Fill(0,358,319,358,0x39E8);
						FlappyBird_Frame_Fill(0,359,319,359,0xB7BD);
						FlappyBird_Frame_Fill(0,360,319,360,0x8FFC);
						FlappyBird_DrawFrame();
						for( x = 1; x <= 10; x++)
						{
							if( x%3 == 0 )
								switch(bird_height)
								{
									case 200: 
										bird_height = 201;
										bird_mode = 'd';
										ud_mode = 0;
										break;
									case 201: 
										if( 0 == ud_mode )
										{
											bird_height = 202;
											bird_mode = 'm';
										}
										else
										{
											bird_height = 200;
											bird_mode = 'u';
										}
										break;
									case 202: 
										if( 0 == ud_mode )
										{
											bird_height = 203;
											bird_mode = 'u';
										}
										else
										{
											bird_height = 201;
											bird_mode = 'd';
										}
										break;
									case 203: 
										if( 0 == ud_mode )
										{
											bird_height = 204;
											bird_mode = 'd';
										}
										else
										{
											bird_height = 202;
											bird_mode = 'm';
										}
										break;
									case 204: 
										bird_height = 203;
										bird_mode = 'u';
										ud_mode = 1;
										break;
								}
							FlappyBird_DrawGreenBar_Play(0,360,5);
							FlappyBird_Frame_DrawBird_Play(106,bird_height,bird_mode);
							FlappyBird_Frame_AllIntensityControl(100-x*10,0);
							FlappyBird_DrawFrame();
						}

						Start_Flag = 0;
					}
					if(t_pad.currY >= 180 && t_pad.currY <= 284 && t_pad.currX <= 358  && t_pad.currX >= 300)
					{
						FlappyBird_Frame_DrawBGPic(0,280);
						FlappyBird_Frame_DrawButton(42,300,0);
						FlappyBird_Frame_DrawButton(181,302,1);
						FlappyBird_DrawFrame();
						while(t_pad.penStat&TP_PRES_DOWN)
						{
							t_pad.scan(0);
							FlappyBird_DrawGreenBar_Play(0,360,5);
							FlappyBird_Frame_DrawBird_Play(106,bird_height,bird_mode);
							FlappyBird_DrawFrame();
							if( x%3 == 0 )
								switch(bird_height)
								{
									case 200: 
										bird_height = 201;
										bird_mode = 'd';
										ud_mode = 0;
										break;
									case 201: 
										if( 0 == ud_mode )
										{
											bird_height = 202;
											bird_mode = 'm';
										}
										else
										{
											bird_height = 200;
											bird_mode = 'u';
										}
										break;
									case 202: 
										if( 0 == ud_mode )
										{
											bird_height = 203;
											bird_mode = 'u';
										}
										else
										{
											bird_height = 201;
											bird_mode = 'd';
										}
										break;
									case 203: 
										if( 0 == ud_mode )
										{
											bird_height = 204;
											bird_mode = 'd';
										}
										else
										{
											bird_height = 202;
											bird_mode = 'm';
										}
										break;
									case 204: 
										bird_height = 203;
										bird_mode = 'u';
										ud_mode = 1;
										break;
								}
								x++;
								if(x == 100)
								{
									x = 0;
								}
						}
						FlappyBird_Frame_DrawBGPic(0,280);
						FlappyBird_Frame_DrawButton(42,300,0);
						FlappyBird_Frame_DrawButton(180,300,1);
						FlappyBird_Frame_Fill(0,358,319,358,0x39E8);
						FlappyBird_Frame_Fill(0,359,319,359,0xB7BD);
						FlappyBird_Frame_Fill(0,360,319,360,0x8FFC);

						FlappyBird_DrawFrame();
						AT24CXX_WriteLenByte(132,(u32)0x00,1);
						lcdDisplayOn();
					}
			}
		}
		if( Start_Flag != 0 )
		{
			
			FlappyBird_DrawGreenBar_Play(0,360,5);
			FlappyBird_Frame_DrawBird_Play(106,bird_height,bird_mode);
			FlappyBird_DrawFrame();
			if( x%3 == 0 )
				switch(bird_height)
								{
									case 200: 
										bird_height = 201;
										bird_mode = 'd';
										ud_mode = 0;
										break;
									case 201: 
										if( 0 == ud_mode )
										{
											bird_height = 202;
											bird_mode = 'm';
										}
										else
										{
											bird_height = 200;
											bird_mode = 'u';
										}
										break;
									case 202: 
										if( 0 == ud_mode )
										{
											bird_height = 203;
											bird_mode = 'u';
										}
										else
										{
											bird_height = 201;
											bird_mode = 'd';
										}
										break;
									case 203: 
										if( 0 == ud_mode )
										{
											bird_height = 204;
											bird_mode = 'd';
										}
										else
										{
											bird_height = 202;
											bird_mode = 'm';
										}
										break;
									case 204: 
										bird_height = 203;
										bird_mode = 'u';
										ud_mode = 1;
										break;
								}
				x++;
				if(x == 100)
				{
					x = 0;
				}
			}
		}
}

void FlappyBird_Stage_PP( void )
{
	u8 x;
	u8 Start_Flag = 1;
	bird_height = bird_height+30;
	FlappyBird_Frame_Clear();
	FlappyBird_DrawGreenBar(0,360,0x5F33);
	FlappyBird_Frame_Fill(0,358,319,358,0x39E8);
	FlappyBird_Frame_Fill(0,359,319,359,0xB7BD);
	FlappyBird_Frame_Fill(0,360,319,360,0x8FFC);
	FlappyBird_Frame_Fill(0,373,319,373,0x1BEB);
	FlappyBird_Frame_Fill(0,374,319,374,0x4D59);
	FlappyBird_Frame_Fill(0,375,319,375,0x4D59);
	FlappyBird_Frame_Fill(0,376,319,480,0x96BB);
	FlappyBird_Frame_DrawBGPic(0,280);
	for( x = 1; x <= 10; x++)
	{
		if( x%3 == 0 )
			switch(bird_height)
								{
									case 230: 
										bird_height = 231;
										bird_mode = 'd';
										ud_mode = 0;
										break;
									case 231: 
										if( 0 == ud_mode )
										{
											bird_height = 232;
											bird_mode = 'm';
										}
										else
										{
											bird_height = 230;
											bird_mode = 'u';
										}
										break;
									case 232: 
										if( 0 == ud_mode )
										{
											bird_height = 233;
											bird_mode = 'u';
										}
										else
										{
											bird_height = 231;
											bird_mode = 'd';
										}
										break;
									case 233: 
										if( 0 == ud_mode )
										{
											bird_height = 234;
											bird_mode = 'd';
										}
										else
										{
											bird_height = 232;
											bird_mode = 'm';
										}
										break;
									case 234: 
										bird_height = 233;
										bird_mode = 'u';
										ud_mode = 1;
										break;
								}
		FlappyBird_DrawGreenBar_Play(0,360,5);
		FlappyBird_Frame_DrawBird_Play(57,bird_height,bird_mode);
		FlappyBird_Frame_DrawGetStart(68,70);
		FlappyBird_Frame_DrawTag(103,120);
		FlappyBird_Frame_AllIntensityControl(x*20,0);
		FlappyBird_DrawFrame();
	}
	while(Start_Flag)
	{
		t_pad.scan(0); 		 
		if(t_pad.penStat&TP_PRES_DOWN)			//触摸屏被按下
		{	
			Start_Flag = 0;
		}
		if(x > 10)
		{
			x = 0;
		}
		if(Start_Flag)
		{
			if( x%3 == 0 )
				switch(bird_height)
				{
					case 230: 
						bird_height = 231;
						bird_mode = 'd';
						ud_mode = 0;
						break;
					case 231: 
						if( 0 == ud_mode )
						{
							bird_height = 232;
							bird_mode = 'm';
						}
						else
						{
							bird_height = 230;
							bird_mode = 'u';
						}
						break;
					case 232: 
						if( 0 == ud_mode )
						{
							bird_height = 233;
							bird_mode = 'u';
						}
						else
						{
							bird_height = 231;
							bird_mode = 'd';
						}
						break;
					case 233: 
						if( 0 == ud_mode )
						{
							bird_height = 234;
							bird_mode = 'd';
						}
						else
						{
							bird_height = 232;
							bird_mode = 'm';
						}
						break;
					case 234: 
						bird_height = 233;
						bird_mode = 'u';
						ud_mode = 1;
						break;
				}
			FlappyBird_DrawGreenBar_Play(0,360,5);
			FlappyBird_Frame_DrawBird_Play(57,bird_height,bird_mode);
			FlappyBird_DrawFrame();
			x++;
		}
	}
	for( x = 1; x <= 10; x++ )
	{
		FlappyBird_Frame_AllIntensityControl(100-x*10,0);
		_delay_ms(30);
	}
}

void FlappyBird_Stage_Play( void )
{
	int x,round;
	int x_c[3];
	u8 level;
	u16 adcx;
	
	u8 Dead_Flag = 1;
	FlappyBird_Clock_Set(16);
	//FSMC设置
	FSMC_Bank1E->BWTR[6]|=2<<0; 	 
	FSMC_Bank1E->BWTR[6]|=3<<8;
 	//SRAM设置
	FSMC_Bank1->BTCR[5]|=6<<8;
	FlappyBird_Frame_Clear();
	FlappyBird_DrawGreenBar(0,360,0x5F33);
	FlappyBird_Frame_Fill(0,358,319,358,0x39E8);
	FlappyBird_Frame_Fill(0,359,319,359,0xB7BD);
	FlappyBird_Frame_Fill(0,360,319,360,0x8FFC);
	FlappyBird_Frame_Fill(0,373,319,373,0x1BEB);
	FlappyBird_Frame_Fill(0,374,319,374,0x4D59);
	FlappyBird_Frame_Fill(0,375,319,375,0x4D59);
	FlappyBird_Frame_Fill(0,376,319,480,0x96BB);
	FlappyBird_Frame_DrawBGPic(0,280);
	adcx=Get_Adc_Average(0x01,10);
	srand((unsigned)adcx);
	level = rand()%36;
	srand((unsigned)adcx*30);
	level = (level + rand())%36;
	round = level;
	FlappyBird_Frame_DrawColumn_Play_times(319,0,level,0,1);
	Font_Num_DrawNum_Multi(150,50,0,0);
	FlappyBird_DrawFrame();
	for( x = 1; x <= 10; x++ )
	{
		FlappyBird_Frame_AllIntensityControl(x*10,0);
		_delay_ms(30);
	}
	for( x = 0; x<= 10; x++)
	{
			if( x%3 == 0 )
			switch(bird_height)
			{
				case 230: 
					bird_height = 231;
					bird_mode = 'd';
					ud_mode = 0;
					break;
				case 231: 
					if( 0 == ud_mode )
					{
						bird_height = 232;
						bird_mode = 'm';
					}
					else
					{
						bird_height = 230;
						bird_mode = 'u';
					}
					break;
				case 232: 
					if( 0 == ud_mode )
					{
						bird_height = 233;
						bird_mode = 'u';
					}
					else
					{
						bird_height = 231;
						bird_mode = 'd';
					}
					break;
				case 233: 
					if( 0 == ud_mode )
					{
						bird_height = 234;
						bird_mode = 'd';
					}
					else
					{
						bird_height = 232;
						bird_mode = 'm';
					}
					break;
				case 234: 
					bird_height = 233;
					bird_mode = 'u';
					ud_mode = 1;
					break;
			}
			FlappyBird_DrawGreenBar_Play(0,360,7);
			FlappyBird_Frame_DrawBird_Play(57,bird_height,bird_mode);
			FlappyBird_DrawFrame();
		}
		x = 0;
		x_c[0] = 316;
		while(Dead_Flag)
		{
			t_pad.scan(0); 		 
			if(t_pad.penStat&TP_PRES_DOWN)			//触摸屏被按下
			{
				FlappyBird_Frame_DrawBird_Play(57,bird_height,'u');
				bird_height = bird_height - 9 - difficult;
				bird_mode = 'd';
				FlappyBird_DrawGreenBar_Play(0,360,7);
				
				if(bird_height < 0)
				{
					bird_height = 0;
				}
				
				FlappyBird_Frame_DrawColumn_Play_times(x_c[0],0,level,0,1);
				if( x_c[0] >= 18 && x_c[0] <= 82)
				{
					if(!((bird_height > 7*level+21) && (bird_height < 7*level+74)))
					{
						Dead_Flag = 0;
					}
					else if( 0 == score_lock )
					{
						score_lock = 1;
						score ++;
						BEEP_ON();
						led_setG(LED_ON);
					}
					
				}
				Font_Num_DrawNum_Multi(150,50,score,0);
				FlappyBird_DrawFrame();
				x_c[0] = x_c[0] - 7;
				if(x_c[0] < -42)
				{
					adcx=Get_Adc_Average(0x01,10);
					srand((unsigned)adcx);
					level = rand()%36;
					srand((unsigned)adcx*20);
					level = (level + rand())%((rand()+20)%36);
					while(round - level > -5 && round - level < 5)
						level = (level+rand()%36)%36;
					round = level;
					x_c[0] = 319;
					score_lock = 0;
					
				}
				BEEP_OFF();
				led_setG(LED_OFF);
			}
			else if(bird_height < 331 && bird_mode != 'f' )
			{
				bird_mode = 'm';
				bird_height = bird_height + 9 + difficult;
				if(bird_height > 330)
				{
					bird_height = 330;
					bird_mode = 'f';
					Dead_Flag = 0;
				}
			}
			if(bird_mode != 'f')
			FlappyBird_DrawGreenBar_Play(0,360,7);
			FlappyBird_Frame_DrawColumn_Play_times(x_c[0],0,level,0,1);
			FlappyBird_Frame_DrawBird_Play(bird_wide,bird_height,bird_mode);
			if( x_c[0] >= 18 && x_c[0] <= 82)
			{
				if(!((bird_height > 7*level+21) && (bird_height < 7*level+74)))
				{
					Dead_Flag = 0;
				}
				else if( 0 == score_lock )
				{
					score_lock = 1;
					score ++;
					BEEP_ON();
					led_setG(LED_ON);
				/*	FlappyBird_Frame_ClearBird();
					FlappyBird_Frame_DrawBird_Play(bird_wide,bird_height,bird_mode);*/
				}
			}
			Font_Num_DrawNum_Multi(150,50,score,0);
			FlappyBird_DrawFrame();
			x_c[0] = x_c[0] - 7;
			if(x_c[0] < -42)
			{
				adcx=Get_Adc_Average(0x01,10);
				srand((unsigned)adcx);
				level = rand()%33;
				srand((unsigned)adcx*15);
				level = (level + rand())%33;
				while(round - level > -5 && round - level < 5)
						level = (level+rand()%36)%36;
					round = level;
				x_c[0] = 319;
				score_lock = 0;
			}
			BEEP_OFF();
			led_setG(LED_OFF);
		}
}

void FlappyBird_Stage_Over( void )
{
	u16 i;
	u32 best_score = 0;
	u8 Over_Flag = 1;
	u8 AT24CXX_FLAG = 0;
	u8 x;
	FlappyBird_Clock_Set(9);
	FSMC_SRAM_init();
	DMA_Init();
	FlappyBird_Frame_DrawGameOver(64,80);
	FlappyBird_Frame_DrawScoreBoard(40,140);
	FlappyBird_Frame_DrawButton(42,300,0);
	FlappyBird_Frame_DrawButton(180,300,1);
	FlappyBird_DrawFrame();
	
	AT24CXX_FLAG = (u8)AT24CXX_ReadLenByte(132,1);
	if( AT24CXX_FLAG != 0x55 )
	{
		AT24CXX_WriteLenByte(112,(u32)0x0000,2);
		//printf("\r\n%ld\r\n",AT24CXX_ReadLenByte(112,2));
		AT24CXX_WriteLenByte(132,(u32)0x55,1);
	}
	best_score = AT24CXX_ReadLenByte(112,2);

	//best_score = score;
	if( score > best_score )
	{
		AT24CXX_WriteLenByte(112,(u32)score,2);
	}
	for( i = 0; i <= ((score)> (best_score) ? (score):(best_score)); i++ )
	{
		if( i <= score )
		{
			Font_Num_DrawNum_Multi(240,175,i,1);
		}
		Font_Num_DrawNum_Multi(240,215,i,1);
		FlappyBird_DrawFrame();
		_delay_ms(30);
	}
	score = 0;
	
	while(Over_Flag)
	{
		t_pad.scan(0);
		if(t_pad.penStat&TP_PRES_DOWN)			//触摸屏被按下
		{	
			lcdDisplayOn();
		 	if(t_pad.currY<Lcd_Dev.width&&t_pad.currX<Lcd_Dev.height)
			{
					if(t_pad.currY >= 42 && t_pad.currY <= 146 && t_pad.currX <= 358  && t_pad.currX >= 300)
					{
						FlappyBird_Frame_DrawButton(43,302,0);
						FlappyBird_Frame_DrawButton(180,300,1);
						while(t_pad.penStat&TP_PRES_DOWN && Over_Flag)
						{
							t_pad.scan(0);
							for( x = 1; x <= 10; x++)
							{
								FlappyBird_Frame_AllIntensityControl(100-x*10,0);
								_delay_ms(30);
							}

							Over_Flag = 0;
						}
					}
					if(t_pad.currY >= 180 && t_pad.currY <= 284 && t_pad.currX <= 358  && t_pad.currX >= 300)
					{
						FlappyBird_Frame_Fill(0,358,319,358,0x39E8);
						FlappyBird_Frame_Fill(0,359,319,359,0xB7BD);
						FlappyBird_Frame_Fill(0,360,319,360,0x8FFC);
						FlappyBird_Frame_DrawButton(42,300,0);
						FlappyBird_Frame_DrawButton(181,302,1);
						FlappyBird_DrawFrame();
						while(t_pad.penStat&TP_PRES_DOWN)
						{
							t_pad.scan(0);
							lcdDisplayOn();
						}
						AT24CXX_WriteLenByte(132,(u32)0x00,1);
					}
			}
		}
		FlappyBird_Frame_DrawButton(42,300,0);
		FlappyBird_Frame_DrawButton(180,300,1);
		ud_mode = 0;
		bird_mode = 'u';
		bird_height = 119;
		bird_wide = 57;
		score_lock = 0;
		l_x = 0;
		l_y = 0;
		hide[0] = 0;
		FlappyBird_DrawFrame();
	}
}

//==================================================================================================================


void FlappyBird_DrawFrame(void)
{
	/*u16 i,j,tmp;
	for(i = 0;i < 480;i++)
		for(j = 0;j < 320;j++)
		{
			tmp = FlappyBird_Frame[i][j]; 
			FlappyBird_Frame[i][j] = ((tmp&0x1f)<<11) + (tmp&0x07e0) + ((tmp&0xf800)>>11);
		}*/
	lcdSetWindow(0,0,320,480);
	DMA_Enable();
}

void FlappyBird_Clock_Set(u8 PLL)
{
	u8 tPLL=PLL;
	u8 temp=0;	 
	RCC->CFGR&=0XFFFFFFFC;	//修改时钟频率为内部8M
	RCC->CR&=~0x01000000;  	//PLLOFF  
 	RCC->CFGR&=~(0XF<<18);	//清空原来的设置
 	PLL-=2;//抵消2个单位
	RCC->CFGR|=PLL<<18;   	//设置PLL值 2~16
	RCC->CFGR|=1<<16;	  	//PLLSRC ON 
	FLASH->ACR|=0x12;	  	//FLASH 2个延时周期
 	RCC->CR|=0x01000000;  	//PLLON
	while(!(RCC->CR>>25));	//等待PLL锁定
	RCC->CFGR|=0x02;		//PLL作为系统时钟	 
	while(temp!=0x02)    	//等待PLL作为系统时钟设置成功
	{   
		temp=RCC->CFGR>>2;
		temp&=0x03;
	}  
 	//顺便设置延时和串口										  
	_delay_init(tPLL*8);		//延时初始化
	usart1_init(tPLL*8,115200); //串口1初始化   
}


