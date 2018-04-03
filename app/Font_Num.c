#include "Font_Num.h"
#include "0_big.c"
#include "0_small.c"
#include "1_big.c"
#include "1_small.c"
#include "2_big.c"
#include "2_small.c"
#include "3_big.c"
#include "3_small.c"
#include "4_big.c"
#include "4_small.c"
#include "5_big.c"
#include "5_small.c"
#include "6_big.c"
#include "6_small.c"
#include "7_big.c"
#include "7_small.c"
#include "8_big.c"
#include "8_small.c"
#include "9_big.c"
#include "9_small.c"

void Font_Num_DrawNum_Multi( u16 x, u16 y, u32 num, u8 mode )
{
	u16 bit_Num,i;
	u32 num_t;
	u8 oneBit_Num;
	num_t = num;
	
	bit_Num = 0;
	while(num_t)
	{
		num_t /= 10;
		bit_Num++;
	}
	if( 0 == num)
	{
		bit_Num = 1;
	}
	
	if( 0 == mode)
	{
		FlappyBird_Frame_Fill(x + bit_Num*10-(bit_Num-1)*20,y,x + bit_Num*10,y+30,0xEED3);
		num_t = num;
		for( i = 0; i < bit_Num; i++ )
		{
			oneBit_Num = num_t%10;
			Font_Num_DrawNum(x + bit_Num*10-i*20,y,oneBit_Num,0);
			num_t /= 10;
		}
		
	}
	else
	{
		num_t = num;
		for( i = 0; i < bit_Num; i++ )
		{
			oneBit_Num = num_t%10;
			Font_Num_DrawNum(x-i*11,y,oneBit_Num,1);
			num_t /= 10;
		}
	}
	
}

void Font_Num_DrawNum( u16 x, u16 y, u16 num, u8 mode )//大小两种模式，数字在0-9之间
{

		if(0 == mode)
		{
			switch(num)
			{
				case 0:
					Font_DrawNum_Big(x,y,(u8 *)gImage_0_big);
					break;
				case 1:
					Font_DrawNum_Big(x,y,(u8 *)gImage_1_big);
					break;
				case 2:
					Font_DrawNum_Big(x,y,(u8 *)gImage_2_big);
					break;
				case 3:
					Font_DrawNum_Big(x,y,(u8 *)gImage_3_big);
					break;
				case 4:
					Font_DrawNum_Big(x,y,(u8 *)gImage_4_big);
					break;
				case 5:
					Font_DrawNum_Big(x,y,(u8 *)gImage_5_big);
					break;
				case 6:
					Font_DrawNum_Big(x,y,(u8 *)gImage_6_big);
					break;
				case 7:
					Font_DrawNum_Big(x,y,(u8 *)gImage_7_big);
					break;
				case 8:
					Font_DrawNum_Big(x,y,(u8 *)gImage_8_big);
					break;
				case 9:
					Font_DrawNum_Big(x,y,(u8 *)gImage_9_big);
					break;
			}
		}
		else
		{
			switch(num)
			{
				case 0:
					Font_DrawNum_Small(x,y,(u8 *)gImage_0_small);
					break;
				case 1:
					Font_DrawNum_Small(x,y,(u8 *)gImage_1_small);
					break;
				case 2:
					Font_DrawNum_Small(x,y,(u8 *)gImage_2_small);
					break;
				case 3:
					Font_DrawNum_Small(x,y,(u8 *)gImage_3_small);
					break;
				case 4:
					Font_DrawNum_Small(x,y,(u8 *)gImage_4_small);
					break;
				case 5:
					Font_DrawNum_Small(x,y,(u8 *)gImage_5_small);
					break;
				case 6:
					Font_DrawNum_Small(x,y,(u8 *)gImage_6_small);
					break;
				case 7:
					Font_DrawNum_Small(x,y,(u8 *)gImage_7_small);
					break;
				case 8:
					Font_DrawNum_Small(x,y,(u8 *)gImage_8_small);
					break;
				case 9:
					Font_DrawNum_Small(x,y,(u8 *)gImage_9_small);
					break;
			}
		}
}
		
void Font_DrawNum_Big( u16 x, u16 y, u8 * num )
{
	u16 color,xx = x,yy = y;
	int R,G,B;
	//Font_DrawNum_Black(x + SHADOW_OFFSET,y + SHADOW_OFFSET,num);
	
	for( yy = y; yy < 28 + y; yy++ )
	{
		for( xx = x; xx < 18 + x; xx++ )
		{
			color = num[((yy-y)*18+(xx-x))*2+1];
			color <<= 8;
			color = color + num[((yy-y)*18+(xx-x))*2];
			R = color & 0xf800;
			R >>= 11;
			G = color & 0x07e0;
			G >>= 5;
			B = color & 0x001f;
			if((B - R) > 7)
			{
				FlappyBird_Frame_DrawPoint(xx,yy,0xEED3);
			}
			else
				FlappyBird_Frame_DrawPoint(xx,yy,color);
		}
	}
}

void Font_DrawNum_Black( u16 x, u16 y, u8 * num )
{
	u16 color,xx = x,yy = y;
	int R,G,B;
	for( yy = y; yy < 28 + y; yy++ )
	{
		for( xx = x; xx < 18 + x; xx++ )
		{
			color = num[((yy-y)*18+(xx-x))*2+1];
			color <<= 8;
			color = color + num[((yy-y)*18+(xx-x))*2];
			R = color & 0xf800;
			R >>= 11;
			G = color & 0x07e0;
			G >>= 5;
			B = color & 0x001f;
			if((B - R) > 7)
			{
				;
			}
			else
				FlappyBird_Frame_DrawPoint(xx,yy,0x0000);
		}
	}

}

void Font_DrawNum_Small( u16 x, u16 y, u8 * num )
{
	u16 color,xx = x,yy = y;
	int R,G,B;

	for( yy = y; yy < 14 + y; yy++ )
	{
		for( xx = x; xx < 9 + x; xx++ )
		{
			color = num[((yy-y)*9+(xx-x))*2+1];
			color <<= 8;
			color = color + num[((yy-y)*9+(xx-x))*2];
			R = color & 0xf800;
			R >>= 11;
			G = color & 0x07e0;
			G >>= 5;
			B = color & 0x001f;
			if((B - R) > 7)
			{
				FlappyBird_Frame_DrawPoint(xx,yy,0xEED3);
			}
			else
				FlappyBird_Frame_DrawPoint(xx,yy,color);
		}
	}
}
