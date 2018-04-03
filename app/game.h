#ifndef __ENGINE_H
#define __ENGINE_H
#include "jpstm32_lcd.h"
#include "jpstm32_usart.h"
#include "jpstm32_sysclk.h"
#include "jpstm32_key.h"
#include "jpstm32_beep.h"
#include "dma.h"
#include "touch.h"
#include "timer.h"
#include "jpstm32_delay.h"
#include "adc.h"
#include "Font_Num.h"
#include "24cxx.h"
#include "fsmc.h"
#include "led.h"
#include "jpstm32_key.h"

extern const unsigned char asc2_1206[95][12];
extern const unsigned char asc2_1608[95][16];
extern u16 FlappyBird_Frame[480][320];
extern TouchTpyDef	t_pad;
void FlappyBire_Change(void);
void FlappyBird_Frame_DrawPoint( u16 x, u16 y, u16 color );
void FlappyBird_Frame_Fill( u16 x, u16 y, u16 xx, u16 yy, u16 color );
void FlappyBird_Frame_DrawLine( u16 x1, u16 y1, u16 x2, u16 y2 , u16 color );
void FlappyBird_DrawFrame( void );
void FlappyBird_Frame_Clear( void );
void FlappyBird_Frame_ClearBird(void);
void FlappyBird_Frame_ShowChar(u16 x,u16 y,u8 num,u8 size,u8 mode,u16 color);
void FlappyBird_Frame_ShowString(u16 x,u16 y,u16 width,u16 height,u8 size,u8 *p,u16 color);
void FlappyBird_Frame_DrawBGPic( u16 x,u16 y );
void FlappyBird_Frame_DrawLogo( u16 x, u16 y );
void FlappyBird_Frame_DrawButton( u16 x, u16 y , u8 mode );
void FlappyBird_Frame_DrawBird( u16 x, u16 y , u8 mode );
void FlappyBird_Frame_DrawBird_Play( u16 x, u16 y, u8 mode );
void FlappyBird_Frame_DrawColumn_Play_times( int x, u16 y, u8 level, u8 column, u8 times );
u16 FlappyBird_Frame_MixColor(u16 color_1,u16 color_2,u8 percent);
u16 FlappyBird_Frame_IntensityControl(u16 color,u8 percent,u8 mode); //调整亮度,0模式是调暗,1模式是调亮
void FlappyBird_Frame_AllIntensityControl( u8 percent, u8 mode );//全屏调亮调暗
//void FlappyBird_Frame_DrawBird_test( u16 x, u16 y , u8 ang );//旋转函数测试不成功
void FlappyBird_DrawGreenBar( u16 x, u16 y, u16 color );
void FlappyBird_DrawGreenBar_Play( u16 x, u16 y,u8 Speed );
void FlappyBird_DrawParallelogram( u16 x, u16 y, u16 color );
void FlappyBird_Clock_Set(u8 PLL);

void FlappyBird_Stage_Start( void );
void FlappyBird_Stage_PP( void );
void FlappyBird_Stage_Play( void );
void FlappyBird_Stage_Over( void );
#endif


