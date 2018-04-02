

#include "game.h"
#include "draw_bird.h"

extern const unsigned char gImage_logo[17088];//48*178
extern const unsigned char gImage_start[12064];//58*104
ImgTypeDef image_logo;
ImgTypeDef image_start;

void game_Init()
{
  	TouchTpyDef get_xy;
   
   briupNVICGroupInit(2);
	_delay_init( 72);
	basePeriphInit();
	briupUsart1Init( 57600);
	setLed( LED_ALL);
	briupKeyInit();
	FSMC_SRAM_init();
   
	briupLcdInit();
 
   tp_init();
   tp_adjust();
   briupLcdClear(WHITE);
   
   briupLcdImageInit(&image_logo,48,178,50,150,(unsigned char*)gImage_logo);
   briupLcdImageDraw(&image_logo);
   
   briupLcdImageInit(&image_start,58,104,115,190,(unsigned char*)gImage_start);
   briupLcdImageDraw(&image_start);
   
   while(1)
   {
      draw_bird(210,220);
      printf("x = %d,y = %d",get_xy.currX,get_xy.currY);
   }
   
}


