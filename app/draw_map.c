//ªÊ÷∆±≥æ∞µÿÕº1£¨2

#include "draw_map.h"
#include "briupLCD.h"

extern const unsigned char gImage_board2[1872];
extern const unsigned char gImage_board1[6384];

ImgTypeDey image_board2;
ImgTypeDey image_board1;

void draw_map(u8 x,u8 y)
{
	briupLcdImageInit(&image_board1,320,80,x,y,(unsigned char*)gImage_board1);
   briupLcdImageDraw(&image_board1);
   
   briupLcdImageInit(&image_board2,320,100,x,y,(unsigned char*)gImage_board2);
   briupLcdImageDraw(&image_board2);
}



