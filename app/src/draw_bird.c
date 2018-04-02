//»æÖÆÐ¡Äñ

#include "draw_bird.h"



extern const unsigned char gImage_borid_up[1632];
extern const unsigned char gImage_borid_move[1632];
extern const unsigned char gImage_borid_down[1632];

ImgTypeDef imageborid_up;
ImgTypeDef imageborid_move;
ImgTypeDef imageborid_down;
	// int counter;
void draw_bird(u16 x,u16 y)
{

     

		briupLcdImageInit(&imageborid_up,24,34,x,y,(unsigned char*)gImage_borid_up);
		briupLcdImageDraw(&imageborid_up);
      //draw_map(300,counter+= 5);
   
		_delay_ms(50);
	
		briupLcdImageInit(&imageborid_move,24,34,x,y,(unsigned char*)gImage_borid_move);
		briupLcdImageDraw(&imageborid_move);
     // draw_map(300,counter+= 5);
		_delay_ms(50);

		briupLcdImageInit(&imageborid_down,24,34,x,y,(unsigned char*)gImage_borid_down);
		briupLcdImageDraw(&imageborid_down);
     // draw_map(300,counter+= 5);
		_delay_ms(50);
//   if(counter == 300)
//      counter = 0;
}




