//

#include "briupLCD.h"


extern const unsigned char gImage_borid_up[1632];
extern const unsigned char gImage_borid_move[1632];
extern const unsigned char gImage_borid_down[1632];

	ImgTypeDey imageborid_up;
	ImgTypeDey imageborid_move;
	ImgTypeDey imageborid_down;
	
void draw_borid(u8 x,u8 y)
{


	
		briupLcdImageInit(&imageborid_up,34,24,x,y,(unsigned char*)gImage_borid_up);
		briupLcdImageDraw(&imageborid_up);
	
		_delay_ms(50);
	
		briupLcdImageInit(&imageborid_move,34,24,x,y,(unsigned char*)gImage_borid_move);
		briupLcdImageDraw(&imageborid_move);
		
		_delay_ms(50);

		briupLcdImageInit(&imageborid_down,34,24,x,y,(unsigned char*)gImage_borid_down);
		briupLcdImageDraw(&imageborid_down);
		
		_delay_ms(50);
}


