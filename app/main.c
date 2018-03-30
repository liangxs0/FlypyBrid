/*
*
*
*
*/

#include "main.h"
extern const unsigned char gImage_test[17088];
extern const unsigned char gImage_borid_up[1632];
extern const unsigned char gImage_borid_move[1632];
extern const unsigned char gImage_borid_down[1632];

int main(void)
{
	ImgTypeDey imageTest;
	ImgTypeDey imageborid_up;
	ImgTypeDey imageborid_move;
	ImgTypeDey imageborid_down;
	
	int i;
	
	briupTIM2Init(4);
	briupNVICGroupInit(2);
	delay_init(72);
	basePeriphInit();
	briupKeyInit();
	briupUsart1Init(57600);
	
	briupLcdInit();

	briupLcdClear(WHITE);


	while(1)
	{
		
	
		briupLcdImageInit(&imageborid_up,34,24,30,30,(unsigned char*)gImage_borid_up);
		briupLcdImageDraw(&imageborid_up);
	
		_delay_ms(50);
	
		briupLcdImageInit(&imageborid_move,34,24,30,30,(unsigned char*)gImage_borid_move);
		briupLcdImageDraw(&imageborid_move);
		
		_delay_ms(50);

		briupLcdImageInit(&imageborid_down,34,24,30,30,(unsigned char*)gImage_borid_down);
		briupLcdImageDraw(&imageborid_down);
		
		_delay_ms(50);
		
	}
}

