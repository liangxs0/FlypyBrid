/*
*
*
*
*/

#include "main.h"


int main(void)
{


	
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
		draw_borid(100,50);
	}
}

