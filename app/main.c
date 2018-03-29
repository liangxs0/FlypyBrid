/*
*
*
*
*/

#include "stm32f10x.h"
#include <string.h>

#include "briupDelay.h"
#include "briupTIM2.h"
#include "briuoNVIC.h"
#include "briupKeyboard.h"
#include "briupBasePeriph.h"

#include "briupLCD.h"
#include "briupMPU6050.h"

int main(void)
{
	briupTIM2Init(4);
	briupNVICGroupInit(2);
	delay_init(72);
	basePeriphInit();
	briupKeyInit();
	briupUsart1Init(57600);
	
	briupLcdInit();
	
	while(1);
}

