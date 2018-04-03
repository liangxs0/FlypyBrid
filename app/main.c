#include "jpstm32_lcd.h"
#include "jpstm32_delay.h"
#include "jpstm32_sysclk.h"
#include "jpstm32_usart.h"
#include "jpstm32_key.h"
#include "jpstm32_beep.h"
#include "touch.h"
#include "led.h"
#include "game.h"

int main(void)
{
	u8 Key=0;
	
	/*u16 color,xx,yy;
	*/
	led_init();
	clk_init(9);
	_delay_init(72);
	usart1_init(72, 115200);

	nvic_set_group(2);
	lcdInit();
 
	FSMC_SRAM_init();
	DMA_Init();
	
	LCD_PWM_Init();
	Adc_Init();
	LCD_BLPWM_VAL = 250;
	t_pad.init();
	AT24CXX_Init();
	key_init();
	beep_init();
	LCD_BLPWM_VAL = 250;
	Key=key_scan();
	if(Key == KEY_RIGHT)
		t_pad.adjust();
	FlappyBird_Stage_Start();		
	while(1) 
	{
		FlappyBird_Stage_PP();
		FlappyBird_Stage_Play();
		BEEP_ON();
		led_setR(LED_ON);
		_delay_ms(500);
		BEEP_OFF();
		led_setR(LED_OFF);
		FlappyBird_Stage_Over();
	}
}
