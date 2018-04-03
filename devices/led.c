/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * File name: jpstm32_led.c
 * Author: Duke An
 * Version: 0.1
 * Date: 2014-06-17
 * Description: LED灯操作实现
 * Copyright @ 2013 by Briup Technology,Inc. All rights Reserved.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "led.h"
#include "jpstm32_delay.h"

extern void led_init()
{
	_BD(RCC->APB2ENR, 4) = 1;
	//PC.1 2 3 推挽输出
	config_gpio(PORTC, O_GPPP|OMODE_T50MHz, Pin0|Pin1|Pin2, PULL_UP);
	led_setR(LED_OFF);
	led_setG(LED_OFF);
	led_setB(LED_OFF);

}
extern void led_setR(LED_STAT s)
{
	if(LED_ON == s){
		PCxOut(2) = VAL_0;
	} else {
		PCxOut(2) = VAL_1;
	}
}
extern void led_setG(LED_STAT s)
{
	if(LED_ON == s){
		PCxOut(1) = VAL_0;
	} else {
		PCxOut(1) = VAL_1;
	}
}
extern void led_setB(LED_STAT s)
{
	if(LED_ON == s){
		PCxOut(0) = VAL_0;
	} else {
		PCxOut(0) = VAL_1;
	}
}
extern void led_col(void)
{
	u8	i = 0;
	for(i=0; i<8; i++){
		set_PINxs(PORTC, Pin0|Pin1|Pin2, VAL_1);
		set_PINxs(PORTC, i, VAL_0);
		_delay_ms(1000);
		
	}
}
