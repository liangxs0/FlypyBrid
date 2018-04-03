/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * File name: jpstm32_led.h
 * Author: Duke An
 * Version: 0.1
 * Date: 2014-06-17
 * Description: LEDµÆ²Ù×÷¶¨Òå
 * Copyright @ 2013 by Briup Technology,Inc. All rights Reserved.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifndef __LED_H__
#define __LED_H__

#include "jpstm32_gpio.h"
#include "jpstm32_common.h"

#define LED_G PCxOut(1)
#define LED_R PCxOut(2)

#define	F_LEDR	1<<0
#define F_LEDG	1<<1
#define F_LEDB	1<<2

typedef enum{
	LED_ON, LED_OFF
} LED_STAT;

extern void led_init(void);

extern void led_setR(LED_STAT);
extern void led_setG(LED_STAT);
extern void led_setB(LED_STAT);
extern void led_col(void);


#endif
