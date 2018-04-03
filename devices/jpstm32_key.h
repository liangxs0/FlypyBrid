/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * File name: jpstm32_key.h
 * Author: Duke An
 * Version: 0.1
 * Date: 2014-06-17
 * Description: 五向按键功能定义
 * Copyright @ 2013 by Briup Technology,Inc. All rights Reserved.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
 #ifndef __JPSTM32_KEY_H__
 #define __JPSTM32_KEY_H__
 
 #include "jpstm32_common.h"
 
 /* * * * * * * * * * * * * * * * * * * * * * * * * * * 
  * * * * * * * * * * * * * * * * * * * * * * * * * * */

 /* * * * * * * * * * * * * * * * * * * * * * * * * * * 
  * 五向按键键值
  * * * * * * * * * * * * * * * * * * * * * * * * * * */
 typedef enum{
	 KEY_NOKEY,		/*没有按键按下*/
	 KEY_UP,		/*上键按下*/
	 KEY_DOWN,		/*下键按下*/
	 KEY_LEFT,		/*左键按下*/
	 KEY_RIGHT,		/*右键按下*/
	 KEY_CENTER		/*中间键按下*/
}KEY_VAL;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	key_init
 * Description: 初始化五向按键
 * Input: NULL
 * Output: NULL
 * Return: NULL
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern void key_init(void);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	key_init
 * Description: 扫描五向按键
 * Input: NULL
 * Output: NULL
 * Return: 如有按键按下返回对应键值 否则返回KEY_NOKEY
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern KEY_VAL key_scan(void);



 #endif

