/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * File name: jpstm32_key.h
 * Author: Duke An
 * Version: 0.1
 * Date: 2014-06-17
 * Description: ���򰴼����ܶ���
 * Copyright @ 2013 by Briup Technology,Inc. All rights Reserved.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
 #ifndef __JPSTM32_KEY_H__
 #define __JPSTM32_KEY_H__
 
 #include "jpstm32_common.h"
 
 /* * * * * * * * * * * * * * * * * * * * * * * * * * * 
  * * * * * * * * * * * * * * * * * * * * * * * * * * */

 /* * * * * * * * * * * * * * * * * * * * * * * * * * * 
  * ���򰴼���ֵ
  * * * * * * * * * * * * * * * * * * * * * * * * * * */
 typedef enum{
	 KEY_NOKEY,		/*û�а�������*/
	 KEY_UP,		/*�ϼ�����*/
	 KEY_DOWN,		/*�¼�����*/
	 KEY_LEFT,		/*�������*/
	 KEY_RIGHT,		/*�Ҽ�����*/
	 KEY_CENTER		/*�м������*/
}KEY_VAL;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	key_init
 * Description: ��ʼ�����򰴼�
 * Input: NULL
 * Output: NULL
 * Return: NULL
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern void key_init(void);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	key_init
 * Description: ɨ�����򰴼�
 * Input: NULL
 * Output: NULL
 * Return: ���а������·��ض�Ӧ��ֵ ���򷵻�KEY_NOKEY
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern KEY_VAL key_scan(void);



 #endif

