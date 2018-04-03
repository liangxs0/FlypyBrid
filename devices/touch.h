/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * File name: jpstm32_touch.h
 * Author: Duke An
 * Version: 0.1
 * Date: 2014-06-17
 * Description: ����������ͷ�ļ�
 * Copyright @ 2013 by Briup Technology,Inc. All rights Reserved.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef __TOUCH_H__
#define __TOUCH_H__

#include "jpstm32_common.h"
#include "jpstm32_lcd.h"

#define TP_DEBUG_ENABLE	1

#define TP_PRES_DOWN 0x80  //����������	  
#define TP_CATH_PRES 0x40  //�а��������� 

//���������Ŷ���
#define TP_PEN		PFxIn(10) //���¼��
#define TP_OUT		PFxIn(8)	//��������
#define	TP_IN			PFxOut(9)	//�������
#define	TP_CLK		PBxOut(1)	//ʱ�����
#define	TP_CS			PBxOut(2)	//Ƭѡ

typedef struct{
	u8 	(*init)(void);
	u8	(*scan)(u8);
	u8 	(*adjust)(void);
	//��һ������
	u16		lastX;
	u16		lastY;
	//�˴�����
	u16		currX;
	u16		currY;
	//��Ļ�Ƿ񱻰��£����� 1 �ɿ� 0
	u8		penStat;
	float			xfac;
	float 		yfac;
	short			xoff;
	short			yoff;
	PenDirect direct;
	u8				cmdRdx;
	u8				cmdRdy;
}TouchTpyDef;

extern TouchTpyDef	t_pad;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	tp_spiWbyte
 * Description: ������д��һ���ֽ�
 * Input: NULL
 * Output: NULL
 * Return: NULL
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void tp_spiWbyte(u8 byte);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	tp_readAD
 * Description: ��ȡADת����ֵ
 * Input: ��ȡADֵ������
 * Output: NULL
 * Return: ���ض�����ADֵ
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
u16 tp_readAD(u8 cmd);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	tp_readXorY
 * Description: ���˲��Ķ�ȡx��y��ֵ
 * Input: ��ȡx��yֵ������
 * Output: NULL
 * Return: ���ض�����ֵ
 * Others: ������ȡN�ζ���Щ���ݽ�������ȥ��������Сֵ
 *				 ʣ�µ�������ƽ��ֵ
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
u16 tp_readXorY(u8 cmd);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	tp_readXandY
 * Description: ��ȡx��y��ֵ
 * Input: ��ȡ����x yֵ�ı���
 * Output: NULL
 * Return: ����0�ɹ� ����ʧ��
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
u8 tp_readXandY(u16* x, u16* y);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	tp_readXandY2
 * Description: �����˲� ��ȡx��y��ֵ
 * Input: ��ȡ����x yֵ�ı���
 * Output: NULL
 * Return: ����0�ɹ� ����ʧ��
 * Others: ����2�ζ�ȡ������IC,�������ε�ƫ��ܳ���
 *  			 ERR_RANGE,��������,����Ϊ������ȷ,�����������
 *		     �ú����ܴ�����׼ȷ��
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
u8 tp_readXandY2(u16* x, u16* y);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	tp_drawAdjustPoint
 * Description: ������ĻУ׼��
 * Input: �������꣬ ��ɫ
 * Output: NULL
 * Return: NULL
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void tp_drawAdjustPoint(u16 x, u16 y, u16 color);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	tp_scan(u8 tp)
 * Description: ������ĻУ׼��
 * Input:  tp: 1 ��Ļ���꣬0 ��������
 * Output: NULL
 * Return: ��ǰ��Ļ״̬ 0 û�а��� 1 �а���
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
u8 tp_scan(u8 tp);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	tp_init
 * Description: ��ʼ��������
 * Input:  NULL
 * Output: NULL
 * Return: �ɹ�����0 ���򷵻ط�0
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
u8 tp_init(void);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	tp_adjust
 * Description: ��ĻУ׼
 * Input:  NULL
 * Output: NULL
 * Return: �ɹ�����0 ���򷵻ط�0
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
u8 tp_adjust(void);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	tp_showAdjustInfo
 * Description: ��ӡУ׼��Ϣ
 * Input:  NULL
 * Output: NULL
 * Return: �ɹ�����0 ���򷵻ط�0
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
u8 tp_printAdjustInfo(u16 pos[][2], u16 fac);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Function:	tp_screenTrack
 * Description: ��Ļ��������
 * Input:  NULL
 * Output: NULL
 * Return: 
 * Others: NULL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void tp_screenTrack(void);

#endif










