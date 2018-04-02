// common.h
//	��;��ͳһ����ͷ�ļ������ȫ�ֶ���
//
#ifndef __COMMON_H__
#define __COMMON_H__

#include <stm32f10x.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "briupBasePeriph.h"
#include "briupDelay.h"
#include "briupUsart.h"

#include "briupRTC.h"
#include "briupIWDG.h"
#include "briupKeyboard.h"
#include "briupLCD.h"
#include "briupFSMC.h"
#include "briupTouch.h"

#include "briupSDIO.h"


#include "briupSPI.h"
#include "briupVS1003.h"
#include "briupEXSRAM.h"




#define	MUSIC_UP		(1<<0)
#define	MUSIC_DOWN		(1<<1)
#define	MUSIC_VOL_SUB	(1<<2)
#define	MUSIC_VOL_ADD	(1<<3)
#define	MUSIC_SEL		(1<<4)
#define	MUSIC_MODE_SINGEL	(1<<5)
#define	MUSIC_MODE_LOOP	(1<<6)
#define	MUSIC_MODE_RAND	(1<<7)
#define	MUSIC_STAT_JMP	(1<<8)


typedef	struct{
	u8	ctrlDev;
	u16	ctrlOpt;
	u16 x, y;
}Mp3Control;


 /* * * * * * * * * * * * * * * * * * * * * * * * * * * 
  * λ����������SRAM����ַת����λ����������ַ
	* >reg Ҫת����λ���ڵļĴ���
	* >bitn Ҫת����λ
	* 0x42000000+(A-0x40000000)*32+n*4
	* 0x22000000+(A-0x20000000)*32+n*4
  * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define _BD(reg, bitn)\
	*((vu32*)(((vu32)(&reg)&0xF0000000)+0x2000000\
	+(((vu32)(&reg)&0xFFFFF)<<5) + (bitn<<2)))

//GPIONx output
#define PAxOut(x)	_BD(GPIOA->ODR, x)	//PORTAx output
#define PBxOut(x)	_BD(GPIOB->ODR, x)	//PORTBx output
#define PCxOut(x)	_BD(GPIOC->ODR, x)	//PORTCx output
#define PDxOut(x)	_BD(GPIOD->ODR, x)	//PORTDx output
#define PExOut(x)	_BD(GPIOE->ODR, x)	//PORTEx output
#define PFxOut(x)	_BD(GPIOF->ODR, x)	//PORTFx output
#define PGxOut(x)	_BD(GPIOG->ODR, x)	//PORTGx output
//GPIONx input
#define PAxIn(x)	_BD(GPIOA->IDR, x)	//PORTAx input
#define PBxIn(x)	_BD(GPIOB->IDR, x)	//PORTBx input
#define PCxIn(x)	_BD(GPIOC->IDR, x)	//PORTCx input
#define PDxIn(x)	_BD(GPIOD->IDR, x)	//PORTDx input
#define PExIn(x)	_BD(GPIOE->IDR, x)	//PORTEx input
#define PFxIn(x)	_BD(GPIOF->IDR, x)	//PORTFx input
#define PGxIn(x)	_BD(GPIOG->IDR, x)	//PORTGx input

#endif

