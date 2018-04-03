/* ****************************************************
 * 描    述：本程序用于定义TFT-LCD驱动相关的操作
 * 作    者：Duke An
 * 时    间：2014年8月
 * 版    权：杰普软件科技有限公司版权所有，
 *			 转载请注明出处，侵权必究！
 * 更新说明：
 * 编码规范：宏定义--大写，单词之间用下划线分割
 *			 结构--首字母大写，驼峰命名
 *			 枚举--首字母大写，驼峰命名
 *			 函数--首字母小写，驼峰命名
 *			 局部变量--小写，单词之间用下划线分割
 *			 全局变量--首字母大写，单词之间用下划线分割
 * ***************************************************/
#ifndef __TFTLCD_H__
#define __TFTLCD_H__

/* ****************************************************
 * JPIOT-STM32-V1开发板采用3.5英寸ILI9486液晶，支持最大
 * 分辨率为480*320。为提高数据访问速度，特别采用16位并行
 * 接口与CPU进行连接，此液晶屏采用的80并口(intel 提出的
 * 一种并口通信协议)主要信号线及连接方式如下：
 *
 * LCD_CS：			TFT-LCD片选信号
 * LCD_WR：			向TFT-LCD写入数据，低电平有效
 * LCD_RD：			从TFT-LCD读取数据，低电平有效
 * LCD_DB[15:0]：	16位双向数据线
 * LCD_RST：		复位	
 * LCD_RS：			命令/数据标志(0-命令操作;1-数据操作)
 *
 * JPIOT-STM32-V1开发板LCD硬件连接方式：
 * ________________
 *           RESET |---LCD_RESET
 *   PG12/FSMC_NE4 |---LCD_CS
 *    PG0/FSMC_A10 |---LCD_RS
 *    PD4/FSMC_NOE |---LCD_RD
 *    PD5/FSMC_NWE |---LCD_WR
 *             PB0 |---LCD_BL		//背光
 *    PD14/FSMC_D0 |---LCD_DB1
 *    PD15/FSMC_D1 |---LCD_DB2
 *     PD0/FSMC_D2 |---LCD_DB3
 *     PD1/FSMC_D3 |---LCD_DB4
 *	   PE7/FSMC_D4 |---LCD_DB5
 *	   PE8/FSMC_D5 |---LCD_DB6
 *	   PE9/FSMC_D6 |---LCD_DB7
 *	  PE10/FSMC_D7 |---LCD_DB8
 *	  PE11/FSMC_D8 |---LCD_DB10
 *	  PE12/FSMC_D9 |---LCD_DB11
 *   PE13/FSMC_D10 |---LCD_DB12
 *   PE14/FSMC_D11 |---LCD_DB13
 *   PE15/FSMC_D12 |---LCD_DB14
 *    PD8/FSMC_D13 |---LCD_DB15
 *    PD9/FSMC_D14 |---LCD_DB16
 *   PD10/FSMC_D15 |---LCD_DB17
 * ________________|
 * ***************************************************/

#include "jpstm32_gpio.h"
#include "stdlib.h"

//横、竖屏切换
#define	LCD_HOR_MODE	1
#define	LCD_VER_MODE	0
#define	LCD_VIEW_MODE	LCD_VER_MODE
//背光控制
#define	BLIGHT			PBxOut(0)
#define	BLIGHT_ON		1
#define	BLIGHT_OFF	0
//扫描方向
#define L2R_U2D  0 //left to right, up to down
#define L2R_D2U  1 
#define R2L_U2D  2 
#define R2L_D2U  3 

#define U2D_L2R  4
#define U2D_R2L  5 
#define D2U_L2R  6
#define D2U_R2L  7
//默认扫描方向
#define DFT_SCAN_DIR  L2R_U2D
/* ***********************************************************
 * 1.LCD9486的并口操作类似于SRAM的并口，STM32F103ZET6专门提供
 *   了针对SRAM/NOR FLASH、NAND Flash/PC Card操作的总线控制器
 *   FSMC(Flexible static memory controller)
 * 2.FSMC可以针对不同类型的外围设备设置不同的数据传输频率、通
 *   信方式等。当FSMC连接SRAM时其工作原理是：
 *		a)指定地址，在地址总线上指定某一个地址
 *		b)准备地址，需要一段时间进行计算该地址
 *		c)指定访问字节序，如果外设支持高低字节访问顺序设置则
 *		  需要配置此选项
 *		d)片选，FSMC可能同时连接多个外设 需要指定具体的设备
 *		e)读/写设置，确定对a)中指定的地址进行读操作还是写操作
 *		f)数据操作
 *		  读 将a)步骤中指定的地址内的数据交给数据总线
 *		  写 将数据总线上的数据写入a)步骤中指定的地址 
 *   上述步骤全部由FSMC自动完成，程序员在编程时只需关心要操作
 *   的地址与数据即可
 * 3.虽然LCD9486的并口与SRAM类似但是LCD9486没有地址线，编程时
 *   交互的数据需要通过LCD_RS信号线来区分写入LCD9486的是命令
 *   还是数据，操作时可以将LCD_RS信号线理解为地址。例如把LCD_RS
 *   连接到FSMC的A0引脚上，那么当FSMC对0x0000000地址进行写操作
 *   时使得A0引脚变为0，对LCD9486来说就是写入命令；当FSMC对0x01
 *   地址进行写操作时使得A0引脚变为1，对于LCD9486来说就是写入数
 *   据。JPIOT_STM32-V1开发板将LCD9486的RS引脚连接到了FSMC的A10
 *   引脚，即当我们访问0Bxxxx x0xx xxxx xxxx地址时表示向LCD写入
 *   命令，当访问0Bxxxx x1xx xxxx xxxx时表示向LCD写入数据。由于
 *   采用16位并口进行传输，内部总线地址与FSMC的地址对应关系为
 *   HADDR[25:1]-->FSMC[24:0]所以当RS连接到A10时访问的实际地址
 *   应该右移1位。为方便操作取相邻的两个地址(两字节为单位)来区
 *   分写入LCD的是命令还是数据，则上述两个地址变为0x7FE(写入命
 *   令)和0x800(写入数据)，将这两个地址再加上实际使用的存储区
 *   的首地址就是最终要操作的地址。所以有如下定义：
 * ***********************************************************/
typedef struct {
	u16		lcd_cmd;	//向LCD写入命令
	u16		lcd_data;	//向LCD写入数据
}LcdAddr;
//LCD使用FSMC的Bank1存储区的第四区，所以其实地址为0x6C000000
#define		LCD_BASE	((u32)(0x6C000000|0x000007FE))
#define		LCD_ADDR	((LcdAddr*)LCD_BASE)

//LCD设备描述结构体
typedef struct {
	u16		dev_no;
	u16		width;	//宽度
	u16		height;	//高度
	u8		d_mode; //显示模式 HOR_MODE 横屏，VER_MODE 竖屏
	u8		wr_cmd;	//写命令
	u8		x_cmd;	//x坐标指令
	u8		y_cmd;
}LcdDev;

extern u16 POINT_COLOR;	//画笔颜色
extern u16 BACK_COLOR;  //背景色 

//设备信息
extern LcdDev	Lcd_Dev;

//颜色定义
#define WHITE       0xFFFF
#define BLACK      	0x0000	  
#define BLUE       	0xF800 
#define BRED        0XF81F
#define GRED 				0XFFE0
#define RED         0x001F
#define MAGENTA     0xF81F
#define GREEN       0x07E0
#define CYAN        0x7FFF
#define YELLOW      0x07FF
#define BROWN 			0XBC40 
#define BRRED 			0XFC07 
#define GRAY  			0X8430 

/**************************
 *	画笔定义
 **************************/
//字符大小
typedef enum
{
	P_S16EN = 16,
	P_S32EN = 32,
	P_S16CN = 16,
	P_S32CN = 32
}PenSize;
typedef enum
{
	P_DIR_HOR,
	P_DIR_VER
}PenDirect;
typedef enum
{
	P_SET_NOBRUSH, P_SET_BRUSH
}PenBrush;
//画笔
typedef struct{
	u16 color;
	u16 bg_color;
	PenSize p_size;
	PenBrush brush;
	PenDirect direct;
}LcdPen;


//初始化
void lcdInit(void);
//清屏，全屏填充
void lcdClear(u16 color);
//显示 开
void lcdDisplayOn(void);
//显示 关
void lcdDisplayOff(void);
//设置光标位置
void lcdSetCursor(u16 x, u16 y);
//设定一个矩形显示区
void lcdSetWindow(u16 x_start, u16 y_start, u16 x_end, u16 y_end);

//在屏幕上绘制一个像素点
void lcdDrawPoint(u16 x, u16 y, LcdPen* pen);

void lcdDrawLine(u16 x1,u16 y1,u16 x2,u16 y2,LcdPen* pen);

void lcdDrawRect(u16 x,u16 y,u16 width,u16 height,LcdPen* pen);

void lcd_write_cmd(void);

u16 LCD_DecToRGB(u8 R, u8 G, u8 B);

//绘制一个字符,成功返回0,否则返回非零
u8 lcdDrawChr(u16 x, u16 y, u8 ch, LcdPen* p);
//在指定位置绘制一个字符串
u8 lcdDrawStr(u16 x, u16 y, u8* str, LcdPen* p);
//绘制一个由四个像素组成的点
void lcdDrawBigPoint(u16 x, u16 y, LcdPen* p);

#endif











