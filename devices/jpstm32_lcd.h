/* ****************************************************
 * ��    �������������ڶ���TFT-LCD������صĲ���
 * ��    �ߣ�Duke An
 * ʱ    �䣺2014��8��
 * ��    Ȩ����������Ƽ����޹�˾��Ȩ���У�
 *			 ת����ע����������Ȩ�ؾ���
 * ����˵����
 * ����淶���궨��--��д������֮�����»��߷ָ�
 *			 �ṹ--����ĸ��д���շ�����
 *			 ö��--����ĸ��д���շ�����
 *			 ����--����ĸСд���շ�����
 *			 �ֲ�����--Сд������֮�����»��߷ָ�
 *			 ȫ�ֱ���--����ĸ��д������֮�����»��߷ָ�
 * ***************************************************/
#ifndef __TFTLCD_H__
#define __TFTLCD_H__

/* ****************************************************
 * JPIOT-STM32-V1���������3.5Ӣ��ILI9486Һ����֧�����
 * �ֱ���Ϊ480*320��Ϊ������ݷ����ٶȣ��ر����16λ����
 * �ӿ���CPU�������ӣ���Һ�������õ�80����(intel �����
 * һ�ֲ���ͨ��Э��)��Ҫ�ź��߼����ӷ�ʽ���£�
 *
 * LCD_CS��			TFT-LCDƬѡ�ź�
 * LCD_WR��			��TFT-LCDд�����ݣ��͵�ƽ��Ч
 * LCD_RD��			��TFT-LCD��ȡ���ݣ��͵�ƽ��Ч
 * LCD_DB[15:0]��	16λ˫��������
 * LCD_RST��		��λ	
 * LCD_RS��			����/���ݱ�־(0-�������;1-���ݲ���)
 *
 * JPIOT-STM32-V1������LCDӲ�����ӷ�ʽ��
 * ________________
 *           RESET |---LCD_RESET
 *   PG12/FSMC_NE4 |---LCD_CS
 *    PG0/FSMC_A10 |---LCD_RS
 *    PD4/FSMC_NOE |---LCD_RD
 *    PD5/FSMC_NWE |---LCD_WR
 *             PB0 |---LCD_BL		//����
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

//�ᡢ�����л�
#define	LCD_HOR_MODE	1
#define	LCD_VER_MODE	0
#define	LCD_VIEW_MODE	LCD_VER_MODE
//�������
#define	BLIGHT			PBxOut(0)
#define	BLIGHT_ON		1
#define	BLIGHT_OFF	0
//ɨ�跽��
#define L2R_U2D  0 //left to right, up to down
#define L2R_D2U  1 
#define R2L_U2D  2 
#define R2L_D2U  3 

#define U2D_L2R  4
#define U2D_R2L  5 
#define D2U_L2R  6
#define D2U_R2L  7
//Ĭ��ɨ�跽��
#define DFT_SCAN_DIR  L2R_U2D
/* ***********************************************************
 * 1.LCD9486�Ĳ��ڲ���������SRAM�Ĳ��ڣ�STM32F103ZET6ר���ṩ
 *   �����SRAM/NOR FLASH��NAND Flash/PC Card���������߿�����
 *   FSMC(Flexible static memory controller)
 * 2.FSMC������Բ�ͬ���͵���Χ�豸���ò�ͬ�����ݴ���Ƶ�ʡ�ͨ
 *   �ŷ�ʽ�ȡ���FSMC����SRAMʱ�乤��ԭ���ǣ�
 *		a)ָ����ַ���ڵ�ַ������ָ��ĳһ����ַ
 *		b)׼����ַ����Ҫһ��ʱ����м���õ�ַ
 *		c)ָ�������ֽ����������֧�ָߵ��ֽڷ���˳��������
 *		  ��Ҫ���ô�ѡ��
 *		d)Ƭѡ��FSMC����ͬʱ���Ӷ������ ��Ҫָ��������豸
 *		e)��/д���ã�ȷ����a)��ָ���ĵ�ַ���ж���������д����
 *		f)���ݲ���
 *		  �� ��a)������ָ���ĵ�ַ�ڵ����ݽ�����������
 *		  д �����������ϵ�����д��a)������ָ���ĵ�ַ 
 *   ��������ȫ����FSMC�Զ���ɣ�����Ա�ڱ��ʱֻ�����Ҫ����
 *   �ĵ�ַ�����ݼ���
 * 3.��ȻLCD9486�Ĳ�����SRAM���Ƶ���LCD9486û�е�ַ�ߣ����ʱ
 *   ������������Ҫͨ��LCD_RS�ź���������д��LCD9486��������
 *   �������ݣ�����ʱ���Խ�LCD_RS�ź������Ϊ��ַ�������LCD_RS
 *   ���ӵ�FSMC��A0�����ϣ���ô��FSMC��0x0000000��ַ����д����
 *   ʱʹ��A0���ű�Ϊ0����LCD9486��˵����д�������FSMC��0x01
 *   ��ַ����д����ʱʹ��A0���ű�Ϊ1������LCD9486��˵����д����
 *   �ݡ�JPIOT_STM32-V1�����彫LCD9486��RS�������ӵ���FSMC��A10
 *   ���ţ��������Ƿ���0Bxxxx x0xx xxxx xxxx��ַʱ��ʾ��LCDд��
 *   ���������0Bxxxx x1xx xxxx xxxxʱ��ʾ��LCDд�����ݡ�����
 *   ����16λ���ڽ��д��䣬�ڲ����ߵ�ַ��FSMC�ĵ�ַ��Ӧ��ϵΪ
 *   HADDR[25:1]-->FSMC[24:0]���Ե�RS���ӵ�A10ʱ���ʵ�ʵ�ʵ�ַ
 *   Ӧ������1λ��Ϊ�������ȡ���ڵ�������ַ(���ֽ�Ϊ��λ)����
 *   ��д��LCD������������ݣ�������������ַ��Ϊ0x7FE(д����
 *   ��)��0x800(д������)������������ַ�ټ���ʵ��ʹ�õĴ洢��
 *   ���׵�ַ��������Ҫ�����ĵ�ַ�����������¶��壺
 * ***********************************************************/
typedef struct {
	u16		lcd_cmd;	//��LCDд������
	u16		lcd_data;	//��LCDд������
}LcdAddr;
//LCDʹ��FSMC��Bank1�洢���ĵ�������������ʵ��ַΪ0x6C000000
#define		LCD_BASE	((u32)(0x6C000000|0x000007FE))
#define		LCD_ADDR	((LcdAddr*)LCD_BASE)

//LCD�豸�����ṹ��
typedef struct {
	u16		dev_no;
	u16		width;	//���
	u16		height;	//�߶�
	u8		d_mode; //��ʾģʽ HOR_MODE ������VER_MODE ����
	u8		wr_cmd;	//д����
	u8		x_cmd;	//x����ָ��
	u8		y_cmd;
}LcdDev;

extern u16 POINT_COLOR;	//������ɫ
extern u16 BACK_COLOR;  //����ɫ 

//�豸��Ϣ
extern LcdDev	Lcd_Dev;

//��ɫ����
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
 *	���ʶ���
 **************************/
//�ַ���С
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
//����
typedef struct{
	u16 color;
	u16 bg_color;
	PenSize p_size;
	PenBrush brush;
	PenDirect direct;
}LcdPen;


//��ʼ��
void lcdInit(void);
//������ȫ�����
void lcdClear(u16 color);
//��ʾ ��
void lcdDisplayOn(void);
//��ʾ ��
void lcdDisplayOff(void);
//���ù��λ��
void lcdSetCursor(u16 x, u16 y);
//�趨һ��������ʾ��
void lcdSetWindow(u16 x_start, u16 y_start, u16 x_end, u16 y_end);

//����Ļ�ϻ���һ�����ص�
void lcdDrawPoint(u16 x, u16 y, LcdPen* pen);

void lcdDrawLine(u16 x1,u16 y1,u16 x2,u16 y2,LcdPen* pen);

void lcdDrawRect(u16 x,u16 y,u16 width,u16 height,LcdPen* pen);

void lcd_write_cmd(void);

u16 LCD_DecToRGB(u8 R, u8 G, u8 B);

//����һ���ַ�,�ɹ�����0,���򷵻ط���
u8 lcdDrawChr(u16 x, u16 y, u8 ch, LcdPen* p);
//��ָ��λ�û���һ���ַ���
u8 lcdDrawStr(u16 x, u16 y, u8* str, LcdPen* p);
//����һ�����ĸ�������ɵĵ�
void lcdDrawBigPoint(u16 x, u16 y, LcdPen* p);

#endif











