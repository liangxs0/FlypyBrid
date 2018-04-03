#include "jpstm32_lcd.h"
#include "jpstm32_lcd_font.h"
#include "jpstm32_delay.h"
#include "jpstm32_usart.h"
#include "stdio.h"

/* **************************************************
 * 全局静态变量
 * *************************************************/

u16 POINT_COLOR=0x0000;	//画笔颜色
u16 BACK_COLOR=0xFFFF;  //背景色 

//设备信息
LcdDev	Lcd_Dev;

/* **************************************************
 * 静态函数声明
 * *************************************************/
//向LCD写入16位指令
static void lcdWriteCmd(u16 cmd);
//向LCD写入16位数据
static void lcdWriteData(u16 data);
//读取LCD数据
static u16 lcdReadData(void);
//写指定寄存器
static void lcdWriteReg(u8 lcd_reg, u16 reg_val);
//读取指定寄存器
static u16 lcdReadReg(u8 lcd_reg);
//开始写GRAM
static void lcdWriteMem(void);
//设置设备参数
static void lcdSetParam(void);
/* **************************************************
 * 提供给用户的函数
 * **************************************************/
 
 
//lcd初始化
void lcdInit(void)
{
	u16 parm;
	
	/* ********************************
	 * I/O端口配置
	 * *******************************/
	//I/O端口初始化
	RCC->AHBENR |= 1<<8;
	//端口B,D,E,G时钟使能
	RCC->APB2ENR |= (1<<3)|(1<<5)|(1<<6)|(1<<8);
	//PB0 推挽输出，LCD背光控制
	GPIOB->CRL &= 0xFFFFFFF0;
	GPIOB->CRL |= 0x00000003;
	//PORTD 复用推挽输出
	GPIOD->CRH &= 0x00FFF000;
	GPIOD->CRH |= 0xBB000BBB;
	GPIOD->CRL &= 0XFF00FF00;
	GPIOD->CRL |= 0X00BB00BB;   	 
	//PORTE 复用推挽输出
	GPIOE->CRH &= 0X00000000;
	GPIOE->CRH |= 0XBBBBBBBB; 
	GPIOE->CRL &= 0X0FFFFFFF;
	GPIOE->CRL |= 0XB0000000;
	//PORTG 复用推挽输出
	GPIOG->CRH &= 0XFFF0FFFF;
	GPIOG->CRH |= 0X000B0000; 
	GPIOG->CRL &= 0XFFFFFFF0;	//PG0->RS
	GPIOG->CRL |= 0X0000000B;
	/* ********************************
	 * FSMC配置
	 * *******************************/
	//寄存器清零
	//Bank1共有NE1~4，每个端口都有一个BCR+TCR，所以共有8个寄存器
	//JPIOT-STM32-V1使用的是NE4，即对应BTCR[6]、BTCR[7]
	FSMC_Bank1->BTCR[6] = 0x00000000;	//BCR
	FSMC_Bank1->BTCR[7] = 0x00000000;	//BTR
	FSMC_Bank1E->BWTR[6] = 0x00000000;
	//BCR 配置	异步模式
	FSMC_Bank1->BTCR[6] |= 1<<12;	//存储器 写 使能
	FSMC_Bank1->BTCR[6] |= 1<<14;	//读 写 使用不同时序
	FSMC_Bank1->BTCR[6] |= 1<<4;	//存储数据宽度为16位
	//BTR 配置，读 时序配置
	FSMC_Bank1->BTCR[7] |= 0<<28;	//采用模式A
	FSMC_Bank1->BTCR[7] |= 1<<0;	//地址建立时间为2*HCLK
	FSMC_Bank1->BTCR[7] |= 0xF<<8;	//数据保存时间为16*HCLK
	//写 控制
	FSMC_Bank1E->BWTR[6] |= 0<<28;	//MODE A
	FSMC_Bank1E->BWTR[6] |= 0<<0;	//地址建立时间为1*HCLK
	//液晶驱动IC写信号脉冲至少需要50ns，72MHz/4=24MHz=55ns, 所以
	FSMC_Bank1E->BWTR[6] |= 3<<8;	//数据保存时间为4*HCLK
	//使能Bank1的第四区
	FSMC_Bank1->BTCR[6] |= 1<<0;	//使能Bank1的第四区
	_delay_ms(50);
	//启动时钟
	lcdWriteReg(0x0000, 0x0001);
	_delay_ms(50);
#if 1
	//获取ID
	Lcd_Dev.dev_no = lcdReadReg(0x0000);
	if(Lcd_Dev.dev_no<0xFF||Lcd_Dev.dev_no==0xFFFF){
		lcdWriteCmd(0xD3);
		lcdReadData();
		lcdReadData();
		Lcd_Dev.dev_no = lcdReadData();
		Lcd_Dev.dev_no <<= 8;
		Lcd_Dev.dev_no |= lcdReadData();
	}
	printf("LCD ID: %x\r\n", Lcd_Dev.dev_no);
	
	parm = lcdReadReg(0x000c);
	printf("data1: %x\r\n", parm);
	parm = lcdReadData();
	printf("data2: %x\r\n", parm);
	
#endif
	/* ********************************
	 * LCD9486参数配置
	 * *******************************/
#if 1
	/* *************************************************
	 * Normal Display Mode ON
	 * This command returns the display to normal mode. 
	 * Normal display mode on means Partial mode off and 
	 * Scroll mode off.
	 * *************************************************/
	lcdWriteCmd(0x0013);
	/* ************************************************* 
	 * Frame Rate Control (In Normal Mode/Full Colors)
	 * args:
	 *	1.Sets the frame frequency of full normal mode.
	 *	2.Set the division ratio for internal clocks when
	 *	Normal Mode
	 * *************************************************/
	lcdWriteCmd(0x00B1);
	lcdWriteData(0x00A0);
	lcdWriteData(0x0000);
	//Display Inversion Control
	lcdWriteCmd(0x00B4);  
	lcdWriteData(0x0000);
	//设置显示区列边界
	lcdWriteCmd(0x002A);			//set_column_address
	lcdWriteData(0x0000);
	lcdWriteData(0x0000);
	lcdWriteData(0x0001);
	lcdWriteData(0x00DF);//480
	//设置显示区行边界
	lcdWriteCmd(0x002B);			//set_page_address
	lcdWriteData(0x0000);
	lcdWriteData(0x0000);
	lcdWriteData(0x0001);
	lcdWriteData(0x003F);//320
	//Tearing Effect Line ON
	lcdWriteCmd(0x0035);
	lcdWriteData(0x0000);
	//Memory Access Control
	lcdWriteCmd(0x0036);
	lcdWriteData(0x0030);
	//PGAMCTRL(Positive Gamma Control)
	//设置TFT屏幕的灰度正向电压调整伽马值
	lcdWriteCmd(0xE0);
	lcdWriteData(0x09);
	lcdWriteData(0x0a);
	lcdWriteData(0x0e);
	lcdWriteData(0x0d);
	lcdWriteData(0x07);
	lcdWriteData(0x18);
	lcdWriteData(0x0d);
	lcdWriteData(0x0d);
	lcdWriteData(0x0e);
	lcdWriteData(0x04);
	lcdWriteData(0x05);
	lcdWriteData(0x06);
	lcdWriteData(0x0e);
	lcdWriteData(0x25);
	lcdWriteData(0x22);
	//NGAMCTRL (Negative Gamma Correction)
	//设置TFT屏幕的灰度负向电压调整伽马值
	lcdWriteCmd(0XE1);
	lcdWriteData(0x1F);
	lcdWriteData(0x3F);
	lcdWriteData(0x3F);
	lcdWriteData(0x0F);
	lcdWriteData(0x1F);
	lcdWriteData(0x0F);
	lcdWriteData(0x46);
	lcdWriteData(0x49);
	lcdWriteData(0x31);
	lcdWriteData(0x05);
	lcdWriteData(0x09);
	lcdWriteData(0x03);
	lcdWriteData(0x1C);
	lcdWriteData(0x1A);
	lcdWriteData(0x00);
	//Interface Pixel Format像素格式
	lcdWriteCmd(0x003A);   //set_pixel_format
	lcdWriteData(0x0055);
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	 * Write Tear Scan Line 
	 * This command turns on the display Tearing Effect output signal 
	 * on the TE signal line when the display reaches line N. The 
	 * TE signal is not affected by changing Memory Access Control 
	 * bit B4. The Tearing Effect Line On has one parameter that 
	 * describes the Tearing Effect Output Line mode. The Tearing 
	 * Effect Output line consists of V-Blanking information only.
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	lcdWriteCmd(0x0044);
	lcdWriteData(0x0000);
	lcdWriteData(0x0001);
	//turn off the sleep mode
	lcdWriteCmd(0x0011);
	_delay_ms(150);
	//NV Memory Write
	lcdWriteCmd(0x00D0);	
	lcdWriteData(0x0007);
	lcdWriteData(0x0007);
	//NV Memory Protection Key
	lcdWriteCmd(0x00D1);
	lcdWriteData(0x0003);
	lcdWriteData(0x0052);
	lcdWriteData(0x0010);
	//NV Memory Status Read
	lcdWriteCmd(0x00D2);
	lcdWriteData(0x0003);
	lcdWriteData(0x0024);
	lcdWriteCmd(0x0020);
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	 * Interface Mode Control 
	 * Sets the operation status of the display interface.
	 * The setting becomes effective as soon as the command is received.
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	lcdWriteCmd(0x00B0);//{setc, [107], W, 0x000B0}
	lcdWriteData(0x0000);//{setp, [104], W, 0x00000}
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	 * Display On 
	 * This command causes ILI9486L to start displaying the image 
	 * data on the display device. The frame memory contents remain
	 * unchanged. No status bits are changed.
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	lcdWriteCmd(0x0029);		//set_display_on
	_delay_ms(10);
#endif
	lcdSetParam();			//设置LCD参数	 
	BLIGHT = BLIGHT_ON;		//点亮背光	 
	lcdClear((u16)WHITE);	//清屏
}

void lcd_write_cmd(void)
{
	lcdWriteMem();
}

//清屏，全屏填充
void lcdClear(u16 color)
{
	u32 index = 0;
	u32	total = 0;
	u8	range = 5;
	total = Lcd_Dev.width*Lcd_Dev.height;
#if 0
	//lcdDisplayOff();
	lcdSetWindow(0, 0, Lcd_Dev.width-1, Lcd_Dev.height);
	for(index=0; index<total; index++){
		LCD_ADDR->lcd_data = color;
	}
	//lcdDisplayOn();
#endif 

#if 1	//隔行填充以提高画面切换的流畅感，貌似不行
	//lcdDisplayOff();
	

#if LCD_VIEW_MODE == LCD_HOR_MODE
	for(index=0; index<320/range; index++){
		if(index%2 == 0){
			lcdSetWindow(0, index*range, 480-1, index*range+range);
			for(total=0; total<(Lcd_Dev.width)*range; total++){
				LCD_ADDR->lcd_data = color;
			}
		}
	}
	for(index=0; index<320/range; index++){
		if(index%2 == 1){
			lcdSetWindow(0, index*range, 480-1, index*range+range);
			for(total=0; total<(Lcd_Dev.width)*range; total++){
				LCD_ADDR->lcd_data = color;
			}
		}
	}

#else
		for(index=0; index<480/range; index++){
		if(index%2 == 0){
			lcdSetWindow(0, index*range, 320-1, index*range+range);
			for(total=0; total<(Lcd_Dev.width)*range; total++){
				LCD_ADDR->lcd_data = color;
			}
		}
	}
	for(index=0; index<480/range; index++){
		if(index%2 == 1){
			lcdSetWindow(0, index*range,320-1, index*range+range);
			for(total=0; total<(Lcd_Dev.width)*range; total++){
				LCD_ADDR->lcd_data = color;
			}
		}
	}

#endif
	
	//lcdDisplayOn();
#endif
}
//显示 开
void lcdDisplayOn(void)
{
	lcdWriteReg(0x0029, 0);
}
//显示 关
void lcdDisplayOff(void)
{
	lcdWriteReg(0x0028, 0);
}
//设置光标位置
void lcdSetCursor(u16 x, u16 y)
{
	lcdWriteCmd(Lcd_Dev.x_cmd);	
	lcdWriteData(x>>8);
	lcdWriteData(0x00FF&x);
	lcdWriteData((x)>>8);
	lcdWriteData((x));

	lcdWriteCmd(Lcd_Dev.y_cmd);	
	lcdWriteData(y>>8);
	lcdWriteData(0x00FF&y);		
	lcdWriteData((y)>>8);
	lcdWriteData((y));
	//开始写寄存器
	lcdWriteMem();
}
//设定一个矩形显示区
void lcdSetWindow(u16 x_start, u16 y_start, u16 x_end, u16 y_end)
{
	//设置列边界
	lcdWriteCmd(Lcd_Dev.x_cmd);	
	lcdWriteData(x_start>>8);		//起始列高8位
	lcdWriteData(0x00FF&x_start);	//起始列低8位
	lcdWriteData(x_end>>8);			//结束列高8位
	lcdWriteData(0x00FF&x_end);		//结束列低8位
	//设置行边界
	lcdWriteCmd(Lcd_Dev.y_cmd);	
	lcdWriteData(y_start>>8);
	lcdWriteData(0x00FF&y_start);
	lcdWriteData(y_end>>8);
	lcdWriteData(0x00FF&y_end);
	//开始写寄存器 
	lcdWriteMem();
}

//在屏幕上绘制一个像素点
void lcdDrawPoint(u16 x, u16 y, LcdPen* pen)
{
	lcdSetCursor(x,y);
	LCD_ADDR->lcd_cmd = 0x2c;
	LCD_ADDR->lcd_data = pen->color;
}

void lcdDrawLine(u16 x1,u16 y1,u16 x2,u16 y2,LcdPen* pen)
{
	float temp,x,y,z;
	u16 i,j,ax,ay;
	ax = x2;
	ay = y2;
	/*
	x = 1;
	y = 0;
	x = x1>x2?x1 - x2:x2 - x1;
	y = y1>y2?y1 - y2:y2 - y1;
	*/

	x = x1 - x2;
	y = y1 - y2;
	if(x1 != x2)
		temp = y/x;
	
	if(x1 > x2)
	{
		ax = x1;
		x1 = x2;
		ay = y1;
		y1 = y2;
	}
	
	if(x1 == ax)
	{
		for(i = y1;i <= ay;i++)
			lcdDrawPoint(x1,i,pen);
		return;
	}
	if(y1 == ay)
	{
		for(i = x1;i <= ax;i++)
			lcdDrawPoint(i,y1,pen);
		return;
	}
	else
		for(i = x1;i <= ax;i++)
		{
			z = temp*(i-x1)+y1;
			if(temp >= 1)
				for(j = 0;j <= (u16)temp;j++)
					lcdDrawPoint(i,z+j,pen);
				lcdDrawPoint(i,z,pen);
		}
}

void lcdDrawRect(u16 x,u16 y,u16 width,u16 height,LcdPen* pen)
{
	u32 i;
	u32 total = width*height;
	if(P_SET_BRUSH == pen->brush)
	{
		/*for(i = 0;i <= height;i++)
			lcdDrawLine(x, y+i, x+width, y+i,pen);
		*/
		lcdSetWindow(x,y,x+width,y+height);
		for(i = 0;i < total;i++)
			LCD_ADDR->lcd_data = pen->color;
		
	}
	else if(P_SET_NOBRUSH == pen->brush)
	{
		lcdDrawLine(x, y, x+width, y,pen);
		lcdDrawLine(x, y, x, y+height,pen);
		lcdDrawLine(x, y+height, x+width, y+height,pen);
		lcdDrawLine(x+width,y,x+width,y+height,pen);
	}
}


//绘制一个字符
u8 lcdDrawChr(u16 x, u16 y, u8 ch, LcdPen* p)
{
	u8	byte_x;//当前点阵库所用字节数
	u8	byte, bit;
	u8	temp;
	u16 y0 = y;//当前字体所占像素高度
	
	ch -= ' ';//字库中从‘ ’开始显示
	
	switch(p->p_size)
	{
		case P_S16EN:
			byte_x = 16;
			break;
		case P_S32EN:
			byte_x = 64;
			break;
		default:
			return 1;
	}
	
	//外层循环表示，打印每个字符
	for(byte=0; byte<byte_x; byte++)
	{
		//获取字符对应的点阵字节
		switch(p->p_size)
		{
			case P_S16EN:
				temp = asc2_1608[ch][byte];
				break;
			case P_S32EN:
				temp = asc2_3210[ch][byte];
				break;
			default:
				return 1;
		}
		
		//逐位显示在液晶上
		for(bit=0; bit<8; bit++)
		{
			if(temp&0x80)
			{
				lcdDrawPoint(x, y, p);
			}
			temp <<= 1;//打印下一位
			y++;//垂直方向下移一个像素
			if(y>Lcd_Dev.height)
			{
				return 1;
			}
		}

		if((y-y0)==p->p_size)
		{
			y = y0;
			x++;
			//判断是否超出水平边界
			if(x>Lcd_Dev.width)
			{
				return 1;
			}
		}
	}
	return 0;
}

//在指定位置绘制一个字符串，成功返回0否则返回非0
u8 lcdDrawStr(u16 x, u16 y, u8* str, LcdPen* p)
{
	u8 inc = 0;
	
	switch(p->p_size)
	{
		case P_S16EN:
			inc = 16;
			break;
		case P_S32EN:
			inc = 32;
			break;
		default:
			return 1;
	}
	
	while(*str)
	{
		if(P_DIR_HOR==p->direct)
		{
			if(*str>=0x81)
			{
				x += inc;
				str += 2;
			}
			else
			{
				if(lcdDrawChr(x, y, *str, p))
				{
					return 1;
				}
				x += inc/2;	
				str += 1;
			}
		}
		else if(P_DIR_VER==p->direct)
		{
			if(*str>=0x81)
			{
				str += 2;
			}
			else
			{
				if(lcdDrawChr(x, y, *str, p))
				{
					return 1;
				}	
				str += 1;
			}
			y += inc;
		}
	}
	return 0;
}


//绘制一个由四个像素组成的点
void lcdDrawBigPoint(u16 x, u16 y, LcdPen* p)
{
	lcdDrawPoint(x, y,  p);
	lcdDrawPoint(x+1, y, p);
	lcdDrawPoint(x, y+1, p);
	lcdDrawPoint(x+1, y+1, p);
}

u16 LCD_DecToRGB(u8 R, u8 G, u8 B)
{
	u16 color;
	color = R;
	color <<= 5;
	color = color + G;
	color <<= 6;
	color = color +B;
	return color;
}


//!-2
/* **************************************************
 * 静态函数实现
 * *************************************************/
//向LCD写入16位指令
static void lcdWriteCmd(u16 cmd)
{
	LCD_ADDR->lcd_cmd = cmd;
}
//向LCD写入16位数据
static void lcdWriteData(u16 data)
{
	LCD_ADDR->lcd_data = data;
}
//读取LCD数据
static u16 lcdReadData(void)
{
	return LCD_ADDR->lcd_data;
}
//写指定寄存器
static void lcdWriteReg(u8 lcd_reg, u16 reg_val)
{
	LCD_ADDR->lcd_cmd = lcd_reg;
	LCD_ADDR->lcd_data = reg_val;
}
//读取指定寄存器
static u16 lcdReadReg(u8 lcd_reg)
{
	lcdWriteCmd(lcd_reg);
	_delay_us(5);
	return lcdReadData();
}
/* *********************************************************** 
 * Memory Write
 * This command transfers image data from the host processor
 * to ILI9486L’s frame memory starting at the pixel location 
 * specified by preceding Column Address Set (2Ah) and Page 
 * Address Set (2Bh) commands.
 * ***********************************************************/
static void lcdWriteMem(void)
{
	LCD_ADDR->lcd_cmd = Lcd_Dev.wr_cmd; //2Ch
}
//设置设备参数
static void lcdSetParam(void)
{
	//Column Address set
	Lcd_Dev.x_cmd = 0x2A;
	//Page Address Set
	Lcd_Dev.y_cmd = 0x2B;
	/* *********************************************************** 
	 * Memory Write
	 * This command transfers image data from the host processor
	 * to ILI9486L’s frame memory starting at the pixel location 
	 * specified by preceding Column Address Set (2Ah) and Page 
	 * Address Set (2Bh) commands.
	 * ***********************************************************/
	Lcd_Dev.wr_cmd = 0x2C;

#if LCD_VIEW_MODE == LCD_HOR_MODE
	Lcd_Dev.d_mode = LCD_HOR_MODE;
	Lcd_Dev.width = 480;
	Lcd_Dev.height = 320;
	lcdWriteReg(0x36, 0x30);
#else
	Lcd_Dev.d_mode = LCD_VER_MODE;
	Lcd_Dev.width = 320;
	Lcd_Dev.height = 480;
	lcdWriteReg(0x36, 0x52);
#endif
}









