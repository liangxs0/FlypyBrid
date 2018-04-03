#line 1 "app\\Font_Num.c"
#line 1 "app\\Font_Num.h"
#line 1 "app\\game.h"
#line 1 ".\\devices\\jpstm32_lcd.h"













 









































 

#line 1 ".\\core\\jpstm32_gpio.h"







 




#line 1 ".\\core\\jpstm32_common.h"







 



#line 1 ".\\core\\stm32f10x_map.h"














 

 







 
#line 1 ".\\core\\stm32f10x_conf.h"













 

 



 
#line 1 ".\\core\\stm32f10x_type.h"














 

 



 
 
typedef signed long  s32;
typedef signed short s16;
typedef signed char  s8;

typedef signed long  const sc32;   
typedef signed short const sc16;   
typedef signed char  const sc8;    

typedef volatile signed long  vs32;
typedef volatile signed short vs16;
typedef volatile signed char  vs8;

typedef volatile signed long  const vsc32;   
typedef volatile signed short const vsc16;   
typedef volatile signed char  const vsc8;    

typedef unsigned long  u32;
typedef unsigned short u16;
typedef unsigned char  u8;

typedef unsigned long  const uc32;   
typedef unsigned short const uc16;   
typedef unsigned char  const uc8;    

typedef volatile unsigned long  vu32;
typedef volatile unsigned short vu16;
typedef volatile unsigned char  vu8;

typedef volatile unsigned long  const vuc32;   
typedef volatile unsigned short const vuc16;   
typedef volatile unsigned char  const vuc8;    

typedef enum {FALSE = 0, TRUE = !FALSE} bool;

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;


typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

#line 73 ".\\core\\stm32f10x_type.h"

 
 
 



 
#line 22 ".\\core\\stm32f10x_conf.h"

 
 


 
 

 
 





 


 


 


 


 


 
#line 66 ".\\core\\stm32f10x_conf.h"

 


 



 


 


 
#line 90 ".\\core\\stm32f10x_conf.h"

 




 


 


 


 


 


 


 





 


 
#line 133 ".\\core\\stm32f10x_conf.h"

 
#line 141 ".\\core\\stm32f10x_conf.h"

 



 


 
#line 167 ".\\core\\stm32f10x_conf.h"



 
#line 27 ".\\core\\stm32f10x_map.h"
#line 28 ".\\core\\stm32f10x_map.h"
#line 1 ".\\core\\cortexm3_macro.h"













 

 



 
#line 22 ".\\core\\cortexm3_macro.h"

 
 
 
 
void __WFI(void);
void __WFE(void);
void __SEV(void);
void __ISB(void);
void __DSB(void);
void __DMB(void);
void __SVC(void);
u32 __MRS_CONTROL(void);
void __MSR_CONTROL(u32 Control);
u32 __MRS_PSP(void);
void __MSR_PSP(u32 TopOfProcessStack);
u32 __MRS_MSP(void);
void __MSR_MSP(u32 TopOfMainStack);
void __RESETPRIMASK(void);
void __SETPRIMASK(void);
u32 __READ_PRIMASK(void);
void __RESETFAULTMASK(void);
void __SETFAULTMASK(void);
u32 __READ_FAULTMASK(void);
void __BASEPRICONFIG(u32 NewPriority);
u32 __GetBASEPRI(void);
u16 __REV_HalfWord(u16 Data);
u32 __REV_Word(u32 Data);



 
#line 29 ".\\core\\stm32f10x_map.h"

 
 
 
 

 
typedef struct
{
  vu32 SR;
  vu32 CR1;
  vu32 CR2;
  vu32 SMPR1;
  vu32 SMPR2;
  vu32 JOFR1;
  vu32 JOFR2;
  vu32 JOFR3;
  vu32 JOFR4;
  vu32 HTR;
  vu32 LTR;
  vu32 SQR1;
  vu32 SQR2;
  vu32 SQR3;
  vu32 JSQR;
  vu32 JDR1;
  vu32 JDR2;
  vu32 JDR3;
  vu32 JDR4;
  vu32 DR;
} ADC_TypeDef;

 
typedef struct
{
  u32  RESERVED0;
  vu16 DR1;
  u16  RESERVED1;
  vu16 DR2;
  u16  RESERVED2;
  vu16 DR3;
  u16  RESERVED3;
  vu16 DR4;
  u16  RESERVED4;
  vu16 DR5;
  u16  RESERVED5;
  vu16 DR6;
  u16  RESERVED6;
  vu16 DR7;
  u16  RESERVED7;
  vu16 DR8;
  u16  RESERVED8;
  vu16 DR9;
  u16  RESERVED9;
  vu16 DR10;
  u16  RESERVED10; 
  vu16 RTCCR;
  u16  RESERVED11;
  vu16 CR;
  u16  RESERVED12;
  vu16 CSR;
  u16  RESERVED13[5];
  vu16 DR11;
  u16  RESERVED14;
  vu16 DR12;
  u16  RESERVED15;
  vu16 DR13;
  u16  RESERVED16;
  vu16 DR14;
  u16  RESERVED17;
  vu16 DR15;
  u16  RESERVED18;
  vu16 DR16;
  u16  RESERVED19;
  vu16 DR17;
  u16  RESERVED20;
  vu16 DR18;
  u16  RESERVED21;
  vu16 DR19;
  u16  RESERVED22;
  vu16 DR20;
  u16  RESERVED23;
  vu16 DR21;
  u16  RESERVED24;
  vu16 DR22;
  u16  RESERVED25;
  vu16 DR23;
  u16  RESERVED26;
  vu16 DR24;
  u16  RESERVED27;
  vu16 DR25;
  u16  RESERVED28;
  vu16 DR26;
  u16  RESERVED29;
  vu16 DR27;
  u16  RESERVED30;
  vu16 DR28;
  u16  RESERVED31;
  vu16 DR29;
  u16  RESERVED32;
  vu16 DR30;
  u16  RESERVED33; 
  vu16 DR31;
  u16  RESERVED34;
  vu16 DR32;
  u16  RESERVED35;
  vu16 DR33;
  u16  RESERVED36;
  vu16 DR34;
  u16  RESERVED37;
  vu16 DR35;
  u16  RESERVED38;
  vu16 DR36;
  u16  RESERVED39;
  vu16 DR37;
  u16  RESERVED40;
  vu16 DR38;
  u16  RESERVED41;
  vu16 DR39;
  u16  RESERVED42;
  vu16 DR40;
  u16  RESERVED43;
  vu16 DR41;
  u16  RESERVED44;
  vu16 DR42;
  u16  RESERVED45;    
} BKP_TypeDef;

 
typedef struct
{
  vu32 TIR;
  vu32 TDTR;
  vu32 TDLR;
  vu32 TDHR;
} CAN_TxMailBox_TypeDef;

typedef struct
{
  vu32 RIR;
  vu32 RDTR;
  vu32 RDLR;
  vu32 RDHR;
} CAN_FIFOMailBox_TypeDef;

typedef struct
{
  vu32 FR1;
  vu32 FR2;
} CAN_FilterRegister_TypeDef;

typedef struct
{
  vu32 MCR;
  vu32 MSR;
  vu32 TSR;
  vu32 RF0R;
  vu32 RF1R;
  vu32 IER;
  vu32 ESR;
  vu32 BTR;
  u32  RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
  u32  RESERVED1[12];
  vu32 FMR;
  vu32 FM1R;
  u32  RESERVED2;
  vu32 FS1R;
  u32  RESERVED3;
  vu32 FFA1R;
  u32  RESERVED4;
  vu32 FA1R;
  u32  RESERVED5[8];
  CAN_FilterRegister_TypeDef sFilterRegister[14];
} CAN_TypeDef;

 
typedef struct
{
  vu32 DR;
  vu8  IDR;
  u8   RESERVED0;
  u16  RESERVED1;
  vu32 CR;
} CRC_TypeDef;


 
typedef struct
{
  vu32 CR;
  vu32 SWTRIGR;
  vu32 DHR12R1;
  vu32 DHR12L1;
  vu32 DHR8R1;
  vu32 DHR12R2;
  vu32 DHR12L2;
  vu32 DHR8R2;
  vu32 DHR12RD;
  vu32 DHR12LD;
  vu32 DHR8RD;
  vu32 DOR1;
  vu32 DOR2;
} DAC_TypeDef;

 
typedef struct
{
  vu32 IDCODE;
  vu32 CR;	
}DBGMCU_TypeDef;

 
typedef struct
{
  vu32 CCR;
  vu32 CNDTR;
  vu32 CPAR;
  vu32 CMAR;
} DMA_Channel_TypeDef;

typedef struct
{
  vu32 ISR;
  vu32 IFCR;
} DMA_TypeDef;

 
typedef struct
{
  vu32 IMR;
  vu32 EMR;
  vu32 RTSR;
  vu32 FTSR;
  vu32 SWIER;
  vu32 PR;
} EXTI_TypeDef;

 
typedef struct
{
  vu32 ACR;
  vu32 KEYR;
  vu32 OPTKEYR;
  vu32 SR;
  vu32 CR;
  vu32 AR;
  vu32 RESERVED;
  vu32 OBR;
  vu32 WRPR;
} FLASH_TypeDef;

typedef struct
{
  vu16 RDP;
  vu16 USER;
  vu16 Data0;
  vu16 Data1;
  vu16 WRP0;
  vu16 WRP1;
  vu16 WRP2;
  vu16 WRP3;
} OB_TypeDef;

 
typedef struct
{
  vu32 BTCR[8];
} FSMC_Bank1_TypeDef; 

typedef struct
{
  vu32 BWTR[7];
} FSMC_Bank1E_TypeDef;

typedef struct
{
  vu32 PCR2;
  vu32 SR2;
  vu32 PMEM2;
  vu32 PATT2;
  u32  RESERVED0;   
  vu32 ECCR2; 
} FSMC_Bank2_TypeDef;  

typedef struct
{
  vu32 PCR3;
  vu32 SR3;
  vu32 PMEM3;
  vu32 PATT3;
  u32  RESERVED0;   
  vu32 ECCR3; 
} FSMC_Bank3_TypeDef; 

typedef struct
{
  vu32 PCR4;
  vu32 SR4;
  vu32 PMEM4;
  vu32 PATT4;
  vu32 PIO4; 
} FSMC_Bank4_TypeDef; 

 
typedef struct
{
  vu32 CRL;
  vu32 CRH;
  vu32 IDR;
  vu32 ODR;
  vu32 BSRR;
  vu32 BRR;
  vu32 LCKR;
} GPIO_TypeDef;

typedef struct
{
  vu32 EVCR;
  vu32 MAPR;
  vu32 EXTICR[4];
} AFIO_TypeDef;

 
typedef struct
{
  vu16 CR1;
  u16  RESERVED0;
  vu16 CR2;
  u16  RESERVED1;
  vu16 OAR1;
  u16  RESERVED2;
  vu16 OAR2;
  u16  RESERVED3;
  vu16 DR;
  u16  RESERVED4;
  vu16 SR1;
  u16  RESERVED5;
  vu16 SR2;
  u16  RESERVED6;
  vu16 CCR;
  u16  RESERVED7;
  vu16 TRISE;
  u16  RESERVED8;
} I2C_TypeDef;

 
typedef struct
{
  vu32 KR;
  vu32 PR;
  vu32 RLR;
  vu32 SR;
} IWDG_TypeDef;

 
typedef struct
{
  vu32 ISER[2];
  u32  RESERVED0[30];
  vu32 ICER[2];
  u32  RSERVED1[30];
  vu32 ISPR[2];
  u32  RESERVED2[30];
  vu32 ICPR[2];
  u32  RESERVED3[30];
  vu32 IABR[2];
  u32  RESERVED4[62];
  vu32 IPR[15];
} NVIC_TypeDef;

typedef struct
{
  vuc32 CPUID;
  vu32 ICSR;
  vu32 VTOR;
  vu32 AIRCR;
  vu32 SCR;
  vu32 CCR;
  vu32 SHPR[3];
  vu32 SHCSR;
  vu32 CFSR;
  vu32 HFSR;
  vu32 DFSR;
  vu32 MMFAR;
  vu32 BFAR;
  vu32 AFSR;
} SCB_TypeDef;

 
typedef struct
{
  vu32 CR;
  vu32 CSR;
} PWR_TypeDef;

 
typedef struct
{
  vu32 CR;
  vu32 CFGR;
  vu32 CIR;
  vu32 APB2RSTR;
  vu32 APB1RSTR;
  vu32 AHBENR;
  vu32 APB2ENR;
  vu32 APB1ENR;
  vu32 BDCR;
  vu32 CSR;
} RCC_TypeDef;

 
typedef struct
{
  vu16 CRH;
  u16  RESERVED0;
  vu16 CRL;
  u16  RESERVED1;
  vu16 PRLH;
  u16  RESERVED2;
  vu16 PRLL;
  u16  RESERVED3;
  vu16 DIVH;
  u16  RESERVED4;
  vu16 DIVL;
  u16  RESERVED5;
  vu16 CNTH;
  u16  RESERVED6;
  vu16 CNTL;
  u16  RESERVED7;
  vu16 ALRH;
  u16  RESERVED8;
  vu16 ALRL;
  u16  RESERVED9;
} RTC_TypeDef;

 
typedef struct
{
  vu32 POWER;
  vu32 CLKCR;
  vu32 ARG;
  vu32 CMD;
  vuc32 RESPCMD;
  vuc32 RESP1;
  vuc32 RESP2;
  vuc32 RESP3;
  vuc32 RESP4;
  vu32 DTIMER;
  vu32 DLEN;
  vu32 DCTRL;
  vuc32 DCOUNT;
  vuc32 STA;
  vu32 ICR;
  vu32 MASK;
  u32  RESERVED0[2];
  vuc32 FIFOCNT;
  u32  RESERVED1[13];
  vu32 FIFO;
} SDIO_TypeDef;

 
typedef struct
{
  vu16 CR1;
  u16  RESERVED0;
  vu16 CR2;
  u16  RESERVED1;
  vu16 SR;
  u16  RESERVED2;
  vu16 DR;
  u16  RESERVED3;
  vu16 CRCPR;
  u16  RESERVED4;
  vu16 RXCRCR;
  u16  RESERVED5;
  vu16 TXCRCR;
  u16  RESERVED6;
  vu16 I2SCFGR;
  u16  RESERVED7;
  vu16 I2SPR;
  u16  RESERVED8;  
} SPI_TypeDef;

 
typedef struct
{
  vu32 CTRL;
  vu32 LOAD;
  vu32 VAL;
  vuc32 CALIB;
} SysTick_TypeDef;

 
typedef struct
{
  vu16 CR1;
  u16  RESERVED0;
  vu16 CR2;
  u16  RESERVED1;
  vu16 SMCR;
  u16  RESERVED2;
  vu16 DIER;
  u16  RESERVED3;
  vu16 SR;
  u16  RESERVED4;
  vu16 EGR;
  u16  RESERVED5;
  vu16 CCMR1;
  u16  RESERVED6;
  vu16 CCMR2;
  u16  RESERVED7;
  vu16 CCER;
  u16  RESERVED8;
  vu16 CNT;
  u16  RESERVED9;
  vu16 PSC;
  u16  RESERVED10;
  vu16 ARR;
  u16  RESERVED11;
  vu16 RCR;
  u16  RESERVED12;
  vu16 CCR1;
  u16  RESERVED13;
  vu16 CCR2;
  u16  RESERVED14;
  vu16 CCR3;
  u16  RESERVED15;
  vu16 CCR4;
  u16  RESERVED16;
  vu16 BDTR;
  u16  RESERVED17;
  vu16 DCR;
  u16  RESERVED18;
  vu16 DMAR;
  u16  RESERVED19;
} TIM_TypeDef;

 
typedef struct
{
  vu16 SR;
  u16  RESERVED0;
  vu16 DR;
  u16  RESERVED1;
  vu16 BRR;
  u16  RESERVED2;
  vu16 CR1;
  u16  RESERVED3;
  vu16 CR2;
  u16  RESERVED4;
  vu16 CR3;
  u16  RESERVED5;
  vu16 GTPR;
  u16  RESERVED6;
} USART_TypeDef;

 
typedef struct
{
  vu32 CR;
  vu32 CFR;
  vu32 SR;
} WWDG_TypeDef;

 
 
 
 



 



 


 




#line 634 ".\\core\\stm32f10x_map.h"

#line 651 ".\\core\\stm32f10x_map.h"



#line 670 ".\\core\\stm32f10x_map.h"

 

 


 






 


 






 
 
 

 



























































































































































































































#line 924 ".\\core\\stm32f10x_map.h"














 
#line 1180 ".\\core\\stm32f10x_map.h"

 
 
 



 
#line 13 ".\\core\\jpstm32_common.h"
#line 14 ".\\core\\jpstm32_common.h"
#line 1 ".\\core\\stm32f10x_nvic.h"














 

 



 
#line 23 ".\\core\\stm32f10x_nvic.h"

 
 
typedef struct
{
  u8 NVIC_IRQChannel;
  u8 NVIC_IRQChannelPreemptionPriority;
  u8 NVIC_IRQChannelSubPriority;
  FunctionalState NVIC_IRQChannelCmd;
} NVIC_InitTypeDef;

 
 
#line 96 ".\\core\\stm32f10x_nvic.h"


#line 158 ".\\core\\stm32f10x_nvic.h"


 
#line 170 ".\\core\\stm32f10x_nvic.h"





#line 182 ".\\core\\stm32f10x_nvic.h"












#line 201 ".\\core\\stm32f10x_nvic.h"











 






 








 
#line 239 ".\\core\\stm32f10x_nvic.h"












 
 
void NVIC_DeInit(void);
void NVIC_SCBDeInit(void);
void NVIC_PriorityGroupConfig(u32 NVIC_PriorityGroup);
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);
void NVIC_StructInit(NVIC_InitTypeDef* NVIC_InitStruct);
void NVIC_SETPRIMASK(void);
void NVIC_RESETPRIMASK(void);
void NVIC_SETFAULTMASK(void);
void NVIC_RESETFAULTMASK(void);
void NVIC_BASEPRICONFIG(u32 NewPriority);
u32 NVIC_GetBASEPRI(void);
u16 NVIC_GetCurrentPendingIRQChannel(void);
ITStatus NVIC_GetIRQChannelPendingBitStatus(u8 NVIC_IRQChannel);
void NVIC_SetIRQChannelPendingBit(u8 NVIC_IRQChannel);
void NVIC_ClearIRQChannelPendingBit(u8 NVIC_IRQChannel);
u16 NVIC_GetCurrentActiveHandler(void);
ITStatus NVIC_GetIRQChannelActiveBitStatus(u8 NVIC_IRQChannel);
u32 NVIC_GetCPUID(void);
void NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset);
void NVIC_GenerateSystemReset(void);
void NVIC_GenerateCoreReset(void);
void NVIC_SystemLPConfig(u8 LowPowerMode, FunctionalState NewState);
void NVIC_SystemHandlerConfig(u32 SystemHandler, FunctionalState NewState);
void NVIC_SystemHandlerPriorityConfig(u32 SystemHandler, u8 SystemHandlerPreemptionPriority,
                                      u8 SystemHandlerSubPriority);
ITStatus NVIC_GetSystemHandlerPendingBitStatus(u32 SystemHandler);
void NVIC_SetSystemHandlerPendingBit(u32 SystemHandler);
void NVIC_ClearSystemHandlerPendingBit(u32 SystemHandler);
ITStatus NVIC_GetSystemHandlerActiveBitStatus(u32 SystemHandler);
u32 NVIC_GetFaultHandlerSources(u32 SystemHandler);
u32 NVIC_GetFaultAddress(u32 SystemHandler);



 
#line 15 ".\\core\\jpstm32_common.h"

 



 





typedef enum{
    RISING = 0,FALLING,BOTH_EDGE
}EXTI_TRIGGER_MODE;


typedef enum{
    IRQ = 0,EVENT,BOTH_INT_EVENT
}INT_EVENT_MODE;


typedef enum{
    PORTA = 0,PORTB,PORTC,PORTD,PORTE,PORTF,PORTG
}PORT;

#line 14 ".\\core\\jpstm32_gpio.h"


typedef enum{
	VAL_0=0, 	 
	VAL_1		 
} PIN_VAL; 

typedef enum{
	PULL_UP,	 
	PULL_DOWN	 
} PULL_STAT;


#line 34 ".\\core\\jpstm32_gpio.h"








#line 50 ".\\core\\jpstm32_gpio.h"


#line 68 ".\\core\\jpstm32_gpio.h"












 
void config_gpio(u32 PORTx, u32 MODE, u32 PINxs, PULL_STAT stat);










 
void set_PINxs(u32	PORTx, u32 PINxs, PIN_VAL val);




 


#line 108 ".\\core\\jpstm32_gpio.h"

#line 116 ".\\core\\jpstm32_gpio.h"




#line 59 ".\\devices\\jpstm32_lcd.h"
#line 1 "D:\\Program Files\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"
 
 
 




 
 



 













  


 








#line 46 "D:\\Program Files\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"


  
  typedef unsigned int size_t;










    



    typedef unsigned short wchar_t;  
#line 75 "D:\\Program Files\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"

typedef struct div_t { int quot, rem; } div_t;
    
typedef struct ldiv_t { long int quot, rem; } ldiv_t;
    

typedef struct lldiv_t { __int64 quot, rem; } lldiv_t;
    


#line 96 "D:\\Program Files\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"
   



 

   




 
#line 115 "D:\\Program Files\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"
   


 
extern __declspec(__nothrow) int __aeabi_MB_CUR_MAX(void);

   




 

   




 




extern __declspec(__nothrow) double atof(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int atoi(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) long int atol(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) __int64 atoll(const char *  ) __attribute__((__nonnull__(1)));
   



 


extern __declspec(__nothrow) double strtod(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

















 

extern __declspec(__nothrow) float strtof(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) long double strtold(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

 

extern __declspec(__nothrow) long int strtol(const char * __restrict  ,
                        char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   



























 
extern __declspec(__nothrow) unsigned long int strtoul(const char * __restrict  ,
                                       char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   


























 

 
extern __declspec(__nothrow) __int64 strtoll(const char * __restrict  ,
                               char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) unsigned __int64 strtoull(const char * __restrict  ,
                                         char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) int rand(void);
   







 
extern __declspec(__nothrow) void srand(unsigned int  );
   






 

struct _rand_state { int __x[57]; };
extern __declspec(__nothrow) int _rand_r(struct _rand_state *);
extern __declspec(__nothrow) void _srand_r(struct _rand_state *, unsigned int);
struct _ANSI_rand_state { int __x[1]; };
extern __declspec(__nothrow) int _ANSI_rand_r(struct _ANSI_rand_state *);
extern __declspec(__nothrow) void _ANSI_srand_r(struct _ANSI_rand_state *, unsigned int);
   


 

extern __declspec(__nothrow) void *calloc(size_t  , size_t  );
   



 
extern __declspec(__nothrow) void free(void *  );
   





 
extern __declspec(__nothrow) void *malloc(size_t  );
   



 
extern __declspec(__nothrow) void *realloc(void *  , size_t  );
   













 

extern __declspec(__nothrow) int posix_memalign(void **  , size_t  , size_t  );
   









 

typedef int (*__heapprt)(void *, char const *, ...);
extern __declspec(__nothrow) void __heapstats(int (*  )(void *  ,
                                           char const *  , ...),
                        void *  ) __attribute__((__nonnull__(1)));
   










 
extern __declspec(__nothrow) int __heapvalid(int (*  )(void *  ,
                                           char const *  , ...),
                       void *  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) __declspec(__noreturn) void abort(void);
   







 

extern __declspec(__nothrow) int atexit(void (*  )(void)) __attribute__((__nonnull__(1)));
   




 
#line 415 "D:\\Program Files\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"


extern __declspec(__nothrow) __declspec(__noreturn) void exit(int  );
   












 

extern __declspec(__nothrow) __declspec(__noreturn) void _Exit(int  );
   







      

extern __declspec(__nothrow) char *getenv(const char *  ) __attribute__((__nonnull__(1)));
   









 

extern __declspec(__nothrow) int  system(const char *  );
   









 

extern  void *bsearch(const void *  , const void *  ,
              size_t  , size_t  ,
              int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,2,5)));
   












 
#line 503 "D:\\Program Files\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"


extern  void qsort(void *  , size_t  , size_t  ,
           int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,4)));
   









 

#line 532 "D:\\Program Files\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"

extern __declspec(__nothrow) __pure int abs(int  );
   



 

extern __declspec(__nothrow) __pure div_t div(int  , int  );
   









 
extern __declspec(__nothrow) __pure long int labs(long int  );
   



 




extern __declspec(__nothrow) __pure ldiv_t ldiv(long int  , long int  );
   











 







extern __declspec(__nothrow) __pure __int64 llabs(__int64  );
   



 




extern __declspec(__nothrow) __pure lldiv_t lldiv(__int64  , __int64  );
   











 
#line 613 "D:\\Program Files\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"



 
typedef struct __sdiv32by16 { int quot, rem; } __sdiv32by16;
typedef struct __udiv32by16 { unsigned int quot, rem; } __udiv32by16;
    
typedef struct __sdiv64by32 { int rem, quot; } __sdiv64by32;

__value_in_regs extern __declspec(__nothrow) __pure __sdiv32by16 __rt_sdiv32by16(
     int  ,
     short int  );
   

 
__value_in_regs extern __declspec(__nothrow) __pure __udiv32by16 __rt_udiv32by16(
     unsigned int  ,
     unsigned short  );
   

 
__value_in_regs extern __declspec(__nothrow) __pure __sdiv64by32 __rt_sdiv64by32(
     int  , unsigned int  ,
     int  );
   

 



 
extern __declspec(__nothrow) unsigned int __fp_status(unsigned int  , unsigned int  );
   







 























 
extern __declspec(__nothrow) int mblen(const char *  , size_t  );
   












 
extern __declspec(__nothrow) int mbtowc(wchar_t * __restrict  ,
                   const char * __restrict  , size_t  );
   















 
extern __declspec(__nothrow) int wctomb(char *  , wchar_t  );
   













 





 
extern __declspec(__nothrow) size_t mbstowcs(wchar_t * __restrict  ,
                      const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 
extern __declspec(__nothrow) size_t wcstombs(char * __restrict  ,
                      const wchar_t * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 

extern __declspec(__nothrow) void __use_realtime_heap(void);
extern __declspec(__nothrow) void __use_realtime_division(void);
extern __declspec(__nothrow) void __use_two_region_memory(void);
extern __declspec(__nothrow) void __use_no_heap(void);
extern __declspec(__nothrow) void __use_no_heap_region(void);

extern __declspec(__nothrow) char const *__C_library_version_string(void);
extern __declspec(__nothrow) int __C_library_version_number(void);











#line 867 "D:\\Program Files\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"


 

#line 60 ".\\devices\\jpstm32_lcd.h"





















































 
typedef struct {
	u16		lcd_cmd;	
	u16		lcd_data;	
}LcdAddr;





typedef struct {
	u16		dev_no;
	u16		width;	
	u16		height;	
	u8		d_mode; 
	u8		wr_cmd;	
	u8		x_cmd;	
	u8		y_cmd;
}LcdDev;

extern u16 POINT_COLOR;	
extern u16 BACK_COLOR;  


extern LcdDev	Lcd_Dev;


#line 153 ".\\devices\\jpstm32_lcd.h"



 

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

typedef struct{
	u16 color;
	u16 bg_color;
	PenSize p_size;
	PenBrush brush;
	PenDirect direct;
}LcdPen;



void lcdInit(void);

void lcdClear(u16 color);

void lcdDisplayOn(void);

void lcdDisplayOff(void);

void lcdSetCursor(u16 x, u16 y);

void lcdSetWindow(u16 x_start, u16 y_start, u16 x_end, u16 y_end);


void lcdDrawPoint(u16 x, u16 y, LcdPen* pen);

void lcdDrawLine(u16 x1,u16 y1,u16 x2,u16 y2,LcdPen* pen);

void lcdDrawRect(u16 x,u16 y,u16 width,u16 height,LcdPen* pen);

void lcd_write_cmd(void);

u16 LCD_DecToRGB(u8 R, u8 G, u8 B);


u8 lcdDrawChr(u16 x, u16 y, u8 ch, LcdPen* p);

u8 lcdDrawStr(u16 x, u16 y, u8* str, LcdPen* p);

void lcdDrawBigPoint(u16 x, u16 y, LcdPen* p);













#line 4 "app\\game.h"
#line 1 ".\\core\\jpstm32_usart.h"







 



#line 13 ".\\core\\jpstm32_usart.h"
#line 1 ".\\core\\jpstm32_nvic.h"







 




#line 14 ".\\core\\jpstm32_nvic.h"










 
extern void select_nvic_vector(u32 NVIC_VectTab, u32 Offset);

extern bool nvic_set_enable(u8 Channel);
extern bool nvic_clear_enable(u8 Channel);
extern bool nvic_set_pending(u8 Channel);
extern bool nvic_clear_pending(u8 Channel);
extern bool nvic_get_active(u8 Channel);
extern bool nvic_set_group(u8 group);
extern bool nvic_set_priority(u8 Channel, u8 PreemptionPriority, u8 SubPriority);
extern vu32 nvic_get_priority(void);

extern bool register_nvic(u8 nvic_Group, u8 nvic_PreemptionPriority, u8 nvic_SubPriority, u8 nvic_Channel);


#line 14 ".\\core\\jpstm32_usart.h"
#line 1 "D:\\Program Files\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"
 
 
 





 






 













#line 38 "D:\\Program Files\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"


  
  typedef unsigned int size_t;    








 
 

 
  typedef struct __va_list __va_list;





   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 129 "D:\\Program Files\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 
extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 948 "D:\\Program Files\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"



 

#line 15 ".\\core\\jpstm32_usart.h"









typedef struct {
	s8	rx_buf[256];
	u16	rx_seek;
	u16	rx_size;
	u8	rx_stat;
}USARTypDef;
	  	

void usart1_init(u32 pclk2,u32 bound);

s8	usart1_getc(void);

u8 usart1_gets(s8* buf, u16 len);


#line 5 "app\\game.h"
#line 1 ".\\core\\jpstm32_sysclk.h"







 




#line 14 ".\\core\\jpstm32_sysclk.h"










 
 extern void clk_init(u8 pll);

#line 6 "app\\game.h"
#line 1 ".\\devices\\jpstm32_key.h"







 


 
#line 13 ".\\devices\\jpstm32_key.h"
 
 
 

 

 
 typedef enum{
	 KEY_NOKEY,		 
	 KEY_UP,		 
	 KEY_DOWN,		 
	 KEY_LEFT,		 
	 KEY_RIGHT,		 
	 KEY_CENTER		 
}KEY_VAL;








 
extern void key_init(void);








 
extern KEY_VAL key_scan(void);





#line 7 "app\\game.h"
#line 1 ".\\devices\\dma.h"
#line 4 ".\\devices\\dma.h"


void DMA_Init(void);

void DMA_LCD(void);


void DMA_Enable(void);





#line 8 "app\\game.h"
#line 1 ".\\devices\\touch.h"







 




#line 14 ".\\devices\\touch.h"
#line 15 ".\\devices\\touch.h"












typedef struct{
	u8 	(*init)(void);
	u8	(*scan)(u8);
	u8 	(*adjust)(void);
	
	u16		lastX;
	u16		lastY;
	
	u16		currX;
	u16		currY;
	
	u8		penStat;
	float			xfac;
	float 		yfac;
	short			xoff;
	short			yoff;
	PenDirect direct;
	u8				cmdRdx;
	u8				cmdRdy;
}TouchTpyDef;

static TouchTpyDef	t_pad;








 
void tp_spiWbyte(u8 byte);








 
u16 tp_readAD(u8 cmd);









 
u16 tp_readXorY(u8 cmd);








 
u8 tp_readXandY(u16* x, u16* y);










 
u8 tp_readXandY2(u16* x, u16* y);








 
void tp_drawAdjustPoint(u16 x, u16 y, u16 color);








 
u8 tp_scan(u8 tp);








 
u8 tp_init(void);








 
u8 tp_adjust(void);








 
u8 tp_printAdjustInfo(u16 pos[][2], u16 fac);








 
void tp_screenTrack(void);












#line 9 "app\\game.h"
#line 1 ".\\devices\\timer.h"
#line 4 ".\\devices\\timer.h"









void TIM3_Int_Init(u16 arr,u16 psc);
void TIM3_PWM_Init(u16 arr,u16 psc);
void TIM5_Cap_Init(u16 arr,u16 psc);
void TIM4_PWM_Init(u16 arr,u16 psc);
void TIM6_Int_Init(u16 arr,u16 psc);
void LCD_PWM_Init(void);





#line 10 "app\\game.h"
#line 1 ".\\core\\jpstm32_delay.h"







 



#line 13 ".\\core\\jpstm32_delay.h"

extern void _delay_init(u8 SYSCLK);
extern void _delay_ms(u16 ms);
extern void _delay_us(u32 us);

#line 11 "app\\game.h"
#line 1 ".\\devices\\adc.h"
#line 4 ".\\devices\\adc.h"












void Adc_Init(void);
u16  Get_Adc(u8 ch); 
u16 Get_Adc_Average(u8 ch,u8 times); 
 
#line 12 "app\\game.h"
#line 1 "app\\Font_Num.h"
#line 13 "app\\game.h"
#line 1 ".\\devices\\24cxx.h"
#line 1 ".\\devices\\iic.h"



#line 5 ".\\devices\\iic.h"







 








void IIC_Init(void);

void IIC_Start(void);

void IIC_Stop(void);

void IIC_Send_Byte(u8 txd);

u8	IIC_Read_Byte(u8 ack);

u8 IIC_Wait_Ack(void);

void IIC_Ack(void);

void IIC_NAck(void);

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);

u8 IIC_Read_One_Byte(u8 daddr,u8 addr);

#line 4 ".\\devices\\24cxx.h"
















					  
u8 AT24CXX_ReadOneByte(u16 ReadAddr);							
void AT24CXX_WriteOneByte(u16 WriteAddr,u8 DataToWrite);		
void AT24CXX_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len);
u32 AT24CXX_ReadLenByte(u16 ReadAddr,u8 Len);					
void AT24CXX_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite);	
void AT24CXX_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead);   	

u8 AT24CXX_Check(void);  
void AT24CXX_Init(void); 

















#line 14 "app\\game.h"
#line 1 ".\\devices\\fsmc.h"




#line 6 ".\\devices\\fsmc.h"







 



void FSMC_SRAM_init(void);

void FSMC_SRAM_WriteBuffer(u8* pBuffer,u32 WriteAddr,u32 NumHalfwordToWrite);

void FSMC_SRAM_ReadBuffer(u8* pBuffer,u32 ReadAddr,u32 NumHalfwordToRead);

#line 15 "app\\game.h"


extern const unsigned char asc2_1206[95][12];
extern const unsigned char asc2_1608[95][16];
extern u16 FlappyBird_Frame[480][320];
void FlappyBird_Frame_DrawPoint( u16 x, u16 y, u16 color );
void FlappyBird_Frame_Fill( u16 x, u16 y, u16 xx, u16 yy, u16 color );
void FlappyBird_Frame_DrawLine( u16 x1, u16 y1, u16 x2, u16 y2 , u16 color );
void FlappyBird_DrawFrame( void );
void FlappyBird_Frame_Clear( void );
void FlappyBird_Frame_ShowChar(u16 x,u16 y,u8 num,u8 size,u8 mode,u16 color);
void FlappyBird_Frame_ShowString(u16 x,u16 y,u16 width,u16 height,u8 size,u8 *p,u16 color);
void FlappyBird_Frame_DrawBGPic( u16 x,u16 y );
void FlappyBird_Frame_DrawLogo( u16 x, u16 y );
void FlappyBird_Frame_DrawButton( u16 x, u16 y , u8 mode );
void FlappyBird_Frame_DrawBird( u16 x, u16 y , u8 mode );
void FlappyBird_Frame_DrawBird_Play( u16 x, u16 y, u8 mode );
void FlappyBird_Frame_DrawColumn_Play_times( int x, u16 y, u8 level, u8 column, u8 times );
u16 FlappyBird_Frame_MixColor(u16 color_1,u16 color_2,u8 percent);
u16 FlappyBird_Frame_IntensityControl(u16 color,u8 percent,u8 mode); 
void FlappyBird_Frame_AllIntensityControl( u8 percent, u8 mode );

void FlappyBird_DrawGreenBar( u16 x, u16 y, u16 color );
void FlappyBird_DrawGreenBar_Play( u16 x, u16 y,u8 Speed );
void FlappyBird_DrawParallelogram( u16 x, u16 y, u16 color );


void FlappyBird_Stage_Start( void );
void FlappyBird_Stage_PP( void );
void FlappyBird_Stage_Play( void );
void FlappyBird_Stage_Over( void );



#line 4 "app\\Font_Num.h"

void Font_DrawNum_Big( u16 x, u16 y, u8 * num );
void Font_DrawNum_Small( u16 x, u16 y, u8 * num );
void Font_Num_DrawNum( u16 x, u16 y, u16 num, u8 mode );
void Font_DrawNum_Black( u16 x, u16 y, u8 * num );
void Font_Num_DrawNum_Multi( u16 x, u16 y, u32 num, u8 mode );
#line 2 "app\\Font_Num.c"
#line 1 ".\\app\\RES\\Num\\0_big.c"
const unsigned char gImage_0_big[1008] = {  
0XC3,0X18,0XC3,0X18,0X82,0X10,0X82,0X10,0XA2,0X10,0XA3,0X10,0X62,0X08,0X82,0X10,
0X82,0X10,0X62,0X08,0X62,0X08,0X82,0X08,0X82,0X08,0XA2,0X10,0X82,0X08,0X82,0X08,
0XC2,0X10,0XC2,0X10,0XC3,0X18,0XE3,0X18,0X82,0X10,0XA2,0X10,0X82,0X10,0XA3,0X10,
0X61,0X08,0X82,0X10,0X82,0X10,0X82,0X10,0X82,0X08,0X82,0X08,0X82,0X08,0X82,0X08,
0X82,0X08,0X82,0X08,0XC2,0X10,0XC2,0X10,0XA2,0X10,0X82,0X10,0XDF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA2,0X10,0XA2,0X10,0X82,0X10,0XA3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA2,0X10,0XA2,0X10,
0XC3,0X18,0XC3,0X18,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XC3,0X10,0XC3,0X10,0XC3,0X18,0XE3,0X18,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XC3,0X10,0XC3,0X10,0XC3,0X18,0XA2,0X10,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC2,0X10,0XC2,0X10,0XA2,0X10,0XA2,0X10,
0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC2,0X10,0XC2,0X10,
0XA2,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0X61,0X08,0X82,0X10,0XBE,0XF7,0XBE,0XF7,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,
0XA2,0X10,0XA2,0X10,0XA2,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0X82,0X08,0X61,0X08,0XDE,0XF7,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA2,0X10,0XA2,0X10,0X82,0X10,0X82,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X82,0X10,0X82,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA2,0X10,0XA2,0X10,0X82,0X10,0X82,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X82,0X10,0X82,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA2,0X10,0XA2,0X10,
0XA2,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,
0X81,0X10,0X81,0X10,0XBE,0XF7,0XBE,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA2,0X10,0XA2,0X10,0XA2,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0X81,0X10,0X82,0X10,0XBE,0XF7,0XBE,0XF7,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA2,0X10,0XA2,0X10,0XA2,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X82,0X10,0XA2,0X10,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA2,0X10,0XA2,0X10,0XA2,0X10,0XA2,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X82,0X10,0X81,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA2,0X10,0XA2,0X10,
0XA2,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0X82,0X10,0X82,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,
0XA2,0X10,0XA2,0X10,0XA2,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0X81,0X10,0X61,0X08,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XA2,0X10,0XA2,0X10,0XA2,0X10,0XC2,0X18,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X81,0X10,0X61,0X08,0XDF,0XFF,0XDE,0XFF,
0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDE,0XFF,0XA2,0X10,0XA2,0X10,0XA2,0X10,0XC2,0X18,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X41,0X08,0X81,0X10,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XDE,0XFF,0XA2,0X10,0XA2,0X10,
0XC2,0X18,0XC2,0X18,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA2,0X10,0XA2,0X10,0XA2,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA2,0X10,0XA2,0X10,0XA2,0X10,0XA2,0X10,0XDE,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA2,0X10,0XA2,0X10,0X82,0X10,0X82,0X10,
0XBE,0XF7,0XDE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA2,0X10,0XA2,0X10,
0XA2,0X10,0X82,0X10,0XFF,0XFF,0XFF,0XFF,0XDE,0XF7,0XDE,0XF7,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA2,0X10,0XA2,0X10,0X82,0X10,0X82,0X08,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XF7,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA2,0X10,0XA2,0X10,0XC2,0X10,0XC3,0X18,0XC3,0X10,0XC2,0X10,
0XA2,0X10,0XA2,0X10,0XA2,0X10,0XC3,0X10,0XA2,0X10,0XA2,0X10,0XC3,0X10,0XC3,0X10,
0XA2,0X10,0XA2,0X10,0XA2,0X10,0XA2,0X10,0XA2,0X10,0XA2,0X10,0XC3,0X18,0XC3,0X18,
0XC2,0X10,0XA2,0X10,0XA2,0X10,0XA2,0X10,0XC3,0X18,0XC3,0X18,0XA2,0X10,0XA2,0X10,
0XC3,0X10,0XC3,0X10,0XA2,0X10,0XA2,0X10,0XA2,0X10,0XA2,0X10,0XA2,0X10,0XA2,0X10,
};
#line 3 "app\\Font_Num.c"
#line 1 ".\\app\\RES\\Num\\0_small.c"
const unsigned char gImage_0_small[252] = {  
0XC3,0X18,0X82,0X10,0XA2,0X10,0X82,0X08,0X82,0X10,0X61,0X08,0X82,0X08,0XA2,0X10,
0XC2,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XA2,0X10,0XC3,0X18,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XC3,0X10,0XA2,0X10,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC2,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0X61,0X08,0XBE,0XF7,0XFF,0XFF,0XFF,0XFF,0XA2,0X10,0X82,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0X82,0X10,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XA2,0X10,0XA2,0X10,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0X82,0X10,0XBE,0XF7,0XFF,0XFF,0XFF,0XFF,0XA2,0X10,0XA2,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X82,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA2,0X10,
0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X82,0X10,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XA2,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X61,0X08,0XDE,0XFF,0XFF,0XFF,
0XDF,0XFF,0XA2,0X10,0XC2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA2,0X10,0XA2,0X10,0XDE,0XF7,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XA2,0X10,0X82,0X10,0XFF,0XFF,0XBE,0XF7,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA2,0X10,0XC3,0X18,0XC3,0X10,0XA2,0X10,
0XA2,0X10,0XA2,0X10,0XE3,0X18,0XA2,0X10,0XA2,0X10,0XA2,0X10,};
#line 4 "app\\Font_Num.c"
#line 1 ".\\app\\RES\\Num\\1_big.c"
const unsigned char gImage_1_big[1008] = {  
0X1F,0X00,0X1F,0X00,0X1F,0X00,0X3C,0X00,0X76,0X00,0X8E,0X00,0XC8,0X08,0XC3,0X08,
0XC3,0X10,0XC3,0X10,0XA3,0X18,0X83,0X10,0XA3,0X10,0XA2,0X10,0XA3,0X10,0XC3,0X10,
0X04,0X11,0XE4,0X08,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X3C,0X00,0X75,0X00,0X8E,0X00,
0XC7,0X08,0XC3,0X08,0XC3,0X08,0XA3,0X10,0XA3,0X10,0X83,0X10,0XA2,0X10,0XA2,0X10,
0XA2,0X10,0XA3,0X08,0XE4,0X08,0XC4,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X3B,0X00,
0X75,0X00,0X8D,0X00,0XC7,0X00,0XC3,0X00,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0X05,0X11,0X05,0X09,0X1F,0X00,0X1F,0X00,
0X1F,0X00,0X3B,0X00,0X75,0X00,0X6D,0X00,0XC7,0X00,0XC3,0X00,0XFF,0XF7,0XFF,0XF7,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XF7,0XC3,0X08,0XC4,0X00,
0X1F,0X00,0X1F,0X00,0X1F,0X00,0X3C,0X00,0X75,0X00,0X6E,0X00,0XC7,0X00,0XC4,0X00,
0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XE4,0X08,0XE4,0X08,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X3C,0X00,0X76,0X00,0X6E,0X00,
0XC7,0X00,0XE3,0X00,0XFF,0XF7,0XFF,0XF7,0XDF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF7,0XC3,0X00,0XC3,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X3C,0X00,
0X76,0X00,0X6E,0X00,0XC7,0X00,0XE3,0X00,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC3,0X08,0XE4,0X00,0X1F,0X00,0X1F,0X00,
0X1F,0X00,0X3C,0X00,0X76,0X00,0X6E,0X00,0XE7,0X00,0X03,0X01,0XFF,0XEF,0XFF,0XF7,
0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XC3,0X08,0XE4,0X00,
0X1F,0X00,0X1F,0X00,0X1F,0X00,0X3C,0X00,0X75,0X00,0X8E,0X00,0XC6,0X00,0X23,0X01,
0XE3,0X00,0XC3,0X08,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XC3,0X08,0XE3,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X3D,0X00,0X58,0X00,0X72,0X00,
0XAD,0X00,0XE9,0X00,0XC4,0X00,0X04,0X11,0XBF,0XF7,0XFF,0XFF,0XFF,0XFF,0XBE,0XF7,
0XFF,0XFF,0XFF,0XF7,0XC3,0X08,0XE3,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,
0X1D,0X00,0X1B,0X00,0X39,0X00,0X93,0X00,0XC8,0X00,0XC2,0X00,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XDE,0XF7,0XFF,0XFF,0XC3,0X08,0XE3,0X00,0X1F,0X00,0X1F,0X00,
0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X58,0X00,0XAA,0X00,0X23,0X09,
0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,0XDE,0XF7,0XFF,0XFF,0XFF,0XF7,0XC3,0X08,0XE3,0X00,
0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X59,0X00,
0XCC,0X00,0XC4,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XC3,0X08,0XE3,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,
0X1F,0X00,0X59,0X00,0XCC,0X00,0XC4,0X00,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XC3,0X08,0XE3,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,
0X1F,0X00,0X1F,0X00,0X1E,0X00,0X78,0X00,0XCC,0X00,0XE4,0X00,0XFF,0XF7,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XC3,0X08,0XE3,0X00,0X1F,0X00,0X1F,0X00,
0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1E,0X00,0X78,0X00,0XEC,0X00,0XC4,0X00,
0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XC3,0X08,0XE3,0X00,
0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X58,0X00,
0XEC,0X00,0XE4,0X00,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XC3,0X08,0XC4,0X08,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,
0X1F,0X00,0X59,0X00,0XEC,0X00,0XE4,0X00,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XC3,0X08,0XC4,0X08,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,
0X1F,0X00,0X1F,0X00,0X1F,0X00,0X59,0X00,0XEC,0X00,0XE4,0X00,0XFF,0XF7,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC3,0X08,0XE4,0X00,0X1F,0X00,0X1F,0X00,
0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X59,0X00,0XEC,0X00,0XE4,0X00,
0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC3,0X08,0XE4,0X00,
0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X59,0X00,
0XEC,0X00,0XE4,0X00,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XC3,0X08,0XE3,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,
0X1F,0X00,0X59,0X00,0XEC,0X00,0XE4,0X00,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XC3,0X08,0XE3,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,
0X1F,0X00,0X1F,0X00,0X1F,0X00,0X59,0X00,0XEC,0X00,0XE4,0X00,0XFF,0XF7,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC3,0X08,0XE3,0X00,0X1F,0X00,0X1F,0X00,
0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X59,0X00,0XEC,0X00,0XE4,0X00,
0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XC3,0X08,0XE3,0X00,
0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X59,0X00,
0XCB,0X00,0X04,0X09,0XFF,0XF7,0XFF,0XFF,0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,
0XC3,0X08,0XE3,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,
0X1F,0X00,0X59,0X00,0X0C,0X09,0XE4,0X00,0XFF,0XF7,0XFF,0XFF,0XFF,0XF7,0XBE,0XF7,
0XFF,0XFF,0XFF,0XF7,0XE4,0X08,0X04,0X09,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,
0X1F,0X00,0X1F,0X00,0X1F,0X00,0X59,0X00,0XAB,0X00,0X04,0X09,0XC3,0X08,0XC2,0X10,
0XE3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X08,0XC3,0X00,0XC3,0X00,0X1F,0X00,0X1F,0X00,
0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X59,0X00,0X0C,0X09,0XE4,0X00,
0XC3,0X08,0XC2,0X10,0X03,0X19,0X81,0X08,0XC3,0X10,0XE3,0X10,0XE3,0X08,0XE4,0X00,
};
#line 5 "app\\Font_Num.c"
#line 1 ".\\app\\RES\\Num\\1_small.c"
const unsigned char gImage_1_small[252] = {  
0X1F,0X00,0X59,0X00,0XCB,0X08,0XC4,0X08,0XA3,0X10,0X83,0X10,0XA2,0X10,0XA2,0X08,
0XC4,0X08,0X1F,0X00,0X58,0X00,0XAA,0X00,0XE4,0X08,0XFF,0XFF,0XDF,0XFF,0XDE,0XFF,
0XFF,0XF7,0XC3,0X00,0X1F,0X00,0X58,0X00,0XAA,0X00,0XC2,0X00,0XDF,0XF7,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF7,0XC3,0X00,0X1F,0X00,0X59,0X00,0XAC,0X00,0XC4,0X00,0XFF,0XFF,
0XDF,0XFF,0XDF,0XF7,0XFF,0XF7,0XE3,0X08,0X3F,0X00,0X1B,0X00,0X51,0X00,0XAA,0X00,
0XA5,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XE3,0X08,0X1F,0X00,0X3D,0X00,0X57,0X00,
0XB1,0X08,0XC8,0X10,0XFF,0XFF,0XDF,0XF7,0XFF,0XF7,0XE3,0X08,0X1F,0X00,0X3F,0X00,
0X3D,0X00,0X35,0X00,0X69,0X08,0XFF,0XF7,0XFF,0XFF,0XFF,0XF7,0XC3,0X08,0X1F,0X00,
0X1F,0X00,0X1F,0X00,0X59,0X08,0X8A,0X08,0XFF,0XF7,0XFF,0XFF,0XFF,0XF7,0XC4,0X08,
0X1F,0X00,0X1F,0X00,0X1F,0X00,0X38,0X00,0X8A,0X08,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,
0XC3,0X08,0X1F,0X00,0X3F,0X00,0X1F,0X00,0X58,0X00,0X8A,0X00,0XFF,0XF7,0XFF,0XF7,
0XFF,0XFF,0XC3,0X08,0X1F,0X00,0X1F,0X00,0X1F,0X00,0X58,0X00,0X89,0X00,0XFF,0XF7,
0XFF,0XF7,0XFF,0XFF,0XC3,0X08,0X1F,0X00,0X1F,0X00,0X3F,0X00,0X78,0X00,0X89,0X00,
0XFF,0XF7,0XFF,0XFF,0XFF,0XF7,0XC3,0X08,0X1F,0X00,0X1F,0X00,0X1E,0X00,0X78,0X00,
0XAA,0X08,0XFF,0XF7,0XBF,0XEF,0XFF,0XF7,0XE3,0X08,0X1F,0X00,0X1F,0X00,0X1F,0X00,
0X59,0X00,0X89,0X00,0XC3,0X08,0XA3,0X08,0XC3,0X10,0XE3,0X08,};
#line 6 "app\\Font_Num.c"
#line 1 ".\\app\\RES\\Num\\2_big.c"
const unsigned char gImage_2_big[1008] = {  
0XC2,0X08,0XC3,0X10,0XC3,0X10,0XA2,0X10,0XC3,0X10,0XA2,0X10,0XC2,0X10,0XA2,0X10,
0XC3,0X10,0XC2,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,
0XC3,0X10,0XC3,0X10,0XC2,0X08,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XA2,0X10,0XC3,0X10,
0XA2,0X08,0XA2,0X10,0XC2,0X10,0XA2,0X10,0XA2,0X10,0XA2,0X10,0XA3,0X10,0XA2,0X10,
0XA2,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,
0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XA2,0X10,0XC3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,
0XA3,0X10,0XA2,0X10,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XA3,0X10,0XA3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XA2,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XC3,0X10,0XC3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,
0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC2,0X10,0XA2,0X10,
0XC3,0X18,0XE3,0X18,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XA3,0X10,0XA2,0X10,0XC3,0X10,0XA2,0X10,0XA3,0X10,0XA2,0X10,0XC3,0X10,
0XA2,0X10,0XC2,0X10,0X82,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XA3,0X10,0XA3,0X10,0XC3,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XC3,0X10,0XA2,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,
0XC3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XA3,0X10,0XA2,0X10,0XA3,0X10,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XA2,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XC3,0X10,0XA3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,
0XA3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X82,0X10,0XA2,0X10,0XA2,0X10,
0XC3,0X10,0XE3,0X18,0XA2,0X10,0XA2,0X10,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA2,0X10,
0XA2,0X10,0XC3,0X18,0X82,0X10,0XA2,0X10,0X82,0X10,0XE3,0X18,0XA3,0X10,0XC3,0X10,
0XA3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XA3,0X10,0XA3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XC3,0X10,0XA3,0X10,0XA3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC3,0X10,0XC3,0X10,
0XA2,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XA3,0X10,0XA3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XA3,0X10,0XE3,0X10,0XC3,0X10,0XE3,0X18,
0XE3,0X18,0XE3,0X18,0XA3,0X10,0XE3,0X18,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,
0XC3,0X18,0XC3,0X18,0XC3,0X18,0XC3,0X10,0XA3,0X10,0XA3,0X10,0XC3,0X10,0XA3,0X10,
0XA2,0X10,0XA2,0X10,0XA2,0X10,0XA2,0X10,0XA3,0X10,0XA2,0X10,0XA3,0X10,0XA3,0X10,
0XA2,0X10,0XA3,0X10,0XA3,0X10,0XA3,0X10,0XA3,0X10,0XA3,0X10,0XA3,0X10,0XA3,0X10,
};
#line 7 "app\\Font_Num.c"
#line 1 ".\\app\\RES\\Num\\2_small.c"
const unsigned char gImage_2_small[252] = {  
0XC3,0X18,0XA3,0X10,0XC3,0X18,0XA2,0X10,0XC3,0X18,0XA2,0X10,0X82,0X10,0XC3,0X10,
0XA3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XA3,0X10,0X82,0X10,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X18,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XA3,0X10,0XC3,0X18,0XC3,0X18,
0XA3,0X10,0XFF,0XFF,0XDF,0XFF,0XDF,0XF7,0XA3,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XBF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XFF,0XFF,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XBF,0XF7,0XFF,0XFF,0XA3,0X10,0XC3,0X10,
0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,
0XA3,0X10,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,
0XA3,0X10,0X82,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA2,0X10,0XA2,0X10,0XA2,0X10,
0X04,0X19,0XA3,0X10,0XC3,0X18,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0X82,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XDF,0XF7,0XA3,0X10,0XA3,0X10,0XFF,0XFF,0XBF,0XF7,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0X82,0X10,0XC3,0X10,
0XA3,0X10,0X82,0X10,0XA3,0X10,0XC3,0X18,0X82,0X08,0XA3,0X10,};
#line 8 "app\\Font_Num.c"
#line 1 ".\\app\\RES\\Num\\3_big.c"
const unsigned char gImage_3_big[1008] = {  
0XC2,0X08,0XC3,0X10,0XC3,0X10,0XA2,0X10,0XC3,0X10,0XA2,0X10,0XC2,0X10,0XA2,0X10,
0XC3,0X10,0XC2,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,
0XC3,0X10,0XC3,0X10,0XC2,0X08,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XA2,0X10,0XC3,0X10,
0XA2,0X08,0XA2,0X10,0XC2,0X10,0XA2,0X10,0XA2,0X10,0XA2,0X10,0XA3,0X10,0XA2,0X10,
0XA2,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,
0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XA2,0X10,0XC3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,
0XA3,0X10,0XA2,0X10,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XA3,0X10,0XA3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XA2,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XC3,0X10,0XC3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,
0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC2,0X10,0XA2,0X10,
0XC3,0X18,0XE3,0X18,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XA3,0X10,0XA2,0X10,0XC3,0X10,0XA2,0X10,0XA3,0X10,0XA2,0X10,0XC3,0X10,
0XA2,0X10,0XC2,0X10,0X82,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XA3,0X10,0XA3,0X10,0XC3,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XC3,0X10,0XA2,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,
0XC3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XA3,0X10,0XA2,0X10,0XA3,0X10,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XA2,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XC3,0X18,0XA3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,
0XA2,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XC3,0X10,0XC3,0X10,0XA2,0X10,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA2,0X08,0XC3,0X10,0XA2,0X10,0XA2,0X10,
0XC3,0X10,0XC2,0X10,0XA2,0X10,0XC2,0X10,0XC3,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XC3,0X10,0XC3,0X10,
0XC3,0X10,0XC2,0X10,0XA2,0X10,0X81,0X08,0XA2,0X10,0XA2,0X10,0X82,0X08,0XA2,0X10,
0XDE,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,
0XC3,0X10,0XC3,0X10,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XA3,0X10,0XC3,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XA2,0X10,0XC3,0X18,0XDF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XA3,0X10,0XA2,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XA3,0X10,0XA3,0X10,
0XA2,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XDF,0XFF,0XDF,0XFF,0XDF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XA3,0X10,0XE3,0X10,0XC3,0X10,0XE3,0X18,
0XE3,0X18,0X04,0X19,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,
0XC3,0X10,0XE3,0X10,0XC3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,0XC3,0X10,0XA3,0X08,
0XA2,0X08,0XC2,0X08,0XA2,0X08,0XA2,0X08,0XC3,0X10,0XA2,0X08,0XC3,0X10,0XA2,0X10,
0XC2,0X08,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,
};
#line 9 "app\\Font_Num.c"
#line 1 ".\\app\\RES\\Num\\3_small.c"
const unsigned char gImage_3_small[252] = {  
0XC3,0X10,0XC3,0X10,0XC3,0X10,0XA2,0X10,0XC3,0X10,0XA2,0X10,0X82,0X08,0XC3,0X10,
0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XC3,0X10,0XA2,0X10,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC3,0X10,0XA3,0X10,0XC3,0X10,0XC3,0X18,0XC3,0X18,
0XA3,0X10,0XFF,0XFF,0XDF,0XFF,0XDF,0XF7,0XC3,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XBF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XFF,0XFF,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XBF,0XF7,0XFF,0XFF,0XA3,0X10,0XC3,0X10,
0XDF,0XF7,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,
0X82,0X08,0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XC3,0X10,0XA2,0X10,0X82,0X08,0X82,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XDF,0XFF,0XFF,0XFF,0XA3,0X10,0X82,0X08,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XBE,0XF7,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC3,0X10,0XA2,0X10,0XA2,0X10,0XA3,0X10,
0X61,0X08,0XA3,0X10,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,};
#line 10 "app\\Font_Num.c"
#line 1 ".\\app\\RES\\Num\\4_big.c"
const unsigned char gImage_4_big[1008] = {  
0XC3,0X10,0XA2,0X08,0XC3,0X08,0XA2,0X08,0XA3,0X08,0XC3,0X08,0XE4,0X18,0XC4,0X18,
0XA3,0X10,0XA3,0X10,0X82,0X10,0X62,0X10,0XC3,0X18,0XC3,0X10,0XA2,0X10,0XC3,0X10,
0XC3,0X10,0XC3,0X10,0XC2,0X08,0XC3,0X08,0XC3,0X08,0XA3,0X08,0XC3,0X08,0XC3,0X10,
0XC3,0X10,0XE4,0X18,0X83,0X10,0XA3,0X10,0X83,0X10,0X82,0X10,0XA3,0X18,0XC3,0X18,
0XA2,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XA2,0X10,0XA2,0X08,0XFF,0XFF,0XFF,0XF7,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X82,0X10,0X82,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC3,0X10,0XC3,0X10,0X82,0X08,0XA2,0X10,
0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X62,0X10,0X82,0X10,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC3,0X10,0XC3,0X10,
0XE3,0X18,0XC2,0X10,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XE4,0X20,0XC3,0X18,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XC3,0X10,0XC3,0X10,0XC3,0X18,0XC3,0X18,0XFF,0XFF,0XFF,0XFF,0XBE,0XF7,0XDE,0XF7,
0XFF,0XFF,0XFF,0XFF,0XC3,0X18,0XC3,0X18,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XC3,0X10,0XC3,0X10,0XA2,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XE3,0X18,0XC3,0X18,0XFF,0XFF,0XDF,0XFF,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC3,0X10,0XC3,0X10,0X81,0X10,0XC2,0X18,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XC3,0X18,0XC3,0X18,
0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC3,0X10,0XC3,0X10,
0XC2,0X18,0XC2,0X18,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0X81,0X08,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XC3,0X10,0XC3,0X10,0XC2,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0X81,0X08,0X81,0X08,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XDF,0XF7,0XC3,0X10,0XC3,0X10,0XA2,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XDE,0XF7,0XA2,0X10,0XC2,0X10,0XFF,0XF7,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC3,0X10,0XC3,0X10,0XA2,0X10,0XA2,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDE,0XFF,0XFE,0XF7,0XDE,0XF7,0XC2,0X10,0XA2,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDE,0XF7,0XC3,0X10,0XC3,0X10,
0XA2,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XC2,0X10,0XA2,0X08,0XFF,0XF7,0XDE,0XF7,0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XC3,0X10,0XC3,0X10,0XA2,0X10,0XC2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XE3,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XA3,0X10,0XC3,0X10,0XA2,0X10,0XC2,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC2,0X10,0XC3,0X10,
0X62,0X10,0X62,0X10,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XDE,0XFF,0XDE,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XC2,0X10,0XC2,0X08,0X62,0X08,0X62,0X08,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XBE,0XF7,0XBE,0XFF,0XDF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XC2,0X10,0XC2,0X08,0XA3,0X08,0X82,0X08,0XFF,0XFF,0XDF,0XF7,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XF7,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XBF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC2,0X10,0XC2,0X10,0X83,0X00,0XA3,0X08,
0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XEF,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XDF,0XFF,0XBF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XC2,0X10,0XC2,0X10,
0X05,0X09,0XC4,0X00,0XE4,0X08,0XE4,0X08,0X04,0X01,0X25,0X09,0X05,0X01,0XE4,0X08,
0XC3,0X10,0XC3,0X18,0XBF,0XFF,0XDF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XC2,0X10,0XC2,0X10,0X26,0X01,0X25,0X01,0X25,0X01,0X45,0X01,0X46,0X01,0X45,0X01,
0X46,0X01,0X25,0X01,0X05,0X09,0XC3,0X10,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA2,0X10,0XC2,0X10,0XF8,0X5D,0XF8,0X5D,0XB7,0X5D,0XD7,0X5D,
0XD7,0X5D,0XD7,0X5D,0XD7,0X65,0XB6,0X6D,0XE3,0X00,0XC3,0X00,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA2,0X10,0XA2,0X10,0X18,0X4E,0X18,0X4E,
0X18,0X56,0X18,0X56,0X18,0X4E,0X18,0X4E,0X17,0X56,0XD7,0X65,0X04,0X01,0XE3,0X00,
0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC2,0X10,0XA2,0X10,
0X19,0X4E,0XF9,0X4D,0XF9,0X55,0XF8,0X55,0XF9,0X55,0X18,0X4E,0XF8,0X55,0XD7,0X65,
0X04,0X01,0XC3,0X00,0XFF,0XF7,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,
0XC2,0X10,0XA2,0X10,0XF9,0X55,0XF9,0X55,0XF9,0X55,0XF9,0X55,0XF9,0X55,0XF9,0X55,
0XF8,0X55,0XD7,0X65,0X25,0X01,0XC4,0X00,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,
0XFF,0XFF,0XFF,0XFF,0XA2,0X10,0XA2,0X10,0XF8,0X55,0XF8,0X55,0XF8,0X55,0XF8,0X55,
0XF9,0X55,0XF9,0X55,0XF8,0X55,0XD7,0X65,0X05,0X01,0X05,0X09,0XC4,0X10,0XC3,0X10,
0XE4,0X10,0XC3,0X10,0X82,0X08,0XA2,0X10,0XA2,0X10,0XA2,0X10,0XF8,0X55,0XF8,0X55,
0XF8,0X55,0XF8,0X55,0XF9,0X55,0XF9,0X55,0XF8,0X55,0XD7,0X65,0X25,0X01,0X05,0X09,
0XC4,0X10,0XA3,0X10,0XC4,0X18,0XC3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,0XA2,0X10,
};
#line 11 "app\\Font_Num.c"
#line 1 ".\\app\\RES\\Num\\4_small.c"
const unsigned char gImage_4_small[252] = {  
0XC2,0X08,0XA2,0X08,0XA3,0X10,0XC3,0X18,0X83,0X10,0X82,0X10,0XA3,0X18,0XA3,0X10,
0XC3,0X10,0XA2,0X08,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X82,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XC3,0X10,0XC3,0X18,0XFF,0XFF,0X9E,0XF7,0XDF,0XFF,0XC3,0X18,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XC3,0X10,0XA2,0X10,0XDF,0XFF,0XFF,0XFF,0XDF,0XFF,0XC3,0X18,
0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XC3,0X10,0XC2,0X10,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,
0X82,0X08,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XC3,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XF7,
0XDE,0XF7,0XA2,0X10,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XC3,0X10,0XA2,0X10,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XC2,0X10,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XC3,0X10,0XA3,0X10,
0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC3,0X10,
0X63,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,
0XC3,0X08,0XA3,0X00,0XFF,0XEF,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XC3,0X08,0X25,0X01,0X25,0X01,0X45,0X01,0X24,0X01,0XE4,0X08,0XFF,0XF7,
0XFF,0XFF,0XFF,0XFF,0XC2,0X10,0XB7,0X65,0XB6,0X65,0XD6,0X6D,0X95,0X75,0XE3,0X00,
0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XC2,0X10,0X18,0X4E,0X38,0X56,0XD6,0X4D,0XF6,0X65,
0X04,0X01,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XA2,0X10,0X38,0X46,0X58,0X46,0X58,0X4E,
0XD6,0X5D,0X45,0X01,0XE3,0X08,0XC3,0X10,0XA2,0X10,0XA2,0X10,};
#line 12 "app\\Font_Num.c"
#line 1 ".\\app\\RES\\Num\\5_big.c"
const unsigned char gImage_5_big[1008] = {  
0XC3,0X08,0XC3,0X08,0XC3,0X10,0XA2,0X08,0XC3,0X10,0XA2,0X10,0XA2,0X10,0XA2,0X10,
0XC2,0X10,0XC2,0X10,0XC2,0X10,0XC2,0X10,0XC3,0X10,0XC3,0X10,0XA3,0X10,0XC3,0X10,
0XA3,0X10,0XC3,0X10,0XC3,0X08,0XC3,0X08,0XC2,0X08,0XA2,0X10,0XA3,0X10,0XC3,0X10,
0XA2,0X10,0XC3,0X10,0XA2,0X10,0XC2,0X10,0XC2,0X10,0XC2,0X10,0XC3,0X10,0XC3,0X10,
0XA3,0X10,0XA3,0X10,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XA2,0X08,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA2,0X08,0XA3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,
0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XC3,0X10,0X82,0X08,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA2,0X10,0XC3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,
0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XC3,0X18,0XC3,0X18,0XC3,0X18,0XC3,0X10,0XC3,0X10,0XC3,0X18,0XC3,0X18,
0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA2,0X10,0XA2,0X10,0XA2,0X10,0XC3,0X10,0XE3,0X18,
0X82,0X10,0XA3,0X10,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,
0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,
0XC3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA3,0X08,0XE3,0X10,0XC2,0X08,0XA2,0X10,
0XA2,0X10,0XA2,0X10,0XA2,0X10,0XC3,0X10,0XC2,0X10,0X82,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XA2,0X08,
0XC2,0X08,0XA2,0X08,0XC3,0X10,0XC3,0X10,0XA3,0X10,0XA2,0X10,0XA2,0X10,0XE3,0X18,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,
0XA2,0X08,0XE3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XC3,0X10,0XC3,0X10,0XA2,0X08,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA2,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,
0XA2,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA2,0X08,0XE3,0X10,0XA2,0X10,0XC3,0X10,
0XE3,0X10,0XE3,0X10,0XA2,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,
0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XE3,0X10,0XE3,0X10,
0XC2,0X08,0XA2,0X08,0XC2,0X08,0XC2,0X08,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,
0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,
};
#line 13 "app\\Font_Num.c"
#line 1 ".\\app\\RES\\Num\\5_small.c"
const unsigned char gImage_5_small[252] = {  
0XC3,0X10,0XA2,0X08,0XA2,0X10,0XE4,0X18,0X82,0X10,0XA2,0X10,0XA3,0X10,0XC3,0X10,
0XC3,0X10,0XA2,0X08,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XC3,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XDF,0XF7,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA2,0X10,0X62,0X08,0XE3,0X18,0XC3,0X10,0XA3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XF7,0XFF,0XFF,0XA3,0X10,0XA2,0X10,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,
0XA3,0X10,0XDF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XA2,0X10,0X82,0X10,0XA3,0X10,0XA3,0X10,0XC3,0X18,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XA3,0X10,0XA2,0X10,0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XF7,0XA3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XE3,0X10,0XA2,0X08,0XA3,0X10,
0X82,0X08,0XC3,0X18,0XC3,0X18,0XA2,0X10,0XC3,0X10,0XA3,0X10,};
#line 14 "app\\Font_Num.c"
#line 1 ".\\app\\RES\\Num\\6_big.c"
const unsigned char gImage_6_big[1008] = {  
0XC3,0X08,0XC3,0X08,0XC3,0X10,0XA2,0X08,0XC3,0X10,0XA2,0X10,0XA2,0X10,0XA2,0X10,
0XC2,0X10,0XC2,0X10,0XC2,0X10,0XC2,0X10,0XC3,0X10,0XC3,0X10,0XA3,0X10,0XC3,0X10,
0XA3,0X10,0XC3,0X10,0XC3,0X08,0XC3,0X08,0XC2,0X08,0XA2,0X10,0XA3,0X10,0XC3,0X10,
0XA2,0X10,0XC3,0X10,0XA2,0X10,0XC2,0X10,0XC2,0X10,0XC2,0X10,0XC3,0X10,0XC3,0X10,
0XA3,0X10,0XA3,0X10,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XA2,0X08,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA2,0X08,0XA3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,
0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XC3,0X10,0X82,0X08,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA2,0X10,0XC3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,
0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XC3,0X18,0XC3,0X18,0XC3,0X18,0XC3,0X10,0XC3,0X10,0XC3,0X18,0XC3,0X18,
0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA2,0X10,0XA2,0X10,0XA2,0X10,0XC3,0X10,0XE3,0X18,
0X82,0X10,0XA3,0X10,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,
0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,
0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA2,0X18,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA2,0X10,
0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,
0XA3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,
0XA2,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0X82,0X08,0XC3,0X10,0XA2,0X10,0XC3,0X10,
0XE3,0X10,0XE3,0X10,0XA2,0X08,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,
0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,
0XA2,0X10,0XA2,0X10,0XA2,0X10,0XA2,0X10,0XC3,0X10,0XC3,0X10,0XC2,0X10,0XC2,0X10,
0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,
};
#line 15 "app\\Font_Num.c"
#line 1 ".\\app\\RES\\Num\\6_small.c"
const unsigned char gImage_6_small[252] = {  
0XC2,0X10,0XA2,0X08,0XA2,0X10,0XE3,0X18,0X82,0X10,0XA2,0X10,0XA3,0X10,0XC3,0X10,
0XC3,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XC3,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC3,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA2,0X10,0X62,0X08,0XE3,0X18,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XF7,0XFF,0XFF,0XA3,0X10,0XA2,0X10,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA2,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,
0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X82,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XA3,0X10,0XC2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC2,0X10,0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC3,0X10,0XC3,0X10,0X82,0X08,0XA2,0X10,
0XA2,0X10,0XC3,0X10,0X82,0X10,0XA3,0X10,0XE3,0X10,0XC3,0X10,};
#line 16 "app\\Font_Num.c"
#line 1 ".\\app\\RES\\Num\\7_big.c"
const unsigned char gImage_7_big[1008] = {  
0XC3,0X08,0XC3,0X10,0XA3,0X08,0X62,0X08,0XC3,0X18,0XA2,0X18,0XA2,0X18,0XC3,0X18,
0XA3,0X10,0XA3,0X10,0XA3,0X10,0XA3,0X10,0XC3,0X10,0XC2,0X10,0XA2,0X10,0XA2,0X10,
0XC2,0X10,0XC2,0X10,0XA3,0X08,0XA3,0X10,0X82,0X08,0X82,0X08,0XA2,0X10,0XC3,0X18,
0XA2,0X10,0XC3,0X18,0XA2,0X10,0XA3,0X10,0XA3,0X10,0XA3,0X10,0XC3,0X10,0XC2,0X10,
0XA2,0X10,0XA2,0X10,0XC2,0X10,0XC2,0X10,0XC3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC2,0X10,0XC2,0X10,0XA2,0X10,0XA3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC2,0X10,0XC2,0X10,
0XE3,0X18,0XC3,0X18,0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA2,0X10,0XA3,0X10,0XA2,0X10,0XC2,0X18,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XE3,0X18,0XC2,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XC2,0X10,0XC2,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,
0XA2,0X08,0XC2,0X08,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0X82,0X08,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XA3,0X10,0XA2,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,0XFF,0XF7,
0XFF,0XF7,0XFF,0XF7,0XA2,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XA2,0X10,0XC3,0X18,0XDF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XF7,0XDF,0XF7,0XFF,0XFF,0XC3,0X18,0XC2,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XC3,0X18,0XA3,0X18,
0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XDF,0XF7,0XDF,0XFF,0XDF,0XFF,0XC3,0X18,0XC3,0X18,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,
0X62,0X08,0X62,0X08,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XA2,0X18,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XA3,0X10,0XC4,0X08,0X83,0X08,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XDF,0XEF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XE4,0X00,0XE4,0X00,0XE4,0X00,0XE4,0X00,
0X04,0X01,0X04,0X01,0XC4,0X00,0XC3,0X08,0XA3,0X08,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0X45,0X01,0X45,0X01,
0X45,0X01,0X25,0X01,0X46,0X01,0X25,0X01,0X25,0X01,0XE4,0X00,0XC3,0X00,0XA2,0X08,
0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,
0XD8,0X55,0XD8,0X55,0XD8,0X5D,0XD8,0X5D,0XD8,0X55,0XD8,0X5D,0XD8,0X5D,0X97,0X6D,
0X04,0X01,0XC3,0X00,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XA3,0X10,0XA3,0X10,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X19,0X46,
0XF9,0X4D,0XD8,0X5D,0XE5,0X00,0XC3,0X00,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XA3,0X10,0XA3,0X10,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0X19,0X46,0XF9,0X4D,0XD8,0X5D,0X25,0X01,0XE4,0X00,0XFF,0XF7,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XF9,0X45,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X19,0X46,0XF9,0X4D,0XD8,0X5D,0X26,0X01,0X04,0X09,
0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,
0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X19,0X46,0XF9,0X4D,0XD8,0X5D,
0XE5,0X00,0XC3,0X00,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XA3,0X10,0XA3,0X10,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X19,0X46,
0XF9,0X4D,0XD8,0X5D,0X05,0X01,0XC3,0X00,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XA3,0X10,0XA3,0X10,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0X19,0X46,0XF9,0X4D,0XD8,0X5D,0X26,0X01,0XE4,0X00,0XFF,0XF7,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,0XF9,0X45,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X19,0X46,0XF9,0X4D,0XD8,0X5D,0X05,0X01,0XE4,0X00,
0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA3,0X10,
0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X19,0X46,0XF9,0X4D,0XD8,0X5D,
0X05,0X01,0XC3,0X00,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XA3,0X10,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X19,0X46,
0XF9,0X4D,0XD8,0X5D,0X05,0X01,0XC3,0X00,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XA3,0X10,0XA3,0X10,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0X19,0X46,0XF9,0X4D,0XD8,0X5D,0X05,0X01,0X04,0X09,0XA2,0X08,0XA2,0X08,
0XA2,0X10,0XC2,0X18,0XA2,0X10,0XE3,0X18,0XA3,0X10,0XA3,0X10,0XF9,0X45,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X19,0X46,0XF9,0X4D,0XD8,0X5D,0X25,0X01,0XE4,0X00,
0XC3,0X08,0XA2,0X10,0XC2,0X10,0XA2,0X10,0XA2,0X10,0XA3,0X10,0XA3,0X10,0XA3,0X10,
};
#line 17 "app\\Font_Num.c"
#line 1 ".\\app\\RES\\Num\\7_small.c"
const unsigned char gImage_7_small[252] = {  
0XA3,0X10,0X82,0X08,0XA2,0X10,0XC3,0X18,0XA3,0X18,0XA3,0X18,0XA2,0X18,0XA2,0X10,
0XC2,0X10,0X83,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XC2,0X10,0XC3,0X18,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC2,0X18,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XA3,0X10,0XA2,0X10,0XFE,0XF7,0XFF,0XF7,0XFF,0XF7,
0X82,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XEF,
0XFF,0XF7,0XC3,0X10,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0X83,0X08,0XFF,0XFF,
0XFF,0XEF,0XFF,0XF7,0XA3,0X10,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XA3,0X10,0X05,0X01,
0X05,0X01,0X25,0X01,0XE4,0X00,0XA3,0X08,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC3,0X10,
0XB7,0X5D,0XB7,0X5D,0XD8,0X65,0X96,0X6D,0XE3,0X00,0XFF,0XF7,0XFF,0XFF,0XDF,0XF7,
0XA3,0X10,0X19,0X4E,0X19,0X4E,0XD8,0X4D,0XB7,0X5D,0X24,0X01,0XFF,0XF7,0XDE,0XF7,
0XFF,0XFF,0XC3,0X10,0XF9,0X45,0XD9,0X45,0XFA,0X4D,0X98,0X5D,0XE4,0X00,0XFF,0XF7,
0XFF,0XFF,0XFF,0XFF,0XC2,0X10,0X1A,0X4E,0XFA,0X45,0X1B,0X4E,0XB9,0X5D,0X05,0X01,
0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XC2,0X10,0XFA,0X45,0XFA,0X4D,0XDA,0X45,0XD9,0X5D,
0XE4,0X00,0XFF,0XF7,0XFF,0XFF,0XDE,0XF7,0XC3,0X10,0XF9,0X45,0X1A,0X4E,0XDA,0X45,
0XD9,0X5D,0XE4,0X00,0XA2,0X08,0XC2,0X10,0XC3,0X10,0XA3,0X10,};
#line 18 "app\\Font_Num.c"
#line 1 ".\\app\\RES\\Num\\8_big.c"
const unsigned char gImage_8_big[1008] = {  
0XC3,0X08,0XC3,0X08,0XC3,0X10,0XA3,0X08,0XC3,0X10,0XA3,0X10,0XA3,0X10,0XA2,0X10,
0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,0XA3,0X10,0XA3,0X10,
0XA3,0X10,0XC3,0X10,0XC3,0X08,0XC3,0X10,0XA3,0X08,0XA3,0X10,0XA3,0X10,0XC3,0X10,
0XA3,0X10,0XC3,0X10,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,
0XA3,0X10,0XA3,0X10,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XA2,0X08,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA2,0X08,0XA3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,
0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XC3,0X10,0X82,0X08,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA2,0X10,0XC3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,
0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA2,0X10,0XC2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA2,0X10,0XC2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XF7,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,
0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XC3,0X10,0XC3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,
0XC3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA2,0X10,0X82,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA2,0X10,0XC3,0X18,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,
0XA3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,
0XA3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XC3,0X10,0XA3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA2,0X08,0XC3,0X10,0XA2,0X10,0XC3,0X10,
0XC3,0X10,0XE3,0X10,0XA2,0X10,0XE3,0X18,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,
0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,
0XC3,0X10,0XA2,0X10,0XC3,0X10,0XC3,0X10,0XA3,0X10,0XC3,0X10,0XA2,0X10,0XA2,0X10,
0XA3,0X10,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,
};
#line 19 "app\\Font_Num.c"
#line 1 ".\\app\\RES\\Num\\8_small.c"
const unsigned char gImage_8_small[252] = {  
0XC3,0X10,0XA2,0X08,0XC3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,0XA2,0X10,0XC3,0X10,
0XC3,0X10,0XA2,0X08,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XC3,0X10,0XE3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XF7,0XFF,0XFF,0XA3,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XA3,0X10,0XC3,0X18,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA2,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,
0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC3,0X18,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XA3,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XE3,0X10,0XA2,0X10,0XA3,0X10,
0XA2,0X10,0XA3,0X10,0XA2,0X10,0XC3,0X18,0XC3,0X10,0XA3,0X10,};
#line 20 "app\\Font_Num.c"
#line 1 ".\\app\\RES\\Num\\9_big.c"
const unsigned char gImage_9_big[1008] = {  
0XC3,0X08,0XC3,0X08,0XC3,0X10,0XA3,0X08,0XC3,0X10,0XA3,0X10,0XA3,0X10,0XA2,0X10,
0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,0XA3,0X10,0XA3,0X10,
0XA3,0X10,0XC3,0X10,0XC3,0X08,0XC3,0X10,0XA3,0X08,0XA3,0X10,0XA3,0X10,0XC3,0X10,
0XA3,0X10,0XC3,0X10,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,
0XA3,0X10,0XA3,0X10,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XA2,0X08,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA2,0X08,0XA3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,
0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XC3,0X10,0X82,0X08,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA2,0X10,0XC3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,
0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA2,0X10,0XC2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA2,0X10,0XC2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XF7,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,
0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XC3,0X10,0XC3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,
0XC3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA3,0X08,0XE3,0X10,0XC2,0X08,0XA2,0X10,
0XA2,0X10,0XA2,0X10,0XA2,0X10,0XC3,0X10,0XC2,0X10,0X82,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XC3,0X10,0XA2,0X08,
0XC2,0X08,0XA2,0X08,0XC3,0X10,0XC3,0X10,0XA3,0X10,0XA2,0X10,0XA2,0X10,0XE3,0X18,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,
0XA2,0X08,0XE3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XC3,0X10,0XC3,0X10,0XA2,0X08,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA2,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA3,0X10,0XA3,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,
0XA3,0X10,0XA3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XC3,0X10,0XA3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XA2,0X08,0XE3,0X10,0XA2,0X10,0XC3,0X10,
0XC3,0X10,0XE3,0X10,0XA2,0X10,0XE3,0X18,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,
0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XE3,0X10,0XE3,0X10,
0XC3,0X10,0XC2,0X08,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,
0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,0XC3,0X10,
};
#line 21 "app\\Font_Num.c"
#line 1 ".\\app\\RES\\Num\\9_small.c"
const unsigned char gImage_9_small[252] = {  
0XC3,0X10,0XA2,0X08,0XC3,0X10,0XC3,0X10,0XA3,0X10,0XC3,0X10,0XA2,0X10,0XC3,0X10,
0XA3,0X10,0XA2,0X08,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XA3,0X10,0XE3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XF7,0XFF,0XFF,0XA3,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XA2,0X10,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0XC3,0X18,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,0X82,0X10,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA3,0X10,
0XA2,0X10,0XDF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X10,0X82,0X10,0X82,0X10,0XA3,0X10,0XA3,0X10,0XC3,0X18,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XA3,0X10,0XA2,0X10,0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XF7,0XA3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC3,0X10,0XC3,0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC3,0X10,0XC3,0X10,0XA2,0X10,0XC3,0X10,
0X82,0X08,0XC3,0X18,0XC3,0X18,0XA2,0X10,0XC3,0X10,0XC3,0X10,};
#line 22 "app\\Font_Num.c"

void Font_Num_DrawNum_Multi( u16 x, u16 y, u32 num, u8 mode )
{
	u16 bit_Num,i;
	u32 num_t;
	u8 oneBit_Num;
	num_t = num;
	
	bit_Num = 0;
	while(num_t)
	{
		num_t /= 10;
		bit_Num++;
	}
	if( 0 == num)
	{
		bit_Num = 1;
	}
	
	if( 0 == mode)
	{
		FlappyBird_Frame_Fill(x + bit_Num*10-(bit_Num-1)*20,y,x + bit_Num*10,y+30,0x4df9);
		num_t = num;
		for( i = 0; i < bit_Num; i++ )
		{
			oneBit_Num = num_t%10;
			Font_Num_DrawNum(x + bit_Num*10-i*20,y,oneBit_Num,0);
			num_t /= 10;
		}
		
	}
	else
	{
		num_t = num;
		for( i = 0; i < bit_Num; i++ )
		{
			oneBit_Num = num_t%10;
			Font_Num_DrawNum(x-i*11,y,oneBit_Num,1);
			num_t /= 10;
		}
	}
	
}

void Font_Num_DrawNum( u16 x, u16 y, u16 num, u8 mode )
{

		if(0 == mode)
		{
			switch(num)
			{
				case 0:
					Font_DrawNum_Big(x,y,(u8 *)gImage_0_big);
					break;
				case 1:
					Font_DrawNum_Big(x,y,(u8 *)gImage_1_big);
					break;
				case 2:
					Font_DrawNum_Big(x,y,(u8 *)gImage_2_big);
					break;
				case 3:
					Font_DrawNum_Big(x,y,(u8 *)gImage_3_big);
					break;
				case 4:
					Font_DrawNum_Big(x,y,(u8 *)gImage_4_big);
					break;
				case 5:
					Font_DrawNum_Big(x,y,(u8 *)gImage_5_big);
					break;
				case 6:
					Font_DrawNum_Big(x,y,(u8 *)gImage_6_big);
					break;
				case 7:
					Font_DrawNum_Big(x,y,(u8 *)gImage_7_big);
					break;
				case 8:
					Font_DrawNum_Big(x,y,(u8 *)gImage_8_big);
					break;
				case 9:
					Font_DrawNum_Big(x,y,(u8 *)gImage_9_big);
					break;
			}
		}
		else
		{
			switch(num)
			{
				case 0:
					Font_DrawNum_Small(x,y,(u8 *)gImage_0_small);
					break;
				case 1:
					Font_DrawNum_Small(x,y,(u8 *)gImage_1_small);
					break;
				case 2:
					Font_DrawNum_Small(x,y,(u8 *)gImage_2_small);
					break;
				case 3:
					Font_DrawNum_Small(x,y,(u8 *)gImage_3_small);
					break;
				case 4:
					Font_DrawNum_Small(x,y,(u8 *)gImage_4_small);
					break;
				case 5:
					Font_DrawNum_Small(x,y,(u8 *)gImage_5_small);
					break;
				case 6:
					Font_DrawNum_Small(x,y,(u8 *)gImage_6_small);
					break;
				case 7:
					Font_DrawNum_Small(x,y,(u8 *)gImage_7_small);
					break;
				case 8:
					Font_DrawNum_Small(x,y,(u8 *)gImage_8_small);
					break;
				case 9:
					Font_DrawNum_Small(x,y,(u8 *)gImage_9_small);
					break;
			}
		}
}
		
void Font_DrawNum_Big( u16 x, u16 y, u8 * num )
{
	u16 color,xx = x,yy = y;
	int R,G,B;
	
	
	for( yy = y; yy < 28 + y; yy++ )
	{
		for( xx = x; xx < 18 + x; xx++ )
		{
			color = num[((yy-y)*18+(xx-x))*2+1];
			color <<= 8;
			color = color + num[((yy-y)*18+(xx-x))*2];
			R = color & 0xf800;
			R >>= 11;
			G = color & 0x07e0;
			G >>= 5;
			B = color & 0x001f;
			if((B - R) > 7)
			{
				FlappyBird_Frame_DrawPoint(xx,yy,0x4df9);
			}
			else
				FlappyBird_Frame_DrawPoint(xx,yy,color);
		}
	}
}

void Font_DrawNum_Black( u16 x, u16 y, u8 * num )
{
	u16 color,xx = x,yy = y;
	int R,G,B;
	for( yy = y; yy < 28 + y; yy++ )
	{
		for( xx = x; xx < 18 + x; xx++ )
		{
			color = num[((yy-y)*18+(xx-x))*2+1];
			color <<= 8;
			color = color + num[((yy-y)*18+(xx-x))*2];
			R = color & 0xf800;
			R >>= 11;
			G = color & 0x07e0;
			G >>= 5;
			B = color & 0x001f;
			if((B - R) > 7)
			{
				;
			}
			else
				FlappyBird_Frame_DrawPoint(xx,yy,0x0000);
		}
	}

}

void Font_DrawNum_Small( u16 x, u16 y, u8 * num )
{
	u16 color,xx = x,yy = y;
	int R,G,B;

	for( yy = y; yy < 14 + y; yy++ )
	{
		for( xx = x; xx < 9 + x; xx++ )
		{
			color = num[((yy-y)*9+(xx-x))*2+1];
			color <<= 8;
			color = color + num[((yy-y)*9+(xx-x))*2];
			R = color & 0xf800;
			R >>= 11;
			G = color & 0x07e0;
			G >>= 5;
			B = color & 0x001f;
			if((B - R) > 7)
			{
				FlappyBird_Frame_DrawPoint(xx,yy,0xdeb2);
			}
			else
				FlappyBird_Frame_DrawPoint(xx,yy,color);
		}
	}
}
