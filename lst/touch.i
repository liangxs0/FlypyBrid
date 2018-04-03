#line 1 "devices\\touch.c"







 

#line 1 "devices\\touch.h"







 




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

#line 14 "devices\\touch.h"
#line 1 "devices\\jpstm32_lcd.h"













 









































 

#line 1 ".\\core\\jpstm32_gpio.h"







 




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




#line 59 "devices\\jpstm32_lcd.h"
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


 

#line 60 "devices\\jpstm32_lcd.h"





















































 
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


#line 153 "devices\\jpstm32_lcd.h"



 

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













#line 15 "devices\\touch.h"












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












#line 11 "devices\\touch.c"
#line 12 "devices\\touch.c"
#line 1 ".\\core\\jpstm32_delay.h"







 



#line 13 ".\\core\\jpstm32_delay.h"

extern void _delay_init(u8 SYSCLK);
extern void _delay_ms(u16 ms);
extern void _delay_us(u32 us);

#line 13 "devices\\touch.c"
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



 

#line 14 "devices\\touch.c"
#line 1 "D:\\Program Files\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"




 





 












 








 






#line 48 "D:\\Program Files\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"

#line 62 "D:\\Program Files\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"

   




 















 
#line 93 "D:\\Program Files\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"











 





extern __softfp unsigned __ARM_dcmp4(double  , double  );
extern __softfp unsigned __ARM_fcmp4(float  , float  );
    




 

extern __declspec(__nothrow) __softfp int __ARM_fpclassifyf(float  );
extern __declspec(__nothrow) __softfp int __ARM_fpclassify(double  );
     
     

__inline __declspec(__nothrow) __softfp int __ARM_isfinitef(float __x)
{
    return (((*(unsigned *)&(__x)) >> 23) & 0xff) != 0xff;
}
__inline __declspec(__nothrow) __softfp int __ARM_isfinite(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff) != 0x7ff;
}
     
     

__inline __declspec(__nothrow) __softfp int __ARM_isinff(float __x)
{
    return ((*(unsigned *)&(__x)) << 1) == 0xff000000;
}
__inline __declspec(__nothrow) __softfp int __ARM_isinf(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) << 1) == 0xffe00000) && ((*(unsigned *)&(__x)) == 0);
}
     
     

__inline __declspec(__nothrow) __softfp int __ARM_islessgreaterf(float __x, float __y)
{
    unsigned __f = __ARM_fcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);  
}
__inline __declspec(__nothrow) __softfp int __ARM_islessgreater(double __x, double __y)
{
    unsigned __f = __ARM_dcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);  
}
    


 

__inline __declspec(__nothrow) __softfp int __ARM_isnanf(float __x)
{
    return (0x7f800000 - ((*(unsigned *)&(__x)) & 0x7fffffff)) >> 31;
}
__inline __declspec(__nothrow) __softfp int __ARM_isnan(double __x)
{
    unsigned __xf = (*(1 + (unsigned *)&(__x))) | (((*(unsigned *)&(__x)) == 0) ? 0 : 1);
    return (0x7ff00000 - (__xf & 0x7fffffff)) >> 31;
}
     
     

__inline __declspec(__nothrow) __softfp int __ARM_isnormalf(float __x)
{
    unsigned __xe = ((*(unsigned *)&(__x)) >> 23) & 0xff;
    return (__xe != 0xff) && (__xe != 0);
}
__inline __declspec(__nothrow) __softfp int __ARM_isnormal(double __x)
{
    unsigned __xe = ((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff;
    return (__xe != 0x7ff) && (__xe != 0);
}
     
     

__inline __declspec(__nothrow) __softfp int __ARM_signbitf(float __x)
{
    return (*(unsigned *)&(__x)) >> 31;
}
__inline __declspec(__nothrow) __softfp int __ARM_signbit(double __x)
{
    return (*(1 + (unsigned *)&(__x))) >> 31;
}
     
     








#line 211 "D:\\Program Files\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"



   
  typedef float float_t;
  typedef double double_t;







extern const int math_errhandling;



extern __declspec(__nothrow) double acos(double  );
    
    
    
extern __declspec(__nothrow) double asin(double  );
    
    
    
    

extern __declspec(__nothrow) __pure double atan(double  );
    
    

extern __declspec(__nothrow) double atan2(double  , double  );
    
    
    
    

extern __declspec(__nothrow) double cos(double  );
    
    
    
    
extern __declspec(__nothrow) double sin(double  );
    
    
    
    

extern void __use_accurate_range_reduction(void);
    
    

extern __declspec(__nothrow) double tan(double  );
    
    
    
    

extern __declspec(__nothrow) double cosh(double  );
    
    
    
    
extern __declspec(__nothrow) double sinh(double  );
    
    
    
    
    

extern __declspec(__nothrow) __pure double tanh(double  );
    
    

extern __declspec(__nothrow) double exp(double  );
    
    
    
    
    

extern __declspec(__nothrow) double frexp(double  , int *  ) __attribute__((__nonnull__(2)));
    
    
    
    
    
    

extern __declspec(__nothrow) double ldexp(double  , int  );
    
    
    
    
extern __declspec(__nothrow) double log(double  );
    
    
    
    
    
extern __declspec(__nothrow) double log10(double  );
    
    
    
extern __declspec(__nothrow) double modf(double  , double *  ) __attribute__((__nonnull__(2)));
    
    
    
    

extern __declspec(__nothrow) double pow(double  , double  );
    
    
    
    
    
    
extern __declspec(__nothrow) double sqrt(double  );
    
    
    




    __inline double _sqrt(double __x) { return sqrt(__x); }




    __inline float _sqrtf(float __x) { return (float)sqrt(__x); }

    



 

extern __declspec(__nothrow) __pure double ceil(double  );
    
    
extern __declspec(__nothrow) __pure double fabs(double  );
    
    

extern __declspec(__nothrow) __pure double floor(double  );
    
    

extern __declspec(__nothrow) double fmod(double  , double  );
    
    
    
    
    

    









 



extern __declspec(__nothrow) double acosh(double  );
    

 
extern __declspec(__nothrow) double asinh(double  );
    

 
extern __declspec(__nothrow) double atanh(double  );
    

 
extern __declspec(__nothrow) double cbrt(double  );
    

 
__inline __declspec(__nothrow) __pure double copysign(double __x, double __y)
    

 
{
    (*(1 + (unsigned *)&(__x))) = ((*(1 + (unsigned *)&(__x))) & 0x7fffffff) | ((*(1 + (unsigned *)&(__y))) & 0x80000000);
    return __x;
}
__inline __declspec(__nothrow) __pure float copysignf(float __x, float __y)
    

 
{
    (*(unsigned *)&(__x)) = ((*(unsigned *)&(__x)) & 0x7fffffff) | ((*(unsigned *)&(__y)) & 0x80000000);
    return __x;
}
extern __declspec(__nothrow) double erf(double  );
    

 
extern __declspec(__nothrow) double erfc(double  );
    

 
extern __declspec(__nothrow) double expm1(double  );
    

 



    

 






#line 445 "D:\\Program Files\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"


extern __declspec(__nothrow) double hypot(double  , double  );
    




 
extern __declspec(__nothrow) int ilogb(double  );
    

 
extern __declspec(__nothrow) int ilogbf(float  );
    

 
extern __declspec(__nothrow) int ilogbl(long double  );
    

 







    

 





    



 





    



 





    

 





    



 





    



 





    



 





    

 





    

 





    


 

extern __declspec(__nothrow) double lgamma (double  );
    


 
extern __declspec(__nothrow) double log1p(double  );
    

 
extern __declspec(__nothrow) double logb(double  );
    

 
extern __declspec(__nothrow) float logbf(float  );
    

 
extern __declspec(__nothrow) long double logbl(long double  );
    

 
extern __declspec(__nothrow) double nextafter(double  , double  );
    


 
extern __declspec(__nothrow) float nextafterf(float  , float  );
    


 
extern __declspec(__nothrow) long double nextafterl(long double  , long double  );
    


 
extern __declspec(__nothrow) double nexttoward(double  , long double  );
    


 
extern __declspec(__nothrow) float nexttowardf(float  , long double  );
    


 
extern __declspec(__nothrow) long double nexttowardl(long double  , long double  );
    


 
extern __declspec(__nothrow) double remainder(double  , double  );
    

 
extern __declspec(__nothrow) __pure double rint(double  );
    

 
extern __declspec(__nothrow) double scalbln(double  , long int  );
    

 
extern __declspec(__nothrow) float scalblnf(float  , long int  );
    

 
extern __declspec(__nothrow) long double scalblnl(long double  , long int  );
    

 
extern __declspec(__nothrow) double scalbn(double  , int  );
    

 
extern __declspec(__nothrow) float scalbnf(float  , int  );
    

 
extern __declspec(__nothrow) long double scalbnl(long double  , int  );
    

 




    

 



 
extern __declspec(__nothrow) __pure float _fabsf(float);  
__inline __declspec(__nothrow) __pure float fabsf(float __f) { return _fabsf(__f); }
extern __declspec(__nothrow) float sinf(float  );
extern __declspec(__nothrow) float cosf(float  );
extern __declspec(__nothrow) float tanf(float  );
extern __declspec(__nothrow) float acosf(float  );
extern __declspec(__nothrow) float asinf(float  );
extern __declspec(__nothrow) float atanf(float  );
extern __declspec(__nothrow) float atan2f(float  , float  );
extern __declspec(__nothrow) float sinhf(float  );
extern __declspec(__nothrow) float coshf(float  );
extern __declspec(__nothrow) float tanhf(float  );
extern __declspec(__nothrow) float expf(float  );
extern __declspec(__nothrow) float logf(float  );
extern __declspec(__nothrow) float log10f(float  );
extern __declspec(__nothrow) float powf(float  , float  );
extern __declspec(__nothrow) float sqrtf(float  );
extern __declspec(__nothrow) float ldexpf(float  , int  );
extern __declspec(__nothrow) float frexpf(float  , int *  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) __pure float ceilf(float  );
extern __declspec(__nothrow) __pure float floorf(float  );
extern __declspec(__nothrow) float fmodf(float  , float  );
extern __declspec(__nothrow) float modff(float  , float *  ) __attribute__((__nonnull__(2)));

 
 













 
__declspec(__nothrow) long double acosl(long double );
__declspec(__nothrow) long double asinl(long double );
__declspec(__nothrow) long double atanl(long double );
__declspec(__nothrow) long double atan2l(long double , long double );
__declspec(__nothrow) long double ceill(long double );
__declspec(__nothrow) long double cosl(long double );
__declspec(__nothrow) long double coshl(long double );
__declspec(__nothrow) long double expl(long double );
__declspec(__nothrow) long double fabsl(long double );
__declspec(__nothrow) long double floorl(long double );
__declspec(__nothrow) long double fmodl(long double , long double );
__declspec(__nothrow) long double frexpl(long double , int* ) __attribute__((__nonnull__(2)));
__declspec(__nothrow) long double ldexpl(long double , int );
__declspec(__nothrow) long double logl(long double );
__declspec(__nothrow) long double log10l(long double );
__declspec(__nothrow) long double modfl(long double  , long double *  ) __attribute__((__nonnull__(2)));
__declspec(__nothrow) long double powl(long double , long double );
__declspec(__nothrow) long double sinl(long double );
__declspec(__nothrow) long double sinhl(long double );
__declspec(__nothrow) long double sqrtl(long double );
__declspec(__nothrow) long double tanl(long double );
__declspec(__nothrow) long double tanhl(long double );





 
extern __declspec(__nothrow) float acoshf(float  );
__declspec(__nothrow) long double acoshl(long double );
extern __declspec(__nothrow) float asinhf(float  );
__declspec(__nothrow) long double asinhl(long double );
extern __declspec(__nothrow) float atanhf(float  );
__declspec(__nothrow) long double atanhl(long double );
__declspec(__nothrow) long double copysignl(long double , long double );
extern __declspec(__nothrow) float cbrtf(float  );
__declspec(__nothrow) long double cbrtl(long double );
extern __declspec(__nothrow) float erff(float  );
__declspec(__nothrow) long double erfl(long double );
extern __declspec(__nothrow) float erfcf(float  );
__declspec(__nothrow) long double erfcl(long double );
extern __declspec(__nothrow) float expm1f(float  );
__declspec(__nothrow) long double expm1l(long double );
extern __declspec(__nothrow) float log1pf(float  );
__declspec(__nothrow) long double log1pl(long double );
extern __declspec(__nothrow) float hypotf(float  , float  );
__declspec(__nothrow) long double hypotl(long double , long double );
extern __declspec(__nothrow) float lgammaf(float  );
__declspec(__nothrow) long double lgammal(long double );
extern __declspec(__nothrow) float remainderf(float  , float  );
__declspec(__nothrow) long double remainderl(long double , long double );
extern __declspec(__nothrow) float rintf(float  );
__declspec(__nothrow) long double rintl(long double );



#line 825 "D:\\Program Files\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"





#line 980 "D:\\Program Files\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"











#line 1182 "D:\\Program Files\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"



 

#line 15 "devices\\touch.c"




static TouchTpyDef	t_pad = {
	tp_init,
	tp_scan,
	tp_adjust,
	0, 0, 0, 0,
	0,
	-0.130680, -0.085418, 508, 332,
	P_DIR_HOR,
	0x90, 0xd0
};


static void drawToolPad(void);








 
void tp_spiWbyte(u8 byte)
{
	u8 count=0;   
	for(count=0;count<8;count++)  
	{ 	  
		if(byte&0x80)
		{
			*((vu32*)(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1C00))->ODR)&0xF0000000)+0x2000000 +(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1C00))->ODR)&0xFFFFF)<<5) + (9<<2))) = 1;
		}
		else
		{
			*((vu32*)(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1C00))->ODR)&0xF0000000)+0x2000000 +(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1C00))->ODR)&0xFFFFF)<<5) + (9<<2))) = 0;
		}			   
		byte<<=1;    
		*((vu32*)(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->ODR)&0xF0000000)+0x2000000 +(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->ODR)&0xFFFFF)<<5) + (1<<2)))=0; 	 
		*((vu32*)(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->ODR)&0xF0000000)+0x2000000 +(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->ODR)&0xFFFFF)<<5) + (1<<2)))=1;		
	}		
}








 
u16 tp_readAD(u8 cmd)
{
	u8 count=0; 	  
	u16 Num=0; 
	*((vu32*)(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->ODR)&0xF0000000)+0x2000000 +(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->ODR)&0xFFFFF)<<5) + (1<<2)))=0;		
	*((vu32*)(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1C00))->ODR)&0xF0000000)+0x2000000 +(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1C00))->ODR)&0xFFFFF)<<5) + (9<<2)))=0; 	
	*((vu32*)(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->ODR)&0xF0000000)+0x2000000 +(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->ODR)&0xFFFFF)<<5) + (2<<2)))=0; 		
	tp_spiWbyte(cmd);
	_delay_us(6);
	*((vu32*)(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->ODR)&0xF0000000)+0x2000000 +(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->ODR)&0xFFFFF)<<5) + (1<<2)))=0; 	     	    
	_delay_us(1);    	   
	*((vu32*)(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->ODR)&0xF0000000)+0x2000000 +(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->ODR)&0xFFFFF)<<5) + (1<<2)))=1;		
	*((vu32*)(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->ODR)&0xF0000000)+0x2000000 +(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->ODR)&0xFFFFF)<<5) + (1<<2)))=0; 	     	    
	for(count=0;count<16;count++)
	{ 				  
		Num<<=1; 	 
		*((vu32*)(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->ODR)&0xF0000000)+0x2000000 +(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->ODR)&0xFFFFF)<<5) + (1<<2)))=0;	
		*((vu32*)(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->ODR)&0xF0000000)+0x2000000 +(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->ODR)&0xFFFFF)<<5) + (1<<2)))=1;
		if(*((vu32*)(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1C00))->IDR)&0xF0000000)+0x2000000 +(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1C00))->IDR)&0xFFFFF)<<5) + (8<<2))))Num++; 		 
	}  	
	Num>>=4;   	
	*((vu32*)(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->ODR)&0xF0000000)+0x2000000 +(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->ODR)&0xFFFFF)<<5) + (2<<2)))=1;		
	return(Num);  
}









 


u16 tp_readXorY(u8 cmd)
{
	u16 i, j;
	u16 buf[5];
	u16 sum=0;
	u16 temp;

	for(i=0;i<5;i++)
	{
		buf[i]=tp_readAD(cmd);		 		    
	}
	for(i=0;i<5-1; i++)
	{
		for(j=i+1;j<5;j++)
		{
			if(buf[i]>buf[j])
			{
				temp=buf[i];
				buf[i]=buf[j];
				buf[j]=temp;
			}
		}
	}	  
	sum=0;
	for(i=1;i<5-1;i++)
	{
		sum+=buf[i];
	}

	temp=sum/(5-2*1);

	return temp;  
}







 
u8 tp_readXandY(u16* x, u16* y)
{
	u16 xtemp,ytemp;			 	 		  
	xtemp=tp_readXorY(t_pad.cmdRdx);
	ytemp=tp_readXorY(t_pad.cmdRdy);	  												   
	*x=xtemp;
	*y=ytemp;
	return 0;
}










 

u8 tp_readXandY2(u16* x, u16* y)
{
	u16 x1,y1;
	u16 x2,y2;
	u8 flag;    
	flag=tp_readXandY(&x1,&y1);  	
	if(flag)
	{
		return(1);
	}
	flag=tp_readXandY(&x2,&y2);	   
	if(flag)
	{
		return(2);   
	}
	if(((x2<=x1&&x1<x2+50)||(x1<=x2&&x2<x1+50))
			&&((y2<=y1&&y1<y2+50)||(y1<=y2&&y2<y1+50)))
	{
		*x=(x1+x2)/2;
		*y=(y1+y2)/2;
		return 0;
	}
	else
	{
		return 3;
	}			
}








 
void tp_drawAdjustPoint(u16 x, u16 y, u16 color)
{
	LcdPen	pen;

	pen.color = color;
	pen.brush = P_SET_BRUSH;
	lcdDrawRect(x-1, y-1, 3, 3, &pen);
	lcdDrawLine(x-15, y, x+15, y, &pen);
	lcdDrawLine(x, y-15, x, y+15, &pen);
	pen.brush = P_SET_NOBRUSH;
	lcdDrawRect(x-5, y-5, 11, 11, &pen);
}








 
u8 tp_scan(u8 tp)
{
	if(*((vu32*)(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1C00))->IDR)&0xF0000000)+0x2000000 +(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1C00))->IDR)&0xFFFFF)<<5) + (10<<2)))==0)
	{
		if(tp)
		{
			tp_readXandY2(&t_pad.currX, &t_pad.currY);
		}
		else if(tp_readXandY2(&t_pad.currX, &t_pad.currY))
		{
			
			t_pad.currX = t_pad.xfac*t_pad.currX+t_pad.xoff;
			t_pad.currY = t_pad.yfac*t_pad.currY+t_pad.yoff;
		} 
		
		if((t_pad.penStat&0x80)==0)
		{			
			
			t_pad.penStat = 0x80|0x40;
			
			t_pad.lastX = t_pad.currX;
			t_pad.lastY = t_pad.currY;

		}			   
	}
	else
	{
		if(t_pad.penStat&0x80)
		{
			t_pad.penStat&=0x7F;
		}
		else
		{
			t_pad.lastX = 0;
			t_pad.lastY = 0;
			t_pad.currX = 0xFFFF;
			t_pad.currY = 0xFFFF;
		}	    
	}
	return t_pad.penStat&0x80;
}








 
u8 tp_init(void)
{

	
	((RCC_TypeDef *) ((((u32)0x40000000) + 0x20000) + 0x1000))->APB2ENR|=1<<3;    
	((RCC_TypeDef *) ((((u32)0x40000000) + 0x20000) + 0x1000))->APB2ENR|=1<<7;    

	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->CRL&=0XFFFFF00F;
	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->CRL|=0X00000330; 
	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->ODR|=3<<1;      
	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1C00))->CRH&=0XFFFFF000;
	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1C00))->CRH|=0X00000838;
	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1C00))->ODR|=7<<8;      
	tp_readXandY(&t_pad.currX, &t_pad.currY);

	lcdClear(0xFFFF);	
	if(t_pad.xfac>-0.000001&&t_pad.xfac<0.000001){
		if(tp_adjust()==0)
		{
			return 0;
		}
		else 
		{
			return 1;
		}
	}
	return 0;
}









 
u8 tp_adjust(void)
{
	u16 pos_temp[4][2];
	u8  cnt=0;	
	u16 d1,d2;
	u32 tem1,tem2;
	float fac; 	
	u16 outtime=0;
				
	LcdPen	pen;
	pen.color = 0x001F;
	pen.p_size = P_S16EN;
	
	cnt=0;
	
	lcdClear(0xFFFF);

	tp_drawAdjustPoint(20, 20, 0x001F);
	
	lcdDrawStr(100, 120, (u8*)"请依次点击屏幕上的校准点进行屏幕校准!", &pen);

	t_pad.penStat = 0;
	t_pad.xfac = 0;

	while(1)
	{
		
		t_pad.scan(1);
		if((t_pad.penStat&0xc0)==0x40)
		{	
			outtime=0;		
			t_pad.penStat&=~(1<<6);

			pos_temp[cnt][0]=t_pad.currX;
			pos_temp[cnt][1]=t_pad.currY;
			cnt++;	  
			switch(cnt)
			{			   
				case 1:						 
					
					tp_drawAdjustPoint(20,20,0xFFFF);
					
					tp_drawAdjustPoint(480-20,20,0x001F);
					break;
				case 2:
					
					tp_drawAdjustPoint(480-20,20,0xFFFF);
					
					tp_drawAdjustPoint(20,320-20,0x001F);
					break;
				case 3:
					
					tp_drawAdjustPoint(20,320-20,0xFFFF);
					
					tp_drawAdjustPoint(480-20,320-20,0x001F);
					break;
				case 4:	 
					
					tem1=abs(pos_temp[0][0]-pos_temp[1][0]);
					tem2=abs(pos_temp[0][1]-pos_temp[1][1]);
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);

					tem1=abs(pos_temp[2][0]-pos_temp[3][0]);
					tem2=abs(pos_temp[2][1]-pos_temp[3][1]);
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05||d1==0||d2==0)
					{
						cnt=0;
						tp_drawAdjustPoint(480-20,320-20,0xFFFF);	
						tp_drawAdjustPoint(20,20,0x001F);
						tp_printAdjustInfo(pos_temp, fac*100);
						continue;
					}
					tem1=abs(pos_temp[0][0]-pos_temp[2][0]);
					tem2=abs(pos_temp[0][1]-pos_temp[2][1]);
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);

					tem1=abs(pos_temp[1][0]-pos_temp[3][0]);
					tem2=abs(pos_temp[1][1]-pos_temp[3][1]);
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05)
					{
						cnt=0;
						tp_drawAdjustPoint(460,300,0xFFFF);	
						tp_drawAdjustPoint(20,20,0x001F);
						tp_printAdjustInfo(pos_temp, fac*100);
						
						continue;
					}

					
					tem1=abs(pos_temp[1][0]-pos_temp[2][0]);
					tem2=abs(pos_temp[1][1]-pos_temp[2][1]);
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);

					tem1=abs(pos_temp[0][0]-pos_temp[3][0]);
					tem2=abs(pos_temp[0][1]-pos_temp[3][1]);
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05)
					{
						cnt=0;
						tp_drawAdjustPoint(460,300,0xFFFF);	
						tp_drawAdjustPoint(20,20,0x001F);
						tp_printAdjustInfo(pos_temp, fac*100);
						
						continue;
					}
					
					t_pad.xfac=(float)(480-40)/(pos_temp[1][0]-pos_temp[0][0]);
					t_pad.xoff=(480-t_pad.xfac*(pos_temp[1][0]+pos_temp[0][0]))/2;

					t_pad.yfac=(float)(320-40)/(pos_temp[2][1]-pos_temp[0][1]);
					t_pad.yoff=(320-t_pad.yfac*(pos_temp[2][1]+pos_temp[0][1]))/2;
					if(abs(t_pad.xfac)>2||abs(t_pad.yfac)>2)
					{
						cnt=0;
						lcdClear(0xFFFF);
						lcdDrawStr(100,120, (u8*)"未完成校验，请重新操作！", &pen);
						tp_drawAdjustPoint(480-20,320-20,0xFFFF);	
						tp_drawAdjustPoint(20,20,0x001F);								

						t_pad.direct = (t_pad.direct==P_DIR_HOR)? P_DIR_VER:P_DIR_HOR;
						if(t_pad.direct)
						{
							t_pad.cmdRdx = 0x90;
							t_pad.cmdRdy = 0xD0; 
						}else				   
						{
							t_pad.cmdRdx = 0xD0;
							t_pad.cmdRdy = 0x90; 
						}			    
						continue;
					}		
					lcdClear(0xFFFF);
					lcdDrawStr(100,120, (u8*)"屏幕校验完成!", &pen);




					_delay_ms(1000);  
					lcdClear(0xFFFF);
					return 0;
			}
		}
		_delay_ms(10);
		outtime++;
		if(outtime>1000)
		{
			lcdClear(0xFFFF);
			lcdDrawStr(100, 120, (u8*)"屏幕校验操作超时！", &pen);
			_delay_ms(1000);
			lcdClear(0xFFFF);
			break;
		} 
	}
	return 0;
}









 
u8 tp_printAdjustInfo(u16 pos[][2], u16 fac)
{
	printf("x1 = %d, y1 = %d\r\n", pos[0][0], pos[0][1]);
	printf("x2 = %d, y2 = %d\r\n", pos[1][0], pos[1][1]);
	printf("x3 = %d, y3 = %d\r\n", pos[2][0], pos[2][1]);
	printf("x4 = %d, y4 = %d\r\n", pos[3][0], pos[3][1]);
	printf("fac = %d\r\n", fac);
	return 0;
}








 
void tp_screenTrack(void)
{
	u16 x, y;
	LcdPen pen;
	pen.color = 0x001F;
	pen.p_size = P_S16EN;
	pen.brush = P_SET_BRUSH;
	pen.direct = P_DIR_HOR;
	
	
	drawToolPad();
	
	while(1)
	{
		if(t_pad.scan(1))
		{
			x = t_pad.currX*t_pad.xfac+t_pad.xoff;
			y = t_pad.currY*t_pad.yfac+t_pad.yoff;

			if(x<480&&y<320)
			{
				if(y>16)
				{
					lcdDrawBigPoint(x, y, &pen);
				}
				else
				{
					if(x>(480-32))
					{
						pen.color = 0xFFFF;
						lcdDrawRect(0, 17, 480, 310, &pen);
					}
					else if(x<(1*32))
					{
						pen.color = 0x001F;
					}
					else if(x>(1*32)&&x<(2*32))
					{
						pen.color = 0xF800;
					}
					else if(x>(2*32)&&x<(3*32))
					{
						pen.color = 0x07E0;
					}
					else if(x>(3*32)&&x<(4*32))
					{
						pen.color = 0x0000;
					}
					else if(x>(4*32)&&x<(5*32))
					{
						pen.color = 0x07FF;
					}
					else if(x>(5*32)&&x<(6*32))
					{
						pen.color = 0XFFE0;
					}
					else if(x>(6*32)&&x<(7*32))
					{
						pen.color = 0xFFFF;
					}
				}
			}		
		}
		else
		{
			_delay_ms(10);
		}
	}
}



 


void drawToolPad(void)
{
	LcdPen pen;
	pen.color = 0x001F;
	pen.p_size = P_S16EN;
	pen.brush = P_SET_BRUSH;
	pen.direct = P_DIR_HOR;
	
	pen.color = 0x001F;
	lcdDrawRect(32*0, 0, 32, 16, &pen);
	pen.color = 0xF800;
	lcdDrawRect(32*1, 0, 32, 16, &pen);
	pen.color = 0x07E0;
	lcdDrawRect(32*2, 0, 32, 16, &pen);
	pen.color = 0x0000;
	lcdDrawRect(32*3, 0, 32, 16, &pen);
	pen.color = 0x07FF;
	lcdDrawRect(32*4, 0, 32, 16, &pen);
	pen.color = 0XFFE0;
	lcdDrawRect(32*5, 0, 32, 16, &pen);
	pen.color = 0xFFFF;
	lcdDrawRect(32*6, 0, 32, 16, &pen);
	pen.color = 0xF800;
	lcdDrawStr(480-32, 0, (u8*)"清屏", &pen);	
}


