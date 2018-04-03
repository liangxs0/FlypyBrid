#line 1 "app\\game.c"
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
#line 1 "app\\game.h"
#line 47 "app\\game.h"


#line 4 "app\\Font_Num.h"

void Font_DrawNum_Big( u16 x, u16 y, u8 * num );
void Font_DrawNum_Small( u16 x, u16 y, u8 * num );
void Font_Num_DrawNum( u16 x, u16 y, u16 num, u8 mode );
void Font_DrawNum_Black( u16 x, u16 y, u8 * num );
void Font_Num_DrawNum_Multi( u16 x, u16 y, u32 num, u8 mode );
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



#line 2 "app\\game.c"
#line 1 ".\\app\\RES\\Pic\\flappybird_background.c"
const unsigned char gImage_flappybird_background[33600] = {  
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,
0XF9,0X45,0X39,0X4E,0X3A,0X56,0XF8,0X55,0X39,0X56,0X39,0X56,0X38,0X5E,0X39,0X66,
0X39,0X66,0X39,0X66,0X39,0X66,0X39,0X66,0X39,0X66,0X39,0X5E,0X39,0X5E,0X39,0X56,
0X39,0X56,0X39,0X56,0X39,0X4E,0XF8,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X39,0X56,0X39,0X56,0X39,0X56,
0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,
0X39,0X4E,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X39,0X4E,0X39,0X56,
0X39,0X56,0X38,0X56,0X39,0X56,0X39,0X5E,0X39,0X6E,0X38,0X6E,0X79,0XDF,0XBA,0XDF,
0XBA,0XDF,0XBA,0XDF,0XBA,0XE7,0XBA,0XDF,0XFA,0XD7,0XBA,0XD7,0XBB,0XD7,0X79,0X76,
0X38,0X5E,0X39,0X5E,0X39,0X4E,0X38,0X56,0X39,0X56,0X39,0X56,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0X39,0X56,0X39,0X4E,0X39,0X4E,0X39,0X56,0X39,0X56,0X38,0X5E,
0X39,0X66,0X39,0X66,0X39,0X5E,0X39,0X5E,0X39,0X66,0X39,0X5E,0X39,0X5E,0X38,0X56,
0X38,0X56,0X39,0X4E,0X39,0X56,0X39,0X4E,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,
0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,
0XF9,0X45,0XF9,0X45,0XF9,0X4D,0X39,0X4E,0X39,0X4E,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0X39,0X56,0X79,0X56,0X39,0X56,
0X39,0X5E,0X7A,0X66,0X79,0X6E,0XFD,0XC7,0XFC,0XCF,0XFC,0XD7,0XFB,0XDF,0XFB,0XE7,
0XFA,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFA,0XDF,0XFA,0XE7,0XFD,0XCF,
0XFC,0XC7,0XFD,0XC7,0X79,0X6E,0X39,0X5E,0X39,0X5E,0X38,0X56,0X39,0X4E,0X39,0X56,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,0X3A,0X5E,0X39,0X56,
0X39,0X66,0X39,0X66,0X39,0X66,0X39,0X66,0X39,0X66,0X79,0X5E,0X39,0X56,0XF9,0X55,
0X3A,0X56,0X3A,0X56,0X39,0X56,0X39,0X56,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,
0XF7,0X55,0XF8,0X55,0XF8,0X4D,0XF8,0X4D,0XF8,0X4D,0XF8,0X4D,0XF8,0X4D,0XF8,0X4D,
0XF8,0X4D,0XF9,0X4D,0XF9,0X4D,0X39,0X56,0X39,0X4E,0XF9,0X45,0XF9,0X4D,0XF9,0X45,
0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0X39,0X46,0X39,0X56,0XFA,0X55,0XF9,0X55,
0X39,0X5E,0X79,0X66,0X78,0X6E,0XBA,0XDF,0XBA,0XDF,0XFA,0XDF,0XFA,0XDF,0XFB,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XEF,0XFA,0XDF,0XFB,0XDF,0XFB,0XE7,
0XBA,0XD7,0XBB,0XD7,0X78,0X76,0X38,0X5E,0X39,0X5E,0X39,0X56,0X39,0X4E,0X39,0X56,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0X39,0X4E,0X39,0X56,
0X39,0X56,0XF9,0X55,0X3A,0X56,0X3A,0X5E,0X39,0X5E,0X78,0X6E,0XBB,0XD7,0XBA,0XD7,
0XBA,0XDF,0XBA,0XDF,0XBA,0XDF,0XBA,0XDF,0XBA,0XDF,0XFA,0XD7,0XBB,0XD7,0X38,0X6E,
0X79,0X6E,0X39,0X56,0X39,0X56,0X3A,0X56,0X39,0X56,0X3A,0X56,0X39,0X4E,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X39,0X4E,0X39,0X4E,0XF9,0X55,0X39,0X56,
0X39,0X56,0X39,0X4E,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,0X38,0X4E,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,
0XF9,0X45,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF8,0X4D,0XF9,0X4D,0XFC,0XDF,0XFC,0XDF,
0XFB,0XDF,0XFC,0XDF,0XFB,0XDF,0XFA,0XDF,0XBA,0XD7,0XFB,0XD7,0XBB,0XCF,0X37,0X7E,
0X7A,0X66,0X39,0X66,0X39,0X5E,0X39,0X56,0X39,0X4E,0X38,0X56,0X39,0X56,0X39,0X56,
0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X5E,0X39,0X66,
0XBA,0XDF,0XFA,0XDF,0XFA,0XDF,0XFA,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XBB,0XE7,0XB9,0XDF,0XBC,0XD7,0X38,0X66,0XF8,0X55,0X39,0X56,
0X39,0X4E,0XF9,0X55,0XF9,0X4D,0XFA,0X4D,0XF9,0X4D,0X39,0X56,0XF9,0X55,0X39,0X56,
0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X4E,0X38,0X56,0X39,0X56,
0X39,0X56,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,
0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X39,0X46,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,
0XFA,0X4D,0X3A,0X56,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,
0X39,0X4E,0X39,0X4E,0XF9,0X4D,0XF9,0X4D,0XF8,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,
0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0X39,0X56,0X39,0X56,
0X39,0X5E,0X39,0X5E,0X79,0X66,0XFD,0XC7,0XFD,0XC7,0XFD,0XCF,0XFB,0XE7,0XFA,0XDF,
0XFB,0XE7,0XFB,0XE7,0XFB,0XE7,0XFB,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XDF,0XFD,0XCF,
0XFD,0XCF,0XFC,0XC7,0X79,0X66,0X7A,0X6E,0X79,0X56,0X38,0X56,0X39,0X56,0X39,0X4E,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0X39,0X4E,0X39,0X4E,0X39,0X56,0X39,0X56,0X38,0X5E,0X3A,0X5E,0X3A,0X5E,
0X79,0X66,0X39,0X66,0X39,0X66,0X39,0X66,0X39,0X66,0X39,0X66,0X3A,0X5E,0X39,0X56,
0X39,0X56,0X39,0X4E,0X39,0X4E,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0X39,0X4E,0X39,0X4E,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0X39,0X46,
0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF8,0X45,0XF7,0X5D,0XFC,0XDF,0XFB,0XDF,0XFC,0XDF,0XFA,0XEF,0XFB,0XE7,
0XFA,0XEF,0XFA,0XE7,0XFB,0XE7,0XFB,0XE7,0XFB,0XE7,0XFB,0XE7,0XFB,0XDF,0XFB,0XD7,
0XFD,0XCF,0XFC,0XC7,0XFD,0XC7,0XFD,0XAF,0X79,0X6E,0X3A,0X5E,0X7A,0X56,0X39,0X56,
0X39,0X4E,0X39,0X4E,0X39,0X4E,0X39,0X4E,0X39,0X56,0X39,0X5E,0XFD,0XB7,0XFD,0XC7,
0XFA,0XDF,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XB9,0XD7,0XBB,0XDF,0XFC,0XC7,0X38,0X6E,0X3A,0X5E,
0X39,0X56,0X39,0X56,0X39,0X4E,0X39,0X4E,0X39,0X4E,0X39,0X56,0X39,0X56,0X38,0X5E,
0X39,0X66,0X39,0X66,0X79,0X66,0X79,0X66,0X39,0X66,0X39,0X5E,0X3A,0X5E,0X39,0X56,
0X38,0X4E,0X39,0X56,0X39,0X56,0X39,0X4E,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0X39,0X4E,0X39,0X4E,0X39,0X4E,0X39,0X56,0X39,0X4E,0X39,0X4E,
0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X4E,0XF9,0X4D,0XF9,0X45,0XF9,0X45,
0XF9,0X45,0XF9,0X45,0XF9,0X45,0X39,0X56,0XFA,0X55,0X39,0X4E,0X39,0X4E,0X39,0X4E,
0X39,0X56,0X39,0X56,0X38,0X56,0X39,0X5E,0X39,0X66,0X39,0X66,0X79,0X66,0X79,0X66,
0X39,0X4E,0X3A,0X4E,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0X39,0X46,0XF9,0X4D,0X39,0X56,0X3A,0X4E,
0X39,0X5E,0X38,0X66,0X39,0X66,0XBA,0XD7,0XBA,0XD7,0XBA,0XE7,0XFA,0XDF,0XFA,0XD7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XDF,0XBA,0XE7,
0XFB,0XDF,0XB9,0XDF,0X38,0X6E,0X39,0X6E,0X39,0X5E,0X39,0X4E,0XF9,0X55,0X39,0X4E,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0X3A,0X4E,0X3A,0X4E,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X5E,0X39,0X5E,
0X38,0X66,0X38,0X66,0X38,0X66,0X38,0X66,0X38,0X66,0X39,0X5E,0X39,0X5E,0X39,0X4E,
0X39,0X56,0X39,0X56,0X39,0X56,0XF8,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0X3A,0X4E,0X3A,0X4E,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF8,0X4D,
0XF9,0X4D,0XF9,0X55,0XFD,0XCF,0XFA,0XE7,0XFA,0XE7,0XFA,0XEF,0XFB,0XE7,0XFB,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XDF,
0XBA,0XDF,0XFA,0XDF,0XBA,0XD7,0XFB,0XCF,0X79,0X66,0X39,0X66,0X39,0X5E,0X39,0X56,
0XF9,0X4D,0X39,0X56,0X3A,0X56,0XF9,0X55,0XF9,0X55,0X38,0X66,0XFC,0XCF,0XBB,0XDF,
0XBA,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFB,0XE7,0XFB,0XEF,0XBA,0XDF,0XBB,0XE7,0XBA,0XDF,0XF6,0X6D,0X3A,0X5E,
0X39,0X56,0X39,0X56,0X39,0X56,0XF9,0X55,0X39,0X56,0X39,0X56,0X39,0X5E,0X39,0X5E,
0X39,0X5E,0X39,0X66,0X39,0X66,0X39,0X66,0X39,0X66,0X79,0X5E,0X39,0X5E,0X39,0X56,
0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,0XF9,0X45,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0X39,0X4E,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,
0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X4E,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X39,0X56,0X39,0X56,0X38,0X56,0X39,0X56,0X39,0X4E,
0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X5E,0X39,0X5E,0X39,0X66,0X39,0X66,0X39,0X66,
0X39,0X56,0X39,0X5E,0X39,0X5E,0X7A,0X5E,0X3A,0X66,0X79,0X5E,0X39,0X5E,0X7A,0X5E,
0X7A,0X5E,0X3A,0X5E,0X39,0X4E,0X3A,0X4E,0XF9,0X45,0X7A,0X56,0X79,0X5E,0X79,0X66,
0XBB,0XD7,0XBB,0XD7,0XFB,0XDF,0XF9,0XE7,0XFA,0XDF,0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XEF,0XFA,0XE7,0XFA,0XEF,
0XFA,0XE7,0XFA,0XE7,0XBA,0XDF,0XFB,0XE7,0XBA,0XD7,0X78,0X66,0X79,0X66,0X39,0X56,
0X39,0X4E,0X39,0X4E,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X39,0X4E,0X39,0X56,
0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X5E,0X7A,0X66,0X78,0X6E,0XBB,0XD7,0XFB,0XD7,
0XBB,0XD7,0XBB,0XDF,0XFB,0XDF,0XFB,0XDF,0XBB,0XD7,0XBB,0XD7,0XFB,0XCF,0X78,0X6E,
0X39,0X5E,0X39,0X56,0X3A,0X56,0X39,0X56,0X39,0X56,0X39,0X56,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0X39,0X56,0X39,0X56,
0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X5E,0X39,0X5E,0X79,0X5E,0X39,0X66,0X7A,0X66,
0X79,0X5E,0X7A,0X66,0X3A,0X5E,0X7A,0X5E,0X3A,0X4E,0XF9,0X4D,0XF9,0X4D,0XB6,0X55,
0XFD,0XD7,0XFB,0XDF,0XBA,0XDF,0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFB,0XE7,0XFA,0XEF,0XFB,0XDF,0XFB,0XDF,0XFA,0XDF,0XBA,0XD7,0XFC,0XC7,0X7A,0X66,
0X39,0X5E,0X39,0X5E,0X79,0X56,0X79,0X66,0X78,0X6E,0XBA,0XD7,0XFA,0XD7,0XFA,0XE7,
0XFA,0XEF,0XFA,0XE7,0XFB,0XEF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,0XBA,0XDF,0X36,0X7E,0X39,0X66,
0X39,0X5E,0X7A,0X56,0X39,0X56,0X38,0X56,0X3A,0X66,0X78,0X6E,0XBB,0XCF,0XBA,0XD7,
0XBA,0XD7,0XBB,0XDF,0XBB,0XDF,0XFB,0XDF,0XFB,0XD7,0XFA,0XDF,0XBB,0XCF,0X78,0X6E,
0X7A,0X66,0X7A,0X56,0X39,0X56,0XF9,0X4D,0X39,0X56,0X39,0X56,0X39,0X4E,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X39,0X56,0X39,0X56,
0X39,0X56,0X39,0X56,0X39,0X56,0X7A,0X5E,0X7A,0X5E,0X79,0X66,0X79,0X66,0X79,0X66,
0X79,0X5E,0X7A,0X5E,0X3A,0X5E,0X3A,0X5E,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,
0XF9,0X55,0XF9,0X4D,0X39,0X56,0X39,0X56,0X3A,0X4E,0X7A,0X56,0X3A,0X56,0X39,0X56,
0X3A,0X5E,0X7A,0X66,0X38,0X6E,0XBB,0XD7,0XFB,0XDF,0XFB,0XD7,0XFB,0XDF,0XFB,0XDF,
0X78,0X66,0XFE,0XAF,0XFD,0XC7,0XFD,0XC7,0XFD,0XCF,0XFD,0XCF,0XFD,0XCF,0XFD,0XC7,
0XFD,0XC7,0XFD,0XC7,0X78,0X7E,0X7A,0X66,0X7A,0X6E,0X79,0X6E,0XFF,0XBF,0XFC,0XCF,
0XFB,0XDF,0XFA,0XDF,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,
0XFA,0XE7,0XFB,0XE7,0XFB,0XEF,0XFA,0XE7,0XB9,0XD7,0XFC,0XC7,0XFF,0XB7,0X39,0X5E,
0X39,0X56,0X39,0X56,0X39,0X4E,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X39,0X4E,0X39,0X56,0X39,0X56,
0X7A,0X56,0X7A,0X66,0X38,0X6E,0XFD,0XBF,0XFD,0XCF,0XFC,0XCF,0XFB,0XE7,0XFB,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XDF,0XFB,0XDF,0XFC,0XCF,
0XFD,0XC7,0XFE,0XC7,0X78,0X6E,0X39,0X56,0X39,0X56,0X39,0X56,0XF9,0X4D,0X39,0X4E,
0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0X39,0X4E,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,
0X39,0X5E,0X7A,0X66,0X78,0X7E,0XFD,0XBF,0XFD,0XC7,0XFD,0XC7,0XFD,0XCF,0XFD,0XCF,
0XFD,0XCF,0XFD,0XC7,0XFD,0XBF,0XBD,0XB7,0X7A,0X6E,0X7B,0X66,0X79,0X7E,0XFD,0XCF,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XEF,0XFA,0XE7,0XFB,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XDF,0XFB,0XDF,0XFD,0XCF,
0XFC,0XC7,0XFD,0XC7,0X7A,0X76,0XFE,0XC7,0XFC,0XCF,0XFA,0XDF,0XFB,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XF9,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFA,0XEF,0XFA,0XE7,0XFB,0XCF,0XFD,0XC7,
0X79,0X76,0X7A,0X76,0X7A,0X66,0XFE,0XC7,0XFD,0XCF,0XFC,0XCF,0XFA,0XDF,0XFA,0XDF,
0XFA,0XE7,0XFB,0XE7,0XFA,0XEF,0XFA,0XE7,0XFB,0XE7,0XFA,0XE7,0XFB,0XE7,0XFD,0XCF,
0XFC,0XC7,0XFD,0XC7,0X79,0X6E,0X7B,0X5E,0X39,0X5E,0X3A,0X56,0X39,0X56,0X39,0X56,
0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0X39,0X56,0X39,0X4E,0X39,0X56,0X39,0X56,0X39,0X56,
0X39,0X56,0X7A,0X66,0X78,0X76,0XFD,0XBF,0XFC,0XC7,0XFC,0XCF,0XFD,0XCF,0XFD,0XCF,
0XFD,0XCF,0XFC,0XCF,0XFC,0XC7,0XFF,0XB7,0X79,0X66,0X79,0X5E,0X39,0X56,0X39,0X56,
0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X5E,0X38,0X6E,0XFC,0XB7,0XF7,0X6D,0X79,0X66,
0XFD,0XC7,0XFD,0XCF,0XFC,0XCF,0XFB,0XE7,0XFB,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0X78,0X6E,0XBB,0XCF,0XBA,0XDF,0XBA,0XDF,0XBA,0XDF,0XBA,0XDF,0XBA,0XDF,0XBA,0XDF,
0XBA,0XDF,0XBA,0XD7,0X36,0X7E,0X79,0X76,0X38,0X7E,0X78,0X76,0XFB,0XCF,0XBA,0XD7,
0XBA,0XDF,0XBA,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XBA,0XDF,0XBA,0XD7,0XFC,0XCF,0X38,0X6E,
0XF9,0X55,0X39,0X56,0X38,0X56,0XF9,0X4D,0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XFA,0X45,0X38,0X56,0X39,0X56,0X38,0X56,
0X38,0X66,0X39,0X6E,0X37,0X7E,0XFA,0XD7,0XBA,0XE7,0XBA,0XDF,0XFA,0XDF,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XDF,0XB9,0XDF,
0XBA,0XDF,0XFC,0XCF,0X37,0X7E,0X38,0X5E,0X39,0X66,0XF9,0X55,0XF9,0X55,0X39,0X4E,
0XF9,0X4D,0XF9,0X4D,0XF8,0X45,0X39,0X56,0X39,0X4E,0XF8,0X4D,0X39,0X5E,0X39,0X56,
0X38,0X66,0X38,0X76,0X35,0X86,0X7A,0XD7,0XBA,0XDF,0XBA,0XDF,0XBA,0XDF,0XBA,0XDF,
0XBA,0XE7,0XBA,0XD7,0XBA,0XDF,0XBA,0XCF,0X78,0X6E,0X38,0X76,0XF6,0X85,0XBA,0XD7,
0XFA,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XDF,0XBA,0XDF,
0XBA,0XDF,0XFB,0XD7,0X78,0X7E,0XFC,0XCF,0XB9,0XE7,0XFA,0XDF,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XDF,
0X38,0X7E,0X38,0X76,0X37,0X76,0XFB,0XCF,0XBA,0XDF,0XBA,0XDF,0XFA,0XDF,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XBA,0XDF,
0XBA,0XDF,0XB9,0XD7,0X38,0X76,0X38,0X66,0X39,0X66,0X38,0X56,0X38,0X56,0X39,0X56,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0X39,0X56,0X39,0X4E,0X39,0X56,0X39,0X56,0X39,0X56,
0X39,0X66,0X79,0X6E,0X36,0X86,0XBA,0XD7,0XFA,0XDF,0XBA,0XDF,0XBA,0XDF,0XBA,0XDF,
0XBA,0XDF,0XF9,0XDF,0XBA,0XDF,0XFD,0XC7,0X77,0X76,0X38,0X66,0X38,0X5E,0X39,0X56,
0X39,0X56,0X38,0X56,0X3A,0X56,0X39,0X66,0X37,0X6E,0XBA,0XD7,0XBB,0XC7,0X78,0X7E,
0XBA,0XD7,0XFA,0XDF,0XB9,0XDF,0XFA,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFC,0XD7,0XFA,0XD7,0XFB,0XDF,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XDF,0XFB,0XDF,0XBB,0XD7,0XFA,0XD7,0XFB,0XD7,0XFB,0XE7,0XFA,0XE7,
0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XDF,0XFB,0XDF,0XFB,0XD7,
0X39,0X5E,0X79,0X5E,0X39,0X56,0X39,0X56,0X39,0X4E,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0X39,0X4E,0X39,0X56,0XF9,0X55,0X39,0X5E,0X39,0X6E,
0XBB,0XCF,0XFB,0XD7,0XFB,0XD7,0XFA,0XDF,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,
0XFA,0XE7,0XFA,0XDF,0XBA,0XD7,0XBB,0XD7,0XBB,0XC7,0X38,0X6E,0X39,0X56,0XF8,0X55,
0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X4E,0X7A,0X5E,0X7A,0X66,0X37,0X76,
0XFA,0XD7,0XFB,0XD7,0XFB,0XD7,0XFB,0XE7,0XFB,0XEF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFB,0XD7,0XFB,0XDF,0XFB,0XD7,0XFB,0XDF,0XFA,0XE7,
0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFB,0XE7,0XFB,0XD7,0XFB,0XDF,0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFB,0XD7,0XFB,0XD7,0XFB,0XD7,0XFB,0XDF,0XFA,0XDF,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,
0XFA,0XE7,0XFB,0XDF,0XFB,0XD7,0XFB,0XD7,0XBB,0XD7,0X39,0X5E,0X3A,0X5E,0XF9,0X5D,
0X39,0X56,0X39,0X56,0X39,0X4E,0X39,0X56,0X38,0X56,0X3A,0X56,0X79,0X66,0X77,0X7E,
0XFB,0XCF,0XFC,0XD7,0XBB,0XDF,0XFA,0XDF,0XFA,0XDF,0XFB,0XE7,0XFB,0XE7,0XFB,0XE7,
0XFB,0XE7,0XFB,0XE7,0XFA,0XDF,0XFB,0XDF,0XFA,0XD7,0XFA,0XCF,0XFE,0XBF,0X7A,0X6E,
0X7A,0X5E,0X79,0X6E,0X7A,0X5E,0XFE,0XBF,0XBB,0XCF,0XFB,0XDF,0XBA,0XD7,0XBA,0XD7,
0XFA,0XDF,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XBA,0XDF,
0XFD,0XC7,0XFE,0XB7,0X39,0X5E,0X39,0X56,0X39,0X56,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X45,0X39,0X4E,0X39,0X4E,0X39,0X56,0X39,0X4E,0X39,0X4E,0X39,0X4E,0X39,0X4E,
0X39,0X4E,0X39,0X4E,0X39,0X56,0X39,0X56,0X39,0X4E,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0X39,0X56,0X39,0X56,0X39,0X5E,0X7A,0X5E,0XFE,0XBF,0XFC,0XC7,
0XFB,0XDF,0XFB,0XE7,0XFA,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFB,0XE7,0XFB,0XE7,0XBA,0XDF,0XBA,0XDF,0XFC,0XCF,0X38,0X76,0X7A,0X66,
0X38,0X5E,0X39,0X5E,0X39,0X56,0X39,0X66,0X38,0X6E,0XFD,0XC7,0XFD,0XD7,0XFB,0XCF,
0XFB,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFB,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFB,0XE7,0XBA,0XDF,0XFD,0XC7,0XFD,0XBF,0X7A,0X66,
0X39,0X5E,0X39,0X5E,0X39,0X5E,0X39,0X66,0X37,0X6E,0XFD,0XC7,0XFD,0XCF,0XFB,0XD7,
0XFB,0XDF,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFB,0XDF,0XFB,0XD7,0XFC,0XCF,
0XFD,0XCF,0XFD,0XCF,0XFD,0XCF,0XFC,0XDF,0XFB,0XE7,0XFB,0XE7,0XFB,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XBA,0XDF,
0X7A,0XDF,0XBC,0XC7,0X39,0X5E,0X39,0X56,0X39,0X56,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0X39,0X4E,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,
0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0X39,0X56,0X39,0X56,0XF8,0X55,0X39,0X66,0XFD,0XCF,0XB9,0XDF,
0XBA,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XEF,0XFA,0XE7,0XFA,0XDF,0XBA,0XDF,0XBA,0XD7,0X36,0X86,0X79,0X6E,
0X79,0X5E,0X3A,0X56,0X39,0X5E,0X39,0X66,0XF6,0X75,0XBA,0XD7,0XBA,0XE7,0XFA,0XDF,
0XFB,0XDF,0XF9,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XEF,0XFA,0XE7,0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFB,0XE7,0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XEF,0XFA,0XE7,0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFA,0XEF,0XFA,0XDF,0XBA,0XDF,0XFC,0XC7,0X39,0X6E,
0X3A,0X5E,0X39,0X5E,0X39,0X56,0X39,0X66,0XF7,0X75,0XBA,0XD7,0XFA,0XDF,0XBA,0XDF,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XEF,0XFA,0XE7,0XFA,0XDF,0XBA,0XDF,
0XB9,0XE7,0XBA,0XDF,0XBA,0XE7,0XFA,0XE7,0XFB,0XE7,0XF9,0XEF,0XFA,0XEF,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XEF,
0XBB,0XDF,0XFC,0XCF,0X79,0X6E,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,
0X39,0X56,0X39,0X56,0X39,0X56,0X79,0X5E,0X7A,0X5E,0X79,0X5E,0X39,0X66,0X79,0X5E,
0X39,0X5E,0X39,0X66,0X79,0X5E,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,0X39,0X56,
0X39,0X56,0X39,0X56,0X39,0X56,0X7A,0X5E,0X39,0X6E,0XFB,0XD7,0XFB,0XDF,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFA,0XD7,0XFB,0XCF,
0X38,0X76,0X7A,0X76,0X78,0X76,0XFB,0XCF,0XBA,0XDF,0XBA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XEF,0XFB,0XE7,
0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,
0XFA,0XEF,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XDF,0XFB,0XDF,0XFA,0XD7,
0X79,0X76,0X79,0X76,0X7A,0X6E,0XFB,0XCF,0XFB,0XDF,0XFA,0XDF,0XF9,0XE7,0XFB,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XEF,0XFB,0XE7,
0XFA,0XE7,0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFC,0XDF,0XFD,0XC7,0X79,0X5E,0X3A,0X66,0X39,0X56,0X39,0X56,0X39,0X4E,
0X79,0X5E,0X7A,0X66,0X78,0X76,0XFD,0XC7,0XFD,0XC7,0XFC,0XCF,0XFD,0XCF,0XFC,0XCF,
0XFD,0XCF,0XFC,0XC7,0XFD,0XBF,0XFE,0XB7,0X79,0X66,0X39,0X66,0X79,0X56,0X39,0X56,
0X39,0X56,0X39,0X56,0X79,0X5E,0XFE,0XB7,0XFC,0XCF,0XFB,0XDF,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFC,0XDF,
0XFC,0XD7,0XFC,0XCF,0XFC,0XD7,0XFB,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XEF,
0XFC,0XD7,0XFC,0XCF,0XFD,0XCF,0XFB,0XDF,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XDF,0XFA,0XDF,0XB9,0XDF,0X39,0X66,0X3A,0X5E,0X38,0X56,0X3A,0X5E,0X38,0X56,
0X39,0X5E,0X79,0X66,0X36,0X7E,0XBB,0XD7,0XB9,0XD7,0XBA,0XDF,0XBA,0XDF,0XBA,0XDF,
0XBA,0XDF,0XBA,0XD7,0XBA,0XD7,0XBD,0XCF,0X39,0X6E,0X39,0X5E,0X39,0X5E,0X39,0X56,
0X39,0X4E,0X39,0X56,0X39,0X5E,0XFD,0XC7,0XFA,0XDF,0XB9,0XDF,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XBA,0XE7,
0XFA,0XE7,0XBA,0XDF,0XBA,0XDF,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XDF,0XFA,0XE7,
0XFA,0XDF,0XFA,0XDF,0XBA,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XEF,
0XFA,0XE7,0XFB,0XE7,0XB9,0XE7,0X79,0X76,0X79,0X76,0X79,0X6E,0X79,0X6E,0X37,0X76,
0XFB,0XCF,0XBB,0XDF,0XFA,0XDF,0XFA,0XDF,0XFA,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XDF,0XFA,0XDF,0XFB,0XD7,0XBA,0XD7,0XFC,0XC7,0X7A,0X66,
0X79,0X66,0X79,0X66,0X7A,0X6E,0XFC,0XCF,0XBA,0XDF,0XFB,0XE7,0XFB,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XEF,0XFA,0XEF,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFB,0XE7,0XFA,0XEF,0XFA,0XEF,0XFC,0XCF,0XFD,0XCF,0XFD,0XC7,0XFD,0XCF,0XFB,0XCF,
0XFB,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFC,0XD7,0XFC,0XCF,
0XFC,0XCF,0XFD,0XCF,0XFD,0XCF,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XEF,0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XBA,0XE7,0XBA,0XDF,0XB9,0XE7,0XFA,0XDF,
0XFB,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFA,0XDF,0XFA,0XE7,0XBA,0XE7,
0XBA,0XDF,0XBA,0XDF,0XBA,0XDF,0XFA,0XE7,0XFB,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,
0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFB,0XE7,0XFA,0XDF,0XFA,0XDF,0XFA,0XDF,
0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XDF,0XFB,0XE7,0XFA,0XDF,0XFA,0XE7,0XFB,0XE7,
0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFA,0XDF,0XFA,0XDF,
0XFA,0XDF,0XFA,0XE7,0XFB,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFA,0XDF,0XFB,0XDF,0XFA,0XDF,0XFA,0XE7,0XFB,0XE7,
0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XDF,0XFA,0XDF,
0XFA,0XDF,0XFA,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XDF,0XFB,0XDF,0XFC,0XD7,0XFC,0XD7,0XFC,0XD7,
0XFC,0XDF,0XFC,0XDF,0XFC,0XDF,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XDF,0XFB,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFC,0XDF,0XFC,0XD7,0XFC,0XD7,0XFC,0XDF,0XFB,0XDF,0XFC,0XDF,
0XFB,0XDF,0XFB,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XDF,0XFB,0XDF,0XFC,0XD7,0XFC,0XD7,
0XFC,0XD7,0XFC,0XDF,0XFC,0XDF,0XFC,0XDF,0XFB,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XDF,0XFB,0XDF,0XFB,0XDF,0XFC,0XD7,0XFB,0XD7,0XFC,0XD7,0XFB,0XD7,0XFC,0XDF,
0XFC,0XDF,0XFB,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XDF,0XFA,0XE7,0XFB,0XE7,0XFB,0XDF,0XFC,0XDF,0XFC,0XD7,
0XFC,0XD7,0XFC,0XD7,0XFC,0XDF,0XFC,0XDF,0XFC,0XDF,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0X78,0XBF,0X38,0XBF,0X39,0XBF,0X38,0XB7,0XF9,0XA6,0XFA,0X9E,0XFA,0XA6,0XFA,0XA6,
0XFA,0XA6,0XFA,0XA6,0XFA,0XA6,0XF9,0XB6,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0X78,0XBF,0X38,0XBF,
0X78,0XB7,0X78,0XB7,0XFA,0XA6,0XFA,0X9E,0XFA,0XA6,0XFA,0XA6,0XFA,0X9E,0X3A,0XA7,
0XFA,0XA6,0XFA,0XA6,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0X78,0XBF,0X78,0XB7,0X78,0XBF,0X77,0XB7,0X39,0XB7,0XF9,0XA6,0XFA,0XA6,
0XF9,0XA6,0XF9,0XA6,0XFA,0XA6,0X3A,0XA7,0XFA,0XA6,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0X78,0XBF,0X78,0XBF,
0X78,0XBF,0X78,0XB7,0XF9,0XA6,0X3A,0XA7,0XFA,0XA6,0XFA,0XA6,0XF9,0XA6,0XFA,0XA6,
0X3A,0XA7,0XFA,0XA6,0XFB,0XE7,0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0X38,0XC7,0X78,0XBF,0X38,0XBF,0X38,0XB7,0X79,0XB7,0XFB,0XA6,0XFA,0XA6,
0XFA,0XA6,0XFA,0XA6,0XFB,0XA6,0XFA,0XA6,0XFA,0XA6,0XFC,0XDF,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0X38,0XBF,0XB7,0XCF,0XB8,0XCF,0X78,0XD7,0XF8,0X9E,0XF8,0XA6,0XB9,0XCF,0X79,0XCF,
0X79,0XCF,0X7A,0XCF,0XF9,0XAE,0XF8,0XAE,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XDF,0XFA,0XDF,0X78,0XBF,0XB9,0XCF,
0XB8,0XCF,0XB8,0XCF,0XF8,0XA6,0XF9,0XA6,0XB8,0XCF,0XB8,0XCF,0X78,0XCF,0XB9,0XD7,
0XF9,0XAE,0XF9,0XAE,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XDF,0X38,0XBF,0X37,0XBF,0X78,0XCF,0XB8,0XCF,0XBA,0XC7,0XFA,0X9E,0XB9,0XCF,
0XB8,0XCF,0X79,0XCF,0X79,0XCF,0XBA,0XC7,0X3A,0XA7,0XFB,0XDF,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XDF,0X38,0XBF,0XB9,0XCF,
0X77,0XD7,0X78,0XCF,0XF8,0XA6,0XFA,0X9E,0XB9,0XCF,0X79,0XCF,0XB8,0XCF,0XBA,0XCF,
0XF8,0XAE,0XFA,0XA6,0XFB,0XE7,0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XDF,0X38,0XBF,0X78,0XBF,0XB8,0XCF,0X78,0XD7,0XBA,0XC7,0XFA,0X9E,0XBA,0XBF,
0X78,0XCF,0XB8,0XCF,0XB9,0XCF,0XBB,0XC7,0XFA,0XA6,0XFC,0XDF,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0X3A,0XB7,0X79,0XC7,0XBA,0XBF,0X7A,0XCF,0XFA,0X9E,0XF8,0XA6,0XB8,0XCF,0XB9,0XCF,
0XB9,0XC7,0XB9,0XCF,0XF9,0XAE,0XF8,0XAE,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XEF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XD7,0XFD,0XD7,0X39,0XB7,0X7A,0XC7,
0XBA,0XC7,0XBA,0XCF,0XF9,0XA6,0XB8,0XA6,0X78,0XD7,0XB9,0XC7,0XB9,0XCF,0XB9,0XCF,
0XF9,0XAE,0XFA,0XAE,0XFB,0XE7,0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFB,0XE7,
0XFC,0XD7,0X79,0XB7,0X7A,0XBF,0X7A,0XC7,0XBA,0XC7,0XBB,0XBF,0XFA,0X9E,0XB9,0XCF,
0XB9,0XCF,0XB9,0XCF,0X79,0XCF,0XBB,0XC7,0X3A,0XA7,0XFB,0XDF,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XD7,0XFC,0XD7,0X39,0XB7,0XBB,0XBF,
0XBA,0XC7,0XBA,0XC7,0XFA,0XA6,0XFA,0X9E,0XB8,0XCF,0XB8,0XC7,0XB9,0XCF,0XB8,0XCF,
0XF9,0XAE,0X3A,0XA7,0XFB,0XE7,0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,
0XFC,0XD7,0X7A,0XBF,0X7A,0XBF,0XBA,0XC7,0X79,0XC7,0XBB,0XBF,0XFA,0X96,0XBA,0XC7,
0XB8,0XD7,0XB9,0XC7,0XB9,0XCF,0XBB,0XC7,0XFA,0XA6,0XFC,0XDF,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0X9E,0XFB,0X9E,0XFA,0X9E,0XFB,0X9E,0XFA,0X9E,0XF8,0XA6,0X78,0XCF,0X38,0XBF,
0X38,0XBF,0XB8,0XCF,0XF9,0XA6,0XF8,0XAE,0XBA,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,
0XB9,0XDF,0XB9,0XDF,0XBA,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XDF,0XF9,0XAE,0X3B,0XA7,0XFB,0X9E,0XFA,0X9E,
0XFA,0X96,0XFA,0X9E,0XFA,0X96,0XF8,0XA6,0XB9,0XCF,0X37,0XBF,0X37,0XBF,0X77,0XD7,
0XF8,0XAE,0XF9,0XA6,0XFA,0XD7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,
0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XBA,0XDF,0XFB,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XEF,0XFB,0XE7,0XFB,0XD7,
0XFB,0XA6,0XF9,0X9E,0XFA,0X9E,0XFB,0X9E,0XFA,0X9E,0XBA,0X9E,0XB9,0X9E,0XB9,0XCF,
0XB9,0XCF,0X38,0XB7,0X78,0XCF,0XBA,0XC7,0XFA,0X9E,0XBB,0XDF,0XBA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,
0XB9,0XDF,0XB9,0XDF,0XBA,0XDF,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XF9,0XAE,0X3B,0XA7,0XFA,0X9E,0XFA,0X9E,
0XFA,0X9E,0XFB,0X9E,0XFA,0X9E,0XF9,0X9E,0XB9,0XCF,0X38,0XBF,0X38,0XBF,0XB8,0XCF,
0XF8,0XAE,0XFA,0XA6,0XBA,0XDF,0XFA,0XEF,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,
0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XBA,0XDF,0XBA,0XDF,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XDF,0XFC,0XD7,
0XFA,0XA6,0XFA,0X9E,0XFA,0X9E,0XFA,0X9E,0XFA,0X9E,0XFA,0X9E,0XFB,0X9E,0XBA,0XC7,
0XB9,0XCF,0X38,0XB7,0XB8,0XCF,0XBA,0XCF,0XFA,0X9E,0XFC,0XD7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XB8,0XCF,0XB8,0XCF,0XB8,0XCF,0X78,0XCF,0XB8,0XCF,0X78,0XCF,0X78,0XD7,0XB8,0XD7,
0XB8,0XD7,0XB8,0XCF,0XF9,0XA6,0XF9,0XAE,0XFB,0XDF,0XFA,0XDF,0XFA,0XDF,0XFA,0XDF,
0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,
0X79,0XDF,0XB9,0XDF,0X7A,0XDF,0XBA,0XDF,0XFA,0XDF,0XFA,0XDF,0XFA,0XDF,0XFA,0XDF,
0XFA,0XDF,0XFA,0XDF,0XFA,0XE7,0XBA,0XDF,0XF9,0XAE,0XF8,0XAE,0XB9,0XCF,0X78,0XCF,
0X79,0XCF,0XB8,0XCF,0XB8,0XCF,0X78,0XCF,0X78,0XD7,0X78,0XD7,0XB8,0XD7,0X77,0XD7,
0XF9,0XA6,0XFA,0X9E,0XFB,0XDF,0XBA,0XE7,0XBA,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,
0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0X7A,0XDF,0X79,0XDF,0X79,0XDF,0XBA,0XE7,0XBA,0XDF,
0XFA,0XDF,0XFA,0XDF,0XFA,0XDF,0XFA,0XDF,0XFA,0XDF,0XFA,0XDF,0XBA,0XDF,0XFC,0XD7,
0XF9,0XA6,0XB9,0XCF,0XB7,0XCF,0XB9,0XCF,0XB8,0XCF,0X78,0XCF,0X78,0XC7,0XB8,0XD7,
0X78,0XD7,0XB9,0XCF,0X78,0XD7,0XBA,0XC7,0XFA,0X9E,0XFB,0XDF,0XBA,0XDF,0XBA,0XDF,
0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XEF,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0X79,0XDF,
0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XF9,0XDF,0XBA,0XDF,0XFB,0XDF,0XFA,0XDF,0XFA,0XDF,
0XFA,0XDF,0XFA,0XDF,0XFB,0XDF,0XFA,0XDF,0XB9,0XAE,0XF9,0XA6,0XB8,0XCF,0XB8,0XCF,
0X78,0XCF,0X78,0XCF,0X79,0XCF,0X78,0XCF,0XB8,0XD7,0X79,0XD7,0XB8,0XD7,0X78,0XCF,
0XF9,0XA6,0XFA,0X9E,0XBA,0XDF,0XBA,0XDF,0XBA,0XE7,0XFB,0XE7,0XFA,0XEF,0XFA,0XEF,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,
0XB9,0XDF,0X79,0XDF,0X79,0XDF,0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0XB9,0XDF,0XFA,0XDF,
0XFA,0XDF,0XFA,0XDF,0XFA,0XDF,0XFA,0XDF,0XFA,0XE7,0XFA,0XDF,0XBA,0XDF,0XFC,0XD7,
0XFA,0XA6,0XBA,0XC7,0X79,0XCF,0XB9,0XCF,0X78,0XCF,0X78,0XCF,0XB8,0XCF,0XB8,0XD7,
0X78,0XD7,0X78,0XD7,0XB7,0XD7,0XBA,0XC7,0XFB,0X9E,0XBB,0XD7,0XBA,0XE7,0XBA,0XDF,
0XFA,0XEF,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XB9,0XCF,0XB8,0XCF,0X79,0XCF,0X78,0XD7,0X78,0XCF,0X78,0XCF,0X78,0XD7,0XB8,0XD7,
0XB8,0XCF,0XB8,0XCF,0XF9,0X9E,0XFA,0XA6,0XFC,0XD7,0XFC,0XD7,0XFB,0XD7,0XFC,0XD7,
0XFC,0XD7,0XFB,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0X79,0XDF,0X79,0XDF,
0XBB,0XD7,0XBB,0XD7,0XBB,0XD7,0XFC,0XD7,0XFC,0XD7,0XFC,0XD7,0XFC,0XD7,0XFC,0XDF,
0XFC,0XD7,0XFC,0XD7,0XFB,0XDF,0XBA,0XDF,0XF9,0XAE,0XF8,0XA6,0XB8,0XCF,0X78,0XC7,
0X78,0XCF,0XB8,0XCF,0XB8,0XCF,0XB9,0XCF,0XB8,0XD7,0X78,0XCF,0XB8,0XCF,0X78,0XCF,
0XF8,0XA6,0XB9,0X9E,0XFC,0XD7,0XFB,0XD7,0XFB,0XD7,0XFC,0XDF,0XFC,0XE7,0XFA,0XDF,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,
0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0X7A,0XCF,0XBB,0XD7,0XBB,0XD7,0XFB,0XDF,0XFC,0XD7,
0XFC,0XDF,0XFC,0XD7,0XFC,0XD7,0XFC,0XDF,0XFC,0XDF,0XFC,0XDF,0XBA,0XDF,0XFB,0XD7,
0XF9,0XA6,0XB9,0XCF,0XB8,0XCF,0XB9,0XCF,0X78,0XD7,0XB9,0XD7,0X78,0XCF,0XB8,0XD7,
0XB8,0XD7,0XB9,0XCF,0X78,0XD7,0XBA,0XC7,0XFB,0X96,0XFC,0XCF,0XFC,0XD7,0XFC,0XD7,
0XFD,0XDF,0XFB,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XB9,0XE7,0X79,0XDF,0X79,0XDF,
0X7A,0XD7,0XBB,0XD7,0X7B,0XD7,0XFC,0XDF,0XFC,0XDF,0XFC,0XD7,0XFC,0XDF,0XFC,0XDF,
0XFC,0XD7,0XFC,0XDF,0XFC,0XD7,0XBA,0XDF,0XB9,0XA6,0XF9,0XA6,0X78,0XCF,0XB8,0XCF,
0XB8,0XC7,0XB8,0XCF,0XB8,0XCF,0XB8,0XC7,0XB8,0XD7,0X78,0XD7,0XB8,0XCF,0X78,0XCF,
0XF8,0XA6,0XFA,0X9E,0XFC,0XCF,0XFC,0XD7,0XFC,0XD7,0XFB,0XD7,0XFC,0XE7,0XFA,0XE7,
0XFB,0XE7,0XFA,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,
0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0X7A,0XDF,0XBB,0XD7,0XBA,0XD7,0XBB,0XD7,0XBC,0XD7,
0XFC,0XDF,0XFC,0XD7,0XFC,0XD7,0XFC,0XD7,0XFC,0XDF,0XFC,0XD7,0XBA,0XDF,0XFB,0XD7,
0XFA,0X9E,0XBA,0XC7,0XB9,0XCF,0XB9,0XC7,0XB8,0XD7,0X78,0XD7,0X78,0XCF,0XB9,0XD7,
0X78,0XD7,0X79,0XCF,0X79,0XCF,0XBA,0XC7,0XFA,0X9E,0XFC,0XC7,0XFC,0XD7,0XFC,0XD7,
0XFC,0XDF,0XFC,0XDF,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0X77,0XCF,0X38,0XB7,0X37,0XBF,0XB8,0XD7,0X38,0XBF,0X78,0XBF,0X78,0XD7,0X38,0XBF,
0X78,0XBF,0X78,0XCF,0XF9,0X9E,0XFA,0X9E,0XF9,0X9E,0XFA,0X96,0XFA,0X9E,0XFA,0X9E,
0XFA,0XA6,0XF9,0XAE,0XFA,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,
0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0X79,0XDF,0X79,0XDF,0X79,0XDF,0XB9,0XDF,0X79,0XE7,
0XFA,0X9E,0XFA,0XA6,0XFA,0XA6,0XF9,0XA6,0XF9,0XA6,0XFA,0XA6,0XFA,0XA6,0XFA,0XA6,
0X3A,0XA7,0XFA,0XA6,0XB8,0XAE,0XBA,0XDF,0XF8,0XA6,0XF8,0XA6,0X78,0XCF,0X38,0XBF,
0X37,0XB7,0X78,0XD7,0X38,0XC7,0X38,0XC7,0X78,0XD7,0X38,0XC7,0X38,0XC7,0X78,0XCF,
0XF8,0XA6,0XFA,0X9E,0XFA,0X96,0XFA,0X9E,0XFA,0X9E,0XFA,0XA6,0XFC,0XDF,0XFA,0XDF,
0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0XB9,0XDF,
0X79,0XDF,0X79,0XDF,0XBA,0XDF,0XF8,0XAE,0XFA,0X9E,0XFA,0XA6,0XFA,0XA6,0XFA,0XA6,
0XFA,0XA6,0XFA,0XA6,0XFA,0XA6,0XFA,0XA6,0XFB,0XA6,0XF9,0XA6,0XBB,0XD7,0XBB,0XCF,
0XF9,0X9E,0XB9,0XCF,0XB9,0XCF,0X77,0XB7,0XB8,0XD7,0XB9,0XCF,0X78,0XBF,0XB8,0XD7,
0XB9,0XD7,0X78,0XBF,0XB8,0XCF,0XBA,0XC7,0XFB,0X9E,0XFA,0X9E,0XFA,0X96,0XFA,0X9E,
0X3A,0XA7,0XF9,0XAE,0XFA,0XE7,0XFB,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,
0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0XBA,0XDF,0X79,0XE7,0X79,0XDF,0XB9,0XDF,
0XB8,0XAE,0XFB,0X9E,0XF9,0XA6,0XFA,0XA6,0XFA,0XA6,0XFA,0XA6,0XFA,0XA6,0XFA,0XA6,
0X3A,0XA7,0XFA,0X9E,0XFA,0X9E,0XFA,0XDF,0XB9,0XA6,0XF9,0X9E,0XB8,0XCF,0X37,0XBF,
0X38,0XB7,0X78,0XD7,0X37,0XC7,0X78,0XBF,0X78,0XD7,0X37,0XBF,0X78,0XBF,0X78,0XD7,
0XF8,0XA6,0XFB,0X9E,0XFA,0X9E,0XFB,0X9E,0XFA,0X9E,0XFA,0XA6,0XFC,0XD7,0XFA,0XDF,
0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XEF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0X79,0XDF,
0X79,0XDF,0X79,0XE7,0X79,0XDF,0XBB,0XD7,0XFA,0X9E,0XF9,0XA6,0XFA,0XA6,0XFA,0XA6,
0XFA,0XA6,0XFA,0XA6,0XF9,0XA6,0X39,0XA7,0XFA,0XA6,0XFA,0X9E,0XFB,0XDF,0XFC,0XD7,
0XFA,0X9E,0XBA,0XC7,0XB9,0XCF,0X38,0XB7,0XB9,0XCF,0XB9,0XD7,0X38,0XBF,0XB9,0XCF,
0XB9,0XD7,0X78,0XBF,0X78,0XC7,0XBA,0XCF,0XFA,0X9E,0XFA,0X9E,0XFA,0XA6,0XFA,0X9E,
0X3B,0XA7,0XF9,0XAE,0XFA,0XDF,0XFA,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,
0XB8,0XD7,0X78,0XBF,0X38,0XC7,0X78,0XD7,0X38,0XBF,0X37,0XC7,0X78,0XD7,0X38,0XC7,
0X38,0XBF,0X78,0XCF,0XF9,0X9E,0XF9,0X9E,0X38,0XAF,0X38,0XAF,0X79,0XC7,0XBA,0XC7,
0XFA,0X9E,0XB9,0XA6,0XBA,0XE7,0XBA,0XDF,0XBA,0XDF,0XFA,0XE7,0XFB,0XE7,0XFA,0XEF,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,
0XB9,0XDF,0X79,0XDF,0X79,0XDF,0X79,0XDF,0X79,0XDF,0XB9,0XDF,0XB9,0XDF,0X79,0XD7,
0XFA,0X9E,0XF9,0XA6,0XBA,0XCF,0XB9,0XCF,0XB8,0XCF,0XB8,0XCF,0XB8,0XCF,0XB9,0XC7,
0X79,0XB7,0XF9,0X9E,0XB9,0XAE,0XBA,0XE7,0XF9,0XA6,0XF8,0XA6,0X78,0XD7,0X37,0XC7,
0X37,0XBF,0X78,0XD7,0X38,0XBF,0X38,0XBF,0X78,0XD7,0X38,0XBF,0X38,0XBF,0XB8,0XCF,
0XF8,0XA6,0XFA,0X9E,0X38,0XAF,0X79,0XC7,0X7A,0XC7,0XFA,0X96,0XFC,0XD7,0XBA,0XE7,
0XBA,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0X79,0XDF,0X79,0XDF,0X79,0XDF,
0X79,0XDF,0X79,0XDF,0X79,0XD7,0XB8,0XA6,0XFA,0X9E,0XBB,0XC7,0XB9,0XCF,0XB8,0XCF,
0XB8,0XCF,0XB8,0XCF,0XB8,0XCF,0X78,0XBF,0X79,0XAF,0XF9,0XA6,0XFB,0XDF,0XFC,0XD7,
0XF9,0XA6,0XB9,0XCF,0XB9,0XCF,0X38,0XBF,0X78,0XD7,0XB9,0XCF,0X38,0XBF,0XB8,0XD7,
0X78,0XCF,0X78,0XBF,0X78,0XCF,0XBA,0XC7,0XFB,0X96,0X39,0XAF,0X38,0XBF,0X7A,0XCF,
0XF9,0X9E,0XF9,0XAE,0XBA,0XE7,0XBA,0XDF,0XBA,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,
0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0X79,0XDF,0X7A,0XDF,0X79,0XDF,0X79,0XDF,0XB9,0XD7,
0XB8,0XA6,0XBA,0X9E,0XB9,0XCF,0XB9,0XCF,0XB8,0XCF,0XB9,0XCF,0X78,0XCF,0XB9,0XCF,
0X79,0XB7,0XF9,0XA6,0XFA,0X9E,0XBA,0XDF,0XB9,0XA6,0XFA,0X9E,0XB8,0XD7,0X37,0XBF,
0X78,0XBF,0X78,0XD7,0X38,0XC7,0X78,0XBF,0X78,0XD7,0X38,0XBF,0X38,0XBF,0XB8,0XD7,
0XF8,0XA6,0XFA,0X96,0X38,0XAF,0XB9,0XC7,0X7A,0XCF,0XFA,0X9E,0XFC,0XD7,0XBA,0XDF,
0XBA,0XE7,0XBA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0X79,0XDF,0X79,0XDF,
0X79,0XDF,0X79,0XE7,0X7A,0XD7,0XBB,0XCF,0XFA,0X9E,0X7A,0XCF,0XB9,0XCF,0XB8,0XCF,
0XB8,0XCF,0XB8,0XCF,0XB8,0XCF,0X78,0XB7,0X7A,0XB7,0XFA,0X9E,0XFA,0XD7,0XFC,0XD7,
0XFA,0X9E,0XBA,0XC7,0XB9,0XCF,0X78,0XBF,0XB9,0XCF,0XB9,0XD7,0X38,0XBF,0XB9,0XD7,
0XB9,0XD7,0X79,0XC7,0XB8,0XCF,0XBA,0XC7,0XFA,0X9E,0X39,0XAF,0X38,0XB7,0X79,0XCF,
0XF9,0XA6,0XF9,0X9E,0XBA,0XE7,0XFA,0XDF,0XFA,0XDF,0XFA,0XDF,0XB9,0XDF,0XFA,0XDF,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,
0X78,0XD7,0X37,0XC7,0X38,0XC7,0XB8,0XD7,0X37,0XC7,0X37,0XC7,0X78,0XD7,0X37,0XC7,
0X37,0XC7,0X78,0XCF,0XF9,0XA6,0XFA,0X9E,0X39,0XA7,0X39,0XAF,0XBB,0XC7,0X7B,0XC7,
0XFA,0X9E,0XFA,0XA6,0XFB,0XDF,0XFB,0XD7,0XFB,0XD7,0XFC,0XD7,0XFC,0XDF,0XFA,0XDF,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,
0XB9,0XDF,0X79,0XDF,0X79,0XDF,0X79,0XDF,0X79,0XDF,0X79,0XDF,0XB9,0XDF,0X79,0XD7,
0XFA,0X9E,0XF9,0XA6,0X7A,0XD7,0XB9,0XC7,0X79,0XCF,0X78,0XCF,0X78,0XCF,0XB9,0XCF,
0X38,0XBF,0XF9,0XA6,0XB8,0XA6,0XBA,0XE7,0XF9,0XA6,0XF8,0XA6,0X78,0XD7,0X37,0XC7,
0X37,0XC7,0XB8,0XD7,0X37,0XC7,0X38,0XC7,0X78,0XD7,0X37,0XC7,0X77,0XC7,0XB7,0XCF,
0XF8,0XA6,0XFA,0X9E,0X39,0XA7,0X7B,0XC7,0X7C,0XBF,0XFA,0X9E,0XFD,0XCF,0XFB,0XD7,
0XFC,0XDF,0XFC,0XDF,0XFC,0XD7,0XFC,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XB9,0XDF,0XBA,0XDF,0XB9,0XDF,0X79,0XDF,0X79,0XDF,0XB9,0XDF,0X79,0XDF,
0X79,0XDF,0X79,0XDF,0X7A,0XD7,0XB9,0XA6,0XF9,0X9E,0XBA,0XCF,0X78,0XCF,0X78,0XCF,
0XB8,0XCF,0X78,0XD7,0X79,0XCF,0X38,0XBF,0X3A,0XAF,0XF9,0X9E,0XFB,0XD7,0XFC,0XD7,
0XFA,0X9E,0XB9,0XD7,0XB8,0XD7,0X38,0XC7,0X78,0XD7,0X78,0XD7,0X38,0XC7,0X78,0XD7,
0X78,0XD7,0X38,0XC7,0X78,0XCF,0XBA,0XC7,0XFB,0X9E,0X39,0XA7,0X38,0XAF,0X7B,0XCF,
0XFA,0X9E,0XF9,0XA6,0XFC,0XD7,0XFC,0XD7,0XFC,0XD7,0XFC,0XD7,0XFC,0XD7,0XFB,0XDF,
0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,
0XB9,0XE7,0X79,0XDF,0X79,0XDF,0XB9,0XDF,0X79,0XDF,0X79,0XDF,0XB9,0XDF,0X79,0XD7,
0XB9,0XA6,0XFB,0X9E,0XB9,0XD7,0X79,0XCF,0XB8,0XC7,0X78,0XCF,0X78,0XCF,0XB8,0XCF,
0X78,0XB7,0XF9,0XA6,0XFA,0X9E,0XBA,0XE7,0XB8,0XA6,0XF9,0X9E,0XB8,0XD7,0X77,0XC7,
0X38,0XC7,0X78,0XD7,0X37,0XC7,0X37,0XBF,0XB7,0XD7,0X37,0XC7,0X38,0XC7,0XB8,0XCF,
0XB8,0XA6,0XFA,0X9E,0X3A,0XA7,0XBB,0XC7,0X7B,0XC7,0XFA,0X96,0XFC,0XCF,0XFC,0XD7,
0XFB,0XD7,0XFB,0XD7,0XFC,0XD7,0XFD,0XDF,0XFB,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0X79,0XDF,0X79,0XDF,
0X79,0XDF,0XB8,0XDF,0X79,0XD7,0XBB,0XCF,0XFB,0X9E,0X79,0XCF,0XB9,0XCF,0X79,0XCF,
0XB9,0XCF,0X78,0XCF,0X78,0XCF,0X37,0XB7,0X7A,0XAF,0XFA,0X9E,0XBA,0XD7,0XFC,0XD7,
0XFA,0X9E,0XBA,0XCF,0X79,0XCF,0X38,0XC7,0X78,0XD7,0X78,0XD7,0X37,0XC7,0XB8,0XD7,
0X78,0XCF,0X38,0XC7,0XB8,0XCF,0XBA,0XC7,0XFA,0X9E,0X39,0XA7,0X39,0XAF,0X7A,0XCF,
0XF9,0XA6,0XFA,0X9E,0XFD,0XD7,0XFC,0XD7,0XFC,0XD7,0XFC,0XDF,0XFB,0XD7,0XFC,0XDF,
0XFD,0XDF,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,
0XB9,0XD7,0XB8,0XD7,0X78,0XD7,0X78,0XDF,0XB9,0XD7,0XB8,0XD7,0X78,0XD7,0X78,0XD7,
0X78,0XD7,0X78,0XCF,0XF9,0XA6,0XFA,0X96,0XF9,0X96,0XFA,0X9E,0XF9,0X9E,0XFA,0X9E,
0XFA,0X9E,0XF9,0X9E,0XF9,0XA6,0XFA,0XA6,0XFA,0XA6,0X39,0XA7,0X3B,0XA7,0XF8,0XAE,
0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0X79,0XDF,
0X79,0XDF,0X79,0XDF,0XBA,0XDF,0X7A,0XDF,0XBA,0XDF,0X7A,0XD7,0X7A,0XD7,0X7A,0XCF,
0XF9,0X9E,0XB8,0XA6,0X79,0XCF,0X38,0XB7,0X38,0XBF,0X79,0XCF,0XB9,0XCF,0X79,0XC7,
0X38,0XB7,0XF9,0XA6,0XB8,0XA6,0XBA,0XDF,0XF9,0XA6,0XB8,0XA6,0XB8,0XD7,0XB8,0XD7,
0X78,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0X77,0XD7,0XB8,0XD7,0X77,0XD7,
0XF8,0XA6,0XFB,0X9E,0XFA,0X96,0XFA,0X9E,0XFA,0X9E,0XFA,0X9E,0XF9,0XA6,0XF9,0XA6,
0XFA,0XA6,0XFA,0XA6,0XFA,0XA6,0XFB,0XA6,0XFB,0XDF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0X7A,0XE7,0XB9,0XDF,0X79,0XDF,0XB9,0XDF,0XBA,0XD7,
0X7A,0XD7,0XB9,0XCF,0X7A,0XD7,0XF8,0XA6,0XF9,0X9E,0X7A,0XC7,0X38,0XBF,0X38,0XB7,
0XB9,0XC7,0XB8,0XC7,0X79,0XC7,0X37,0XB7,0X39,0XAF,0XF9,0XA6,0XFB,0XDF,0XFC,0XD7,
0XF9,0X9E,0XB9,0XCF,0X78,0XD7,0XB8,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0XB8,0XD7,
0X78,0XD7,0XB9,0XD7,0X78,0XD7,0XBB,0XC7,0XFB,0X9E,0XFA,0X96,0XFA,0X96,0XFA,0X9E,
0XFA,0X9E,0XFA,0X9E,0XFA,0XA6,0XF9,0XA6,0XF9,0XA6,0X3A,0XA7,0XFA,0XA6,0XF8,0XAE,
0XBA,0XEF,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,0XFB,0XEF,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,
0X79,0XDF,0X79,0XDF,0XBA,0XE7,0XBA,0XDF,0XBA,0XDF,0X7A,0XD7,0X79,0XD7,0X7A,0XD7,
0XF9,0XA6,0XFA,0X9E,0X79,0XCF,0X38,0XB7,0X78,0XAF,0XB9,0XCF,0X78,0XC7,0X79,0XC7,
0X39,0XB7,0XF9,0X9E,0XF9,0X9E,0XBA,0XDF,0XB8,0XA6,0XFA,0X9E,0XB8,0XD7,0XB8,0XD7,
0X78,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0XB8,0XD7,0XB9,0XD7,0X78,0XCF,
0XF8,0XA6,0XFB,0X9E,0XF9,0X96,0XFA,0X9E,0XFA,0XA6,0XFA,0X9E,0XFA,0XA6,0XF9,0XA6,
0XF9,0XA6,0XFA,0XA6,0X39,0XA7,0X3B,0XA7,0XF9,0XAE,0XFA,0XE7,0XFA,0XE7,0XFA,0XE7,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0XB9,0XDF,0X79,0XE7,0XB9,0XDF,0XBA,0XDF,
0XBA,0XD7,0XBA,0XCF,0X79,0XD7,0XBB,0XC7,0XFA,0X9E,0X79,0XC7,0X79,0XCF,0X39,0XAF,
0X79,0XCF,0XB8,0XC7,0X79,0XCF,0X38,0XB7,0X39,0XAF,0XFA,0X9E,0XBB,0XDF,0XFC,0XD7,
0XFA,0X9E,0XBA,0XCF,0X78,0XD7,0XB9,0XD7,0X78,0XD7,0X78,0XD7,0XB8,0XD7,0XB8,0XD7,
0X78,0XD7,0XB8,0XD7,0XB8,0XCF,0XBA,0XC7,0XFA,0X9E,0XFA,0X9E,0XFA,0X9E,0XFA,0X9E,
0XFA,0X9E,0XFA,0X9E,0XF9,0XA6,0XFA,0XA6,0XFA,0XAE,0XF9,0XA6,0X3A,0XA7,0XFA,0XA6,
0X3B,0X9F,0XFC,0XDF,0XFA,0XE7,0XFB,0XE7,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,
0XB8,0XD7,0X37,0XBF,0X37,0XC7,0XB8,0XD7,0X37,0XBF,0X77,0XC7,0XB8,0XD7,0X37,0XC7,
0X78,0XBF,0XB8,0XCF,0XF9,0X9E,0XFA,0XA6,0X38,0XAF,0X39,0XAF,0XB9,0XC7,0X79,0XCF,
0X79,0XC7,0XB9,0XCF,0X79,0XD7,0XB8,0XC7,0XB8,0XCF,0XB9,0XCF,0XB9,0XA6,0XF8,0XAE,
0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0X79,0XDF,
0X79,0XDF,0X79,0XDF,0X79,0XDF,0XBB,0XD7,0XF9,0XAE,0XFA,0XA6,0XFA,0XA6,0XFA,0X9E,
0XFB,0X96,0XFA,0X96,0XF9,0X9E,0XFA,0X9E,0XFA,0X9E,0XB9,0X9E,0XF9,0X9E,0XFA,0X9E,
0XF9,0X9E,0XFA,0X9E,0XB8,0XA6,0XBA,0XE7,0XF9,0XA6,0XF8,0XA6,0XB8,0XD7,0X37,0XC7,
0X37,0XC7,0XB9,0XD7,0X38,0XC7,0X38,0XBF,0XB9,0XD7,0X38,0XC7,0X38,0XBF,0X78,0XCF,
0XF8,0XA6,0XFA,0X9E,0X39,0XAF,0XBA,0XC7,0X79,0XC7,0XB9,0XC7,0X79,0XD7,0X79,0XCF,
0XB8,0XC7,0XB8,0XCF,0XB9,0XC7,0XFA,0X9E,0XBA,0XD7,0X79,0XDF,0XBA,0XDF,0XFB,0XE7,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0X79,0XE7,0XB9,0XDF,0X79,0XDF,0XB9,0XDF,0XF9,0XAE,
0XFA,0X9E,0XFA,0X9E,0XFA,0X9E,0XFB,0X9E,0XFA,0X9E,0XF9,0X9E,0XF9,0X9E,0XFA,0X9E,
0XFA,0X9E,0XFA,0X9E,0XFA,0XA6,0XFA,0X9E,0XFA,0X9E,0XF9,0XA6,0XFB,0XD7,0XFC,0XD7,
0XFA,0X9E,0XB9,0XCF,0XB9,0XCF,0X38,0XBF,0XB9,0XD7,0XB9,0XCF,0X38,0XBF,0X78,0XD7,
0XB8,0XD7,0X38,0XBF,0X78,0XCF,0XBA,0XC7,0XFA,0X96,0X39,0XAF,0X38,0XB7,0XB9,0XCF,
0XB9,0XCF,0XB9,0XCF,0X79,0XD7,0X79,0XCF,0X79,0XCF,0XB9,0XCF,0XF9,0XA6,0XF8,0XAE,
0X79,0XE7,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,
0X79,0XDF,0X79,0XDF,0X79,0XDF,0X7A,0XD7,0XFA,0XA6,0XFA,0XA6,0XFA,0X9E,0XFA,0X9E,
0XFA,0X96,0XFB,0X9E,0XF9,0X9E,0XFA,0X9E,0XFA,0X9E,0XB9,0X9E,0XFA,0X9E,0XFA,0XA6,
0XF9,0X9E,0XFA,0X96,0XFA,0X9E,0XBA,0XDF,0XB8,0XA6,0XFA,0X9E,0XB8,0XD7,0X37,0XC7,
0X38,0XBF,0XB9,0XD7,0X37,0XC7,0X38,0XBF,0XB8,0XD7,0X37,0XC7,0X37,0XBF,0XB8,0XCF,
0XF8,0XA6,0XFA,0X9E,0X38,0XB7,0XB9,0XCF,0X79,0XCF,0XB9,0XC7,0XB9,0XCF,0X79,0XD7,
0X79,0XCF,0XB9,0XC7,0X79,0XCF,0XFA,0X9E,0XF8,0XAE,0XB9,0XDF,0XBA,0XDF,0XBA,0XDF,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0XB9,0XDF,0X79,0XE7,0XB9,0XDF,0XB9,0XAE,
0XFA,0XA6,0XFA,0X9E,0XFA,0XA6,0XFA,0X9E,0XFB,0X96,0XFA,0XA6,0XFA,0X9E,0XFA,0X9E,
0XFA,0X9E,0XFA,0X9E,0XFA,0X9E,0XF9,0X9E,0XFA,0X9E,0XFB,0X96,0XBA,0XDF,0XFC,0XD7,
0XFA,0X9E,0XBA,0XCF,0XB9,0XCF,0X37,0XC7,0XB8,0XD7,0X79,0XD7,0X37,0XBF,0XB8,0XCF,
0X78,0XCF,0X37,0XC7,0X78,0XCF,0XBA,0XC7,0XFA,0X9E,0X3A,0XAF,0X38,0XB7,0X79,0XCF,
0X79,0XCF,0X79,0XC7,0X79,0XD7,0X79,0XD7,0X79,0XD7,0X79,0XCF,0X79,0XC7,0X79,0XCF,
0XFA,0X9E,0XBB,0XD7,0XB9,0XDF,0XBA,0XE7,0XFB,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,
0X78,0XD7,0X38,0XBF,0X38,0XC7,0X78,0XD7,0X38,0XC7,0X38,0XC7,0X78,0XD7,0X38,0XC7,
0X38,0XBF,0X78,0XCF,0XF9,0X9E,0XF9,0XA6,0X37,0XB7,0X38,0XB7,0XB8,0XC7,0X78,0XCF,
0X78,0XCF,0X78,0XCF,0X78,0XD7,0XB8,0XCF,0X78,0XCF,0XB9,0XCF,0XFA,0XA6,0XF8,0XAE,
0X79,0XDF,0XB9,0XDF,0XBA,0XDF,0XB9,0XDF,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0X79,0XDF,
0X79,0XDF,0X79,0XDF,0XB9,0XE7,0XBC,0XD7,0XFA,0X9E,0XF9,0XA6,0XF8,0XA6,0XF8,0XA6,
0XB8,0XA6,0XF8,0XA6,0XF8,0X9E,0XF8,0XA6,0XF8,0XA6,0XF8,0XA6,0XF9,0X9E,0XF9,0X9E,
0XF9,0XA6,0XFA,0X9E,0XF8,0XA6,0XBA,0XDF,0XF9,0XA6,0XF8,0XA6,0X78,0XD7,0X38,0XBF,
0X78,0XBF,0XB9,0XD7,0X37,0XBF,0X38,0XC7,0X78,0XDF,0X38,0XBF,0X37,0XC7,0X78,0XCF,
0XF8,0XA6,0XFA,0XA6,0X38,0XB7,0XB8,0XCF,0XB8,0XCF,0X78,0XCF,0X78,0XDF,0X78,0XD7,
0X78,0XD7,0X78,0XD7,0X79,0XCF,0XFA,0XA6,0XBA,0XD7,0XBA,0XDF,0XBA,0XE7,0XFB,0XE7,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0X79,0XE7,0XB9,0XDF,0X79,0XDF,0XBA,0XDF,0XF9,0XA6,
0XF9,0XA6,0XF8,0XA6,0XF8,0XA6,0XF8,0XA6,0XF8,0XA6,0XF8,0X9E,0XF8,0XA6,0XF9,0XA6,
0XB9,0XA6,0XF8,0X9E,0XF9,0X9E,0XF9,0XA6,0XF9,0X9E,0XF9,0X9E,0XFB,0XD7,0XFC,0XD7,
0XFA,0X9E,0XB9,0XCF,0XB8,0XCF,0X38,0XBF,0XB8,0XD7,0XB9,0XCF,0X37,0XBF,0X78,0XCF,
0XB9,0XCF,0X78,0XBF,0X78,0XCF,0XBA,0XC7,0XFB,0X96,0X38,0XAF,0X37,0XB7,0X78,0XCF,
0XB8,0XD7,0X78,0XD7,0XB8,0XD7,0X78,0XD7,0X78,0XCF,0XB8,0XCF,0XF9,0XA6,0XF9,0XAE,
0X79,0XE7,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,
0X79,0XDF,0X79,0XDF,0X79,0XDF,0XBA,0XD7,0XFA,0X9E,0XF9,0XA6,0XF8,0XA6,0XF8,0XA6,
0XF8,0XA6,0XF7,0XA6,0XF8,0X9E,0XF8,0XA6,0XF8,0X9E,0XF8,0XA6,0XF9,0XA6,0XF9,0XA6,
0XF9,0XA6,0XF9,0X9E,0XFA,0X9E,0XFA,0XDF,0XB8,0XA6,0XFA,0X9E,0XB8,0XD7,0X37,0XC7,
0X39,0XBF,0X78,0XD7,0X38,0XC7,0X38,0XBF,0XB8,0XD7,0X38,0XC7,0X38,0XBF,0XB8,0XCF,
0XB8,0XA6,0XFA,0X96,0X38,0XB7,0XB8,0XCF,0XB8,0XCF,0X78,0XCF,0XB8,0XD7,0X78,0XD7,
0XB8,0XD7,0XB8,0XCF,0XB8,0XCF,0XFA,0XA6,0XF8,0XB6,0X79,0XDF,0XBA,0XDF,0XBA,0XE7,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0XB9,0XDF,0X79,0XDF,0XB9,0XDF,0XFA,0XA6,
0XFA,0XA6,0XF8,0XA6,0XB8,0XA6,0XF8,0XA6,0XB8,0XA6,0XF8,0XA6,0XF8,0X9E,0XB8,0XA6,
0XF8,0XA6,0XF9,0XA6,0XF9,0XA6,0XF9,0XA6,0XF9,0XA6,0XFA,0X9E,0XBB,0XD7,0XFC,0XD7,
0XFA,0X9E,0XBA,0XCF,0XB9,0XCF,0X38,0XBF,0XB9,0XD7,0XB9,0XD7,0X38,0XBF,0XB9,0XCF,
0XB9,0XCF,0X79,0XBF,0X78,0XCF,0XBA,0XC7,0XFA,0X9E,0X39,0XAF,0X37,0XB7,0X78,0XCF,
0X77,0XCF,0X78,0XCF,0X78,0XD7,0X78,0XD7,0X78,0XD7,0XB8,0XD7,0XB9,0XD7,0X79,0XCF,
0XFA,0XA6,0XBB,0XD7,0XB9,0XDF,0XBB,0XE7,0XFB,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,
0XB8,0XD7,0X37,0XBF,0X38,0XC7,0XB8,0XD7,0X38,0XBF,0X37,0XC7,0XB8,0XD7,0X37,0XC7,
0X38,0XBF,0X78,0XCF,0XF9,0X9E,0XF9,0XA6,0X38,0XB7,0X38,0XB7,0XB8,0XCF,0X78,0XCF,
0X78,0XD7,0X78,0XD7,0X78,0XD7,0XB8,0XD7,0XB8,0XD7,0XB8,0XCF,0XFA,0XA6,0XB8,0XAE,
0X79,0XDF,0X79,0XDF,0XB9,0XDF,0XB9,0XDF,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0X79,0XDF,
0X79,0XDF,0X79,0XDF,0X79,0XDF,0XBB,0XD7,0XF9,0XA6,0XB9,0XCF,0XB8,0XCF,0X78,0XCF,
0X78,0XCF,0X78,0XCF,0X78,0XCF,0XB9,0XCF,0XB8,0XCF,0XB8,0XCF,0X38,0XB7,0X38,0XB7,
0X38,0XB7,0XF9,0XA6,0XF9,0XA6,0XBB,0XDF,0XF9,0XA6,0XF8,0XA6,0XB8,0XD7,0X38,0XC7,
0X37,0XC7,0X78,0XD7,0X37,0XBF,0X38,0XBF,0XB8,0XD7,0X38,0XBF,0X78,0XBF,0X77,0XCF,
0XF8,0XA6,0XFA,0X9E,0X79,0XB7,0XB9,0XCF,0X77,0XCF,0X78,0XD7,0XB8,0XD7,0X78,0XD7,
0X78,0XD7,0X78,0XCF,0XB9,0XCF,0XFA,0XA6,0XBB,0XD7,0XBA,0XDF,0X79,0XDF,0XFA,0XE7,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0X79,0XE7,0X79,0XDF,0X79,0XDF,0XBA,0XDF,0XF9,0XA6,
0X7A,0XC7,0X78,0XD7,0X78,0XCF,0X78,0XCF,0X78,0XCF,0X78,0XCF,0XB8,0XCF,0XB8,0XCF,
0XB8,0XCF,0X78,0XCF,0X39,0XB7,0X38,0XB7,0X39,0XAF,0XFA,0X9E,0XFB,0XD7,0XFC,0XD7,
0XFA,0X9E,0XB9,0XD7,0XB9,0XCF,0X38,0XBF,0X78,0XD7,0X79,0XCF,0X37,0XBF,0X78,0XCF,
0XB9,0XD7,0X38,0XBF,0X78,0XCF,0XBA,0XC7,0XFA,0X9E,0X39,0XB7,0X38,0XB7,0XB8,0XCF,
0XB9,0XD7,0XB8,0XD7,0XB8,0XD7,0X78,0XD7,0XB8,0XD7,0XB8,0XCF,0XF9,0XA6,0XB9,0XAE,
0X79,0XE7,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,
0X79,0XDF,0X79,0XDF,0XB9,0XDF,0XBB,0XD7,0XFA,0XA6,0XBA,0XCF,0XB8,0XCF,0X78,0XCF,
0XB8,0XCF,0X78,0XD7,0X78,0XCF,0X78,0XCF,0XB8,0XD7,0XB8,0XCF,0X37,0XBF,0X38,0XB7,
0X38,0XB7,0XF9,0XA6,0XFA,0X9E,0XBA,0XDF,0XB8,0XA6,0XFA,0X9E,0XB8,0XD7,0X37,0XC7,
0X38,0XBF,0X78,0XD7,0X37,0XBF,0X38,0XBF,0XB7,0XD7,0X38,0XC7,0X38,0XBF,0XB8,0XCF,
0XF8,0XA6,0XFA,0X9E,0X38,0XB7,0XB9,0XC7,0X78,0XCF,0X78,0XD7,0X78,0XD7,0XB8,0XD7,
0X78,0XD7,0XB8,0XD7,0XB8,0XCF,0XFA,0XA6,0XF9,0XAE,0X79,0XDF,0XB9,0XDF,0XB9,0XDF,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0XB9,0XDF,0X79,0XDF,0XBA,0XDF,0XFA,0XA6,
0XF8,0XAE,0XB8,0XCF,0XB8,0XCF,0XB8,0XCF,0XB8,0XCF,0X78,0XCF,0XB8,0XCF,0XB8,0XCF,
0XB8,0XCF,0XB9,0XC7,0X37,0XB7,0X38,0XB7,0X39,0XB7,0XFA,0X9E,0XBB,0XD7,0XFC,0XCF,
0XFA,0X9E,0XBA,0XCF,0XB9,0XCF,0X37,0XBF,0XB9,0XD7,0XB9,0XD7,0X38,0XBF,0XB9,0XCF,
0XB8,0XD7,0X78,0XBF,0XB8,0XCF,0XBA,0XC7,0XFA,0X9E,0X79,0XB7,0X38,0XBF,0X78,0XCF,
0XB8,0XCF,0XB9,0XD7,0XB8,0XD7,0XB8,0XD7,0XB8,0XD7,0X78,0XD7,0X78,0XD7,0X79,0XCF,
0XFA,0XA6,0XBB,0XD7,0XB9,0XDF,0X7A,0XDF,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,
0X78,0XD7,0XB8,0XD7,0X78,0XD7,0X78,0XDF,0X78,0XD7,0XB8,0XD7,0X78,0XD7,0XB9,0XD7,
0XB8,0XD7,0X78,0XCF,0XF9,0X9E,0XF9,0X9E,0X78,0XB7,0X38,0XB7,0XB8,0XCF,0X78,0XD7,
0X38,0XBF,0X38,0XBF,0X78,0XD7,0X38,0XBF,0X37,0XC7,0XB9,0XCF,0XFA,0XA6,0XF8,0XA6,
0X79,0XDF,0X79,0XDF,0XB9,0XDF,0XB9,0XDF,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0X79,0XDF,
0X79,0XDF,0X79,0XDF,0X79,0XE7,0XBB,0XD7,0XFB,0X9E,0X38,0XB7,0X38,0XBF,0X78,0XCF,
0X38,0XBF,0X38,0XC7,0XB8,0XDF,0X38,0XB7,0X37,0XBF,0XB8,0XCF,0X38,0XB7,0X38,0XAF,
0X38,0XAF,0XF9,0X9E,0XB8,0XA6,0X79,0XCF,0XF9,0XA6,0XB8,0XA6,0XB8,0XD7,0XB8,0XD7,
0X78,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0X77,0XD7,0XB8,0XD7,0X78,0XCF,
0XF8,0XA6,0XFA,0X9E,0X79,0XB7,0XB9,0XCF,0X78,0XCF,0X37,0XBF,0XB9,0XCF,0X78,0XCF,
0X38,0XBF,0X78,0XCF,0X79,0XD7,0XFA,0XA6,0XBA,0XD7,0X79,0XE7,0X79,0XDF,0XFA,0XE7,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0X79,0XE7,0XB9,0XDF,0X79,0XDF,0XBA,0XDF,0XF9,0X9E,
0X39,0XB7,0X38,0XB7,0XB9,0XCF,0X37,0XBF,0X37,0XBF,0XB8,0XD7,0X37,0XBF,0X38,0XBF,
0XB9,0XCF,0X79,0XCF,0X38,0XB7,0X38,0XB7,0X3A,0XA7,0XF9,0X9E,0X7A,0XC7,0XBB,0XC7,
0XFA,0X9E,0XB9,0XD7,0XB8,0XD7,0XB8,0XD7,0X78,0XD7,0X78,0XD7,0XB8,0XD7,0XB8,0XD7,
0XB8,0XD7,0XB9,0XD7,0X78,0XD7,0XBB,0XC7,0XFB,0X96,0X39,0XB7,0X37,0XBF,0XB8,0XCF,
0X37,0XBF,0X38,0XC7,0X78,0XDF,0X37,0XBF,0X37,0XBF,0XB9,0XD7,0XF9,0XA6,0XF9,0XAE,
0X79,0XE7,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,
0X79,0XDF,0X79,0XDF,0X79,0XDF,0XBB,0XD7,0XFA,0X9E,0X7A,0XB7,0X37,0XBF,0XB8,0XCF,
0X37,0XBF,0X38,0XBF,0X78,0XDF,0X37,0XBF,0X78,0XB7,0XB8,0XD7,0X37,0XB7,0X38,0XB7,
0X38,0XAF,0XF8,0X96,0XFA,0X96,0X79,0XCF,0XB9,0XA6,0XFA,0X9E,0XB8,0XD7,0XB8,0XD7,
0X78,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0XB8,0XD7,0X78,0XCF,
0XF8,0XA6,0XFB,0X9E,0X38,0XB7,0XB8,0XCF,0XB9,0XD7,0X38,0XBF,0XB9,0XCF,0X78,0XD7,
0X37,0XC7,0X37,0XBF,0XB9,0XCF,0XFA,0XA6,0XF9,0XAE,0X79,0XDF,0X79,0XDF,0XB9,0XDF,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0XB9,0XDF,0X79,0XDF,0XB9,0XE7,0XFA,0X9E,
0XF9,0X9E,0X38,0XB7,0XB8,0XCF,0XB9,0XCF,0X37,0XBF,0X77,0XD7,0XB9,0XCF,0X37,0XB7,
0X78,0XCF,0XB8,0XC7,0X38,0XB7,0X38,0XAF,0X3A,0XA7,0XFB,0X96,0X79,0XCF,0X7A,0XC7,
0XFA,0X9E,0XBA,0XCF,0XB8,0XD7,0XB9,0XD7,0X78,0XD7,0X78,0XD7,0XB8,0XD7,0XB8,0XD7,
0X78,0XD7,0XB8,0XD7,0XB8,0XD7,0XBA,0XC7,0XFA,0X9E,0X7A,0XAF,0X37,0XBF,0XB9,0XD7,
0X37,0XBF,0X37,0XBF,0X78,0XD7,0X78,0XC7,0X38,0XBF,0X78,0XD7,0X78,0XD7,0XB9,0XCF,
0XFA,0X9E,0XBB,0XD7,0XB9,0XDF,0X79,0XDF,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,
0XB8,0XD7,0XB8,0XD7,0X79,0XD7,0XB8,0XD7,0X78,0XD7,0X78,0XD7,0XB8,0XD7,0XB8,0XD7,
0X78,0XD7,0XB8,0XCF,0XF9,0X9E,0XF9,0XA6,0X78,0XB7,0X38,0XB7,0XB8,0XCF,0XB8,0XD7,
0X38,0XBF,0X78,0XBF,0XB8,0XD7,0X78,0XBF,0X37,0XC7,0X78,0XD7,0XF9,0XA6,0XF8,0XAE,
0X79,0XDF,0X79,0XDF,0XB9,0XDF,0XB9,0XDF,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0X79,0XDF,
0X79,0XDF,0X79,0XDF,0X79,0XDF,0XBB,0XD7,0XFA,0X9E,0X39,0XB7,0X38,0XBF,0X78,0XD7,
0X38,0XBF,0X78,0XC7,0X78,0XD7,0X78,0XBF,0X38,0XB7,0X78,0XD7,0X38,0XB7,0X38,0XB7,
0X37,0XAF,0XF9,0X96,0XF9,0X9E,0X79,0XC7,0XF9,0X9E,0XF8,0XA6,0X78,0XCF,0XB8,0XD7,
0XB8,0XD7,0X78,0XD7,0XB8,0XD7,0X78,0XD7,0XB8,0XD7,0XB8,0XD7,0XB8,0XD7,0X78,0XCF,
0XF8,0XA6,0XFA,0X9E,0X79,0XB7,0XB8,0XCF,0XB8,0XD7,0X38,0XBF,0XB8,0XCF,0X78,0XD7,
0X78,0XBF,0XB9,0XD7,0X79,0XD7,0XFA,0XA6,0XBA,0XD7,0XB9,0XDF,0X79,0XDF,0XFA,0XE7,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0X79,0XE7,0X79,0XDF,0X79,0XDF,0XBA,0XDF,0XF9,0XA6,
0X39,0XB7,0X38,0XBF,0X78,0XCF,0X77,0XC7,0X38,0XBF,0X78,0XD7,0X38,0XBF,0X38,0XBF,
0XB9,0XCF,0XB8,0XC7,0X39,0XB7,0X38,0XAF,0X39,0XA7,0XFA,0X96,0X7A,0XC7,0X7A,0XBF,
0XFA,0X9E,0XB9,0XCF,0XB8,0XD7,0X78,0XD7,0XB9,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,
0XB8,0XD7,0X78,0XD7,0X77,0XCF,0XBA,0XC7,0XFB,0X9E,0X39,0XB7,0X38,0XB7,0XB8,0XD7,
0X38,0XBF,0X37,0XC7,0X78,0XD7,0X38,0XBF,0X37,0XBF,0X78,0XD7,0XF9,0XA6,0XB9,0XAE,
0X79,0XE7,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0X79,0XDF,
0X79,0XDF,0X79,0XDF,0X79,0XDF,0XBB,0XD7,0XFA,0X9E,0X39,0XB7,0X38,0XBF,0X78,0XCF,
0X37,0XC7,0X38,0XBF,0X78,0XD7,0X37,0XB7,0X38,0XBF,0X78,0XCF,0X37,0XB7,0X39,0XB7,
0X38,0XAF,0XF8,0X96,0XFA,0X96,0XB9,0XC7,0XFA,0XA6,0XFA,0X9E,0X78,0XD7,0XB8,0XD7,
0XB8,0XD7,0X78,0XD7,0X78,0XD7,0XB8,0XD7,0XB8,0XD7,0X78,0XD7,0XB8,0XD7,0X78,0XCF,
0XF8,0XA6,0XFA,0X96,0X38,0XB7,0XB8,0XCF,0X79,0XCF,0X38,0XBF,0XB9,0XCF,0XB8,0XD7,
0X38,0XBF,0X37,0XBF,0X78,0XD7,0XFA,0XA6,0XB8,0XB6,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0X79,0XDF,0X79,0XDF,0XBA,0XDF,0XFA,0X9E,
0XF9,0XA6,0X78,0XB7,0XB8,0XCF,0XB9,0XCF,0X78,0XBF,0X78,0XD7,0XB9,0XCF,0X38,0XBF,
0X78,0XCF,0XB9,0XCF,0X38,0XB7,0X38,0XB7,0X3A,0XAF,0XFB,0X96,0X79,0XC7,0XBB,0XC7,
0XFA,0X9E,0XBA,0XCF,0XB8,0XD7,0XB8,0XD7,0XB8,0XDF,0XB8,0XD7,0X78,0XD7,0X78,0XD7,
0XB8,0XD7,0X78,0XD7,0X78,0XCF,0XBA,0XC7,0XFA,0X9E,0X79,0XB7,0X37,0XBF,0X78,0XCF,
0X38,0XBF,0X38,0XBF,0XB8,0XD7,0X37,0XBF,0X38,0XBF,0X78,0XD7,0X78,0XD7,0X79,0XD7,
0XFA,0X9E,0XBB,0XD7,0XB9,0XDF,0X7A,0XDF,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,
0X78,0XD7,0XB8,0XCF,0X78,0XD7,0XB8,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,
0XB8,0XD7,0X77,0XCF,0XF9,0X9E,0XF9,0XA6,0X78,0XB7,0X38,0XB7,0XB8,0XCF,0XB9,0XD7,
0X38,0XC7,0X38,0XC7,0XB8,0XD7,0X78,0XBF,0X37,0XC7,0X79,0XD7,0XFA,0X9E,0XB8,0XAE,
0X79,0XDF,0X79,0XDF,0XB9,0XDF,0XB9,0XDF,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0X79,0XDF,
0X79,0XDF,0X79,0XDF,0X79,0XDF,0XBB,0XD7,0XFA,0X9E,0X78,0XB7,0X78,0XBF,0X78,0XD7,
0X38,0XBF,0X38,0XC7,0X78,0XD7,0X78,0XBF,0X37,0XB7,0X77,0XCF,0X37,0XB7,0X38,0XB7,
0X37,0XB7,0XB9,0X9E,0XF9,0X9E,0X38,0XAF,0XF9,0X9E,0XF8,0XA6,0X78,0XCF,0XB9,0XD7,
0XB8,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XCF,0X78,0XCF,
0XF8,0XA6,0XFA,0X9E,0X79,0XB7,0XB8,0XCF,0X78,0XCF,0X38,0XBF,0XB9,0XD7,0X78,0XD7,
0X78,0XBF,0X78,0XCF,0XB9,0XCF,0XFA,0XA6,0XBA,0XD7,0X79,0XE7,0X79,0XDF,0XFA,0XE7,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0X79,0XE7,0X79,0XDF,0X79,0XDF,0XBA,0XDF,0XF9,0XA6,
0X79,0XB7,0X78,0XB7,0X78,0XD7,0X37,0XBF,0X38,0XBF,0XB8,0XD7,0X37,0XBF,0X37,0XB7,
0XB8,0XCF,0X79,0XCF,0X38,0XB7,0X37,0XB7,0X39,0XAF,0XB9,0X96,0X39,0XAF,0X39,0XAF,
0XF9,0XA6,0XB8,0XC7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,
0XB8,0XD7,0X79,0XD7,0X78,0XCF,0XBA,0XCF,0XFB,0X96,0X39,0XB7,0X38,0XBF,0X78,0XCF,
0X38,0XC7,0X38,0XC7,0X78,0XD7,0X38,0XBF,0X37,0XC7,0XB8,0XD7,0XF9,0XA6,0XB9,0XAE,
0X79,0XE7,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0X79,0XDF,
0X79,0XDF,0X79,0XDF,0X79,0XDF,0XBB,0XD7,0XFA,0X9E,0X39,0XB7,0X37,0XBF,0X78,0XD7,
0X78,0XC7,0X78,0XBF,0X78,0XD7,0X38,0XBF,0X78,0XBF,0XB8,0XCF,0X37,0XBF,0X38,0XB7,
0X38,0XB7,0XB8,0X9E,0XFA,0X96,0X38,0XAF,0XF9,0XA6,0XF9,0X9E,0XB8,0XCF,0X78,0XCF,
0X78,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0XB8,0XD7,0X78,0XD7,0XB8,0XCF,0X78,0XD7,
0XF8,0XA6,0XFA,0X9E,0X38,0XB7,0XB8,0XC7,0XB8,0XCF,0X37,0XBF,0X79,0XCF,0XB8,0XD7,
0X38,0XC7,0X37,0XC7,0X78,0XD7,0XFA,0XA6,0XB8,0XB6,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0X79,0XDF,0X79,0XDF,0XBA,0XDF,0XFA,0XA6,
0XF9,0XA6,0X78,0XB7,0XB8,0XD7,0X78,0XCF,0X38,0XBF,0X78,0XD7,0XB9,0XCF,0X78,0XB7,
0XB7,0XCF,0XB8,0XCF,0X37,0XB7,0X38,0XB7,0X39,0XAF,0XB9,0X96,0X38,0XAF,0X39,0XA7,
0XFA,0X96,0XBA,0XC7,0X79,0XCF,0XB8,0XCF,0X78,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,
0XB8,0XD7,0X78,0XD7,0X78,0XCF,0XBA,0XC7,0XFA,0X9E,0X79,0XB7,0X37,0XB7,0XB8,0XCF,
0X38,0XC7,0X37,0XC7,0X78,0XD7,0X38,0XC7,0X38,0XBF,0X78,0XD7,0X78,0XD7,0X79,0XD7,
0XFA,0X9E,0XBB,0XD7,0XB9,0XDF,0X7A,0XDF,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,
0XB8,0XCF,0X37,0XC7,0X38,0XC7,0X78,0XD7,0X37,0XC7,0X37,0XC7,0X78,0XD7,0X37,0XC7,
0X37,0XC7,0XB8,0XCF,0XF9,0X9E,0XF9,0XA6,0X78,0XB7,0X38,0XB7,0XB8,0XCF,0X78,0XD7,
0XB9,0XD7,0XB8,0XD7,0X78,0XD7,0XB9,0XD7,0XB8,0XD7,0X78,0XD7,0XFA,0X9E,0XB8,0XAE,
0X79,0XDF,0X79,0XDF,0XB9,0XDF,0XB9,0XDF,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0X79,0XDF,
0X79,0XDF,0X79,0XDF,0XB9,0XDF,0XBB,0XD7,0XF9,0XA6,0X79,0XC7,0XB8,0XCF,0XB8,0XD7,
0XB8,0XD7,0XB8,0XD7,0X77,0XD7,0XB9,0XCF,0XBB,0XC7,0XFA,0X9E,0XFA,0X9E,0XFA,0X9E,
0XFA,0X9E,0XFA,0X96,0XFA,0X96,0XFA,0X96,0XFA,0X9E,0XF8,0XA6,0X78,0XCF,0X37,0XC7,
0X37,0XC7,0X78,0XD7,0X37,0XC7,0X37,0XC7,0X78,0XD7,0X37,0XC7,0X37,0XC7,0X78,0XCF,
0XF8,0XA6,0XFA,0X9E,0X79,0XB7,0XB9,0XCF,0X78,0XCF,0X78,0XD7,0X78,0XD7,0X78,0XD7,
0XB8,0XD7,0X78,0XD7,0XB9,0XD7,0XFA,0XA6,0XBA,0XD7,0X79,0XE7,0X79,0XDF,0XFA,0XE7,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0X79,0XE7,0X79,0XDF,0X79,0XDF,0XBA,0XDF,0XF9,0XA6,
0XBA,0XC7,0X78,0XCF,0X78,0XD7,0XB8,0XCF,0X78,0XD7,0X78,0XD7,0X78,0XD7,0XB9,0XC7,
0XF9,0XA6,0XFA,0X9E,0XFA,0X96,0XFA,0X9E,0XFA,0X9E,0XFB,0X96,0XFA,0X96,0XFA,0X96,
0XF9,0X9E,0XB9,0XC7,0XB8,0XCF,0X38,0XBF,0XB8,0XD7,0XB9,0XCF,0X37,0XC7,0XB8,0XD7,
0XB8,0XCF,0X38,0XC7,0XB8,0XCF,0XBA,0XC7,0XFB,0X96,0X39,0XB7,0X37,0XBF,0X78,0XCF,
0XB9,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0XB9,0XD7,0XF9,0XA6,0XB9,0XAE,
0X79,0XE7,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0X79,0XDF,
0X79,0XDF,0X79,0XDF,0X79,0XDF,0XBB,0XD7,0XFA,0XA6,0XBA,0XC7,0XB8,0XCF,0X77,0XD7,
0X78,0XD7,0X78,0XD7,0XB8,0XD7,0XB8,0XCF,0XB8,0XC7,0XFA,0X9E,0XFA,0X9E,0XFA,0X96,
0XFA,0X9E,0XFA,0X96,0XFA,0X8E,0XFA,0X96,0XFA,0X9E,0XFA,0X9E,0XB8,0XCF,0X37,0XC7,
0X38,0XC7,0XB9,0XD7,0X37,0XC7,0X38,0XC7,0XB8,0XD7,0X37,0XC7,0X37,0XBF,0X78,0XCF,
0XF8,0XA6,0XFB,0X9E,0X38,0XB7,0XB8,0XC7,0X78,0XD7,0X78,0XCF,0X78,0XD7,0XB8,0XD7,
0XB9,0XD7,0XB8,0XD7,0X78,0XD7,0XFA,0XA6,0XB8,0XB6,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0X79,0XDF,0XB9,0XDF,0XBA,0XDF,0XFA,0XA6,
0XF9,0XAE,0X78,0XCF,0X78,0XCF,0X78,0XD7,0XB8,0XD7,0X78,0XD7,0XB8,0XD7,0X79,0XD7,
0XF9,0XA6,0XFA,0X9E,0XFB,0X9E,0XFA,0X9E,0XFA,0X9E,0XFB,0X96,0XF9,0X96,0XFA,0X96,
0XFB,0X96,0XBA,0XC7,0X79,0XCF,0X38,0XBF,0XB9,0XD7,0XB8,0XD7,0X38,0XC7,0XB8,0XCF,
0X78,0XD7,0X37,0XC7,0X78,0XCF,0XBA,0XC7,0XFB,0X9E,0X79,0XB7,0X78,0XBF,0X78,0XD7,
0XB8,0XD7,0XB8,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0XB9,0XCF,
0XFA,0X9E,0XBB,0XD7,0XB9,0XDF,0X7A,0XDF,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,
0XB8,0XCF,0X38,0XBF,0X38,0XC7,0X78,0XD7,0X38,0XBF,0X37,0XC7,0X78,0XD7,0X38,0XC7,
0X38,0XBF,0X78,0XCF,0XF9,0X9E,0XF9,0XA6,0X78,0XB7,0X38,0XB7,0XB8,0XCF,0X78,0XD7,
0X79,0XD7,0X78,0XCF,0X78,0XD7,0XB9,0XCF,0X78,0XD7,0XB9,0XD7,0XFA,0XA6,0XB8,0XAE,
0X79,0XDF,0X79,0XDF,0XB9,0XDF,0XB9,0XDF,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0X79,0XDF,
0X79,0XDF,0X79,0XDF,0XB9,0XDF,0XBB,0XD7,0XFA,0XA6,0X79,0XC7,0XB9,0XCF,0X78,0XDF,
0X78,0XCF,0X78,0XCF,0X77,0XD7,0XB8,0XCF,0XB9,0XC7,0XFA,0X96,0XF9,0XA6,0XF8,0XA6,
0XF8,0XA6,0XF8,0X9E,0XF8,0X9E,0XF9,0X9E,0XFA,0X9E,0XF8,0XA6,0X78,0XCF,0X38,0XBF,
0X38,0XBF,0X78,0XD7,0X37,0XBF,0X38,0XBF,0X78,0XD7,0X38,0XBF,0X78,0XBF,0X78,0XCF,
0XF8,0XA6,0XFA,0X9E,0X79,0XB7,0XB9,0XCF,0XB8,0XCF,0X79,0XD7,0XB9,0XD7,0X78,0XD7,
0X79,0XD7,0X78,0XD7,0X79,0XD7,0XFB,0XA6,0XBA,0XD7,0X79,0XE7,0X79,0XDF,0XFA,0XE7,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0X79,0XE7,0X79,0XDF,0X79,0XDF,0XBA,0XDF,0XF9,0XA6,
0X7A,0XC7,0X79,0XCF,0XB8,0XD7,0X78,0XD7,0X78,0XD7,0XB9,0XD7,0X78,0XCF,0XB9,0XC7,
0XFA,0X9E,0XF9,0X9E,0XB8,0XA6,0XF9,0XA6,0XF8,0XA6,0XF9,0XA6,0XF9,0X9E,0XF9,0X9E,
0XF9,0X9E,0XB9,0XCF,0XB8,0XCF,0X38,0XBF,0XB8,0XD7,0XB9,0XCF,0X38,0XBF,0XB8,0XD7,
0XB8,0XCF,0X78,0XBF,0XB8,0XCF,0XBA,0XC7,0XFB,0X96,0X39,0XB7,0X38,0XB7,0X78,0XCF,
0XB9,0XCF,0X78,0XCF,0X78,0XD7,0XB8,0XD7,0X78,0XD7,0XB9,0XD7,0XF9,0XA6,0XB9,0XAE,
0X79,0XE7,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0X79,0XDF,
0X79,0XDF,0X79,0XDF,0X79,0XDF,0XBB,0XD7,0XFA,0X9E,0X7A,0XC7,0XB8,0XCF,0X78,0XD7,
0X78,0XD7,0X78,0XD7,0XB8,0XD7,0XB8,0XCF,0XB9,0XC7,0XFB,0X9E,0XF9,0XA6,0XB8,0XA6,
0XF9,0XA6,0XB8,0XA6,0XF8,0XA6,0XFA,0X96,0XB9,0X9E,0XFA,0X9E,0XB8,0XCF,0X37,0XBF,
0X38,0XBF,0XB9,0XD7,0X38,0XBF,0X38,0XBF,0XB8,0XD7,0X38,0XBF,0X38,0XBF,0XB8,0XCF,
0XF8,0XA6,0XFA,0X9E,0X38,0XB7,0XB8,0XCF,0X78,0XD7,0X78,0XCF,0X78,0XD7,0X77,0XD7,
0XB9,0XD7,0X78,0XCF,0XB8,0XD7,0XFA,0XA6,0XB8,0XB6,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0X79,0XDF,0XB9,0XDF,0XBA,0XDF,0XFA,0XA6,
0XF9,0XAE,0XB9,0XCF,0XB9,0XD7,0XB9,0XD7,0XB8,0XD7,0XB8,0XD7,0XB8,0XD7,0XB9,0XCF,
0XFA,0X9E,0XF9,0X96,0XB8,0XA6,0XF8,0XA6,0XF8,0XA6,0XF8,0XA6,0XF8,0X9E,0XF9,0X9E,
0XFB,0X96,0XBA,0XC7,0XB9,0XCF,0X78,0XBF,0XB9,0XCF,0XB9,0XD7,0X78,0XBF,0XB9,0XCF,
0X79,0XCF,0X78,0XBF,0XB8,0XCF,0XBA,0XC7,0XFB,0X9E,0X79,0XB7,0X37,0XBF,0X78,0XCF,
0X78,0XCF,0XB9,0XD7,0XB8,0XD7,0X78,0XD7,0XB9,0XD7,0X78,0XD7,0X78,0XD7,0XB9,0XD7,
0XFA,0X9E,0XBB,0XD7,0XB9,0XDF,0X7A,0XDF,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,
0XB8,0XCF,0X37,0XC7,0X38,0XC7,0X78,0XD7,0X37,0XBF,0X37,0XC7,0X78,0XD7,0X37,0XC7,
0X38,0XBF,0X78,0XD7,0XF9,0X9E,0XF9,0XA6,0X78,0XB7,0X38,0XB7,0XB9,0XCF,0XB8,0XD7,
0X38,0XC7,0X38,0XC7,0X78,0XCF,0X78,0XBF,0X37,0XBF,0X78,0XD7,0XFA,0XA6,0XB8,0XAE,
0X79,0XDF,0X79,0XDF,0XB9,0XDF,0XB9,0XDF,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0X79,0XDF,
0X79,0XDF,0X79,0XDF,0X79,0XDF,0XBB,0XD7,0XFA,0X9E,0X38,0XB7,0X37,0XBF,0XB8,0XD7,
0X38,0XBF,0X38,0XC7,0X78,0XD7,0X79,0XB7,0X79,0XB7,0XFA,0X9E,0X79,0XCF,0X78,0XCF,
0XB8,0XCF,0X78,0XCF,0X79,0XCF,0X38,0XB7,0XFA,0XA6,0XF8,0XA6,0X78,0XCF,0X38,0XBF,
0X38,0XBF,0X78,0XD7,0X37,0XBF,0X38,0XC7,0X78,0XD7,0X38,0XBF,0X78,0XBF,0X78,0XCF,
0XF8,0XA6,0XF9,0X9E,0X79,0XB7,0XB9,0XCF,0XB8,0XCF,0X78,0XBF,0XB9,0XD7,0X78,0XD7,
0X78,0XBF,0XB9,0XCF,0X79,0XCF,0XFB,0XA6,0XBA,0XD7,0X79,0XE7,0X79,0XDF,0XFA,0XE7,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0X79,0XE7,0X79,0XDF,0X79,0XDF,0XBA,0XDF,0XF9,0XA6,
0X79,0XB7,0X38,0XB7,0X78,0XD7,0X37,0XC7,0X37,0XBF,0XB8,0XD7,0X37,0XB7,0X78,0XB7,
0XF9,0X9E,0XF8,0XA6,0X78,0XCF,0XB8,0XCF,0X78,0XCF,0X78,0XCF,0X38,0XB7,0X39,0XAF,
0XFA,0X9E,0XB9,0XCF,0XB8,0XCF,0X38,0XBF,0X78,0XDF,0XB9,0XCF,0X38,0XBF,0XB8,0XD7,
0XB8,0XD7,0X79,0XBF,0X78,0XCF,0XBA,0XC7,0XFA,0X9E,0X39,0XB7,0X38,0XB7,0X78,0XCF,
0X38,0XBF,0X37,0XC7,0XB8,0XDF,0X79,0XBF,0X38,0XBF,0X78,0XD7,0XF9,0XA6,0XB9,0XAE,
0X79,0XE7,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0X79,0XDF,
0X79,0XDF,0X79,0XDF,0X79,0XDF,0XBB,0XD7,0XFA,0X9E,0X39,0XB7,0X37,0XBF,0X78,0XD7,
0X38,0XBF,0X78,0XBF,0XB8,0XD7,0X38,0XBF,0X78,0XB7,0XFA,0X9E,0X7B,0XBF,0X38,0XCF,
0XB8,0XCF,0X78,0XCF,0X78,0XCF,0X39,0XB7,0XF8,0XA6,0XFA,0XA6,0XB8,0XCF,0X37,0XC7,
0X38,0XBF,0X78,0XD7,0X38,0XC7,0X78,0XBF,0X78,0XD7,0X38,0XC7,0X38,0XBF,0X78,0XCF,
0XF8,0XA6,0XFA,0X9E,0X38,0XB7,0XB8,0XCF,0XB8,0XCF,0X38,0XBF,0XB9,0XCF,0X78,0XD7,
0X38,0XBF,0X37,0XBF,0X78,0XD7,0XFA,0XA6,0XB8,0XB6,0XB9,0XDF,0XB9,0XDF,0XB9,0XDF,
0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0X79,0XDF,0XB9,0XDF,0XBA,0XDF,0XFA,0XA6,
0XF9,0XA6,0X79,0XB7,0XB8,0XD7,0X78,0XD7,0X39,0XB7,0X77,0XD7,0XB8,0XD7,0X38,0XBF,
0XFA,0X9E,0XF8,0XA6,0X79,0XCF,0X79,0XCF,0X78,0XCF,0X78,0XCF,0X38,0XB7,0X39,0XAF,
0XFB,0X9E,0XBA,0XC7,0XB9,0XCF,0X78,0XBF,0XB9,0XCF,0X79,0XD7,0X38,0XBF,0XB9,0XCF,
0X79,0XD7,0X78,0XBF,0XB8,0XCF,0XB9,0XC7,0XFB,0X9E,0X79,0XB7,0X38,0XB7,0X78,0XCF,
0X38,0XBF,0X38,0XBF,0X78,0XD7,0X37,0XBF,0X78,0XBF,0XB8,0XD7,0X78,0XD7,0XB8,0XD7,
0XFA,0X9E,0XBB,0XD7,0XB9,0XDF,0X7A,0XDF,0XFA,0XE7,0XFA,0XE7,0XB9,0XDF,0XB9,0XDF,
0XB8,0XCF,0XB8,0XD7,0X78,0XD7,0X78,0XDF,0X78,0XD7,0X78,0XD7,0XB8,0XD7,0X78,0XD7,
0X78,0XD7,0X78,0XCF,0XF9,0XA6,0XF9,0X9E,0X38,0XB7,0X78,0XB7,0XB9,0XCF,0XB7,0XD7,
0X38,0XBF,0X78,0XBF,0XB8,0XD7,0X78,0XBF,0X77,0XBF,0X78,0XCF,0XFA,0XA6,0XF8,0XAE,
0X79,0XE7,0X7A,0XE7,0XB9,0XDF,0X79,0XDF,0X79,0XDF,0X79,0XDF,0XB9,0XDF,0XB9,0XDF,
0XB9,0XDF,0XB9,0XDF,0XBA,0XDF,0XBB,0XD7,0XF9,0X9E,0X38,0XAF,0X37,0XBF,0XB8,0XDF,
0X38,0XBF,0X37,0XBF,0X77,0XDF,0X78,0XB7,0X7A,0XB7,0XFB,0X9E,0X78,0XCF,0XB9,0XC7,
0X78,0XB7,0XB8,0XCF,0XB9,0XCF,0X37,0XAF,0XF8,0X9E,0XF8,0X9E,0X78,0XD7,0XB8,0XCF,
0X77,0XCF,0XB8,0XD7,0X77,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XCF,0XB8,0XCF,0X78,0XD7,
0XF9,0XA6,0XFA,0X9E,0X37,0XB7,0XB9,0XCF,0XB8,0XCF,0X77,0XB7,0XB8,0XCF,0X78,0XD7,
0X78,0XBF,0X79,0XCF,0X79,0XD7,0XFA,0X9E,0XBA,0XD7,0XBA,0XE7,0XB9,0XE7,0X79,0XDF,
0X79,0XDF,0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0XB9,0XDF,0X79,0XDF,0X79,0XCF,0XFA,0X9E,
0X79,0XAF,0X38,0XB7,0XB8,0XCF,0X37,0XB7,0X37,0XB7,0X78,0XD7,0X38,0XB7,0X39,0XB7,
0XFA,0X9E,0XF8,0XA6,0X78,0XCF,0X36,0XB7,0X37,0XBF,0X78,0XCF,0X37,0XB7,0X39,0XAF,
0XFA,0X9E,0XB9,0XC7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,
0XB8,0XD7,0XB9,0XD7,0XB8,0XCF,0XBA,0XC7,0XFB,0X96,0X39,0XB7,0X38,0XBF,0XB8,0XCF,
0X38,0XBF,0X38,0XBF,0X78,0XD7,0X78,0XBF,0X37,0XBF,0X78,0XD7,0XF9,0XAE,0XF8,0XAE,
0X79,0XE7,0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0X79,0XDF,0X79,0XDF,0XB9,0XDF,0XB9,0XDF,
0X79,0XE7,0XB9,0XDF,0XBA,0XDF,0XBB,0XD7,0XFA,0XA6,0X39,0XAF,0X38,0XBF,0X78,0XD7,
0X37,0XC7,0X78,0XBF,0X78,0XD7,0X37,0XBF,0X79,0XB7,0XFA,0X9E,0XBA,0XC7,0XB8,0XCF,
0X38,0XB7,0XB8,0XC7,0X78,0XD7,0X37,0XA7,0XF8,0X9E,0XF9,0X9E,0XB8,0XCF,0X78,0XCF,
0X78,0XCF,0X78,0XD7,0X78,0XD7,0XB8,0XD7,0X78,0XD7,0X78,0XD7,0XB8,0XD7,0XB7,0XD7,
0XF9,0XA6,0XFA,0X9E,0X37,0XB7,0XB9,0XCF,0XB8,0XCF,0X37,0XB7,0XB9,0XCF,0X78,0XDF,
0X38,0XBF,0X37,0XC7,0XB8,0XD7,0XFA,0XA6,0XB8,0XB6,0X79,0XDF,0XB9,0XDF,0X79,0XDF,
0X79,0XDF,0XB9,0XDF,0X79,0XDF,0X79,0XDF,0XB9,0XDF,0XBA,0XDF,0X79,0XD7,0XFA,0X9E,
0XF9,0X9E,0X37,0XAF,0X78,0XCF,0XB8,0XCF,0X77,0XB7,0XB8,0XCF,0XB9,0XCF,0X38,0XB7,
0XFA,0X9E,0XF8,0XA6,0X78,0XCF,0X77,0XB7,0X37,0XBF,0X78,0XCF,0X37,0XB7,0X38,0XA7,
0XFB,0X96,0XBA,0XC7,0XB8,0XCF,0XB8,0XD7,0XB8,0XD7,0X78,0XD7,0X78,0XD7,0XB8,0XD7,
0X78,0XD7,0XB8,0XD7,0X78,0XCF,0XBA,0XC7,0XFA,0X9E,0X79,0XAF,0X37,0XB7,0X78,0XCF,
0X37,0XBF,0X37,0XBF,0XB8,0XD7,0X37,0XBF,0X37,0XBF,0X78,0XDF,0XB8,0XCF,0XB9,0XCF,
0XFA,0X9E,0XBA,0XD7,0XBA,0XE7,0XB9,0XE7,0X79,0XDF,0X79,0XDF,0XB9,0XDF,0X79,0XDF,
0XB8,0XD7,0XB8,0XCF,0XB8,0XD7,0X78,0XD7,0XB8,0XD7,0X78,0XCF,0X78,0XD7,0XB8,0XD7,
0XB8,0XCF,0XB8,0XCF,0XF9,0XA6,0XF9,0X9E,0X38,0XB7,0X78,0XB7,0XB9,0XCF,0XB8,0XCF,
0X38,0XC7,0X78,0XC7,0XB8,0XD7,0X78,0XC7,0X77,0XC7,0XB8,0XCF,0XFA,0X9E,0XF8,0XAE,
0X79,0XDF,0X7A,0XDF,0X79,0XDF,0XB9,0XDF,0X79,0XDF,0X79,0XDF,0X79,0XDF,0XB9,0XDF,
0X79,0XDF,0X79,0XDF,0X79,0XDF,0XBB,0XD7,0XFA,0X9E,0X39,0XB7,0X37,0XBF,0XB9,0XD7,
0X38,0XC7,0X78,0XC7,0X78,0XD7,0X38,0XBF,0X7A,0XB7,0XFB,0X96,0X79,0XD7,0X78,0XCF,
0X78,0XB7,0XB8,0XC7,0XB8,0XCF,0X37,0XAF,0XF8,0X96,0XF7,0X9E,0X78,0XCF,0X78,0XC7,
0X78,0XCF,0XB9,0XD7,0XB8,0XCF,0XB9,0XCF,0XB8,0XD7,0X78,0XCF,0XB8,0XCF,0X77,0XD7,
0XF8,0XA6,0XFA,0X9E,0X37,0XAF,0XB8,0XCF,0XB8,0XCF,0X37,0XBF,0XB8,0XCF,0XB8,0XCF,
0X78,0XC7,0X78,0XCF,0XB8,0XCF,0XFA,0XA6,0XBA,0XD7,0X79,0XE7,0X79,0XDF,0X79,0XDF,
0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0X79,0XDF,0XB9,0XDF,0X79,0XDF,0X7A,0XD7,0XFA,0XA6,
0X78,0XB7,0X37,0XBF,0XB8,0XCF,0X37,0XBF,0X37,0XBF,0X78,0XD7,0X77,0XBF,0X39,0XB7,
0XF9,0X9E,0XF8,0XA6,0X79,0XCF,0X37,0XB7,0X37,0XB7,0X77,0XC7,0X36,0XAF,0X39,0XAF,
0XF9,0X9E,0XB8,0XCF,0XB8,0XCF,0XB8,0XD7,0XB8,0XD7,0XB8,0XD7,0XB8,0XCF,0X78,0XD7,
0X78,0XD7,0XB9,0XCF,0XB8,0XCF,0XBA,0XC7,0XFB,0X96,0X79,0XB7,0X37,0XB7,0XB8,0XCF,
0X78,0XC7,0X37,0XC7,0X78,0XD7,0X77,0XC7,0X77,0XC7,0XB8,0XD7,0XF9,0XA6,0XB9,0XAE,
0X79,0XDF,0X7A,0XDF,0X79,0XDF,0X79,0XDF,0X79,0XE7,0X79,0XE7,0X79,0XDF,0XB9,0XDF,
0XB9,0XDF,0X7A,0XDF,0X7A,0XDF,0XBB,0XD7,0XFA,0X9E,0X39,0XBF,0X37,0XC7,0X77,0XD7,
0X38,0XCF,0X77,0XC7,0X78,0XD7,0X37,0XBF,0X78,0XB7,0XFA,0X96,0XBA,0XC7,0X78,0XCF,
0X78,0XB7,0XB8,0XBF,0X78,0XCF,0X37,0XAF,0XB8,0X9E,0XF9,0X9E,0X77,0XCF,0XB8,0XCF,
0X78,0XCF,0XB8,0XD7,0X78,0XCF,0XB8,0XCF,0XB8,0XD7,0XB8,0XCF,0XB9,0XCF,0X78,0XD7,
0XF8,0XA6,0XFA,0X9E,0X37,0XB7,0XB8,0XCF,0X77,0XCF,0X36,0XBF,0X78,0XCF,0X78,0XD7,
0X78,0XC7,0X78,0XC7,0X78,0XCF,0XFA,0XA6,0XB8,0XAE,0X79,0XDF,0X79,0XDF,0X79,0XDF,
0XB9,0XDF,0XB9,0XDF,0X79,0XDF,0X79,0XDF,0X79,0XDF,0X79,0XE7,0X79,0XD7,0XFA,0X9E,
0XF8,0X9E,0X37,0XB7,0X78,0XCF,0XB8,0XCF,0X37,0XBF,0X78,0XCF,0X78,0XCF,0X38,0XBF,
0XFA,0X96,0XF8,0XA6,0X78,0XCF,0X36,0XB7,0X37,0XBF,0X77,0XCF,0XF7,0XB6,0X38,0XA7,
0XFA,0X9E,0XBA,0XC7,0XB8,0XCF,0XB8,0XCF,0XB9,0XD7,0XB9,0XD7,0X78,0XCF,0XB8,0XD7,
0XB9,0XD7,0XB8,0XCF,0XB9,0XCF,0XBA,0XC7,0XFA,0X9E,0X39,0XB7,0X37,0XBF,0X78,0XD7,
0X37,0XC7,0X37,0XC7,0XB8,0XD7,0X37,0XCF,0X37,0XC7,0X78,0XD7,0X78,0XD7,0X79,0XD7,
0XFA,0XA6,0XBB,0XD7,0XBA,0XDF,0X79,0XDF,0X79,0XDF,0XB9,0XDF,0XB9,0XDF,0X79,0XDF,
0X78,0XCF,0X37,0XB7,0X36,0XBF,0X78,0XD7,0X37,0XBF,0X37,0XC7,0XB8,0XD7,0X37,0XC7,
0X37,0XBF,0XB8,0XCF,0XF9,0XA6,0XF9,0X9E,0X78,0XB7,0X78,0XB7,0XB8,0XCF,0X78,0XCF,
0X78,0XCF,0XB8,0XD7,0X77,0XD7,0X78,0XCF,0X78,0XCF,0XB9,0XCF,0XFB,0X9E,0XF9,0XA6,
0X79,0XD7,0X79,0XDF,0X79,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XDF,0X78,0XDF,0X78,0XD7,
0X78,0XD7,0X79,0XDF,0X7A,0XD7,0X7B,0XCF,0XF9,0X9E,0X79,0XCF,0X77,0XCF,0X78,0XD7,
0X78,0XDF,0XB8,0XD7,0XB8,0XD7,0X77,0XCF,0X79,0XC7,0XFA,0X9E,0X79,0XCF,0XB8,0XC7,
0X37,0XB7,0X78,0XC7,0X38,0XC7,0X38,0XAF,0XB9,0X9E,0X77,0X9E,0X38,0XCF,0XF6,0XBE,
0X37,0XBF,0X78,0XCF,0X37,0XBF,0X38,0XBF,0X78,0XD7,0X37,0XBF,0X37,0XBF,0X78,0XCF,
0XB8,0X9E,0XB9,0X96,0X37,0XB7,0X78,0XC7,0X77,0XC7,0X77,0XCF,0X77,0XCF,0X77,0XCF,
0X78,0XCF,0X78,0XCF,0XB9,0XCF,0XFB,0XA6,0XBB,0XD7,0XB9,0XDF,0X79,0XDF,0X79,0XDF,
0X79,0XDF,0X79,0XDF,0X79,0XDF,0X79,0XDF,0XB9,0XDF,0X79,0XD7,0X7A,0XCF,0XF9,0X9E,
0XB9,0XBF,0X78,0XCF,0X77,0XC7,0X78,0XD7,0XB7,0XD7,0X77,0XCF,0X77,0XCF,0X78,0XCF,
0XF8,0XA6,0XF7,0XA6,0X78,0XCF,0X37,0XBF,0XF6,0XB6,0X78,0XCF,0XF7,0XB6,0XF9,0XAE,
0XB8,0X9E,0X79,0XCF,0X79,0XC7,0X37,0XBF,0X78,0XD7,0XB8,0XCF,0X37,0XBF,0XB8,0XD7,
0XB8,0XD7,0X78,0XBF,0XB8,0XCF,0XBA,0XC7,0XFB,0X9E,0X79,0XB7,0X37,0XB7,0X78,0XCF,
0XB9,0XD7,0X78,0XD7,0XB9,0XD7,0X78,0XD7,0XB8,0XD7,0XB8,0XCF,0XF9,0XA6,0XF9,0XA6,
0X79,0XD7,0X79,0XD7,0X78,0XD7,0XB8,0XDF,0X78,0XDF,0X78,0XDF,0X78,0XD7,0X78,0XD7,
0X78,0XD7,0X79,0XDF,0X79,0XD7,0XBB,0XCF,0XFB,0X9E,0X7A,0XC7,0XB8,0XCF,0X77,0XD7,
0X78,0XD7,0X78,0XD7,0X78,0XD7,0X78,0XD7,0XB9,0XCF,0XFA,0X9E,0X7A,0XC7,0X77,0XCF,
0X37,0XB7,0XB8,0XC7,0X37,0XCF,0X38,0XAF,0XB8,0X9E,0XB9,0X9E,0X78,0XC7,0X37,0XB7,
0X37,0XB7,0X78,0XCF,0X77,0XBF,0X37,0XBF,0X78,0XD7,0X36,0XBF,0X37,0XB7,0X78,0XCF,
0XB7,0X9E,0XB9,0X96,0X38,0XAF,0X78,0XC7,0X78,0XCF,0X77,0XCF,0X77,0XCF,0X77,0XCF,
0XB8,0XCF,0X78,0XD7,0XB8,0XCF,0XFA,0XA6,0XF8,0XAE,0XB9,0XDF,0XB9,0XDF,0X79,0XDF,
0X79,0XDF,0X79,0XDF,0X79,0XDF,0X79,0XDF,0XB9,0XDF,0X7A,0XDF,0X7A,0XD7,0XBA,0X9E,
0XB7,0XA6,0X78,0XCF,0X37,0XC7,0X77,0XCF,0X77,0XCF,0X78,0XD7,0X77,0XD7,0X78,0XCF,
0XF9,0X96,0XB8,0XA6,0X78,0XCF,0X37,0XAF,0XF7,0XB6,0X78,0XC7,0XF7,0XAE,0X39,0XAF,
0XB9,0X96,0X7A,0XC7,0X79,0XCF,0X37,0XB7,0XB9,0XCF,0XB8,0XD7,0X37,0XBF,0XB9,0XD7,
0XB9,0XD7,0X78,0XBF,0XB9,0XCF,0XBA,0XC7,0XFA,0X9E,0X79,0XB7,0X37,0XB7,0XB8,0XCF,
0XB7,0XCF,0X77,0XCF,0X77,0XCF,0X77,0XD7,0X77,0XCF,0X77,0XD7,0X77,0XD7,0X79,0XC7,
0XFA,0X9E,0XBB,0XD7,0XBA,0XDF,0X79,0XDF,0X79,0XDF,0X79,0XDF,0X79,0XDF,0X79,0XDF,
0X77,0XC7,0X36,0XAF,0XF5,0XB6,0X77,0XD7,0X37,0XBF,0X37,0XBF,0X78,0XD7,0X38,0XBF,
0X38,0XB7,0X78,0XCF,0XF9,0XA6,0XF9,0X9E,0X78,0XB7,0XF9,0X9E,0X7A,0XC7,0XB9,0XCF,
0XFA,0XA6,0XF8,0XAE,0XB9,0XCF,0XFA,0XA6,0XF8,0XA6,0X78,0XCF,0XF9,0XA6,0XF7,0XA6,
0X78,0XD7,0X78,0XCF,0XB8,0XC7,0X70,0X76,0X6F,0X6E,0X6F,0X6E,0X6F,0X6E,0X70,0X76,
0X30,0X86,0X78,0XCF,0X77,0XD7,0X79,0XC7,0XF9,0X96,0XFA,0X9E,0XB8,0XA6,0XB9,0XCF,
0XFA,0X9E,0XF9,0XA6,0X78,0XCF,0XBA,0X96,0XBA,0X96,0XB9,0X8E,0X77,0XC7,0X77,0XC7,
0X77,0XC7,0X2F,0X7E,0X6E,0X66,0X6E,0X66,0X6F,0X5E,0X6F,0X5E,0X6F,0X66,0XF6,0X86,
0XB6,0X96,0X77,0XC7,0XF8,0XA6,0XB7,0XA6,0X77,0XCF,0XF7,0X9E,0XB7,0X9E,0X77,0XCF,
0X30,0X76,0X71,0X66,0X6F,0X66,0X6F,0X66,0X6F,0X66,0X6F,0X6E,0XB6,0XBF,0X77,0XC7,
0X37,0XB7,0X78,0XC7,0X78,0XC7,0XFB,0X9E,0XBB,0XCF,0X79,0XDF,0X79,0XDF,0X79,0XDF,
0X79,0XDF,0X79,0XDF,0XB8,0XDF,0X79,0XDF,0X78,0XD7,0X78,0XCF,0X78,0XC7,0X30,0X6E,
0X6F,0X6E,0X6F,0X66,0X6F,0X66,0XAF,0X66,0X2F,0X76,0X77,0XC7,0X36,0XAF,0X36,0X9F,
0XF8,0X8E,0XB7,0X96,0X77,0XB7,0X2F,0X6E,0X6F,0X66,0X6E,0X66,0X6E,0X66,0X6F,0X66,
0X30,0X6E,0XB6,0XBF,0X77,0XBF,0X36,0XAF,0X78,0XCF,0XB8,0XC7,0X37,0XBF,0XB8,0XD7,
0X78,0XCF,0X78,0XB7,0XB8,0XCF,0XBA,0XC7,0XFA,0X96,0X79,0XB7,0X38,0XBF,0XB8,0XCF,
0X37,0XBF,0X37,0XC7,0XB8,0XD7,0X37,0XB7,0X77,0XBF,0XB9,0XD7,0XF9,0XA6,0XB8,0XA6,
0X78,0XD7,0X78,0XCF,0XF8,0XC7,0X70,0X76,0X6F,0X6E,0X6F,0X6E,0X6F,0X6E,0X70,0X76,
0X30,0X7E,0X78,0XD7,0X79,0XCF,0X79,0XC7,0XFA,0X9E,0X38,0XB7,0X37,0XBF,0X78,0XD7,
0X37,0XC7,0X37,0XBF,0XB8,0XCF,0X36,0XBF,0X37,0XB7,0XB9,0X96,0X79,0XBF,0X77,0XCF,
0X78,0XC7,0X30,0X7E,0X6F,0X6E,0X6F,0X66,0X6F,0X66,0X6F,0X66,0X2E,0X66,0X75,0X9F,
0X36,0XA7,0X77,0XCF,0X37,0XBF,0X37,0XB7,0X77,0XCF,0X36,0XB7,0XF7,0XAE,0X78,0XC7,
0X31,0X76,0X70,0X66,0X6E,0X66,0X6E,0X66,0X6F,0X66,0X6E,0X66,0XB7,0XBF,0X77,0XC7,
0X37,0XAF,0X36,0XB7,0X79,0XCF,0XFA,0X9E,0XB8,0XA6,0X79,0XDF,0XB9,0XDF,0X79,0XDF,
0X79,0XDF,0X79,0XDF,0X79,0XDF,0X78,0XDF,0X77,0XDF,0X78,0XD7,0X77,0XCF,0X31,0X6E,
0X70,0X6E,0X6F,0X66,0X6F,0X66,0X6F,0X66,0X6F,0X6E,0X77,0XB7,0X79,0XAF,0XB9,0X8E,
0XF9,0X86,0XB7,0X96,0X77,0XBF,0X2E,0X66,0X6F,0X66,0X2E,0X66,0X2E,0X66,0X70,0X66,
0X70,0X5E,0XB7,0XB7,0X77,0XBF,0X36,0XAF,0X78,0XCF,0XB8,0XCF,0X37,0XB7,0XB8,0XCF,
0XB9,0XCF,0X37,0XB7,0XB8,0XC7,0XBA,0XC7,0XFA,0X96,0X38,0XA7,0XF6,0XB6,0X77,0XC7,
0X77,0XC7,0X77,0XC7,0X6E,0X66,0X6F,0X6E,0X6F,0X6E,0X6E,0X5E,0X30,0X76,0X38,0XCF,
0XFA,0X96,0XBB,0XCF,0X79,0XD7,0X79,0XDF,0X79,0XDF,0X79,0XDF,0X79,0XDF,0XB9,0XDF,
0XB7,0XB7,0X76,0XA7,0X35,0XAF,0XB7,0XCF,0X36,0XB7,0X36,0XBF,0X78,0XD7,0X37,0XBF,
0X37,0XBF,0X78,0XCF,0XF9,0X9E,0XF8,0X9E,0X78,0XB7,0XF9,0X9E,0XBA,0XC7,0X78,0XCF,
0XFA,0XA6,0XF9,0XA6,0XB7,0XCF,0XF9,0X9E,0XF8,0XA6,0XB8,0XCF,0XF9,0X9E,0XF8,0X9E,
0XB7,0XBF,0XB7,0XBF,0XF7,0XB7,0XAF,0X6E,0XAE,0X6E,0XAE,0X6E,0X6E,0X6E,0XAF,0X6E,
0X70,0X76,0XB7,0XBF,0XB8,0XBF,0XBA,0XBF,0XF9,0X96,0XFA,0X96,0XF8,0XA6,0X78,0XCF,
0XFA,0X9E,0XF8,0XA6,0X78,0XCF,0XF9,0X96,0XF9,0X96,0XF9,0X96,0XB7,0XB7,0XB7,0XB7,
0XB7,0XBF,0X6F,0X6E,0XAF,0X66,0X6E,0X5E,0X6E,0X56,0X6E,0X5E,0X6F,0X66,0X35,0X87,
0XF5,0X8E,0XB6,0XB7,0XF6,0X9E,0XB6,0XA6,0X77,0XCF,0XF6,0X9E,0XF6,0X96,0XB7,0XAF,
0X2F,0X66,0X6F,0X5E,0X6E,0X5E,0X6E,0X66,0X6E,0X66,0XAE,0X66,0XF6,0XAF,0XB7,0XB7,
0X76,0XA7,0XB8,0XC7,0X78,0XC7,0XFB,0X96,0XBB,0XCF,0X78,0XDF,0XB9,0XDF,0X79,0XDF,
0X79,0XDF,0X79,0XDF,0XB8,0XDF,0XB9,0XDF,0XB8,0XCF,0XB7,0XC7,0XB7,0XBF,0X6E,0X6E,
0X6E,0X66,0XAE,0X5E,0X6E,0X66,0XAE,0X6E,0X6E,0X6E,0XF6,0XB7,0X35,0X9F,0X76,0X9F,
0X37,0X8F,0XF6,0X8E,0XB6,0XAF,0X6E,0X66,0X6E,0X66,0X6E,0X66,0X6E,0X66,0X6E,0X5E,
0X70,0X66,0XF7,0XB7,0XB7,0XB7,0X75,0XAF,0XB7,0XCF,0XB7,0XC7,0X36,0XBF,0XB8,0XD7,
0X78,0XD7,0X78,0XBF,0X78,0XCF,0XBA,0XC7,0XFA,0X96,0X39,0XB7,0X38,0XBF,0X78,0XCF,
0X78,0XBF,0X38,0XC7,0XB8,0XD7,0X78,0XBF,0X38,0XBF,0XB8,0XCF,0XF8,0X9E,0XF8,0X9E,
0XB7,0XC7,0XB7,0XBF,0XF7,0XB7,0XB0,0X76,0X6E,0X66,0XAE,0X66,0XAE,0X66,0XAF,0X6E,
0X70,0X76,0XB8,0XBF,0XB8,0XBF,0XB9,0XB7,0XF9,0X96,0X78,0XAF,0X37,0XB7,0X78,0XCF,
0X36,0XBF,0X77,0XBF,0XB8,0XD7,0X36,0XB7,0X37,0XAF,0XB9,0X8E,0XB9,0XB7,0XB6,0XBF,
0XB7,0XB7,0X6F,0X6E,0XAE,0X66,0X6E,0X66,0XAE,0X5E,0X6E,0X5E,0X6E,0X5E,0X75,0X97,
0X75,0X9F,0XB7,0XBF,0X76,0XB7,0X36,0XB7,0X77,0XD7,0X35,0XAF,0X76,0XA7,0XB7,0XBF,
0X2F,0X6E,0X6F,0X5E,0X6E,0X66,0X6E,0X5E,0X6E,0X66,0XAE,0X66,0XF6,0XA7,0XB7,0XB7,
0X76,0XA7,0X35,0XAF,0X78,0XCF,0XFA,0X9E,0XF8,0XA6,0X79,0XDF,0XB9,0XDF,0X79,0XDF,
0XB9,0XDF,0X79,0XDF,0XB9,0XDF,0XB8,0XDF,0XB8,0XC7,0XF8,0XC7,0XB7,0XBF,0X6F,0X6E,
0XAE,0X66,0X6E,0X5E,0X6E,0X66,0XAE,0X5E,0XAF,0X66,0XB6,0XAF,0XB8,0XAF,0XF7,0X86,
0XF7,0X8E,0XF6,0X8E,0XB7,0XAF,0X6E,0X66,0X6F,0X66,0X6E,0X66,0X6E,0X66,0X6F,0X66,
0X6F,0X5E,0XF7,0XB7,0XB7,0XB7,0X75,0X9F,0XB7,0XC7,0XB8,0XCF,0X36,0XB7,0XB8,0XD7,
0XB8,0XD7,0X36,0XB7,0XB8,0XCF,0XB9,0XC7,0XF9,0X96,0X38,0XA7,0X36,0XA7,0XB7,0XB7,
0XB7,0XB7,0XB6,0XB7,0XAE,0X6E,0XAE,0X6E,0X6F,0X6E,0XAE,0X66,0X6F,0X6E,0X78,0XC7,
0XFA,0X9E,0XBA,0XCF,0X79,0XD7,0X79,0XDF,0X79,0XDF,0X79,0XDF,0XB9,0XDF,0XB8,0XDF,
0X6F,0X66,0X6F,0X66,0XF0,0X7D,0X77,0XCF,0X37,0XD7,0X77,0XCF,0X77,0XCF,0X76,0XCF,
0X77,0XCF,0X77,0XC7,0XB8,0X9E,0XB8,0X9E,0X37,0XAF,0XF8,0X96,0X7A,0XC7,0X78,0XCF,
0XFA,0X9E,0XF9,0XA6,0XB8,0XC7,0XF9,0X9E,0XB8,0XAE,0X78,0XC7,0XB8,0X96,0XF7,0X8E,
0X30,0X66,0X6F,0X66,0X6E,0X66,0XEE,0X6E,0XEE,0X66,0X2E,0X67,0XEE,0X66,0X2F,0X6F,
0XF0,0X6E,0X6E,0X66,0X30,0X66,0X33,0X6E,0XB9,0X8E,0XB9,0X8E,0XB8,0X96,0X78,0XC7,
0XF9,0X9E,0XB8,0XA6,0X37,0XCF,0XB9,0X8E,0XBA,0X8E,0XB8,0X86,0X31,0X66,0X30,0X66,
0X6F,0X5E,0XEF,0X66,0X2E,0X67,0X2E,0X67,0X2E,0X67,0X2E,0X67,0XEF,0X66,0X6E,0X56,
0X6F,0X5E,0X6F,0X66,0X76,0XB7,0X76,0XC7,0X76,0XC7,0X2F,0X76,0X6F,0X66,0X2E,0X5E,
0XF0,0X66,0XEF,0X66,0XEE,0X66,0XEE,0X66,0XEE,0X66,0XEE,0X6E,0X6D,0X5E,0X2E,0X66,
0X2E,0X66,0XB7,0XBF,0X77,0XC7,0XBA,0X96,0X79,0XCF,0X78,0XDF,0X78,0XDF,0X78,0XDF,
0X78,0XDF,0X78,0XDF,0X37,0XD7,0X77,0XCF,0X30,0X7E,0X2F,0X66,0X6F,0X66,0XEF,0X6E,
0XEE,0X66,0XEE,0X66,0XEE,0X66,0X2E,0X67,0XEE,0X6E,0X6E,0X5E,0X6F,0X5E,0X2F,0X5E,
0X6F,0X5E,0X6F,0X5E,0X6E,0X5E,0XEF,0X66,0XEE,0X66,0XEE,0X66,0X2E,0X67,0XEE,0X66,
0XEF,0X66,0X6E,0X66,0X6F,0X66,0X2F,0X76,0X77,0XC7,0X37,0XCF,0X37,0XD7,0X77,0XCF,
0X77,0XD7,0X77,0XCF,0X77,0XCF,0X7A,0XC7,0XB9,0X8E,0XF8,0XAE,0X37,0XB7,0X78,0XD7,
0X77,0XB7,0X37,0XBF,0X78,0XDF,0X77,0XBF,0X37,0XB7,0X77,0XC7,0XB8,0X96,0XF6,0X86,
0X30,0X6E,0X2F,0X66,0XAE,0X66,0XEF,0X6E,0XEE,0X66,0XEE,0X66,0X2E,0X67,0X2F,0X6F,
0XEF,0X6E,0X6E,0X66,0X2F,0X66,0X31,0X6E,0XB9,0X8E,0X39,0XAF,0XF7,0XB6,0X78,0XD7,
0X36,0XBF,0X37,0XB7,0X77,0XC7,0XF6,0XAE,0XF7,0XA6,0XB8,0X8E,0X32,0X6E,0X6F,0X66,
0X6F,0X5E,0XEF,0X66,0XED,0X66,0XEE,0X66,0XEE,0X66,0X2E,0X67,0XEE,0X66,0X6E,0X56,
0X6F,0X5E,0X6F,0X6E,0X76,0XB7,0X36,0XC7,0X36,0XCF,0X2F,0X7E,0X6F,0X6E,0X2E,0X66,
0XEF,0X66,0X2E,0X67,0XEE,0X66,0X2E,0X67,0XEE,0X66,0XEE,0X66,0X6D,0X66,0X2E,0X66,
0X6F,0X66,0X30,0X76,0X78,0XCF,0XB9,0X9E,0XB7,0XA6,0X78,0XD7,0X77,0XD7,0X78,0XD7,
0X78,0XD7,0X38,0XDF,0X78,0XDF,0X77,0XCF,0X2F,0X6E,0X70,0X6E,0X6F,0X66,0XEF,0X6E,
0X2F,0X6F,0XEE,0X66,0XEE,0X66,0XEE,0X66,0XEF,0X6E,0X2E,0X5E,0X6F,0X5E,0X6F,0X5E,
0X6F,0X5E,0X6F,0X5E,0X6F,0X5E,0XEE,0X66,0XEE,0X66,0XEF,0X66,0X2D,0X67,0XEE,0X66,
0X2F,0X5F,0X6F,0X66,0X2F,0X66,0X6F,0X66,0XB7,0XBF,0X77,0XCF,0X77,0XCF,0XB7,0XD7,
0XB7,0XCF,0X77,0XCF,0X77,0XCF,0X7A,0XBF,0XB9,0X8E,0X39,0XA7,0X75,0X9F,0X6F,0X66,
0X6E,0X66,0X6E,0X66,0X2D,0X67,0XEE,0X66,0X2E,0X67,0XEE,0X66,0XAF,0X76,0X78,0XC7,
0XBA,0X96,0X7A,0XCF,0X78,0XDF,0X78,0XDF,0X78,0XDF,0X78,0XDF,0X78,0XD7,0X78,0XD7,
0XED,0X66,0XEE,0X66,0XAF,0X66,0X6F,0X6E,0X6F,0X6E,0X2F,0X76,0X76,0XBF,0XB6,0X96,
0XF6,0X8E,0X30,0X76,0X71,0X66,0X31,0X6E,0X36,0X9F,0XB8,0X8E,0XB9,0XBF,0X77,0XC7,
0X78,0XC7,0X77,0XC7,0X77,0XC7,0X78,0XC7,0XB7,0XBF,0X6F,0X6E,0X2F,0X56,0X6F,0X56,
0XEF,0X66,0XEE,0X66,0XEE,0X66,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0XEE,0X66,0XEE,0X66,0XF0,0X66,0X6F,0X5E,0X70,0X5E,0X30,0X66,0X76,0XC7,
0X78,0XBF,0XB8,0XAF,0X2F,0X66,0X6F,0X5E,0X6F,0X5E,0X2F,0X56,0X6E,0X56,0X6D,0X4E,
0X2E,0X5F,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0X2E,0X5F,0XEE,0X5E,0X2E,0X67,
0XEE,0X66,0XED,0X5E,0X6D,0X56,0X2E,0X5E,0X2E,0X66,0XEF,0X66,0XEE,0X66,0X2F,0X67,
0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XED,0X5E,0XEC,0X5E,0XED,0X5E,0X2E,0X67,0X2E,0X67,
0XED,0X5E,0X6E,0X5E,0X2F,0X5E,0X70,0X66,0XF6,0X8E,0XF8,0X96,0X78,0XC7,0X30,0X76,
0X70,0X66,0X6E,0X6E,0X6E,0X66,0X2F,0X66,0X6E,0X5E,0XEF,0X66,0XEE,0X66,0XEE,0X5E,
0XED,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,
0X6D,0X4E,0X6E,0X56,0X6C,0X4E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,
0XEE,0X5E,0XEE,0X66,0X2E,0X67,0XEF,0X66,0X6E,0X66,0X2F,0X66,0X2E,0X6E,0X75,0XB7,
0X78,0XB7,0XB8,0X8E,0X31,0X6E,0X70,0X6E,0X31,0X66,0X36,0XA7,0X35,0XAF,0X37,0XCF,
0X78,0XCF,0X78,0XCF,0X77,0XCF,0X77,0XC7,0XB6,0XAF,0XAF,0X6E,0X6F,0X5E,0X6E,0X56,
0XEE,0X66,0XEE,0X66,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2F,0X67,0X2F,0X67,0XF0,0X66,0X6F,0X5E,0X6F,0X5E,0X2F,0X76,0X76,0XC7,
0X76,0XBF,0X76,0XBF,0X6F,0X6E,0X2F,0X5E,0X6F,0X5E,0X70,0X5E,0X6E,0X56,0X6E,0X56,
0XEE,0X5E,0XEE,0X5E,0XED,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X66,
0XEF,0X66,0XEE,0X66,0X2D,0X5E,0X2E,0X5E,0X2E,0X5E,0XEF,0X66,0XEE,0X66,0X2E,0X67,
0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XEE,0X66,
0X2E,0X67,0XEF,0X66,0X2F,0X5E,0X70,0X66,0X32,0X6E,0XB8,0X96,0X78,0XC7,0XB7,0XBF,
0X70,0X6E,0X6F,0X6E,0X6F,0X66,0X2F,0X66,0X6E,0X5E,0X6D,0X5E,0XEE,0X66,0XED,0X5E,
0XEE,0X5E,0XEE,0X56,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XEF,0X5E,0XEE,0X5E,0XEE,0X5E,
0X6D,0X4E,0X6D,0X56,0X6D,0X56,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,
0XEE,0X56,0X2E,0X67,0X2E,0X67,0XEE,0X5E,0X6E,0X66,0X2F,0X66,0X2F,0X6E,0XB6,0XBF,
0X77,0XC7,0X36,0XAF,0X30,0X76,0X70,0X6E,0X6F,0X5E,0X6F,0X5E,0X6E,0X5E,0X6E,0X56,
0XEF,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0X2D,0X5F,0XEE,0X5E,0X2F,0X66,
0X70,0X66,0XF6,0X8E,0XB7,0X96,0X78,0XC7,0X30,0X7E,0X6F,0X6E,0X6F,0X6E,0X6F,0X66,
0X2D,0X5F,0X2E,0X67,0XEE,0X66,0XAE,0X5E,0X6F,0X66,0X6F,0X76,0XB5,0XAF,0XF5,0X8E,
0X34,0X7F,0X6F,0X66,0X6F,0X5E,0X70,0X6E,0X75,0X97,0XF6,0X8E,0XB7,0XAF,0XB7,0XB7,
0XB6,0XBF,0X76,0XBF,0X77,0XBF,0X76,0XB7,0XB6,0XAF,0XAF,0X6E,0X6E,0X56,0XAD,0X5E,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2D,0X5F,0X2F,0X67,0XEF,0X66,0X6E,0X5E,0X6F,0X5E,0X2F,0X66,0XB6,0XAF,
0XB6,0XB7,0XB4,0X9F,0X6E,0X5E,0X6E,0X56,0X6E,0X56,0X6E,0X4E,0X6D,0X4E,0XAD,0X56,
0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0X2D,0X57,0X2E,0X57,0X2D,0X57,0X2D,0X5F,
0X2E,0X5F,0X2D,0X5F,0XAD,0X56,0X6D,0X56,0X6D,0X56,0XEE,0X5E,0X2E,0X67,0X2E,0X5F,
0XED,0X5E,0X2E,0X5F,0X2E,0X5F,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X66,
0XED,0X5E,0XAD,0X5E,0X6E,0X5E,0X6F,0X5E,0X34,0X87,0XF5,0X86,0XB7,0XBF,0X6F,0X76,
0XAF,0X66,0X6E,0X6E,0XAE,0X5E,0X6E,0X66,0X6E,0X5E,0XEE,0X66,0XED,0X5E,0XEE,0X5E,
0XEE,0X5E,0XEE,0X56,0XED,0X56,0X2D,0X5F,0XED,0X5E,0XED,0X5E,0XED,0X56,0XED,0X5E,
0XAD,0X4E,0X6C,0X4E,0X6C,0X4E,0XEE,0X5E,0XEE,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,
0X2D,0X5F,0X2D,0X5F,0X2E,0X67,0XEE,0X66,0XAE,0X5E,0X6F,0X66,0X6F,0X6E,0XB6,0XAF,
0XB7,0XAF,0XF6,0X86,0X70,0X6E,0X6F,0X66,0XAF,0X5E,0X76,0X97,0X75,0X9F,0XB6,0XB7,
0XB6,0XBF,0X76,0XBF,0X77,0XCF,0XB7,0XBF,0XF6,0XAF,0XAE,0X6E,0X6E,0X56,0X6D,0X56,
0XEE,0X66,0X2E,0X67,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2E,0X57,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0XEF,0X5E,0X6E,0X56,0X6E,0X5E,0X2E,0X66,0XB6,0XB7,
0XB5,0XB7,0XF6,0XBF,0XAF,0X6E,0X6E,0X5E,0X6D,0X56,0X6E,0X56,0X6D,0X4E,0X6D,0X4E,
0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XED,0X56,0X2D,0X5F,
0X2E,0X67,0XED,0X66,0X6C,0X56,0X6D,0X56,0X6D,0X56,0XEE,0X66,0X2E,0X67,0X2D,0X67,
0XED,0X5E,0X2D,0X57,0XED,0X5E,0XED,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X66,
0XEE,0X5E,0XEE,0X5E,0X6E,0X5E,0XAF,0X5E,0X70,0X66,0XF6,0X86,0XB6,0XB7,0XF6,0XAF,
0XAF,0X6E,0XAE,0X6E,0XAE,0X66,0X6E,0X66,0X6D,0X56,0XAD,0X56,0XEE,0X66,0XEE,0X5E,
0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XED,0X5E,0XEE,0X5E,0X2D,0X57,0XED,0X56,
0X6D,0X4E,0X6D,0X4E,0X6D,0X4E,0XEE,0X5E,0XEE,0X5E,0XED,0X5E,0XEE,0X5E,0XED,0X5E,
0X2E,0X57,0X2E,0X5F,0X2E,0X67,0XEE,0X66,0XAE,0X5E,0X6E,0X66,0X6F,0X66,0XF5,0XA7,
0XB6,0XAF,0X75,0X9F,0XAF,0X6E,0XAF,0X66,0X6E,0X5E,0X6E,0X56,0XAE,0X56,0X6E,0X56,
0XEE,0X56,0XEE,0X5E,0XEF,0X5E,0X2E,0X57,0XED,0X5E,0XED,0X5E,0XED,0X5E,0X6E,0X56,
0X6F,0X5E,0X35,0X87,0XF5,0X86,0XB7,0XB7,0X6F,0X76,0XAF,0X66,0X6E,0X66,0X6E,0X66,
0XED,0X5E,0X6C,0X4E,0X6D,0X56,0X6D,0X56,0X6E,0X5E,0X6E,0X5E,0X6E,0X5E,0X6E,0X66,
0X6E,0X5E,0X2F,0X67,0XEF,0X66,0XF0,0X66,0X6F,0X56,0X2F,0X5E,0X6F,0X5E,0X2F,0X66,
0X6F,0X6E,0X2F,0X76,0X76,0XC7,0X6F,0X6E,0X6E,0X66,0XEF,0X66,0XEE,0X5E,0XEE,0X5E,
0XEE,0X5E,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0XED,0X5E,0XEE,0X56,0XEE,0X5E,0X6E,0X56,0X6E,0X4E,0X6E,0X56,0X6E,0X5E,
0X6E,0X5E,0X6D,0X56,0XEF,0X66,0XEE,0X66,0X2E,0X67,0XEE,0X5E,0XEE,0X5E,0XEF,0X5E,
0X6D,0X4E,0X6D,0X56,0X6C,0X4E,0XED,0X5E,0XEC,0X5E,0XEC,0X5E,0X2D,0X5F,0X2D,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X57,0XEE,0X5E,0XEE,0X5E,0X6C,0X56,0X6C,0X56,0XED,0X5E,
0XED,0X5E,0XED,0X5E,0XED,0X5E,0X6C,0X56,0X6D,0X56,0X6D,0X56,0X6D,0X56,0X6D,0X56,
0X2D,0X56,0XEE,0X5E,0XEE,0X5E,0XEF,0X66,0X6E,0X5E,0X70,0X5E,0X2F,0X56,0XEF,0X6E,
0XEF,0X6E,0XEE,0X66,0XED,0X66,0XEE,0X66,0XEE,0X56,0X6C,0X4E,0X6D,0X56,0X6D,0X4E,
0XEE,0X5E,0XED,0X5E,0XED,0X5E,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0XEC,0X56,0XED,0X5E,
0XED,0X5E,0XED,0X5E,0XED,0X5E,0X6D,0X4E,0X6C,0X4E,0XED,0X5E,0XED,0X5E,0XED,0X5E,
0XED,0X5E,0XED,0X5E,0XEE,0X5E,0X6D,0X56,0X6D,0X56,0X6E,0X56,0X6E,0X5E,0X6E,0X5E,
0X6E,0X5E,0X6E,0X5E,0XEF,0X66,0X2F,0X67,0X2E,0X67,0X6F,0X66,0X6F,0X5E,0X6F,0X66,
0X70,0X6E,0X2F,0X7E,0X76,0XC7,0X2E,0X66,0XAE,0X5E,0XEF,0X6E,0X2E,0X67,0X2E,0X5F,
0X2E,0X57,0X2E,0X57,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0XED,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XED,0X5E,0XEE,0X66,0XEE,0X66,0X2D,0X5E,
0X2E,0X5E,0X6D,0X56,0X2E,0X6F,0X2E,0X67,0XEE,0X66,0XEE,0X5E,0XEE,0X56,0XEF,0X5E,
0X6D,0X4E,0X6D,0X4E,0X6D,0X4E,0XED,0X56,0XED,0X5E,0XED,0X5E,0X2D,0X57,0X2D,0X5F,
0X2E,0X5F,0X2E,0X5F,0XEE,0X56,0XEE,0X5E,0X2E,0X57,0X6C,0X4E,0X6C,0X4E,0X2D,0X5F,
0XED,0X56,0XEC,0X5E,0XED,0X5E,0X6C,0X4E,0X6D,0X4E,0X6D,0X4E,0X6D,0X56,0X6D,0X56,
0X6C,0X56,0X6D,0X4E,0X2F,0X57,0X2F,0X67,0XF0,0X6E,0X70,0X56,0X6F,0X5E,0X6E,0X56,
0XEF,0X6E,0XEE,0X66,0XEE,0X66,0X2E,0X67,0X2E,0X5F,0XEE,0X5E,0X6D,0X56,0X6D,0X56,
0X6C,0X56,0XED,0X5E,0XED,0X56,0XED,0X56,0X2D,0X5F,0X2C,0X57,0X2D,0X5F,0X2D,0X5F,
0XED,0X5E,0XED,0X5E,0XED,0X5E,0X6D,0X4E,0X6D,0X4E,0XED,0X5E,0XED,0X5E,0XEC,0X5E,
0XEC,0X5E,0XEC,0X5E,0XED,0X5E,0X6C,0X56,0X2D,0X4E,0X2C,0X4E,0X6D,0X56,0X6E,0X56,
0X6E,0X5E,0X2F,0X5E,0XEF,0X66,0XEE,0X66,0X2D,0X67,0XED,0X5E,0XEE,0X5E,0XED,0X56,
0X6C,0X4E,0X6C,0X4E,0X6D,0X56,0X2D,0X5F,0XED,0X5E,0XED,0X5E,0XEC,0X56,0XED,0X56,
0XEE,0X5E,0X6E,0X56,0X70,0X5E,0X2F,0X5E,0XF0,0X6E,0XEF,0X6E,0XEE,0X66,0XEE,0X66,
0X6B,0X4E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XEF,0X5E,
0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XEF,0X5E,0XEF,0X66,0XEE,0X66,0XEF,0X66,0XEF,0X66,
0XEF,0X66,0XAF,0X6E,0X2E,0X6E,0X2E,0X56,0X6D,0X56,0XEE,0X66,0XEE,0X56,0XED,0X5E,
0XED,0X5E,0X2D,0X57,0X2D,0X57,0X2D,0X57,0XED,0X5E,0XED,0X5E,0X2C,0X5F,0XEC,0X5E,
0XED,0X5E,0XEE,0X5E,0XED,0X5E,0XEE,0X5E,0X6D,0X56,0X6D,0X4E,0X6D,0X56,0XEE,0X66,
0XEF,0X66,0XEF,0X66,0X2E,0X67,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0XED,0X5E,0XEE,0X5E,0XEE,0X5E,0X6C,0X4E,0X6C,0X4E,0XAD,0X56,0XED,0X56,0X2D,0X5F,
0X2D,0X5F,0X2D,0X5F,0XED,0X56,0XED,0X5E,0XED,0X5E,0X6C,0X4E,0X6D,0X4E,0XED,0X5E,
0XAC,0X4E,0X6C,0X4E,0X6C,0X56,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,
0X2D,0X5F,0X6C,0X56,0X6C,0X56,0X6D,0X4E,0XEE,0X5E,0XEE,0X5E,0XEF,0X5E,0XEE,0X5E,
0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0XED,0X5E,0XED,0X5E,0XED,0X5E,
0XAC,0X56,0X6C,0X4E,0X6C,0X4E,0XED,0X56,0XEC,0X5E,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0XEC,0X5E,0XED,0X5E,0X6D,0X56,0X6C,0X4E,0XEC,0X5E,0XED,0X5E,0XED,0X56,
0X6C,0X4E,0X6C,0X4E,0X6C,0X56,0XED,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XEF,0X66,
0XEE,0X5E,0XEE,0X5E,0XEF,0X66,0XEE,0X5E,0X2E,0X5F,0XEF,0X66,0XEF,0X66,0XEE,0X66,
0XEE,0X6E,0XEF,0X6E,0XEE,0X6D,0X6E,0X5E,0X6D,0X56,0XEE,0X66,0X2D,0X5F,0XED,0X5E,
0XED,0X5E,0XED,0X5E,0XEC,0X5E,0XEC,0X5E,0XEC,0X5E,0XED,0X5E,0XEC,0X5E,0X2D,0X57,
0X2D,0X5F,0XED,0X5E,0X2D,0X57,0XED,0X56,0X6D,0X4E,0X6D,0X56,0X6C,0X4E,0XEF,0X5E,
0XEF,0X66,0XEF,0X66,0X2D,0X67,0X2E,0X57,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0XED,0X5E,0XED,0X5E,0XEE,0X5E,0X6C,0X4E,0X6C,0X4E,0X6C,0X4E,0XED,0X56,0X2D,0X5F,
0X2E,0X5F,0X2D,0X57,0XEE,0X56,0X2D,0X5F,0XED,0X56,0X6D,0X56,0X6D,0X56,0XEE,0X5E,
0X6B,0X4E,0X6C,0X4E,0X6C,0X4E,0XED,0X5E,0XED,0X5E,0XEE,0X5E,0XED,0X5E,0XED,0X5E,
0XEE,0X5E,0XAD,0X5E,0X6C,0X4E,0X6D,0X56,0X6D,0X56,0XEF,0X5E,0XEE,0X5E,0XEE,0X5E,
0X2E,0X67,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0XED,0X5E,0XEE,0X5E,
0XEE,0X56,0X6C,0X4E,0X6C,0X4E,0X6C,0X4E,0XED,0X5E,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0XED,0X5E,0XED,0X5E,0XEC,0X5E,0X6C,0X56,0X6C,0X4E,0XED,0X5E,0XED,0X5E,0XED,0X5E,
0X6C,0X4E,0X6D,0X4E,0X6C,0X56,0XEE,0X66,0X6D,0X56,0X6D,0X56,0X6D,0X56,0XEF,0X66,
0XEE,0X5E,0XEE,0X5E,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,
0XED,0X56,0XEE,0X5E,0XEE,0X5E,0X6C,0X56,0X6C,0X4E,0X6D,0X56,0XAD,0X5E,0X6C,0X56,
0X6D,0X56,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,
0X6C,0X4E,0XED,0X56,0XEC,0X5E,0XEC,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0X2D,0X57,
0XEE,0X56,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2E,0X67,0XEE,0X5E,
0XEE,0X66,0XEE,0X66,0X6D,0X56,0X6D,0X56,0X6D,0X56,0XEE,0X5E,0XEE,0X5E,0XEC,0X5E,
0XED,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XEE,0X5E,
0XED,0X5E,0X2D,0X5F,0XED,0X5E,0XED,0X5E,0X6C,0X4E,0X6C,0X4E,0X6C,0X4E,0XED,0X5E,
0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2C,0X57,0XEC,0X5E,0XED,0X5E,0X6C,0X4E,0X6C,0X56,0X6C,0X4E,0XEE,0X5E,0X2D,0X5F,
0X2D,0X5F,0X2C,0X5F,0XED,0X5E,0XED,0X5E,0XED,0X5E,0X6C,0X56,0X6C,0X4E,0XED,0X5E,
0X6D,0X56,0X6D,0X56,0X6C,0X4E,0XED,0X56,0XEC,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,
0XED,0X5E,0XAC,0X56,0X6C,0X4E,0X6C,0X4E,0XED,0X5E,0X2D,0X5F,0XEE,0X5E,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2C,0X5F,0XEC,0X56,
0XAC,0X4E,0X6C,0X56,0X6C,0X4E,0XED,0X5E,0XED,0X5E,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0XED,0X5E,0XED,0X5E,0X6C,0X4E,0X6C,0X56,0XED,0X5E,0XEE,0X5E,0XEE,0X5E,
0X6C,0X4E,0X6C,0X4E,0XAC,0X4E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0X2D,0X57,
0XED,0X5E,0XED,0X5E,0X2E,0X5F,0XEE,0X56,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2E,0X5F,0XEE,0X5E,0X6D,0X56,0X6D,0X4E,0XAD,0X4E,0XEE,0X5E,0XED,0X5E,0XED,0X5E,
0XED,0X5E,0XED,0X5E,0X2D,0X5F,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XEE,0X5E,
0XED,0X5E,0X2D,0X5F,0XED,0X5E,0XEE,0X5E,0X6C,0X4E,0X6C,0X4E,0XAC,0X4E,0XED,0X5E,
0X2E,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0XEC,0X5E,0XEC,0X5E,0XED,0X5E,0X6C,0X4E,0X6C,0X56,0X6C,0X4E,0XED,0X5E,0X2D,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0XED,0X5E,0XED,0X5E,0X6D,0X56,0X6C,0X56,0XED,0X5E,
0X6D,0X56,0X6C,0X4E,0X6C,0X4E,0XED,0X5E,0XEC,0X5E,0XEC,0X5E,0XED,0X5E,0XED,0X5E,
0XED,0X5E,0XED,0X5E,0X6C,0X4E,0X6C,0X56,0X6C,0X4E,0XED,0X5E,0X2D,0X57,0X2D,0X57,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0XED,0X5E,
0XED,0X56,0X6C,0X4E,0X6C,0X4E,0X6C,0X4E,0XED,0X5E,0X2D,0X5F,0X2C,0X5F,0X2D,0X5F,
0XED,0X5E,0XED,0X5E,0XED,0X5E,0X6C,0X4E,0X6C,0X56,0XED,0X5E,0XED,0X5E,0XEE,0X5E,
0X6D,0X56,0X6D,0X56,0X6C,0X56,0XED,0X5E,0X6C,0X4E,0X6C,0X4E,0X6C,0X4E,0XED,0X5E,
0XED,0X56,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0XEC,0X5E,0XEC,0X5E,0X6C,0X4E,0X6C,0X4E,0X6C,0X56,0X6C,0X4E,0X6C,0X56,
0X6C,0X4E,0XED,0X5E,0X2D,0X5F,0X2D,0X57,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,
0XEC,0X56,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X57,
0X2E,0X5F,0X2E,0X5F,0XEE,0X56,0XEE,0X5E,0XEF,0X5E,0X6D,0X56,0X6D,0X56,0X6C,0X4E,
0XEC,0X5E,0XED,0X5E,0XED,0X5E,0X6D,0X56,0X6D,0X56,0X6C,0X4E,0X6D,0X4E,0X6D,0X4E,
0X6B,0X4E,0XEC,0X5E,0X6D,0X56,0X6C,0X4E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,
0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0XED,0X5E,0XED,0X5E,0X2D,0X5F,0X6D,0X56,0XEE,0X5E,
0XED,0X5E,0XED,0X5E,0XED,0X5E,0XEE,0X5E,0X6C,0X4E,0X6D,0X56,0X6D,0X56,0X6D,0X56,
0X6D,0X56,0X6C,0X56,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,
0X6C,0X56,0XEE,0X5E,0XED,0X5E,0XEC,0X5E,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X57,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0XED,0X5E,0XED,0X56,0X6C,0X4E,0X6C,0X56,0XED,0X5E,0XEC,0X5E,0XEC,0X5E,
0XED,0X5E,0XEC,0X5E,0XED,0X5E,0X6D,0X56,0X6D,0X56,0X6C,0X56,0X6C,0X56,0X6C,0X4E,
0X6C,0X4E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0X2D,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0X6D,0X56,0X6C,0X4E,0X6C,0X4E,
0XED,0X5E,0XEC,0X5E,0XEE,0X5E,0X6D,0X4E,0X6D,0X56,0X6D,0X4E,0X6D,0X4E,0X6C,0X4E,
0X6C,0X4E,0XED,0X5E,0X6C,0X4E,0X6C,0X4E,0XEC,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X56,
0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X57,0X2E,0X57,0X2E,0X5F,0X2E,0X57,0X2E,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0XED,0X5E,0XEC,0X5E,0XED,0X5E,0X6C,0X56,0XEE,0X5E,
0XED,0X5E,0XEC,0X5E,0XED,0X5E,0XED,0X5E,0X6C,0X4E,0X6D,0X56,0X6D,0X4E,0X6D,0X4E,
0X6D,0X56,0X6D,0X4E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XEC,0X56,0XED,0X5E,
0X6C,0X4E,0X6C,0X4E,0X2C,0X5F,0XED,0X5E,0XEC,0X5E,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0XED,0X5E,0XED,0X5E,0XEE,0X5E,0X6D,0X4E,0XEC,0X5E,0XED,0X5E,0XED,0X5E,
0XED,0X5E,0XED,0X5E,0XEC,0X5E,0X6D,0X4E,0X6D,0X4E,0X6D,0X4E,0X6D,0X4E,0X6D,0X4E,
0X6C,0X4E,0XED,0X5E,0XEE,0X5E,0X6D,0X56,0XEE,0X66,0XED,0X5E,0XED,0X5E,0XED,0X5E,
0XED,0X5E,0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X57,0X2E,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0XEC,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,
0XED,0X5E,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X6C,0X4E,0X6C,0X4E,0X6C,0X4E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0X2D,0X5F,0X2D,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0XED,0X56,0X2E,0X5F,0X2E,0X5F,0XEE,0X5E,0XAD,0X56,
0X6C,0X4E,0X6C,0X4E,0X6C,0X4E,0XEE,0X5E,0XED,0X5E,0XEE,0X5E,0XED,0X5E,0XED,0X5E,
0XED,0X5E,0X6C,0X4E,0X6D,0X56,0XAD,0X56,0XED,0X5E,0XEC,0X5E,0XED,0X5E,0X2D,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0XED,0X5E,0XEC,0X5E,0XEE,0X5E,0X6D,0X56,0XED,0X5E,
0XED,0X5E,0X6C,0X4E,0X6C,0X4E,0XAD,0X56,0XEE,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,
0XED,0X5E,0XED,0X5E,0X6C,0X4E,0X6C,0X56,0X6C,0X4E,0XED,0X5E,0X2C,0X5F,0XED,0X5E,
0X6D,0X56,0XEE,0X5E,0XED,0X5E,0X2D,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X57,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2E,0X5F,
0X2D,0X5F,0X2D,0X5F,0XED,0X5E,0X6C,0X4E,0X6C,0X4E,0XED,0X5E,0XEC,0X5E,0XED,0X5E,
0X6C,0X4E,0X6C,0X4E,0X6C,0X4E,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XED,0X5E,0XED,0X5E,
0XEE,0X66,0X6C,0X4E,0X6C,0X4E,0X6C,0X4E,0XED,0X5E,0XED,0X5E,0XEC,0X5E,0X2D,0X5F,
0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X57,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X57,0X2E,0X5F,0X2D,0X5F,0XED,0X5E,0X2E,0X5F,0XEE,0X5E,0XED,0X5E,0XED,0X5E,
0X6C,0X4E,0X6C,0X4E,0XAC,0X56,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,
0XAE,0X5E,0X6C,0X4E,0X2C,0X4E,0X6C,0X4E,0X2C,0X57,0XED,0X5E,0XED,0X5E,0X2D,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X57,0X2E,0X57,0X2E,0X5F,0X2E,0X57,0X2E,0X5F,
0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0XEC,0X5E,0XEC,0X5E,0XED,0X5E,0X6D,0X56,0XED,0X5E,
0XED,0X5E,0X6C,0X4E,0X6D,0X4E,0X6C,0X4E,0XEE,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,
0XEE,0X5E,0XEE,0X5E,0X6C,0X4E,0X6C,0X56,0X6C,0X4E,0XED,0X5E,0XEC,0X5E,0XED,0X5E,
0X6D,0X56,0X6D,0X4E,0XEC,0X5E,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0X2D,0X5F,0XEC,0X5E,0XEE,0X5E,0X6D,0X4E,0XED,0X5E,0X2D,0X57,0XED,0X5E,
0X6C,0X4E,0X6C,0X4E,0X6C,0X4E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,
0XEE,0X5E,0XAC,0X56,0X6D,0X56,0X6D,0X56,0XED,0X5E,0XEC,0X5E,0XED,0X5E,0X2D,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X57,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0XED,0X5E,
0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X6C,0X4E,0X6C,0X4E,0XAD,0X56,0XEE,0X5E,0XED,0X5E,0XED,0X56,0X2E,0X5F,0X2D,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0XEC,0X56,0XED,0X5E,0XEE,0X5E,0XEE,0X5E,0XED,0X5E,
0X6C,0X4E,0X6C,0X4E,0XAC,0X4E,0XEC,0X5E,0XEC,0X56,0XEC,0X56,0XEC,0X56,0X2D,0X5F,
0XED,0X5E,0X6C,0X4E,0X6C,0X4E,0XAD,0X56,0XEE,0X5E,0XED,0X5E,0XED,0X5E,0X2D,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X57,0X2E,0X57,0X2E,0X5F,0X2D,0X5F,0XED,0X56,0XED,0X5E,0X6D,0X4E,0XAD,0X5E,
0XED,0X56,0X6C,0X4E,0X6C,0X4E,0X6C,0X4E,0XED,0X5E,0XEC,0X56,0XEC,0X56,0XEC,0X56,
0XEC,0X56,0XED,0X56,0X6C,0X4E,0X6C,0X4E,0X6C,0X4E,0XEE,0X5E,0XED,0X5E,0XED,0X5E,
0X6C,0X4E,0XED,0X5E,0XED,0X56,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0XAC,0X4E,0X6C,0X4E,0XED,0X5E,0XEE,0X5E,0XED,0X5E,
0X6C,0X4E,0X6D,0X56,0X6C,0X4E,0XED,0X5E,0XEC,0X56,0XEC,0X56,0XEC,0X56,0XEC,0X56,
0X2D,0X5F,0X6C,0X4E,0X6C,0X4E,0X6C,0X56,0XEE,0X5E,0XEE,0X5E,0XED,0X5E,0XED,0X5E,
0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X57,
0X2E,0X57,0X2E,0X5F,0X2D,0X5F,0XEC,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0XED,0X5E,
0X6C,0X4E,0X6C,0X4E,0X6C,0X4E,0XED,0X56,0XED,0X5E,0XED,0X56,0XED,0X56,0X2D,0X5F,
0XED,0X5E,0X6C,0X4E,0X6C,0X4E,0XAD,0X56,0XEE,0X5E,0XEE,0X5E,0XED,0X56,0X2D,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X57,0XED,0X5E,0XED,0X5E,0X6D,0X4E,0XEE,0X5E,
0XEE,0X5E,0X6C,0X4E,0X6C,0X56,0X6C,0X4E,0XED,0X5E,0XED,0X5E,0XED,0X56,0XED,0X56,
0XED,0X56,0XEC,0X5E,0X6C,0X4E,0X6C,0X4E,0X6C,0X4E,0XED,0X5E,0XEE,0X5E,0XEE,0X5E,
0X6C,0X4E,0X6B,0X4E,0XED,0X56,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2D,0X5F,0XED,0X5E,0XED,0X5E,0X6B,0X4E,0XEE,0X5E,0XED,0X5E,0XEE,0X5E,
0X6C,0X4E,0X6C,0X4E,0X6C,0X4E,0XED,0X5E,0XED,0X56,0XED,0X56,0XED,0X56,0XEC,0X56,
0X2D,0X5F,0X6C,0X4E,0X6C,0X56,0X6B,0X4E,0XAD,0X5E,0XED,0X5E,0XED,0X5E,0X2E,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X57,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X57,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0XED,0X5E,
0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0XEC,0X56,0X2D,0X5F,0XEE,0X5E,0X6C,0X4E,0X6D,0X56,0X6C,0X4E,0XED,0X5E,0X2D,0X5F,
0X2E,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0XED,0X5E,0XEE,0X5E,0X6D,0X56,0X6D,0X56,0X6C,0X4E,
0XED,0X5E,0XEC,0X5E,0XEC,0X5E,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0XEC,0X5E,0XED,0X5E,0XED,0X5E,0X6C,0X56,0X6C,0X4E,0X6C,0X4E,0X2D,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0XEE,0X5E,0X6D,0X4E,0X6C,0X4E,
0X6C,0X4E,0XEC,0X5E,0XED,0X5E,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0X2D,0X5F,0XED,0X5E,0XED,0X5E,0XED,0X5E,0X6D,0X4E,0X6D,0X56,0X6C,0X4E,
0XED,0X5E,0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2C,0X5F,0XED,0X5E,0X6C,0X4E,0X6C,0X56,0X6C,0X4E,
0XEE,0X5E,0XED,0X5E,0XED,0X5E,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0X2D,0X5F,0XEC,0X5E,0XED,0X5E,0X6C,0X4E,0X6D,0X4E,0X6C,0X4E,0XED,0X56,
0XEC,0X56,0X2D,0X5F,0X2E,0X5F,0X2E,0X57,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0XEC,0X5E,0XEE,0X5E,0X6D,0X56,0X6D,0X4E,0XAC,0X56,
0XED,0X5E,0XEC,0X5E,0X2D,0X57,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0XED,0X5E,0XEC,0X5E,0XED,0X5E,0X6D,0X4E,0X6D,0X4E,0X6C,0X4E,0XED,0X56,
0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0XEC,0X56,0XED,0X5E,0X6C,0X56,0X6D,0X56,
0X6C,0X4E,0XED,0X5E,0XED,0X5E,0XED,0X5E,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0X2D,0X5F,0XED,0X5E,0XEC,0X5E,0XED,0X5E,0X6C,0X4E,0X6D,0X56,0X6C,0X4E,
0XED,0X5E,0XED,0X5E,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0XED,0X5E,0X6C,0X4E,0X6D,0X4E,0X6C,0X4E,
0XED,0X5E,0XED,0X5E,0XED,0X5E,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0XED,0X5E,0XED,0X5E,0XED,0X5E,0X6C,0X4E,0X6D,0X4E,0X6C,0X56,0XEE,0X5E,
0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X57,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2E,0X67,0XED,0X5E,0XEE,0X5E,0XED,0X5E,0XAC,0X4E,0X2D,0X5F,
0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X6C,0X4E,0X6C,0X56,0XEE,0X5E,0XEE,0X5E,0XED,0X5E,
0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2E,0X67,0XED,0X5E,0XEE,0X5E,0XEE,0X5E,0XAC,0X4E,
0XED,0X5E,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0XAC,0X56,0XAC,0X56,0XEE,0X5E,0XEE,0X5E,
0XED,0X5E,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0XEC,0X5E,0X2D,0X5F,0XEE,0X5E,0X6D,0X56,0X6C,0X4E,
0XED,0X5E,0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2D,0X57,0X2D,0X57,0XED,0X5E,0X6D,0X4E,0XEE,0X5E,0XED,0X5E,
0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0XEE,0X5E,0XEE,0X5E,0XEE,0X5E,0XAD,0X56,
0XAC,0X4E,0X2D,0X5F,0X2D,0X5F,0X2E,0X57,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0XAC,0X4E,0XAD,0X56,0XEE,0X5E,0XED,0X5E,0XED,0X5E,
0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0XEE,0X5E,0XEE,0X5E,0XED,0X5E,0XAC,0X56,
0X2E,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0XAC,0X4E,0X6C,0X4E,0XED,0X5E,0XEE,0X5E,
0XED,0X5E,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2D,0X5F,0X2E,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0XED,0X5E,0XED,0X5E,0XED,0X56,0X6D,0X56,0X6C,0X4E,
0XED,0X5E,0XEC,0X5E,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0XED,0X5E,0X6C,0X56,0XAD,0X56,0XED,0X5E,
0XED,0X5E,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0XED,0X56,0XEE,0X5E,0XEE,0X5E,0XAC,0X56,
0XAC,0X4E,0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0XAD,0X56,0XED,0X56,0X2D,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2D,0X5F,0X2E,0X5F,0X2D,0X5F,0XEC,0X56,0XEC,0X5E,0XED,0X5E,0XAC,0X4E,0X2D,0X5F,
0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0XAC,0X4E,0XAC,0X56,0XEC,0X5E,0X2C,0X5F,0XEC,0X56,
0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2D,0X5F,0X2E,0X5F,0X2D,0X5F,0XEC,0X5E,0XEC,0X5E,0XED,0X5E,0XAC,0X4E,
0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X57,0X2E,0X57,0X2E,0X5F,0X2D,0X5F,0X6B,0X46,0X6B,0X4E,0XED,0X5E,0XEC,0X5E,
0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X57,0XED,0X5E,0XED,0X5E,0X6B,0X4E,0X6C,0X4E,
0XED,0X5E,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2D,0X57,0X2D,0X57,0XED,0X5E,0X6C,0X4E,0XED,0X5E,0XEC,0X56,
0X2C,0X57,0XED,0X5E,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0XEC,0X5E,0XEC,0X5E,0X6C,0X4E,
0XAC,0X4E,0X2D,0X5F,0X2D,0X5F,0X2E,0X57,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X6C,0X4E,0X6B,0X4E,0XEC,0X5E,0X2D,0X5F,0XED,0X56,
0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X57,0X2C,0X5F,0XEC,0X5E,0XED,0X5E,0XAC,0X4E,
0X2E,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X6B,0X4E,0X6C,0X4E,0X2C,0X57,0X2D,0X5F,
0XEC,0X56,0X2D,0X5F,0X2D,0X5F,0X2D,0X57,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0XED,0X56,0X2D,0X5F,0XEC,0X56,0X6B,0X4E,0X6B,0X4E,
0XED,0X5E,0XEC,0X5E,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0XED,0X5E,0X6C,0X4E,0X6B,0X4E,0X2D,0X5F,
0XED,0X5E,0X2D,0X57,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0XEC,0X5E,0XED,0X5E,0XAC,0X4E,
0X6B,0X4E,0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0XED,0X5E,0XAC,0X4E,0XAC,0X4E,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X57,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2D,0X5F,
0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X57,0X2E,0X57,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X57,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2E,0X57,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X57,0X2E,0X57,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2E,0X5F,0X2D,0X5F,
0X2D,0X5F,0X2E,0X5F,0X2E,0X57,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,
0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2D,0X5F,0X2D,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2D,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X57,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,0X2E,0X5F,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2D,0X57,0X2D,0X57,0X2D,0X57,0X2D,0X57,0X2D,0X57,
0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,0X2E,0X57,
0X6D,0X4F,0X6D,0X57,0X6D,0X57,0X6D,0X4F,0X6D,0X57,0X6D,0X57,0X6D,0X4F,0X6D,0X57,
0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,0X2D,0X4F,0X6D,0X57,0X6D,0X57,0X6D,0X57,
0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,
0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,
0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,
0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,
0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X4F,0X6D,0X4F,0X6D,0X4F,
0X6D,0X4F,0X6D,0X4F,0X6D,0X4F,0X6D,0X4F,0X6E,0X57,0X6D,0X4F,0X6D,0X4F,0X6D,0X57,
0X6D,0X4F,0X6D,0X57,0X6D,0X57,0X6D,0X4F,0X6D,0X57,0X6D,0X57,0X6D,0X4F,0X6D,0X57,
0X6D,0X57,0X6D,0X4F,0X6D,0X57,0X6D,0X57,0X6D,0X4F,0X6D,0X57,0X6D,0X57,0X6D,0X4F,
0X6D,0X57,0X6D,0X4F,0X6D,0X4F,0X6D,0X57,0X6D,0X4F,0X6D,0X4F,0X6D,0X57,0X6D,0X4F,
0X6D,0X4F,0X6D,0X57,0X6D,0X4F,0X6D,0X4F,0X6D,0X57,0X6D,0X4F,0X6D,0X4F,0X6D,0X57,
0X6D,0X4F,0X6D,0X57,0X6D,0X57,0X6D,0X4F,0X6D,0X57,0X6D,0X57,0X6D,0X4F,0X6E,0X57,
0X6E,0X57,0X6D,0X4F,0X6D,0X57,0X6D,0X57,0X6D,0X4F,0X6D,0X57,0X6D,0X57,0X6D,0X4F,
0X6D,0X57,0X6D,0X4F,0X6D,0X4F,0X6D,0X57,0X6D,0X4F,0X6D,0X4F,0X6E,0X57,0X6D,0X4F,
0X6D,0X4F,0X6D,0X57,0X6D,0X4F,0X6D,0X4F,0X6D,0X57,0X6D,0X4F,0X6D,0X4F,0X6D,0X57,
0X6D,0X4F,0X6D,0X57,0X6D,0X57,0X6D,0X4F,0X6D,0X57,0X6D,0X57,0X6D,0X4F,0X6D,0X57,
0X6D,0X57,0X6D,0X4F,0X6D,0X57,0X6D,0X57,0X6D,0X4F,0X6D,0X57,0X6D,0X57,0X6D,0X4F,
0X6D,0X57,0X6D,0X4F,0X6D,0X4F,0X6D,0X57,0X6D,0X4F,0X6D,0X4F,0X6D,0X57,0X6D,0X4F,
0X6D,0X4F,0X6D,0X57,0X6D,0X4F,0X6D,0X4F,0X6D,0X57,0X6D,0X4F,0X6D,0X4F,0X6D,0X57,
0X6D,0X4F,0X6D,0X57,0X6D,0X57,0X6D,0X4F,0X6D,0X57,0X6D,0X57,0X6D,0X4F,0X6D,0X57,
0X6D,0X57,0X6D,0X4F,0X6E,0X57,0X6E,0X57,0X6D,0X4F,0X6E,0X57,0X6D,0X57,0X6D,0X4F,
0X6D,0X57,0X6D,0X4F,0X6D,0X4F,0X6D,0X57,0X6D,0X4F,0X6D,0X4F,0X6D,0X57,0X6D,0X4F,
0X6D,0X4F,0X6D,0X4F,0X6D,0X4F,0X6D,0X4F,0X6D,0X4F,0X6D,0X4F,0X6D,0X4F,0X6E,0X4F,
0X6D,0X4F,0X6D,0X4F,0X6D,0X4F,0X6D,0X4F,0X6D,0X4F,0X6D,0X4F,0X6D,0X4F,0X6D,0X4F,
0X6D,0X4F,0X6D,0X4F,0X6D,0X4F,0X6E,0X4F,0X6D,0X4F,0X6E,0X4F,0X6E,0X4F,0X6D,0X4F,
0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,0X6D,0X57,
0X6D,0X57,0X6D,0X57,0X6D,0X4F,0X6D,0X4F,0X6D,0X4F,0X6D,0X4F,0X6D,0X57,0X6D,0X57,
0X6D,0X57,0X6D,0X57,0X6D,0X57,0X2D,0X57,0X2D,0X57,0X2D,0X57,0X6E,0X57,0X2D,0X57,
0X6D,0X57,0X6D,0X4F,0X6D,0X57,0X6D,0X57,0X6D,0X4F,0X6E,0X57,0X6E,0X57,0X6D,0X4F,
};
#line 3 "app\\game.c"
#line 1 ".\\app\\RES\\Pic\\flappybird_logo.c"
const unsigned char gImage_flappybird_Logo[9720] = {  
0X1B,0X46,0XFA,0X45,0XF9,0X4D,0XB7,0X5D,0X76,0X6D,0X6A,0X22,0XE9,0X29,0XE9,0X39,
0X09,0X3A,0XC8,0X31,0X08,0X32,0X08,0X32,0XE8,0X31,0X09,0X3A,0XEA,0X39,0XE9,0X39,
0X29,0X32,0X29,0X32,0X09,0X3A,0XE9,0X39,0X29,0X3A,0X2A,0X2A,0X6B,0X1A,0X97,0X6D,
0XD8,0X5D,0XF9,0X4D,0XF9,0X4D,0XFA,0X4D,0XDA,0X4D,0XDA,0X55,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X45,0X1A,0X46,0X19,0X46,0X19,0X46,
0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0X1A,0X46,0XFA,0X45,0XFA,0X4D,0XDA,0X55,0XF9,0X4D,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X55,0XD9,0X55,0XF9,0X4D,0XF9,0X45,0XF9,0X45,
0XFA,0X45,0XFA,0X45,0XDA,0X45,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XF9,0X45,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X19,0X46,0X19,0X46,0XF9,0X45,0XF9,0X4D,0XFA,0X4D,
0X1A,0X56,0XD9,0X5D,0X6B,0X1A,0X2A,0X2A,0X29,0X32,0X08,0X32,0X09,0X42,0XE9,0X39,
0XE9,0X31,0XEA,0X39,0XC9,0X39,0XC9,0X31,0X2B,0X32,0X0A,0X1A,0X98,0X6D,0XD9,0X5D,
0XD9,0X4D,0X1A,0X4E,0XDA,0X45,0X1B,0X4E,0XFA,0X45,0XFA,0X3D,0XFA,0X45,0XFA,0X45,
0XFA,0X55,0XD9,0X4D,0X19,0X46,0X19,0X46,0XF8,0X4D,0XD8,0X4D,0X1A,0X56,0XF9,0X4D,
0XD9,0X4D,0XD9,0X4D,0XF9,0X45,0X19,0X46,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,
0XFA,0X45,0XFA,0X45,0X1A,0X4E,0XB8,0X45,0XF9,0X55,0X97,0X65,0X2A,0X2A,0XE9,0X31,
0X29,0X3A,0XE8,0X31,0X28,0X32,0X08,0X32,0XE8,0X31,0X09,0X32,0X4B,0X32,0XDA,0X3D,
0XFA,0X4D,0XF9,0X5D,0X97,0X65,0X76,0X7D,0X29,0X2A,0X09,0X32,0XE8,0X39,0XE8,0X39,
0XE8,0X39,0X08,0X32,0XE8,0X31,0XC8,0X31,0XC8,0X31,0XC9,0X39,0XC8,0X39,0XC7,0X29,
0X08,0X32,0XC8,0X31,0X2A,0X42,0X87,0X29,0X09,0X2A,0X4A,0X1A,0XB8,0X6D,0XD8,0X55,
0XF9,0X4D,0XF9,0X45,0XFA,0X4D,0XDA,0X4D,0XDA,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X45,0XFA,0X45,0X19,0X46,0X1A,0X4E,
0X19,0X4E,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0X19,0X46,0XFA,0X4D,0XDA,0X4D,0XDA,0X55,0XF9,0X4D,0X19,0X46,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XD9,0X55,0XD9,0X55,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,
0XF9,0X4D,0X1A,0X4E,0XF9,0X4D,0XFA,0X4D,0X1A,0X4E,0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X45,0X19,0X46,0X19,0X46,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XFA,0X4D,
0XB8,0X5D,0X2A,0X1A,0X09,0X2A,0X29,0X3A,0XC7,0X31,0XC7,0X39,0XC8,0X39,0XC8,0X31,
0XA8,0X31,0XA8,0X31,0XC8,0X31,0X09,0X32,0X09,0X22,0X77,0X7D,0X77,0X65,0XF9,0X55,
0XFA,0X4D,0XD9,0X4D,0XDA,0X4D,0X1A,0X4E,0XFA,0X45,0X1A,0X4E,0XFA,0X45,0XFA,0X4D,
0XFA,0X4D,0X19,0X4E,0XF8,0X45,0X19,0X4E,0XF9,0X4D,0XD9,0X45,0XF9,0X4D,0XFA,0X4D,
0XFA,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XFA,0X4D,
0XFA,0X4D,0XD9,0X45,0XF9,0X4D,0XD8,0X4D,0X97,0X65,0X2A,0X2A,0X09,0X3A,0XE8,0X31,
0X08,0X32,0XC7,0X29,0XE8,0X31,0X08,0X3A,0XE8,0X31,0XE9,0X29,0XF9,0X4D,0XB8,0X4D,
0X56,0X65,0X4A,0X1A,0XE8,0X21,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC9,0X41,0XC8,0X41,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XEF,0X4A,0X1A,0XB8,0X6D,0XD8,0X55,0XF9,0X4D,
0X19,0X46,0X19,0X46,0XFA,0X45,0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XF9,0X4D,0XFA,0X4D,0XFA,0X4D,
0XD9,0X4D,0XD9,0X55,0XD9,0X55,0XF9,0X4D,0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XDA,0X55,0XDA,0X4D,0XFA,0X45,0X19,0X46,0X1A,0X4E,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0X19,0X4E,0XD9,0X45,
0XF9,0X45,0XF9,0X45,0XD8,0X45,0XD9,0X4D,0XD9,0X45,0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,
0XF9,0X45,0X19,0X46,0X19,0X3E,0XF9,0X45,0XD9,0X4D,0XD9,0X55,0XD8,0X4D,0XB8,0X65,
0X8B,0X32,0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X08,0X2A,0X6B,0X22,0XB7,0X65,0XB8,0X4D,
0XD9,0X55,0XD9,0X55,0XB9,0X4D,0XDA,0X4D,0XFA,0X45,0XF9,0X45,0XD9,0X45,0XD9,0X45,
0XF9,0X4D,0XFA,0X4D,0XD9,0X4D,0XFA,0X4D,0XF9,0X4D,0X1A,0X4E,0XFA,0X4D,0XFA,0X4D,
0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XFA,0X55,
0XDA,0X4D,0XB9,0X4D,0XF8,0X55,0X97,0X65,0X09,0X2A,0XBF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0X09,0X3A,0X0A,0X32,0XB8,0X55,0XF8,0X6D,0X8B,0X22,
0XFF,0XEF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFE,0XFF,0XDE,0XF7,0XDE,0XF7,
0XFF,0XFF,0XBE,0XF7,0XFF,0XFF,0XC8,0X49,0XC7,0X41,0XFF,0XFF,0XFE,0XFF,0XDF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XFF,0XF7,0X4A,0X22,0X97,0X6D,0XD9,0X55,0X19,0X46,0X19,0X46,
0X19,0X46,0XFA,0X45,0XFA,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XDA,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X55,
0XD9,0X55,0XD9,0X55,0XF9,0X4D,0XFA,0X4D,0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X45,0X1A,0X46,0X1A,0X4E,0XFA,0X4D,0XFA,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X19,0X4E,
0XF9,0X4D,0XF8,0X4D,0X19,0X4E,0X19,0X4E,0XD9,0X45,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,
0X1A,0X46,0X1A,0X46,0XF9,0X45,0XD9,0X4D,0XD9,0X55,0XD8,0X55,0XB8,0X6D,0X2A,0X2A,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFE,0XFF,0X9D,0XF7,0XFF,0XFF,0X9E,0XEF,0XFF,0XF7,0X8B,0X12,0XD8,0X65,0X98,0X55,
0XD9,0X55,0XFA,0X55,0XD9,0X4D,0XF9,0X45,0X19,0X4E,0X19,0X46,0XD8,0X45,0XFA,0X4D,
0XD9,0X4D,0XFA,0X4D,0XD9,0X45,0XFA,0X4D,0XB9,0X45,0XF9,0X4D,0XF9,0X4D,0XFA,0X4D,
0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XDA,0X4D,0XDA,0X4D,
0XFA,0X4D,0XB8,0X55,0XB8,0X75,0XE9,0X29,0XDF,0XFF,0XBF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XE8,0X39,0X09,0X32,0X56,0X6D,0X56,0X75,0X4A,0X2A,0XBF,0XEF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XDE,0XF7,
0XFF,0XFF,0XDF,0XFF,0XA7,0X41,0XE7,0X49,0XFE,0XFF,0XFE,0XFF,0XDE,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF7,0X4B,0X22,0XB8,0X6D,0XD9,0X55,0X19,0X46,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X55,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X19,0X46,0XFA,0X45,
0XFA,0X4D,0XFA,0X4D,0XFA,0X45,0XFA,0X45,0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,0XFA,0X45,
0XFA,0X45,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XD9,0X4D,0XD9,0X45,0XF9,0X4D,
0XF9,0X45,0XD8,0X45,0XF8,0X45,0XF9,0X45,0XD9,0X45,0XD9,0X4D,0XDA,0X4D,0XDA,0X4D,
0XFA,0X45,0XD9,0X45,0XD9,0X4D,0XD8,0X4D,0X19,0X66,0XB7,0X6D,0X4A,0X32,0XFF,0XFF,
0XFF,0XFF,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,
0XFE,0XF7,0XFE,0XF7,0XDE,0XF7,0XDF,0XEF,0X4A,0X2A,0X56,0X75,0XB7,0X65,0XD8,0X55,
0XD8,0X4D,0XF8,0X4D,0XF8,0X45,0X19,0X4E,0XD8,0X45,0X19,0X4E,0XF9,0X45,0XFA,0X4D,
0XFA,0X4D,0XFA,0X4D,0XF9,0X4D,0X1A,0X4E,0XD9,0X4D,0XD9,0X4D,0XFA,0X4D,0XFA,0X4D,
0XDA,0X4D,0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,0XFA,0X4D,0XFA,0X4D,0X99,0X45,0X1A,0X56,
0XD9,0X5D,0X57,0X6D,0XC9,0X31,0XBF,0XFF,0XFF,0XFF,0XFF,0XF7,0XDE,0XF7,0XDF,0XFF,
0XDF,0XFF,0XC8,0X39,0X09,0X32,0XF1,0X5B,0X6A,0X2A,0XFF,0XF7,0XDF,0XF7,0XFE,0XF7,
0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFF,0XFF,0X87,0X41,0XC7,0X41,0XDE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XF7,0X4B,0X22,0X98,0X6D,0XD9,0X55,0XF9,0X4D,0XF9,0X4D,0XD9,0X55,0XD9,0X4D,
0XD9,0X55,0XD9,0X55,0XD9,0X55,0XF9,0X55,0XF9,0X4D,0XD9,0X55,0XD9,0X55,0XD9,0X55,
0XD9,0X55,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X55,0XD9,0X55,0XD9,0X55,0XD9,0X55,
0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XFA,0X4D,0XDA,0X4D,
0XDA,0X4D,0XFA,0X4D,0XF9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XF9,0X4D,
0XD9,0X4D,0XD9,0X4D,0XDA,0X4D,0XDA,0X4D,0XD9,0X4D,0XFA,0X55,0XB8,0X55,0XD8,0X5D,
0XB7,0X5D,0XB7,0X55,0XD8,0X5D,0XB8,0X5D,0XB9,0X65,0XB9,0X65,0XB9,0X65,0XD9,0X5D,
0XD9,0X5D,0XB8,0X5D,0XB8,0X65,0X97,0X65,0X76,0X7D,0X29,0X3A,0XFF,0XFF,0XFF,0XFF,
0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XDE,0XF7,0XFF,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,
0XFD,0XF7,0XFF,0XFF,0XFF,0XFF,0XDF,0XF7,0X2A,0X2A,0XEC,0X22,0X96,0X65,0XD8,0X65,
0XD8,0X5D,0XB7,0X5D,0XD8,0X5D,0XD8,0X5D,0XB8,0X55,0XD9,0X4D,0XFA,0X4D,0XB9,0X4D,
0X1A,0X56,0XF9,0X4D,0XD9,0X4D,0XF9,0X55,0XD9,0X55,0XD9,0X55,0XD9,0X4D,0XD9,0X55,
0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,0XFA,0X4D,0XF9,0X45,0XFA,0X4D,0XD9,0X4D,0X98,0X55,
0X97,0X75,0XE9,0X31,0XDF,0XFF,0XBF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XC7,0X39,0X2A,0X3A,0X6F,0X63,0X09,0X32,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFE,0XF7,
0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XBE,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,
0XC7,0X41,0XC7,0X41,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,
0X2B,0X22,0X98,0X75,0XB9,0X55,0XD9,0X55,0XD8,0X5D,0XB8,0X5D,0XD8,0X55,0XD8,0X55,
0XB8,0X5D,0XB8,0X5D,0XD8,0X55,0XD8,0X55,0XB8,0X55,0XB8,0X55,0XB9,0X55,0XB9,0X55,
0XB9,0X55,0XD9,0X55,0XD9,0X55,0XD9,0X55,0XB8,0X55,0XB8,0X55,0XB9,0X55,0XD9,0X55,
0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XD9,0X55,0XB9,0X5D,0XB9,0X5D,
0XB9,0X55,0XD9,0X55,0XD8,0X5D,0XB8,0X5D,0XB8,0X5D,0XB8,0X5D,0XB8,0X55,0XD9,0X4D,
0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,0XB9,0X4D,0X78,0X5D,0XAC,0X12,0X4A,0X1A,0X8B,0X2A,
0X6A,0X22,0X4A,0X22,0X4A,0X22,0X4A,0X2A,0X4B,0X2A,0X4B,0X2A,0X4B,0X22,0X6B,0X22,
0X6A,0X22,0X6A,0X22,0X6B,0X2A,0X0A,0X2A,0XE8,0X39,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0X29,0X3A,0XE9,0X29,0X2A,0X2A,0X6B,0X2A,0X4B,0X22,
0X4A,0X22,0X6B,0X22,0X2B,0X22,0X6C,0X1A,0XB9,0X5D,0XD9,0X4D,0XFA,0X5D,0X98,0X4D,
0X19,0X5E,0XF8,0X55,0XD8,0X55,0XD8,0X55,0XB8,0X55,0XD9,0X55,0XD9,0X55,0XB9,0X55,
0XD8,0X55,0XD8,0X4D,0XF9,0X45,0X19,0X46,0XF9,0X4D,0XF9,0X55,0XD8,0X5D,0X77,0X75,
0XC9,0X31,0XBF,0XFF,0XFF,0XFF,0XDE,0XF7,0XFF,0XFF,0XDE,0XF7,0XFF,0XFF,0X08,0X3A,
0XE9,0X31,0X6F,0X6B,0X09,0X3A,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,0XFE,0XF7,0XFE,0XFF,
0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XBE,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XA7,0X41,
0XC7,0X41,0XFF,0XFF,0XFE,0XF7,0XFE,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0X2B,0X22,
0X78,0X75,0XB8,0X5D,0XB8,0X5D,0X97,0X6D,0X77,0X6D,0X97,0X6D,0X97,0X6D,0X77,0X75,
0X77,0X6D,0X97,0X6D,0X97,0X6D,0X77,0X6D,0X77,0X6D,0X77,0X6D,0X77,0X6D,0X77,0X6D,
0X77,0X6D,0X77,0X6D,0X77,0X6D,0X77,0X6D,0X77,0X6D,0X77,0X6D,0X97,0X6D,0X97,0X65,
0XB8,0X5D,0XD8,0X55,0XD8,0X55,0XD8,0X55,0X98,0X5D,0X78,0X6D,0X57,0X75,0X77,0X6D,
0X77,0X6D,0X76,0X6D,0X76,0X6D,0X77,0X75,0X77,0X6D,0X77,0X6D,0X98,0X65,0XB8,0X5D,
0XD8,0X55,0XF9,0X55,0X98,0X55,0XB8,0X75,0X4B,0X22,0X29,0X2A,0X29,0X3A,0XE8,0X31,
0X29,0X3A,0X08,0X3A,0X08,0X3A,0XE9,0X41,0XE9,0X41,0X09,0X3A,0X09,0X32,0X28,0X32,
0X29,0X32,0X4A,0X3A,0X09,0X3A,0XE9,0X49,0XDF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDE,0XF7,0XDF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XE8,0X41,0X09,0X42,0XE9,0X41,0X0A,0X3A,0XE9,0X31,0X2A,0X3A,
0XE9,0X39,0X0B,0X3A,0X4C,0X2A,0XB8,0X65,0XB8,0X55,0XB9,0X5D,0XB8,0X6D,0X76,0X65,
0X96,0X6D,0X76,0X6D,0X76,0X6D,0X77,0X6D,0X77,0X6D,0X77,0X6D,0X77,0X6D,0X78,0X6D,
0X98,0X65,0XD8,0X55,0X19,0X56,0XD8,0X5D,0X97,0X5D,0X76,0X6D,0X56,0X85,0XE9,0X39,
0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,0XE7,0X39,0X09,0X32,
0X6F,0X6B,0XE9,0X39,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XDE,0XFF,0XFF,0XFF,
0XC7,0X39,0XC7,0X39,0X08,0X4A,0XA7,0X39,0X08,0X4A,0XC8,0X41,0XC8,0X49,0XC8,0X41,
0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0X4A,0X22,0X77,0X75,
0XB8,0X65,0XF9,0X75,0X0A,0X22,0X0A,0X32,0XE9,0X29,0X0A,0X32,0X0A,0X32,0X09,0X32,
0X29,0X32,0X29,0X32,0XE9,0X31,0XC8,0X31,0X09,0X3A,0X2A,0X3A,0XC8,0X31,0XE9,0X31,
0XE9,0X31,0XE8,0X29,0XE9,0X31,0XC8,0X29,0X29,0X32,0XE8,0X29,0X09,0X2A,0X35,0X7D,
0X97,0X6D,0XF8,0X65,0XD8,0X5D,0X97,0X65,0X4B,0X2A,0XEA,0X31,0XC9,0X31,0XE9,0X31,
0X08,0X2A,0XE8,0X21,0X4A,0X3A,0XE9,0X31,0XEA,0X31,0X2A,0X2A,0X36,0X6D,0XB7,0X65,
0XB8,0X55,0XB8,0X5D,0X77,0X75,0X2A,0X22,0XFF,0XF7,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XA7,0X41,0XC8,0X49,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XFF,0XFF,0XE7,0X41,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0X48,0X4A,0X07,0X42,0XBE,0XF7,0XDF,0XFF,0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,
0XDF,0XFF,0XE8,0X49,0XC8,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,
0XFF,0XFF,0X6B,0X32,0XD8,0X6D,0XB8,0X5D,0X78,0X75,0X2B,0X1A,0XC8,0X21,0XE8,0X31,
0XE8,0X31,0X29,0X32,0XE8,0X29,0XE9,0X31,0XC9,0X31,0X2A,0X3A,0X0B,0X32,0X4B,0X22,
0XB7,0X65,0XB7,0X5D,0X55,0X6D,0X29,0X22,0X09,0X32,0XE8,0X39,0XE8,0X39,0XDF,0XFF,
0XFF,0XFF,0XFE,0XF7,0XFF,0XF7,0XFE,0XF7,0XFF,0XFF,0XE8,0X39,0X09,0X2A,0X8F,0X63,
0X09,0X32,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDE,0XFF,0XDE,0XFF,0XFF,0XFF,0XBE,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0X9F,0XFF,0X09,0X52,0XC8,0X41,0XFE,0XFF,
0XFE,0XFF,0XFF,0XFF,0XDE,0XFF,0XDE,0XF7,0XFF,0XF7,0X6B,0X32,0X56,0X7D,0X76,0X75,
0XCC,0X2A,0XFF,0XF7,0XBF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XBF,0XFF,0XBF,0XFF,0XC8,0X49,0X9F,0XFF,0XDF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XBF,0XFF,0XFF,0XFF,0XDF,0XFF,0XAB,0X42,0X4A,0X1A,
0XAB,0X12,0X76,0X6D,0X76,0X7D,0XC8,0X29,0XBF,0XFF,0X9F,0XFF,0XDF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XBF,0XFF,0XDF,0XFF,0X9F,0XFF,0XFF,0XFF,0X29,0X2A,0X6A,0X1A,0X8B,0X0A,
0XB8,0X75,0X15,0X7D,0X29,0X2A,0XFF,0XF7,0XFF,0XFF,0XDE,0XFF,0XDE,0XFF,0XFF,0XFF,
0XDF,0XFF,0XA7,0X41,0XC8,0X49,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,
0XFF,0XFF,0XE7,0X41,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XE7,0X39,0XE7,0X39,0XBE,0XFF,0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XE8,0X49,0XA7,0X41,0XBF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0X2A,0X32,0X76,0X75,0X77,0X6D,0X2A,0X22,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XDF,0XFF,0XBF,0XFF,0XCA,0X49,0XC9,0X31,0X56,0X75,
0XAA,0X1A,0X29,0X2A,0XFF,0XF7,0XDF,0XFF,0XDF,0XFF,0XBF,0XF7,0XFF,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XE8,0X39,0X09,0X2A,0XAF,0X5B,0X28,0X32,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XDF,0XFF,0XBE,0XFF,0XFF,0XFF,0X87,0X39,0XC7,0X41,0XFF,0XFF,0XFE,0XFF,
0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0XDE,0XF7,0X4A,0X3A,0XF5,0X8C,0X35,0X85,0X29,0X2A,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XDE,0XF7,
0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0X86,0X39,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X08,0X32,0XE9,0X29,0X0A,0X2A,
0XD4,0X7C,0X56,0X95,0XA7,0X31,0XDF,0XFF,0XFF,0XFF,0XDE,0XFF,0XDE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XBF,0XF7,0X29,0X3A,0X09,0X32,0X4A,0X32,0XF5,0X84,
0XF4,0X8C,0X29,0X3A,0XFF,0XF7,0XFF,0XF7,0XFE,0XF7,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,
0XA7,0X41,0XE8,0X49,0XFF,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,
0X07,0X3A,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XDF,0XFF,0XDF,0XFF,0XDE,0XFF,0X07,0X42,
0XE7,0X41,0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XBF,0XFF,0XE8,0X49,
0XE8,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XBF,0XF7,0X4A,0X3A,
0X15,0X85,0X36,0X8D,0XC9,0X29,0XDF,0XF7,0XFF,0XF7,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,
0XFE,0XFF,0XFF,0XFF,0XBE,0XF7,0XFF,0XFF,0XC9,0X41,0XC9,0X39,0X35,0X8D,0X49,0X2A,
0X08,0X3A,0XDF,0XFF,0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XE8,0X39,0XE9,0X29,0XAF,0X5B,0X28,0X2A,0XFF,0XF7,
0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XF7,0XFF,0XF7,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0X08,0X4A,0XC7,0X41,0XFF,0XFF,0XFE,0XFF,0XFE,0XF7,
0XFF,0XF7,0XFE,0XF7,0XFF,0XFF,0XE8,0X39,0XE8,0X39,0X29,0X3A,0XDF,0XF7,0XFF,0XFF,
0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XE7,0X41,0XBE,0XFF,0XFE,0XFF,0XDE,0XF7,0XFF,0XFF,0XFF,0XFF,
0XDE,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XDF,0XFF,0X09,0X3A,
0X09,0X42,0XC8,0X41,0X9E,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XA7,0X31,0X08,0X42,
0X28,0X42,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XC7,0X41,
0XE7,0X49,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XF7,0XDF,0XF7,0XE7,0X39,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFF,0XFF,0XDE,0XFF,0XDF,0XFF,0XC8,0X49,0XA6,0X39,
0XFF,0XFF,0XFE,0XF7,0XFE,0XFF,0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XE8,0X39,0XE8,0X31,
0XE8,0X31,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,
0XFE,0XFF,0XFF,0XF7,0XFF,0XFF,0XC8,0X41,0XA8,0X41,0XE8,0X39,0XFF,0XF7,0XDF,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,
0XFE,0XFF,0XFF,0XFF,0XE8,0X39,0XE9,0X31,0XAF,0X5B,0X29,0X32,0XFF,0XF7,0XFF,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFF,0XF7,0XFF,0XFF,0XFE,0XF7,0XFF,0XF7,0XFE,0XF7,
0XFF,0XF7,0XFF,0XFF,0XC8,0X41,0XA7,0X41,0XFF,0XFF,0XFE,0XF7,0XFF,0XFF,0XFE,0XF7,
0XFE,0XFF,0XFF,0XFF,0X08,0X42,0XFF,0XFF,0XBE,0XF7,0XFF,0XFF,0XFE,0XFF,0XFD,0XFF,
0XFE,0XFF,0XFE,0XF7,0XFD,0XF7,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XDE,0XFF,
0XFF,0XFF,0XA6,0X39,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFE,0XFF,0XFF,0XFF,
0XFE,0XFF,0XFF,0XFF,0XFE,0XF7,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XBF,0XF7,0X9F,0XF7,
0XC8,0X49,0XDF,0XFF,0XDE,0XFF,0XFF,0XFF,0XDE,0XF7,0XDE,0XF7,0XFF,0XFF,0XFF,0XFF,
0XDE,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XC6,0X39,
0XFF,0XFF,0XDE,0XF7,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XDF,0XFF,0XA7,0X41,0XE7,0X49,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XE7,0X41,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,0XFE,0XF7,
0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XDE,0XFF,0XFF,0XFF,0XE8,0X49,0XE7,0X41,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XE8,0X41,0XDF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFE,0XFF,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,
0XFF,0XF7,0XFF,0XFF,0XC8,0X41,0X09,0X4A,0X9F,0XFF,0XBF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDE,0XF7,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XE8,0X39,0XE9,0X31,0X8F,0X5B,0X09,0X32,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFE,0XF7,0XFE,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,
0XFF,0XFF,0XE8,0X41,0XE8,0X49,0XDE,0XFF,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0XDE,0XFF,
0XFF,0XFF,0XE7,0X41,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XBD,0XF7,
0XFF,0XFF,0XFE,0XFF,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDE,0XFF,
0XC7,0X39,0XFF,0XFF,0XDE,0XFF,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XDE,0XF7,0XFE,0XFF,
0XFE,0XFF,0XFF,0XFF,0XDE,0XF7,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC8,0X49,
0XBF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XDE,0XF7,0XFE,0XFF,0XFF,0XFF,
0XDE,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XBE,0XF7,0X9E,0XFF,0XE7,0X41,0XFF,0XFF,
0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XA7,0X41,0XE7,0X49,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XBE,0XFF,0XE7,0X41,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,0XFF,0XF7,
0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XDF,0XFF,0XE8,0X41,0XE8,0X41,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XE8,0X49,0X9E,0XFF,0XDF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFF,0XF7,0XDE,0XF7,
0XDF,0XFF,0XE8,0X49,0X87,0X39,0XFF,0XFF,0XBF,0XFF,0XDE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFF,0XFF,0XDE,0XF7,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XE8,0X39,0XE9,0X31,0X8F,0X63,0X09,0X3A,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFF,0XF7,0XFE,0XF7,0XFF,0XF7,0XFF,0XF7,0XDF,0XFF,
0XC8,0X49,0XA7,0X41,0XDF,0XFF,0XFF,0XFF,0XDF,0XF7,0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,
0XC7,0X39,0XDF,0XFF,0XFF,0XFF,0XDE,0XF7,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,
0XBE,0XF7,0XFF,0XFF,0XFE,0XF7,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XC7,0X39,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XDE,0XFF,0XA7,0X41,0XBF,0XFF,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDE,0XF7,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,
0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XDE,0XFF,0XDF,0XFF,0XC7,0X41,0XFF,0XFF,0XDE,0XF7,
0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XFF,0XFF,0XA7,0X49,0XC7,0X49,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XBE,0XF7,0X07,0X42,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFF,0XFF,0XFE,0XFF,0XDE,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0XE8,0X49,0XA7,0X39,0XC8,0X41,0X09,0X4A,0XA7,0X41,
0XA8,0X41,0XE8,0X49,0XE8,0X49,0XC7,0X41,0XFF,0XFF,0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDE,0XFF,0XDE,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XDF,0XFF,
0XE8,0X49,0XC7,0X41,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XDE,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XE8,0X39,
0XE9,0X29,0X70,0X63,0XE9,0X39,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0X87,0X41,
0XC8,0X49,0XFF,0XFF,0XDE,0XF7,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0X08,0X42,
0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,0XDE,0XF7,0XFF,0XFF,0XFF,0XFF,0X08,0X42,0XA6,0X31,
0XDE,0XF7,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XC7,0X39,0XFF,0XFF,
0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0X07,0X42,0XC6,0X39,0XFF,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XE7,0X41,0X9E,0XFF,0XFF,0XFF,
0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,0XDE,0XF7,0X07,0X42,0X27,0X42,0XFE,0XFF,0XFF,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XC7,0X41,0XFF,0XFF,0XFF,0XFF,0XDE,0XF7,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XA7,0X41,0XC7,0X49,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XBD,0XF7,0X07,0X42,0XFF,0XFF,0XDE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFF,0XFF,0XFE,0XFF,
0XFF,0XFF,0XDF,0XFF,0XC7,0X41,0XA7,0X39,0XDF,0XFF,0X9F,0XFF,0XBF,0XFF,0XBF,0XFF,
0XDF,0XFF,0XBF,0XFF,0XC7,0X41,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XDE,0XFF,
0XFF,0XFF,0XFF,0XFF,0XDE,0XFF,0X08,0X42,0XE8,0X49,0X87,0X41,0XE8,0X51,0XE8,0X49,
0XA7,0X41,0XFF,0XFF,0XFE,0XFF,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0X28,0X42,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XE8,0X39,0X09,0X2A,
0X8F,0X63,0X09,0X32,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XDF,0XFF,0XBF,0XFF,0XDF,0XFF,0XA8,0X41,0XC8,0X41,
0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X08,0X3A,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XE8,0X41,0XA7,0X39,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XC7,0X39,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XE7,0X41,0XC7,0X41,0XFF,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XC7,0X39,0XDF,0XFF,0XDF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0X07,0X42,0XE7,0X39,0XDE,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XC7,0X39,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XF7,0XFF,0XFF,0XE8,0X41,0XE8,0X49,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFE,0XFF,0XDE,0XFF,0XE7,0X41,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,
0XFF,0XFF,0X08,0X42,0XC7,0X41,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XE7,0X41,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XBF,0XFF,0XC8,0X49,0XC8,0X49,0X88,0X49,0XA8,0X49,0XE8,0X49,0XC7,0X41,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XA7,0X39,0XDF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XE8,0X39,0X09,0X2A,0X8F,0X63,
0X09,0X32,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X09,0X42,
0XA7,0X39,0XE8,0X49,0XC8,0X49,0XC8,0X49,0XE9,0X49,0X09,0X52,0XC8,0X41,0XFF,0XFF,
0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0X08,0X42,0XFF,0XFF,0XFF,0XFF,
0XFE,0XFF,0XFE,0XFF,0XDE,0XFF,0XFF,0XFF,0XE8,0X41,0XA7,0X39,0XFF,0XFF,0XFF,0XFF,
0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XA7,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,
0XFF,0XFF,0XDE,0XF7,0XDE,0XFF,0X07,0X4A,0XE7,0X41,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XC7,0X41,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFE,0XFF,0XFF,0XFF,0XE7,0X41,0X07,0X42,0XFF,0XFF,0XDE,0XF7,0XFF,0XFF,0XFE,0XF7,
0XDE,0XF7,0XDF,0XFF,0XE7,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XE7,0X41,0XE7,0X41,0XDF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XDE,0XFF,0XE8,0X41,0XFF,0XFF,0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,
0XE8,0X41,0XC7,0X41,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XE7,0X41,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XE8,0X49,
0XE8,0X49,0XE9,0X49,0XC9,0X41,0X0A,0X42,0XC9,0X39,0XE9,0X49,0XA7,0X39,0XFF,0XFF,
0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XC8,0X41,0XBF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XE8,0X39,0X09,0X2A,0X8F,0X63,0X08,0X32,
0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XC7,0X39,0X09,0X4A,
0XC8,0X41,0XC8,0X41,0XC9,0X41,0XC8,0X41,0XA8,0X41,0XC7,0X41,0XFF,0XFF,0XFE,0XF7,
0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XDF,0XFF,0XE8,0X41,0XDF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XDE,0XFF,0XBF,0XFF,0XE8,0X49,0XE8,0X41,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XA7,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,
0XFF,0XF7,0XFE,0XFF,0XE7,0X41,0X08,0X42,0XFE,0XF7,0XFF,0XF7,0XFE,0XF7,0XFE,0XF7,
0XFF,0XFF,0XFF,0XFF,0XE8,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFF,0XFF,0X08,0X42,0XE7,0X41,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XF7,0XFF,0XFF,
0XDF,0XFF,0XC7,0X41,0XFF,0XFF,0XDE,0XF7,0XFF,0XFF,0XDE,0XF7,0XFF,0XFF,0XFE,0XFF,
0XFF,0XFF,0XDE,0XF7,0XFF,0XFF,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XDE,0XFF,
0XE8,0X41,0XFF,0XFF,0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0X08,0X3A,
0XE7,0X39,0XBE,0XF7,0XFF,0XFF,0XFE,0XFF,0XDE,0XF7,0XFF,0XFF,0XDF,0XFF,0XE8,0X49,
0XC7,0X41,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XE7,0X41,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XC8,0X41,0X09,0X4A,
0X4B,0X3A,0X36,0X8D,0X36,0X75,0X76,0X85,0XE9,0X31,0XC8,0X41,0XFF,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XA6,0X39,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XE8,0X39,0X09,0X2A,0X8F,0X63,0X09,0X32,0XFF,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XE8,0X41,0XE8,0X39,0X09,0X32,
0X2A,0X32,0X4B,0X32,0XE9,0X31,0X09,0X42,0XC7,0X41,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,
0XDF,0XFF,0XDF,0XFF,0XDF,0XFF,0XE8,0X41,0XDF,0XFF,0XDE,0XFF,0XDE,0XFF,0XFF,0XFF,
0XDE,0XFF,0XDF,0XFF,0XC7,0X41,0XE7,0X39,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,
0XFE,0XFF,0XFF,0XFF,0XA7,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,
0XFF,0XFF,0XE7,0X41,0XE7,0X41,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFE,0XF7,0XDF,0XFF,
0XFF,0XFF,0XE8,0X41,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,
0XC7,0X39,0XE8,0X41,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XF7,0XDE,0XF7,0XDF,0XFF,
0XE8,0X49,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFE,0XFF,0XDE,0XFF,0XDE,0XF7,0XFF,0XF7,
0XFF,0XF7,0XFF,0XF7,0XFE,0XF7,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XDE,0XF7,0XE7,0X41,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XE7,0X39,0XE7,0X41,
0XDF,0XFF,0XDE,0XFF,0XDE,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XE8,0X49,0XC7,0X41,
0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XE7,0X41,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XC8,0X41,0X0A,0X3A,0X2A,0X22,
0XB8,0X75,0XF8,0X65,0X97,0X6D,0X2A,0X32,0XA8,0X39,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,
0XFF,0XFF,0XFE,0XF7,0XDE,0XF7,0XC6,0X39,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XE8,0X39,0X09,0X2A,0X90,0X5B,0X09,0X32,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0X29,0X3A,0X77,0X95,0X76,0X6D,0X97,0X65,
0XD8,0X65,0X56,0X6D,0X09,0X32,0XC8,0X41,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XE8,0X41,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XDE,0XFF,
0XFF,0XFF,0XE7,0X41,0XE7,0X41,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFF,0XFF,0XA7,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA7,0X41,0X86,0X39,0XFF,0XFF,0XDE,0XF7,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,
0XC7,0X41,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XF7,0XFF,0XFF,0X08,0X42,
0XE7,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XDE,0XFF,0XDF,0XFF,0XA7,0X41,
0XFF,0XFF,0XFE,0XF7,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XDE,0XF7,0XE7,0X39,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XDE,0XFF,0XE7,0X41,0XC7,0X41,0XDE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFF,0XF7,0XFF,0XFF,0XE8,0X41,0XC7,0X41,0XFF,0XFF,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XE7,0X41,0XFF,0XFF,0XFF,0XFF,
0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XC9,0X41,0X2B,0X32,0X98,0X75,0XB8,0X55,
0XD8,0X4D,0XD8,0X65,0X4A,0X32,0XA7,0X39,0XDF,0XFF,0XDE,0XFF,0XFE,0XF7,0XFE,0XF7,
0XFF,0XFF,0XFF,0XFF,0XE7,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,
0XFF,0XFF,0X08,0X3A,0X09,0X2A,0X90,0X5B,0X09,0X32,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0X0A,0X32,0X36,0X7D,0XB8,0X5D,0XF9,0X4D,0XF8,0X4D,
0XD8,0X6D,0XC9,0X21,0XE8,0X41,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0X07,0X42,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XDE,0XF7,0XFE,0XFF,0XFE,0XFF,
0XDE,0XFF,0XBE,0XF7,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XDE,0XFF,0XDF,0XFF,
0XA7,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDE,0XF7,0XFE,0XFF,0XFF,0XFF,0XBE,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XC7,0X39,
0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFD,0XF7,0XFE,0XF7,0XFF,0XFF,0XDE,0XF7,0XDF,0XFF,
0XFF,0XFF,0XBE,0XF7,0XFE,0XFF,0XFF,0XFF,0XDE,0XFF,0XDF,0XFF,0XA7,0X41,0X08,0X4A,
0X07,0X42,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFE,0XFF,0XDE,0XF7,0X07,0X42,0XFF,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFF,0XFF,0XFF,0XF7,0XDF,0XFF,0XE8,0X41,0XC7,0X41,0XFF,0XFF,0XDE,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XE7,0X41,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,
0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0X09,0X42,0X4B,0X2A,0X98,0X5D,0XF9,0X4D,0XF8,0X45,
0XF8,0X65,0X09,0X22,0X09,0X4A,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XDE,0XFF,
0XDE,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,
0X08,0X3A,0X09,0X2A,0X90,0X5B,0X09,0X32,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFF,0XF7,0X2A,0X32,0X57,0X85,0XB9,0X65,0XD9,0X4D,0XD9,0X4D,0XB8,0X65,
0X0A,0X2A,0XE8,0X41,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0X07,0X42,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XDE,0XF7,0XFE,0XFF,0XDE,0XF7,
0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDE,0XFF,0XDF,0XFF,0XA7,0X41,
0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XBE,0XF7,0XFF,0XFF,
0XDE,0XF7,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC7,0X39,0XFF,0XFF,
0XFF,0XFF,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XDE,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFE,0XF7,0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0X87,0X39,0XA8,0X41,0XE8,0X49,
0XDF,0XFF,0XFF,0XFF,0XDE,0XFF,0XFE,0XFF,0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XDE,0XFF,0XE7,0X41,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XDD,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFF,0XF7,0XDF,0XFF,0XE8,0X41,0XC7,0X39,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XE7,0X41,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFF,0XFF,0XE9,0X39,0X4A,0X2A,0XB9,0X5D,0X1A,0X4E,0XF8,0X45,0XB8,0X65,
0X6A,0X3A,0XA7,0X39,0XDF,0XFF,0XFE,0XFF,0XBE,0XF7,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XE8,0X39,
0X09,0X2A,0X8F,0X63,0X09,0X32,0XFF,0XFF,0XDF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFF,0XF7,0X0A,0X32,0X37,0X7D,0X99,0X5D,0XDA,0X4D,0XB9,0X45,0XB8,0X65,0X0A,0X2A,
0XC8,0X39,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XE8,0X41,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XA7,0X41,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XDF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XA7,0X41,0XFF,0XFF,0XFF,0XFF,
0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,
0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X09,0X42,0XE9,0X41,0XE9,0X41,0XE9,0X41,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XBE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,
0XFF,0XFF,0XFE,0XFF,0XDE,0XFF,0XE7,0X41,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XE8,0X49,0XC7,0X39,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XC7,0X41,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XDE,0XFF,
0XFF,0XFF,0XE8,0X39,0X6B,0X2A,0XD8,0X5D,0XFA,0X45,0XF9,0X4D,0X98,0X65,0X6B,0X3A,
0XE8,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XF7,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XE8,0X39,0X09,0X2A,
0XAF,0X63,0XE8,0X31,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XF7,
0X09,0X2A,0X77,0X7D,0XB9,0X5D,0XFA,0X4D,0XD9,0X4D,0XB8,0X65,0X2A,0X32,0XC8,0X39,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XBF,0XFF,0XA7,0X41,0XC8,0X41,
0XC8,0X41,0XBE,0XFF,0XDE,0XF7,0XFF,0XF7,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XA6,0X39,0XFF,0XFF,0XFE,0XFF,
0XDE,0XFF,0XDE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFE,0XF7,0XDE,0XF7,0XC8,0X41,0X87,0X41,0X87,0X41,0XDF,0XFF,0XFF,0XF7,0XFF,0XF7,
0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XF7,0XFF,0XFF,0XFF,0XF7,0XFE,0XF7,0XFE,0XF7,
0XFF,0XF7,0XC7,0X39,0X87,0X39,0XA8,0X31,0X56,0X8D,0X15,0X85,0XC9,0X39,0XE9,0X49,
0XC8,0X41,0XC8,0X41,0XA7,0X41,0XC7,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,
0XFE,0XFF,0XBE,0XF7,0XE8,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFF,0XFF,0XFE,0XF7,0XFE,0XFF,0XDE,0XFF,0XFF,0XFF,0XDF,0XFF,0XBF,0XFF,
0XA8,0X49,0XC8,0X51,0X66,0X31,0XFF,0XFF,0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XC7,0X41,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,
0X08,0X3A,0X4A,0X2A,0XD8,0X5D,0XF9,0X45,0XFA,0X4D,0X98,0X65,0X0A,0X2A,0X88,0X39,
0XE8,0X49,0XDF,0XFF,0XDE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,
0XFF,0XFF,0XFF,0XFF,0XDE,0XF7,0XFF,0XFF,0XFF,0XFF,0XC7,0X31,0X0A,0X32,0X6E,0X5B,
0X29,0X3A,0XDF,0XFF,0XFF,0XFF,0XFE,0XFF,0XDE,0XF7,0XFF,0XFF,0XFF,0XFF,0X09,0X32,
0X77,0X85,0X98,0X5D,0XFA,0X4D,0XB8,0X45,0XB8,0X65,0XE9,0X29,0XC8,0X41,0XFF,0XFF,
0XDE,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XDF,0XFF,0XC8,0X41,0XE9,0X49,0XE8,0X49,
0XBF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDE,0XF7,0XFF,0XFF,0XFF,0XF7,
0XFE,0XFF,0XFF,0XFF,0XBE,0XFF,0XDF,0XFF,0XA7,0X39,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFD,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XC8,0X41,0XE9,0X49,0XC8,0X49,0XDF,0XFF,0XFF,0XF7,0XFE,0XF7,0XFE,0XFF,
0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XDE,0XF7,0XFF,0XFF,0XFE,0XF7,0XFF,0XF7,0XFF,0XF7,
0XE8,0X39,0XE9,0X39,0X0A,0X2A,0X77,0X75,0X77,0X75,0X4B,0X32,0XA9,0X31,0XC9,0X41,
0XC8,0X41,0XC8,0X49,0XC7,0X49,0XDE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,
0XDE,0XFF,0XE8,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XF7,0XFF,0XFF,0XFE,0XF7,0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XBF,0XFF,0XA8,0X49,
0XA8,0X49,0XE8,0X41,0XDF,0XFF,0XFE,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XC7,0X41,0XDF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XF7,0XDF,0XFF,0XFF,0XFF,0X08,0X3A,
0X6A,0X2A,0XD8,0X5D,0XF9,0X45,0XD9,0X4D,0XB8,0X5D,0X2B,0X22,0X0A,0X3A,0XC9,0X41,
0XDF,0XFF,0XBF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,
0XDE,0XFF,0XFF,0XFF,0XDE,0XFF,0XDF,0XFF,0X08,0X42,0XE9,0X29,0X8F,0X63,0X09,0X3A,
0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XE9,0X29,0X57,0X85,
0XB9,0X5D,0XD9,0X4D,0XD9,0X4D,0XB8,0X65,0X2A,0X2A,0XC8,0X41,0XDF,0XFF,0XDF,0XFF,
0XDF,0XFF,0XDF,0XFF,0XDF,0XFF,0X9F,0XFF,0XE9,0X49,0XE8,0X41,0XA8,0X39,0XC8,0X41,
0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XE8,0X41,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFE,0XF7,0XFE,0XFF,0XDE,0XFF,0XFF,0XFF,0XFE,0XFF,0XDF,0XFF,0XDF,0XFF,0XE8,0X41,
0XE8,0X39,0XE8,0X39,0XA8,0X41,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XDE,0XFF,
0XFF,0XFF,0XDE,0XFF,0XBE,0XFF,0XBF,0XFF,0XDF,0XFF,0XDF,0XFF,0XE8,0X41,0X29,0X3A,
0X6B,0X2A,0X77,0X75,0XF8,0X65,0XB7,0X5D,0X97,0X75,0X4B,0X2A,0X0A,0X42,0XE9,0X51,
0XE8,0X49,0XC7,0X41,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XDE,0XF7,0XBD,0XF7,0XFF,0XFF,
0XE8,0X49,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XC7,0X39,0X09,0X42,0XE9,0X41,0XE9,0X41,
0X87,0X39,0XDF,0XFF,0XDF,0XFF,0XBF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XE8,0X49,
0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0X08,0X3A,0X4A,0X2A,
0XB8,0X5D,0XF9,0X45,0X1A,0X4E,0XD9,0X4D,0XD8,0X6D,0X56,0X7D,0XEA,0X31,0XA8,0X39,
0XE8,0X39,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XE8,0X41,0X09,0X32,0X8F,0X63,0XA8,0X31,0X0A,0X4A,
0XE8,0X49,0XC7,0X39,0XC7,0X39,0XE8,0X41,0X09,0X3A,0X2A,0X3A,0X57,0X85,0XB8,0X5D,
0XF9,0X4D,0XF9,0X4D,0XB7,0X5D,0X0A,0X2A,0XE9,0X49,0XC8,0X41,0XC8,0X41,0XC8,0X49,
0XC8,0X49,0XC8,0X49,0XA8,0X41,0X09,0X42,0XF4,0X94,0X15,0X9D,0X09,0X3A,0X09,0X4A,
0XA8,0X41,0XC8,0X41,0XC8,0X41,0XC7,0X41,0XC7,0X41,0XE8,0X41,0XC8,0X41,0XC8,0X49,
0XC8,0X49,0XA7,0X41,0XA8,0X41,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,
0XFE,0XFF,0XC6,0X39,0XC7,0X41,0XE7,0X41,0XC7,0X41,0XA7,0X39,0XE9,0X41,0XD4,0X8C,
0XF4,0X8C,0X09,0X42,0XBF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XDE,0XF7,
0XE8,0X41,0X09,0X4A,0XC8,0X49,0XC8,0X49,0X2A,0X52,0XC9,0X31,0X15,0X85,0XB8,0X75,
0X97,0X5D,0XF9,0X55,0XD8,0X55,0X77,0X65,0X6B,0X22,0XC9,0X31,0X7F,0XFF,0XDF,0XFF,
0X9E,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0XBE,0XFF,0X87,0X41,
0XC8,0X49,0XE8,0X41,0X87,0X39,0X08,0X4A,0XA7,0X41,0XA8,0X41,0XA7,0X41,0XC7,0X41,
0XC7,0X41,0XE8,0X49,0XA7,0X39,0X09,0X3A,0X09,0X32,0X36,0X95,0XE9,0X39,0X2A,0X52,
0XA8,0X49,0XC8,0X51,0XA7,0X49,0XA8,0X49,0XC8,0X49,0X09,0X52,0XC8,0X49,0XC8,0X41,
0XC8,0X41,0XE8,0X41,0XE8,0X41,0XC8,0X39,0XE8,0X39,0XE9,0X41,0X4A,0X2A,0XB8,0X5D,
0XF9,0X4D,0XFA,0X45,0XD9,0X45,0XD8,0X55,0X97,0X65,0X37,0X85,0X2A,0X32,0X09,0X3A,
0XC8,0X39,0XE8,0X41,0XE7,0X41,0XC7,0X49,0XC7,0X49,0XA7,0X41,0XE8,0X49,0XA7,0X41,
0XA7,0X41,0XC8,0X49,0X4A,0X52,0XA8,0X29,0X6F,0X63,0XEA,0X39,0XCA,0X49,0X88,0X49,
0X0A,0X52,0XA8,0X41,0XC9,0X49,0XEA,0X41,0XC9,0X31,0X37,0X85,0XB7,0X5D,0X18,0X4E,
0XD7,0X3D,0XD8,0X65,0X2B,0X32,0XA9,0X49,0X88,0X41,0XA9,0X49,0XA9,0X49,0X88,0X49,
0X89,0X49,0XA9,0X39,0X2A,0X2A,0X76,0X85,0X76,0X85,0X2A,0X2A,0XEA,0X39,0XEA,0X49,
0XA9,0X49,0XC9,0X51,0XC9,0X51,0XC9,0X49,0XC9,0X51,0XA9,0X49,0XC9,0X51,0XE9,0X51,
0XA9,0X49,0XC8,0X49,0XDF,0XFF,0XFF,0XFF,0XDF,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,
0XC7,0X41,0XE8,0X49,0XC8,0X41,0XE9,0X41,0X0A,0X3A,0X2A,0X2A,0X76,0X7D,0X76,0X85,
0XE9,0X31,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XA7,0X39,
0XA8,0X41,0XC9,0X51,0XA9,0X41,0XE9,0X39,0X2A,0X22,0X77,0X6D,0XD8,0X5D,0X19,0X56,
0XD9,0X4D,0XD9,0X55,0X98,0X65,0X0A,0X12,0X0A,0X3A,0XDF,0XFF,0X9E,0XF7,0XFF,0XFF,
0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XFF,0XBE,0XFF,0XC8,0X51,0XA8,0X51,
0XA8,0X49,0XE9,0X51,0XC9,0X49,0XC9,0X49,0XEA,0X51,0XC9,0X51,0XC9,0X51,0XC9,0X49,
0XC9,0X49,0XA8,0X31,0X6A,0X32,0X29,0X1A,0X15,0X7D,0XE9,0X21,0X88,0X31,0XC9,0X51,
0X88,0X51,0XA8,0X59,0X68,0X49,0X88,0X49,0XC9,0X49,0X87,0X41,0XE9,0X51,0XC8,0X51,
0XC8,0X49,0XC8,0X49,0XC9,0X49,0XC9,0X41,0XEA,0X49,0X2B,0X32,0X98,0X65,0XD9,0X4D,
0XFA,0X45,0XFA,0X45,0XD9,0X4D,0XD9,0X5D,0X97,0X65,0X4B,0X12,0X2B,0X32,0X0A,0X42,
0XC9,0X41,0XE9,0X51,0XA8,0X49,0XC9,0X51,0XC9,0X51,0X88,0X49,0XC9,0X49,0XE9,0X51,
0XE9,0X51,0XC9,0X41,0XA9,0X31,0X90,0X53,0X2B,0X2A,0XEA,0X31,0X0A,0X3A,0XCA,0X39,
0XEA,0X39,0X2B,0X3A,0XEA,0X29,0XEA,0X21,0X57,0X75,0XD8,0X55,0X18,0X46,0X39,0X4E,
0XB8,0X55,0X4B,0X22,0XCA,0X31,0X0A,0X3A,0XEA,0X39,0XEA,0X39,0XEA,0X39,0X0B,0X3A,
0X6C,0X32,0X6B,0X12,0X76,0X6D,0XB7,0X6D,0X77,0X75,0X4B,0X2A,0XEB,0X31,0XCA,0X31,
0XEA,0X39,0X0A,0X3A,0XEA,0X39,0X0B,0X42,0XCA,0X39,0XEA,0X39,0XC9,0X39,0XA9,0X41,
0XE9,0X49,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFF,0XFF,0X09,0X42,
0X0A,0X42,0XE9,0X39,0X0A,0X2A,0X4B,0X22,0X36,0X6D,0X76,0X65,0X97,0X75,0XE9,0X29,
0XBF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XDF,0XFF,0XE8,0X41,0XE9,0X41,
0XC9,0X39,0X0A,0X32,0X6B,0X2A,0X57,0X6D,0XB8,0X5D,0XD8,0X4D,0XD9,0X45,0XD9,0X45,
0XD9,0X55,0XB8,0X6D,0X4A,0X1A,0X09,0X3A,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDE,0XFF,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XE9,0X51,0XC9,0X49,0XE9,0X41,
0XE9,0X39,0XC9,0X31,0X2A,0X3A,0XA9,0X31,0X0B,0X42,0XEA,0X39,0XEA,0X31,0X0A,0X3A,
0X2A,0X2A,0X77,0X75,0XB8,0X75,0X56,0X65,0XCC,0X22,0X4B,0X32,0XC9,0X31,0XA9,0X39,
0XCA,0X41,0X0A,0X42,0X0A,0X3A,0X0A,0X3A,0X2A,0X3A,0XE9,0X39,0X0A,0X3A,0X2A,0X42,
0XE9,0X39,0X0A,0X3A,0XEA,0X31,0X4B,0X3A,0X2B,0X1A,0X98,0X5D,0XD9,0X45,0XFA,0X45,
0X1A,0X46,0XD9,0X4D,0XD9,0X4D,0X19,0X5E,0XD8,0X65,0X37,0X75,0X4C,0X32,0XCA,0X29,
0XEA,0X39,0XEA,0X39,0X0A,0X3A,0XEB,0X39,0XCA,0X39,0XEA,0X39,0XEA,0X39,0XC9,0X31,
0XE9,0X31,0X4B,0X2A,0X98,0X65,0X98,0X6D,0XB8,0X75,0X78,0X75,0XB9,0X7D,0X57,0X6D,
0X77,0X75,0X98,0X75,0X98,0X6D,0XD8,0X65,0XF9,0X4D,0XF9,0X45,0XD9,0X45,0XD9,0X55,
0XB9,0X6D,0X98,0X75,0XB8,0X75,0X57,0X6D,0X77,0X6D,0X98,0X75,0X98,0X75,0X77,0X65,
0X1A,0X6E,0XD8,0X5D,0X98,0X55,0XB9,0X5D,0X98,0X65,0XB9,0X75,0XB9,0X75,0XD8,0X75,
0X97,0X75,0XB8,0X75,0X98,0X75,0X98,0X75,0X97,0X75,0X77,0X85,0X56,0X9D,0XC8,0X39,
0XDE,0XFF,0XFE,0XFF,0XDE,0XFF,0XDE,0XF7,0XFF,0XFF,0XDF,0XF7,0X09,0X3A,0X4B,0X32,
0X15,0X7D,0X97,0X75,0X97,0X5D,0XB8,0X5D,0XB7,0X5D,0X97,0X75,0X2A,0X32,0XBF,0XFF,
0XDE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XDF,0XFF,0X08,0X3A,0X09,0X2A,0X56,0X85,
0X97,0X75,0X77,0X5D,0X19,0X5E,0XB8,0X45,0XFA,0X4D,0XFA,0X45,0XFA,0X4D,0XD9,0X55,
0X98,0X65,0X6A,0X12,0X29,0X3A,0XDF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XF7,0XC8,0X41,0XC8,0X41,0XE9,0X39,0X36,0X8D,0X97,0X7D,0XB7,0X75,
0XD8,0X75,0X97,0X75,0XB8,0X75,0XB9,0X7D,0XD9,0X75,0XB8,0X75,0XB8,0X75,0XB9,0X6D,
0X78,0X5D,0XB9,0X55,0XD9,0X5D,0XB8,0X5D,0XB9,0X6D,0XB9,0X75,0X77,0X75,0X77,0X75,
0X98,0X75,0X77,0X6D,0XB8,0X75,0XD9,0X75,0X97,0X6D,0XB8,0X75,0XB8,0X75,0XB8,0X75,
0XB8,0X75,0XB8,0X75,0XB8,0X6D,0XD8,0X65,0XF9,0X55,0XF9,0X4D,0X1A,0X46,0XFA,0X45,
0XD9,0X45,0XF9,0X4D,0XD8,0X4D,0XF8,0X5D,0X97,0X5D,0X98,0X6D,0XB9,0X75,0X99,0X75,
0X98,0X75,0X98,0X75,0X98,0X75,0X78,0X75,0X98,0X75,0XB8,0X75,0XB7,0X7D,0X97,0X75,
0XB8,0X65,0XD8,0X55,0X19,0X5E,0XD8,0X55,0XB8,0X55,0XB8,0X55,0XB8,0X55,0X98,0X55,
0X98,0X55,0X98,0X4D,0XF9,0X55,0XF9,0X4D,0XF9,0X45,0XFA,0X4D,0XFA,0X4D,0XB9,0X55,
0XB8,0X55,0XB8,0X55,0XB8,0X55,0XD8,0X55,0XD9,0X55,0XB8,0X55,0XF9,0X55,0XD9,0X4D,
0XD9,0X4D,0XD9,0X4D,0XFA,0X4D,0XD9,0X4D,0XD9,0X55,0XD9,0X55,0XD9,0X55,0XD8,0X55,
0XF9,0X55,0XB8,0X4D,0XB8,0X55,0XF8,0X5D,0XD7,0X6D,0X56,0X85,0XE8,0X31,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X09,0X32,0X4B,0X2A,0X77,0X75,
0X98,0X5D,0XD9,0X55,0XF8,0X4D,0XB8,0X55,0X97,0X75,0XEA,0X31,0XDF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0X08,0X32,0X6A,0X2A,0XB7,0X75,0XF8,0X65,
0XF8,0X4D,0XF9,0X4D,0XF9,0X45,0XFA,0X45,0XFA,0X4D,0XD9,0X45,0XFA,0X55,0X97,0X5D,
0X4A,0X12,0X29,0X3A,0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XBE,0XFF,0XFF,0XFF,0XDF,0XFF,
0XBF,0XF7,0XC9,0X39,0XEA,0X31,0X4C,0X32,0X57,0X75,0X97,0X55,0XF8,0X55,0XD8,0X55,
0XD8,0X55,0XB8,0X55,0X98,0X55,0XD8,0X55,0XD8,0X55,0XF9,0X5D,0XD9,0X55,0XD9,0X4D,
0XD9,0X4D,0XD9,0X4D,0XFA,0X4D,0XD9,0X55,0XB8,0X55,0XD9,0X55,0XD9,0X5D,0X98,0X4D,
0XD8,0X55,0XB8,0X55,0XB8,0X4D,0XD8,0X55,0XB8,0X55,0XD8,0X55,0XD9,0X55,0XB8,0X4D,
0XF9,0X55,0X19,0X56,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0X1A,0X46,0X1A,0X46,0X19,0X46,
0X19,0X4E,0XF9,0X4D,0XD8,0X4D,0XF9,0X55,0XD9,0X55,0XF9,0X5D,0XD9,0X5D,0X98,0X4D,
0XD9,0X55,0X98,0X55,0XB9,0X55,0XF9,0X55,0XB8,0X4D,0XD8,0X5D,0XB8,0X55,0XF9,0X55,
0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,
0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,
0XF9,0X4D,0XFA,0X4D,0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XD9,0X4D,0XB9,0X4D,0XF9,0X55,0XB8,0X5D,0X77,0X7D,0X09,0X2A,0XFF,0XFF,0XDF,0XFF,
0XDF,0XFF,0XDF,0XFF,0XDF,0XFF,0XDF,0XFF,0X09,0X3A,0X6B,0X2A,0XB8,0X65,0XF9,0X55,
0XF9,0X4D,0XF9,0X4D,0XD9,0X55,0XB8,0X75,0X0A,0X32,0XBF,0XFF,0XFF,0XFF,0XBF,0XFF,
0XDF,0XFF,0XDF,0XFF,0XBF,0XFF,0X09,0X3A,0X4A,0X22,0X97,0X6D,0XD8,0X55,0XD9,0X45,
0XF9,0X45,0XF9,0X45,0XFA,0X4D,0XD9,0X45,0XFA,0X4D,0XD9,0X55,0X98,0X65,0X4B,0X1A,
0X09,0X3A,0XDF,0XFF,0XDF,0XFF,0XBF,0XFF,0XDF,0XFF,0XDF,0XFF,0XDF,0XFF,0X4B,0X42,
0X2B,0X3A,0X6D,0X32,0X78,0X6D,0XB8,0X5D,0X19,0X56,0X19,0X4E,0XD8,0X45,0XF9,0X4D,
0XD9,0X55,0XD9,0X55,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,
0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,
0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,
0XF9,0X45,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XD9,0X4D,0XB8,0X5D,0X98,0X7D,0XC8,0X21,0X4A,0X4A,0XA7,0X41,0XE8,0X41,
0XA8,0X41,0XC8,0X41,0XC8,0X41,0XC8,0X31,0X4B,0X2A,0XB8,0X5D,0XD9,0X45,0XD9,0X45,
0XB9,0X4D,0XD9,0X5D,0X78,0X75,0X0A,0X32,0XC9,0X49,0XC8,0X41,0XC8,0X41,0XA8,0X41,
0XC8,0X41,0XC8,0X41,0XE9,0X39,0X8C,0X2A,0XB8,0X6D,0XD9,0X55,0XF9,0X45,0XFA,0X45,
0XF9,0X4D,0XFA,0X4D,0XD9,0X4D,0XFA,0X4D,0XD9,0X55,0X98,0X65,0X4B,0X1A,0X09,0X42,
0XA7,0X41,0X86,0X39,0XC8,0X41,0XC8,0X49,0XC8,0X41,0XE9,0X41,0X0A,0X32,0X79,0X8D,
0X58,0X75,0XD9,0X65,0XD8,0X4D,0XD8,0X3D,0X19,0X46,0XFA,0X45,0XFA,0X4D,0XD9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XFA,0X4D,
0XFA,0X4D,0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,
0XD9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XD9,0X45,0XFA,0X4D,
0XD9,0X4D,0X98,0X55,0X77,0X7D,0X09,0X2A,0XC8,0X39,0X09,0X4A,0XE8,0X49,0XC8,0X41,
0XE9,0X49,0XE8,0X49,0XC8,0X39,0X2A,0X22,0XB9,0X65,0XFA,0X4D,0XFA,0X4D,0XD9,0X4D,
0XB8,0X55,0X77,0X6D,0X2A,0X3A,0XA8,0X41,0XC8,0X49,0XE8,0X49,0XE9,0X49,0XE9,0X49,
0X67,0X39,0X2A,0X42,0X2A,0X1A,0X77,0X65,0XD9,0X55,0XD9,0X45,0XFA,0X4D,0XF9,0X4D,
0XFA,0X4D,0XD9,0X4D,0XFA,0X4D,0XD9,0X55,0XB9,0X65,0X4B,0X1A,0XE9,0X39,0XE8,0X49,
0XE8,0X49,0XE8,0X49,0XC8,0X49,0XC8,0X41,0XC9,0X39,0XE9,0X29,0X37,0X75,0XB9,0X65,
0XD8,0X55,0X19,0X4E,0X19,0X46,0X19,0X46,0XD9,0X45,0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,
0XF9,0X45,0XF9,0X45,0XD9,0X4D,0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,0XF9,0X45,
0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,
0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XFA,0X45,0XFA,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X1A,0X4E,0XFA,0X4D,0XD9,0X4D,
0XD8,0X5D,0X98,0X85,0X09,0X32,0X09,0X42,0XC8,0X41,0XE8,0X49,0XE8,0X49,0XC8,0X49,
0XC8,0X41,0XC9,0X39,0X6B,0X2A,0XB8,0X5D,0XF9,0X4D,0X1A,0X4E,0XB8,0X45,0XF9,0X5D,
0X77,0X6D,0XC9,0X31,0XE9,0X51,0XE9,0X49,0XC8,0X49,0XC8,0X49,0XE9,0X49,0XE9,0X49,
0XC8,0X39,0X4B,0X22,0X98,0X6D,0XD9,0X55,0XFA,0X45,0XFA,0X4D,0XF9,0X4D,0XFA,0X4D,
0XF9,0X4D,0XFA,0X4D,0XD9,0X55,0XB9,0X65,0X4B,0X1A,0X29,0X42,0XA7,0X49,0XC8,0X49,
0XC8,0X49,0XC8,0X49,0X87,0X41,0X2A,0X4A,0X36,0X8D,0X98,0X75,0XD8,0X5D,0XF9,0X4D,
0XD8,0X45,0X19,0X46,0X1A,0X4E,0XD9,0X45,0XFA,0X4D,0XD9,0X4D,0XFA,0X4D,0XF9,0X45,
0XF9,0X45,0XF9,0X4D,0XD9,0X4D,0XD9,0X55,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,};
#line 4 "app\\game.c"
#line 1 ".\\app\\RES\\Pic\\flappybird_button_play.c"
const unsigned char gImage_flappybird_button_play[6864] = {  
0XD9,0X4D,0X1A,0X4E,0XF9,0X45,0X19,0X4E,0XF9,0X55,0XB7,0X65,0X09,0X2A,0XE8,0X41,
0XC7,0X41,0XC7,0X41,0XC8,0X49,0XC8,0X41,0XC8,0X41,0XC8,0X41,0XC8,0X41,0XC7,0X41,
0XC8,0X49,0XC8,0X49,0XC8,0X49,0XC7,0X41,0XC8,0X49,0XE8,0X49,0XE8,0X49,0XC7,0X41,
0XC8,0X49,0XC8,0X49,0XC8,0X49,0XC8,0X49,0XC8,0X49,0XC8,0X49,0XC8,0X49,0XC8,0X49,
0XC8,0X49,0XC8,0X49,0XC8,0X49,0XC8,0X49,0XE8,0X41,0XE8,0X41,0XC7,0X49,0XC7,0X49,
0XC7,0X49,0XC7,0X49,0XA8,0X49,0XA8,0X49,0XC8,0X49,0XC7,0X49,0XC7,0X49,0XC7,0X49,
0XC7,0X49,0XC7,0X49,0XC7,0X49,0XC7,0X49,0XC7,0X49,0XC7,0X49,0XC7,0X49,0XC7,0X49,
0XC7,0X49,0XC7,0X49,0XC7,0X49,0XC7,0X49,0XC7,0X49,0XC7,0X49,0XC7,0X49,0XC7,0X49,
0XC7,0X49,0XC7,0X49,0XE7,0X41,0XE7,0X41,0XE7,0X41,0XC7,0X49,0XA8,0X51,0X09,0X3A,
0XD7,0X65,0XF8,0X45,0XF9,0X4D,0XD9,0X45,0XF9,0X4D,0X1A,0X56,0XF9,0X55,0XB8,0X4D,
0XB9,0X55,0XD8,0X5D,0X97,0X5D,0X96,0X75,0X29,0X32,0X87,0X39,0XA6,0X39,0XA6,0X39,
0XA7,0X39,0XA7,0X41,0XA7,0X39,0XC7,0X39,0XC7,0X39,0XC7,0X41,0XA7,0X39,0XA7,0X39,
0X87,0X39,0X86,0X39,0XA7,0X39,0XA7,0X41,0XA7,0X41,0X86,0X39,0XA7,0X41,0XA7,0X41,
0XA7,0X41,0XA7,0X41,0XA7,0X41,0XA7,0X41,0XA7,0X41,0XA7,0X41,0XC7,0X41,0XA7,0X41,
0XC8,0X41,0XA7,0X41,0XC7,0X39,0XC7,0X39,0XC6,0X41,0XC6,0X41,0XC6,0X41,0XC7,0X41,
0XA7,0X41,0XA8,0X41,0XC7,0X41,0XC7,0X41,0XC6,0X41,0XC6,0X41,0XC7,0X41,0XC7,0X41,
0XC7,0X41,0XC7,0X41,0XC7,0X41,0XC7,0X41,0XC7,0X41,0XC7,0X41,0XC7,0X41,0XC7,0X41,
0XC7,0X41,0XC7,0X41,0XC7,0X41,0XC7,0X41,0XC7,0X41,0XC7,0X41,0XA7,0X41,0XA6,0X41,
0XC7,0X39,0XC7,0X39,0XC6,0X39,0XA6,0X41,0X87,0X41,0XC8,0X31,0X76,0X6D,0XD8,0X5D,
0XD9,0X5D,0XF9,0X5D,0XB8,0X55,0XB8,0X4D,0XB8,0X5D,0XB8,0X65,0X77,0X7D,0X09,0X1A,
0X29,0X2A,0X29,0X32,0XBF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0X09,0X2A,0XE9,0X19,0X2A,0X1A,0X77,0X6D,
0X97,0X5D,0XF8,0X5D,0X97,0X65,0XAB,0X1A,0X2A,0X2A,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFE,0XF7,
0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,
0XDF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XF7,0X29,0X22,0XEC,0X1A,0XB7,0X5D,
0X76,0X75,0XAB,0X2A,0XC7,0X29,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XDE,0XF7,0XFF,0XF7,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XF7,
0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,
0XFF,0XF7,0XFE,0XF7,0XFF,0XF7,0XFE,0XF7,0XFF,0XF7,0XFE,0XF7,0XFF,0XF7,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,
0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X28,0X32,0X49,0X12,0X96,0X6D,0X35,0X85,0X29,0X2A,
0XE7,0X31,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XF7,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFF,0XF7,0XFE,0XF7,0XFE,0XF7,
0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,
0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,
0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,
0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFD,0XF7,
0XFF,0XFF,0XE7,0X31,0XE8,0X21,0X55,0X85,0X28,0X3A,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,
0XFE,0XF7,0XFF,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,
0XFE,0XFF,0XFE,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XF7,
0XFE,0XFF,0XFE,0XF7,0XFE,0XFF,0XFE,0XF7,0XFE,0XFF,0XFE,0XF7,0XFE,0XFF,0XFE,0XF7,
0XFE,0XFF,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XF7,0XFF,0XF7,
0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XF7,
0XFF,0XFF,0XC8,0X31,0XA6,0X39,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,
0XFE,0XF7,0XFF,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,
0XFF,0XF7,0XFF,0XF7,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,
0XFE,0XF7,0XFE,0XFF,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XDE,0XFF,0XDF,0XFF,0XE8,0X41,
0XA7,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,
0XFE,0XF7,0XFE,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,
0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,
0XFE,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,0XDF,0XFF,0XC7,0X41,0XA7,0X49,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,
0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XF7,0XFE,0XF7,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XDE,0XF7,0XFF,0XF7,0XFE,0XF7,0XFE,0XF7,
0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XF7,0XFF,0XF7,0XFE,0XF7,0XFE,0XF7,
0XFF,0XF7,0XFF,0XFF,0XDF,0XFF,0XC7,0X41,0XA7,0X41,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFE,0XF7,
0XDD,0XFF,0XFE,0XFF,0XFF,0XFF,0XDE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,
0XFE,0XF7,0XFF,0XF7,0XFF,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XC7,0X41,0XA7,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,0XFE,0XEF,0XFE,0XEF,0XFE,0XEF,0XFE,0XEF,0XFD,0XF7,
0XFE,0XF7,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFE,0XF7,0XFF,0XF7,0XFF,0XF7,
0XFF,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XDF,0XFF,0XC8,0X41,
0XA7,0X41,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,
0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,
0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,
0XFF,0XFF,0XFF,0XF7,0XEA,0X3B,0X0A,0X2C,0X0A,0X34,0XFC,0XCF,0XFE,0XEF,0XFE,0XF7,
0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,
0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,
0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFE,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XDF,0XFF,0XC8,0X41,0XA7,0X41,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,
0XFE,0XFF,0XFE,0XF7,0XFE,0XFF,0XFE,0XF7,0XFE,0XFF,0XFE,0XF7,0XFE,0XFF,0XFE,0XF7,
0XFE,0XFF,0XFE,0XF7,0XFE,0XFF,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XDE,0XFF,0XFE,0XEF,
0X2A,0X2C,0X69,0X1C,0XCA,0X24,0XF9,0XA7,0XFB,0XCF,0XFD,0XE7,0XFF,0XF7,0XFF,0XFF,
0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,
0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,
0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,
0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFF,0XFF,0XDF,0XFF,0XC8,0X41,0XA7,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFD,0XE7,0X6A,0X2C,0XC9,0X0C,
0X09,0X05,0XE9,0X0C,0X6A,0X24,0XFC,0XCF,0XFE,0XEF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,0XDF,0XFF,
0XBF,0XFF,0XC8,0X41,0XA7,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFF,0XF7,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XE7,0X49,0X24,0XE9,0X0C,0X49,0X05,0X29,0X05,
0XA9,0X0C,0X6A,0X24,0XFD,0XD7,0XFE,0XEF,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,0XDF,0XFF,0XBF,0XFF,0XA8,0X41,
0XA7,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XF7,0XFE,0XE7,0X4A,0X24,0XC9,0X0C,0X09,0X05,0X29,0X05,0X09,0X05,0XCA,0X14,
0XF9,0XAF,0XFC,0XD7,0XFE,0XEF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,
0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XC8,0X41,0XC7,0X39,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF7,0XFE,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XDF,
0X6A,0X24,0XC9,0X14,0X08,0X05,0X28,0X05,0X29,0X05,0XE9,0X04,0XA9,0X14,0X4A,0X2C,
0XFD,0XDF,0XFE,0XF7,0XDE,0XEF,0XFF,0XFF,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XC7,0X41,0XC7,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFE,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XE7,0X69,0X2C,0XE8,0X14,
0X08,0X05,0X28,0X05,0X28,0X05,0X29,0X05,0X09,0X05,0XCA,0X1C,0X49,0X24,0XFC,0XCF,
0XFE,0XEF,0XFE,0XF7,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,0XDF,0XFF,
0XDF,0XFF,0XE8,0X41,0XC7,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XFF,0XFE,0XF7,0XFE,0XFF,0XFE,0XF7,
0XFE,0XFF,0XFE,0XF7,0XFE,0XFF,0XFE,0XF7,0XFE,0XFF,0XFE,0XF7,0XFE,0XFF,0XFE,0XF7,
0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XE7,0X89,0X24,0XE8,0X0C,0X08,0X05,0X28,0X05,
0X28,0X05,0X49,0X05,0X29,0X05,0X08,0X05,0XEA,0X14,0XFA,0XAF,0XFB,0XCF,0XFC,0XE7,
0XFF,0XF7,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XC7,0X41,
0XC7,0X41,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFE,0XFF,0XFD,0XE7,0X69,0X24,0XE8,0X0C,0X08,0X05,0X28,0X05,0X28,0X05,0X48,0X05,
0X48,0X05,0X69,0X05,0X29,0X05,0XA9,0X14,0X4B,0X34,0XFD,0XDF,0XFE,0XEF,0XFE,0XF7,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XE8,0X41,0XA7,0X39,0XBF,0XFF,
0X9E,0XF7,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XF7,0X9D,0XF7,
0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XF7,0X9D,0XF7,0X7D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XF7,0X9D,0XF7,0XFC,0XDF,
0X49,0X24,0XE8,0X0C,0X08,0X05,0X28,0X05,0X28,0X05,0X48,0X05,0X48,0X05,0X69,0X05,
0X29,0X05,0XEA,0X14,0X0A,0X2C,0XFD,0XD7,0XBD,0XE7,0X9E,0XF7,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7E,0XF7,0X7E,0XFF,0XA7,0X41,0XC8,0X41,0XDF,0XFF,0X9E,0XF7,0X9E,0XF7,
0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7C,0XEF,0XFC,0XDF,0X49,0X24,0XC9,0X0C,
0X08,0X05,0X28,0X05,0X08,0X05,0X08,0X05,0X08,0X05,0X08,0X05,0X0A,0X15,0X6A,0X24,
0X0C,0X4C,0XDD,0XE7,0X9E,0XF7,0X7D,0XF7,0X7E,0XF7,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9E,0XF7,0X9E,0XF7,0X9E,0XF7,0X9E,0XF7,
0X9F,0XFF,0XA7,0X41,0XC8,0X41,0XBF,0XFF,0X9E,0XF7,0X9E,0XF7,0X9E,0XF7,0X9E,0XF7,
0X9D,0XF7,0X7D,0XEF,0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,
0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X9E,0XF7,0X9E,0XF7,
0X9E,0XF7,0X9D,0XEF,0X9D,0XEF,0XFD,0XDF,0X4A,0X24,0XC9,0X14,0X09,0X05,0X08,0X05,
0X08,0X05,0X08,0X05,0X29,0X05,0XE9,0X0C,0X8A,0X1C,0XFC,0XBF,0XFE,0XE7,0X7D,0XE7,
0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7E,0XF7,0X7E,0XF7,0X7E,0XFF,0XA7,0X41,
0XC8,0X41,0XBF,0XFF,0X7E,0XF7,0X9E,0XF7,0X9E,0XF7,0X9E,0XEF,0X7D,0XEF,0X7E,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XF7,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X9E,0XF7,0X9E,0XF7,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X9D,0XEF,
0X9D,0XEF,0XFD,0XDF,0X29,0X24,0XC9,0X14,0X09,0X0D,0X08,0X05,0X08,0X05,0X29,0X05,
0XE8,0X04,0XA9,0X14,0XFB,0XBF,0XFD,0XDF,0XDE,0XE7,0X9E,0XEF,0X9E,0XF7,0X7E,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7E,0XEF,0X7E,0XEF,0X7E,0XF7,0X7E,0XF7,0X5E,0XF7,0XA8,0X41,0XC8,0X41,0XBF,0XFF,
0X7E,0XF7,0X7E,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XF7,
0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0XFC,0XDF,
0X49,0X24,0XA9,0X14,0XE8,0X04,0X49,0X0D,0X29,0X05,0XE9,0X04,0XEA,0X1C,0X2A,0X2C,
0XFC,0XD7,0XDE,0XEF,0X7D,0XE7,0XBF,0XF7,0XBE,0XF7,0X9E,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7E,0XEF,0X7E,0XEF,
0X7E,0XEF,0X7E,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7E,0XEF,0X9E,0XF7,
0X7E,0XF7,0X7E,0XF7,0X7E,0XFF,0XC8,0X41,0XC8,0X41,0XBF,0XFF,0X7E,0XF7,0X7E,0XEF,
0X9E,0XF7,0X7E,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X9D,0XF7,0X7D,0XF7,0XFC,0XDF,0X49,0X24,0X09,0X15,
0X08,0X0D,0X08,0X05,0X29,0X0D,0XA9,0X14,0XFB,0XC7,0XFD,0XDF,0XBC,0XE7,0X9D,0XEF,
0XFF,0XFF,0X7E,0XEF,0X9D,0XEF,0X7D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XF7,0X7E,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,0X7D,0XEF,0X7E,0XEF,0X9E,0XF7,0X7E,0XF7,0X7E,0XF7,
0X7E,0XFF,0XC8,0X49,0XA7,0X41,0XBF,0XFF,0X7E,0XF7,0X7E,0XEF,0X9E,0XEF,0X9E,0XEF,
0X7D,0XEF,0X9E,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,
0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7E,0XF7,0X9E,0XFF,0XDC,0XDF,0X49,0X24,0XE8,0X14,0X09,0X0D,0X09,0X15,
0X0B,0X25,0XFA,0XAF,0XFD,0XE7,0XDD,0XF7,0X9D,0XF7,0XBE,0XF7,0X7D,0XEF,0X9E,0XEF,
0X9D,0XEF,0X9E,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XF7,0X7E,0XF7,0X7D,0XF7,0X7D,0XF7,
0X7D,0XEF,0X7D,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XF7,0X7E,0XF7,0X7E,0XFF,0XC8,0X41,
0XA7,0X41,0X9E,0XFF,0X5D,0XF7,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X9E,0XF7,
0X7E,0XF7,0X7E,0XF7,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XF7,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7E,0XF7,
0X7D,0XF7,0XFD,0XE7,0X4A,0X24,0XC9,0X14,0X0A,0X1D,0XAA,0X1C,0XE9,0X23,0XFD,0XD7,
0XDD,0XEF,0X7D,0XF7,0X7D,0XFF,0X9D,0XFF,0XDE,0XF7,0XBE,0XEF,0X9E,0XF7,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,0X7D,0XEF,
0X7E,0XEF,0X9E,0XF7,0X7E,0XF7,0X9E,0XF7,0X7E,0XFF,0XC8,0X41,0XC7,0X41,0XBF,0XFF,
0X7E,0XF7,0X7E,0XEF,0X7E,0XEF,0X9E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XF7,0X7E,0XEF,
0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X9E,0XF7,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X9E,0XF7,0X9E,0XF7,0X9E,0XF7,0XDD,0XE7,
0X2A,0X34,0X6A,0X1C,0X8A,0X24,0XFB,0XB7,0XFE,0XD7,0XDD,0XE7,0X7D,0XEF,0X7D,0XF7,
0X7D,0XFF,0X9D,0XF7,0XBE,0XEF,0XBE,0XEF,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,0X7D,0XEF,0X7E,0XEF,0X7E,0XEF,
0X7E,0XF7,0X9E,0XF7,0X7E,0XFF,0XC8,0X41,0XC7,0X41,0XBF,0XFF,0X7E,0XF7,0X9E,0XF7,
0X9E,0XEF,0X9D,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XF7,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,
0X9D,0XEF,0X9D,0XEF,0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X9D,0XF7,0X9E,0XF7,0X9E,0XF7,0XBD,0XEF,0XFC,0XD7,0XFC,0XCF,
0XDC,0XCF,0XFE,0XE7,0XDE,0XE7,0XBE,0XEF,0X9E,0XF7,0X9E,0XF7,0X7D,0XEF,0X9E,0XEF,
0X7E,0XEF,0X9E,0XF7,0X7E,0XF7,0X7D,0XF7,0X7D,0XF7,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7E,0XF7,0X7E,0XF7,
0X7E,0XFF,0XC8,0X41,0XC8,0X41,0XBF,0XFF,0X7E,0XF7,0X7D,0XEF,0X9E,0XF7,0X9D,0XEF,
0X7E,0XEF,0X9E,0XF7,0X7E,0XF7,0X7E,0XEF,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X7D,0XEF,0X7D,0XF7,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X9E,0XF7,0XBE,0XF7,0X7D,0XEF,0X9D,0XEF,0XDE,0XEF,0XFE,0XEF,0X9D,0XE7,0XDF,0XF7,
0X7E,0XEF,0X9E,0XF7,0X7E,0XEF,0X7D,0XEF,0XBE,0XEF,0X9E,0XE7,0X9E,0XEF,0X9E,0XF7,
0X7E,0XF7,0X7D,0XF7,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,
0X7D,0XEF,0X7D,0XEF,0X9E,0XF7,0X9E,0XF7,0X9E,0XF7,0X7E,0XF7,0X7E,0XF7,0XC8,0X41,
0XE8,0X39,0XBF,0XF7,0X9E,0XF7,0X7E,0XF7,0X7D,0XF7,0X7D,0XEF,0X7E,0XF7,0X7E,0XF7,
0X9E,0XF7,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XF7,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X9D,0XF7,0X9D,0XEF,
0X9D,0XF7,0XBE,0XF7,0XBE,0XEF,0X9D,0XEF,0XBE,0XF7,0XBE,0XEF,0X9E,0XEF,0X9E,0XF7,
0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9E,0XEF,0X9E,0XEF,0X9E,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X9E,0XF7,0X9E,0XF7,
0X7D,0XEF,0X9E,0XF7,0X9E,0XEF,0X5D,0XE7,0X9E,0XF7,0XC8,0X49,0X08,0X3A,0XDF,0XFF,
0X5E,0XF7,0X7D,0XF7,0X7D,0XF7,0X9D,0XF7,0X9E,0XF7,0X7E,0XF7,0X9E,0XF7,0X9D,0XF7,
0X7D,0XF7,0X9D,0XF7,0X9D,0XEF,0X9D,0XEF,0X9E,0XF7,0X7E,0XF7,0X7D,0XF7,0X7D,0XEF,
0X7D,0XF7,0X7D,0XEF,0X7D,0XF7,0X7D,0XEF,0X7D,0XF7,0X7D,0XEF,0X7D,0XF7,0X7D,0XEF,
0X7D,0XF7,0X7D,0XEF,0X7D,0XF7,0X7D,0XEF,0X7D,0XF7,0X9D,0XF7,0X9D,0XF7,0XBE,0XF7,
0X9E,0XF7,0X9E,0XEF,0XBE,0XF7,0X9E,0XF7,0X9D,0XEF,0XBE,0XF7,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,0X5D,0XEF,
0X9E,0XEF,0X9E,0XEF,0X7E,0XFF,0XE8,0X49,0XA6,0X41,0XBE,0XFF,0X9D,0XFF,0X7D,0XF7,
0X7D,0XEF,0X7D,0XEF,0X9E,0XF7,0X5D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7D,0XF7,0X7D,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,
0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,0X7D,0XF7,0X7E,0XF7,0X7D,0XF7,
0X7E,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X9E,0XF7,0X7D,0XF7,0X7D,0XF7,
0X9E,0XF7,0X9D,0XF7,0X7D,0XF7,0X9D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7E,0XF7,0X7E,0XF7,0X7E,0XEF,0X7E,0XEF,
0X7E,0XEF,0X7E,0XEF,0X7E,0XF7,0X7E,0XF7,0X9E,0XF7,0X9D,0XF7,0X9E,0XF7,0X9E,0XF7,
0X7E,0XFF,0XA7,0X41,0XC7,0X49,0XBE,0XFF,0X7C,0XF7,0XDE,0XFF,0X9E,0XF7,0XBE,0XF7,
0X7E,0XEF,0X7E,0XF7,0X7D,0XF7,0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7E,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,
0X7E,0XEF,0X7E,0XEF,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,
0X7E,0XF7,0X7D,0XF7,0X7D,0XF7,0X9D,0XF7,0X7E,0XF7,0X7D,0XF7,0X9E,0XF7,0X7E,0XF7,
0X7D,0XF7,0X7E,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7E,0XF7,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X9E,0XEF,0X7E,0XF7,
0X7E,0XF7,0X7E,0XF7,0X9E,0XF7,0X5C,0XEF,0XDE,0XFF,0X7D,0XF7,0X9E,0XFF,0XC7,0X41,
0XA6,0X41,0X58,0XD6,0X58,0XCE,0X9D,0XF7,0XDE,0XF7,0X9E,0XEF,0X5D,0XEF,0X7E,0XF7,
0X7E,0XF7,0X7D,0XEF,0X9D,0XEF,0X9E,0XEF,0X9E,0XEF,0X9E,0XEF,0X7E,0XF7,0X7E,0XF7,
0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XF7,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,0X7E,0XEF,0X7E,0XF7,
0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,
0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,
0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7D,0XEF,
0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7E,0XEF,0X7E,0XF7,0X7E,0XF7,0X9E,0XF7,0X9D,0XEF,
0X9D,0XF7,0XDE,0XFF,0XBD,0XFF,0X17,0XCE,0XF7,0XCD,0XE7,0X49,0X08,0X32,0X17,0XBE,
0X17,0XC6,0X9D,0XF7,0X9D,0XF7,0XBE,0XF7,0X9E,0XF7,0X9E,0XF7,0X7E,0XEF,0X7D,0XEF,
0X7E,0XEF,0X9E,0XEF,0X7E,0XEF,0X9E,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XEF,0X7E,0XEF,
0X7E,0XEF,0X7E,0XEF,0X7D,0XEF,0X9D,0XEF,0X7D,0XEF,0X9D,0XEF,0X7D,0XEF,0X9D,0XEF,
0X7D,0XEF,0X9D,0XEF,0X7D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,
0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,
0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,
0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X7E,0XEF,0X7E,0XF7,0X7E,0XF7,0X7D,0XEF,0X9D,0XEF,0XBD,0XF7,0X9C,0XF7,
0X7C,0XF7,0XF6,0XC5,0X58,0XC6,0XE7,0X39,0X55,0X7D,0XCB,0X32,0X06,0X3A,0XBD,0XFF,
0X7C,0XFF,0X7D,0XF7,0X9E,0XF7,0X5E,0XEF,0X7E,0XEF,0X7E,0XF7,0X7E,0XF7,0X9D,0XF7,
0X9E,0XF7,0X9D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XEF,
0X7D,0XF7,0X7D,0XEF,0X7D,0XF7,0X7D,0XEF,0X7D,0XF7,0X7D,0XEF,0X7D,0XF7,0X7D,0XEF,
0X7D,0XF7,0X7D,0XEF,0X7D,0XF7,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X9D,0XEF,0X7D,0XF7,0X7D,0XF7,0X7E,0XEF,0X7E,0XEF,
0X7E,0XEF,0X9E,0XEF,0X7D,0XF7,0X7C,0XF7,0X9D,0XEF,0XBD,0XF7,0X7D,0XFF,0X28,0X42,
0X29,0X2A,0X76,0X7D,0X97,0X6D,0X8A,0X1A,0X47,0X3A,0X37,0XCE,0XF7,0XCD,0XB6,0XC5,
0X9D,0XFF,0XBE,0XFF,0XBE,0XFF,0XBE,0XFF,0XBE,0XFF,0XBE,0XFF,0XBD,0XFF,0XBD,0XF7,
0X9D,0XF7,0X7C,0XF7,0XBE,0XFF,0XBE,0XFF,0XBE,0XFF,0XBE,0XFF,0XBE,0XFF,0XBE,0XFF,
0XBE,0XFF,0XBE,0XFF,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7C,0XEF,0X9D,0XF7,0X9C,0XF7,0X7C,0XEF,0X9C,0XF7,
0X9D,0XF7,0X9D,0XF7,0X9D,0XF7,0X9D,0XF7,0X9D,0XF7,0X9D,0XF7,0X9D,0XF7,0X9D,0XF7,
0X9D,0XF7,0X9D,0XF7,0X9D,0XF7,0X9C,0XF7,0X7D,0XF7,0X9C,0XF7,0X7D,0XF7,0X9C,0XF7,
0X7D,0XF7,0X9C,0XF7,0X7D,0XF7,0X9C,0XF7,0X7D,0XF7,0X9C,0XF7,0X7D,0XF7,0X9C,0XF7,
0X7D,0XF7,0X7C,0XF7,0X7C,0XF7,0X7D,0XF7,0X7D,0XF7,0X9D,0XF7,0X9D,0XEF,0X9D,0XF7,
0X9C,0XF7,0X9C,0XF7,0XF6,0XC5,0X17,0XC6,0X38,0XCE,0X08,0X32,0X8A,0X1A,0X96,0X65,
0XD8,0X5D,0X8A,0X0A,0X27,0X2A,0X37,0XCE,0XF7,0XCD,0X38,0XD6,0X5C,0XFF,0X7C,0XFF,
0X7C,0XFF,0X9D,0XFF,0X9C,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0X9C,0XFF,0X9C,0XFF,
0X7C,0XFF,0X7C,0XFF,0X7C,0XFF,0X7C,0XFF,0X7C,0XFF,0X7C,0XFF,0X7C,0XFF,0X7C,0XFF,
0X7C,0XFF,0X7C,0XFF,0X9C,0XFF,0X9C,0XFF,0X9D,0XFF,0X9D,0XFF,0X9D,0XFF,0X9C,0XFF,
0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,
0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,
0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,
0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,
0X9C,0XFF,0XBC,0XFF,0X9C,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBB,0XFF,0X9B,0XFF,
0X57,0XD6,0X37,0XCE,0X58,0XBE,0X68,0X2A,0XCA,0X0A,0XD6,0X5D,0XB8,0X55,0XD7,0X65,
0X75,0X75,0X28,0X22,0X69,0X3A,0X07,0X3A,0X37,0XCE,0XD5,0XCD,0XD6,0XCD,0XF6,0XCD,
0XF5,0XCD,0XF6,0XD5,0XF5,0XCD,0XF6,0XD5,0XF6,0XCD,0XF6,0XD5,0XF6,0XD5,0XF6,0XD5,
0XF6,0XD5,0XF6,0XD5,0XF6,0XD5,0XF6,0XD5,0XF6,0XD5,0XF6,0XD5,0XD6,0XCD,0XD6,0XCD,
0XF6,0XD5,0XF6,0XD5,0XF6,0XD5,0XF6,0XD5,0XF6,0XD5,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,
0XF6,0XCD,0XF6,0XCD,0XD5,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,
0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,
0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,
0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF5,0XCD,0XF5,0XD5,0XF5,0XD5,
0X15,0XCE,0X16,0XCE,0X15,0XCE,0XF5,0XCD,0XF5,0XCD,0XF5,0XC5,0XE6,0X31,0X28,0X2A,
0X69,0X22,0X55,0X6D,0XF7,0X5D,0XF7,0X55,0XDA,0X4D,0XF9,0X55,0XD7,0X55,0XD7,0X5D,
0X97,0X5D,0X76,0X75,0X07,0X32,0XE7,0X49,0XC7,0X49,0XC6,0X51,0XA6,0X51,0XA6,0X51,
0XA6,0X51,0XC7,0X51,0XA7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC6,0X51,0XC6,0X51,
0XC6,0X51,0XC6,0X51,0XC6,0X51,0XC6,0X51,0XC6,0X51,0XC6,0X51,0XC6,0X51,0XC6,0X51,
0XC6,0X51,0XC6,0X51,0XC6,0X51,0XC6,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,
0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,
0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,
0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,
0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XA6,0X51,0XC6,0X51,0XE6,0X49,0XE7,0X49,
0XC7,0X49,0XC7,0X49,0XA6,0X51,0XE8,0X39,0X77,0X7D,0X98,0X65,0XB8,0X5D,0XB8,0X55,
0X19,0X56,0XD8,0X45,0XFB,0X4D,0X1A,0X4E,0X19,0X46,0XF8,0X45,0X19,0X4E,0XB7,0X5D,
0X28,0X2A,0XE7,0X49,0XE7,0X51,0XC7,0X51,0XA7,0X59,0XC7,0X59,0XC7,0X51,0XC8,0X51,
0XA8,0X51,0XA8,0X51,0XA7,0X51,0XA7,0X51,0XA7,0X51,0XA7,0X51,0XA7,0X51,0XA7,0X51,
0XA7,0X51,0XA7,0X51,0XA7,0X51,0XA7,0X51,0XA7,0X49,0XA7,0X49,0XA7,0X51,0XA7,0X51,
0XA7,0X51,0XA7,0X51,0X87,0X51,0X87,0X51,0X87,0X51,0X87,0X51,0X87,0X51,0X87,0X51,
0X87,0X51,0X87,0X51,0XA7,0X51,0XA7,0X51,0XA7,0X51,0XA7,0X51,0XA7,0X51,0XA7,0X51,
0XA7,0X51,0XA7,0X51,0X87,0X51,0X87,0X51,0X87,0X51,0X87,0X51,0X87,0X51,0X87,0X51,
0X87,0X51,0X87,0X51,0X87,0X51,0X87,0X51,0X87,0X51,0X87,0X51,0X87,0X51,0X87,0X51,
0X87,0X51,0X87,0X51,0XA7,0X59,0XC7,0X51,0XE7,0X49,0XE7,0X49,0XC8,0X49,0XC8,0X49,
0X88,0X51,0XE9,0X39,0X78,0X65,0X1A,0X56,0XD9,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,
};
#line 5 "app\\game.c"
#line 1 ".\\app\\RES\\Pic\\flappybird_button_stop.c"
const unsigned char gImage_flappybird_button_stop[6864] = {  
0XD9,0X4D,0X1A,0X4E,0XF9,0X45,0X19,0X4E,0XF9,0X55,0XB7,0X65,0X09,0X2A,0XE8,0X41,
0XC7,0X41,0XC7,0X41,0XC8,0X49,0XC8,0X41,0XC8,0X41,0XC8,0X41,0XC8,0X41,0XC7,0X41,
0XC7,0X49,0XC7,0X49,0XC7,0X49,0XC7,0X41,0XE7,0X49,0XE8,0X49,0XC8,0X49,0XA7,0X49,
0XC7,0X49,0XC7,0X49,0XC8,0X41,0XC8,0X41,0XC8,0X41,0XC8,0X41,0XC8,0X41,0XC8,0X49,
0XC8,0X49,0XC8,0X49,0XC8,0X49,0XC7,0X49,0XC7,0X49,0XE7,0X49,0XE7,0X49,0XE7,0X49,
0XC7,0X49,0XC7,0X49,0XC7,0X49,0XE7,0X41,0XE7,0X41,0XE7,0X41,0XC7,0X41,0XC7,0X49,
0XC7,0X49,0XC7,0X49,0XC7,0X49,0XC7,0X49,0XC7,0X49,0XC7,0X49,0XC7,0X49,0XC7,0X49,
0XC7,0X49,0XC7,0X49,0XC7,0X49,0XC7,0X49,0XC7,0X49,0XC7,0X49,0XC7,0X49,0XC7,0X49,
0XC7,0X49,0XC7,0X49,0XE7,0X41,0XE7,0X41,0XE7,0X41,0XC7,0X49,0XA8,0X51,0X09,0X3A,
0XD7,0X65,0XF8,0X45,0XF9,0X4D,0XD9,0X45,0XF9,0X4D,0X1A,0X56,0XF9,0X55,0XB8,0X4D,
0XB9,0X55,0XD8,0X5D,0X97,0X5D,0X96,0X75,0X29,0X32,0X87,0X39,0XA6,0X39,0XA6,0X39,
0XA7,0X39,0XA7,0X41,0XA7,0X39,0XC7,0X39,0XC7,0X39,0XC7,0X41,0XA7,0X41,0XA7,0X41,
0X86,0X39,0X86,0X39,0XA6,0X39,0XA7,0X41,0XA7,0X41,0X86,0X41,0XA7,0X41,0XA7,0X41,
0XA7,0X41,0XC7,0X39,0XC7,0X39,0XA7,0X39,0XA7,0X41,0XA7,0X41,0XC7,0X41,0XC7,0X41,
0XA7,0X41,0XA7,0X41,0XA7,0X41,0XC6,0X41,0XC7,0X39,0XC7,0X41,0XC7,0X41,0XC7,0X41,
0XC7,0X41,0XC7,0X41,0XC7,0X39,0XC7,0X39,0XC7,0X41,0XC7,0X41,0XC7,0X41,0XC7,0X41,
0XC7,0X41,0XC7,0X41,0XC7,0X41,0XC7,0X41,0XC7,0X41,0XC7,0X41,0XC7,0X41,0XC7,0X41,
0XC7,0X41,0XC7,0X41,0XC7,0X41,0XC7,0X41,0XC7,0X41,0XC7,0X41,0XA7,0X41,0XA6,0X41,
0XC7,0X39,0XC7,0X39,0XC6,0X39,0XA6,0X41,0X87,0X41,0XC8,0X31,0X76,0X6D,0XD8,0X5D,
0XD9,0X5D,0XF9,0X5D,0XB8,0X55,0XB8,0X4D,0XB8,0X5D,0XB8,0X65,0X77,0X7D,0X09,0X1A,
0X29,0X2A,0X29,0X32,0XBF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0X09,0X2A,0XE9,0X19,0X2A,0X1A,0X77,0X6D,
0X97,0X5D,0XF8,0X5D,0X97,0X65,0XAB,0X1A,0X2A,0X2A,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XF7,0XFE,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,
0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,
0XDF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XF7,0X29,0X22,0XEC,0X1A,0XB7,0X5D,
0X76,0X75,0XAB,0X2A,0XC7,0X29,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XDE,0XF7,0XFF,0XF7,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XF7,
0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,
0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X28,0X32,0X49,0X12,0X96,0X6D,0X35,0X85,0X29,0X2A,
0XE7,0X31,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XF7,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XF7,
0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,
0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFD,0XF7,
0XFF,0XFF,0XE7,0X31,0XE8,0X21,0X55,0X85,0X28,0X3A,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,
0XFE,0XF7,0XFF,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,
0XFE,0XFF,0XFE,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFE,0XF7,0XFF,0XF7,0XFF,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,
0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,
0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XF7,
0XFF,0XFF,0XC8,0X31,0XA6,0X39,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,
0XFE,0XF7,0XFF,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,
0XFF,0XF7,0XFF,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,
0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,
0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XDE,0XFF,0XDF,0XFF,0XE8,0X41,
0XA7,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,
0XFE,0XF7,0XFE,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFF,0XF7,0XFF,0XF7,
0XFF,0XF7,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFF,0XF7,0XFF,0XF7,
0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,0XDE,0XF7,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,
0XFE,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,0XDF,0XFF,0XC7,0X41,0XA7,0X49,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,
0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFF,0XF7,0XFF,0XF7,0XFE,0XF7,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XF7,0XFF,0XF7,0XFF,0XF7,0XFE,0XF7,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,
0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XF7,0XFF,0XF7,0XFE,0XF7,0XFE,0XF7,
0XFF,0XF7,0XFF,0XFF,0XDF,0XFF,0XC7,0X41,0XA7,0X41,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDE,0XFF,0XDE,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XF7,0XFF,0XF7,0XFE,0XEF,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFF,0XF7,0XFF,0XF7,0XDE,0XF7,
0XFE,0XF7,0XFE,0XFF,0XDE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,
0XFE,0XF7,0XFF,0XF7,0XFF,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XC7,0X41,0XA7,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XBE,0XF7,0XFF,0XFF,
0XFF,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,
0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFF,0XF7,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0XDE,0XFF,
0XDE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFE,0XF7,0XFF,0XF7,0XFF,0XF7,
0XFF,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XDF,0XFF,0XC8,0X41,
0XA7,0X41,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,
0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,
0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XBD,0XF7,0XFE,0XFF,
0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFF,0XFF,0XBE,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XF7,
0XFF,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,
0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,0XFE,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XDF,0XFF,0XC8,0X41,0XA7,0X41,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,
0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFE,0XF7,0XFE,0XEF,
0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFD,0XFF,0XFE,0XFF,0XBD,0XFF,0XBD,0XFF,0XBD,0XFF,
0XBD,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBD,0XFF,
0X9D,0XFF,0X9D,0XFF,0X9D,0XFF,0XBE,0XFF,0XBD,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,
0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,
0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,
0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFF,0XFF,0XDF,0XFF,0XC8,0X41,0XA7,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFE,0XF7,0XFE,0XF7,0XFD,0XF7,
0XFD,0XFF,0XFD,0XFF,0X9C,0XFF,0X90,0XC4,0X4E,0XDC,0X2D,0XE4,0X2D,0XE4,0X2D,0XE4,
0X2D,0XE4,0X2D,0XE4,0X2D,0XE4,0X2D,0XE4,0X2D,0XE4,0X2D,0XE4,0X0D,0XEC,0X2E,0XEC,
0X2E,0XE4,0X2E,0XD4,0XAF,0XCC,0XBB,0XFF,0XFD,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,0XDF,0XFF,
0XBF,0XFF,0XC8,0X41,0XA7,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFF,0XF7,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFD,0XF7,0XFE,0XF7,0XFE,0XFF,0XBD,0XFF,
0X9D,0XFF,0X4F,0XD4,0X0D,0XF4,0XEB,0XFB,0XEB,0XFB,0XEB,0XFB,0XCB,0XFB,0XCB,0XFB,
0XEB,0XFB,0XEB,0XFB,0XEB,0XFB,0XCB,0XFB,0XCB,0XFB,0XAB,0XFB,0XCB,0XFB,0X2C,0XEC,
0X6D,0XD4,0XBA,0XFF,0XDD,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,0XDF,0XFF,0XBF,0XFF,0XA8,0X41,
0XA7,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XDE,0XFF,0X9C,0XFF,0X4F,0XD4,
0X0C,0XF4,0XEB,0XFB,0XCB,0XFB,0XCB,0XFB,0XCB,0XFB,0XCB,0XFB,0XEB,0XFB,0XEB,0XFB,
0XEB,0XFB,0XCB,0XFB,0XCB,0XFB,0XAB,0XFB,0XEC,0XFB,0X0C,0XF4,0X4D,0XDC,0X9B,0XFF,
0XDD,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,
0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XC8,0X41,0XC7,0X39,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF7,0XFE,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XF7,
0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XDE,0XFF,0X9C,0XFF,0X2E,0XDC,0X0C,0XF4,0XEB,0XFB,
0XCB,0XFB,0XCB,0XFB,0XCB,0XFB,0XCB,0XFB,0XEB,0XFB,0XEB,0XFB,0XEB,0XFB,0XEB,0XFB,
0XCB,0XFB,0XCB,0XFB,0XCB,0XFB,0XEC,0XF3,0X4D,0XDC,0X9B,0XFF,0XDD,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XC7,0X41,0XC7,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFE,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF7,
0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,
0XFE,0XFF,0XDE,0XFF,0X7C,0XFF,0X2E,0XDC,0X0C,0XF4,0XEB,0XFB,0XCB,0XFB,0XCB,0XFB,
0XCB,0XFB,0XCB,0XFB,0XEB,0XFB,0XEA,0XFB,0XEB,0XFB,0XEB,0XFB,0XCB,0XFB,0XCB,0XFB,
0XEB,0XFB,0X0C,0XEC,0X6D,0XD4,0X9B,0XFF,0XDD,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,0XDF,0XFF,
0XDF,0XFF,0XE8,0X41,0XC7,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,0XFE,0XF7,
0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XBD,0XFF,
0X7B,0XFF,0X4E,0XDC,0X0C,0XF4,0XEB,0XFB,0XCB,0XFB,0XCB,0XFB,0XCB,0XFB,0XCB,0XFB,
0XEA,0XFB,0XEA,0XFB,0XEA,0XFB,0XEB,0XFB,0XEB,0XFB,0XCB,0XFB,0XEB,0XFB,0X0C,0XEC,
0X6E,0XDC,0X9B,0XFF,0XDD,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,
0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XC7,0X41,
0XC7,0X41,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFE,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XBD,0XFF,0X7B,0XFF,0X6E,0XDC,
0X0C,0XF4,0XEB,0XFB,0XCB,0XFB,0XCB,0XFB,0XCB,0XFB,0XCB,0XFB,0XCB,0XFB,0XEA,0XFB,
0XEA,0XFB,0XEB,0XFB,0XEB,0XFB,0XCB,0XFB,0XEB,0XFB,0X0C,0XF4,0X6E,0XDC,0X7B,0XFF,
0XDD,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XE8,0X41,0XA7,0X39,0XBF,0XFF,
0X9E,0XF7,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XF7,0X9D,0XF7,
0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XF7,0X9D,0XF7,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7E,0XEF,0X7D,0XEF,0X7D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X9D,0XF7,0X7C,0XFF,0X3A,0XFF,0X2D,0XD4,0X0C,0XF4,0XEB,0XFB,
0XCB,0XFB,0XCB,0XFB,0XCB,0XFB,0XCB,0XFB,0XEB,0XFB,0XEA,0XFB,0XEB,0XFB,0XEB,0XFB,
0XEB,0XFB,0XCB,0XFB,0XCB,0XFB,0XEC,0XF3,0X4E,0XDC,0X3A,0XFF,0X5C,0XFF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7E,0XF7,0X7E,0XFF,0XA7,0X41,0XC8,0X41,0XDF,0XFF,0X9E,0XF7,0X9E,0XF7,
0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X7E,0XF7,0X7E,0XF7,0X7D,0XF7,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,
0XBD,0XEF,0X7C,0XFF,0X3A,0XFF,0X4E,0XDC,0X0C,0XF4,0XEB,0XFB,0XEB,0XFB,0XEB,0XFB,
0XEB,0XFB,0XEB,0XFB,0XEA,0XFB,0XEA,0XFB,0XEB,0XFB,0XEB,0XFB,0XCB,0XFB,0XCB,0XFB,
0XCB,0XFB,0XED,0XF3,0X2E,0XDC,0X1A,0XFF,0X5C,0XFF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9E,0XF7,0X9E,0XF7,0X9E,0XF7,0X9E,0XF7,
0X9F,0XFF,0XA7,0X41,0XC8,0X41,0XBF,0XFF,0X9E,0XF7,0X9E,0XF7,0X9E,0XF7,0X9E,0XF7,
0X9D,0XF7,0X7D,0XEF,0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,
0X7D,0XF7,0X7D,0XF7,0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,
0X7E,0XF7,0X7E,0XF7,0X7E,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X5B,0XF7,
0X3A,0XFF,0X4E,0XDC,0X0C,0XF4,0XCB,0XFB,0XEB,0XFB,0XEB,0XFB,0XEB,0XFB,0XEB,0XFB,
0XEA,0XFB,0XEA,0XFB,0XEB,0XFB,0XCB,0XFB,0XCB,0XFB,0XCB,0XFB,0XCB,0XFB,0XEC,0XF3,
0X2E,0XDC,0XFA,0XFE,0X5C,0XFF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7E,0XF7,0X7E,0XF7,0X7E,0XFF,0XA7,0X41,
0XC8,0X41,0XBF,0XFF,0X7E,0XF7,0X9E,0XF7,0X9E,0XF7,0X9E,0XEF,0X7D,0XEF,0X7E,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XF7,
0X9D,0XEF,0X9D,0XEF,0X7E,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X5E,0XF7,0X7E,0XF7,
0X9E,0XF7,0X9E,0XEF,0X7D,0XEF,0XBD,0XEF,0X9D,0XEF,0X5C,0XFF,0X1A,0XFF,0X4E,0XDC,
0XEC,0XF3,0XCB,0XFB,0XCB,0XFB,0XEB,0XFB,0XEB,0XFB,0XEB,0XFB,0XEA,0XFB,0XEA,0XFB,
0XCA,0XFB,0XCA,0XFB,0XCB,0XFB,0XAB,0XFB,0XCB,0XFB,0XEC,0XF3,0X2E,0XDC,0X1A,0XFF,
0X5C,0XFF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7E,0XEF,0X7E,0XEF,0X7E,0XF7,0X7E,0XF7,0X5E,0XF7,0XA8,0X41,0XC8,0X41,0XBF,0XFF,
0X7E,0XF7,0X7E,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XF7,
0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XF7,0X9D,0XEF,0X9D,0XEF,
0X7E,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7E,0XF7,0X7E,0XF7,0X9E,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X7D,0XF7,0X5C,0XFF,0XDA,0XFE,0X2E,0XDC,0XEC,0XFB,0XCA,0XFB,
0XCA,0XFB,0XCA,0XFB,0XCB,0XFB,0XCA,0XFB,0XEA,0XFB,0XCA,0XFB,0XCA,0XFB,0XCB,0XFB,
0XCB,0XFB,0XCB,0XFB,0XCB,0XFB,0XEC,0XF3,0X6E,0XDC,0X3A,0XFF,0X5C,0XFF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7E,0XEF,0X7E,0XEF,
0X7E,0XEF,0X7E,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7E,0XEF,0X9E,0XF7,
0X7E,0XF7,0X7E,0XF7,0X7E,0XFF,0XC8,0X41,0XC8,0X41,0XBF,0XFF,0X7E,0XF7,0X7E,0XEF,
0X9E,0XF7,0X7E,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XF7,0X9D,0XEF,0X9D,0XEF,0X7E,0XF7,0X7D,0XF7,
0X7D,0XF7,0X7D,0XEF,0X7E,0XEF,0X7E,0XEF,0X7D,0XEF,0XBE,0XEF,0XBE,0XF7,0X5D,0XEF,
0X5D,0XF7,0X5D,0XFF,0XFB,0XFE,0X2F,0XE4,0XEC,0XFB,0XCB,0XFB,0XEB,0XFB,0XEB,0XFB,
0XEB,0XFB,0XEB,0XFB,0XEB,0XFB,0XEB,0XFB,0XEB,0XFB,0XAB,0XFB,0XAB,0XFB,0XEC,0XFB,
0XEC,0XFB,0XEC,0XEB,0X6E,0XD4,0X1A,0XFF,0X5C,0XFF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XF7,0X7E,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,0X7D,0XEF,0X7E,0XEF,0X9E,0XF7,0X7E,0XF7,0X7E,0XF7,
0X7E,0XFF,0XC8,0X49,0XA7,0X41,0XBF,0XFF,0X7E,0XF7,0X7E,0XEF,0X9E,0XEF,0X9E,0XEF,
0X7D,0XEF,0X9E,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,
0X7D,0XF7,0X7D,0XF7,0X9D,0XEF,0X9D,0XEF,0X7E,0XF7,0X7D,0XF7,0X9D,0XEF,0X9D,0XEF,
0X9E,0XEF,0X9E,0XEF,0XBE,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XF7,0X7E,0XF7,0X3C,0XFF,
0XDA,0XFE,0X0E,0XDC,0XEC,0XF3,0XCB,0XFB,0XCB,0XFB,0XCB,0XFB,0XCB,0XFB,0XCB,0XFB,
0XCB,0XFB,0XCB,0XFB,0XAB,0XFB,0XCB,0XFB,0XCB,0XFB,0XCB,0XFB,0XCB,0XF3,0X0C,0XEC,
0X8E,0XDC,0X3A,0XFF,0X5C,0XFF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XF7,0X7E,0XF7,0X7D,0XF7,0X7D,0XF7,
0X7D,0XEF,0X7D,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XF7,0X7E,0XF7,0X7E,0XFF,0XC8,0X41,
0XA7,0X41,0X9E,0XFF,0X5D,0XF7,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X9E,0XF7,
0X7E,0XF7,0X7E,0XF7,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XF7,
0X7E,0XEF,0X7E,0XEF,0X7E,0XF7,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9E,0XEF,0X9E,0XEF,
0X9D,0XEF,0X9E,0XEF,0X9E,0XF7,0X7D,0XF7,0X5D,0XEF,0X5D,0XFF,0X3C,0XFF,0X4F,0XDC,
0X0C,0XF4,0XEB,0XFB,0XEB,0XFB,0XEB,0XFB,0XCB,0XFB,0XCB,0XFB,0XCB,0XFB,0XCB,0XFB,
0XEB,0XFB,0XCB,0XFB,0XCB,0XFB,0XEB,0XFB,0X0B,0XFC,0X0C,0XF4,0X2D,0XDC,0X19,0XFF,
0X5C,0XFF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,0X7D,0XEF,
0X7E,0XEF,0X9E,0XF7,0X7E,0XF7,0X9E,0XF7,0X7E,0XFF,0XC8,0X41,0XC7,0X41,0XBF,0XFF,
0X7E,0XF7,0X7E,0XEF,0X7E,0XEF,0X9E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XF7,0X7E,0XEF,
0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7D,0XF7,0X7D,0XF7,0X7E,0XF7,0X7E,0XF7,
0X7E,0XF7,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9E,0XF7,
0X5D,0XEF,0X7E,0XEF,0X9E,0XF7,0X9D,0XFF,0X1B,0XFF,0X90,0XCC,0X4E,0XDC,0X2D,0XE4,
0X2D,0XE4,0X2D,0XE4,0X0D,0XEC,0X0D,0XEC,0X0D,0XEC,0X2D,0XEC,0X0D,0XE4,0X2D,0XE4,
0X4D,0XE4,0X2D,0XE4,0X0D,0XE4,0X0D,0XDC,0X90,0XD4,0X1B,0XFF,0X5D,0XFF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,0X7D,0XEF,0X7E,0XEF,0X7E,0XEF,
0X7E,0XF7,0X9E,0XF7,0X7E,0XFF,0XC8,0X41,0XC7,0X41,0XBF,0XFF,0X7E,0XF7,0X9E,0XF7,
0X9E,0XEF,0X9D,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XF7,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,
0X9D,0XEF,0X9D,0XEF,0X7D,0XF7,0X7D,0XF7,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,0X7D,0XF7,
0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XF7,0X7E,0XF7,0X5D,0XF7,0X5D,0XF7,0X9E,0XF7,
0X9E,0XEF,0X7D,0XE7,0X9C,0XFF,0X5B,0XFF,0X3B,0XFF,0X3B,0XFF,0X3B,0XFF,0X3B,0XFF,
0X3B,0XFF,0X3B,0XFF,0X3B,0XFF,0X3B,0XFF,0X3B,0XFF,0X5B,0XFF,0X5B,0XFF,0X1B,0XFF,
0X1B,0XFF,0X5C,0XFF,0X1C,0XFF,0X5D,0XF7,0X7D,0XF7,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7E,0XF7,0X7E,0XF7,
0X7E,0XFF,0XC8,0X41,0XC8,0X41,0XBF,0XFF,0X7E,0XF7,0X7D,0XEF,0X9E,0XF7,0X9D,0XEF,
0X7E,0XEF,0X9E,0XF7,0X7E,0XF7,0X7E,0XEF,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X7D,0XEF,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7E,0XF7,0X7D,0XF7,0X9D,0XEF,0X9D,0XEF,
0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X5D,0XEF,0X9E,0XEF,0XBE,0XEF,
0X7C,0XEF,0X9C,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,
0X7D,0XF7,0X9D,0XEF,0X9E,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XF7,0X7D,0XF7,0X3D,0XF7,
0X9F,0XF7,0X7E,0XEF,0X7E,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,
0X7D,0XEF,0X7D,0XEF,0X9E,0XF7,0X9E,0XF7,0X9E,0XF7,0X7E,0XF7,0X7E,0XF7,0XC8,0X41,
0XE8,0X39,0XBF,0XF7,0X9E,0XF7,0X7E,0XF7,0X7D,0XF7,0X7D,0XEF,0X7E,0XF7,0X7E,0XF7,
0X9E,0XF7,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XF7,
0X7D,0XF7,0X7D,0XEF,0X7D,0XF7,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7D,0XF7,
0X7D,0XF7,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,0X7D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XF7,0X7E,0XEF,0X7E,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X9D,0XEF,0X9D,0XEF,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X9E,0XF7,0X9E,0XF7,
0X7D,0XEF,0X9E,0XF7,0X9E,0XEF,0X5D,0XE7,0X9E,0XF7,0XC8,0X49,0X08,0X3A,0XDF,0XFF,
0X5E,0XF7,0X7D,0XF7,0X7D,0XF7,0X9D,0XF7,0X9E,0XF7,0X7E,0XF7,0X9E,0XF7,0X9D,0XF7,
0X7D,0XF7,0X9D,0XF7,0X9D,0XEF,0X9D,0XEF,0X9E,0XF7,0X7E,0XF7,0X7D,0XF7,0X7D,0XEF,
0X7D,0XF7,0X7D,0XEF,0X7D,0XF7,0X7D,0XEF,0X7D,0XF7,0X7D,0XEF,0X7D,0XF7,0X7D,0XEF,
0X7D,0XF7,0X7D,0XEF,0X7D,0XF7,0X7D,0XEF,0X7D,0XF7,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XEF,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,0X5D,0XEF,
0X9E,0XEF,0X9E,0XEF,0X7E,0XFF,0XE8,0X49,0XA6,0X41,0XBE,0XFF,0X9D,0XFF,0X7D,0XF7,
0X7D,0XEF,0X7D,0XEF,0X9E,0XF7,0X5D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7D,0XF7,0X7D,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,
0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,0X7D,0XF7,0X7E,0XF7,0X7D,0XF7,
0X7E,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7E,0XF7,0X7E,0XF7,0X7E,0XEF,0X7E,0XEF,
0X7E,0XEF,0X7E,0XEF,0X7E,0XF7,0X7E,0XF7,0X9E,0XF7,0X9D,0XF7,0X9E,0XF7,0X9E,0XF7,
0X7E,0XFF,0XA7,0X41,0XC7,0X49,0XBE,0XFF,0X7C,0XF7,0XDE,0XFF,0X9E,0XF7,0XBE,0XF7,
0X7E,0XEF,0X7E,0XF7,0X7D,0XF7,0X9D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7E,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,
0X7E,0XEF,0X7E,0XEF,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,0X7E,0XF7,
0X7E,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7E,0XF7,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X9E,0XEF,0X7E,0XF7,
0X7E,0XF7,0X7E,0XF7,0X9E,0XF7,0X5C,0XEF,0XDE,0XFF,0X7D,0XF7,0X9E,0XFF,0XC7,0X41,
0XA6,0X41,0X58,0XD6,0X58,0XCE,0X9D,0XF7,0XDE,0XF7,0X9E,0XEF,0X5D,0XEF,0X7E,0XF7,
0X7E,0XF7,0X7D,0XEF,0X9D,0XEF,0X9E,0XEF,0X9E,0XEF,0X9E,0XEF,0X7E,0XF7,0X7E,0XF7,
0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,
0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,
0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,
0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7D,0XEF,
0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,0X7E,0XEF,0X7E,0XF7,0X7E,0XF7,0X9E,0XF7,0X9D,0XEF,
0X9D,0XF7,0XDE,0XFF,0XBD,0XFF,0X17,0XCE,0XF7,0XCD,0XE7,0X49,0X08,0X32,0X17,0XBE,
0X17,0XC6,0X9D,0XF7,0X9D,0XF7,0XBE,0XF7,0X9E,0XF7,0X9E,0XF7,0X7E,0XEF,0X7D,0XEF,
0X7E,0XEF,0X9E,0XEF,0X7E,0XEF,0X9E,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XEF,0X7E,0XEF,
0X7E,0XEF,0X7E,0XEF,0X7D,0XEF,0X9D,0XEF,0X7D,0XEF,0X9D,0XEF,0X7D,0XEF,0X9D,0XEF,
0X7D,0XEF,0X9D,0XEF,0X7D,0XEF,0X9D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7E,0XEF,0X7E,0XEF,
0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,
0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,
0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,
0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7E,0XEF,0X7D,0XEF,0X9D,0XEF,0X9D,0XEF,
0X9D,0XEF,0X7E,0XEF,0X7E,0XF7,0X7E,0XF7,0X7D,0XEF,0X9D,0XEF,0XBD,0XF7,0X9C,0XF7,
0X7C,0XF7,0XF6,0XC5,0X58,0XC6,0XE7,0X39,0X55,0X7D,0XCB,0X32,0X06,0X3A,0XBD,0XFF,
0X7C,0XFF,0X7D,0XF7,0X9E,0XF7,0X5E,0XEF,0X7E,0XEF,0X7E,0XF7,0X7E,0XF7,0X9D,0XF7,
0X9E,0XF7,0X9D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XF7,0X7D,0XEF,
0X7D,0XF7,0X7D,0XEF,0X7D,0XF7,0X7D,0XEF,0X7D,0XF7,0X7D,0XEF,0X7D,0XF7,0X7D,0XEF,
0X7D,0XF7,0X7D,0XEF,0X7D,0XF7,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,
0X7D,0XEF,0X7D,0XEF,0X7D,0XEF,0X9D,0XEF,0X7D,0XF7,0X7D,0XF7,0X7E,0XEF,0X7E,0XEF,
0X7E,0XEF,0X9E,0XEF,0X7D,0XF7,0X7C,0XF7,0X9D,0XEF,0XBD,0XF7,0X7D,0XFF,0X28,0X42,
0X29,0X2A,0X76,0X7D,0X97,0X6D,0X8A,0X1A,0X47,0X3A,0X37,0XCE,0XF7,0XCD,0XB6,0XC5,
0X9D,0XFF,0XBE,0XFF,0XBE,0XFF,0XBE,0XFF,0XBE,0XFF,0XBE,0XFF,0XBD,0XFF,0XBD,0XF7,
0X9D,0XF7,0X7C,0XF7,0XBE,0XFF,0XBE,0XFF,0XBE,0XFF,0XBE,0XFF,0XBE,0XFF,0XBE,0XFF,
0XBE,0XFF,0XBE,0XFF,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,
0X7D,0XF7,0X7D,0XF7,0X7D,0XF7,0X9D,0XF7,0X7D,0XF7,0X9D,0XF7,0X7D,0XF7,0X9D,0XF7,
0X7D,0XF7,0X9D,0XF7,0X7D,0XF7,0X9D,0XF7,0X7D,0XF7,0X9D,0XF7,0X7D,0XF7,0X9D,0XF7,
0X7D,0XF7,0X9D,0XF7,0X7D,0XF7,0X9C,0XF7,0X7D,0XF7,0X9C,0XF7,0X7D,0XF7,0X9C,0XF7,
0X7D,0XF7,0X9C,0XF7,0X7D,0XF7,0X9C,0XF7,0X7D,0XF7,0X9C,0XF7,0X7D,0XF7,0X9C,0XF7,
0X7D,0XF7,0X7C,0XF7,0X7C,0XF7,0X7D,0XF7,0X7D,0XF7,0X9D,0XF7,0X9D,0XEF,0X9D,0XF7,
0X9C,0XF7,0X9C,0XF7,0XF6,0XC5,0X17,0XC6,0X38,0XCE,0X08,0X32,0X8A,0X1A,0X96,0X65,
0XD8,0X5D,0X8A,0X0A,0X27,0X2A,0X37,0XCE,0XF7,0XCD,0X38,0XD6,0X5C,0XFF,0X7C,0XFF,
0X7C,0XFF,0X9D,0XFF,0X9C,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0X9C,0XFF,0X9C,0XFF,
0X7C,0XFF,0X7C,0XFF,0X7C,0XFF,0X7C,0XFF,0X7C,0XFF,0X7C,0XFF,0X7C,0XFF,0X7C,0XFF,
0X7C,0XFF,0X7C,0XFF,0X9C,0XFF,0X9C,0XFF,0X9D,0XFF,0X9D,0XFF,0X9D,0XFF,0X9C,0XFF,
0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,
0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,
0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,
0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,
0X9C,0XFF,0XBC,0XFF,0X9C,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBB,0XFF,0X9B,0XFF,
0X57,0XD6,0X37,0XCE,0X58,0XBE,0X68,0X2A,0XCA,0X0A,0XD6,0X5D,0XB8,0X55,0XD7,0X65,
0X75,0X75,0X28,0X22,0X69,0X3A,0X07,0X3A,0X37,0XCE,0XD5,0XCD,0XD6,0XCD,0XF6,0XCD,
0XF5,0XCD,0XF6,0XD5,0XF5,0XCD,0XF6,0XD5,0XF6,0XCD,0XF6,0XD5,0XF6,0XD5,0XF6,0XD5,
0XF6,0XD5,0XF6,0XD5,0XF6,0XD5,0XF6,0XD5,0XF6,0XD5,0XF6,0XD5,0XD6,0XCD,0XD6,0XCD,
0XF6,0XD5,0XF6,0XD5,0XF6,0XD5,0XF6,0XD5,0XF6,0XD5,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,
0XF6,0XCD,0XF6,0XCD,0XF5,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,
0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,
0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,
0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF6,0XCD,0XF5,0XCD,0XF5,0XD5,0XF5,0XD5,
0X15,0XCE,0X16,0XCE,0X15,0XCE,0XF5,0XCD,0XF5,0XCD,0XF5,0XC5,0XE6,0X31,0X28,0X2A,
0X69,0X22,0X55,0X6D,0XF7,0X5D,0XF7,0X55,0XDA,0X4D,0XF9,0X55,0XD7,0X55,0XD7,0X5D,
0X97,0X5D,0X76,0X75,0X07,0X32,0XE7,0X49,0XC7,0X49,0XC6,0X51,0XA6,0X51,0XA6,0X51,
0XA6,0X51,0XC7,0X51,0XA7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC6,0X51,0XC6,0X51,
0XC6,0X51,0XC6,0X51,0XC6,0X51,0XC6,0X51,0XC6,0X51,0XC6,0X51,0XC6,0X51,0XC6,0X51,
0XC6,0X51,0XC6,0X51,0XC6,0X51,0XC6,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,
0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,
0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,
0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,
0XC7,0X51,0XC7,0X51,0XC7,0X51,0XC7,0X51,0XA6,0X51,0XC6,0X51,0XE6,0X49,0XE7,0X49,
0XC7,0X49,0XC7,0X49,0XA6,0X51,0XE8,0X39,0X77,0X7D,0X98,0X65,0XB8,0X5D,0XB8,0X55,
0X19,0X56,0XD8,0X45,0XFB,0X4D,0X1A,0X4E,0X19,0X46,0XF8,0X45,0X19,0X4E,0XB7,0X5D,
0X28,0X2A,0XE7,0X49,0XE7,0X51,0XC7,0X51,0XA7,0X59,0XC7,0X59,0XC7,0X51,0XC8,0X51,
0XA8,0X51,0XA8,0X51,0XA7,0X51,0XA7,0X51,0XA7,0X51,0XA7,0X51,0XA7,0X51,0XA7,0X51,
0XA7,0X51,0XA7,0X51,0XA7,0X51,0XA7,0X51,0XA7,0X49,0XA7,0X49,0XA7,0X51,0XA7,0X51,
0XA7,0X51,0XA7,0X51,0X87,0X51,0X87,0X51,0X87,0X51,0X87,0X51,0X87,0X51,0X87,0X51,
0X87,0X51,0X87,0X51,0XA7,0X51,0XA7,0X51,0XA7,0X51,0XA7,0X51,0XA7,0X51,0XA7,0X51,
0XA7,0X51,0XA7,0X51,0X87,0X51,0X87,0X51,0X87,0X51,0X87,0X51,0X87,0X51,0X87,0X51,
0X87,0X51,0X87,0X51,0X87,0X51,0X87,0X51,0X87,0X51,0X87,0X51,0X87,0X51,0X87,0X51,
0X87,0X51,0X87,0X51,0XA7,0X59,0XC7,0X51,0XE7,0X49,0XE7,0X49,0XC8,0X49,0XC8,0X49,
0X88,0X51,0XE9,0X39,0X78,0X65,0X1A,0X56,0XD9,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,
};
#line 6 "app\\game.c"
#line 1 ".\\app\\RES\\Pic\\flappybird_bird_u.c"
const unsigned char gImage_flappybird_bird_u[936] = {  
0XD8,0X45,0X19,0X4E,0XF8,0X45,0XF8,0X45,0XF9,0X4D,0XD9,0X55,0X78,0X6D,0X36,0X7D,
0X14,0X95,0XE6,0X49,0X83,0X69,0X63,0X79,0X64,0X71,0X65,0X69,0XC7,0X59,0XC8,0X51,
0XA7,0X51,0XE8,0X49,0X56,0X8D,0X57,0X65,0XD9,0X55,0X1A,0X4E,0XF9,0X45,0X19,0X46,
0XF9,0X45,0XF9,0X45,0X19,0X4E,0XD8,0X45,0XF8,0X45,0XF8,0X4D,0XD8,0X4D,0XB8,0X65,
0X6A,0X22,0X29,0X3A,0X06,0X4A,0X2E,0XB4,0XCB,0XCB,0XEB,0XD3,0XCB,0XCB,0X0E,0XBC,
0XC7,0X59,0XBF,0XFF,0XBF,0XFF,0XBF,0XFF,0X2A,0X32,0X8C,0X1A,0XB8,0X5D,0XB8,0X45,
0X3A,0X4E,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0X19,0X4E,0XF9,0X4D,
0X97,0X55,0X76,0X75,0X06,0X42,0XA2,0X61,0X60,0X71,0XC9,0XDB,0X87,0XEB,0X66,0XEB,
0XA8,0XDB,0X4D,0XCC,0X85,0X49,0XFF,0XFF,0XBE,0XFF,0XFF,0XFF,0XE8,0X31,0X29,0X2A,
0X77,0X75,0XB8,0X5D,0XFA,0X4D,0XD9,0X45,0XF9,0X4D,0XD9,0X4D,0XFA,0X4D,0XFA,0X4D,
0XD9,0X5D,0X77,0X5D,0X77,0X6D,0X69,0X32,0X0D,0XAC,0XCA,0XCB,0XA8,0XEB,0X42,0XDA,
0X00,0XE2,0X41,0XDA,0X20,0X89,0X81,0X69,0XDE,0XFF,0XFF,0XF7,0XDE,0XF7,0XFF,0XFF,
0XDF,0XFF,0XFF,0XFF,0X4A,0X22,0X57,0X6D,0X98,0X5D,0XFA,0X55,0XD9,0X4D,0XF9,0X4D,
0XDA,0X45,0X99,0X55,0X6C,0X1A,0X2A,0X2A,0X6B,0X3A,0XC7,0X41,0XA4,0X61,0X41,0X81,
0X23,0XDA,0XE0,0XE9,0XE0,0XF1,0X22,0XE2,0X61,0X79,0XC5,0X59,0XF9,0XE6,0XFE,0XF7,
0XFE,0XF7,0XDF,0XF7,0XC8,0X41,0XC9,0X41,0XDF,0XF7,0X4A,0X2A,0X8C,0X1A,0X98,0X5D,
0XD9,0X4D,0XF9,0X45,0XB9,0X65,0X78,0X75,0X4B,0X32,0X09,0X42,0X08,0X42,0X07,0X4A,
0XA5,0X59,0X62,0X79,0X44,0XD2,0X02,0XEA,0XC0,0XF1,0X23,0XE2,0X42,0X79,0XC5,0X59,
0XF9,0XDE,0XFE,0XF7,0XFF,0XF7,0XFF,0XFF,0XE8,0X49,0XE8,0X49,0XDF,0XFF,0X29,0X3A,
0X6B,0X22,0XB8,0X6D,0XD8,0X55,0XF9,0X45,0X2A,0X3A,0X09,0X3A,0XBF,0XF7,0XDF,0XF7,
0XFE,0XFF,0XDD,0XFF,0XBE,0XFF,0XBD,0XFF,0X42,0X79,0X86,0XCA,0X03,0XE2,0X24,0XDA,
0X21,0X89,0X84,0X69,0XFA,0XEE,0XDE,0XF7,0XDE,0XF7,0XFF,0XFF,0XA6,0X41,0XE7,0X49,
0XDF,0XFF,0XE8,0X31,0X4A,0X2A,0X56,0X6D,0XD8,0X55,0XF9,0X4D,0XC8,0X51,0XE8,0X51,
0XDF,0XFF,0XDE,0XF7,0XDD,0XF7,0XDD,0XF7,0XFF,0XF7,0XDE,0XFF,0X7C,0XFF,0X62,0X81,
0XE0,0X98,0X24,0XDA,0X45,0XCA,0XA8,0XB2,0XA5,0X61,0XFB,0XEE,0XFB,0XE6,0XFF,0XFF,
0X9E,0XFF,0XBE,0XFF,0XDF,0XFF,0X08,0X3A,0X29,0X2A,0X76,0X7D,0XD8,0X55,0XF8,0X4D,
0XC9,0X51,0XE8,0X49,0XFF,0XFF,0XFD,0XF7,0XFD,0XF7,0XFE,0XF7,0XFF,0XF7,0XFE,0XFF,
0X9C,0XFF,0XA4,0X69,0X41,0X89,0X64,0XD2,0X03,0XDA,0X44,0XD2,0X43,0X79,0XBA,0XFE,
0X1B,0XEF,0XBE,0XFF,0XDF,0XFF,0XBF,0XFF,0XBF,0XFF,0X09,0X42,0X29,0X32,0X76,0X7D,
0XB7,0X5D,0XF9,0X4D,0X2B,0X3A,0XE9,0X31,0X3B,0XDF,0XFE,0XF7,0XFE,0XFF,0XFE,0XF7,
0XFF,0XF7,0XDE,0XFF,0XFA,0XF6,0X84,0X61,0X41,0X89,0X23,0XD2,0X02,0XEA,0X43,0XE2,
0X47,0XAA,0XA5,0X69,0XA4,0X51,0XC3,0X49,0XC4,0X51,0XC4,0X51,0XC4,0X51,0XC5,0X49,
0X25,0X4A,0X26,0X32,0X75,0X75,0XB7,0X65,0X78,0X6D,0X97,0X7D,0X28,0X32,0X1B,0XDF,
0XFA,0XDE,0XFA,0XDE,0X1B,0XD7,0XDA,0XE6,0XA5,0X61,0X88,0XAA,0X24,0XD2,0X02,0XEA,
0X01,0XF2,0X02,0XDA,0X42,0X81,0X72,0XE5,0XCF,0XED,0XAD,0XE5,0XCC,0XED,0X8B,0XE5,
0XCC,0XED,0XCC,0XE5,0XCC,0XE5,0XEF,0XDD,0X25,0X42,0X28,0X3A,0XB9,0X55,0X97,0X65,
0X49,0X22,0X3C,0XE7,0XFB,0XE6,0X1B,0XEF,0XFA,0XEE,0XD9,0XFE,0X43,0X79,0X87,0XCA,
0X03,0XE2,0X02,0XEA,0X02,0XE2,0X63,0XCA,0X81,0X79,0X90,0XE5,0X8D,0XE5,0XAC,0XED,
0XAB,0XED,0XAB,0XED,0XCB,0XE5,0XCB,0XE5,0XAB,0XE5,0XAE,0XDD,0X25,0X4A,0X28,0X3A,
0XF9,0X4D,0XB8,0X5D,0X76,0X75,0X08,0X2A,0XE7,0X51,0X85,0X61,0XA5,0X69,0X42,0X79,
0X04,0XC2,0X81,0XC9,0X81,0XD1,0XA1,0XC9,0X00,0X91,0X80,0X81,0X6D,0XE5,0XC0,0X61,
0XA1,0X61,0XE2,0X61,0XC1,0X59,0XE2,0X59,0XE3,0X59,0XE3,0X59,0XE4,0X59,0X06,0X4A,
0X55,0X7D,0X97,0X6D,0XF8,0X45,0XB8,0X55,0X98,0X6D,0X6B,0X32,0XE7,0X51,0X69,0X82,
0X06,0X8A,0X05,0XAA,0X81,0XC9,0X60,0XD9,0X60,0XD1,0XA1,0XC9,0X03,0XAA,0X22,0X92,
0X00,0X7A,0X8F,0XE5,0X91,0XDD,0X92,0XD5,0XB2,0XD5,0X92,0XCD,0XB4,0XD5,0X94,0XCD,
0XB5,0XCD,0X08,0X32,0X97,0X65,0XF9,0X4D,0X19,0X4E,0XF9,0X55,0X98,0X5D,0X8B,0X22,
0XE8,0X41,0X69,0X7A,0X68,0X9A,0XE4,0XA9,0XA1,0XC9,0XA0,0XE1,0X60,0XD1,0X80,0XC9,
0XC1,0XC1,0XE2,0XB1,0X20,0X79,0X91,0XF5,0X90,0XE5,0XD0,0XE5,0XB0,0XE5,0XB1,0XE5,
0X91,0XDD,0XD3,0XD5,0XB3,0XC5,0X68,0X3A,0XB7,0X65,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,
0XB8,0X55,0XB8,0X6D,0X35,0X8D,0XE7,0X41,0XC6,0X59,0X43,0X71,0XE3,0XB9,0XA2,0XC1,
0XC2,0XC1,0XC2,0XC1,0XC3,0XB9,0XE4,0XB1,0X68,0X9A,0X84,0X61,0XE3,0X51,0X03,0X4A,
0X03,0X52,0XE3,0X49,0X04,0X4A,0XE4,0X39,0X46,0X3A,0X54,0X7D,0XD7,0X5D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD8,0X55,0X97,0X65,0X77,0X75,0X35,0X7D,0XF3,0X9C,
0X85,0X69,0X43,0X81,0X44,0X79,0X44,0X79,0X64,0X79,0X85,0X69,0XA6,0X59,0XF4,0XA4,
0X54,0X85,0X95,0X75,0X95,0X75,0X95,0X75,0X96,0X75,0X96,0X75,0XB6,0X6D,0XB7,0X65,
0XD8,0X55,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XD9,0X5D,
0X98,0X65,0X36,0X85,0XC7,0X51,0X66,0X69,0X86,0X69,0X86,0X61,0X86,0X61,0XA7,0X59,
0XC8,0X49,0X36,0X95,0X97,0X6D,0XD8,0X55,0XD8,0X5D,0XD8,0X5D,0XD9,0X55,0XD9,0X55,
0XD9,0X55,0XF9,0X55,0XF9,0X4D,0XF9,0X4D,};
#line 7 "app\\game.c"
#line 1 ".\\app\\RES\\Pic\\flappybird_bird_m.c"
const unsigned char gImage_flappybird_bird_m[988] = {  
0XD9,0X4D,0XF9,0X4D,0X18,0X4E,0XF8,0X45,0XF8,0X4D,0XD8,0X55,0X77,0X6D,0X55,0X7D,
0X13,0X8D,0XE6,0X41,0XC5,0X51,0XC6,0X69,0X65,0X61,0XA6,0X61,0XE8,0X51,0XC7,0X41,
0XE8,0X41,0X09,0X3A,0X57,0X8D,0X77,0X75,0X98,0X55,0XF9,0X4D,0XDA,0X45,0XDA,0X4D,
0XFA,0X4D,0XD9,0X4D,0XB9,0X4D,0XFA,0X55,0XF9,0X4D,0XF8,0X4D,0X97,0X4D,0XD8,0X6D,
0X8A,0X2A,0X27,0X3A,0X26,0X4A,0X0D,0XA4,0X0D,0XBC,0X0D,0XC4,0XED,0XBB,0X0E,0XB4,
0XC6,0X51,0XBE,0XFF,0XDF,0XFF,0XFF,0XF7,0X2A,0X2A,0X8B,0X1A,0XF9,0X65,0XD9,0X4D,
0XFA,0X4D,0XFA,0X55,0XFA,0X4D,0XF9,0X4D,0XFA,0X4D,0XDA,0X4D,0XDA,0X4D,0XB9,0X55,
0X97,0X65,0X75,0X85,0XC6,0X49,0X83,0X69,0X61,0X79,0XC9,0XDB,0XA9,0XE3,0XA9,0XE3,
0XCB,0XCB,0X0D,0XBC,0X84,0X49,0XFF,0XFF,0XDF,0XF7,0XFF,0XF7,0X09,0X2A,0X6A,0X22,
0X76,0X6D,0XB7,0X5D,0XB8,0X55,0XB9,0X4D,0XD9,0X4D,0XFA,0X4D,0XD9,0X4D,0XD9,0X55,
0XB9,0X55,0X97,0X6D,0X55,0X85,0XC6,0X41,0XED,0XC3,0X89,0XDB,0X87,0XEB,0X41,0XD2,
0X41,0XDA,0X62,0XCA,0X40,0X81,0XA2,0X69,0XBC,0XFF,0XBD,0XFF,0XFE,0XF7,0XFF,0XF7,
0XBF,0XEF,0XFF,0XEF,0X6A,0X22,0X77,0X6D,0X77,0X65,0XD9,0X5D,0XD9,0X4D,0XFA,0X4D,
0XF9,0X4D,0XB7,0X55,0X76,0X6D,0X48,0X2A,0XE5,0X51,0X0B,0XC4,0X64,0XC2,0X42,0XDA,
0X41,0XE2,0X20,0XE2,0X00,0XE2,0X22,0XD2,0X20,0X89,0XC3,0X69,0XF9,0XEE,0XDD,0XF7,
0XDE,0XF7,0XDE,0XF7,0XE8,0X41,0X08,0X42,0XDF,0XE7,0X29,0X22,0X8B,0X12,0X98,0X65,
0XD9,0X55,0XF9,0X45,0XD8,0X4D,0X96,0X5D,0X54,0X8D,0X05,0X52,0X61,0X71,0X88,0XE3,
0X22,0XE2,0XE0,0XE9,0X00,0XEA,0XE0,0XF1,0XE0,0XF1,0X23,0XE2,0X40,0X89,0XA3,0X69,
0XF9,0XEE,0XFE,0XF7,0XFE,0XFF,0XFE,0XFF,0XE7,0X49,0XE8,0X49,0XDF,0XFF,0X2A,0X32,
0X8B,0X22,0X77,0X65,0XD8,0X55,0XF9,0X45,0XB9,0X5D,0X56,0X6D,0XE7,0X49,0XA8,0XA2,
0X64,0XC2,0X22,0XDA,0X02,0XEA,0XE1,0XF1,0XE1,0XF9,0XC0,0XF9,0XA0,0XF9,0XE2,0XE1,
0X00,0X89,0XA3,0X69,0XF9,0XEE,0XFD,0XFF,0XDD,0XF7,0XFE,0XFF,0XA6,0X41,0XC8,0X41,
0XDF,0XFF,0XE9,0X31,0X4A,0X22,0X97,0X6D,0XD8,0X55,0XF8,0X45,0X78,0X65,0X36,0X7D,
0XC6,0X51,0XC8,0XAA,0X85,0XC2,0X44,0XCA,0X44,0XDA,0X24,0XDA,0X03,0XE2,0X03,0XEA,
0XE2,0XF9,0X03,0XEA,0X85,0XC2,0X08,0XAB,0XA4,0X59,0XFA,0XE6,0X5B,0XE7,0XDE,0XF7,
0XBE,0XFF,0XBF,0XFF,0XBF,0XFF,0XE9,0X39,0X8A,0X32,0X76,0X75,0XD7,0X55,0XF8,0X4D,
0XB9,0X7D,0X36,0X85,0XC7,0X41,0XA5,0X59,0XA4,0X61,0X42,0X61,0X43,0X71,0X23,0X79,
0X22,0X81,0X66,0XC2,0X04,0XE2,0X03,0XE2,0X64,0XDA,0X86,0XBA,0X43,0X69,0XDA,0XFE,
0X3B,0XEF,0XFE,0XFF,0XFF,0XFF,0XDF,0XFF,0XBF,0XFF,0X09,0X3A,0X29,0X2A,0X76,0X7D,
0X97,0X65,0XD8,0X55,0X4B,0X1A,0X09,0X22,0XFF,0XFF,0XDE,0XFF,0XBE,0XFF,0XBE,0XFF,
0XBF,0XFF,0X5D,0XFF,0X1B,0XFF,0XE2,0X70,0XE0,0XA0,0X03,0XDA,0XE2,0XD1,0XA6,0XD2,
0X43,0X79,0XB9,0XFE,0XD9,0XEE,0XDC,0XFF,0XBC,0XFF,0XBD,0XFF,0X9D,0XFF,0XE7,0X41,
0X07,0X32,0X75,0X8D,0X76,0X6D,0X97,0X65,0X6B,0X32,0X49,0X32,0XFF,0XF7,0XFE,0XFF,
0XFF,0XFF,0XDF,0XFF,0XBF,0XFF,0XBF,0XFF,0X9D,0XFF,0XE4,0X79,0XE0,0X90,0X44,0XDA,
0X64,0XEA,0XE3,0XC9,0XC8,0XB2,0X63,0X69,0XC3,0X61,0XE2,0X51,0X02,0X5A,0XE2,0X51,
0XE3,0X51,0X03,0X52,0X24,0X4A,0X25,0X3A,0X34,0X85,0X55,0X7D,0X4A,0X2A,0X49,0X2A,
0X5C,0XD7,0XFF,0XF7,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XBE,0XFF,0XD9,0XFE,0X82,0X71,
0X20,0X91,0X24,0XD2,0X23,0XDA,0X64,0XD2,0X61,0X89,0X50,0XED,0XB0,0XED,0XCF,0XE5,
0XCF,0XDD,0XCF,0XDD,0XD0,0XDD,0XD0,0XDD,0XF1,0XD5,0XF3,0XC5,0X47,0X32,0X69,0X2A,
0X77,0X6D,0X56,0X7D,0X08,0X22,0X07,0X3A,0XC7,0X41,0XA7,0X51,0XA7,0X51,0XA6,0X59,
0X43,0X61,0X86,0XA2,0XC3,0XA9,0X23,0XC2,0X65,0XD2,0X84,0XC2,0X60,0X79,0X6F,0XED,
0X4D,0XED,0X6D,0XED,0X8D,0XE5,0X8D,0XE5,0X8D,0XE5,0X8E,0XDD,0XCF,0XDD,0XD1,0XCD,
0X47,0X3A,0X49,0X2A,0X97,0X5D,0X76,0X65,0X8A,0X2A,0X08,0X32,0X86,0X41,0XA6,0X59,
0X85,0X61,0X64,0X69,0X22,0X79,0X24,0XAA,0XC2,0XB1,0X02,0XC2,0X00,0X99,0X00,0X89,
0X4F,0XFD,0XA0,0X71,0XC0,0X71,0X00,0X7A,0XC0,0X69,0XC0,0X71,0XE0,0X71,0XE0,0X69,
0XC0,0X59,0X23,0X4A,0X33,0X8D,0X55,0X85,0XB9,0X55,0XB8,0X5D,0XF5,0X5C,0X09,0X22,
0XC7,0X49,0X89,0X82,0X47,0X8A,0X26,0X9A,0XE4,0XB1,0X81,0XB1,0XA0,0XB9,0XE1,0XC1,
0XC1,0XB9,0X43,0XB2,0X60,0X79,0X6F,0XED,0X8E,0XED,0X6D,0XE5,0XAE,0XE5,0X8E,0XE5,
0X6F,0XDD,0X90,0XDD,0XD1,0XCD,0X04,0X3A,0X74,0X7D,0X97,0X6D,0XD9,0X4D,0XD9,0X55,
0XB9,0X65,0X57,0X75,0X35,0X9D,0X08,0X5A,0XA5,0X69,0X63,0X79,0X25,0XB2,0XA2,0XB1,
0XE2,0XC1,0XA1,0XB9,0XE2,0XB1,0XE3,0XA1,0X61,0X71,0XB1,0XE5,0X8F,0XD5,0XD0,0XD5,
0XD0,0XD5,0XD1,0XD5,0XB1,0XD5,0XF3,0XCD,0XD2,0XBD,0X87,0X32,0X95,0X6D,0XB7,0X65,
0X1A,0X46,0XD9,0X45,0XD9,0X4D,0XB8,0X5D,0X56,0X65,0X35,0X7D,0X29,0X42,0XA6,0X59,
0X07,0X92,0X06,0XAA,0XC4,0XA9,0XE5,0XA9,0X05,0XAA,0X45,0X9A,0X66,0X8A,0XE4,0X59,
0X24,0X4A,0X64,0X42,0X44,0X3A,0X45,0X3A,0X46,0X3A,0X47,0X3A,0X67,0X2A,0X94,0X7D,
0XB6,0X65,0XD7,0X5D,0XD9,0X3D,0XFA,0X45,0XFA,0X45,0XB9,0X4D,0XFA,0X5D,0X97,0X65,
0X56,0X85,0XD4,0X9C,0X86,0X61,0X44,0X79,0X65,0X81,0X44,0X81,0X64,0X79,0X84,0X69,
0XC5,0X59,0XF2,0XA4,0X32,0X8D,0X73,0X85,0X73,0X85,0X74,0X85,0X54,0X7D,0X95,0X7D,
0X76,0X75,0XB7,0X6D,0XD7,0X5D,0XD7,0X55,0XFA,0X45,0XD9,0X45,0XD9,0X45,0XB9,0X45,
0XD9,0X4D,0X98,0X55,0X98,0X75,0X15,0X8D,0XA7,0X49,0X86,0X61,0X66,0X61,0X86,0X61,
0X86,0X61,0XC6,0X59,0XC7,0X41,0X76,0X95,0X76,0X75,0X96,0X6D,0X96,0X65,0XB7,0X6D,
0XB8,0X6D,0X77,0X5D,0XB8,0X65,0XB8,0X55,0XD8,0X4D,0XF8,0X4D,};
#line 8 "app\\game.c"
#line 1 ".\\app\\RES\\Pic\\flappybird_bird_d.c"
const unsigned char gImage_flappybird_bird_d[936] = {  
0XF8,0X45,0XF8,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD8,0X55,0X96,0X65,0X55,0X7D,
0XF3,0X94,0X26,0X52,0XA4,0X61,0X83,0X71,0X63,0X71,0XA5,0X69,0XC8,0X51,0XE9,0X49,
0XE9,0X41,0X0A,0X42,0X37,0X8D,0X77,0X6D,0XD8,0X4D,0X39,0X46,0XF8,0X45,0X19,0X46,
0XF9,0X45,0XF9,0X45,0XF8,0X45,0X19,0X4E,0XF9,0X4D,0XD9,0X4D,0X98,0X4D,0XD8,0X65,
0X8A,0X22,0X27,0X3A,0X05,0X52,0X0C,0XAC,0XCB,0XC3,0XCB,0XCB,0XCB,0XCB,0XED,0XB3,
0XC6,0X49,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0X09,0X2A,0X6B,0X1A,0XF9,0X65,0XF9,0X4D,
0XF9,0X4D,0X19,0X4E,0XF9,0X4D,0XF9,0X4D,0X19,0X4E,0XF9,0X45,0XFA,0X4D,0XDA,0X4D,
0XB9,0X5D,0X77,0X7D,0XE5,0X49,0XA2,0X69,0X80,0X79,0XC8,0XE3,0X87,0XF3,0X87,0XF3,
0XA9,0XD3,0X0C,0XBC,0XA5,0X49,0XFF,0XFF,0XDE,0XF7,0XFF,0XFF,0XE8,0X39,0X4A,0X2A,
0X76,0X6D,0XB8,0X55,0XB9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF8,0X45,0XF9,0X4D,
0XD9,0X55,0X98,0X65,0X56,0X7D,0XE6,0X41,0X0C,0XC4,0X88,0XE3,0X86,0XF3,0X20,0XDA,
0X20,0XF2,0X21,0XE2,0X20,0X89,0XA2,0X69,0XFD,0XFF,0XDE,0XEF,0XFE,0XF7,0XDE,0XFF,
0X9E,0XFF,0XFF,0XF7,0X6A,0X22,0X97,0X6D,0X98,0X5D,0XD9,0X55,0XD9,0X4D,0XF9,0X4D,
0X19,0X46,0XB7,0X4D,0X77,0X65,0X49,0X22,0XE5,0X51,0XEB,0XCB,0X42,0XCA,0X21,0XE2,
0X21,0XEA,0XE0,0XF1,0XC0,0XF9,0X02,0XE2,0X41,0X81,0XC4,0X59,0X1A,0XE7,0XFE,0XF7,
0XDE,0XF7,0XBE,0XF7,0XC7,0X49,0XE8,0X49,0XBF,0XEF,0X29,0X22,0X8B,0X1A,0X98,0X65,
0XD9,0X4D,0X19,0X46,0XF9,0X45,0XB7,0X5D,0X55,0X8D,0XE6,0X51,0X40,0X79,0X66,0XF3,
0X00,0XEA,0XC0,0XF1,0XC0,0XF9,0XC0,0XF9,0XC0,0XF9,0X02,0XEA,0X42,0X89,0XC4,0X59,
0XF9,0XDE,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0XC8,0X49,0XC8,0X49,0XDF,0XFF,0X29,0X3A,
0X6B,0X2A,0X77,0X6D,0XD9,0X55,0X19,0X46,0XFA,0X4D,0X77,0X65,0XC6,0X51,0X87,0XB2,
0X23,0XD2,0X00,0XF2,0XE0,0XF9,0XC0,0XF9,0XC0,0XF9,0XC0,0XF9,0X80,0XF9,0XE2,0XE9,
0XE0,0X90,0X84,0X71,0XF9,0XE6,0XFE,0XF7,0XDD,0XEF,0XFF,0XFF,0XA6,0X41,0XC8,0X49,
0XDF,0XFF,0XE9,0X31,0X2B,0X22,0X98,0X75,0XD8,0X55,0XF9,0X4D,0XDA,0X45,0X77,0X6D,
0X85,0X61,0X86,0XC2,0X23,0XE2,0XE1,0XE9,0XE2,0XF1,0XC1,0XF1,0XC0,0XF1,0XE0,0XF9,
0XC1,0XF9,0XE2,0XF1,0X45,0XD2,0XC8,0XB2,0XA5,0X51,0XFA,0XDE,0X5B,0XDF,0XFE,0XF7,
0XBE,0XFF,0XBF,0XFF,0XBF,0XFF,0XE9,0X39,0X4B,0X32,0X57,0X75,0XB8,0X5D,0XF8,0X4D,
0XD8,0X4D,0X97,0X75,0XC6,0X51,0X88,0XAA,0XA7,0XCA,0X45,0XCA,0X26,0XD2,0X25,0XDA,
0X03,0XDA,0X22,0XF2,0XE1,0XF9,0XC1,0XF1,0X02,0XE2,0X86,0XD2,0X43,0X69,0XDA,0XF6,
0X1A,0XE7,0XDD,0XFF,0X9D,0XFF,0XBE,0XFF,0X9F,0XFF,0X09,0X42,0X09,0X3A,0X56,0X85,
0XB7,0X65,0XD8,0X55,0XB8,0X65,0X35,0X75,0XE7,0X49,0XA5,0X69,0X63,0X71,0X63,0X79,
0X22,0X79,0X42,0X89,0X21,0X91,0X44,0XCA,0X02,0XE2,0X02,0XF2,0X02,0XF2,0X23,0XD2,
0XC9,0XB2,0XA5,0X61,0XE3,0X51,0X03,0X4A,0X04,0X52,0XE4,0X51,0XE5,0X51,0XC4,0X49,
0X25,0X52,0X26,0X3A,0X75,0X7D,0X97,0X6D,0X2A,0X2A,0X4A,0X3A,0X1C,0XEF,0XBE,0XFF,
0XBD,0XFF,0XDD,0XFF,0X9C,0XFF,0X9C,0XFF,0XD8,0XFE,0X62,0X71,0X00,0X91,0X03,0XD2,
0X22,0XEA,0X23,0XD2,0X82,0X81,0X51,0XE5,0XCE,0XED,0XED,0XED,0XCC,0XE5,0XCD,0XE5,
0XCD,0XE5,0XED,0XE5,0XCD,0XDD,0XF0,0XD5,0X25,0X42,0X28,0X32,0XC9,0X41,0XE8,0X41,
0X1C,0XE7,0XBE,0XF7,0XFF,0XFF,0XDE,0XFF,0XBD,0XFF,0XBC,0XFF,0XD8,0XFE,0X62,0X69,
0X20,0X91,0X23,0XCA,0X43,0XDA,0X85,0XD2,0X61,0X79,0X90,0XED,0X6B,0XED,0X8A,0XED,
0XAA,0XED,0XAA,0XED,0XAB,0XED,0X8B,0XED,0XAB,0XE5,0XAE,0XD5,0X25,0X42,0X28,0X3A,
0XE9,0X51,0XE9,0X49,0X9F,0XF7,0XFF,0XFF,0XDE,0XF7,0XBD,0XFF,0XDE,0XFF,0X9D,0XFF,
0X42,0X71,0X24,0XB2,0XC1,0XC1,0XE1,0XC9,0X00,0XA1,0X20,0X81,0X6F,0XED,0XA0,0X69,
0XA0,0X71,0XC0,0X79,0XC0,0X69,0XE0,0X69,0XC0,0X71,0XA0,0X69,0XE1,0X69,0X03,0X52,
0X53,0X85,0X96,0X6D,0X0A,0X42,0X09,0X42,0XBF,0XF7,0XBF,0XF7,0XDE,0XF7,0X1B,0XEF,
0XE7,0X51,0X65,0X69,0X05,0XB2,0X82,0XC1,0XA0,0XC9,0XA1,0XC9,0XC2,0XB9,0X44,0XAA,
0XA0,0X79,0X8E,0XE5,0X8E,0XED,0X6D,0XED,0X8D,0XE5,0XAE,0XE5,0X6E,0XE5,0X90,0XE5,
0X91,0XD5,0X05,0X3A,0X96,0X75,0XD8,0X5D,0X6A,0X2A,0X49,0X2A,0XDF,0XF7,0XDF,0XFF,
0XBE,0XFF,0XFB,0XE6,0XC7,0X59,0X65,0X71,0XE4,0XC1,0X81,0XD1,0X80,0XD1,0X80,0XC9,
0XA1,0XC9,0XE2,0XB1,0X20,0X79,0X90,0XED,0X8E,0XDD,0XAE,0XDD,0XAF,0XDD,0XAF,0XDD,
0X8F,0XE5,0XD1,0XDD,0X91,0XC5,0X66,0X3A,0X96,0X6D,0XD8,0X55,0X76,0X6D,0X76,0X75,
0X6A,0X2A,0X29,0X2A,0X49,0X3A,0X29,0X3A,0X08,0X42,0X86,0X61,0XE5,0XA9,0XA3,0XC1,
0XC3,0XB9,0XC3,0XB9,0XC3,0XB9,0X04,0XAA,0X66,0X92,0XE4,0X61,0X03,0X4A,0X23,0X4A,
0X23,0X4A,0X23,0X4A,0X03,0X4A,0X25,0X4A,0X46,0X3A,0X74,0X7D,0XB7,0X5D,0XD9,0X55,
0XD8,0X55,0XB8,0X5D,0X98,0X65,0X97,0X6D,0X97,0X6D,0X97,0X75,0X76,0X7D,0X15,0X95,
0XA6,0X61,0X64,0X79,0X65,0X71,0X65,0X71,0X64,0X71,0X85,0X69,0XC6,0X51,0X13,0X9D,
0X74,0X7D,0X95,0X75,0X95,0X75,0X95,0X75,0X95,0X75,0X96,0X75,0XB6,0X6D,0XD7,0X65,
0XD8,0X55,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X55,0XD9,0X5D,0XD9,0X5D,0XB8,0X5D,
0X98,0X65,0X56,0X7D,0XE8,0X49,0XA6,0X59,0XA7,0X59,0XA7,0X59,0XA7,0X59,0XC7,0X51,
0XE8,0X41,0X56,0X8D,0X97,0X6D,0XD8,0X5D,0XD8,0X5D,0XD8,0X5D,0XD9,0X5D,0XD9,0X55,
0XD9,0X55,0XF9,0X55,0XF9,0X4D,0XF9,0X4D,};
#line 9 "app\\game.c"
#line 1 ".\\app\\RES\\Pic\\flappybird_tag.c"
const unsigned char gImage_flappybird_tag[15960] = {  
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF8,0X45,0X19,0X4E,0XD9,0X45,0XF9,0X4D,0XD9,0X4D,0XD9,0X55,0XDA,0X55,0XD9,0X55,
0XF9,0X4D,0XD8,0X55,0X76,0X75,0X6A,0X2A,0X28,0X3A,0XE8,0X39,0XEA,0X49,0XA9,0X49,
0XCA,0X49,0XA9,0X49,0XC9,0X49,0XE9,0X49,0XE9,0X41,0X4A,0X2A,0X97,0X5D,0XF9,0X4D,
0XF9,0X45,0XF9,0X45,0X1A,0X46,0XF9,0X45,0X19,0X4E,0XD8,0X45,0XF9,0X45,0XF9,0X45,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X19,0X4E,
0XD8,0X45,0X1A,0X4E,0X1A,0X4E,0XD9,0X4D,0XB8,0X55,0XB9,0X5D,0X98,0X5D,0XB8,0X5D,
0X77,0X65,0X35,0X7D,0X29,0X2A,0X28,0X3A,0XE8,0X41,0XC9,0X41,0XE9,0X49,0XC9,0X49,
0XA9,0X41,0XA8,0X41,0XC8,0X41,0XE9,0X39,0X4A,0X2A,0XD8,0X75,0XB8,0X55,0XD8,0X4D,
0XD9,0X4D,0X1A,0X4E,0XF9,0X45,0XD8,0X45,0XF9,0X4D,0X1A,0X4E,0XF9,0X45,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,
0XD9,0X45,0XD9,0X4D,0XD8,0X55,0XD8,0X65,0X76,0X6D,0X8B,0X22,0X0A,0X22,0X4B,0X3A,
0X09,0X3A,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0X9F,0XFF,0XDF,0XFF,0XC8,0X39,0X09,0X4A,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XEF,0X6B,0X2A,0X77,0X75,0XB8,0X65,0XD8,0X55,
0XB8,0X4D,0XF9,0X4D,0XF9,0X55,0XD8,0X4D,0XF9,0X4D,0X19,0X46,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XFA,0X45,0X1A,0X46,0XFA,0X45,
0XD9,0X55,0XB8,0X65,0X56,0X6D,0X4A,0X1A,0XFF,0XEF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,
0XBB,0XDE,0XDB,0XDE,0XBA,0XDE,0XFC,0XE6,0XC8,0X41,0XDF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XF7,0X4A,0X32,0X4A,0X1A,0X97,0X6D,0XD9,0X5D,
0XD8,0X4D,0XF9,0X55,0XD8,0X4D,0XD8,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XD8,0X45,0XD8,0X5D,
0X8B,0X1A,0X49,0X2A,0XFF,0XEF,0X1B,0XD7,0XBA,0XD6,0X9A,0XD6,0X9A,0XDE,0X9A,0XDE,
0X99,0XD6,0XDB,0XDE,0X9B,0XDE,0XE8,0X49,0XFB,0XDE,0XB9,0XD6,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XE8,0X49,0XDF,0XFF,0XFF,0XFF,0X2A,0X22,0X97,0X65,0XD8,0X55,
0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD8,0X4D,0XF9,0X5D,0X97,0X6D,0X4A,0X32,
0XE8,0X39,0XFF,0XFF,0XB9,0XCE,0XDA,0XD6,0XBA,0XD6,0XBA,0XDE,0XBA,0XDE,0XBA,0XD6,
0XFB,0XE6,0X9B,0XDE,0XC8,0X41,0X99,0XD6,0XB9,0XD6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XE8,0X49,0XBF,0XFF,0X9F,0XFF,0X4B,0X3A,0X77,0X65,0XD8,0X4D,0XF9,0X4D,
0XF9,0X45,0XF9,0X4D,0XD9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0X19,0X4E,0XD9,0X55,0X57,0X75,0X29,0X2A,0X9B,0XCE,0XDB,0XDE,
0XDA,0XD6,0XDA,0XCE,0XBA,0XD6,0XBA,0XD6,0XBA,0XD6,0XDA,0XD6,0XB9,0XD6,0X9A,0XD6,
0XBB,0XE6,0XE8,0X49,0XDA,0XDE,0XDA,0XDE,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0X08,0X42,0XFF,0XFF,0XBF,0XFF,0X09,0X3A,0X97,0X65,0XF8,0X4D,0XF9,0X4D,0XD9,0X45,
0X1A,0X4E,0XD9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X45,0XB8,0X5D,0X36,0X7D,0X2A,0X3A,0XBB,0XDE,0X9A,0XDE,0XBA,0XD6,
0XDA,0XD6,0XBA,0XD6,0XBB,0XDE,0XBA,0XD6,0XDA,0XD6,0XBA,0XD6,0XBA,0XD6,0XDB,0XE6,
0XE8,0X49,0X7A,0XDE,0X9A,0XDE,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XE8,0X39,
0XDF,0XFF,0XBF,0XFF,0X09,0X3A,0X76,0X6D,0XB8,0X55,0XF9,0X55,0XF9,0X4D,0XD9,0X45,
0XFA,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XD9,0X4D,0XB9,0X5D,0X57,0X7D,0X09,0X32,0XDB,0XDE,0XBB,0XE6,0X9A,0XD6,0XBB,0XDE,
0X9B,0XDE,0X9A,0XDE,0XBA,0XDE,0XBA,0XDE,0XB9,0XD6,0XB9,0XD6,0XDA,0XD6,0XDA,0XDE,
0XC7,0X41,0XE9,0X49,0X9B,0XDE,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0X9F,0XFF,0X09,0X4A,0X15,0X7D,0X97,0X6D,0XB8,0X5D,0XD8,0X4D,0XF9,0X45,0X1A,0X4E,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XB8,0X5D,0X77,0X7D,0X2A,0X32,0X09,0X4A,0XC8,0X49,0XA7,0X41,0XC8,0X41,0XE8,0X49,
0XC8,0X49,0XE7,0X41,0XDB,0XDE,0XBA,0XD6,0XFA,0XDE,0XDA,0XCE,0XDA,0XD6,0XBB,0XDE,
0X7A,0XDE,0XC9,0X49,0X09,0X4A,0XC8,0X41,0XE8,0X49,0XE8,0X41,0X09,0X4A,0XE8,0X51,
0XE9,0X49,0X2A,0X3A,0X6B,0X22,0XEC,0X0A,0XD8,0X55,0XD9,0X4D,0XD9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X55,0X97,0X5D,
0X57,0X7D,0X0A,0X32,0X09,0X42,0XE8,0X49,0X09,0X4A,0XE8,0X49,0XC8,0X49,0XE8,0X49,
0XE8,0X41,0X9B,0XD6,0XBB,0XDE,0XBA,0XD6,0XDA,0XCE,0XDA,0XD6,0XDB,0XDE,0X9B,0XDE,
0XE9,0X49,0XC8,0X49,0XE8,0X49,0XE8,0X51,0XC8,0X49,0XC8,0X49,0XC8,0X51,0XA8,0X51,
0XC9,0X41,0X2A,0X3A,0X8B,0X1A,0XB7,0X5D,0XD8,0X4D,0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XB8,0X55,0X97,0X6D,0XF5,0X7C,
0XE9,0X39,0X09,0X4A,0XA7,0X39,0XE8,0X41,0XC7,0X41,0XC7,0X49,0XC8,0X49,0XE8,0X41,
0XDC,0XDE,0XBC,0XDE,0X9B,0XD6,0XBA,0XD6,0XFB,0XDE,0X9A,0XD6,0XBC,0XE6,0XC8,0X41,
0XC8,0X41,0XA7,0X41,0XA7,0X41,0XA8,0X41,0XA8,0X41,0XA7,0X49,0XC8,0X51,0XA8,0X49,
0X0A,0X3A,0X6A,0X22,0XD7,0X6D,0XD8,0X55,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD8,0X55,0X77,0X65,0X6B,0X22,0X09,0X3A,0X9B,0XDE,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XBB,0XDE,0X09,0X4A,
0XC8,0X41,0X9C,0XE6,0XFC,0XE6,0X9A,0XD6,0XE8,0X41,0XE8,0X49,0XBB,0XDE,0XBB,0XDE,
0XDB,0XE6,0X9B,0XE6,0XDC,0XE6,0XBB,0XDE,0XBA,0XDE,0XBA,0XDE,0X7B,0XDE,0XDC,0XDE,
0X3D,0XBF,0X77,0X6D,0XD8,0X55,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XD8,0X55,0X56,0X6D,0X29,0X2A,0XE8,0X39,0XDF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X08,0X4A,0XBB,0XE6,0X9C,0XE6,
0X7B,0XDE,0XBB,0XDE,0X08,0X42,0X9A,0XDE,0X7A,0XD6,0XE8,0X41,0XE8,0X41,0XA7,0X41,
0XA7,0X41,0XE9,0X41,0X08,0X42,0XE8,0X41,0XC7,0X41,0XE9,0X49,0X0A,0X42,0X6B,0X2A,
0XB8,0X75,0XD8,0X55,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XD8,0X55,0X77,0X75,0X2A,0X2A,0XE8,0X31,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X28,0X4A,0X9A,0XDE,0XBB,0XDE,0XBA,0XDE,
0X9A,0XDE,0XE8,0X49,0XBA,0XDE,0X9A,0XDE,0XE8,0X49,0XC9,0X49,0XC9,0X51,0XC9,0X51,
0XA9,0X49,0XA8,0X49,0XC8,0X49,0XA8,0X49,0XEA,0X51,0XC9,0X39,0X4A,0X22,0X97,0X65,
0XF9,0X55,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XB8,0X55,0X57,0X75,0X4A,0X2A,0XE8,0X31,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XC6,0X39,0XBA,0XDE,0XDA,0XDE,0XBA,0XD6,0XBA,0XDE,
0XE7,0X41,0XBA,0XDE,0XDB,0XDE,0XC8,0X49,0XA8,0X49,0XA8,0X49,0XC8,0X49,0XA8,0X49,
0XC9,0X49,0XC8,0X49,0XC8,0X49,0XC9,0X49,0X0A,0X3A,0X6B,0X1A,0X97,0X5D,0XD9,0X55,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,
0XF9,0X45,0XF9,0X4D,0XFA,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD8,0X4D,0XB8,0X55,
0X77,0X6D,0X6B,0X22,0XE9,0X31,0XBF,0XFF,0XFF,0XFF,0XDF,0XFF,0X9B,0XDE,0XC9,0X41,
0XA8,0X41,0XE8,0X49,0XBA,0XDE,0XBA,0XDE,0XDA,0XDE,0XBA,0XDE,0XBA,0XD6,0XDB,0XDE,
0XE8,0X39,0XE8,0X41,0X9A,0XDE,0X9A,0XDE,0X9A,0XDE,0XBB,0XE6,0X7A,0XD6,0XBB,0XDE,
0X9B,0XE6,0X7B,0XDE,0X0A,0X3A,0X2B,0X2A,0X8C,0X12,0XB8,0X5D,0XB9,0X4D,0X1A,0X4E,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,
0XF9,0X4D,0XB9,0X4D,0X1A,0X56,0XB8,0X45,0XF8,0X4D,0XED,0X02,0XB8,0X5D,0X98,0X65,
0X77,0X6D,0X36,0X8D,0X4A,0X42,0X2A,0X42,0XE9,0X39,0X0A,0X42,0X0A,0X42,0XC9,0X39,
0XE9,0X41,0X9A,0XD6,0X99,0XD6,0XBA,0XDE,0X99,0XDE,0X9A,0XD6,0XBB,0XD6,0XBB,0XDE,
0X7A,0XD6,0X29,0X42,0X29,0X42,0X29,0X42,0X29,0X42,0X69,0X42,0X09,0X3A,0X29,0X42,
0X4A,0X3A,0X36,0X85,0X56,0X75,0X98,0X65,0XD9,0X5D,0XF9,0X4D,0XD9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,
0XF9,0X4D,0XB8,0X45,0XF9,0X55,0XD9,0X55,0XCD,0X0A,0X98,0X65,0XB9,0X55,0XB8,0X5D,
0X57,0X6D,0X4A,0X1A,0X4A,0X2A,0X2A,0X2A,0X4B,0X32,0X2A,0X32,0X4A,0X32,0X09,0X32,
0XDB,0XD6,0XDB,0XDE,0XBA,0XDE,0XDA,0XDE,0XDA,0XDE,0XBB,0XD6,0XDC,0XDE,0XBC,0XD6,
0X2A,0X3A,0X4A,0X32,0X2A,0X2A,0X2A,0X2A,0X2A,0X2A,0X2A,0X2A,0X2B,0X2A,0X6B,0X1A,
0X77,0X65,0XF8,0X5D,0XD8,0X4D,0XF8,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X55,0XB8,0X55,0XAD,0X0A,0X98,0X65,0XB9,0X55,0XB8,0X55,0XD9,0X6D,
0X77,0X6D,0X76,0X6D,0X77,0X75,0X56,0X75,0X57,0X75,0X77,0X7D,0X35,0X7D,0X4A,0X32,
0X08,0X3A,0X29,0X42,0X08,0X42,0X29,0X42,0X09,0X3A,0X0A,0X3A,0X6B,0X3A,0X15,0X7D,
0X56,0X75,0X77,0X75,0X98,0X75,0X37,0X75,0X78,0X7D,0X58,0X75,0XB9,0X6D,0XD8,0X55,
0XF8,0X45,0XF8,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0X19,0X4E,0XD8,0X45,0XD9,0X45,0XD8,0X45,
0XD8,0X4D,0XD9,0X55,0XCD,0X0A,0X98,0X65,0XB9,0X5D,0XB9,0X5D,0X77,0X5D,0XB7,0X65,
0XD7,0X65,0XB7,0X5D,0XD7,0X55,0XD8,0X5D,0XB7,0X5D,0X97,0X6D,0X4B,0X22,0X6B,0X32,
0X29,0X2A,0X2A,0X32,0X2A,0X32,0X2A,0X32,0X2B,0X2A,0X6C,0X22,0X97,0X65,0XD8,0X5D,
0XD8,0X5D,0XB8,0X55,0XB9,0X5D,0X99,0X55,0X99,0X5D,0XB9,0X5D,0XB9,0X4D,0XD9,0X45,
0X1A,0X4E,0XF8,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X1A,0X4E,0X19,0X4E,0X19,0X4E,0XF9,0X4D,
0XD8,0X55,0X98,0X5D,0XAC,0X02,0XAD,0X02,0XB8,0X65,0XB8,0X65,0XB7,0X65,0XB7,0X5D,
0XB7,0X55,0X19,0X4E,0XD8,0X45,0XD8,0X4D,0XD8,0X5D,0X57,0X6D,0X57,0X75,0X77,0X75,
0X56,0X75,0X37,0X7D,0X37,0X75,0X58,0X75,0X77,0X65,0XD8,0X55,0XF8,0X45,0XF9,0X45,
0XF9,0X45,0XFA,0X4D,0XD9,0X4D,0XFA,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X45,
0XF9,0X4D,0XF9,0X4D,0X19,0X4E,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XD9,0X45,0XF9,0X4D,0XF9,0X45,0XF8,0X45,0XF8,0X45,0XD8,0X4D,
0XB8,0X5D,0X98,0X65,0X97,0X65,0XAC,0X0A,0XAC,0X12,0X6B,0X12,0XAC,0X1A,0X8C,0X0A,
0XD8,0X55,0X19,0X4E,0XFA,0X4D,0XD9,0X4D,0XB8,0X55,0XB8,0X5D,0X98,0X55,0XD9,0X5D,
0XB9,0X5D,0XD9,0X5D,0XB8,0X55,0XD9,0X55,0XF9,0X4D,0X19,0X4E,0XF9,0X45,0X1A,0X46,
0XF9,0X45,0XF9,0X45,0XF9,0X45,0X19,0X46,0X19,0X46,0X19,0X46,0XF9,0X4D,0XD9,0X4D,
0XF9,0X4D,0XD9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0X19,0X4E,0XF9,0X45,0X19,0X46,0X19,0X4E,0XF9,0X55,0XAD,0X02,
0X98,0X65,0X97,0X6D,0X77,0X6D,0X77,0X6D,0X77,0X6D,0X77,0X6D,0X98,0X65,0XD9,0X4D,
0XF9,0X45,0XF9,0X45,0XFA,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X45,0XF9,0X4D,
0XD9,0X45,0X19,0X4E,0XD9,0X45,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0X19,0X46,0X19,0X46,
0X19,0X46,0X19,0X46,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XD9,0X45,0X1A,0X4E,0XF9,0X45,0XD8,0X45,0XF8,0X4D,0XB8,0X55,0XAD,0X0A,0X78,0X75,
0X58,0X75,0X78,0X75,0XB7,0X65,0XB7,0X5D,0XF8,0X5D,0XB8,0X4D,0XD9,0X4D,0XF9,0X4D,
0XDA,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0X19,0X46,0XF9,0X45,0XF9,0X45,
0XF8,0X4D,0XF8,0X4D,0XF8,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X45,0XF9,0X45,0X39,0X4E,0XF9,0X4D,0XB8,0X55,0X99,0X6D,0X8D,0X1A,0X8D,0X1A,
0X8C,0X12,0XAC,0X02,0XD8,0X4D,0XF8,0X4D,0X19,0X4E,0XF9,0X4D,0XFA,0X4D,0XD9,0X4D,
0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF8,0X4D,
0XF8,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0X19,0X4E,0XD8,0X45,0X19,0X4E,0XD8,0X4D,0X98,0X5D,0XAD,0X0A,0X8D,0X12,0X8C,0X0A,
0XCD,0X02,0XD9,0X55,0XF8,0X45,0XF9,0X45,0XF9,0X4D,0XD9,0X45,0XF9,0X45,0X19,0X46,
0X19,0X46,0X19,0X46,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XD9,0X4D,0XD9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X19,0X4E,0XD8,0X45,
0XF9,0X45,0X19,0X4E,0XF9,0X4D,0XD9,0X55,0XB8,0X5D,0XB9,0X65,0XB8,0X5D,0XB8,0X55,
0XF9,0X55,0X19,0X46,0XF9,0X45,0X1A,0X4E,0X19,0X4E,0X19,0X46,0X19,0X46,0X19,0X46,
0X19,0X46,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,
0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,
0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XD9,0X45,0XD9,0X4D,0XFA,0X4D,0X19,0X4E,
0X19,0X46,0XF9,0X45,0XF8,0X45,0XD8,0X4D,0XD8,0X4D,0XF8,0X55,0XF9,0X55,0XB8,0X45,
0X19,0X46,0X19,0X46,0XB8,0X45,0XD9,0X4D,0XF9,0X45,0X19,0X46,0X19,0X46,0X19,0X46,
0X19,0X46,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X45,0XF9,0X45,0XF9,0X45,0XFA,0X4D,0XD9,0X4D,0X1A,0X56,0XB9,0X45,0XF9,0X45,
0X19,0X46,0X19,0X46,0XF9,0X4D,0X18,0X4E,0XF8,0X4D,0XD8,0X45,0XF9,0X4D,0X19,0X46,
0X1A,0X4E,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0X19,0X46,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X45,0X19,0X46,0XF9,0X45,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X45,0X19,0X46,0XF9,0X45,0XD9,0X45,0XD9,0X4D,0X1A,0X56,0XF9,0X4D,0XD9,0X45,
0X19,0X4E,0XD8,0X4D,0XD8,0X4D,0XF8,0X4D,0X19,0X46,0X3A,0X46,0X1A,0X46,0XF9,0X45,
0XF9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,
0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,
0XD9,0X4D,0XF9,0X4D,0X19,0X46,0X19,0X46,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,
0XF9,0X45,0XF9,0X45,0XFA,0X4D,0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XD9,0X4D,0XF9,0X4D,0X19,0X46,0XF9,0X3D,0XF9,0X45,0XFA,0X4D,0XD9,0X4D,
0XFA,0X4D,0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X55,0XD9,0X55,0XD9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0X19,0X46,0X19,0X46,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XFA,0X4D,0XF9,0X45,0XF9,0X45,0X19,0X46,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X55,0XB8,0X55,0XB8,0X5D,0XB8,0X55,
0XD9,0X4D,0XF9,0X45,0XF9,0X45,0X1A,0X4E,0XD9,0X45,0XF9,0X4D,0XF9,0X4D,0XD8,0X45,
0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X45,
0XF9,0X4D,0X19,0X4E,0X19,0X4E,0XD8,0X45,0XF9,0X4D,0X1A,0X4E,0XF9,0X45,0XD9,0X4D,
0XF9,0X4D,0XD8,0X4D,0X97,0X55,0X97,0X65,0XAC,0X0A,0XAC,0X12,0X98,0X65,0XF9,0X55,
0XF9,0X45,0XF9,0X45,0XF9,0X45,0X19,0X4E,0XF9,0X4D,0XF8,0X4D,0X19,0X4E,0XF9,0X45,
0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X45,0X1A,0X4E,
0XD9,0X45,0XF9,0X45,0X3A,0X4E,0XF9,0X45,0XD9,0X45,0XD9,0X45,0XF9,0X4D,0XF9,0X55,
0XB8,0X5D,0XB8,0X75,0X56,0X7D,0X6B,0X2A,0X4A,0X2A,0X56,0X75,0XF8,0X55,0X19,0X46,
0XF9,0X4D,0XD9,0X45,0X1A,0X4E,0XF9,0X45,0XF8,0X45,0XD8,0X45,0XF9,0X4D,0XF9,0X4D,
0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X19,0X4E,0XD9,0X45,
0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0X1A,0X4E,0XF9,0X45,0XD8,0X55,0X77,0X6D,
0X6B,0X2A,0X2A,0X32,0XFF,0XF7,0XBF,0XEF,0X8A,0X2A,0X96,0X65,0XF8,0X55,0XF9,0X4D,
0XF9,0X4D,0XD9,0X45,0XF9,0X45,0X19,0X4E,0X19,0X4E,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X19,0X46,0XF9,0X45,0XF9,0X4D,0X19,0X4E,
0XD8,0X45,0X19,0X4E,0XF9,0X45,0XF8,0X45,0X18,0X46,0XD7,0X5D,0X6B,0X22,0XFF,0XF7,
0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,0XDF,0XEF,0X4A,0X2A,0X8B,0X12,0XB8,0X55,0XF9,0X4D,
0X1A,0X4E,0XF9,0X45,0X19,0X4E,0XD8,0X45,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF8,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XD8,0X45,0X19,0X4E,0XF7,0X55,0X76,0X65,0X09,0X2A,0XFF,0XFF,0XFF,0XFF,
0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,0X09,0X3A,0X6B,0X2A,0XB8,0X65,0XB8,0X45,0XF9,0X4D,
0XF9,0X45,0XD9,0X45,0X19,0X4E,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF8,0X45,0XF9,0X4D,0XB8,0X4D,
0XF9,0X5D,0X77,0X5D,0X56,0X65,0X76,0X85,0X09,0X32,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XDF,0XFF,0X09,0X42,0X2A,0X2A,0X77,0X65,0XFA,0X5D,0XFA,0X4D,0XD9,0X45,
0X19,0X4E,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0X19,0X46,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X55,0XB8,0X45,0X19,0X46,0X19,0X46,0XF9,0X4D,0XD9,0X55,0XB9,0X55,
0XCD,0X02,0X8B,0X12,0X4A,0X22,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XDF,0XFF,0XFF,0XEF,0XCC,0X12,0XB9,0X55,0XFA,0X4D,0XFA,0X45,0XF9,0X45,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0X19,0X46,0X19,0X46,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0X19,0X46,0XF9,0X45,0XFA,0X45,0XDA,0X4D,0XD9,0X4D,0XB9,0X55,
0XD8,0X5D,0X76,0X75,0X29,0X32,0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XE8,0X39,0X6A,0X32,0X77,0X6D,0XB9,0X55,0XFA,0X4D,0XFA,0X45,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0X19,0X46,0X19,0X46,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X45,0XF9,0X45,0XF9,0X45,0XFA,0X45,0XFA,0X45,0XDA,0X4D,0XD9,0X4D,0XD8,0X4D,
0XB7,0X65,0X2A,0X2A,0XBF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XE8,0X39,
0X29,0X2A,0X98,0X65,0XD9,0X55,0XD9,0X4D,0XFA,0X45,0XF9,0X45,0X19,0X4E,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0X19,0X46,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XFA,0X4D,0XD9,0X4D,0XD9,0X4D,0X19,0X4E,0XB7,0X65,
0X2A,0X2A,0XEA,0X39,0XE9,0X39,0XE9,0X39,0XC8,0X39,0XE9,0X39,0XE9,0X39,0X4B,0X2A,
0X97,0X5D,0XF9,0X4D,0XFA,0X4D,0XF9,0X45,0X19,0X4E,0XD9,0X45,0XF9,0X4D,0XF9,0X4D,
0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,0XD8,0X4D,0XD8,0X5D,0X6B,0X1A,
0X4B,0X2A,0X6B,0X2A,0X6B,0X2A,0X4B,0X2A,0X6B,0X2A,0X4B,0X22,0XAC,0X1A,0XB8,0X55,
0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XD9,0X45,0X19,0X4E,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,
0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0X19,0X46,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XD8,0X4D,0XD8,0X55,0XD8,0X55,0X97,0X5D,
0XB7,0X5D,0XB8,0X5D,0XD8,0X5D,0X97,0X55,0XD8,0X55,0XB7,0X4D,0XF9,0X55,0XF9,0X4D,
0X1A,0X4E,0XF9,0X45,0X19,0X4E,0XD8,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0X19,0X46,0X19,0X46,0XF8,0X4D,0XF8,0X4D,
0XF8,0X4D,0XF9,0X4D,0XF9,0X4D,0X19,0X4E,0XD9,0X45,0X19,0X4E,0X19,0X4E,0XD8,0X45,
0XF9,0X4D,0X19,0X46,0X19,0X46,0X18,0X46,0X18,0X46,0XF9,0X4D,0XD9,0X4D,0XD9,0X45,
0X19,0X4E,0XD8,0X45,0X19,0X4E,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XD9,0X3D,0X1A,0X46,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XD9,0X4D,
0XD9,0X4D,0XF9,0X45,0XF9,0X45,0X1A,0X4E,0XD9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,
0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,
0XF9,0X4D,0XFA,0X45,0X19,0X3E,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XD9,0X4D,0XF9,0X4D,
0XF9,0X4D,0X1A,0X4E,0XD9,0X45,0X1A,0X4E,0XF9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF8,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XD9,0X4D,0XF9,0X4D,0XF8,0X4D,0XF8,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,
0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF8,0X4D,0XF8,0X4D,
0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,
0XF8,0X4D,0XF8,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,
0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0X19,0X4E,0XF8,0X45,
0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,
0XF9,0X4D,0XF9,0X45,0X19,0X46,0X19,0X46,0X18,0X46,0X18,0X46,0X19,0X46,0XF9,0X45,
0XF9,0X4D,0XD9,0X4D,0XDA,0X4D,0XDA,0X4D,0XF9,0X4D,0XF9,0X4D,0XF8,0X45,0XF9,0X4D,
0XF9,0X4D,0XF8,0X4D,0XF8,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,
0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF8,0X4D,0XF8,0X4D,0XF9,0X4D,
0XF9,0X4D,0X19,0X46,0X19,0X46,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF8,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF8,0X4D,
0XF8,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X45,
0X19,0X4E,0XF9,0X4D,0XD8,0X45,0XFA,0X4D,0XF9,0X4D,0XF8,0X45,0X18,0X46,0XF9,0X45,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,
0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X45,0X18,0X46,0X18,0X46,0X18,0X46,0XF9,0X45,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XDA,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X45,0XF8,0X45,0XF9,0X45,0XF9,0X45,
0XF8,0X45,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X45,0X19,0X46,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XD9,0X4D,0XD8,0X55,0XF8,0X4D,
0X19,0X46,0X19,0X46,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,
0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,
0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X45,0X19,0X46,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,0X19,0X46,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,
0XD9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0X19,0X46,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X45,
0XF9,0X45,0XF8,0X4D,0XF8,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XFA,0X45,0XF9,0X4D,
0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0X19,0X46,0XF9,0X45,0XF9,0X45,0XD9,0X4D,
0XB9,0X4D,0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XFA,0X45,0XFA,0X45,0XF9,0X45,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,
0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0X19,0X46,0X19,0X46,0XF9,0X4D,
0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XFA,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XDA,0X4D,0XDA,0X4D,
0XD9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF8,0X4D,0XF8,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X45,0X19,0X46,0XF9,0X45,0XF9,0X45,
0XF9,0X45,0XF9,0X45,0X19,0X46,0X19,0X46,0XF9,0X45,0XFA,0X45,0XF9,0X45,0XF9,0X45,
0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XFA,0X4D,0XF9,0X4D,0XB9,0X4D,
0XF9,0X55,0XD9,0X55,0XD8,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XFA,0X45,0XFA,0X45,0XFA,0X45,0X19,0X46,0XF9,0X45,0XF9,0X45,
0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,0XD9,0X55,
0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XFA,0X4D,0XDA,0X4D,0XDA,0X4D,0XDA,0X4D,0XDA,0X4D,
0XDA,0X4D,0XFA,0X4D,0XF9,0X4D,0XF8,0X4D,0XF8,0X4D,0XF8,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XDA,0X4D,0XFA,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XD9,0X4D,0XDA,0X4D,0XD9,0X4D,
0XF9,0X4D,0XF9,0X45,0X19,0X46,0X19,0X46,0XF9,0X45,0XDA,0X4D,0XD9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,0XF9,0X45,0X19,0X46,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,
0XF9,0X45,0X18,0X46,0X19,0X46,0XF9,0X45,0XFA,0X4D,0XF9,0X4D,0XF9,0X45,0X19,0X46,
0X19,0X46,0XF9,0X45,0XF9,0X4D,0XD9,0X45,0XF9,0X4D,0XD9,0X4D,0XD9,0X55,0XB8,0X5D,
0XED,0X02,0XCC,0X02,0XD9,0X5D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X45,0XFA,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF8,0X45,0XF8,0X45,
0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,
0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X45,
0XF9,0X45,0XF9,0X4D,0XF8,0X4D,0XF8,0X4D,0XF8,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,
0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0X19,0X46,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X45,0X19,0X46,0XF9,0X45,0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XDA,0X4D,0XDA,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF8,0X4D,0X19,0X46,0X19,0X46,
0X19,0X46,0XF9,0X45,0X1A,0X46,0X1A,0X46,0XF9,0X4D,0XD9,0X5D,0X97,0X6D,0X6B,0X12,
0X8B,0X0A,0XB8,0X5D,0XD9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF8,0X4D,0XF8,0X4D,0XF8,0X4D,0XF8,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0X18,0X46,0X18,0X46,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0X19,0X46,0XF9,0X45,0XF8,0X4D,
0XF9,0X4D,0XB8,0X5D,0XB8,0X55,0XD8,0X55,0XD8,0X55,0XD8,0X55,0XD8,0X55,0XD8,0X55,
0XD8,0X55,0XD8,0X55,0XF8,0X4D,0XD7,0X55,0XD8,0X55,0XD8,0X55,0XD8,0X55,0XD8,0X55,
0XD8,0X55,0XD8,0X4D,0XD8,0X55,0XB8,0X55,0XB8,0X55,0XB9,0X55,0XD9,0X55,0XB9,0X55,
0XB9,0X55,0XD8,0X55,0XD8,0X55,0XB8,0X55,0XB8,0X55,0XD9,0X55,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,0XF8,0X4D,0X19,0X46,0X19,0X46,0XF8,0X4D,
0XF9,0X4D,0X19,0X46,0XF9,0X45,0XF9,0X4D,0XB8,0X55,0X77,0X6D,0X6B,0X1A,0XAC,0X12,
0X97,0X5D,0XD9,0X4D,0XF9,0X45,0XF9,0X45,0XF8,0X4D,0XF8,0X4D,0XD9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF8,0X4D,0XF8,0X4D,0XF8,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF8,0X4D,0XF8,0X4D,0XD8,0X55,0XD8,0X55,0XD8,0X55,
0XD8,0X55,0XD8,0X55,0XD8,0X55,0XD9,0X55,0XD9,0X55,0XD8,0X55,0XD8,0X55,0XB8,0X55,
0XD8,0X55,0XD8,0X55,0XD8,0X55,0XD8,0X55,0XD8,0X55,0XD7,0X55,0XD8,0X55,0XB9,0X55,
0XB9,0X55,0XB8,0X55,0XD8,0X55,0XD8,0X4D,0XF8,0X4D,0XF8,0X4D,0XD8,0X55,0XD8,0X55,
0X77,0X6D,0X77,0X6D,0XB8,0X6D,0X97,0X65,0X97,0X6D,0X97,0X6D,0X76,0X6D,0X97,0X6D,
0X97,0X6D,0X76,0X65,0X96,0X6D,0X97,0X6D,0X76,0X6D,0X97,0X75,0X97,0X6D,0X97,0X6D,
0X97,0X6D,0X97,0X6D,0X76,0X65,0XB7,0X6D,0X97,0X6D,0X77,0X65,0XB7,0X75,0X76,0X65,
0X96,0X6D,0X97,0X6D,0X77,0X6D,0X97,0X6D,0XB7,0X5D,0XF8,0X55,0XF8,0X45,0X19,0X46,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD8,0X45,0XF8,0X45,0XF8,0X55,0XD8,0X55,
0XD8,0X4D,0X1A,0X4E,0XF9,0X45,0X98,0X4D,0X98,0X75,0X6B,0X12,0X8B,0X0A,0XB8,0X5D,
0XF9,0X4D,0X19,0X4E,0XF9,0X4D,0XD8,0X4D,0XB8,0X55,0XD9,0X55,0XD9,0X4D,0X19,0X4E,
0X19,0X46,0XF8,0X45,0XF8,0X45,0X19,0X4E,0XF8,0X45,0XF9,0X45,0X1A,0X4E,0XF9,0X4D,
0XFA,0X4D,0XD9,0X4D,0X97,0X5D,0X97,0X6D,0X97,0X65,0X96,0X65,0XB7,0X6D,0X96,0X6D,
0X97,0X6D,0X77,0X6D,0X98,0X6D,0X97,0X6D,0X96,0X6D,0XB7,0X6D,0X97,0X6D,0X97,0X6D,
0X96,0X75,0X76,0X6D,0X96,0X6D,0X96,0X6D,0XB6,0X6D,0X96,0X65,0X77,0X6D,0X77,0X6D,
0X97,0X6D,0X97,0X6D,0X97,0X65,0XB7,0X65,0XB7,0X6D,0X97,0X6D,0X56,0X65,0XFF,0XEF,
0XFF,0XEF,0XFF,0XE7,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XF7,0XFF,0XEF,0XFF,0XEF,
0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,
0XFF,0XF7,0XFF,0XEF,0XFF,0XE7,0XFF,0XEF,0XFF,0XE7,0XFF,0XEF,0XFF,0XF7,0XFF,0XF7,
0XFF,0XF7,0XFF,0XF7,0XFF,0XEF,0XFF,0XDF,0XB7,0X6D,0XB7,0X4D,0X19,0X4E,0X19,0X4E,
0XD8,0X45,0XD9,0X45,0XD9,0X4D,0X19,0X4E,0XF9,0X4D,0X97,0X55,0XB8,0X5D,0XD9,0X55,
0XB9,0X4D,0XF9,0X4D,0XF9,0X55,0X77,0X65,0XAC,0X12,0XAC,0X02,0XD9,0X5D,0XB8,0X4D,
0XD9,0X4D,0XD9,0X55,0XB9,0X5D,0XB8,0X5D,0X98,0X55,0XD9,0X4D,0XF9,0X45,0XF9,0X45,
0X19,0X46,0XF9,0X45,0XF9,0X4D,0X19,0X4E,0XF9,0X45,0XD9,0X45,0XD9,0X4D,0X98,0X5D,
0X97,0X75,0XFF,0XE7,0XFF,0XEF,0XFF,0XF7,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,
0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFE,0XEF,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,
0XFF,0XF7,0XFF,0XF7,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,
0XFF,0XEF,0XFF,0XF7,0XFF,0XEF,0XFF,0XEF,0XFF,0XE7,0XFF,0XE7,0XDE,0XFF,0XBD,0XFF,
0X9D,0XFF,0X7C,0XFF,0X7C,0XFF,0XBD,0XFF,0X7D,0XFF,0X9D,0XFF,0X9C,0XFF,0X3B,0XFF,
0X7C,0XFF,0X5D,0XFF,0X7D,0XFF,0XBD,0XFF,0X9D,0XFF,0X7C,0XFF,0X3B,0XFF,0X5C,0XFF,
0X9D,0XFF,0XBD,0XFF,0X9E,0XFF,0XBE,0XFF,0X7C,0XFF,0X5C,0XFF,0X5C,0XFF,0X7D,0XFF,
0X3C,0XFF,0X7E,0XFF,0XBF,0XFF,0XFF,0XE7,0X97,0X6D,0XB8,0X55,0XD8,0X4D,0XF9,0X45,
0XFA,0X4D,0XFA,0X4D,0XD9,0X4D,0XD8,0X55,0XCD,0X0A,0X78,0X6D,0X99,0X65,0XB9,0X5D,
0XF9,0X55,0XB8,0X4D,0XB8,0X65,0XAC,0X02,0XCD,0X02,0XB8,0X55,0XF9,0X55,0XF9,0X55,
0X98,0X5D,0XAD,0X0A,0XAD,0X0A,0XCD,0X02,0XF9,0X55,0XD9,0X4D,0XD9,0X4D,0X1A,0X56,
0XD9,0X4D,0XFA,0X4D,0XF9,0X45,0XF9,0X45,0XD9,0X55,0X97,0X65,0XFF,0XE7,0XDF,0XFF,
0X7D,0XFF,0X7C,0XFF,0X7C,0XFF,0X7D,0XFF,0X9D,0XFF,0X9E,0XFF,0X9E,0XFF,0X9E,0XFF,
0X9D,0XFF,0X7C,0XFF,0X9B,0XFF,0X9B,0XFF,0X5B,0XFF,0X7C,0XFF,0X7C,0XFF,0X5C,0XFF,
0X5C,0XFF,0X7D,0XFF,0X9D,0XFF,0X7D,0XFF,0XBE,0XFF,0XBE,0XFF,0XBE,0XFF,0X9D,0XFF,
0X5B,0XFF,0X7C,0XFF,0X9D,0XFF,0XDE,0XFF,0XFF,0XFF,0X9D,0XFF,0X88,0X8A,0X06,0XA2,
0X46,0XAA,0X68,0X92,0X68,0X8A,0X68,0X9A,0X47,0XA2,0X05,0XAA,0X26,0XB2,0XE6,0XA9,
0X28,0XAA,0X68,0X9A,0X68,0X92,0X68,0X9A,0X47,0XAA,0X06,0XBA,0X27,0XB2,0X68,0X92,
0X89,0X82,0X88,0X8A,0X48,0X92,0X48,0X9A,0X26,0XAA,0X06,0XBA,0X06,0XC2,0X26,0XC2,
0X07,0XB2,0X1B,0XFF,0X9E,0XFF,0XFF,0XEF,0XB7,0X6D,0XB7,0X55,0XF8,0X55,0XD9,0X4D,
0XDA,0X4D,0XD9,0X45,0XD9,0X55,0XB8,0X65,0X8C,0X02,0X8C,0X02,0XED,0X02,0XB8,0X4D,
0XB8,0X4D,0XD8,0X5D,0XB8,0X5D,0XB8,0X5D,0XB8,0X55,0XD8,0X55,0XB8,0X55,0XCD,0X02,
0X99,0X65,0X99,0X65,0XB9,0X5D,0XD9,0X55,0XD9,0X4D,0XD9,0X55,0XBA,0X4D,0XDA,0X4D,
0XDA,0X4D,0X19,0X4E,0XF8,0X55,0X76,0X65,0XFF,0XEF,0XBE,0XFF,0XB9,0XFE,0X27,0XB2,
0X06,0XBA,0X06,0XB2,0X47,0XAA,0X68,0X92,0X89,0X8A,0X69,0X8A,0X69,0X92,0X48,0X9A,
0X47,0XAA,0X26,0XAA,0X26,0XB2,0X26,0XAA,0X06,0XA2,0X27,0XA2,0X27,0XA2,0X27,0XB2,
0X06,0XAA,0X48,0XA2,0X68,0X9A,0X28,0X8A,0X69,0X8A,0X48,0X8A,0X48,0X9A,0X47,0XAA,
0X67,0XB2,0X88,0XA2,0XFA,0XFE,0XDE,0XFF,0X5B,0XFF,0X47,0XA2,0XC5,0XC1,0XE5,0XC1,
0X3B,0XFF,0X5B,0XFF,0X99,0XFE,0X47,0XC2,0XC4,0XC9,0XE5,0XC9,0XA5,0XB1,0X28,0XB2,
0X3A,0XFF,0X1A,0XFF,0XFA,0XFE,0XE7,0XB1,0X85,0XD1,0XE6,0XC9,0X7C,0XFF,0XBD,0XFF,
0X7C,0XFF,0X3A,0XFF,0X3A,0XFF,0X26,0XBA,0XC4,0XD9,0X82,0XE1,0XA2,0XF1,0X82,0XE9,
0X84,0XD9,0X07,0XBA,0X7D,0XFF,0XFF,0XE7,0X95,0X75,0X96,0X5D,0X1A,0X56,0XD9,0X45,
0XFA,0X45,0XF9,0X4D,0XD8,0X4D,0XF9,0X5D,0XF9,0X55,0XB8,0X4D,0XF9,0X4D,0XF9,0X55,
0XD8,0X5D,0X97,0X65,0X97,0X65,0XB7,0X65,0XD8,0X55,0XB8,0X4D,0XF9,0X55,0XB8,0X4D,
0XF9,0X55,0XB8,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XBA,0X4D,0XFA,0X4D,0XF9,0X4D,
0XB7,0X4D,0XB6,0X65,0XFF,0XEF,0X5C,0XFF,0X06,0XB2,0X83,0XD1,0X84,0XE9,0X83,0XE9,
0XA4,0XE1,0XE5,0XC1,0X3B,0XFF,0X5C,0XFF,0X7D,0XFF,0X5C,0XFF,0XB9,0XFE,0XA5,0XB9,
0XE6,0XC9,0XA5,0XB9,0XE6,0XB1,0XFA,0XFE,0XFA,0XFE,0X06,0XAA,0X84,0XB1,0X06,0XBA,
0X68,0XAA,0X5C,0XFF,0X5D,0XFF,0X1D,0XFF,0X1C,0XFF,0X3B,0XFF,0XE6,0XA9,0X26,0XB2,
0X26,0XB2,0XD9,0XFE,0XBE,0XFF,0X5B,0XFF,0X27,0XAA,0X83,0XD1,0XA4,0XD1,0X47,0XA2,
0X1A,0XFF,0XE6,0XB9,0XA5,0XC9,0XA4,0XD1,0X36,0XFE,0XBA,0XFE,0X99,0XFE,0X67,0XBA,
0X05,0XAA,0X27,0XAA,0X99,0XFE,0X84,0XD9,0XA5,0XC9,0X9D,0XFF,0XBD,0XFF,0X67,0X92,
0X26,0XAA,0X47,0XAA,0X36,0XFE,0X82,0XE1,0X61,0XF9,0X20,0XF9,0X41,0XF9,0X22,0XF9,
0X64,0XD9,0X67,0XA2,0X9C,0XFF,0XFE,0XEF,0XFF,0XD7,0XD8,0X5D,0XD9,0X4D,0X1A,0X4E,
0XF9,0X45,0XF8,0X45,0XF8,0X45,0XF8,0X45,0X19,0X4E,0XF9,0X4D,0XB8,0X55,0X77,0X65,
0X8B,0X12,0X6B,0X1A,0X8B,0X12,0XB7,0X65,0XB8,0X4D,0XF9,0X4D,0X19,0X46,0XD8,0X45,
0X3A,0X4E,0XF9,0X45,0XF9,0X45,0XFA,0X4D,0XB9,0X4D,0XD9,0X4D,0XB8,0X55,0XD7,0X6D,
0XFF,0XE7,0X9D,0XFF,0X28,0XAA,0X83,0XD9,0X42,0XF9,0X21,0XF9,0X22,0XF9,0X42,0XF1,
0X83,0XD1,0X68,0XAA,0X7C,0XFF,0X1B,0XFF,0X68,0XA2,0XE6,0XC9,0X84,0XD1,0XC5,0XD1,
0XF6,0XFD,0X99,0XFE,0X28,0XAA,0X48,0XA2,0XB9,0XFE,0X78,0XFE,0X26,0XBA,0X67,0XA2,
0XFA,0XFE,0X3C,0XFF,0X49,0X9A,0X28,0XAA,0X27,0XAA,0XDA,0XFE,0X98,0XFE,0X46,0XB2,
0XB8,0XFE,0XBD,0XFF,0X5C,0XFF,0XE6,0XA1,0XC4,0XE1,0X62,0XE1,0XA4,0XB1,0X97,0XFE,
0X83,0XD9,0X83,0XE1,0XC4,0XC9,0X1A,0XFF,0X3C,0XFF,0X3B,0XFF,0XC5,0XB9,0XC4,0XC9,
0X47,0XB2,0XF9,0XFE,0XA4,0XD1,0XE5,0XC1,0X7C,0XFF,0X9D,0XFF,0X26,0XB2,0XA4,0XB9,
0X26,0XBA,0X98,0XFE,0X42,0XE1,0X62,0XF9,0X00,0XF9,0X21,0XF9,0X62,0XF9,0X62,0XE1,
0XC4,0XC1,0X1A,0XFF,0XBD,0XFF,0XFF,0XF7,0X97,0X6D,0XB8,0X4D,0XFA,0X4D,0X19,0X4E,
0XD8,0X45,0X19,0X4E,0X19,0X46,0XB8,0X45,0XD9,0X5D,0XB8,0X6D,0X36,0X75,0X2A,0X2A,
0XE9,0X39,0X2A,0X3A,0X37,0X75,0XD9,0X65,0XD8,0X4D,0XF9,0X45,0X19,0X4E,0XF9,0X45,
0XD8,0X45,0X19,0X46,0XD9,0X45,0XD9,0X4D,0XF9,0X5D,0X77,0X6D,0XFF,0XEF,0X7D,0XFF,
0X1B,0XFF,0X84,0XC9,0X63,0XE9,0X22,0XF9,0X62,0XF9,0X21,0XF9,0X21,0XF9,0XA3,0XE9,
0XE5,0XB1,0X1B,0XFF,0X78,0XFE,0X06,0XC2,0X42,0XE1,0X62,0XE1,0X05,0XCA,0X97,0XFE,
0XB9,0XFE,0X27,0XA2,0X89,0XA2,0XB9,0XFE,0X1A,0XFF,0X68,0XA2,0X88,0X9A,0X1A,0XFF,
0X1B,0XFF,0XE6,0XA9,0X83,0XC9,0XE5,0XC1,0X1A,0XFF,0X7B,0XFF,0X25,0X9A,0XD9,0XFE,
0X9D,0XFF,0X3B,0XFF,0X26,0XAA,0X83,0XE1,0X62,0XE9,0XE4,0XC1,0X76,0XFE,0X62,0XE9,
0X62,0XE9,0XC4,0XC9,0X1A,0XFF,0XBE,0XFF,0X7D,0XFF,0X06,0XC2,0XC4,0XC9,0X46,0XAA,
0XF9,0XFE,0XA4,0XC9,0XC4,0XB9,0X9D,0XFF,0X7C,0XFF,0X06,0XBA,0XC4,0XC9,0X26,0XBA,
0X57,0XFE,0X43,0XE1,0X22,0XF9,0X21,0XF9,0X21,0XF9,0X21,0XF9,0X62,0XF1,0XC4,0XD9,
0X06,0XBA,0X88,0X92,0XDE,0XFF,0XFF,0XD7,0XB8,0X65,0XD9,0X55,0XD9,0X45,0X19,0X4E,
0XD8,0X45,0XF9,0X45,0XD9,0X4D,0XB8,0X65,0X6C,0X1A,0X4B,0X32,0XFF,0XFF,0XBF,0XFF,
0XDF,0XFF,0X4B,0X2A,0X8C,0X12,0XD9,0X55,0XF9,0X45,0XF9,0X45,0X19,0X4E,0XF9,0X45,
0X19,0X46,0X19,0X46,0XF8,0X4D,0X77,0X65,0XFF,0XE7,0XFF,0XFF,0X89,0X92,0XC5,0XB9,
0XA4,0XE1,0X42,0XF1,0X62,0XF9,0X21,0XF9,0X42,0XF9,0X41,0XF9,0X62,0XE9,0X05,0XC2,
0XFA,0XFE,0X58,0XFE,0XC5,0XD1,0X42,0XF1,0X61,0XF1,0X04,0XC2,0X97,0XFE,0X1A,0XFF,
0X68,0X92,0X68,0X8A,0X3B,0XFF,0X3B,0XFF,0X47,0X92,0X88,0X9A,0X3A,0XFF,0X1A,0XFF,
0X26,0XB2,0XA4,0XD9,0XE5,0XC9,0X7B,0XFF,0X7B,0XFF,0X06,0X9A,0XB9,0XFE,0XBE,0XFF,
0X3B,0XFF,0X26,0XAA,0XA3,0XE1,0X62,0XE9,0XC3,0XC1,0X96,0XFE,0X62,0XE9,0X82,0XE9,
0XC4,0XC9,0XFA,0XFE,0X9E,0XFF,0X7D,0XFF,0X27,0XA2,0X05,0XB2,0X67,0X9A,0XF9,0XFE,
0XA4,0XC9,0XC5,0XC1,0X9D,0XFF,0X9D,0XFF,0X27,0XAA,0XE5,0XB1,0X06,0XA2,0XD9,0XFE,
0X43,0XE1,0X02,0XF9,0X42,0XF9,0X21,0XF9,0X41,0XF9,0X41,0XF1,0X83,0XD9,0XC5,0XB9,
0X47,0X92,0XDD,0XFF,0XFF,0XEF,0XFF,0XD7,0XB8,0X55,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X45,0XD8,0X55,0X78,0X75,0X4B,0X2A,0X0A,0X3A,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,
0X0A,0X3A,0X4B,0X22,0X98,0X55,0XFA,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0X19,0X46,
0XF8,0X45,0XB7,0X55,0XFF,0XD7,0XFF,0XF7,0XBE,0XFF,0X68,0X92,0XE5,0XB9,0XA4,0XD9,
0X62,0XF1,0X41,0XF9,0X21,0XF9,0X21,0XF9,0X42,0XF9,0X63,0XE9,0XE6,0XC9,0XDA,0XFE,
0X98,0XFE,0XC4,0XC1,0X62,0XE9,0X62,0XE9,0XE5,0XC9,0X97,0XFE,0X3A,0XFF,0X88,0X82,
0XA9,0X82,0X3B,0XFF,0X3C,0XFF,0X48,0X9A,0X89,0X9A,0X1B,0XFF,0X3B,0XFF,0X47,0X9A,
0XA5,0XB1,0XE6,0XB9,0X19,0XFF,0X5A,0XFF,0XE6,0XA1,0XB9,0XFE,0XBD,0XFF,0X5C,0XFF,
0X06,0XAA,0X62,0XE1,0X62,0XE9,0XC3,0XC1,0X76,0XFE,0X82,0XE9,0X83,0XE9,0XA4,0XC1,
0X3B,0XFF,0X9E,0XFF,0XBE,0XFF,0X3B,0XFF,0XD8,0XFE,0X1A,0XFF,0X1A,0XFF,0XC5,0XC9,
0XC5,0XB9,0X7D,0XFF,0XBE,0XFF,0XFA,0XFE,0X98,0XFE,0XD8,0XFE,0XE5,0XC1,0X43,0XE9,
0X43,0XF9,0X01,0XF9,0X41,0XF9,0X00,0XF9,0X41,0XE9,0X26,0XD2,0XD9,0XFE,0X1B,0XFF,
0XDE,0XFF,0XFF,0XEF,0X76,0X6D,0XB8,0X55,0XD8,0X4D,0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,
0XD8,0X55,0XB8,0X7D,0X2A,0X2A,0X09,0X42,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XE8,0X39,
0X6B,0X32,0XB9,0X65,0XD9,0X45,0XFA,0X4D,0XD9,0X45,0X19,0X4E,0XF8,0X45,0X19,0X4E,
0XD8,0X55,0X56,0X6D,0XFF,0XEF,0XDF,0XFF,0X3C,0XFF,0XB9,0XFE,0X05,0XCA,0X82,0XE9,
0X41,0XF9,0X21,0XF9,0X42,0XF9,0X41,0XF9,0X63,0XE9,0XC6,0XC1,0XFA,0XFE,0X78,0XFE,
0X05,0XCA,0X62,0XE9,0X42,0XE9,0XC4,0XC1,0X98,0XFE,0X7B,0XFF,0X3A,0XFF,0X7C,0XFF,
0X9D,0XFF,0X1C,0XFF,0X69,0X9A,0XAA,0XA2,0X1B,0XFF,0X5B,0XFF,0X1A,0XFF,0XB9,0XFE,
0X98,0XFE,0X45,0XB2,0X46,0XB2,0X07,0XBA,0XBA,0XFE,0XBD,0XFF,0X5C,0XFF,0XE6,0XA9,
0XC4,0XE9,0X42,0XE9,0XA4,0XC1,0X77,0XFE,0X83,0XE1,0X83,0XE1,0XE5,0XC1,0X3B,0XFF,
0X9E,0XFF,0XBE,0XFF,0XF9,0XFE,0XB8,0XFE,0XFA,0XFE,0X3B,0XFF,0XC4,0XC9,0XE4,0XB9,
0X9D,0XFF,0XBE,0XFF,0X1A,0XFF,0XB8,0XFE,0X56,0XFE,0XC4,0XD9,0X43,0XF1,0X01,0XF9,
0X21,0XF9,0X41,0XF9,0X62,0XF9,0X83,0XD9,0X26,0XA2,0X7D,0XFF,0XDF,0XFF,0XFF,0XF7,
0XFF,0XD7,0X97,0X5D,0XD8,0X4D,0X19,0X4E,0XF9,0X4D,0XD9,0X4D,0XF9,0X4D,0XD8,0X55,
0X56,0X6D,0X6A,0X32,0X29,0X42,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0X09,0X42,0X0A,0X2A,
0X98,0X65,0XFA,0X4D,0XD9,0X45,0X1A,0X4E,0XF9,0X45,0XF9,0X4D,0XD8,0X45,0XD8,0X4D,
0XD8,0X5D,0X96,0X6D,0XFF,0XE7,0XBE,0XFF,0X7C,0XFF,0X06,0XA2,0X83,0XD1,0X21,0XF1,
0X62,0XF9,0X21,0XF9,0X41,0XF9,0X82,0XE1,0X05,0XC2,0X19,0XFF,0XB8,0XFE,0XE5,0XC1,
0X63,0XE9,0X43,0XE9,0X26,0XCA,0XB9,0XFE,0X3A,0XFF,0X1A,0XFF,0X3A,0XFF,0X7C,0XFF,
0X5C,0XFF,0X48,0X92,0XA9,0X9A,0X1A,0XFF,0X7B,0XFF,0XB8,0XFE,0XB7,0XFE,0X56,0XFE,
0X63,0XD1,0XC5,0XE1,0XE6,0XC1,0X99,0XFE,0XBE,0XFF,0X5C,0XFF,0X27,0XAA,0X63,0XE1,
0X42,0XF1,0XA4,0XC9,0X37,0XFE,0X63,0XE9,0XA3,0XE9,0XA3,0XC1,0X19,0XFF,0X9D,0XFF,
0X5D,0XFF,0X47,0XAA,0X06,0XB2,0X48,0XA2,0XD9,0XFE,0XA3,0XC9,0XE4,0XC1,0X7D,0XFF,
0X7D,0XFF,0X46,0XA2,0XC4,0XB9,0X83,0XD1,0X83,0XE9,0X42,0XF9,0X21,0XF9,0X41,0XF9,
0X21,0XF1,0X62,0XE9,0XC5,0XC9,0X5C,0XFF,0XFF,0XFF,0XFF,0XEF,0X56,0X7D,0X77,0X65,
0XD8,0X5D,0XF8,0X45,0X19,0X46,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD8,0X55,0XB8,0X7D,
0X4A,0X2A,0XE8,0X41,0XDF,0XFF,0XFF,0XFF,0XDF,0XFF,0XE9,0X41,0X2A,0X2A,0XB8,0X6D,
0XD9,0X55,0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X45,0X19,0X4E,0XF8,0X4D,
0XB7,0X55,0XB7,0X6D,0XFF,0XE7,0XFF,0XFF,0X7C,0XFF,0XC5,0XC1,0X42,0XE1,0X62,0XF1,
0X41,0XF9,0X40,0XF1,0XA2,0XE9,0X24,0XC2,0XF8,0XFE,0X76,0XFE,0XE5,0XC9,0X43,0XE9,
0X23,0XE9,0XE5,0XC1,0X78,0XFE,0X3B,0XFF,0X68,0X92,0X88,0X8A,0X3A,0XFF,0X3B,0XFF,
0X27,0X9A,0X88,0XA2,0X3A,0XFF,0XF9,0XFE,0X66,0XAA,0X04,0XBA,0XE4,0XD1,0X83,0XF1,
0X22,0XE1,0X06,0XCA,0XB9,0XFE,0XBE,0XFF,0X5B,0XFF,0X26,0XAA,0X84,0XE9,0X22,0XE9,
0X84,0XD1,0X77,0XFE,0X43,0XE9,0X63,0XE9,0XC4,0XD1,0XF9,0XFE,0X1A,0XFF,0XFA,0XFE,
0XC5,0XC9,0XA5,0XD9,0XA5,0XB9,0XB9,0XFE,0X83,0XD9,0X83,0XC9,0X1B,0XFF,0X3B,0XFF,
0XE5,0XC1,0XA3,0XE1,0X42,0XE9,0X42,0XF9,0X41,0XF9,0X21,0XF9,0X42,0XF9,0X83,0XF1,
0X83,0XC9,0X26,0XAA,0X7D,0XFF,0XFF,0XF7,0XFF,0XDF,0X98,0X65,0XB9,0X5D,0XF9,0X55,
0XF9,0X45,0X19,0X46,0X1A,0X4E,0XF9,0X45,0XF9,0X45,0XB8,0X55,0X56,0X75,0X2A,0X32,
0XE8,0X41,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0X09,0X42,0X29,0X2A,0X56,0X6D,0XB8,0X65,
0XB8,0X5D,0XB8,0X55,0XF9,0X4D,0XD8,0X45,0XF9,0X4D,0XFA,0X45,0XF9,0X45,0XF9,0X45,
0XD8,0X55,0X97,0X6D,0XFF,0XEF,0X9E,0XFF,0X07,0XA2,0X84,0XC9,0X83,0XE9,0X42,0XF1,
0X61,0XF9,0XA2,0XE9,0XC3,0XC9,0XD8,0XFE,0X56,0XFE,0XC4,0XD9,0X43,0XF1,0X43,0XF1,
0XA4,0XD1,0X37,0XFE,0X99,0XFE,0X06,0XB2,0X46,0XAA,0XB8,0XFE,0XF9,0XFE,0X26,0XAA,
0X68,0XBA,0X77,0XFE,0XD9,0XFE,0XC4,0XC1,0XA3,0XD9,0X41,0XE1,0X42,0XF1,0X83,0XE9,
0XC5,0XC1,0XD9,0XFE,0XBD,0XFF,0X7C,0XFF,0X26,0X9A,0XC5,0XD1,0XA5,0XE1,0XC5,0XC1,
0XC5,0XC1,0XA4,0XD9,0XA4,0XD9,0XA3,0XC1,0X25,0XB2,0X68,0XA2,0X88,0XB2,0XA5,0XC9,
0XA4,0XD1,0XE6,0XC1,0XE6,0XC1,0XA5,0XC9,0XE6,0XC1,0X27,0XA2,0X06,0XA2,0XC5,0XC1,
0X83,0XD1,0X83,0XE1,0XA3,0XE9,0X83,0XE1,0XA3,0XE1,0X84,0XE1,0XA5,0XD1,0XFA,0XFE,
0X1B,0XFF,0XDE,0XFF,0XFF,0XEF,0X97,0X65,0XD9,0X55,0XD9,0X55,0XD9,0X4D,0X19,0X4E,
0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XB8,0X5D,0X56,0X85,0X2A,0X3A,0X09,0X4A,
0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XE8,0X39,0X29,0X3A,0X6B,0X2A,0X6B,0X1A,0XAC,0X1A,
0XAC,0X12,0XB7,0X55,0XF9,0X4D,0XF9,0X4D,0XFA,0X4D,0X1A,0X46,0X3A,0X46,0XF9,0X45,
0XB8,0X55,0X77,0X6D,0XFF,0XEF,0X7D,0XFF,0XB9,0XFE,0XC5,0XC9,0XA4,0XD9,0XA3,0XE1,
0XC3,0XD1,0XE5,0XC1,0XC4,0XB1,0X05,0XC2,0XA4,0XC9,0X63,0XD9,0XA4,0XE1,0XC5,0XC9,
0X06,0XC2,0X07,0XBA,0XE6,0XB1,0X26,0XB2,0X46,0XB2,0X26,0XAA,0X06,0XAA,0X27,0XB2,
0X06,0XBA,0XC5,0XB9,0X06,0XCA,0XA4,0XD9,0XA4,0XE1,0XA3,0XE1,0XE4,0XD1,0X26,0XB2,
0XFA,0XFE,0XBE,0XFF,0XBD,0XFF,0XA9,0X82,0X27,0XA2,0X27,0XAA,0X27,0XAA,0X27,0XAA,
0X06,0XB2,0X05,0XAA,0X67,0XA2,0X67,0X9A,0X47,0X92,0X68,0X9A,0X27,0XAA,0X26,0XB2,
0X26,0XA2,0X47,0XAA,0X07,0XA2,0X28,0XAA,0X48,0X9A,0X48,0X9A,0X47,0XAA,0X46,0XAA,
0X46,0XAA,0X05,0XAA,0X06,0XB2,0X06,0XB2,0X27,0XAA,0X69,0X9A,0X1C,0XFF,0XBE,0XF7,
0XFF,0XF7,0XFF,0XDF,0XB7,0X55,0XF9,0X4D,0XF9,0X55,0XD9,0X4D,0XF9,0X4D,0XD9,0X45,
0XF9,0X4D,0XFA,0X4D,0XF9,0X4D,0XB8,0X65,0X37,0X8D,0XC9,0X41,0X09,0X4A,0XFF,0XFF,
0XDF,0XFF,0XFF,0XFF,0XC7,0X41,0X09,0X42,0XE8,0X31,0XE9,0X31,0XE9,0X31,0X4A,0X2A,
0X97,0X6D,0XD8,0X55,0X1A,0X4E,0XB9,0X3D,0X1A,0X4E,0XD9,0X45,0XF9,0X4D,0XF9,0X4D,
0XB8,0X55,0X97,0X6D,0XFF,0XD7,0X9D,0XFF,0X69,0X92,0X28,0XAA,0X06,0XAA,0X26,0XAA,
0X47,0XAA,0X68,0XAA,0X47,0XAA,0X47,0XB2,0X06,0XB2,0X06,0XB2,0X47,0XAA,0X27,0XA2,
0X27,0X9A,0X68,0XA2,0X48,0XA2,0X47,0XA2,0X68,0XAA,0X48,0XA2,0X68,0XA2,0X68,0XA2,
0X68,0XA2,0X27,0XA2,0X47,0XAA,0X46,0XAA,0X06,0XB2,0X26,0XA2,0X88,0X9A,0X3C,0XFF,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XBE,0XFF,0X9D,0XFF,0X9D,0XFF,0X7D,0XFF,0X9D,0XFF,
0XBD,0XFF,0XBC,0XFF,0XBD,0XFF,0XBD,0XFF,0X9D,0XFF,0X9D,0XFF,0X7C,0XFF,0XBD,0XFF,
0X9D,0XFF,0X9E,0XFF,0X9E,0XFF,0X9E,0XFF,0X9D,0XFF,0XBD,0XFF,0X9C,0XFF,0XBC,0XFF,
0XBD,0XFF,0X9D,0XFF,0X7D,0XFF,0X9E,0XFF,0XBF,0XFF,0XFF,0XEF,0XFF,0XDF,0XFF,0XDF,
0X96,0X65,0XD8,0X4D,0X19,0X4E,0XD8,0X45,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XFA,0X4D,
0XD9,0X4D,0XD8,0X5D,0X97,0X6D,0X2B,0X3A,0XEA,0X49,0XC8,0X41,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XF7,0X8B,0X22,
0X97,0X65,0XD8,0X55,0XF9,0X4D,0XF9,0X45,0X19,0X4E,0XF8,0X45,0XF8,0X45,0X1A,0X4E,
0XD8,0X4D,0XB7,0X5D,0X55,0X7D,0XFF,0XFF,0X9E,0XFF,0X7D,0XFF,0X9D,0XFF,0X9E,0XFF,
0X7D,0XFF,0X9D,0XFF,0X9D,0XFF,0X9D,0XFF,0X9D,0XFF,0X7C,0XFF,0XBD,0XFF,0XBD,0XFF,
0X9E,0XFF,0X7D,0XFF,0X9D,0XFF,0X7D,0XFF,0X9D,0XFF,0X9D,0XFF,0X9E,0XFF,0X9D,0XFF,
0XBE,0XFF,0X9D,0XFF,0X9D,0XFF,0X9D,0XFF,0X9D,0XFF,0XBE,0XFF,0XDF,0XFF,0XFF,0XFF,
0XFF,0XDF,0XFF,0XDF,0XFF,0XE7,0XFF,0XEF,0XFF,0XF7,0XFF,0XF7,0XFF,0XEF,0XFF,0XEF,
0XFF,0XE7,0XFF,0XE7,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XE7,0XFF,0XE7,0XFF,0XE7,
0XFF,0XE7,0XFF,0XE7,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,
0XFF,0XEF,0XFF,0XE7,0XFF,0XEF,0XFF,0XDF,0XFF,0XCF,0XB8,0X65,0XB8,0X5D,0XD7,0X55,
0XF8,0X4D,0XF8,0X4D,0X19,0X4E,0XF9,0X45,0X1A,0X4E,0XD9,0X45,0XF9,0X55,0XB8,0X5D,
0X97,0X65,0X76,0X85,0X0A,0X3A,0XC9,0X49,0XE9,0X49,0XDF,0XFF,0XFF,0XFF,0XFF,0XF7,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0X4A,0X32,0X36,0X7D,
0X98,0X65,0XD9,0X55,0XF9,0X4D,0XF8,0X45,0X18,0X46,0XF8,0X45,0X1A,0X4E,0XD9,0X45,
0XF9,0X55,0X97,0X5D,0XFF,0XDF,0XFF,0XE7,0XFF,0XE7,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,
0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XE7,0XFF,0XE7,0XFF,0XE7,0XFF,0XEF,
0XFF,0XEF,0XFF,0XEF,0XFF,0XE7,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,
0XFF,0XE7,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XE7,0XFF,0XE7,0XFF,0XDF,0X97,0X65,
0X97,0X65,0X97,0X6D,0X97,0X6D,0X77,0X6D,0X77,0X6D,0X97,0X6D,0X97,0X65,0X97,0X65,
0X97,0X65,0X77,0X6D,0X77,0X6D,0X97,0X65,0X97,0X65,0X97,0X65,0X97,0X65,0XB7,0X65,
0X97,0X65,0X97,0X65,0X97,0X65,0X97,0X6D,0X77,0X6D,0X97,0X6D,0X97,0X6D,0X77,0X65,
0XB8,0X6D,0X97,0X65,0X97,0X65,0XF8,0X5D,0XB8,0X4D,0XD9,0X4D,0XF9,0X4D,0XF8,0X4D,
0XF8,0X4D,0XF9,0X4D,0X1A,0X4E,0XD9,0X45,0XFA,0X4D,0XD9,0X5D,0X8B,0X0A,0X4A,0X1A,
0XE8,0X29,0XFF,0XFF,0XE8,0X49,0XC8,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0X4A,0X32,0X8C,0X12,
0X77,0X55,0XF9,0X55,0XF8,0X45,0X39,0X4E,0X18,0X46,0XF9,0X45,0X1A,0X4E,0XF9,0X4D,
0XD8,0X4D,0XB7,0X5D,0XB7,0X65,0X97,0X65,0X97,0X6D,0X97,0X6D,0X97,0X6D,0X97,0X6D,
0X97,0X6D,0X96,0X65,0X97,0X65,0XB7,0X65,0XB7,0X65,0X97,0X65,0X97,0X65,0X97,0X65,
0X97,0X65,0X96,0X65,0X96,0X65,0X97,0X6D,0X97,0X6D,0X97,0X6D,0XB7,0X6D,0XB7,0X6D,
0XB7,0X6D,0XB7,0X75,0XB7,0X75,0XD7,0X6D,0XB7,0X6D,0XB7,0X6D,0XD8,0X55,0XD8,0X55,
0XD8,0X55,0XD8,0X55,0XD8,0X55,0XD8,0X55,0XD8,0X55,0XD8,0X55,0XD8,0X55,0XD8,0X55,
0XB8,0X55,0XB8,0X55,0XD8,0X55,0XD8,0X55,0XD9,0X55,0XD9,0X55,0XF8,0X4D,0XD8,0X4D,
0XD9,0X55,0XD9,0X55,0XB9,0X55,0XD8,0X55,0XD8,0X55,0XD8,0X55,0XF9,0X55,0XD9,0X55,
0XD8,0X4D,0XF8,0X4D,0XF8,0X4D,0XF9,0X4D,0XB8,0X4D,0XD9,0X55,0XF9,0X4D,0XD8,0X4D,
0XF9,0X4D,0XFA,0X45,0XFA,0X45,0XD9,0X4D,0X97,0X5D,0XAC,0X22,0XE9,0X31,0XE9,0X49,
0XDF,0XFF,0XC7,0X39,0XE8,0X41,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XDF,0XFF,0X09,0X32,0X6B,0X12,0XD8,0X65,
0XD9,0X55,0XF9,0X45,0X18,0X46,0X18,0X46,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0X19,0X56,
0XD7,0X4D,0XF7,0X55,0XF8,0X55,0XD8,0X55,0XD8,0X55,0XD8,0X55,0XD8,0X55,0XD8,0X55,
0XD8,0X55,0XF8,0X4D,0XF8,0X4D,0XF8,0X4D,0XD8,0X55,0XD8,0X55,0XD8,0X55,0XD8,0X55,
0XD8,0X55,0XB8,0X55,0XD8,0X55,0XD8,0X5D,0XD8,0X5D,0XF8,0X5D,0XF8,0X5D,0XF8,0X5D,
0XD8,0X65,0XD8,0X65,0XF8,0X5D,0XF8,0X5D,0XF8,0X5D,0X19,0X46,0X19,0X46,0XF9,0X45,
0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XD9,0X45,0X19,0X4E,
0X18,0X46,0X18,0X46,0X19,0X4E,0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,0XD9,0X4D,0XFA,0X4D,
0XD9,0X45,0XF9,0X45,0XFA,0X4D,0X98,0X65,0X4B,0X22,0XC9,0X41,0X88,0X49,0XDF,0XFF,
0XDF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0X09,0X3A,0X6A,0X1A,0X97,0X65,0XB8,0X4D,
0X19,0X4E,0XF8,0X45,0XF8,0X45,0X19,0X4E,0XF9,0X4D,0XF9,0X4D,0XD8,0X45,0X39,0X4E,
0XF8,0X45,0X19,0X46,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X45,0X19,0X46,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0X19,0X46,0XF9,0X45,0XF9,0X4D,
0XD8,0X4D,0XF8,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF8,0X4D,0XF8,0X4D,0XD8,0X4D,
0XD8,0X55,0XF8,0X4D,0XF8,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF8,0X4D,0XF9,0X4D,0XF9,0X4D,0XFA,0X4D,0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XD9,0X45,0XFA,0X4D,0XF9,0X45,0X19,0X46,
0XD8,0X3D,0X19,0X4E,0XF9,0X4D,0XF9,0X4D,0XD9,0X45,0X3A,0X4E,0XD9,0X45,0XF9,0X4D,
0XFA,0X4D,0XB9,0X4D,0X98,0X65,0X4B,0X22,0XCA,0X41,0XC9,0X49,0XDF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XDF,0XFF,0XBF,0XFF,0X09,0X3A,0X6B,0X1A,0XF8,0X6D,0X1A,0X5E,0X1A,0X4E,
0X3A,0X4E,0X39,0X56,0XF9,0X4D,0XD9,0X4D,0X19,0X4E,0X19,0X46,0XF9,0X45,0X19,0X46,
0XF9,0X45,0XF9,0X45,0X1A,0X4E,0XD9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X45,0X19,0X46,0X19,0X46,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,
0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XFA,0X4D,0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD8,0X45,0X19,0X4E,0XF8,0X45,
0XF8,0X45,0XD9,0X45,0XD9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0X19,0X4E,0XD9,0X45,
0XFA,0X55,0XB9,0X5D,0X8C,0X12,0X2A,0X2A,0X09,0X32,0XFF,0XFF,0XDF,0XF7,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XDF,0XFF,0X09,0X42,0X4A,0X1A,0X97,0X65,0XF9,0X55,0XD9,0X45,0X1A,0X4E,
0X3A,0X56,0XF9,0X4D,0XF9,0X4D,0XB8,0X45,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XFA,0X4D,
0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,0XF8,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X19,0X46,0X19,0X46,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,
0X19,0X46,0XFA,0X45,0X19,0X56,0X19,0X56,0X19,0X56,0X19,0X56,0X19,0X56,0X19,0X56,
0X19,0X4E,0X1A,0X4E,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,
0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF8,0X45,0X18,0X46,0XF8,0X45,
0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0X19,0X4E,0XF9,0X4D,
0XFA,0X4D,0XD9,0X4D,0XFA,0X4D,0XD9,0X45,0XF8,0X45,0X18,0X46,0X19,0X46,0XF9,0X4D,
0XB9,0X4D,0XB8,0X55,0XF8,0X65,0X96,0X6D,0X35,0X85,0X09,0X32,0X09,0X3A,0XBB,0XD6,
0X9B,0XDE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,
0X9B,0XE6,0X09,0X42,0X4A,0X22,0X77,0X65,0XF9,0X55,0X19,0X4E,0XF9,0X4D,0XF8,0X4D,
0XF9,0X4D,0XF9,0X4D,0X7B,0X5E,0X5B,0X5E,0X3A,0X56,0XD8,0X4D,0X19,0X56,0X19,0X56,
0XF9,0X4D,0X3A,0X4E,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XFA,0X45,0XFA,0X45,
0XF9,0X4D,0XF8,0X4D,0XD9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XD9,0X4D,0XD9,0X4D,0XF9,0X45,0XF9,0X45,
0XF8,0X4D,0X19,0X56,0X19,0X56,0X19,0X56,0X19,0X56,0X19,0X56,0X19,0X56,0X19,0X4E,
0X19,0X4E,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0X18,0X46,0X19,0X46,0XF9,0X4D,
0XD9,0X4D,0XD9,0X45,0XF9,0X45,0X19,0X46,0XF9,0X45,0XF8,0X45,0XF8,0X4D,0XD9,0X4D,
0XF9,0X4D,0XB9,0X4D,0X1A,0X56,0XF8,0X45,0X18,0X46,0X19,0X46,0XF9,0X45,0XF9,0X4D,
0X19,0X4E,0XF8,0X45,0XD7,0X55,0X76,0X6D,0X6A,0X22,0X29,0X32,0XBB,0XD6,0X9B,0XDE,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDB,0XDE,
0X09,0X32,0X6B,0X1A,0XB8,0X65,0XD9,0X4D,0XF9,0X45,0XD8,0X4D,0XF9,0X4D,0X19,0X4E,
0XF9,0X4D,0XD8,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD8,0X4D,0X19,0X56,0X19,0X4E,
0X39,0X4E,0XF8,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XFA,0X45,0XF9,0X45,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF8,0X4D,
0X19,0X56,0X39,0X56,0X5A,0X5E,0X5A,0X5E,0X3A,0X56,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X19,0X4E,0X1A,0X4E,0X19,0X4E,0XF9,0X45,0XF9,0X4D,
0XD9,0X4D,0X19,0X4E,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0X19,0X4E,0X19,0X56,0XF9,0X55,
0X1A,0X56,0X19,0X4E,0XF9,0X45,0X19,0X46,0X19,0X4E,0X19,0X4E,0X19,0X56,0XF8,0X55,
0X39,0X56,0X19,0X56,0XF9,0X5D,0XD8,0X6D,0X56,0X7D,0X49,0X32,0XE8,0X39,0XBB,0XDE,
0XBA,0XDE,0XDB,0XDE,0X9A,0XD6,0XBB,0XE6,0X9A,0XDE,0XDB,0XD6,0X6A,0X32,0X56,0X75,
0X98,0X65,0XD9,0X55,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XF9,0X4D,0X3A,0X56,0X3A,0X56,0X19,0X4E,
0X19,0X4E,0X19,0X4E,0X19,0X56,0X1A,0X56,0X1A,0X4E,0X1A,0X4E,0X19,0X4E,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0X19,0X56,0X39,0X56,0X19,0X56,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,
0X19,0X4E,0XF9,0X45,0X19,0X46,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,
0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0X19,0X4E,0X19,0X56,0X19,0X56,0XF9,0X4D,
0XF9,0X4D,0XF9,0X45,0XF9,0X45,0X19,0X4E,0XF9,0X4D,0X19,0X5E,0X19,0X56,0XF9,0X4D,
0X19,0X56,0X19,0X56,0XD8,0X5D,0XB7,0X7D,0X29,0X2A,0X09,0X42,0X9B,0XDE,0XBA,0XDE,
0XBA,0XDE,0XBB,0XDE,0X9A,0XDE,0X9A,0XDE,0XFC,0XD6,0X29,0X22,0X97,0X6D,0XD8,0X55,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XD9,0X45,0XF9,0X4D,0X19,0X4E,0XF9,0X4D,0XF8,0X4D,0XF9,0X4D,
0XF9,0X4D,0X19,0X4E,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF8,0X4D,0XF8,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X19,0X4E,
0XF9,0X45,0X19,0X46,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X19,0X4E,
0XFA,0X4D,0X1A,0X4E,0X19,0X4E,0X19,0X4E,0X39,0X5E,0X39,0X5E,0XF9,0X4D,0XF9,0X4D,
0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0X19,0X56,0X39,0X5E,0X39,0X56,0X39,0X56,
0XD8,0X4D,0XD8,0X5D,0XB7,0X7D,0X49,0X2A,0XE8,0X39,0XC8,0X41,0XE8,0X41,0X08,0X42,
0XC8,0X41,0XE8,0X49,0XC7,0X41,0X29,0X42,0X6A,0X22,0X97,0X6D,0XD8,0X55,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF8,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X19,0X4E,0XF9,0X45,
0X19,0X46,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X19,0X5E,0X19,0X5E,0XD9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X45,0XF9,0X4D,0XF8,0X4D,0XF8,0X55,0X39,0X5E,0X19,0X56,0XD8,0X4D,0XF9,0X4D,
0XF9,0X65,0XD7,0X7D,0X29,0X2A,0XE8,0X39,0XC7,0X41,0XE7,0X41,0XC7,0X39,0XC8,0X41,
0XC8,0X49,0XC7,0X41,0X28,0X3A,0X4A,0X22,0X76,0X65,0XD8,0X55,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,};
#line 10 "app\\game.c"
#line 1 ".\\app\\RES\\Pic\\flappybird_getstart.c"
const unsigned char gImage_flappybird_getstart[9030] = {  
0XF8,0X45,0XF8,0X45,0XF9,0X4D,0X98,0X55,0X38,0X75,0X0C,0X2A,0X0A,0X42,0XE9,0X49,
0XE9,0X49,0XC9,0X41,0XCA,0X49,0XCA,0X41,0XCA,0X41,0X0A,0X42,0XCA,0X41,0XEA,0X41,
0XEA,0X49,0X0A,0X32,0X57,0X65,0XD9,0X55,0XD9,0X45,0XF9,0X45,0X1A,0X46,0XF9,0X45,
0X19,0X46,0XF9,0X45,0XD9,0X45,0X1A,0X4E,0XD9,0X45,0XD9,0X55,0X77,0X65,0X4A,0X22,
0X0A,0X3A,0XC9,0X41,0XC9,0X41,0XE9,0X49,0XC9,0X49,0XC9,0X49,0XEA,0X49,0XCA,0X49,
0XE9,0X49,0X09,0X32,0X77,0X6D,0XB8,0X4D,0XD8,0X45,0X19,0X4E,0XFA,0X45,0X98,0X5D,
0X2A,0X32,0XC8,0X49,0XC9,0X49,0XEA,0X49,0XCA,0X49,0XCA,0X49,0XC9,0X49,0XEA,0X49,
0XCA,0X49,0XCA,0X49,0XE9,0X41,0X09,0X42,0XE9,0X41,0X2B,0X2A,0X78,0X5D,0XFA,0X55,
0XFA,0X4D,0XF9,0X45,0XD9,0X45,0X1A,0X4E,0XD8,0X4D,0XD8,0X4D,0X1A,0X4E,0XD9,0X45,
0XFB,0X45,0XFA,0X45,0XFA,0X45,0X1A,0X4E,0XD9,0X4D,0XFA,0X4D,0XF9,0X45,0X19,0X46,
0XD9,0X45,0XF9,0X45,0XF9,0X45,0X19,0X46,0X19,0X46,0XF9,0X4D,0XD9,0X4D,0XFA,0X4D,
0XF9,0X4D,0XD9,0X45,0X19,0X4E,0XF9,0X45,0XF8,0X45,0XF8,0X4D,0XD9,0X4D,0XD9,0X4D,
0XB8,0X55,0X77,0X6D,0X0B,0X32,0XEA,0X41,0XE9,0X41,0X0A,0X42,0XAA,0X49,0XAA,0X49,
0XC9,0X49,0XE9,0X49,0XE9,0X49,0X2A,0X32,0X77,0X65,0XB8,0X55,0XD8,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XD7,0X4D,0XD7,0X55,0X76,0X6D,0X4B,0X22,
0X0A,0X3A,0XEA,0X49,0XCA,0X49,0XEA,0X49,0XC9,0X41,0XE9,0X49,0XC9,0X49,0XC9,0X49,
0XC9,0X49,0XF8,0X4D,0XF8,0X4D,0X97,0X55,0X98,0X65,0X38,0X7D,0X2B,0X32,0X09,0X3A,
0X08,0X3A,0X08,0X42,0X08,0X42,0X09,0X42,0X09,0X42,0X29,0X42,0X08,0X3A,0X09,0X42,
0XE8,0X39,0XE9,0X41,0X09,0X32,0X77,0X75,0X98,0X5D,0XB9,0X55,0XFA,0X55,0XD9,0X4D,
0XF9,0X4D,0XD8,0X4D,0XF9,0X4D,0X19,0X56,0XD9,0X4D,0XD9,0X4D,0XB8,0X55,0X77,0X6D,
0X4A,0X22,0X09,0X3A,0X09,0X4A,0X08,0X42,0X07,0X3A,0X49,0X4A,0XE8,0X41,0XC9,0X39,
0XE9,0X41,0XC8,0X41,0X2A,0X3A,0X36,0X65,0XD9,0X55,0XF9,0X4D,0XF9,0X45,0X1A,0X46,
0X98,0X5D,0X09,0X32,0XE8,0X51,0X09,0X4A,0XC8,0X39,0X09,0X42,0X29,0X42,0X08,0X3A,
0X08,0X3A,0XE8,0X41,0X29,0X42,0X28,0X3A,0X48,0X42,0X09,0X42,0X2A,0X32,0X37,0X6D,
0X98,0X5D,0XD8,0X4D,0XF9,0X4D,0XFA,0X4D,0XD9,0X4D,0XF9,0X55,0XD8,0X55,0XB8,0X4D,
0XF9,0X55,0XB9,0X45,0XFA,0X4D,0XF9,0X4D,0XB9,0X45,0XD9,0X4D,0XD9,0X4D,0X1A,0X4E,
0XF9,0X45,0X19,0X4E,0XF9,0X45,0X1A,0X46,0XF9,0X45,0X19,0X4E,0XF8,0X45,0XD9,0X45,
0XFA,0X4D,0XD9,0X45,0X19,0X4E,0XF9,0X45,0X19,0X46,0XF8,0X45,0X39,0X4E,0XF9,0X4D,
0XD9,0X4D,0XB8,0X55,0X98,0X6D,0X2A,0X32,0XE9,0X41,0X08,0X3A,0X08,0X3A,0XE9,0X39,
0X2A,0X4A,0X08,0X42,0XE8,0X41,0X09,0X4A,0X09,0X32,0X98,0X6D,0XB8,0X55,0XD8,0X4D,
0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF8,0X4D,0XB7,0X55,0X77,0X6D,
0X6B,0X2A,0X09,0X3A,0XE9,0X41,0XE8,0X39,0X09,0X42,0X07,0X42,0X08,0X42,0XE9,0X41,
0XE9,0X49,0XC9,0X49,0XB8,0X4D,0XB7,0X55,0XB7,0X6D,0X6A,0X1A,0X09,0X2A,0XFF,0XF7,
0XFE,0XEF,0XFD,0XEF,0XFD,0XEF,0XFC,0XEF,0XFD,0XEF,0XFC,0XEF,0XFC,0XEF,0XFC,0XEF,
0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0X49,0X3A,0X16,0X75,0X36,0X65,0X78,0X65,0X98,0X65,
0X77,0X5D,0X97,0X5D,0X98,0X65,0X77,0X55,0XD9,0X5D,0XD9,0X4D,0XD9,0X4D,0XB8,0X55,
0X56,0X6D,0X6A,0X2A,0XFF,0XFF,0XFF,0XFF,0XFE,0XEF,0XFD,0XEF,0XFC,0XE7,0XFD,0XEF,
0XFF,0XF7,0XFF,0XFF,0XDF,0XFF,0XE9,0X39,0X57,0X6D,0X77,0X55,0XB8,0X4D,0XF9,0X4D,
0XD9,0X45,0X98,0X5D,0X0A,0X3A,0XA7,0X49,0XFE,0XFF,0XFE,0XF7,0XFE,0XEF,0XFD,0XE7,
0XFD,0XE7,0XFD,0XE7,0XFD,0XEF,0XFD,0XEF,0XFD,0XEF,0XFC,0XEF,0XFE,0XF7,0XFF,0XF7,
0X6B,0X32,0X16,0X75,0X97,0X5D,0XD8,0X55,0XB8,0X4D,0XD9,0X55,0XB8,0X5D,0X77,0X5D,
0X98,0X65,0X98,0X5D,0XB8,0X5D,0XB9,0X5D,0XB8,0X55,0XF9,0X55,0XF9,0X55,0XD9,0X4D,
0XD8,0X4D,0XF9,0X4D,0XF9,0X4D,0XD8,0X45,0XF9,0X4D,0XF9,0X4D,0XD8,0X45,0XF9,0X4D,
0XFA,0X4D,0XFA,0X4D,0XD9,0X45,0XFA,0X4D,0XF9,0X45,0XD9,0X45,0X19,0X4E,0XD8,0X45,
0XF9,0X4D,0XD8,0X4D,0X97,0X55,0X76,0X6D,0XFF,0XEF,0XFF,0XFF,0XFD,0XEF,0XFD,0XE7,
0XFE,0XEF,0XFE,0XEF,0XFE,0XEF,0XFE,0XFF,0XDF,0XFF,0XE9,0X31,0X98,0X6D,0XB8,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X1A,0X4E,0XD9,0X4D,0XB8,0X45,0XD8,0X4D,0XB8,0X5D,
0X57,0X75,0X2A,0X2A,0XFF,0XF7,0XFE,0XF7,0XFE,0XEF,0XFD,0XE7,0XFD,0XEF,0XFC,0XEF,
0XFF,0XFF,0XBF,0XFF,0XA8,0X49,0X97,0X5D,0X97,0X6D,0X6B,0X22,0XFF,0XE7,0XFF,0XF7,
0XFE,0XF7,0X90,0X85,0XF0,0X85,0XCF,0X7D,0XEF,0X85,0XEE,0X7D,0XEE,0X7D,0X0F,0X86,
0XAE,0X7D,0X90,0X8D,0XFC,0XEF,0XFF,0XFF,0XE8,0X39,0X2A,0X2A,0X4B,0X22,0X4C,0X22,
0X4B,0X1A,0X6B,0X1A,0X8B,0X1A,0X6B,0X1A,0XAC,0X1A,0X8C,0X0A,0X98,0X5D,0XB8,0X5D,
0X57,0X55,0X56,0X75,0X4A,0X22,0XFF,0XF7,0XFE,0XEF,0XD1,0X85,0XCF,0X75,0XEF,0X7D,
0XEF,0X85,0XB1,0X8D,0XFD,0XEF,0XBE,0XFF,0XE9,0X39,0X36,0X6D,0XB8,0X5D,0XD8,0X55,
0XD9,0X4D,0XD9,0X45,0X98,0X5D,0X0A,0X3A,0XA7,0X41,0XFE,0XF7,0X91,0X85,0XD0,0X85,
0XEF,0X7D,0X10,0X7E,0XEF,0X7D,0XCF,0X7D,0XCF,0X85,0XAF,0X7D,0XD0,0X8D,0XFC,0XEF,
0XFE,0XF7,0XDF,0XF7,0X4B,0X32,0X8B,0X12,0X96,0X65,0X97,0X5D,0X98,0X65,0X8C,0X12,
0X8C,0X1A,0X6A,0X1A,0X8B,0X1A,0XAB,0X1A,0X6B,0X12,0XAC,0X0A,0X97,0X5D,0XD8,0X55,
0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XF9,0X45,0XF9,0X45,0XF9,0X4D,0XD9,0X4D,0X1A,0X4E,
0XD9,0X45,0X1A,0X4E,0XFA,0X45,0XF9,0X45,0XF9,0X4D,0XD9,0X45,0X1A,0X4E,0X1A,0X4E,
0XF9,0X45,0XF9,0X4D,0XB8,0X4D,0XB8,0X55,0X96,0X6D,0XFF,0XEF,0XFD,0XEF,0XB0,0X85,
0XCE,0X7D,0XF0,0X7D,0XD0,0X7D,0XB0,0X85,0XFC,0XE7,0XFF,0XFF,0X2A,0X3A,0X77,0X65,
0XB8,0X4D,0XB8,0X45,0XF9,0X4D,0XF9,0X45,0XF9,0X45,0XFA,0X4D,0XFA,0X4D,0XD9,0X4D,
0X77,0X55,0X36,0X75,0X4A,0X2A,0XFE,0XEF,0X91,0X8D,0XD0,0X85,0XEF,0X7D,0XEF,0X7D,
0XD0,0X8D,0XFD,0XF7,0XDF,0XFF,0X87,0X49,0X36,0X6D,0X36,0X75,0X29,0X2A,0XFF,0XF7,
0XFE,0XF7,0XFB,0XE7,0X2E,0X7E,0X4D,0X6E,0X4D,0X6E,0X6C,0X6E,0X6C,0X6E,0X6C,0X6E,
0X4C,0X6E,0X6E,0X76,0X2F,0X7E,0XFA,0XD7,0XFF,0XFF,0XE9,0X41,0XEA,0X41,0X2B,0X42,
0X0A,0X3A,0X2A,0X3A,0X29,0X3A,0X29,0X3A,0X09,0X3A,0X2A,0X32,0X4B,0X2A,0X37,0X75,
0X57,0X65,0X57,0X65,0X36,0X6D,0X2A,0X22,0XFF,0XEF,0XFD,0XEF,0X0E,0X76,0X6C,0X66,
0X6B,0X66,0X6D,0X76,0XEE,0X7D,0XFC,0XE7,0XFF,0XFF,0XE9,0X39,0X36,0X6D,0X97,0X55,
0XB7,0X4D,0XB7,0X45,0XF8,0X4D,0X56,0X5D,0XEA,0X39,0XC7,0X49,0XFC,0XE7,0XEF,0X7D,
0X4D,0X76,0X4C,0X66,0X6D,0X6E,0X6D,0X6E,0X6C,0X6E,0X6C,0X76,0X4C,0X6E,0X0D,0X76,
0XF9,0XD7,0XFC,0XE7,0XFF,0XF7,0XE8,0X31,0X4A,0X2A,0X35,0X7D,0X17,0X6D,0X16,0X75,
0X4B,0X2A,0X29,0X32,0X49,0X32,0X49,0X3A,0X49,0X3A,0X29,0X32,0X4A,0X2A,0X77,0X75,
0XB8,0X55,0X19,0X4E,0XD9,0X45,0XF9,0X45,0X1A,0X46,0X19,0X46,0X1A,0X4E,0XFA,0X4D,
0XD9,0X45,0XF9,0X45,0XF9,0X3D,0X1A,0X46,0XF9,0X45,0XD9,0X45,0X1A,0X4E,0XFA,0X45,
0XD9,0X45,0XF9,0X4D,0XD9,0X45,0XD9,0X4D,0X98,0X55,0X56,0X65,0XFF,0XE7,0XFB,0XDF,
0X4E,0X76,0X8D,0X6E,0X4C,0X66,0X6E,0X76,0X2F,0X7E,0XFA,0XD7,0XFE,0XFF,0X09,0X32,
0X76,0X6D,0XB8,0X4D,0XD9,0X4D,0XF9,0X45,0X19,0X46,0XF8,0X45,0X19,0X46,0XD8,0X45,
0XD8,0X4D,0XB8,0X5D,0X56,0X7D,0X69,0X2A,0XFD,0XE7,0XF0,0X7D,0X6E,0X76,0X6D,0X66,
0X6D,0X6E,0X0E,0X7E,0XFC,0XEF,0XDE,0XFF,0XA7,0X49,0X4B,0X2A,0X2A,0X2A,0XFF,0XF7,
0XFD,0XEF,0XFA,0XDF,0XEE,0X7D,0X8D,0X6E,0X8B,0X5E,0X8B,0X5E,0XAC,0X5E,0X8B,0X5E,
0XAB,0X5E,0XCC,0X5E,0X6C,0X5E,0X4E,0X6E,0XFA,0XD7,0XFE,0XF7,0XE9,0X49,0XE9,0X49,
0XE8,0X41,0XFF,0XFF,0XFD,0XF7,0XFC,0XEF,0XFC,0XEF,0XFE,0XF7,0XFE,0XEF,0XFF,0XF7,
0X4A,0X2A,0X6B,0X1A,0XF5,0X6C,0XF5,0X6C,0X6A,0X2A,0XFF,0XF7,0XFC,0XE7,0X2D,0X66,
0XCC,0X5E,0XCB,0X5E,0X6B,0X66,0XED,0X75,0XFB,0XE7,0XFE,0XFF,0XE9,0X39,0XF5,0X64,
0X76,0X5D,0X97,0X55,0XD8,0X4D,0XD8,0X4D,0X36,0X55,0X0A,0X42,0XC7,0X41,0XFC,0XE7,
0X0E,0X6E,0X6C,0X66,0XEC,0X6E,0X6A,0X5E,0XAB,0X5E,0XAB,0X5E,0XAB,0X5E,0X8B,0X5E,
0X8C,0X66,0X4D,0X6E,0XEF,0X7D,0XFD,0XEF,0XFE,0XF7,0XFF,0XF7,0X2A,0X32,0X2B,0X2A,
0X4B,0X32,0XFF,0XEF,0XFE,0XEF,0XFD,0XEF,0XFD,0XEF,0XFD,0XEF,0XFE,0XEF,0XFF,0XF7,
0X4A,0X22,0X77,0X5D,0XD9,0X55,0XB8,0X4D,0XF9,0X4D,0XF9,0X55,0XB8,0X55,0XB8,0X55,
0XB8,0X5D,0XB8,0X5D,0XD9,0X5D,0XB8,0X55,0XB8,0X55,0XB8,0X5D,0XB8,0X5D,0X98,0X5D,
0XB8,0X5D,0XB9,0X5D,0XB8,0X5D,0X98,0X55,0X98,0X55,0X77,0X55,0X56,0X65,0XFF,0XE7,
0XFB,0XDF,0X4D,0X6E,0XAB,0X5E,0XAB,0X5E,0X6C,0X66,0X4D,0X76,0XFA,0XD7,0XFE,0XFF,
0X49,0X42,0X14,0X6D,0X35,0X55,0X97,0X5D,0X97,0X5D,0XB7,0X55,0XB7,0X55,0XD8,0X5D,
0X97,0X5D,0X77,0X5D,0X56,0X65,0XD4,0X74,0X68,0X2A,0XFD,0XE7,0XEE,0X75,0X6C,0X66,
0XEC,0X66,0XAB,0X5E,0X2D,0X6E,0XFC,0XE7,0XFF,0XFF,0XA7,0X49,0XE9,0X31,0X29,0X32,
0XFE,0XEF,0X8F,0X85,0X4E,0X76,0X6C,0X66,0X8A,0X56,0XCB,0X5E,0XCC,0X56,0XAC,0X56,
0XAC,0X56,0XCC,0X56,0XCC,0X4E,0XAD,0X56,0X6F,0X66,0XFB,0XCF,0XFE,0XF7,0X08,0X4A,
0XDF,0XFF,0XFF,0XFF,0XFC,0XE7,0XEF,0X85,0X0D,0X6E,0X4E,0X76,0X0E,0X6E,0XCF,0X75,
0XFD,0XE7,0XFF,0XEF,0XFF,0XEF,0X73,0X6C,0X94,0X74,0X29,0X2A,0XFF,0XEF,0XFC,0XE7,
0X4E,0X6E,0X8B,0X56,0XAC,0X5E,0X8D,0X66,0X4F,0X6E,0XFB,0XD7,0XFE,0XF7,0XE9,0X39,
0XD4,0X74,0X15,0X65,0X57,0X5D,0XB8,0X5D,0XB7,0X45,0X36,0X55,0XEA,0X39,0XC6,0X41,
0XFC,0XE7,0X2D,0X6E,0X8B,0X5E,0XAA,0X5E,0XAB,0X5E,0XAB,0X5E,0XCC,0X56,0XAB,0X4E,
0XEC,0X56,0XCB,0X5E,0X8B,0X56,0X6E,0X6E,0XCF,0X75,0XFD,0XE7,0XFF,0XFF,0X09,0X42,
0XFF,0XFF,0XFE,0XEF,0XFD,0XE7,0XD0,0X7D,0X0E,0X76,0X0D,0X6E,0X0F,0X76,0XD0,0X7D,
0XFE,0XE7,0XFF,0XEF,0X6A,0X1A,0X56,0X6D,0X97,0X5D,0X97,0X5D,0X56,0X65,0XAB,0X22,
0X6B,0X2A,0X29,0X22,0X4A,0X2A,0X4A,0X2A,0X4A,0X2A,0X4A,0X22,0X6A,0X2A,0X29,0X22,
0X6A,0X32,0X4A,0X2A,0X4A,0X2A,0X4A,0X22,0XAB,0X22,0X35,0X6D,0XF5,0X64,0XD4,0X6C,
0XFF,0XEF,0XFB,0XDF,0X6D,0X6E,0XCA,0X56,0XAB,0X5E,0X8C,0X66,0X0D,0X6E,0XFA,0XDF,
0XFE,0XFF,0X08,0X42,0X48,0X32,0X69,0X2A,0X29,0X2A,0X2A,0X2A,0X4A,0X2A,0X29,0X22,
0X2A,0X2A,0X6B,0X2A,0X4A,0X2A,0X09,0X2A,0X6A,0X42,0X27,0X32,0XFC,0XE7,0X2F,0X86,
0X6B,0X66,0X8A,0X56,0XAB,0X5E,0X6E,0X6E,0XFC,0XE7,0XFF,0XFF,0XA7,0X49,0XE9,0X41,
0X08,0X3A,0XFD,0XEF,0XEF,0X7D,0X8D,0X66,0XAA,0X56,0XEB,0X5E,0XCB,0X56,0XC5,0X1C,
0XA6,0X1C,0XA6,0X1C,0XC6,0X1C,0XC6,0X14,0XA7,0X1C,0X69,0X34,0XFC,0XCF,0XFE,0XF7,
0XFF,0XFF,0XFF,0XFF,0XFD,0XEF,0XEF,0X85,0X2C,0X6E,0X8B,0X5E,0XAB,0X5E,0XAC,0X5E,
0X4D,0X66,0XF0,0X85,0XFC,0XE7,0XFF,0XEF,0X4A,0X32,0X4A,0X3A,0X29,0X3A,0XFF,0XF7,
0XFB,0XE7,0X2D,0X6E,0XAC,0X5E,0XAC,0X5E,0X8C,0X5E,0X4F,0X6E,0XFB,0XD7,0XFE,0XF7,
0XE8,0X39,0X2A,0X32,0X4B,0X22,0X8C,0X1A,0X36,0X5D,0X77,0X45,0X36,0X55,0X0A,0X42,
0XC6,0X41,0XFB,0XE7,0X4E,0X76,0X8B,0X5E,0XCA,0X5E,0XAB,0X5E,0X8B,0X5E,0XC5,0X1C,
0XCD,0X56,0XCB,0X4E,0XCB,0X56,0XEB,0X56,0X8C,0X5E,0X2E,0X76,0XFA,0XD7,0XFE,0XF7,
0XDF,0XFF,0XFE,0XFF,0XFC,0XEF,0XEF,0X85,0X2D,0X6E,0X6B,0X66,0XCC,0X66,0X6C,0X5E,
0X4E,0X6E,0XD0,0X85,0XFE,0XEF,0XFF,0XE7,0X8B,0X2A,0X6B,0X12,0X56,0X6D,0X6A,0X2A,
0XFF,0XEF,0XFF,0XEF,0XFF,0XF7,0XFE,0XF7,0XFE,0XEF,0XFF,0XF7,0XFE,0XEF,0XFE,0XEF,
0XFE,0XF7,0XFF,0XF7,0XDE,0XF7,0XFF,0XFF,0X29,0X3A,0X09,0X2A,0X4A,0X2A,0X4A,0X2A,
0X49,0X2A,0XFF,0XF7,0XFB,0XDF,0X6D,0X66,0XAB,0X56,0XAB,0X5E,0X6C,0X66,0X4E,0X76,
0XFA,0XD7,0XFE,0XF7,0XFF,0XFF,0XFE,0XF7,0XFE,0XEF,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,
0XFF,0XF7,0XFF,0XF7,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XF7,0XFD,0XF7,0XFB,0XEF,
0X0E,0X7E,0X6B,0X66,0XAB,0X5E,0XCB,0X5E,0X4D,0X6E,0XFD,0XE7,0XFF,0XFF,0XC8,0X49,
0XE9,0X41,0X08,0X3A,0XFD,0XEF,0XEF,0X7D,0X6D,0X66,0XAB,0X56,0XCB,0X5E,0X8C,0X5E,
0X88,0X2C,0X69,0X2C,0X69,0X34,0X69,0X34,0X49,0X2C,0X4A,0X34,0X2B,0X44,0XFD,0XDF,
0XFF,0XF7,0XFF,0XF7,0XFD,0XEF,0XFB,0XD7,0X2D,0X6E,0X8C,0X66,0X8A,0X56,0XCA,0X5E,
0XAB,0X5E,0X6C,0X5E,0X4E,0X76,0XFB,0XDF,0XFD,0XE7,0X48,0X3A,0X09,0X42,0XE9,0X41,
0XFF,0XFF,0XFB,0XE7,0X4D,0X76,0XAB,0X5E,0XAB,0X56,0XAC,0X5E,0X4E,0X6E,0XFB,0XD7,
0XFE,0XF7,0XE8,0X41,0XE9,0X41,0X2A,0X3A,0X6B,0X2A,0X16,0X65,0X77,0X4D,0X16,0X5D,
0XEA,0X41,0XC6,0X41,0XFB,0XE7,0X2E,0X76,0X8C,0X66,0XAB,0X5E,0XAB,0X5E,0X8C,0X5E,
0XA7,0X2C,0X8E,0X5E,0XCC,0X5E,0XAA,0X56,0XCA,0X56,0X8B,0X5E,0X2D,0X6E,0XFA,0XD7,
0XFD,0XF7,0XFE,0XF7,0XFD,0XEF,0XFA,0XD7,0X2D,0X6E,0XAC,0X6E,0X8B,0X5E,0XAB,0X5E,
0XAC,0X5E,0X6C,0X5E,0X0E,0X76,0XFC,0XDF,0XFF,0XF7,0X4A,0X3A,0X4A,0X2A,0XF5,0X84,
0X4A,0X3A,0XFF,0XF7,0XFE,0XF7,0XFD,0XEF,0XFC,0XE7,0XFC,0XE7,0XFC,0XDF,0XFC,0XDF,
0XFC,0XDF,0XFC,0XE7,0XFC,0XE7,0XFD,0XEF,0XFF,0XFF,0X29,0X4A,0XE9,0X39,0X0A,0X42,
0X09,0X42,0X29,0X42,0XFE,0XF7,0XFB,0XE7,0X4D,0X6E,0XAB,0X5E,0XAB,0X5E,0X6C,0X66,
0X2D,0X76,0XFA,0XD7,0XFD,0XEF,0XFD,0XEF,0XFD,0XE7,0XFC,0XE7,0XFD,0XDF,0XFE,0XE7,
0XFE,0XEF,0XFE,0XEF,0XFD,0XE7,0XFC,0XDF,0XFC,0XDF,0XFC,0XDF,0XFC,0XE7,0XFC,0XDF,
0XFB,0XDF,0X0E,0X76,0X8C,0X66,0XAB,0X5E,0XAB,0X5E,0X2E,0X6E,0XFC,0XE7,0XFF,0XFF,
0XA7,0X49,0XE9,0X41,0X08,0X3A,0XFD,0XEF,0XEF,0X7D,0X6D,0X66,0XAB,0X5E,0XCB,0X5E,
0X6D,0X5E,0XFC,0XCF,0XFE,0XD7,0XFD,0XD7,0XFD,0XDF,0XFE,0XDF,0XFE,0XDF,0XFE,0XE7,
0XFE,0XEF,0XFE,0XEF,0XFD,0XEF,0XD0,0X7D,0X4F,0X76,0X8C,0X66,0X8A,0X5E,0XEA,0X5E,
0XCA,0X56,0XCA,0X5E,0XCC,0X5E,0X6C,0X66,0X0E,0X6E,0XD1,0X8D,0XFD,0XEF,0XFF,0XFF,
0XFE,0XF7,0XFD,0XEF,0XF9,0XDF,0X4C,0X6E,0XAA,0X66,0XAA,0X5E,0XCC,0X5E,0X4D,0X66,
0XF9,0XCF,0XFC,0XE7,0XFD,0XF7,0XFF,0XFF,0XFF,0XFF,0XE9,0X21,0X16,0X6D,0X57,0X55,
0X16,0X5D,0XEA,0X41,0XC6,0X41,0XFC,0XE7,0X2E,0X76,0X8C,0X66,0XAB,0X5E,0XAB,0X5E,
0X6C,0X66,0XF9,0XBF,0X2F,0X6E,0X6C,0X5E,0XCB,0X5E,0XAA,0X5E,0XAB,0X66,0X4D,0X6E,
0XF9,0XCF,0XFC,0XE7,0XFC,0XEF,0XD0,0X7D,0X2E,0X76,0X8C,0X66,0X8A,0X5E,0XCB,0X5E,
0XAB,0X5E,0XCB,0X5E,0X8B,0X5E,0X6C,0X66,0X0E,0X7E,0XFD,0XEF,0XFF,0XF7,0XFF,0XF7,
0X29,0X3A,0XFF,0XF7,0XFE,0XF7,0XFC,0XE7,0XAE,0X7D,0X2F,0X7E,0X2E,0X76,0X2E,0X76,
0X4F,0X76,0X4F,0X76,0X0E,0X76,0X30,0X86,0X90,0X8D,0XFD,0XF7,0X08,0X4A,0XA8,0X41,
0XFF,0XFF,0XFE,0XFF,0XFC,0XEF,0XFD,0XEF,0XF9,0XCF,0X6C,0X66,0XCB,0X5E,0XAB,0X5E,
0XAC,0X66,0X4E,0X76,0XF9,0XCF,0XFB,0XDF,0XB1,0X8D,0XEF,0X85,0XEE,0X75,0X50,0X7E,
0XF0,0X75,0X91,0X85,0XFC,0XE7,0XAF,0X7D,0X0F,0X7E,0X2F,0X7E,0X0E,0X76,0X2F,0X76,
0X0F,0X76,0XFA,0XD7,0X2E,0X76,0X8C,0X66,0XAB,0X5E,0XAB,0X5E,0X2D,0X76,0XFC,0XE7,
0XFF,0XFF,0XA7,0X49,0XE8,0X49,0X07,0X42,0XFD,0XEF,0XCF,0X85,0X4D,0X6E,0X8B,0X5E,
0XCB,0X5E,0X4D,0X66,0XFC,0XDF,0XFF,0XEF,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFE,0XEF,
0XFF,0XF7,0XFE,0XF7,0XFC,0XE7,0XF0,0X85,0X4E,0X6E,0X6D,0X66,0X6A,0X5E,0XAA,0X5E,
0XCA,0X5E,0XAA,0X56,0XAA,0X56,0XCB,0X56,0XAB,0X56,0X6D,0X5E,0X0F,0X7E,0XFC,0XE7,
0XFC,0XE7,0XD0,0X8D,0X2E,0X76,0XAD,0X6E,0X8B,0X5E,0XAB,0X5E,0XCB,0X5E,0X8B,0X5E,
0X6B,0X5E,0X8C,0X6E,0X4D,0X6E,0XEE,0X7D,0XFC,0XE7,0XFF,0XEF,0X6A,0X2A,0XF5,0X6C,
0X36,0X4D,0X16,0X5D,0XEA,0X41,0XC7,0X41,0XFC,0XE7,0X0E,0X76,0X8C,0X66,0XAB,0X5E,
0XAA,0X5E,0X6B,0X66,0XFA,0XD7,0XEF,0X7D,0X6C,0X66,0XCB,0X5E,0XAA,0X5E,0X6A,0X5E,
0X6D,0X76,0XF8,0XC7,0XFB,0XDF,0XCF,0X7D,0X6E,0X76,0X8C,0X66,0X8A,0X5E,0XAB,0X5E,
0XAB,0X5E,0X8B,0X5E,0XAA,0X5E,0XAA,0X5E,0XCB,0X66,0X4C,0X66,0XCF,0X7D,0XFD,0XEF,
0XFF,0XF7,0XFF,0XEF,0XFD,0XE7,0XD0,0X85,0X2E,0X76,0X6D,0X6E,0X8D,0X66,0X8C,0X5E,
0X8C,0X66,0X6C,0X66,0X8C,0X66,0X4C,0X66,0X2D,0X6E,0XF0,0X85,0XFD,0XF7,0XFF,0XFF,
0XDF,0XFF,0XFE,0XF7,0XCF,0X7D,0X6E,0X76,0X4D,0X6E,0XAD,0X6E,0XCB,0X66,0XCA,0X5E,
0XCA,0X5E,0X6B,0X56,0X6D,0X6E,0XF9,0XC7,0XFB,0XDF,0XEF,0X7D,0X2C,0X6E,0X6B,0X66,
0X8C,0X66,0X4D,0X6E,0XAF,0X7D,0XFB,0XE7,0XEE,0X75,0X4D,0X6E,0X2C,0X66,0X8D,0X66,
0X6D,0X66,0X4D,0X6E,0XFA,0XD7,0X2E,0X76,0XAC,0X5E,0XCB,0X56,0XAB,0X5E,0X4D,0X6E,
0XFC,0XE7,0XFF,0XFF,0XA7,0X49,0XE8,0X41,0X07,0X3A,0XFD,0XEF,0XCF,0X85,0X4C,0X6E,
0X8B,0X5E,0XAB,0X5E,0X4D,0X6E,0XFD,0XEF,0XFE,0XFF,0XFD,0XF7,0XFD,0XEF,0XFD,0XEF,
0XFD,0XF7,0XFC,0XE7,0XFD,0XEF,0XFB,0XE7,0X0E,0X76,0X4C,0X66,0XCC,0X5E,0XCB,0X5E,
0XCB,0X5E,0X8B,0X5E,0XAB,0X5E,0XCB,0X5E,0XAB,0X56,0XEC,0X56,0X8B,0X56,0X0E,0X76,
0XFA,0XDF,0XFB,0XDF,0XEE,0X75,0X8C,0X66,0XAB,0X56,0XAB,0X5E,0XAB,0X5E,0XAB,0X5E,
0XAB,0X5E,0XCB,0X5E,0XAA,0X5E,0XAB,0X5E,0X4D,0X6E,0XFB,0XDF,0XFF,0XEF,0X29,0X22,
0X16,0X6D,0X36,0X4D,0X15,0X5D,0X0A,0X42,0XC7,0X41,0XFC,0XE7,0X0E,0X76,0X8C,0X66,
0XAA,0X5E,0XCA,0X5E,0X6B,0X66,0XF9,0XD7,0XAF,0X7D,0X8D,0X6E,0XAB,0X5E,0X8A,0X5E,
0X8B,0X66,0X4D,0X6E,0XF8,0XC7,0XFA,0XD7,0X0E,0X76,0X6B,0X5E,0XCB,0X5E,0XCB,0X5E,
0XAB,0X5E,0X8C,0X5E,0XAC,0X5E,0XAB,0X66,0X89,0X56,0XAA,0X56,0X8C,0X66,0X2E,0X7E,
0XFB,0XE7,0XFD,0XEF,0XFD,0XEF,0XFB,0XDF,0X0E,0X6E,0X8D,0X6E,0X8B,0X5E,0XAB,0X56,
0XCB,0X5E,0XCB,0X5E,0XAB,0X5E,0XAB,0X56,0XAC,0X66,0X4D,0X6E,0X0F,0X7E,0XFC,0XEF,
0XFD,0XF7,0XFE,0XEF,0XFB,0XDF,0X2C,0X66,0XAB,0X5E,0XCC,0X56,0XAB,0X56,0XAA,0X56,
0XCA,0X5E,0XAA,0X56,0XCB,0X5E,0X6D,0X66,0XF8,0XC7,0XFA,0XD7,0X2E,0X7E,0X6B,0X66,
0XCA,0X66,0X89,0X56,0X8C,0X6E,0XCE,0X7D,0XFA,0XDF,0X2E,0X76,0X6C,0X6E,0XAC,0X66,
0XAB,0X5E,0XAB,0X5E,0X6C,0X66,0XF9,0XCF,0X2E,0X76,0X8C,0X66,0XAB,0X5E,0XAB,0X5E,
0X4D,0X6E,0XFB,0XE7,0XFE,0XFF,0XA7,0X49,0XEA,0X41,0X08,0X3A,0XFD,0XEF,0XCF,0X7D,
0X6C,0X6E,0XAB,0X5E,0XCB,0X5E,0X6D,0X66,0XFC,0XE7,0XFC,0XDF,0X0F,0X7E,0X2E,0X6E,
0X4E,0X6E,0X0E,0X76,0XEF,0X85,0XFA,0XDF,0XFA,0XD7,0X2E,0X7E,0X8C,0X66,0XAB,0X56,
0XEB,0X56,0XAB,0X56,0X6D,0X66,0XA7,0X2C,0X8D,0X66,0X8C,0X5E,0XAB,0X56,0XAC,0X66,
0X2E,0X7E,0XF9,0XD7,0XFB,0XDF,0X0E,0X76,0X8C,0X66,0XCB,0X56,0XCB,0X5E,0XCB,0X5E,
0XCB,0X56,0XCB,0X5E,0XAA,0X5E,0XCA,0X5E,0XCA,0X56,0X8D,0X6E,0XFB,0XDF,0XFF,0XF7,
0X4A,0X2A,0XB4,0X64,0X36,0X4D,0X15,0X5D,0X09,0X42,0XC6,0X41,0XFC,0XE7,0X0E,0X76,
0X8C,0X66,0XAB,0X5E,0XCA,0X5E,0X6B,0X66,0XFA,0XDF,0XAF,0X7D,0X6D,0X66,0XAB,0X5E,
0XAA,0X5E,0XAB,0X66,0X4C,0X66,0XF8,0XC7,0XF9,0XCF,0X4E,0X76,0X8C,0X5E,0XAA,0X56,
0XEB,0X5E,0X8B,0X56,0X8D,0X5E,0XA6,0X24,0X8C,0X66,0XCB,0X66,0XEB,0X5E,0X8B,0X5E,
0X2F,0X7E,0XFB,0XDF,0XFD,0XEF,0XFB,0XDF,0X2D,0X6E,0XAC,0X66,0X6A,0X5E,0XAA,0X5E,
0XCC,0X5E,0XCC,0X5E,0XAB,0X56,0XAB,0X56,0XCC,0X5E,0X8B,0X56,0X6D,0X6E,0X0E,0X76,
0XFB,0XDF,0XFC,0XE7,0XFA,0XCF,0X2E,0X6E,0X8C,0X5E,0XCB,0X5E,0XAB,0X56,0XCC,0X56,
0XAB,0X5E,0XCB,0X66,0XAA,0X5E,0XAB,0X5E,0X4D,0X66,0XF9,0XC7,0XFA,0XD7,0XEE,0X75,
0X8C,0X66,0XCB,0X5E,0XCB,0X5E,0X8C,0X66,0XAE,0X7D,0XFB,0XDF,0X0D,0X6E,0X6C,0X66,
0X8B,0X5E,0XAB,0X5E,0XAB,0X5E,0X6B,0X66,0XF9,0XCF,0X2D,0X76,0X8C,0X66,0XAB,0X5E,
0XAB,0X5E,0X4D,0X6E,0XFB,0XE7,0XFE,0XFF,0XA7,0X49,0XEA,0X41,0X08,0X3A,0XFD,0XE7,
0XEF,0X7D,0X6C,0X66,0XAB,0X5E,0XAB,0X5E,0X4D,0X66,0XFB,0XE7,0XFA,0XD7,0X2D,0X66,
0XAC,0X5E,0XCC,0X5E,0X6C,0X5E,0X4D,0X76,0XF8,0XCF,0XFA,0XDF,0X2E,0X76,0X8C,0X66,
0XAA,0X56,0XAA,0X56,0XAC,0X5E,0X2E,0X6E,0XFA,0XCF,0X2D,0X66,0X8C,0X66,0XCC,0X5E,
0X8B,0X66,0X4E,0X7E,0XFA,0XDF,0XFB,0XDF,0X0E,0X76,0X8D,0X5E,0XAB,0X56,0XCB,0X56,
0XAB,0X56,0XAA,0X56,0XEB,0X56,0XCB,0X5E,0XCB,0X5E,0XCA,0X56,0X6D,0X66,0XFC,0XDF,
0XFF,0XEF,0X2A,0X2A,0XF5,0X6C,0X36,0X4D,0XF5,0X5C,0X09,0X42,0XC6,0X41,0XFC,0XE7,
0X2E,0X76,0X8C,0X66,0XCB,0X56,0XCA,0X56,0X8C,0X66,0XFA,0XD7,0XCF,0X7D,0X6D,0X66,
0XAB,0X56,0XCB,0X5E,0X6A,0X5E,0X4C,0X66,0XF8,0XC7,0XFA,0XD7,0X2E,0X76,0XAC,0X66,
0XAA,0X56,0XAB,0X56,0XAC,0X5E,0X4E,0X66,0XF9,0XBF,0X6C,0X5E,0XAB,0X5E,0XAA,0X56,
0X6B,0X5E,0X0E,0X76,0XFC,0XDF,0XFB,0XDF,0XEF,0X7D,0X8C,0X66,0XAA,0X5E,0XCB,0X66,
0XAA,0X5E,0X8B,0X56,0XAB,0X56,0XCB,0X5E,0XCB,0X56,0XCB,0X56,0XAC,0X5E,0X6D,0X6E,
0X0D,0X76,0XFB,0XDF,0XAE,0X75,0X4E,0X6E,0X6C,0X5E,0XCC,0X5E,0XAA,0X56,0XEC,0X56,
0XAB,0X56,0XAB,0X5E,0X8A,0X5E,0XCB,0X5E,0X8B,0X5E,0X4D,0X66,0XF9,0XCF,0XFA,0XD7,
0X0E,0X76,0X8C,0X66,0XAA,0X56,0XCB,0X56,0X4C,0X66,0XEF,0X7D,0XFA,0XD7,0X4D,0X76,
0X8C,0X66,0X8B,0X56,0XEB,0X5E,0XCB,0X5E,0X6C,0X66,0XF9,0XCF,0X2D,0X76,0X8C,0X66,
0XAB,0X5E,0XAB,0X5E,0X4D,0X6E,0XFC,0XE7,0XFF,0XFF,0XA7,0X49,0XE9,0X41,0X08,0X3A,
0XFD,0XE7,0XEF,0X7D,0X8C,0X66,0XAB,0X5E,0XAB,0X5E,0X4D,0X6E,0XFB,0XE7,0XFB,0XE7,
0X6D,0X6E,0X8B,0X5E,0XAB,0X56,0X8C,0X5E,0X4E,0X6E,0XF9,0XCF,0XFA,0XD7,0X0E,0X76,
0X8C,0X6E,0XCB,0X5E,0XCB,0X5E,0X6C,0X5E,0XEE,0X75,0XFA,0XD7,0X0D,0X6E,0X6C,0X66,
0X8B,0X5E,0X8C,0X66,0X0D,0X6E,0XFA,0XD7,0XFB,0XD7,0X2F,0X6E,0X8E,0X5E,0XCC,0X56,
0XAB,0X56,0XCB,0X56,0XCA,0X56,0XEB,0X5E,0X8B,0X56,0XCC,0X56,0X0C,0X57,0X6D,0X5E,
0XFC,0XDF,0XFF,0XF7,0X49,0X2A,0XF4,0X64,0X35,0X4D,0XF4,0X54,0X09,0X42,0XC6,0X41,
0XFC,0XE7,0X0E,0X76,0X8C,0X66,0XCB,0X56,0XCA,0X56,0X8B,0X5E,0XF8,0XCF,0XEE,0X75,
0X8C,0X66,0XAB,0X5E,0X8A,0X56,0XAC,0X66,0X4D,0X6E,0XF8,0XC7,0XFA,0XD7,0X0F,0X76,
0XAC,0X66,0XCA,0X56,0XEC,0X5E,0X6C,0X66,0XEE,0X75,0XF9,0XCF,0X8D,0X66,0XAB,0X5E,
0XCC,0X5E,0X8C,0X66,0X0E,0X76,0XFB,0XDF,0XFB,0XDF,0X0E,0X76,0X6B,0X5E,0XCB,0X5E,
0XAB,0X5E,0X8B,0X5E,0XAC,0X56,0XCC,0X56,0XAB,0X56,0XCA,0X56,0XAA,0X56,0XAB,0X5E,
0X8C,0X66,0X4D,0X6E,0XF9,0XCF,0X2E,0X76,0X4C,0X66,0XCC,0X66,0XAB,0X5E,0XCB,0X5E,
0XCB,0X56,0XAC,0X56,0XCC,0X5E,0XAB,0X5E,0XAA,0X5E,0X8B,0X66,0X4D,0X6E,0XF9,0XCF,
0XFA,0XD7,0X2F,0X7E,0X8C,0X66,0XAB,0X56,0XAB,0X56,0X8C,0X66,0XEE,0X75,0XFA,0XD7,
0X0D,0X6E,0X6C,0X66,0XCC,0X66,0X8B,0X56,0X8B,0X5E,0X6C,0X66,0XF9,0XCF,0X2E,0X76,
0X8C,0X66,0XAB,0X5E,0XAB,0X5E,0X4E,0X6E,0XFC,0XE7,0XFF,0XFF,0XA7,0X49,0XC9,0X49,
0X08,0X3A,0XFD,0XEF,0XEF,0X7D,0X8C,0X66,0XAB,0X56,0XAB,0X5E,0X4D,0X6E,0XFC,0XE7,
0XFB,0XDF,0X2C,0X6E,0XCB,0X5E,0XCB,0X56,0X8C,0X5E,0X6E,0X6E,0XF9,0XCF,0XFB,0XD7,
0X0F,0X7E,0X4C,0X66,0XAB,0X5E,0XAB,0X56,0XAC,0X66,0XED,0X75,0XFA,0XD7,0X2D,0X6E,
0X8C,0X66,0XAB,0X5E,0X6B,0X66,0X2E,0X76,0XFB,0XD7,0XFC,0XD7,0X2A,0X44,0X47,0X2C,
0XA6,0X2C,0X8C,0X5E,0XAB,0X5E,0XCB,0X5E,0XAB,0X56,0X8C,0X5E,0XA6,0X24,0X66,0X24,
0X69,0X3C,0XFD,0XE7,0XFF,0XF7,0X49,0X2A,0XF4,0X64,0X55,0X4D,0X15,0X55,0XEA,0X41,
0XA6,0X41,0XFC,0XE7,0X2E,0X76,0X8C,0X5E,0XCB,0X56,0XCA,0X56,0XAB,0X5E,0X4C,0X66,
0X6D,0X6E,0X8B,0X5E,0XCB,0X5E,0XAB,0X5E,0X8C,0X66,0X4D,0X6E,0XF9,0XC7,0XFA,0XD7,
0X2F,0X7E,0X8B,0X5E,0XCA,0X56,0XAB,0X5E,0X8D,0X6E,0X0E,0X76,0XF9,0XCF,0X6C,0X66,
0XCC,0X5E,0XAB,0X5E,0X6C,0X66,0X0F,0X7E,0XFB,0XDF,0XFB,0XDF,0X0E,0X76,0X8C,0X66,
0XCB,0X5E,0X8A,0X56,0XAC,0X66,0XA6,0X24,0XA6,0X24,0XAC,0X5E,0XCA,0X5E,0XCA,0X5E,
0XAA,0X5E,0X8C,0X66,0X4C,0X6E,0XF9,0XCF,0X2D,0X6E,0X8C,0X66,0XAB,0X5E,0XCB,0X5E,
0XAB,0X5E,0X85,0X2C,0X85,0X24,0XAC,0X5E,0XAB,0X56,0XCB,0X5E,0X8B,0X66,0X4D,0X6E,
0XF8,0XCF,0XFA,0XDF,0X0E,0X76,0X8C,0X66,0XCB,0X56,0XCB,0X5E,0X6C,0X66,0XCE,0X75,
0XFB,0XDF,0X0D,0X6E,0X6D,0X6E,0X8B,0X5E,0XAB,0X5E,0XCC,0X5E,0X8C,0X66,0XF9,0XCF,
0X2E,0X6E,0X8D,0X66,0XAC,0X56,0XAC,0X56,0X4E,0X66,0XFD,0XE7,0XFF,0XFF,0XA8,0X49,
0XC9,0X49,0X08,0X42,0XFD,0XEF,0XEE,0X7D,0X8C,0X66,0XCB,0X5E,0XCB,0X5E,0X4D,0X6E,
0XFD,0XE7,0XFC,0XDF,0X4C,0X6E,0XCB,0X5E,0XCB,0X56,0X8C,0X5E,0X4D,0X6E,0XF8,0XCF,
0XFB,0XDF,0XEE,0X75,0X6D,0X66,0XAB,0X5E,0XAB,0X5E,0XAB,0X66,0X2D,0X6E,0XF9,0XCF,
0X6C,0X66,0X8B,0X5E,0XAB,0X56,0XAC,0X66,0X2E,0X6E,0XFC,0XDF,0XFE,0XEF,0XFE,0XF7,
0XFD,0XDF,0XFA,0XCF,0X4C,0X6E,0XAB,0X66,0XAB,0X5E,0XAC,0X5E,0X6D,0X6E,0XF9,0XCF,
0XFC,0XDF,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0X29,0X2A,0XF4,0X64,0X35,0X4D,0XF4,0X54,
0XEA,0X41,0XA7,0X41,0XFC,0XE7,0X2E,0X76,0XAC,0X5E,0XEB,0X56,0XCA,0X56,0XCA,0X5E,
0XAB,0X66,0XAB,0X66,0XCB,0X56,0XCA,0X56,0XAB,0X5E,0X6B,0X66,0X2E,0X6E,0XFA,0XC7,
0XFA,0XCF,0X2F,0X76,0X8C,0X5E,0XCB,0X5E,0X8B,0X5E,0X8C,0X66,0X4C,0X6E,0XF8,0XC7,
0X8C,0X5E,0XCC,0X5E,0X8B,0X56,0X6C,0X66,0XEF,0X7D,0XFB,0XDF,0XFB,0XDF,0X0E,0X76,
0X8B,0X66,0XAA,0X5E,0XCA,0X5E,0X6B,0X66,0XFA,0XCF,0XFA,0XCF,0X6C,0X66,0XCB,0X5E,
0XAB,0X56,0XCB,0X5E,0X8B,0X66,0X4C,0X6E,0XF9,0XCF,0X2D,0X6E,0X6B,0X5E,0XCB,0X5E,
0XCB,0X5E,0X6C,0X66,0XFB,0XD7,0XFA,0XCF,0X6C,0X66,0XCB,0X56,0XCB,0X5E,0X8C,0X66,
0X2D,0X76,0XF9,0XCF,0XF9,0XD7,0X0D,0X76,0X8C,0X66,0XAB,0X56,0XAB,0X5E,0X6C,0X66,
0XCF,0X7D,0XFA,0XDF,0X2D,0X76,0X6B,0X66,0X8B,0X66,0XCB,0X5E,0XAC,0X5E,0X8D,0X5E,
0XFA,0XC7,0X6F,0X6E,0X8D,0X56,0XAC,0X4E,0XEE,0X56,0X2F,0X5E,0XFE,0XE7,0XFF,0XFF,
0XA8,0X49,0XC9,0X49,0XE8,0X39,0XFC,0XE7,0X0F,0X7E,0XAC,0X66,0XAA,0X56,0XCB,0X5E,
0X4E,0X6E,0XFD,0XEF,0XFC,0XE7,0X6D,0X6E,0XAA,0X56,0XAA,0X56,0X8B,0X5E,0X4D,0X6E,
0XF9,0XCF,0XFA,0XD7,0X0E,0X76,0X8D,0X66,0XCB,0X5E,0XAA,0X56,0XAB,0X5E,0X2B,0X5E,
0X8D,0X6E,0X6B,0X5E,0XAB,0X5E,0XCB,0X56,0X8C,0X5E,0X4F,0X76,0XFC,0XE7,0XFF,0XFF,
0XE8,0X41,0XFF,0XF7,0XFC,0XE7,0X2D,0X6E,0X8B,0X66,0XAB,0X5E,0XAC,0X66,0X0D,0X6E,
0XFB,0XDF,0XFE,0XF7,0XA7,0X41,0XC8,0X49,0XE8,0X39,0X29,0X2A,0XF5,0X64,0X35,0X4D,
0X15,0X5D,0XEA,0X41,0XC6,0X41,0XFC,0XE7,0X2E,0X76,0XAC,0X5E,0XCB,0X56,0XCA,0X56,
0XCA,0X5E,0XAA,0X5E,0XCB,0X5E,0XCA,0X56,0XCB,0X56,0XAB,0X56,0XAE,0X66,0X68,0X34,
0XFA,0XC7,0XFB,0XD7,0X2E,0X76,0X8B,0X5E,0XAA,0X56,0XCC,0X66,0XAB,0X66,0X6B,0X5E,
0X8B,0X5E,0XAB,0X5E,0XCB,0X5E,0XCB,0X56,0XAD,0X66,0X0F,0X7E,0XFB,0XDF,0XFA,0XD7,
0X2E,0X76,0X8C,0X66,0XAA,0X5E,0XAA,0X5E,0X6C,0X6E,0XFC,0XE7,0XFC,0XE7,0X6D,0X6E,
0XAB,0X56,0XCB,0X56,0XAB,0X56,0X8B,0X66,0X2C,0X6E,0XF8,0XCF,0X4E,0X76,0XAD,0X66,
0XAB,0X56,0X8A,0X56,0X6D,0X6E,0XFD,0XEF,0XFC,0XE7,0X6D,0X6E,0XAA,0X56,0XCB,0X56,
0X8C,0X66,0X4E,0X76,0XF8,0XCF,0XF9,0XD7,0X2E,0X7E,0X8C,0X66,0XAB,0X56,0XAB,0X5E,
0X6C,0X66,0X0F,0X7E,0XFA,0XD7,0X2C,0X6E,0X8C,0X66,0X8B,0X5E,0X8A,0X56,0XAC,0X5E,
0X8D,0X66,0XFB,0XCF,0X69,0X34,0XA8,0X24,0XE8,0X24,0XC8,0X24,0X69,0X34,0XFE,0XE7,
0XFF,0XFF,0XA8,0X49,0XCA,0X49,0X08,0X42,0XFD,0XE7,0XEE,0X75,0X6B,0X66,0XAA,0X5E,
0XCB,0X5E,0X4D,0X6E,0XFD,0XE7,0XFB,0XDF,0X6D,0X6E,0XAA,0X56,0XCB,0X5E,0XAB,0X66,
0X4E,0X6E,0XF9,0XC7,0XFA,0XD7,0X0E,0X76,0X6C,0X66,0XCB,0X56,0XEA,0X5E,0XCA,0X5E,
0XAB,0X66,0X8B,0X5E,0XAB,0X56,0XEC,0X5E,0XCB,0X4E,0XAC,0X5E,0X2F,0X76,0XFC,0XE7,
0XFF,0XFF,0XC8,0X41,0XFF,0XF7,0XFC,0XE7,0X4D,0X76,0X8B,0X5E,0XCB,0X5E,0XAC,0X5E,
0X2D,0X6E,0XFA,0XD7,0XFE,0XFF,0XE8,0X49,0XC8,0X49,0XE8,0X41,0X4A,0X32,0XD4,0X64,
0X36,0X4D,0XF5,0X5C,0X09,0X42,0XC6,0X41,0XFC,0XE7,0X0E,0X76,0X6C,0X66,0XAB,0X5E,
0XCA,0X5E,0XCA,0X5E,0XAB,0X5E,0XAB,0X5E,0XCA,0X56,0XCB,0X56,0XAC,0X4E,0X6E,0X5E,
0X2A,0X3C,0XFC,0XD7,0XFB,0XD7,0X2F,0X76,0X8B,0X5E,0XCB,0X56,0XCB,0X5E,0XAB,0X5E,
0XCB,0X5E,0XAA,0X5E,0XAA,0X5E,0XCB,0X5E,0XEB,0X56,0X8B,0X56,0X0E,0X76,0XFB,0XDF,
0XFA,0XD7,0X0E,0X76,0X6C,0X5E,0XCC,0X5E,0XCA,0X56,0X4C,0X5E,0XFC,0XE7,0XFC,0XE7,
0X4D,0X66,0XEC,0X5E,0XCB,0X56,0X8B,0X5E,0X8C,0X6E,0X4D,0X76,0XF9,0XCF,0X0D,0X6E,
0X6C,0X5E,0XCB,0X5E,0XEB,0X5E,0X4D,0X66,0XFC,0XE7,0XFC,0XE7,0X4C,0X66,0XCA,0X56,
0XEB,0X5E,0X8B,0X5E,0X2D,0X76,0XF9,0XCF,0XFB,0XDF,0X0E,0X76,0X6C,0X5E,0XCB,0X56,
0XAA,0X56,0X8B,0X5E,0X4D,0X6E,0XF7,0XBF,0X8C,0X66,0X8B,0X5E,0XAB,0X5E,0XAB,0X5E,
0XCC,0X66,0X4D,0X66,0XFB,0XD7,0XAA,0X3B,0X2B,0X44,0X0A,0X34,0XE9,0X33,0XEA,0X43,
0XFE,0XEF,0XFF,0XFF,0XA8,0X49,0XCA,0X49,0XE8,0X39,0XFD,0XE7,0X0E,0X76,0X8B,0X66,
0XCA,0X5E,0XAB,0X5E,0X6D,0X6E,0XFC,0XE7,0XFB,0XDF,0X6D,0X6E,0XCB,0X56,0XCA,0X5E,
0X8B,0X5E,0X4E,0X6E,0XF9,0XCF,0XF9,0XD7,0X2E,0X76,0X8C,0X66,0XAA,0X56,0XCA,0X56,
0XAA,0X56,0XAB,0X5E,0XCB,0X5E,0XAC,0X56,0XCC,0X56,0XCC,0X4E,0X6C,0X56,0X50,0X7E,
0XFC,0XE7,0XFF,0XFF,0XE8,0X49,0XFF,0XFF,0XFB,0XE7,0X2C,0X6E,0XAB,0X5E,0XAB,0X5E,
0X6B,0X5E,0X4D,0X76,0XFA,0XDF,0XFE,0XF7,0XE8,0X49,0XC7,0X49,0X09,0X4A,0X4A,0X32,
0XF5,0X6C,0X16,0X4D,0XD5,0X5C,0X09,0X42,0XC6,0X41,0XFB,0XE7,0X2E,0X76,0X6C,0X66,
0XAB,0X5E,0XAB,0X5E,0XAB,0X5E,0X8B,0X56,0XCB,0X5E,0XAA,0X56,0XCC,0X5E,0XA7,0X24,
0X49,0X34,0XFE,0XDF,0XFD,0XE7,0XFB,0XD7,0X4F,0X7E,0X8C,0X5E,0XCB,0X56,0X8B,0X56,
0XAB,0X56,0XCA,0X56,0XEA,0X5E,0XCB,0X5E,0XAC,0X56,0XEC,0X4E,0XAD,0X56,0X0F,0X6E,
0XFB,0XD7,0XFA,0XD7,0X2E,0X76,0X8D,0X66,0XAB,0X56,0XCB,0X56,0X8D,0X66,0XFB,0XDF,
0XFC,0XE7,0X4D,0X66,0XAB,0X56,0XAB,0X56,0XAC,0X5E,0X8C,0X6E,0X2D,0X6E,0XF9,0XCF,
0X4E,0X76,0X8C,0X66,0XCB,0X5E,0XAA,0X56,0X6D,0X6E,0XFC,0XE7,0XFB,0XDF,0X6D,0X6E,
0XCB,0X56,0XCA,0X56,0XCB,0X5E,0X2D,0X6E,0XFA,0XCF,0XFB,0XD7,0XEE,0X6D,0XAD,0X66,
0XCB,0X56,0XCB,0X5E,0XCB,0X5E,0X8C,0X66,0X8D,0X66,0XAC,0X5E,0XCB,0X5E,0XAB,0X56,
0XCC,0X5E,0X8B,0X5E,0X4D,0X6E,0XFD,0XE7,0XFF,0XEF,0XFE,0XE7,0XFE,0XE7,0XFE,0XE7,
0XFD,0XEF,0XFE,0XF7,0XFF,0XFF,0XA8,0X49,0XCA,0X49,0XE8,0X39,0XFD,0XE7,0XEE,0X75,
0X6B,0X66,0XAA,0X5E,0XAB,0X5E,0X6C,0X6E,0XFB,0XDF,0XF9,0XD7,0X6C,0X66,0XCB,0X56,
0XCA,0X5E,0X8B,0X5E,0X6E,0X6E,0XF9,0XC7,0XFA,0XD7,0X0E,0X76,0X8C,0X66,0XCB,0X5E,
0XCB,0X56,0XAB,0X5E,0XAB,0X66,0X8B,0X5E,0XC6,0X1C,0XE8,0X1C,0X08,0X1D,0XC8,0X24,
0X29,0X34,0XFE,0XE7,0XFF,0XF7,0XE8,0X41,0XFE,0XF7,0XFC,0XE7,0X6D,0X6E,0XAB,0X5E,
0X8B,0X56,0X8C,0X66,0X0D,0X6E,0XFA,0XDF,0XFE,0XF7,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,
0X09,0X2A,0XD5,0X6C,0X16,0X55,0XF5,0X5C,0X09,0X3A,0XE6,0X39,0XFB,0XE7,0X2E,0X76,
0X6C,0X66,0XAB,0X5E,0XAB,0X5E,0XAB,0X5E,0XCC,0X5E,0XAB,0X5E,0XAB,0X56,0X6D,0X66,
0XFB,0XD7,0XFC,0XDF,0XFE,0XF7,0XFE,0XF7,0XFB,0XDF,0X0D,0X76,0X8C,0X66,0XAB,0X5E,
0XCC,0X5E,0XAC,0X56,0XCC,0X5E,0XAB,0X56,0XE7,0X1C,0XE8,0X1C,0XE8,0X14,0XC9,0X1C,
0X6A,0X34,0XFC,0XCF,0XFA,0XD7,0X0D,0X76,0X8C,0X66,0XCB,0X56,0XCA,0X56,0X6C,0X66,
0XFB,0XDF,0XFC,0XE7,0X4C,0X66,0XCB,0X5E,0XAB,0X5E,0XAC,0X5E,0X8D,0X66,0X4D,0X6E,
0XF9,0XCF,0X2D,0X6E,0X6B,0X66,0XAA,0X5E,0XCB,0X5E,0X4D,0X66,0XFC,0XE7,0XFB,0XDF,
0X4D,0X6E,0XAB,0X5E,0XCA,0X56,0XAB,0X5E,0X6E,0X6E,0XF9,0XC7,0XFB,0XCF,0X69,0X3C,
0X6D,0X5E,0XCB,0X5E,0XAA,0X5E,0X8A,0X56,0X8B,0X5E,0XAB,0X5E,0XCB,0X66,0X8A,0X56,
0XAB,0X5E,0XAB,0X5E,0XAB,0X5E,0X6E,0X6E,0XFD,0XE7,0XFE,0XF7,0XFF,0XF7,0XFF,0XF7,
0XFE,0XF7,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0XA8,0X49,0XC9,0X41,0X08,0X3A,0XFD,0XEF,
0X10,0X7E,0X8D,0X66,0XAB,0X5E,0XCB,0X5E,0X6B,0X66,0XF8,0XCF,0XF6,0XBF,0X8C,0X66,
0XCB,0X56,0XAA,0X56,0X8B,0X66,0X4E,0X76,0XF9,0XC7,0XFA,0XD7,0X0E,0X76,0X6B,0X5E,
0XAB,0X56,0XCC,0X5E,0X8B,0X56,0XAA,0X5E,0X8C,0X66,0X88,0X2C,0X49,0X24,0X49,0X2C,
0X2A,0X34,0XAA,0X3B,0XFF,0XE7,0XFF,0XFF,0X08,0X42,0XFF,0XFF,0XFA,0XD7,0X4E,0X6E,
0X8C,0X5E,0XAB,0X5E,0X8C,0X5E,0X4D,0X6E,0XF8,0XC7,0XFA,0XCF,0XFB,0XDF,0XFF,0XF7,
0XFF,0XFF,0X2A,0X32,0XB4,0X6C,0X36,0X55,0XF5,0X54,0X0A,0X3A,0XE6,0X39,0XFB,0XE7,
0X2E,0X76,0X6C,0X66,0XAB,0X5E,0XAB,0X5E,0XAB,0X5E,0X8C,0X66,0X4B,0X5E,0XAC,0X5E,
0X6D,0X66,0XF9,0XD7,0XFD,0XF7,0XFE,0XFF,0XFD,0XF7,0XFB,0XD7,0X2D,0X6E,0X8C,0X66,
0XAB,0X5E,0XAB,0X5E,0X8B,0X56,0XAC,0X5E,0XAD,0X5E,0X48,0X24,0X6A,0X2C,0X4A,0X24,
0X2A,0X2C,0XEB,0X3B,0XFD,0XD7,0XFB,0XD7,0X2E,0X76,0X8C,0X66,0XCB,0X5E,0XAA,0X56,
0X8C,0X66,0XF8,0XC7,0XF9,0XCF,0X6B,0X66,0XAA,0X5E,0XAB,0X5E,0X8B,0X5E,0X8D,0X66,
0X4D,0X6E,0XF9,0XCF,0X4E,0X76,0XAC,0X66,0XAA,0X5E,0XAB,0X56,0X6C,0X66,0XF9,0XCF,
0XF8,0XC7,0X6D,0X6E,0XAB,0X5E,0XCB,0X5E,0XAB,0X5E,0X6D,0X6E,0XF8,0XBF,0XFD,0XDF,
0X0A,0X3C,0X2F,0X6E,0X6D,0X5E,0X8C,0X56,0XCC,0X5E,0XEC,0X5E,0X8B,0X4E,0XAB,0X5E,
0XCB,0X66,0XCB,0X5E,0XAB,0X5E,0XAB,0X5E,0X4D,0X6E,0XFD,0XE7,0XFD,0XE7,0XFC,0XE7,
0XFC,0XE7,0XFB,0XE7,0XFC,0XE7,0XFE,0XF7,0XFF,0XFF,0XA7,0X49,0XE9,0X49,0XE7,0X39,
0XFE,0XEF,0XD0,0X75,0X4E,0X66,0XAD,0X5E,0X8B,0X5E,0XAB,0X5E,0X6C,0X66,0XAD,0X6E,
0XAB,0X56,0XCB,0X56,0XCA,0X5E,0X8B,0X66,0X2D,0X76,0XF9,0XCF,0XFB,0XD7,0X0E,0X66,
0XCE,0X66,0XAB,0X56,0XAC,0X5E,0XCB,0X5E,0XAA,0X56,0X4B,0X5E,0XFA,0XCF,0XFE,0XEF,
0XFE,0XEF,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0X08,0X42,0XFE,0XFF,0XFC,0XE7,
0X4F,0X6E,0XAD,0X5E,0XAB,0X5E,0XAB,0X5E,0XAB,0X5E,0X8C,0X5E,0X8D,0X5E,0X6F,0X76,
0XFC,0XDF,0XFF,0XF7,0X29,0X2A,0XF5,0X6C,0X35,0X4D,0X15,0X55,0X0A,0X3A,0XC7,0X41,
0XFC,0XE7,0X2E,0X76,0X6B,0X6E,0XAA,0X5E,0XCA,0X56,0X8B,0X5E,0X2D,0X76,0X2E,0X76,
0X8C,0X5E,0XAD,0X66,0XF7,0XBF,0XFA,0XDF,0XFD,0XF7,0XFD,0XF7,0XFC,0XD7,0X2E,0X66,
0XCD,0X66,0XCB,0X5E,0XAB,0X56,0XCB,0X5E,0XAB,0X5E,0X4D,0X66,0XFC,0XDF,0XFE,0XE7,
0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFE,0XEF,0XFC,0XD7,0X4F,0X76,0X8C,0X66,0X8A,0X56,
0XCB,0X5E,0XAB,0X5E,0X6C,0X5E,0XAD,0X66,0X8A,0X5E,0XAA,0X5E,0XAA,0X5E,0XAB,0X5E,
0X8C,0X66,0X2D,0X66,0XFB,0XCF,0X50,0X6E,0X6C,0X5E,0XAB,0X5E,0XAB,0X5E,0X8B,0X5E,
0X8C,0X6E,0X6C,0X66,0X8B,0X5E,0XEC,0X5E,0X8B,0X56,0XAC,0X66,0X4D,0X6E,0XF9,0XCF,
0XFD,0XEF,0XFE,0XEF,0XAA,0X3B,0X29,0X34,0XD0,0X5E,0XCE,0X4E,0XEE,0X4E,0X0E,0X57,
0XCC,0X56,0XAA,0X56,0XAB,0X5E,0X8A,0X5E,0XCB,0X5E,0X6C,0X66,0XFA,0XCF,0XF0,0X7D,
0X4F,0X76,0X4D,0X6E,0X2D,0X6E,0X2F,0X7E,0XFC,0XEF,0XFF,0XFF,0XA7,0X49,0XE8,0X41,
0X08,0X3A,0XFE,0XEF,0XCA,0X43,0X68,0X2C,0X8E,0X5E,0XAC,0X5E,0XAB,0X5E,0XCC,0X5E,
0X8B,0X56,0XEC,0X5E,0XAA,0X56,0XCB,0X5E,0X8B,0X66,0X2D,0X6E,0XFA,0XCF,0XFB,0XCF,
0X6A,0X3C,0X4E,0X5E,0XCD,0X56,0XAC,0X5E,0XAB,0X5E,0XCB,0X56,0X8D,0X66,0XFB,0XDF,
0XFF,0XF7,0X08,0X42,0XE9,0X39,0XE9,0X31,0X0A,0X3A,0XE9,0X39,0X08,0X3A,0XFE,0XF7,
0XFC,0XE7,0X48,0X34,0X6D,0X5E,0XAB,0X5E,0XAA,0X56,0XEB,0X56,0XEB,0X56,0XEB,0X56,
0X6C,0X5E,0XFC,0XDF,0XFF,0XF7,0X28,0X2A,0XF4,0X64,0X35,0X45,0X15,0X55,0X0A,0X42,
0XC7,0X41,0XFC,0XE7,0X2E,0X76,0X6B,0X6E,0XAA,0X5E,0XCA,0X56,0X8B,0X5E,0XF9,0XCF,
0XCF,0X75,0X8D,0X66,0XAC,0X5E,0X8D,0X66,0X0E,0X76,0XFC,0XEF,0XFD,0XEF,0XFD,0XD7,
0X6A,0X3C,0X6E,0X5E,0XCC,0X56,0XAB,0X56,0XAA,0X56,0XAC,0X5E,0X4E,0X6E,0XFC,0XE7,
0XFF,0XFF,0XE8,0X39,0XE8,0X41,0XE8,0X41,0XFF,0XF7,0XFD,0XDF,0X49,0X3C,0X6C,0X5E,
0XCB,0X5E,0XAB,0X56,0XCB,0X56,0XCC,0X56,0XAB,0X56,0XAA,0X5E,0XAA,0X5E,0XAB,0X5E,
0XCC,0X5E,0X6C,0X66,0X4E,0X6E,0XFB,0XCF,0X6A,0X44,0X67,0X24,0XAD,0X5E,0XAB,0X56,
0XCB,0X5E,0XAA,0X5E,0XCB,0X5E,0XAB,0X56,0XCB,0X56,0XAC,0X5E,0X8C,0X5E,0X4D,0X6E,
0XFA,0XCF,0XFD,0XEF,0XFF,0XF7,0XFF,0XEF,0XFD,0XD7,0X6A,0X34,0XA9,0X24,0XA9,0X24,
0XA7,0X1C,0XAD,0X5E,0XCB,0X5E,0XAB,0X5E,0XAA,0X5E,0XAA,0X5E,0X8C,0X66,0XFA,0XCF,
0X2F,0X76,0X8D,0X66,0XAB,0X5E,0X8B,0X5E,0X4D,0X6E,0XFC,0XE7,0XFE,0XFF,0XC7,0X49,
0X08,0X42,0X07,0X3A,0XFF,0XF7,0XFD,0XE7,0XFA,0XBF,0X67,0X24,0X8D,0X5E,0XAC,0X5E,
0XAB,0X5E,0XAB,0X56,0XAC,0X56,0XCB,0X56,0XCB,0X56,0X8C,0X5E,0X2E,0X6E,0XFA,0XD7,
0XFD,0XEF,0XFD,0XE7,0X69,0X24,0XC7,0X1C,0X8C,0X56,0XAC,0X5E,0XAB,0X56,0X6D,0X66,
0XFB,0XDF,0XFF,0XF7,0X6A,0X32,0X52,0X64,0X73,0X5C,0X93,0X5C,0X32,0X64,0X29,0X32,
0XFF,0XFF,0XFE,0XEF,0XFC,0XD7,0X67,0X2C,0XAD,0X5E,0XCB,0X56,0XCB,0X56,0XCB,0X56,
0XEB,0X56,0X8D,0X66,0XFB,0XDF,0XFE,0XEF,0X48,0X2A,0XF4,0X64,0X35,0X4D,0X14,0X55,
0XE9,0X41,0XC6,0X41,0XFC,0XE7,0X0E,0X76,0X8C,0X66,0XAA,0X56,0X0B,0X57,0X8B,0X56,
0XFA,0XD7,0XCF,0X7D,0X6D,0X66,0XCC,0X5E,0XAB,0X5E,0X4C,0X66,0X10,0X7E,0XFC,0XD7,
0XFF,0XEF,0XFE,0XE7,0X69,0X2C,0XC6,0X1C,0XCC,0X5E,0XCC,0X56,0XAC,0X56,0X6E,0X6E,
0XFD,0XE7,0XFF,0XFF,0X29,0X3A,0XF1,0X73,0XE9,0X41,0XFF,0XFF,0XFF,0XEF,0XFC,0XD7,
0X87,0X24,0XAD,0X56,0XCC,0X56,0XCC,0X56,0XCC,0X5E,0XAB,0X5E,0XAA,0X5E,0XCB,0X5E,
0XAB,0X5E,0XCC,0X5E,0X6C,0X66,0X0E,0X6E,0XFD,0XE7,0XFC,0XDF,0XFA,0XBF,0X87,0X1C,
0XCC,0X56,0XCB,0X56,0XCB,0X56,0XCB,0X56,0XCB,0X56,0XCB,0X56,0XCC,0X56,0XAD,0X5E,
0X4E,0X6E,0XFA,0XD7,0XFE,0XF7,0XE7,0X39,0XFF,0XFF,0XFE,0XF7,0XFE,0XEF,0XFD,0XE7,
0XFD,0XE7,0XFC,0XDF,0X2D,0X66,0XAC,0X66,0XCB,0X5E,0XAA,0X56,0XAA,0X5E,0X8C,0X66,
0XF9,0XCF,0X2F,0X76,0X8D,0X5E,0XCC,0X5E,0X8B,0X56,0X4D,0X6E,0XFB,0XE7,0XFE,0XFF,
0XA7,0X49,0X49,0X2A,0X49,0X2A,0XDF,0XEF,0XFF,0XEF,0XFC,0XD7,0X29,0X34,0X6F,0X6E,
0X8E,0X66,0X8D,0X66,0XAD,0X66,0X8E,0X5E,0XAE,0X66,0X8D,0X5E,0X8E,0X66,0X2F,0X76,
0XFB,0XD7,0XFE,0XF7,0XFF,0XF7,0XEA,0X33,0X69,0X2C,0X8F,0X66,0X6D,0X5E,0XAD,0X66,
0X0E,0X6E,0XFC,0XDF,0XFF,0XEF,0X29,0X22,0X73,0X5C,0XB4,0X4C,0XD4,0X4C,0XB4,0X54,
0X4A,0X1A,0XFF,0XF7,0XFF,0XEF,0XFE,0XDF,0X09,0X34,0X2E,0X5E,0X8D,0X56,0XCD,0X5E,
0X8C,0X5E,0X8C,0X56,0X6E,0X6E,0XFC,0XDF,0XFF,0XF7,0X49,0X2A,0XF4,0X64,0X35,0X4D,
0X14,0X55,0X09,0X42,0XE7,0X49,0XFC,0XE7,0X0F,0X76,0X6E,0X66,0XAE,0X66,0XAC,0X56,
0X8E,0X66,0XFB,0XDF,0X90,0X7D,0X4F,0X6E,0XAE,0X5E,0XAD,0X5E,0X8E,0X66,0X0F,0X6E,
0XFC,0XD7,0XFF,0XF7,0XFF,0XF7,0XEA,0X3B,0X48,0X2C,0X6E,0X66,0XAE,0X5E,0X8E,0X5E,
0X50,0X6E,0XFD,0XE7,0XFF,0XF7,0X29,0X32,0X11,0X6C,0X2A,0X32,0XFF,0XF7,0XFF,0XF7,
0XFF,0XEF,0X6A,0X3C,0X2E,0X56,0X8E,0X5E,0XAE,0X5E,0X6D,0X5E,0XAD,0X66,0XAD,0X66,
0X8C,0X5E,0X8D,0X5E,0X6D,0X5E,0X6F,0X6E,0X10,0X7E,0XFD,0XEF,0XFF,0XF7,0XFD,0XD7,
0X4A,0X34,0X6E,0X66,0XAC,0X5E,0X8D,0X5E,0X8D,0X5E,0X8D,0X5E,0XAD,0X5E,0XAD,0X56,
0X6E,0X5E,0X30,0X76,0XFB,0XD7,0XFE,0XF7,0X08,0X4A,0XDF,0XFF,0XDE,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XF7,0XFC,0XE7,0X4E,0X76,0X8C,0X5E,0X8A,0X56,0XEB,0X5E,0XCB,0X5E,
0X8C,0X66,0XFA,0XD7,0X30,0X7E,0X4E,0X66,0X6D,0X5E,0XAE,0X66,0X4F,0X76,0XFC,0XE7,
0XFE,0XFF,0XA7,0X49,0X56,0X4D,0XF5,0X5C,0X4A,0X2A,0XFF,0XF7,0XFF,0XF7,0XFE,0XE7,
0XEA,0X3B,0X29,0X34,0X49,0X2C,0X69,0X2C,0X8A,0X34,0X49,0X2C,0X69,0X34,0X6A,0X3C,
0X2B,0X44,0XFD,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XEF,0XFD,0XDF,0X4A,0X3C,0X89,0X34,
0X48,0X34,0X4A,0X44,0XFD,0XE7,0XFF,0XF7,0X6A,0X32,0XD4,0X64,0XD5,0X4C,0XF5,0X44,
0XF5,0X4C,0XB4,0X54,0X6B,0X22,0X09,0X22,0XFF,0XF7,0XFE,0XE7,0X2A,0X34,0X49,0X24,
0X89,0X34,0X89,0X34,0X89,0X34,0X4A,0X44,0XFE,0XE7,0XFF,0XF7,0X09,0X32,0XD4,0X6C,
0X35,0X4D,0XF5,0X54,0XE9,0X41,0X87,0X41,0XFE,0XF7,0X2B,0X4C,0X4B,0X34,0X49,0X2C,
0X69,0X2C,0X49,0X3C,0XFC,0XDF,0XAB,0X53,0X2A,0X3C,0X49,0X34,0X49,0X2C,0X6A,0X34,
0X2B,0X44,0XFC,0XD7,0XFF,0XFF,0XFF,0XFF,0XFE,0XEF,0XFD,0XE7,0X4A,0X3C,0X69,0X2C,
0XAA,0X2C,0X4A,0X3C,0XFE,0XEF,0XFF,0XF7,0X2B,0X32,0X33,0X6C,0X52,0X5C,0X8B,0X2A,
0XE9,0X29,0XFF,0XF7,0XFE,0XDF,0X2C,0X44,0X6A,0X3C,0X49,0X2C,0XAA,0X34,0X69,0X2C,
0X69,0X34,0X89,0X34,0X8A,0X2C,0X8A,0X34,0X2A,0X34,0XEB,0X43,0XFF,0XFF,0XDF,0XFF,
0XFF,0XEF,0XFE,0XDF,0X4B,0X3C,0X69,0X34,0X69,0X34,0X69,0X34,0X69,0X34,0X69,0X34,
0X69,0X34,0X6A,0X34,0X2B,0X44,0XFC,0XDF,0XFF,0XFF,0XC8,0X49,0XC8,0X51,0X29,0X52,
0XFF,0XFF,0XFE,0XEF,0XFD,0XE7,0XFB,0XD7,0X2D,0X66,0X6B,0X5E,0XCC,0X66,0XAB,0X5E,
0X6A,0X56,0X6D,0X66,0XFA,0XCF,0X09,0X44,0X6A,0X3C,0X6A,0X34,0X49,0X34,0X0A,0X44,
0XFE,0XEF,0XFF,0XFF,0XA8,0X49,0X56,0X35,0X56,0X4D,0XB4,0X64,0X2A,0X32,0XE9,0X31,
0XFF,0XEF,0XFF,0XE7,0XFF,0XDF,0XFE,0XD7,0XFE,0XD7,0XFE,0XD7,0XFE,0XDF,0XFD,0XD7,
0XFE,0XDF,0XFF,0XDF,0XFF,0XEF,0XDF,0XF7,0X09,0X4A,0XBF,0XFF,0XFF,0XF7,0XFE,0XDF,
0XFD,0XD7,0XFD,0XD7,0XFE,0XDF,0XFF,0XF7,0XFF,0XEF,0X2A,0X2A,0XB4,0X64,0XD5,0X4C,
0X57,0X4D,0X36,0X45,0X15,0X4D,0XB4,0X54,0X73,0X64,0X08,0X32,0XFF,0XEF,0XFF,0XDF,
0XFE,0XD7,0XFD,0XD7,0XFE,0XD7,0XFE,0XD7,0XFF,0XE7,0XFF,0XF7,0XFF,0XF7,0X2A,0X32,
0XF5,0X6C,0X35,0X4D,0X15,0X55,0X2A,0X3A,0XC8,0X41,0XFF,0XF7,0XFE,0XE7,0XFF,0XDF,
0XFF,0XD7,0XFE,0XD7,0XFE,0XDF,0XFF,0XEF,0XFE,0XE7,0XFF,0XE7,0XFE,0XDF,0XFE,0XD7,
0XFE,0XDF,0XFE,0XDF,0XFF,0XEF,0XFF,0XFF,0XC8,0X41,0XFF,0XFF,0XFF,0XF7,0XFE,0XDF,
0XFE,0XD7,0XFE,0XD7,0XFE,0XDF,0XFF,0XEF,0XFF,0XF7,0X0B,0X2A,0X94,0X6C,0XD4,0X54,
0XB3,0X54,0X73,0X6C,0X29,0X32,0XFF,0XEF,0XFF,0XEF,0XFD,0XD7,0XFE,0XD7,0XFE,0XD7,
0XFE,0XD7,0XFE,0XD7,0XFE,0XD7,0XFE,0XCF,0XFE,0XD7,0XFF,0XDF,0XFF,0XE7,0XDF,0XF7,
0XE9,0X49,0XE9,0X41,0XFF,0XF7,0XFF,0XE7,0XFE,0XD7,0XFE,0XD7,0XFE,0XD7,0XFE,0XD7,
0XFE,0XDF,0XFE,0XD7,0XFE,0XDF,0XFE,0XE7,0XFF,0XF7,0XFF,0XFF,0XC8,0X49,0XC9,0X51,
0XC8,0X41,0XFE,0XEF,0XB2,0X85,0XEF,0X75,0X6E,0X76,0X6C,0X66,0XAC,0X66,0X8B,0X56,
0XCC,0X5E,0X8C,0X5E,0X6E,0X6E,0XFC,0XDF,0XFC,0XDF,0XFE,0XDF,0XFF,0XD7,0XFE,0XD7,
0XFF,0XE7,0XFF,0XF7,0XDF,0XFF,0XC8,0X49,0X57,0X45,0X36,0X4D,0XD3,0X4C,0XCB,0X22,
0X4A,0X2A,0XFF,0XF7,0XFF,0XF7,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XE7,0XFF,0XEF,
0XFF,0XEF,0XFF,0XF7,0XFF,0XEF,0XFF,0XEF,0XFF,0XFF,0X09,0X42,0XDF,0XF7,0XFF,0XF7,
0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XFF,0XFF,0XF7,0X8B,0X32,0XD4,0X64,
0X56,0X55,0X15,0X3D,0X56,0X45,0X15,0X3D,0X15,0X4D,0XB4,0X54,0X4A,0X2A,0XFF,0XF7,
0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XF7,0XFF,0XEF,0XFF,0XF7,0XFF,0XFF,0XFF,0XF7,
0X4B,0X2A,0XF5,0X64,0X36,0X4D,0XF5,0X4C,0X4A,0X22,0X29,0X32,0XFF,0XF7,0XFF,0XF7,
0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XF7,0XFF,0XF7,0XFF,0XEF,0XFF,0XEF,
0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XDF,0XFF,0X09,0X42,0XDF,0XFF,0XFF,0XFF,
0XFF,0XF7,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XF7,0XFF,0XF7,0X4B,0X2A,0X94,0X5C,
0XD4,0X4C,0X15,0X55,0XB4,0X54,0X6A,0X1A,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,
0XFF,0XEF,0XFF,0XE7,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XE7,0XFF,0XEF,0XFF,0XF7,
0XFF,0XFF,0X08,0X42,0XE9,0X41,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XEF,0XFF,0XEF,
0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XE9,0X41,
0XE9,0X49,0XE7,0X41,0XFE,0XEF,0XF0,0X7D,0X4D,0X6E,0X8C,0X66,0X8B,0X66,0X8B,0X5E,
0XCC,0X56,0XCD,0X56,0X8D,0X5E,0X0E,0X6E,0XFB,0XE7,0XFE,0XF7,0XFF,0XEF,0XFF,0XEF,
0XFF,0XEF,0XFF,0XEF,0XFF,0XFF,0XDF,0XFF,0XC9,0X41,0X57,0X45,0X77,0X4D,0X35,0X4D,
0XD3,0X4C,0XB4,0X5C,0X2A,0X1A,0X4A,0X2A,0X4A,0X2A,0X2A,0X2A,0X4A,0X2A,0X6A,0X2A,
0X4A,0X2A,0X4A,0X2A,0X2A,0X2A,0X4A,0X2A,0X4A,0X2A,0X4A,0X2A,0X2A,0X2A,0X6A,0X2A,
0X6A,0X2A,0X2A,0X2A,0X4A,0X2A,0X4A,0X2A,0X4A,0X2A,0X2A,0X2A,0X4A,0X2A,0X6B,0X1A,
0XD4,0X54,0X56,0X45,0X97,0X45,0X76,0X45,0X76,0X45,0X15,0X3D,0X15,0X55,0XB3,0X5C,
0X6A,0X22,0X2A,0X22,0X4A,0X2A,0X2A,0X2A,0X2A,0X2A,0X29,0X2A,0X4A,0X2A,0X2A,0X22,
0X4B,0X22,0XCC,0X1A,0XF5,0X54,0X36,0X45,0X35,0X4D,0XF4,0X5C,0XB4,0X64,0X4A,0X22,
0X4A,0X2A,0X49,0X22,0X49,0X2A,0X4A,0X2A,0X4A,0X2A,0X2A,0X2A,0X4A,0X2A,0X4A,0X2A,
0X49,0X2A,0X49,0X2A,0X4A,0X32,0X2A,0X2A,0X0A,0X2A,0X4A,0X32,0X4B,0X32,0X4A,0X2A,
0X4A,0X2A,0X09,0X22,0X29,0X2A,0X29,0X2A,0X69,0X2A,0X49,0X2A,0X4A,0X22,0X6C,0X1A,
0XD5,0X54,0X16,0X4D,0X15,0X3D,0X35,0X4D,0XD4,0X54,0X6A,0X22,0X4A,0X22,0X69,0X22,
0X49,0X22,0X4A,0X2A,0X2A,0X2A,0X4B,0X2A,0X2A,0X22,0X4A,0X2A,0X6A,0X2A,0X29,0X22,
0X4A,0X2A,0X49,0X2A,0X49,0X2A,0X29,0X32,0X0A,0X32,0X2A,0X32,0X4A,0X2A,0X4A,0X2A,
0X4A,0X2A,0X49,0X2A,0X49,0X2A,0X49,0X2A,0X49,0X2A,0X2A,0X2A,0X4A,0X2A,0X4A,0X2A,
0X4B,0X32,0XE9,0X41,0X08,0X42,0XFD,0XEF,0XEF,0X75,0X6C,0X66,0XAB,0X5E,0XAB,0X5E,
0XCC,0X5E,0XCC,0X56,0XCE,0X56,0X8F,0X66,0X29,0X3C,0XFD,0XEF,0X28,0X42,0X29,0X32,
0X4A,0X2A,0X4A,0X22,0X6A,0X2A,0X4A,0X2A,0X0A,0X2A,0X2A,0X32,0XB8,0X4D,0XB8,0X4D,
0X56,0X45,0X56,0X45,0X56,0X4D,0X15,0X4D,0XD4,0X54,0XD4,0X54,0XF4,0X4C,0XD4,0X4C,
0XD4,0X4C,0XD4,0X4C,0XF5,0X4C,0XD4,0X4C,0XD4,0X54,0XD4,0X54,0XD4,0X4C,0XB4,0X4C,
0XD4,0X54,0XB3,0X4C,0XD5,0X54,0XD5,0X4C,0XF4,0X4C,0XF4,0X4C,0X14,0X55,0X15,0X55,
0X35,0X4D,0X77,0X4D,0X97,0X45,0XB8,0X45,0XD8,0X4D,0X97,0X45,0X97,0X4D,0X36,0X45,
0X35,0X4D,0XF5,0X4C,0X15,0X55,0XF5,0X54,0XB4,0X54,0XD4,0X54,0XB3,0X4C,0X14,0X55,
0X36,0X55,0XF5,0X44,0X15,0X45,0X77,0X4D,0X97,0X4D,0X77,0X45,0X36,0X4D,0X16,0X4D,
0XF5,0X4C,0XF4,0X4C,0XF3,0X4C,0X14,0X55,0XD4,0X4C,0XD4,0X4C,0XD5,0X54,0XD4,0X54,
0XD4,0X54,0XD4,0X4C,0XD4,0X4C,0XD4,0X4C,0XD4,0X54,0XF5,0X54,0XD5,0X4C,0XD5,0X4C,
0XD4,0X4C,0XD4,0X4C,0XD5,0X54,0XF5,0X54,0XD4,0X4C,0XF4,0X4C,0X15,0X55,0X15,0X55,
0X36,0X4D,0X36,0X45,0X97,0X45,0X77,0X3D,0X56,0X4D,0X56,0X4D,0X15,0X55,0X15,0X55,
0XF4,0X4C,0XF4,0X4C,0XF4,0X4C,0XD4,0X4C,0XB4,0X54,0XD5,0X54,0XD4,0X54,0XD4,0X54,
0XD4,0X54,0XD4,0X54,0XF4,0X4C,0XD4,0X4C,0XD4,0X54,0XD4,0X54,0XD4,0X4C,0XD4,0X4C,
0XD4,0X54,0XD4,0X54,0XD4,0X4C,0XD4,0X4C,0XD4,0X54,0XD4,0X54,0XD4,0X4C,0XD4,0X4C,
0XD4,0X44,0X73,0X54,0X0B,0X32,0X49,0X42,0XFC,0XEF,0XEF,0X7D,0X6C,0X66,0XCB,0X5E,
0X8A,0X5E,0XAB,0X56,0XAC,0X56,0XA6,0X1C,0X69,0X34,0XFE,0XDF,0XFF,0XF7,0X29,0X32,
0X73,0X5C,0XB4,0X54,0XB3,0X4C,0XF4,0X4C,0XF5,0X54,0X16,0X55,0X15,0X55,0XD9,0X4D,
0XB8,0X45,0X77,0X45,0X77,0X3D,0X36,0X3D,0X56,0X3D,0X15,0X3D,0X15,0X3D,0X14,0X3D,
0X15,0X3D,0X15,0X3D,0X15,0X3D,0X15,0X3D,0X15,0X3D,0XF5,0X3C,0XF5,0X44,0X16,0X3D,
0X15,0X3D,0XF4,0X3C,0X15,0X3D,0XF5,0X3C,0X15,0X3D,0X14,0X3D,0X35,0X3D,0X35,0X3D,
0X35,0X3D,0X76,0X3D,0X77,0X3D,0X98,0X3D,0XD9,0X4D,0XB8,0X45,0X98,0X45,0X77,0X3D,
0X77,0X45,0X56,0X45,0X35,0X3D,0X15,0X3D,0X16,0X3D,0X16,0X45,0XF5,0X3C,0X14,0X3D,
0X14,0X3D,0X36,0X3D,0X77,0X3D,0X36,0X35,0X97,0X45,0X77,0X45,0X77,0X45,0X57,0X45,
0X37,0X45,0X16,0X3D,0X35,0X3D,0X14,0X3D,0X14,0X3D,0X15,0X3D,0X16,0X3D,0XD5,0X3C,
0X15,0X45,0XF5,0X3C,0X15,0X3D,0X16,0X3D,0X15,0X35,0X15,0X3D,0XF5,0X3C,0X16,0X3D,
0XF5,0X3C,0XF5,0X3C,0X15,0X3D,0XF5,0X3C,0XF5,0X3C,0X35,0X3D,0X15,0X35,0X56,0X45,
0X15,0X3D,0X56,0X3D,0X97,0X45,0X77,0X3D,0X77,0X3D,0X77,0X45,0X36,0X45,0X36,0X3D,
0X36,0X45,0X15,0X3D,0X15,0X3D,0X15,0X3D,0X15,0X3D,0XD5,0X3C,0XF5,0X3C,0X14,0X3D,
0XF4,0X3C,0XF5,0X44,0XF5,0X44,0XF5,0X3C,0X15,0X35,0X15,0X3D,0XF5,0X34,0X15,0X3D,
0X15,0X3D,0XF5,0X3C,0XF5,0X3C,0X15,0X3D,0X15,0X3D,0XF5,0X3C,0XF5,0X3C,0X15,0X3D,
0XF4,0X34,0X15,0X35,0XD4,0X4C,0X2B,0X2A,0XE9,0X39,0XFE,0XF7,0XB0,0X85,0X4E,0X76,
0X6D,0X66,0X6D,0X66,0X8D,0X66,0X6E,0X66,0X69,0X34,0XEA,0X3B,0XFF,0XE7,0XFF,0XEF,
0X4A,0X2A,0XB4,0X4C,0XF5,0X44,0XF4,0X34,0X15,0X35,0X56,0X3D,0X36,0X3D,0X56,0X3D,
0XD9,0X4D,0XB8,0X45,0XB8,0X45,0X97,0X45,0X97,0X45,0X57,0X3D,0X56,0X3D,0X36,0X3D,
0X36,0X3D,0X36,0X3D,0X36,0X3D,0X36,0X3D,0X36,0X3D,0X36,0X3D,0X36,0X3D,0X36,0X3D,
0X36,0X3D,0X36,0X3D,0X36,0X3D,0X36,0X3D,0X56,0X3D,0X36,0X3D,0X36,0X3D,0X56,0X3D,
0X56,0X3D,0X56,0X3D,0X97,0X45,0XB8,0X45,0XB8,0X45,0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,
0XB8,0X45,0XB8,0X45,0X77,0X45,0X77,0X45,0X56,0X3D,0X56,0X3D,0X36,0X3D,0X36,0X3D,
0X36,0X3D,0X36,0X3D,0X56,0X3D,0X56,0X3D,0X77,0X3D,0X97,0X45,0X97,0X45,0X97,0X45,
0X77,0X45,0X77,0X45,0X77,0X45,0X56,0X3D,0X56,0X3D,0X56,0X3D,0X56,0X3D,0X36,0X3D,
0X36,0X3D,0X36,0X3D,0X36,0X3D,0X36,0X3D,0X56,0X3D,0X56,0X3D,0X36,0X3D,0X36,0X3D,
0X36,0X3D,0X36,0X3D,0X36,0X3D,0X36,0X3D,0X36,0X3D,0X36,0X3D,0X56,0X3D,0X56,0X3D,
0X56,0X3D,0X56,0X3D,0X77,0X45,0XB7,0X45,0XB8,0X45,0X98,0X45,0X98,0X45,0X77,0X45,
0X57,0X45,0X56,0X3D,0X36,0X3D,0X36,0X3D,0X36,0X3D,0X36,0X3D,0X36,0X3D,0X36,0X3D,
0X36,0X3D,0X36,0X3D,0X36,0X45,0X36,0X3D,0X36,0X3D,0X56,0X3D,0X36,0X3D,0X36,0X3D,
0X36,0X3D,0X36,0X3D,0X36,0X45,0X36,0X45,0X56,0X3D,0X56,0X3D,0X36,0X3D,0X36,0X3D,
0X36,0X3D,0X36,0X3D,0X36,0X35,0X15,0X4D,0X4A,0X2A,0XE9,0X39,0XFF,0XFF,0X4B,0X53,
0XAA,0X43,0X09,0X3C,0X09,0X3C,0XE9,0X3B,0XCA,0X43,0XFE,0XE7,0XFF,0XEF,0XFF,0XE7,
0X6B,0X2A,0X73,0X54,0X15,0X4D,0X56,0X3D,0X76,0X3D,0X56,0X3D,0X57,0X45,0X36,0X45,
0X77,0X45,0XD9,0X45,0XD9,0X45,0XD9,0X4D,0XB8,0X45,0XB8,0X45,0X98,0X45,0X97,0X45,
0X77,0X45,0X77,0X45,0X77,0X45,0X77,0X45,0X77,0X45,0X77,0X45,0X77,0X45,0X77,0X45,
0X77,0X45,0X77,0X45,0X77,0X45,0X77,0X45,0X77,0X45,0X97,0X45,0X77,0X45,0X77,0X45,
0X97,0X45,0X97,0X45,0X97,0X45,0XB8,0X45,0XD8,0X4D,0XD9,0X45,0XF9,0X4D,0XF9,0X4D,
0XD9,0X45,0XD9,0X4D,0XD9,0X45,0XB8,0X45,0X98,0X45,0X97,0X45,0X97,0X45,0X77,0X45,
0X77,0X45,0X77,0X45,0X77,0X45,0X97,0X45,0X97,0X45,0XB8,0X45,0XD8,0X4D,0XB8,0X45,
0XB8,0X45,0XB8,0X45,0XB8,0X45,0X97,0X45,0X97,0X45,0X97,0X45,0X77,0X45,0X77,0X45,
0X77,0X45,0X77,0X45,0X77,0X45,0X77,0X45,0X77,0X45,0X77,0X45,0X77,0X45,0X77,0X45,
0X77,0X45,0X77,0X45,0X77,0X45,0X77,0X45,0X77,0X45,0X77,0X45,0X77,0X45,0X97,0X45,
0X97,0X45,0X97,0X45,0X97,0X45,0XB8,0X45,0XB8,0X4D,0XD8,0X4D,0XB9,0X4D,0XB9,0X45,
0XB9,0X45,0X97,0X45,0X97,0X45,0X97,0X45,0X77,0X45,0X77,0X45,0X77,0X45,0X77,0X45,
0X77,0X45,0X77,0X45,0X77,0X45,0X77,0X45,0X77,0X45,0X77,0X45,0X77,0X45,0X77,0X45,
0X77,0X45,0X77,0X45,0X77,0X45,0X77,0X45,0X77,0X45,0X97,0X45,0X97,0X45,0X77,0X45,
0X77,0X45,0X77,0X45,0X77,0X45,0X77,0X3D,0X15,0X4D,0X29,0X22,0X29,0X42,0XDF,0XFF,
0XFF,0XFF,0XFF,0XF7,0XFF,0XEF,0XFE,0XE7,0XFE,0XEF,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,
0X2A,0X22,0X93,0X5C,0X36,0X55,0X35,0X45,0X97,0X3D,0X97,0X3D,0X77,0X3D,0X77,0X45,
0X97,0X45,0XB8,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD8,0X4D,0XD9,0X4D,0XD8,0X4D,
0XD8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,
0XB8,0X45,0XB8,0X45,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,
0XB8,0X4D,0XB8,0X4D,0XD8,0X4D,0XD8,0X4D,0XD8,0X4D,0XF9,0X4D,0XD9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XD8,0X4D,0XD8,0X4D,0XD8,0X4D,
0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,0XD8,0X4D,0XB8,0X4D,0XD8,0X4D,0XD9,0X4D,
0XD9,0X4D,0XD9,0X4D,0XD9,0X4D,0XD8,0X4D,0XD8,0X4D,0XB8,0X45,0XB8,0X45,0XB8,0X45,
0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,
0XB8,0X45,0XB8,0X45,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X45,0XB8,0X45,
0XB8,0X45,0XB8,0X45,0XD8,0X4D,0XD8,0X4D,0XD8,0X4D,0XD8,0X4D,0XD9,0X4D,0XD9,0X4D,
0XD9,0X4D,0XD9,0X4D,0XD8,0X4D,0XD8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,
0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,
0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X4D,0XB8,0X45,0XB8,0X45,
0XB8,0X45,0XB8,0X4D,0XB8,0X4D,0X97,0X45,0X97,0X45,0X56,0X55,0X29,0X2A,0XE9,0X41,
0XC8,0X41,0XC8,0X41,0XC7,0X41,0XE7,0X41,0XE7,0X41,0XE7,0X41,0XE7,0X41,0X09,0X42,
0X8A,0X32,0X93,0X5C,0X35,0X55,0X56,0X45,0X77,0X45,0XD8,0X4D,0XD8,0X45,0XD8,0X4D,
0XD8,0X4D,0XB8,0X4D,0XD8,0X4D,};
#line 11 "app\\game.c"
#line 1 ".\\app\\RES\\Pic\\flappybird_column_bottom.c"
const unsigned char gImage_flappybird_column_bottom[1638] = {  
0XF9,0X4D,0XD7,0X65,0X67,0X22,0XCE,0X8C,0X0C,0X8D,0XAC,0X95,0X2C,0X9E,0X4C,0XA6,
0XAF,0XB6,0X10,0XC7,0X30,0XC7,0X91,0XCF,0XD2,0XDF,0XF2,0XDF,0XF4,0XDF,0XD3,0XD7,
0XB2,0XD7,0X50,0XC7,0X0F,0XBF,0X30,0XC7,0X8E,0XAE,0X2C,0XA6,0X2D,0XA6,0XCB,0X95,
0X4A,0X85,0X4A,0X85,0XC8,0X74,0X87,0X6C,0X67,0X64,0X25,0X5C,0XE5,0X53,0X05,0X54,
0XC4,0X53,0X06,0X5C,0XA7,0X5B,0XC9,0X5B,0X86,0X22,0X95,0X75,0X97,0X65,0XD8,0X5D,
0X96,0X6D,0X68,0X2A,0X8A,0X63,0X87,0X63,0XC5,0X5B,0X05,0X5C,0XE4,0X5B,0XC4,0X5B,
0XE5,0X63,0XC5,0X5B,0XC4,0X5B,0XC4,0X63,0XC4,0X63,0XC5,0X5B,0XC5,0X5B,0XE5,0X63,
0XC4,0X5B,0XC4,0X5B,0XE5,0X63,0XC5,0X63,0XC5,0X5B,0XE6,0X63,0XA5,0X5B,0XC5,0X5B,
0XE6,0X5B,0XC5,0X5B,0XC6,0X5B,0XE6,0X5B,0XE6,0X5B,0XC5,0X5B,0XE6,0X5B,0X06,0X64,
0XE7,0X5B,0XA7,0X5B,0XA9,0X5B,0XA7,0X2A,0X54,0X75,0XB8,0X75,0X76,0X75,0X75,0X7D,
0X06,0X32,0X29,0X63,0XA9,0X6B,0X87,0X63,0X87,0X5B,0XA7,0X63,0X87,0X63,0X87,0X6B,
0X87,0X63,0X87,0X63,0X86,0X6B,0X86,0X63,0X87,0X63,0X87,0X63,0X87,0X63,0X88,0X63,
0X87,0X63,0X67,0X63,0X88,0X63,0X88,0X63,0X67,0X63,0X88,0X63,0X88,0X63,0X88,0X63,
0X87,0X63,0X87,0X63,0X87,0X63,0X87,0X5B,0XA7,0X63,0XA7,0X63,0XA6,0X5B,0XA7,0X5B,
0X88,0X63,0X8A,0X63,0X06,0X2A,0X55,0X85,0X77,0X7D,0X89,0X2A,0X47,0X32,0X05,0X3A,
0X44,0X42,0X23,0X3A,0X43,0X3A,0X42,0X3A,0X42,0X3A,0X43,0X42,0X43,0X42,0X43,0X42,
0X63,0X42,0X42,0X42,0X42,0X42,0X42,0X42,0X42,0X3A,0X43,0X3A,0X43,0X3A,0X43,0X42,
0X43,0X3A,0X43,0X42,0X43,0X3A,0X23,0X3A,0X63,0X3A,0X43,0X3A,0X42,0X32,0X22,0X32,
0X42,0X32,0X42,0X32,0X42,0X32,0X62,0X3A,0X62,0X3A,0XA2,0X3A,0X62,0X32,0X63,0X3A,
0X24,0X32,0XA7,0X3A,0X48,0X32,0X09,0X22,0X46,0X3A,0X04,0X32,0X88,0X63,0X86,0X5B,
0X86,0X5B,0XC6,0X63,0XA5,0X63,0XA5,0X63,0XA5,0X5B,0X85,0X5B,0X86,0X5B,0X85,0X5B,
0X65,0X63,0X85,0X63,0XA5,0X63,0XA5,0X5B,0XA6,0X5B,0X85,0X5B,0XA5,0X5B,0XC6,0X5B,
0XA5,0X5B,0XA5,0X5B,0XA5,0X5B,0XC5,0X5B,0XA5,0X5B,0XC5,0X5B,0XC5,0X5B,0XE5,0X5B,
0XE5,0X5B,0XE5,0X5B,0XE5,0X5B,0XE5,0X5B,0X25,0X5C,0XC4,0X53,0XE6,0X5B,0X86,0X5B,
0XA9,0X63,0X8A,0X5B,0X27,0X32,0X05,0X4A,0X24,0X42,0XA7,0X63,0XA5,0X5B,0XE5,0X63,
0XA3,0X5B,0XA3,0X5B,0XE4,0X63,0XE4,0X63,0XC4,0X5B,0XE5,0X63,0XC4,0X63,0XA4,0X5B,
0XC4,0X63,0XC4,0X63,0XC4,0X5B,0XE5,0X63,0XA3,0X53,0XC4,0X5B,0X04,0X5C,0XC4,0X5B,
0XA3,0X53,0XE4,0X5B,0XC4,0X53,0XC4,0X5B,0XE4,0X5B,0XE4,0X5B,0X04,0X5C,0X04,0X5C,
0X04,0X5C,0X04,0X5C,0X03,0X54,0XC3,0X4B,0X44,0X5C,0XE4,0X53,0X06,0X5C,0XE7,0X63,
0X88,0X5B,0X05,0X3A,0XC5,0X49,0X45,0X4A,0X55,0XDF,0X93,0XD7,0XD2,0XDF,0XD1,0XDF,
0XF2,0XE7,0XF2,0XE7,0XD2,0XDF,0XB1,0XD7,0XB2,0XD7,0X71,0XD7,0X30,0XCF,0X30,0XCF,
0XEF,0XBE,0XAE,0XB6,0XAE,0XB6,0X4D,0XAE,0X2C,0XA6,0X2C,0XA6,0XCB,0X95,0X89,0X8D,
0X69,0X8D,0X28,0X85,0X07,0X7D,0X08,0X7D,0XA6,0X74,0X86,0X6C,0X86,0X6C,0X45,0X64,
0X24,0X5C,0X24,0X5C,0X03,0X4C,0X03,0X4C,0X24,0X54,0X05,0X5C,0X85,0X53,0XA7,0X63,
0X66,0X42,0XE6,0X51,0XE3,0X41,0X33,0XD7,0XB2,0XDF,0XB0,0XDF,0XF1,0XE7,0XF1,0XE7,
0XF1,0XE7,0XD1,0XDF,0XB1,0XD7,0XD1,0XDF,0X70,0XCF,0X4F,0XC7,0X4F,0XC7,0X0F,0XC7,
0XCD,0XB6,0XAD,0XB6,0X8D,0XAE,0X2B,0XA6,0X0B,0XA6,0XEA,0X9D,0XA9,0X95,0X89,0X8D,
0X28,0X85,0XE7,0X7C,0XE7,0X7C,0XA6,0X74,0X86,0X6C,0XA6,0X6C,0X65,0X64,0X24,0X5C,
0X04,0X54,0X24,0X54,0X23,0X54,0X03,0X4C,0X04,0X54,0X06,0X5C,0XA7,0X5B,0X45,0X42,
0XE5,0X49,0X23,0X4A,0X32,0XD7,0XB1,0XDF,0XB0,0XDF,0XD0,0XE7,0XF1,0XE7,0XF1,0XE7,
0XF1,0XDF,0XB0,0XDF,0XB1,0XDF,0X90,0XD7,0X4F,0XCF,0X6F,0XCF,0X2E,0XC7,0XAD,0XB6,
0XAD,0XB6,0X8C,0XAE,0X4B,0XA6,0X4B,0XA6,0XEA,0X95,0XA9,0X8D,0XA9,0X8D,0X48,0X85,
0X27,0X7D,0XE7,0X74,0XC6,0X6C,0X86,0X6C,0X86,0X6C,0X66,0X64,0X04,0X5C,0X24,0X5C,
0X04,0X54,0X24,0X54,0X24,0X54,0X04,0X54,0XE6,0X5B,0XA7,0X63,0X24,0X42,0X04,0X4A,
0X22,0X4A,0X52,0XDF,0XB1,0XDF,0XB0,0XDF,0XB0,0XDF,0XF1,0XE7,0XF1,0XE7,0XF1,0XE7,
0XD0,0XDF,0XB1,0XDF,0X70,0XD7,0X4F,0XC7,0X4F,0XC7,0X0E,0XC7,0XAC,0XB6,0XAD,0XB6,
0X8C,0XAE,0X4B,0XA6,0X2B,0XA6,0XEA,0X95,0XA9,0X8D,0XA9,0X8D,0X48,0X7D,0X07,0X7D,
0XE7,0X74,0XC7,0X74,0X86,0X64,0X86,0X6C,0X65,0X64,0X04,0X5C,0X24,0X5C,0X04,0X54,
0X04,0X54,0X04,0X54,0X05,0X54,0XE6,0X5B,0XA7,0X63,0X24,0X42,0X23,0X42,0X42,0X42,
0X73,0XDF,0X91,0XDF,0XB1,0XDF,0X90,0XDF,0XF1,0XE7,0XF1,0XE7,0XF0,0XDF,0XD0,0XDF,
0XB0,0XDF,0X90,0XD7,0X4F,0XC7,0X6F,0XC7,0X2E,0XBF,0XAD,0XB6,0XAD,0XB6,0X6C,0XAE,
0X4B,0XA6,0X2B,0XA6,0XCA,0X95,0X89,0X8D,0XA9,0X8D,0X47,0X85,0X07,0X7D,0X07,0X75,
0XC6,0X74,0XA5,0X6C,0X86,0X6C,0X65,0X64,0X04,0X5C,0X24,0X5C,0X04,0X5C,0X04,0X54,
0X04,0X54,0X05,0X54,0XE6,0X5B,0XA7,0X63,0X24,0X42,0X23,0X42,0X42,0X42,0X73,0XDF,
0X92,0XDF,0XB1,0XDF,0XB0,0XDF,0XF1,0XE7,0XF1,0XE7,0XF0,0XDF,0XAF,0XDF,0XB0,0XDF,
0X90,0XCF,0X4F,0XC7,0X4F,0XC7,0X2E,0XBF,0XCD,0XB6,0XAD,0XB6,0X6C,0XAE,0X2B,0XA6,
0X2B,0XA6,0XCA,0X95,0X89,0X8D,0X89,0X8D,0X27,0X85,0X07,0X7D,0X06,0X7D,0XC5,0X74,
0XA5,0X6C,0XA5,0X6C,0X64,0X64,0X03,0X5C,0X04,0X5C,0X04,0X5C,0X04,0X54,0X04,0X54,
0X05,0X54,0XE6,0X5B,0XA7,0X63,0X24,0X42,0X04,0X42,0X22,0X42,0X52,0XD7,0XB2,0XDF,
0XB1,0XDF,0XD1,0XE7,0XF1,0XE7,0XF0,0XE7,0XEF,0XE7,0XAE,0XDF,0XAF,0XDF,0X8F,0XCF,
0X4E,0XC7,0X4E,0XC7,0X0D,0XBF,0XAC,0XB6,0XAD,0XB6,0X6C,0XAE,0X4B,0XA6,0X2B,0XA6,
0XC9,0X95,0XA8,0X8D,0XA8,0X8D,0X27,0X85,0X27,0X7D,0X06,0X7D,0XC5,0X6C,0X85,0X6C,
0XA5,0X6C,0X64,0X64,0X23,0X5C,0X23,0X5C,0X03,0X54,0X23,0X54,0X24,0X54,0X04,0X54,
0XE6,0X5B,0XA7,0X63,0X24,0X42,0X04,0X4A,0X22,0X42,0X32,0XD7,0XB1,0XDF,0XB0,0XDF,
0XD1,0XE7,0XF1,0XE7,0XF1,0XEF,0XEF,0XE7,0XAF,0XDF,0XAF,0XD7,0X8F,0XCF,0X4E,0XC7,
0X4E,0XC7,0X0E,0XBF,0XAC,0XB6,0XAD,0XB6,0X6C,0XAE,0X2B,0XA6,0X2B,0XA6,0XC9,0X95,
0XA8,0X8D,0XA8,0X8D,0X27,0X85,0X27,0X7D,0X07,0X7D,0XA5,0X6C,0X85,0X6C,0XA5,0X6C,
0X64,0X64,0X23,0X5C,0X23,0X5C,0X22,0X54,0X23,0X54,0X23,0X54,0X04,0X54,0XC6,0X5B,
0XA7,0X63,0X25,0X42,0XE4,0X49,0X22,0X4A,0X52,0XD7,0XB1,0XD7,0XD0,0XD7,0XF0,0XDF,
0XF1,0XEF,0XF1,0XE7,0XF0,0XE7,0XB0,0XDF,0XB0,0XD7,0X90,0XCF,0X4F,0XC7,0X4F,0XC7,
0X2E,0XC7,0XCD,0XBE,0XAD,0XB6,0X6C,0XAE,0X2B,0XA6,0X2B,0XA6,0XCA,0X95,0X89,0X8D,
0XA9,0X8D,0X47,0X85,0X27,0X7D,0X07,0X7D,0XA6,0X6C,0XA5,0X6C,0XA6,0X6C,0X44,0X64,
0X04,0X5C,0X24,0X5C,0X03,0X54,0X23,0X54,0X03,0X54,0X05,0X5C,0XC6,0X5B,0X88,0X63,
0X05,0X42,0XE5,0X49,0X23,0X42,0X52,0XDF,0XB0,0XD7,0XD0,0XDF,0XD0,0XDF,0XF2,0XEF,
0XF2,0XE7,0XF2,0XE7,0XB1,0XDF,0XB1,0XD7,0X70,0XCF,0X2F,0XC7,0X2F,0XC7,0X0E,0XC7,
0XCD,0XBE,0XAD,0XB6,0X6C,0XAE,0X2B,0XA6,0X2B,0XA6,0XCA,0X9D,0X89,0X95,0XA9,0X8D,
0X28,0X85,0X07,0X7D,0X07,0X7D,0XA6,0X74,0XA6,0X6C,0XA6,0X6C,0X45,0X64,0X04,0X5C,
0X24,0X5C,0X03,0X54,0X23,0X54,0X04,0X54,0XE5,0X5B,0XC6,0X63,0X88,0X63,0X05,0X42,
0X05,0X42,0X24,0X42,0X33,0XDF,0X92,0XDF,0XD1,0XE7,0XD0,0XDF,0XF2,0XEF,0XF2,0XE7,
0XB1,0XE7,0XB1,0XDF,0X90,0XDF,0X70,0XD7,0X4F,0XD7,0X2F,0XCF,0XEE,0XC6,0XAD,0XBE,
0XAE,0XBE,0X6D,0XAE,0X2C,0XA6,0X2C,0XA6,0XEB,0X9D,0X8A,0X95,0X89,0X8D,0X28,0X85,
0XE7,0X7C,0X07,0X7D,0XC6,0X74,0X85,0X6C,0X86,0X6C,0X45,0X64,0X04,0X5C,0X24,0X5C,
0X03,0X54,0X24,0X5C,0XE4,0X53,0XC5,0X5B,0XC7,0X63,0X88,0X63,0X45,0X42,0XE6,0X41,
0X45,0X4A,0X14,0XDF,0X53,0XDF,0XB2,0XE7,0XB2,0XE7,0XD3,0XEF,0XD4,0XEF,0X73,0XDF,
0X93,0XDF,0X92,0XDF,0X71,0XDF,0X31,0XD7,0XF0,0XCE,0XD0,0XC6,0XD0,0XC6,0X8F,0XBE,
0X4E,0XAE,0X0D,0XA6,0X0D,0XAE,0XCC,0X9D,0X8B,0X95,0X8B,0X95,0X2A,0X8D,0XE9,0X84,
0X09,0X85,0XA8,0X7C,0X87,0X74,0X87,0X74,0X26,0X64,0XE5,0X5B,0X05,0X5C,0XE4,0X5B,
0XE5,0X5B,0XC5,0X5B,0XA6,0X5B,0XA7,0X63,0X88,0X63,0X25,0X42,0XE7,0X41,0XE5,0X41,
0X74,0XD6,0X35,0XE7,0XD2,0XD6,0X33,0XDF,0X95,0XE7,0X55,0XDF,0X96,0XE7,0XB6,0XEF,
0XB6,0XEF,0X95,0XEF,0X75,0XE7,0X55,0XDF,0X55,0XDF,0X55,0XDF,0X75,0XE7,0X55,0XDF,
0X14,0XD7,0X14,0XD7,0XB3,0XCE,0X72,0XC6,0X71,0XBE,0X30,0XB6,0XCF,0XAD,0XCF,0XAD,
0X8E,0XA5,0X4D,0X95,0X2D,0X95,0XEC,0X8C,0X8B,0X84,0XAB,0X84,0X8B,0X7C,0X4A,0X74,
0X4B,0X74,0XEA,0X6B,0X89,0X6B,0X4A,0X6B,0X26,0X42,0XE8,0X41,0XE7,0X41,0X25,0X42,
0X24,0X42,0X43,0X42,0X43,0X3A,0X64,0X42,0X65,0X42,0X24,0X3A,0X44,0X3A,0X44,0X42,
0X44,0X3A,0X44,0X3A,0X44,0X3A,0X44,0X3A,0X44,0X3A,0X44,0X3A,0X24,0X3A,0X24,0X3A,
0X45,0X42,0X44,0X3A,0X44,0X3A,0X65,0X42,0X44,0X3A,0X44,0X3A,0X65,0X3A,0X44,0X3A,
0X44,0X3A,0X65,0X3A,0X45,0X3A,0X25,0X3A,0X45,0X3A,0X45,0X32,0X24,0X32,0X46,0X3A,
0X26,0X3A,0X06,0X3A,0X47,0X42,0XC6,0X39,0XC9,0X31,0X09,0X3A,0X48,0X3A,0X47,0X3A,
0X26,0X32,0X67,0X3A,0X07,0X2A,0X68,0X3A,0X48,0X32,0X47,0X32,0X47,0X32,0X47,0X32,
0X67,0X32,0X68,0X32,0X67,0X32,0X47,0X32,0X68,0X3A,0X68,0X3A,0X48,0X32,0X68,0X3A,
0X48,0X32,0X48,0X32,0X48,0X32,0X47,0X32,0X27,0X32,0X47,0X32,0X27,0X32,0X27,0X32,
0X28,0X32,0X27,0X32,0X07,0X2A,0X27,0X2A,0X68,0X32,0X68,0X32,0X48,0X32,0X48,0X32,
0X07,0X2A,0X08,0X32,0X08,0X32,};
#line 12 "app\\game.c"
#line 1 ".\\app\\RES\\Pic\\flappybird_column_middle.c"
const unsigned char gImage_flappybird_column_middle[490] = {  
0X62,0X42,0X0B,0X95,0X49,0X8D,0XA9,0X95,0X2B,0XA6,0X2B,0XA6,0X8D,0XB6,0XEE,0XC6,
0X0E,0XC7,0X6F,0XD7,0XD0,0XDF,0XD0,0XDF,0XF1,0XE7,0XB0,0XDF,0XB1,0XDF,0X6F,0XD7,
0X0E,0XC7,0X0E,0XC7,0XAD,0XB6,0X2B,0XA6,0X2B,0XA6,0XA9,0X95,0X48,0X85,0X68,0X85,
0XC7,0X74,0XA6,0X6C,0XA6,0X6C,0X24,0X5C,0X04,0X54,0X24,0X54,0X24,0X54,0X24,0X54,
0X04,0X54,0XE6,0X5B,0X63,0X3A,0X42,0X42,0X0B,0X95,0X49,0X8D,0XA9,0X95,0X2B,0XA6,
0X2B,0XA6,0XAD,0XBE,0X0E,0XCF,0X0E,0XC7,0X6F,0XD7,0XB0,0XDF,0XD0,0XDF,0XF1,0XE7,
0XB0,0XDF,0XB1,0XDF,0X6F,0XD7,0X0E,0XC7,0X0E,0XC7,0XAD,0XBE,0X4B,0XA6,0X2B,0XA6,
0XC9,0X95,0X68,0X85,0X68,0X85,0XC7,0X74,0XA6,0X6C,0XA6,0X6C,0X24,0X5C,0X04,0X54,
0X24,0X54,0X24,0X54,0X24,0X54,0X04,0X54,0XE6,0X5B,0X63,0X3A,0X42,0X42,0X0B,0X95,
0X49,0X8D,0XA9,0X95,0X2B,0XA6,0X2B,0XA6,0XAD,0XBE,0X2F,0XCF,0X2E,0XCF,0X6F,0XD7,
0XB0,0XDF,0XD0,0XDF,0XF1,0XE7,0XB0,0XDF,0XB1,0XDF,0X6F,0XD7,0XEE,0XC6,0X0E,0XC7,
0XAD,0XB6,0X2B,0XA6,0X2B,0XA6,0XC9,0X95,0X68,0X85,0X68,0X85,0XC7,0X74,0XA6,0X6C,
0XA6,0X6C,0X24,0X5C,0X04,0X54,0X24,0X54,0X24,0X54,0X24,0X54,0X04,0X54,0XE6,0X5B,
0X63,0X3A,0X62,0X42,0X0C,0X95,0X49,0X8D,0XA9,0X95,0X2B,0XA6,0X2B,0XA6,0XAD,0XBE,
0X0E,0XCF,0X0E,0XC7,0X6F,0XD7,0XB0,0XDF,0XD0,0XDF,0XF1,0XE7,0XB0,0XDF,0XB1,0XDF,
0X6F,0XD7,0X0E,0XC7,0X0E,0XC7,0XAD,0XBE,0X2B,0XA6,0X2B,0XA6,0XA9,0X95,0X48,0X85,
0X68,0X85,0XC7,0X74,0XA6,0X6C,0XA6,0X6C,0X24,0X5C,0X04,0X54,0X24,0X54,0X24,0X54,
0X24,0X54,0X04,0X54,0XE6,0X5B,0X63,0X3A,0X62,0X42,0X0B,0X95,0X49,0X8D,0XA9,0X95,
0X2B,0XA6,0X2B,0XA6,0X8D,0XBE,0X0E,0XC7,0X0E,0XC7,0X6F,0XD7,0XD0,0XDF,0XD0,0XDF,
0XF1,0XE7,0XB0,0XDF,0XB1,0XDF,0X6F,0XD7,0X0E,0XC7,0X0E,0XC7,0XAD,0XBE,0X4B,0XA6,
0X2B,0XA6,0XC9,0X95,0X68,0X85,0X68,0X85,0XC7,0X74,0XA6,0X6C,0XA6,0X6C,0X24,0X5C,
0X04,0X54,0X24,0X54,0X24,0X54,0X24,0X54,0X04,0X54,0XE6,0X5B,0X63,0X3A,0X42,0X42,
0XEB,0X8C,0X29,0X8D,0XA9,0X95,0X2B,0XA6,0X2B,0XA6,0X8D,0XBE,0X0E,0XC7,0X0E,0XC7,
0X6F,0XD7,0XD0,0XDF,0XD0,0XDF,0XF1,0XE7,0XB0,0XDF,0XB1,0XDF,0X6F,0XD7,0X2E,0XCF,
0X2E,0XC7,0XAD,0XBE,0X4B,0XA6,0X2B,0XA6,0XA9,0X95,0X48,0X85,0X68,0X85,0XC7,0X74,
0XA6,0X6C,0XA6,0X6C,0X24,0X5C,0X04,0X54,0X24,0X54,0X24,0X54,0X24,0X54,0X04,0X54,
0XE6,0X5B,0X63,0X3A,0X42,0X42,0XEB,0X8C,0X29,0X8D,0XA9,0X95,0X2B,0XA6,0X2B,0XA6,
0XAD,0XBE,0X0E,0XC7,0X0E,0XC7,0X6F,0XD7,0XB0,0XDF,0XD0,0XDF,0XF1,0XE7,0XB0,0XDF,
0XB1,0XDF,0X6F,0XD7,0X0E,0XC7,0X0E,0XC7,0XAD,0XBE,0X2B,0XA6,0X2B,0XA6,0XA9,0X95,
0X48,0X85,0X48,0X85,0XC7,0X74,0XA6,0X6C,0XA6,0X6C,0X24,0X5C,0X04,0X54,0X24,0X54,
0X24,0X54,0X24,0X54,0X04,0X54,0XE6,0X5B,0X63,0X3A,};
#line 13 "app\\game.c"
#line 1 ".\\app\\RES\\Pic\\flappybird_gameover.c"
const unsigned char gImage_flappybird_gameover[7830] = {  
0XF9,0X4D,0XD8,0X55,0XD9,0X5D,0X57,0X6D,0X2A,0X32,0XE8,0X41,0X08,0X4A,0XE8,0X49,
0XE8,0X49,0X09,0X4A,0XC7,0X49,0XE8,0X49,0X08,0X4A,0XC8,0X41,0XE8,0X41,0XE8,0X49,
0X09,0X4A,0X09,0X3A,0X4B,0X2A,0X77,0X6D,0XFA,0X55,0XF9,0X45,0XF9,0X4D,0XF9,0X55,
0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XFA,0X4D,0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XFA,0X4D,0XD9,0X4D,0XD9,0X4D,0X1A,0X4E,0XB9,0X4D,0XDA,0X55,0XFA,0X4D,0XF9,0X45,
0XF9,0X4D,0XFA,0X4D,0XDA,0X4D,0XFA,0X4D,0XF9,0X45,0X19,0X46,0X19,0X4E,0XF9,0X45,
0XFA,0X4D,0XFA,0X4D,0XF9,0X45,0XFA,0X4D,0XFA,0X4D,0XDA,0X4D,0XFA,0X4D,0XF9,0X45,
0X19,0X4E,0XD9,0X45,0XFA,0X4D,0XD9,0X45,0XFA,0X4D,0XF9,0X55,0X97,0X5D,0X56,0X75,
0X29,0X3A,0XE8,0X49,0XE8,0X49,0XE8,0X41,0XE8,0X49,0XE8,0X49,0XE9,0X41,0XE9,0X41,
0XC8,0X49,0X29,0X42,0X37,0X7D,0X99,0X65,0XDA,0X55,0XDA,0X4D,0XFA,0X45,0X1A,0X46,
0X1A,0X46,0XFA,0X45,0XFA,0X4D,0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X19,0X46,
0XF9,0X45,0X19,0X46,0XFA,0X4D,0XDA,0X4D,0XFA,0X4D,0XD9,0X45,0XF9,0X4D,0XF9,0X4D,
0XFA,0X4D,0XD9,0X45,0XFA,0X4D,0XFA,0X4D,0XD9,0X4D,0XF9,0X55,0XDA,0X4D,0XDA,0X4D,
0XD9,0X55,0XD9,0X4D,0X1A,0X4E,0X19,0X46,0XF8,0X45,0X19,0X4E,0XD9,0X45,0X1A,0X56,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XFB,0X4D,0XDA,0X45,0XFA,0X4D,0XD8,0X4D,
0XD8,0X55,0X77,0X65,0X8B,0X22,0XFF,0XF7,0XDF,0XFF,0XDE,0XFF,0XBD,0XFF,0XBE,0XFF,
0XBE,0XFF,0XBD,0XFF,0XBE,0XFF,0XDE,0XFF,0XDE,0XFF,0XDE,0XFF,0XBE,0XFF,0XBF,0XFF,
0X09,0X3A,0X4B,0X32,0X78,0X6D,0XB9,0X4D,0X1A,0X46,0XF9,0X4D,0XF9,0X4D,0XFA,0X4D,
0XFA,0X4D,0XFA,0X45,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,
0XFA,0X4D,0X19,0X4E,0XF9,0X4D,0X1A,0X56,0XD9,0X4D,0XD9,0X4D,0XFA,0X5D,0XD9,0X55,
0XF9,0X5D,0XB9,0X55,0X99,0X55,0XF9,0X55,0XF8,0X55,0XD8,0X55,0XF9,0X55,0XD9,0X4D,
0XFA,0X4D,0X1A,0X4E,0XD9,0X45,0XFA,0X4D,0XDA,0X4D,0XF9,0X4D,0X1A,0X4E,0XF9,0X45,
0XF9,0X45,0XF9,0X4D,0X1A,0X56,0XB8,0X4D,0X98,0X55,0X97,0X6D,0X8B,0X22,0XFF,0XF7,
0XDF,0XFF,0XDE,0XFF,0XDE,0XFF,0XDE,0XFF,0XDE,0XFF,0XDF,0XFF,0XDF,0XFF,0XDE,0XFF,
0XDF,0XFF,0X6C,0X2A,0X78,0X6D,0XB9,0X5D,0XD9,0X55,0XF9,0X45,0X1A,0X4E,0XFA,0X45,
0XFA,0X45,0XFA,0X4D,0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0X19,0X4E,
0X1A,0X4E,0XFA,0X45,0XDA,0X4D,0XD9,0X55,0XD9,0X5D,0XB9,0X55,0XB8,0X55,0XD9,0X55,
0XB9,0X55,0X98,0X55,0XB9,0X5D,0X98,0X55,0XD9,0X55,0XD9,0X4D,0XDA,0X4D,0XFA,0X55,
0XF9,0X4D,0XD9,0X45,0XF9,0X45,0X19,0X4E,0XF9,0X4D,0X1A,0X4E,0XD9,0X4D,0XFA,0X4D,
0XF9,0X4D,0XD8,0X45,0X1A,0X4E,0XDA,0X45,0XFB,0X4D,0X1A,0X4E,0XF9,0X4D,0XB8,0X5D,
0X36,0X75,0X4B,0X32,0XFF,0XFF,0XBE,0XFF,0XDC,0XFF,0XBB,0XFF,0XBB,0XFF,0XBB,0XFF,
0XBB,0XFF,0XDB,0XFF,0XBB,0XFF,0XBB,0XFF,0XBB,0XFF,0X9B,0XFF,0XDE,0XFF,0XE8,0X39,
0X2B,0X2A,0X78,0X75,0XB8,0X55,0X19,0X4E,0XD9,0X45,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,
0XFA,0X45,0XFA,0X45,0XFA,0X4D,0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XFA,0X4D,
0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,
0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XF9,0X4D,0XD9,0X4D,0XFA,0X4D,
0X1A,0X4E,0XD8,0X3D,0X19,0X46,0XF9,0X4D,0XB8,0X55,0X98,0X65,0X77,0X6D,0X57,0X75,
0X78,0X75,0X98,0X75,0X58,0X6D,0X57,0X6D,0X78,0X6D,0X98,0X6D,0XB9,0X65,0X98,0X55,
0XB8,0X4D,0X1A,0X4E,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XD9,0X45,0X19,0X46,0X3A,0X4E,
0XF9,0X4D,0XB8,0X4D,0XB8,0X5D,0X77,0X65,0X36,0X7D,0X2A,0X32,0XFF,0XFF,0XDE,0XFF,
0XDB,0XFF,0XBA,0XFF,0X9A,0XFF,0XBB,0XFF,0XDB,0XFF,0XDB,0XFF,0XBD,0XFF,0XFF,0XFF,
0X2A,0X2A,0X57,0X75,0X78,0X65,0XF9,0X5D,0XF9,0X4D,0XF9,0X45,0XFA,0X4D,0XFA,0X4D,
0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XF9,0X4D,0XF9,0X55,0XB8,0X4D,
0X1A,0X4E,0XDA,0X55,0X58,0X65,0X78,0X75,0X77,0X6D,0X77,0X6D,0X77,0X65,0X77,0X6D,
0X78,0X75,0X57,0X6D,0X77,0X6D,0XB8,0X65,0XD9,0X55,0XFA,0X4D,0XD9,0X4D,0XF9,0X4D,
0XF9,0X4D,0X1A,0X4E,0XB9,0X45,0XFA,0X4D,0XFA,0X4D,0XD9,0X4D,0XD9,0X4D,0XFA,0X4D,
0XFA,0X4D,0XD9,0X4D,0XDA,0X4D,0XDA,0X4D,0XB8,0X45,0X77,0X65,0X36,0X6D,0X4B,0X32,
0XDF,0XFF,0XDD,0XFF,0XBB,0XFF,0X4F,0XDD,0X2E,0XE5,0X2E,0XE5,0X2E,0XE5,0X2E,0XE5,
0X4E,0XE5,0X4F,0XE5,0X4F,0XE5,0X2E,0XED,0X50,0XE5,0XBD,0XFF,0X09,0X42,0X6C,0X32,
0X78,0X6D,0XD8,0X5D,0X19,0X56,0XFA,0X4D,0XFA,0X45,0XFA,0X4D,0XF9,0X45,0X19,0X46,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,
0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,
0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XF9,0X4D,0XFA,0X4D,0XF9,0X4D,0XF9,0X45,
0X3A,0X46,0X39,0X46,0XF8,0X55,0X77,0X65,0X8B,0X22,0X4B,0X2A,0X4A,0X2A,0X6A,0X2A,
0X4A,0X22,0X4B,0X2A,0X6C,0X32,0X4B,0X22,0X4A,0X22,0X8B,0X1A,0X98,0X6D,0XF9,0X5D,
0XD9,0X45,0XF9,0X45,0XFA,0X4D,0XF9,0X45,0X19,0X4E,0XF9,0X45,0XF9,0X4D,0XD9,0X45,
0XF9,0X5D,0X77,0X65,0X6B,0X22,0X29,0X32,0XFF,0XFF,0XFD,0XFF,0XBB,0XFF,0X6E,0XDD,
0X4D,0XE5,0X4E,0XE5,0X6F,0XED,0X4E,0XE5,0X4E,0XDD,0X70,0XD5,0XDC,0XFF,0XFF,0XFF,
0X6B,0X2A,0X6B,0X1A,0X77,0X65,0XD9,0X55,0XF9,0X4D,0XFA,0X4D,0XFA,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0XFA,0X4D,0XF9,0X4D,0XB8,0X4D,0XF9,0X55,0XD9,0X55,
0X98,0X5D,0X8C,0X2A,0X2A,0X2A,0X2A,0X22,0X6A,0X2A,0X6A,0X22,0X8B,0X2A,0X2A,0X22,
0X6B,0X2A,0X6B,0X22,0X6B,0X12,0X97,0X6D,0XB8,0X5D,0XF9,0X55,0X19,0X4E,0XF9,0X4D,
0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XB9,0X4D,0XFA,0X55,0XFA,0X4D,0XF9,0X4D,0XD9,0X45,
0XFA,0X4D,0XD9,0X55,0XD9,0X4D,0X1A,0X56,0X0A,0X3A,0X2A,0X3A,0XFF,0XFF,0XFD,0XFF,
0X2E,0XDD,0X2B,0XED,0X0A,0XED,0X2A,0XF5,0X0B,0XFD,0X2B,0XFD,0X0A,0XF5,0X0A,0XED,
0X2B,0XF5,0X2B,0XFD,0X0A,0XF5,0X2D,0XED,0XDD,0XFF,0X09,0X3A,0X2B,0X22,0X57,0X6D,
0XB7,0X5D,0XF8,0X4D,0XD8,0X4D,0X19,0X4E,0X19,0X46,0X19,0X46,0X19,0X46,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X4D,0XF9,0X4D,0X19,0X4E,0XF8,0X45,
0XF9,0X4D,0X97,0X5D,0X6A,0X2A,0XFF,0XF7,0XFF,0XFF,0XDF,0XF7,0XFF,0XF7,0XFF,0XFF,
0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0X6A,0X22,0XB7,0X5D,0X1A,0X56,
0XF9,0X4D,0XF9,0X4D,0X19,0X4E,0XD9,0X4D,0XDA,0X55,0XBA,0X4D,0XF9,0X55,0XB8,0X5D,
0X4A,0X22,0XFF,0XF7,0XFF,0XFF,0XFD,0XFF,0X2E,0XDD,0X4C,0XF5,0X2A,0XF5,0X29,0XF5,
0X2B,0XF5,0X0A,0XF5,0XE9,0XF4,0X4A,0XFD,0X4A,0XF5,0X0C,0XD5,0XFD,0XFF,0XFF,0XFF,
0XFF,0XF7,0X6C,0X2A,0XB8,0X5D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0XFA,0X4D,0XFA,0X4D,0XF9,0X4D,0X19,0X4E,0XF9,0X55,0X98,0X65,0X6B,0X1A,
0XFF,0XEF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XF7,
0XFF,0XF7,0XFF,0XF7,0X6A,0X3A,0X6A,0X22,0XB7,0X5D,0XD9,0X45,0XFA,0X4D,0XFA,0X4D,
0XD9,0X45,0X1A,0X4E,0XFA,0X4D,0XD9,0X4D,0XF9,0X4D,0X19,0X4E,0XF9,0X4D,0XFA,0X4D,
0XB8,0X4D,0XD8,0X4D,0XD9,0X4D,0XC9,0X51,0XE8,0X51,0XDE,0XFF,0XDA,0XFF,0X6C,0XF5,
0X29,0XFD,0X29,0XFD,0X09,0XFD,0XE9,0XFC,0X09,0XFD,0X29,0XFD,0X08,0XFD,0X09,0XFD,
0X09,0XFD,0XE8,0XFC,0X2C,0XF5,0XDC,0XFF,0XE9,0X39,0X4B,0X3A,0X56,0X7D,0XB7,0X6D,
0X97,0X65,0XB8,0X65,0XB8,0X65,0XB8,0X5D,0XD8,0X5D,0XB7,0X65,0XB7,0X65,0XB8,0X65,
0XB8,0X65,0XB8,0X65,0XB8,0X65,0XB8,0X65,0XB8,0X65,0XB8,0X65,0XB8,0X65,0XB8,0X65,
0XB8,0X65,0XB8,0X65,0XB8,0X65,0XB8,0X65,0XB8,0X65,0XB8,0X65,0XB8,0X65,0XB8,0X65,
0XB8,0X65,0XB8,0X65,0XB8,0X65,0X98,0X65,0XB8,0X65,0XD8,0X65,0XB8,0X5D,0X98,0X65,
0X97,0X7D,0X29,0X3A,0XBE,0XFF,0XDD,0XFF,0XDC,0XFF,0XDB,0XFF,0XDC,0XFF,0XDC,0XFF,
0XDD,0XFF,0XDD,0XFF,0XDD,0XFF,0XFE,0XFF,0X29,0X3A,0X77,0X75,0XB8,0X5D,0XF9,0X55,
0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XDA,0X55,0XDA,0X55,0XF9,0X4D,0XB8,0X65,0X4A,0X32,
0XBF,0XFF,0XFE,0XFF,0XDB,0XFF,0X2C,0XF5,0X09,0XFD,0X08,0XFD,0X29,0XFD,0X09,0XFD,
0X09,0XFD,0X29,0XFD,0X08,0XFD,0X07,0XFD,0X6B,0XF5,0XDB,0XFF,0XFE,0XFF,0XDF,0XFF,
0XE9,0X31,0X77,0X75,0X97,0X65,0XB8,0X65,0XB8,0X65,0XB7,0X65,0XB7,0X65,0XB8,0X65,
0XB8,0X65,0XB8,0X65,0XB8,0X65,0XB7,0X5D,0X96,0X65,0X57,0X7D,0X2A,0X32,0XFF,0XFF,
0XDD,0XFF,0XDC,0XFF,0XDD,0XFF,0XDD,0XFF,0XBC,0XFF,0XDC,0XFF,0XDC,0XFF,0XFD,0XFF,
0XFD,0XFF,0XC7,0X41,0X29,0X3A,0XB8,0X75,0XB8,0X65,0X98,0X5D,0XB9,0X65,0XB8,0X5D,
0X98,0X65,0XB8,0X65,0XB8,0X65,0XD7,0X5D,0XB7,0X5D,0XB8,0X65,0X98,0X65,0XB8,0X65,
0XB8,0X65,0X98,0X65,0XC8,0X49,0XC7,0X51,0XDC,0XFF,0X4F,0XD5,0X2B,0XED,0X49,0XFD,
0X09,0XFD,0XE9,0XFC,0X0A,0XFD,0XC8,0XFC,0XE8,0XFC,0X08,0XFD,0XE9,0XFC,0XE9,0XFC,
0XE8,0XFC,0X0C,0XF5,0X9D,0XFF,0X0A,0X4A,0XC9,0X41,0X2A,0X42,0X29,0X32,0X29,0X2A,
0X2A,0X32,0X2B,0X32,0X2A,0X32,0X2A,0X32,0X09,0X3A,0X29,0X32,0X29,0X32,0X49,0X2A,
0X29,0X32,0X29,0X32,0X29,0X32,0X29,0X32,0X29,0X32,0X29,0X32,0X29,0X32,0X29,0X32,
0X29,0X32,0X29,0X32,0X29,0X32,0X29,0X32,0X29,0X32,0X29,0X32,0X29,0X32,0X29,0X32,
0X29,0X32,0X29,0X32,0X09,0X32,0X2A,0X3A,0X2A,0X2A,0X2A,0X32,0X2B,0X3A,0XC9,0X39,
0XDF,0XFF,0XBC,0XFF,0X2E,0XE5,0X4D,0XED,0X2B,0XED,0X2B,0XED,0X6D,0XF5,0XEC,0XE4,
0X4F,0XED,0X99,0XFF,0XDC,0XFF,0XFE,0XFF,0X09,0X3A,0X4B,0X1A,0XD8,0X5D,0X19,0X4E,
0XD9,0X45,0XFA,0X4D,0XFA,0X4D,0XD9,0X4D,0XF9,0X4D,0XB7,0X5D,0X4B,0X32,0XDF,0XFF,
0XDD,0XFF,0X50,0XDD,0X2C,0XF5,0X09,0XFD,0X09,0XFD,0XC8,0XFC,0X0A,0XFD,0XE9,0XFC,
0X09,0XFD,0X29,0XFD,0X29,0XFD,0X0A,0XF5,0X4E,0XDD,0XDC,0XFF,0XDF,0XFF,0X09,0X42,
0X2A,0X42,0X09,0X32,0X2A,0X32,0X49,0X2A,0X49,0X32,0X29,0X32,0X2A,0X32,0X0A,0X32,
0X29,0X32,0X29,0X32,0X29,0X32,0X49,0X32,0X29,0X32,0XFF,0XFF,0XDD,0XFF,0XDA,0XFF,
0X4E,0XDD,0X4C,0XED,0X2C,0XED,0X6D,0XF5,0X2C,0XED,0X2D,0XED,0X4E,0XE5,0X9A,0XFF,
0XDE,0XFF,0XFF,0XFF,0X2A,0X2A,0XD5,0X7C,0X2B,0X32,0X2B,0X32,0X4B,0X3A,0X4A,0X32,
0X29,0X32,0X49,0X32,0X4A,0X32,0X4A,0X2A,0X4A,0X32,0X4A,0X32,0X0A,0X2A,0X2A,0X32,
0X0A,0X32,0XE9,0X51,0XE7,0X51,0XBB,0XFF,0X6F,0XE5,0X2B,0XF5,0X08,0XF5,0X29,0XFD,
0X09,0XFD,0XEA,0XFC,0XA5,0XCB,0X64,0XC3,0XA5,0XC3,0X85,0XC3,0X86,0XC3,0X86,0XC3,
0X88,0XB3,0XBE,0XFF,0XC8,0X49,0X09,0X52,0XBF,0XFF,0XDD,0XFF,0XDD,0XFF,0XBD,0XFF,
0XBD,0XFF,0XBD,0XFF,0XBD,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XDD,0XFF,0XDE,0XFF,
0XDE,0XFF,0XDD,0XFF,0XDC,0XFF,0XDC,0XFF,0XDC,0XFF,0XDC,0XFF,0XBC,0XFF,0XBC,0XFF,
0XBC,0XFF,0XBC,0XFF,0XDC,0XFF,0XDC,0XFF,0XDC,0XFF,0XDC,0XFF,0XBC,0XFF,0XBC,0XFF,
0XBD,0XFF,0XBE,0XFF,0XBF,0XFF,0XBF,0XFF,0X09,0X4A,0XE9,0X49,0XC8,0X51,0XBD,0XFF,
0X50,0XDD,0X4C,0XF5,0X08,0XF5,0X29,0XFD,0X29,0XFD,0X09,0XFD,0X2A,0XFD,0X0B,0XF5,
0X4D,0XED,0X50,0XD5,0XBC,0XFF,0XE8,0X51,0X2A,0X32,0X97,0X6D,0XD9,0X55,0XFA,0X4D,
0XFA,0X4D,0XF9,0X4D,0X19,0X4E,0X19,0X46,0XD8,0X5D,0X2A,0X32,0XBE,0XFF,0X9B,0XFF,
0X4F,0XE5,0X0B,0XF5,0X0A,0XFD,0XE9,0XFC,0X2A,0XFD,0X65,0XC3,0X85,0XC3,0X2A,0XFD,
0X09,0XFD,0X09,0XFD,0X0B,0XFD,0X4E,0XE5,0XBA,0XFF,0XDD,0XFF,0XBD,0XFF,0XBD,0XFF,
0XBD,0XFF,0XBD,0XFF,0XDD,0XFF,0XFD,0XFF,0XDD,0XFF,0XBD,0XFF,0XBD,0XFF,0XBC,0XFF,
0XBC,0XFF,0XBC,0XFF,0XDD,0XFF,0XFF,0XFF,0XFE,0XFF,0X70,0XD5,0X2D,0XE5,0X2A,0XF5,
0X09,0XFD,0X09,0XFD,0X09,0XFD,0X09,0XFD,0X2A,0XFD,0X0B,0XF5,0X4F,0XE5,0XDD,0XFF,
0XFF,0XFF,0XE9,0X41,0X0A,0X42,0XBF,0XFF,0XDF,0XFF,0XBE,0XFF,0XDD,0XFF,0XBC,0XFF,
0XBC,0XFF,0XBC,0XFF,0XDD,0XFF,0XBD,0XFF,0XBE,0XFF,0XDF,0XFF,0XDF,0XFF,0XC9,0X49,
0XC9,0X51,0XC7,0X51,0XBB,0XFF,0X4E,0XDD,0X2B,0XF5,0X29,0XFD,0X28,0XFD,0X29,0XF5,
0X2D,0XED,0X98,0XFF,0XBA,0XFF,0XDB,0XFF,0XBB,0XFF,0XBC,0XFF,0XBD,0XFF,0XBD,0XFF,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0XFF,0X4E,0XD5,0X6D,0XED,0X0C,0XED,0X2D,0XF5,
0X2D,0XED,0X2D,0XED,0X2C,0XED,0X2C,0XED,0X0C,0XED,0X2E,0XED,0XBA,0XFF,0X50,0XD5,
0X4E,0XE5,0X2C,0XED,0X4D,0XED,0X2D,0XED,0X4D,0XED,0X2C,0XED,0X2C,0XED,0X2C,0XED,
0X2C,0XED,0X2C,0XED,0X2C,0XED,0X2C,0XED,0X2C,0XED,0X2C,0XED,0X2D,0XED,0X2D,0XE5,
0X70,0XDD,0XBB,0XFF,0XDE,0XFF,0XFF,0XFF,0XBF,0XFF,0XDE,0XFF,0XBB,0XFF,0X4E,0XE5,
0X2B,0XED,0X29,0XF5,0X29,0XFD,0X09,0XFD,0X09,0XFD,0X29,0XFD,0X09,0XFD,0X2B,0XF5,
0X4E,0XE5,0XDB,0XFF,0XBE,0XFF,0XFF,0XFF,0X4B,0X22,0X78,0X6D,0XBA,0X55,0XFA,0X4D,
0XF9,0X4D,0X19,0X46,0X19,0X46,0XB7,0X5D,0X4A,0X32,0XDE,0XFF,0XBB,0XFF,0X4E,0XE5,
0X2B,0XF5,0X09,0XFD,0X28,0XFD,0X2B,0XF5,0XB9,0XFF,0XB9,0XFF,0X0B,0XF5,0XE9,0XFC,
0XEA,0XFC,0X0B,0XFD,0X4E,0XE5,0XB9,0XFF,0XDA,0XFF,0X70,0XD5,0X4E,0XE5,0X2D,0XED,
0X2D,0XED,0X4E,0XE5,0X50,0XD5,0XBA,0XFF,0X2E,0XE5,0X2D,0XED,0X2C,0XF5,0X2B,0XF5,
0X2C,0XED,0X4E,0XE5,0XFC,0XFF,0XDB,0XFF,0X4D,0XE5,0X2A,0XF5,0X09,0XFD,0X08,0XFD,
0X09,0XFD,0XE9,0XFC,0X29,0XFD,0XE9,0XFC,0X0A,0XFD,0X2D,0XED,0XDC,0XFF,0XFF,0XFF,
0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFD,0XFF,0X6F,0XDD,0X0C,0XE5,0X4C,0XF5,0X0C,0XF5,
0X2D,0XED,0X2D,0XED,0X2D,0XED,0X4F,0XDD,0XDD,0XFF,0XDF,0XFF,0XE8,0X49,0XC9,0X51,
0XC7,0X51,0XBB,0XFF,0X4E,0XE5,0X2B,0XF5,0X09,0XFD,0X29,0XFD,0X2A,0XF5,0X2E,0XE5,
0XBA,0XFF,0XDC,0XFF,0XFD,0XFF,0XFD,0XFF,0XDD,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XDA,0XFF,0X6C,0XED,0X08,0XF5,0X09,0XFD,0XE8,0XFC,0X09,0XFD,
0X29,0XFD,0X29,0XFD,0X09,0XFD,0X09,0XFD,0X0B,0XFD,0X97,0XFF,0X4E,0XE5,0X2A,0XF5,
0X29,0XFD,0X09,0XFD,0X09,0XFD,0XE9,0XFC,0X2A,0XFD,0X29,0XFD,0X09,0XFD,0X09,0XFD,
0X29,0XFD,0X09,0XFD,0X29,0XFD,0X09,0XFD,0X29,0XFD,0X09,0XFD,0X2B,0XFD,0X4D,0XED,
0XB9,0XFF,0XDC,0XFF,0XFE,0XFF,0XFF,0XFF,0XDD,0XFF,0XB8,0XFF,0X2B,0XED,0X2A,0XF5,
0X4A,0XFD,0X09,0XFD,0XE9,0XFC,0X09,0XFD,0X09,0XFD,0X29,0XFD,0X2A,0XFD,0X2C,0XED,
0XB9,0XFF,0XDD,0XFF,0XDF,0XF7,0X6C,0X32,0X78,0X75,0X99,0X55,0XD9,0X45,0XFA,0X4D,
0XF9,0X45,0XF9,0X4D,0XB8,0X5D,0X49,0X32,0XDE,0XFF,0XBA,0XFF,0X4E,0XE5,0X2A,0XF5,
0X09,0XFD,0X28,0XFD,0X4B,0XED,0XDB,0XFF,0XDB,0XFF,0X0B,0XF5,0XE9,0XFC,0XEA,0XFC,
0X0C,0XFD,0X4F,0XE5,0XB9,0XFF,0XB9,0XFF,0X6E,0XE5,0X2A,0XF5,0X29,0XFD,0X29,0XFD,
0X2B,0XF5,0X2E,0XE5,0XB8,0XFF,0X2B,0XF5,0X09,0XFD,0X09,0XFD,0X08,0XFD,0X29,0XFD,
0X0B,0XED,0XBA,0XFF,0XDA,0XFF,0X0B,0XED,0X29,0XFD,0X09,0XFD,0X09,0XFD,0X09,0XFD,
0X09,0XFD,0XC8,0XFC,0X29,0XFD,0XE9,0XFC,0X2C,0XF5,0XDA,0XFF,0XDC,0XFF,0XDE,0XFF,
0XFE,0XFF,0XFC,0XFF,0XD9,0XFF,0X2B,0XED,0X29,0XFD,0X09,0XFD,0X09,0XFD,0XE9,0XFC,
0X09,0XFD,0X0A,0XFD,0X0C,0XED,0XDC,0XFF,0XDF,0XFF,0XE7,0X49,0XC9,0X49,0XC7,0X51,
0XBB,0XFF,0X2E,0XE5,0X0B,0XFD,0XE9,0XFC,0X0A,0XFD,0X0B,0XF5,0X2E,0XE5,0XB9,0XFF,
0X99,0XFF,0XBA,0XFF,0XDA,0XFF,0XDA,0XFF,0XDA,0XFF,0XDB,0XFF,0XFE,0XFF,0XFE,0XFF,
0XDC,0XFF,0X90,0XDD,0X0A,0XED,0X29,0XFD,0XE9,0XFC,0X09,0XFD,0X09,0XFD,0X29,0XFD,
0X09,0XFD,0X29,0XFD,0X09,0XFD,0X2B,0XF5,0X97,0XFF,0X4E,0XE5,0X2A,0XF5,0X08,0XFD,
0X09,0XFD,0X29,0XFD,0X29,0XFD,0XE9,0XFC,0XE9,0XFC,0X09,0XFD,0X08,0XFD,0X08,0XFD,
0X08,0XFD,0X08,0XFD,0X09,0XFD,0X09,0XFD,0X09,0XFD,0X2A,0XFD,0X2A,0XF5,0X4C,0XED,
0X4E,0XD5,0XDB,0XFF,0XDE,0XFF,0XDC,0XFF,0X2D,0XED,0X4A,0XFD,0X0A,0XFD,0XE9,0XFC,
0X09,0XFD,0X09,0XFD,0X29,0XFD,0X09,0XFD,0XE8,0XFC,0X09,0XFD,0X0A,0XFD,0X2E,0XE5,
0XBD,0XFF,0XFF,0XFF,0X0A,0X22,0X77,0X75,0XF9,0X55,0X1A,0X46,0XFA,0X4D,0XFA,0X4D,
0XFA,0X4D,0XB8,0X65,0X29,0X32,0XDD,0XFF,0XBB,0XFF,0X4E,0XE5,0X2A,0XF5,0X09,0XFD,
0X09,0XFD,0X2C,0XED,0XDC,0XFF,0XDB,0XFF,0X2C,0XED,0X09,0XFD,0X09,0XFD,0X0B,0XF5,
0X4E,0XE5,0XB9,0XFF,0XB9,0XFF,0X4E,0XE5,0X2A,0XF5,0X28,0XFD,0X28,0XFD,0X2A,0XF5,
0X2E,0XE5,0X98,0XFF,0X0C,0XF5,0X0A,0XFD,0X2A,0XFD,0X09,0XFD,0X09,0XFD,0X2B,0XFD,
0X78,0XFF,0X2F,0XE5,0X2B,0XFD,0XE8,0XFC,0X09,0XFD,0X09,0XFD,0X29,0XFD,0X09,0XFD,
0X29,0XFD,0X09,0XFD,0X2A,0XFD,0X0A,0XF5,0X2B,0XED,0X6E,0XDD,0XFC,0XFF,0XFC,0XFF,
0X6F,0XDD,0X4C,0XED,0X0A,0XFD,0X09,0XFD,0X09,0XFD,0XE8,0XFC,0X29,0XFD,0X08,0XFD,
0X08,0XFD,0X4C,0XF5,0XBB,0XFF,0XFF,0XFF,0XE8,0X49,0XC9,0X51,0XC7,0X51,0XBA,0XFF,
0X4D,0XE5,0X0A,0XFD,0X09,0XFD,0X0A,0XFD,0X0B,0XF5,0X2E,0XE5,0X97,0XFF,0X6E,0XE5,
0X6E,0XE5,0X2E,0XE5,0X4E,0XE5,0X6E,0XE5,0X6F,0XDD,0XDC,0XFF,0XDC,0XFF,0XB9,0XFF,
0X2D,0XE5,0X4A,0XFD,0X09,0XFD,0XE9,0XFC,0X09,0XFD,0X09,0XFD,0X09,0XFD,0X29,0XFD,
0X09,0XFD,0X09,0XFD,0X2B,0XF5,0X97,0XFF,0X4E,0XE5,0X2A,0XFD,0XE8,0XFC,0X29,0XFD,
0X09,0XFD,0XE8,0XFC,0X4A,0XFD,0XE9,0XFC,0X29,0XFD,0X29,0XFD,0X08,0XFD,0X08,0XFD,
0X29,0XFD,0X09,0XFD,0X09,0XFD,0X0A,0XFD,0X09,0XFD,0X09,0XF5,0X4B,0XFD,0X4D,0XED,
0XB9,0XFF,0XDD,0XFF,0XDC,0XFF,0X2C,0XF5,0XC8,0XFC,0XE9,0XFC,0X0A,0XFD,0X0A,0XFD,
0X0A,0XFD,0X0B,0XF5,0X2A,0XF5,0X29,0XFD,0X08,0XFD,0X09,0XFD,0X2D,0XED,0XDD,0XFF,
0XFF,0XF7,0X6A,0X2A,0X77,0X6D,0XD8,0X55,0X19,0X46,0XFA,0X4D,0XDA,0X4D,0XFA,0X4D,
0XB8,0X65,0X29,0X32,0XDD,0XFF,0XBB,0XFF,0X4E,0XE5,0X2A,0XF5,0X09,0XFD,0X09,0XFD,
0X2D,0XED,0XDC,0XFF,0XDC,0XFF,0X2C,0XED,0X09,0XFD,0X09,0XFD,0X2A,0XF5,0X6E,0XE5,
0XB9,0XFF,0XB9,0XFF,0X4E,0XE5,0X0A,0XF5,0X08,0XFD,0X28,0XFD,0X2A,0XF5,0X2D,0XE5,
0X97,0XFF,0X0B,0XFD,0X0A,0XFD,0X2A,0XFD,0X0A,0XFD,0XE9,0XFC,0X2B,0XFD,0X98,0XFF,
0X2E,0XED,0X2B,0XFD,0X29,0XFD,0X09,0XFD,0X09,0XFD,0X0A,0XF5,0X63,0XC3,0X09,0XFD,
0X09,0XFD,0X09,0XFD,0X2A,0XFD,0X29,0XFD,0X2B,0XED,0XBA,0XFF,0XDA,0XFF,0X4C,0XED,
0X09,0XFD,0X29,0XFD,0X09,0XFD,0XE8,0XFC,0X29,0XFD,0X08,0XFD,0X08,0XFD,0X08,0XFD,
0X0B,0XED,0XDC,0XFF,0XDF,0XFF,0XE8,0X49,0XAA,0X51,0XC7,0X59,0XBA,0XFF,0X4D,0XE5,
0X2A,0XF5,0X29,0XFD,0X09,0XFD,0X2B,0XF5,0X4E,0XE5,0X98,0XFF,0X0C,0XED,0X2B,0XF5,
0X2B,0XF5,0X2B,0XF5,0X2B,0XF5,0X0C,0XED,0XB9,0XFF,0X70,0XDD,0X2D,0XED,0X0A,0XF5,
0X09,0XFD,0XE8,0XFC,0XE9,0XFC,0X09,0XFD,0X08,0XFD,0X28,0XFD,0X09,0XFD,0X09,0XFD,
0X08,0XFD,0X2A,0XF5,0X97,0XFF,0X2E,0XE5,0X0A,0XFD,0X08,0XFD,0X08,0XFD,0X29,0XFD,
0X09,0XFD,0X09,0XFD,0XE9,0XFC,0X09,0XFD,0XE8,0XFC,0X29,0XFD,0XE8,0XFC,0X09,0XFD,
0X09,0XFD,0X2A,0XFD,0XEA,0XFC,0X2A,0XFD,0X0A,0XFD,0X0A,0XFD,0X0A,0XFD,0X2D,0XE5,
0XDD,0XFF,0XDD,0XFF,0X0D,0XED,0X2A,0XFD,0XE9,0XFC,0X2A,0XFD,0X0C,0XED,0X97,0XFF,
0X97,0XFF,0X2C,0XE5,0X2A,0XFD,0X09,0XFD,0XE9,0XFC,0X2C,0XED,0XFC,0XFF,0XFF,0XF7,
0X2A,0X22,0X77,0X75,0XF9,0X55,0XD9,0X45,0XFA,0X4D,0XDA,0X55,0XFA,0X4D,0X98,0X65,
0X29,0X32,0XDE,0XFF,0XDA,0XFF,0X4E,0XE5,0X2B,0XF5,0X09,0XFD,0X09,0XFD,0X4D,0XED,
0XDC,0XFF,0XDC,0XFF,0X2C,0XED,0X09,0XFD,0X09,0XFD,0X2A,0XF5,0X6E,0XE5,0XD9,0XFF,
0XB9,0XFF,0X4E,0XE5,0X0A,0XFD,0X09,0XFD,0X09,0XFD,0X2B,0XF5,0X2D,0XED,0X97,0XFF,
0X0A,0XFD,0X09,0XFD,0X09,0XFD,0X0A,0XFD,0X2A,0XFD,0XEA,0XF4,0X97,0XFF,0X6E,0XE5,
0X0A,0XF5,0X08,0XFD,0X09,0XFD,0X2B,0XF5,0X6D,0XED,0X75,0XFF,0X2A,0XFD,0X0A,0XFD,
0XEA,0XFC,0X09,0XFD,0X29,0XFD,0X2B,0XF5,0XB8,0XFF,0X4F,0XDD,0X4C,0XF5,0X09,0XFD,
0XE9,0XFC,0XE8,0XFC,0X09,0XFD,0X09,0XFD,0X29,0XFD,0X29,0XFD,0X29,0XFD,0X4D,0XF5,
0XDC,0XFF,0XFF,0XFF,0XE8,0X49,0XAA,0X51,0XC7,0X59,0XBA,0XFF,0X4D,0XE5,0X2A,0XF5,
0X29,0XFD,0X08,0XFD,0X2A,0XF5,0X4E,0XE5,0X77,0XFF,0X2B,0XF5,0XE9,0XFC,0X0A,0XFD,
0X0A,0XFD,0X09,0XFD,0X2B,0XFD,0X97,0XFF,0X4E,0XE5,0X0B,0XF5,0X09,0XFD,0X09,0XFD,
0X09,0XFD,0X0A,0XFD,0X0A,0XFD,0X29,0XFD,0X29,0XFD,0X09,0XFD,0X09,0XFD,0X08,0XFD,
0X2A,0XF5,0X97,0XFF,0X2E,0XE5,0X0A,0XF5,0X29,0XFD,0XE8,0XFC,0X09,0XFD,0X09,0XFD,
0X0A,0XFD,0X09,0XFD,0X09,0XFD,0X29,0XFD,0XE8,0XFC,0X29,0XFD,0X09,0XFD,0X09,0XFD,
0XE9,0XF4,0X2B,0XFD,0X0A,0XFD,0X0A,0XFD,0XE9,0XFC,0X09,0XFD,0X0C,0XED,0XDC,0XFF,
0XDC,0XFF,0X2D,0XED,0X0A,0XFD,0X09,0XFD,0X0A,0XF5,0X4D,0XED,0X97,0XFF,0X98,0XFF,
0X4E,0XE5,0X0A,0XF5,0X09,0XFD,0X4A,0XFD,0X2C,0XE5,0XFC,0XFF,0XFF,0XF7,0X4A,0X2A,
0X77,0X75,0XB8,0X55,0X1A,0X4E,0XF9,0X4D,0XD9,0X4D,0XFA,0X4D,0X98,0X65,0X2A,0X32,
0XDE,0XFF,0XDA,0XFF,0X4E,0XE5,0X2B,0XF5,0X09,0XFD,0X29,0XFD,0X4D,0XED,0XDD,0XFF,
0XDD,0XFF,0X2C,0XED,0X09,0XFD,0X09,0XFD,0X2A,0XF5,0X6E,0XE5,0XB9,0XFF,0XB9,0XFF,
0X4E,0XE5,0X0A,0XFD,0X09,0XFD,0X09,0XFD,0X2B,0XF5,0X2D,0XE5,0X96,0XFF,0X0A,0XFD,
0X09,0XFD,0X09,0XFD,0X09,0XFD,0X09,0XFD,0X2B,0XFD,0XB7,0XFF,0X4D,0XDD,0X2A,0XFD,
0X4A,0XFD,0X09,0XFD,0X0B,0XF5,0X4E,0XDD,0XB7,0XFF,0X0B,0XF5,0XE9,0XFC,0X0A,0XFD,
0XC9,0XFC,0X09,0XFD,0X2A,0XF5,0X97,0XFF,0X6E,0XE5,0X2B,0XF5,0X49,0XFD,0X09,0XFD,
0XE9,0XFC,0X2A,0XFD,0X0A,0XFD,0X0A,0XF5,0X2A,0XF5,0X0B,0XFD,0X2E,0XED,0XDD,0XFF,
0XFF,0XFF,0XE9,0X49,0XC9,0X49,0XC7,0X51,0XBA,0XFF,0X4D,0XE5,0X2A,0XF5,0X29,0XFD,
0X29,0XFD,0X2A,0XF5,0X4E,0XE5,0X97,0XFF,0X4B,0XFD,0X09,0XFD,0X09,0XFD,0XE9,0XFC,
0X08,0XFD,0X29,0XF5,0XB6,0XFF,0X6E,0XED,0X2B,0XF5,0X09,0XFD,0XE8,0XFC,0X0A,0XFD,
0X0C,0XFD,0X65,0XC3,0X09,0XFD,0X09,0XFD,0X09,0XFD,0X09,0XFD,0X08,0XFD,0X2A,0XF5,
0X97,0XFF,0X4E,0XE5,0X2A,0XF5,0X29,0XFD,0X29,0XFD,0X09,0XFD,0X0A,0XFD,0X63,0XCB,
0XE9,0XFC,0X2A,0XFD,0X09,0XFD,0X09,0XFD,0X29,0XFD,0X09,0XFD,0X0A,0XFD,0X84,0XC3,
0XA5,0XCB,0X0B,0XFD,0X0A,0XFD,0X29,0XFD,0X09,0XFD,0X4C,0XED,0XFC,0XFF,0XDC,0XFF,
0X4D,0XED,0X09,0XFD,0X29,0XFD,0X29,0XF5,0X2B,0XF5,0X75,0XFF,0X76,0XFF,0X2C,0XF5,
0X0A,0XFD,0X09,0XFD,0XE8,0XFC,0X2C,0XED,0XDC,0XFF,0XFF,0XF7,0X4A,0X2A,0X57,0X6D,
0XF9,0X5D,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XFA,0X4D,0XB8,0X5D,0X2A,0X32,0XDE,0XFF,
0XDA,0XFF,0X4D,0XE5,0X2A,0XF5,0X08,0XFD,0X09,0XFD,0X2D,0XED,0XDD,0XFF,0XDD,0XFF,
0X2C,0XED,0X09,0XFD,0X09,0XFD,0X2A,0XF5,0X6E,0XE5,0XB9,0XFF,0XB9,0XFF,0X4E,0XE5,
0X0A,0XF5,0X09,0XFD,0X09,0XFD,0X2B,0XF5,0X2E,0XE5,0X97,0XFF,0X2A,0XF5,0X08,0XFD,
0X09,0XFD,0X09,0XFD,0X08,0XFD,0X0A,0XF5,0XB7,0XFF,0X4D,0XE5,0X0A,0XF5,0XC8,0XFC,
0X09,0XFD,0X2B,0XF5,0X2E,0XE5,0X97,0XFF,0X2B,0XF5,0X09,0XFD,0X08,0XFD,0X4A,0XFD,
0X09,0XFD,0X0A,0XF5,0XB7,0XFF,0X4E,0XED,0XE9,0XF4,0X08,0XFD,0X09,0XFD,0XE9,0XFC,
0X2A,0XF5,0XA6,0XBB,0XA7,0XBB,0X87,0XBB,0X67,0XBB,0X8A,0XB3,0XDE,0XFF,0XFF,0XFF,
0XE9,0X49,0XC9,0X49,0XC7,0X51,0XBB,0XFF,0X2E,0XE5,0X2B,0XF5,0X29,0XFD,0X09,0XFD,
0X2A,0XF5,0X2D,0XE5,0XB6,0XFF,0X0A,0XF5,0X09,0XFD,0X09,0XFD,0X09,0XFD,0X28,0XFD,
0X29,0XF5,0X96,0XFF,0X4D,0XE5,0X0A,0XF5,0X09,0XFD,0X09,0XFD,0X0A,0XF5,0X2D,0XF5,
0X76,0XFF,0X0A,0XFD,0X09,0XFD,0X09,0XFD,0X09,0XFD,0X08,0XFD,0X2A,0XFD,0X97,0XFF,
0X4E,0XE5,0X2B,0XF5,0X09,0XFD,0X09,0XFD,0X2A,0XFD,0XEA,0XF4,0X74,0XFF,0X4B,0XFD,
0X09,0XFD,0X09,0XFD,0X29,0XFD,0X29,0XFD,0X0A,0XF5,0X2B,0XF5,0X95,0XFF,0X75,0XFF,
0X2C,0XF5,0X0A,0XFD,0X09,0XFD,0X08,0XFD,0X2B,0XED,0XFC,0XFF,0XDC,0XFF,0X2D,0XED,
0X0A,0XFD,0X29,0XFD,0X29,0XFD,0X09,0XFD,0X64,0XCB,0X65,0XCB,0XEB,0XFC,0X2A,0XFD,
0X29,0XFD,0X09,0XFD,0X2C,0XED,0XDC,0XFF,0XFF,0XF7,0X4A,0X2A,0X77,0X75,0XB8,0X55,
0X19,0X46,0XF9,0X45,0XF9,0X4D,0XF9,0X4D,0XB8,0X65,0X2A,0X32,0XDE,0XFF,0XBB,0XFF,
0X4D,0XE5,0X2A,0XFD,0X08,0XFD,0X09,0XFD,0X2C,0XED,0XDD,0XFF,0XDD,0XFF,0X2C,0XED,
0X09,0XFD,0X09,0XFD,0X2A,0XF5,0X6D,0XE5,0XB8,0XFF,0XB9,0XFF,0X4E,0XE5,0X0A,0XF5,
0X09,0XFD,0X29,0XFD,0X2B,0XF5,0X2E,0XE5,0X97,0XFF,0X2A,0XF5,0X09,0XFD,0X09,0XFD,
0X09,0XFD,0X09,0XFD,0X2A,0XF5,0XB7,0XFF,0X4D,0XE5,0X2B,0XFD,0X09,0XFD,0X29,0XFD,
0X0A,0XF5,0X4D,0XED,0X76,0XFF,0X2A,0XF5,0X29,0XFD,0X08,0XFD,0X08,0XFD,0X09,0XFD,
0X0B,0XF5,0X97,0XFF,0X4D,0XE5,0X2A,0XFD,0X09,0XFD,0XE9,0XFC,0X2A,0XFD,0X2C,0XED,
0X97,0XFF,0X99,0XFF,0X9A,0XFF,0X9B,0XFF,0X9C,0XFF,0XDF,0XFF,0XDF,0XFF,0XE9,0X49,
0XC9,0X49,0XE8,0X59,0X9B,0XFF,0X2E,0XE5,0X2A,0XF5,0X29,0XFD,0XE9,0XFC,0X2B,0XF5,
0X4D,0XE5,0XB6,0XFF,0X0A,0XF5,0X09,0XFD,0X09,0XFD,0X29,0XFD,0X09,0XFD,0X0A,0XFD,
0X97,0XFF,0X2D,0XE5,0X2B,0XFD,0X09,0XFD,0X09,0XFD,0X0A,0XF5,0X4D,0XED,0X96,0XFF,
0X2B,0XF5,0X09,0XFD,0X09,0XFD,0X09,0XFD,0X08,0XFD,0X0A,0XFD,0X98,0XFF,0X4F,0XE5,
0X0A,0XFD,0X08,0XFD,0X08,0XFD,0X2A,0XFD,0X2D,0XED,0X97,0XFF,0X2A,0XF5,0X29,0XFD,
0X08,0XFD,0X08,0XFD,0X09,0XFD,0X2A,0XF5,0X4D,0XED,0XB7,0XFF,0XB7,0XFF,0X4D,0XE5,
0X4B,0XF5,0X08,0XFD,0X08,0XFD,0X2B,0XED,0XDC,0XFF,0XDD,0XFF,0X2D,0XED,0X0A,0XFD,
0X29,0XFD,0X09,0XFD,0XE8,0XFC,0X63,0XDB,0X43,0XD3,0X09,0XFD,0X29,0XFD,0X28,0XFD,
0X09,0XFD,0X2C,0XF5,0XDC,0XFF,0XFF,0XFF,0X4A,0X2A,0X77,0X6D,0XF9,0X55,0XF9,0X45,
0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X97,0X65,0X2A,0X32,0XDE,0XFF,0XBB,0XFF,0X4E,0XED,
0X0A,0XFD,0X08,0XFD,0X29,0XFD,0X4C,0XE5,0XDC,0XFF,0XDC,0XFF,0X2C,0XED,0X09,0XFD,
0X29,0XFD,0X2A,0XF5,0X4D,0XE5,0XD8,0XFF,0XB9,0XFF,0X2E,0XE5,0X2A,0XFD,0X08,0XFD,
0X09,0XFD,0X2B,0XF5,0X4E,0XE5,0XB7,0XFF,0X2B,0XF5,0X09,0XFD,0X09,0XFD,0X0A,0XFD,
0XE9,0XFC,0X2B,0XFD,0X97,0XFF,0X4E,0XE5,0X0A,0XF5,0X09,0XFD,0X08,0XFD,0X29,0XFD,
0X4C,0XED,0X95,0XFF,0X0A,0XFD,0X0A,0XFD,0X08,0XF5,0X29,0XFD,0XE8,0XFC,0X0A,0XFD,
0XB7,0XFF,0X6E,0XE5,0X0A,0XF5,0X09,0XFD,0X09,0XFD,0X0A,0XF5,0X4D,0XE5,0XB9,0XFF,
0XBC,0XFF,0XBE,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XDF,0XFF,0XE9,0X49,0XC9,0X49,
0X86,0X51,0XBB,0XFF,0X4E,0XE5,0X4A,0XF5,0X29,0XFD,0X0A,0XFD,0X2B,0XF5,0X6D,0XE5,
0X75,0XFF,0X2B,0XFD,0X09,0XFD,0X09,0XFD,0X29,0XFD,0X09,0XFD,0X0B,0XF5,0X97,0XFF,
0X2E,0XE5,0X0A,0XFD,0X09,0XFD,0X2A,0XFD,0X0B,0XF5,0X6E,0XED,0X96,0XFF,0X2B,0XF5,
0X09,0XFD,0X09,0XFD,0X09,0XFD,0X08,0XFD,0X0A,0XFD,0X98,0XFF,0X4E,0XE5,0X2A,0XF5,
0X08,0XFD,0X08,0XFD,0X2B,0XF5,0X2E,0XE5,0X98,0XFF,0X2B,0XF5,0X28,0XFD,0X08,0XFD,
0X08,0XFD,0X09,0XFD,0X2A,0XF5,0X4D,0XE5,0XB9,0XFF,0XD9,0XFF,0X4E,0XDD,0X0A,0XED,
0X29,0XFD,0XE8,0XFC,0X4C,0XF5,0XDC,0XFF,0XDC,0XFF,0X4D,0XF5,0X0A,0XFD,0X09,0XFD,
0X09,0XFD,0X09,0XFD,0XC8,0XFC,0XE8,0XFC,0XE8,0XFC,0X09,0XFD,0X09,0XFD,0X09,0XFD,
0XEC,0XEC,0XDC,0XFF,0XFF,0XFF,0X4A,0X2A,0X97,0X75,0XD9,0X55,0X3A,0X4E,0X1A,0X4E,
0XB8,0X45,0XF9,0X4D,0X98,0X65,0X4A,0X32,0XDE,0XFF,0XBB,0XFF,0X2E,0XED,0X0A,0XFD,
0X29,0XFD,0X08,0XF5,0X4C,0XED,0XBC,0XFF,0XDC,0XFF,0X4C,0XED,0X29,0XFD,0X09,0XFD,
0X2B,0XFD,0X2D,0XDD,0XB8,0XFF,0XB9,0XFF,0X4F,0XE5,0X0A,0XF5,0X29,0XFD,0X29,0XFD,
0X0A,0XF5,0X4E,0XE5,0X97,0XFF,0X4B,0XFD,0X29,0XFD,0X09,0XFD,0XE9,0XFC,0X09,0XFD,
0XEA,0XFC,0X96,0XFF,0X4E,0XE5,0X2B,0XFD,0X09,0XFD,0X08,0XFD,0X09,0XFD,0X2A,0XF5,
0X0A,0XF5,0X0B,0XFD,0XEA,0XFC,0X09,0XFD,0X29,0XFD,0X09,0XFD,0X0A,0XFD,0XB7,0XFF,
0X6E,0XE5,0X2A,0XF5,0X09,0XFD,0X09,0XFD,0X2B,0XF5,0X4E,0XE5,0XDB,0XFF,0XE7,0X51,
0X09,0X3A,0X4A,0X2A,0X4A,0X22,0X4B,0X2A,0X4B,0X32,0X0A,0X32,0XC9,0X51,0XE7,0X59,
0XBA,0XFF,0X4D,0XE5,0X2A,0XF5,0X09,0XFD,0X0A,0XFD,0X0B,0XF5,0X4D,0XDD,0XB6,0XFF,
0X0A,0XF5,0X09,0XFD,0X29,0XFD,0X08,0XFD,0X29,0XFD,0X2B,0XF5,0X97,0XFF,0X6F,0XE5,
0X0A,0XF5,0XE8,0XFC,0X29,0XFD,0X2B,0XF5,0X2D,0XE5,0X97,0XFF,0X2B,0XF5,0X09,0XFD,
0X09,0XFD,0X09,0XFD,0X08,0XFD,0X2A,0XF5,0X97,0XFF,0X4E,0XE5,0X2A,0XF5,0X29,0XFD,
0X28,0XFD,0X2B,0XF5,0X2E,0XE5,0X98,0XFF,0X2A,0XF5,0X28,0XFD,0X09,0XFD,0X09,0XFD,
0X29,0XFD,0X2B,0XF5,0X4E,0XE5,0XB9,0XFF,0XB9,0XFF,0X4D,0XE5,0X4B,0XFD,0X08,0XFD,
0X09,0XFD,0X0C,0XED,0XDC,0XFF,0XDC,0XFF,0X0B,0XE5,0X09,0XFD,0X09,0XF5,0X2A,0XFD,
0X29,0XFD,0X09,0XFD,0X09,0XFD,0XE9,0XFC,0X0A,0XFD,0XC9,0XFC,0XEA,0XFC,0X2D,0XF5,
0XDC,0XFF,0XFF,0XF7,0X8A,0X2A,0X77,0X75,0XB9,0X55,0XDA,0X4D,0XD9,0X4D,0X1A,0X56,
0XF9,0X4D,0X97,0X5D,0X49,0X2A,0XDE,0XFF,0XBB,0XFF,0X2E,0XE5,0X2B,0XFD,0X09,0XFD,
0X29,0XFD,0X4D,0XED,0XDC,0XFF,0XBC,0XFF,0X2C,0XED,0XE8,0XFC,0X09,0XFD,0X0B,0XF5,
0X6F,0XED,0XB9,0XFF,0X99,0XFF,0X4F,0XED,0X0B,0XF5,0XE9,0XFC,0X09,0XFD,0X2B,0XF5,
0X4D,0XE5,0XB7,0XFF,0X0A,0XED,0X28,0XFD,0X08,0XFD,0X29,0XFD,0X09,0XFD,0X4C,0XFD,
0X96,0XFF,0X4D,0XDD,0X0A,0XFD,0X09,0XFD,0X29,0XFD,0X09,0XFD,0X29,0XFD,0X0A,0XFD,
0XEA,0XFC,0XCA,0XFC,0X09,0XFD,0XE9,0XFC,0XC9,0XFC,0X0B,0XFD,0X97,0XFF,0X4D,0XE5,
0X2A,0XF5,0X09,0XFD,0X09,0XFD,0X0B,0XF5,0X4E,0XE5,0XDB,0XFF,0X28,0X3A,0X77,0X85,
0X98,0X6D,0XB8,0X5D,0XB9,0X65,0X98,0X65,0X97,0X65,0XA8,0X51,0XA6,0X51,0XDA,0XFF,
0X6D,0XE5,0X0A,0XF5,0X09,0XFD,0XE9,0XFC,0X0B,0XFD,0X4D,0XE5,0X96,0XFF,0X0A,0XF5,
0X09,0XFD,0XE8,0XFC,0X29,0XFD,0X09,0XFD,0X0B,0XF5,0X97,0XFF,0X2E,0XDD,0X4B,0XFD,
0X29,0XFD,0X29,0XFD,0X2A,0XF5,0X2D,0XE5,0X96,0XFF,0X2B,0XF5,0X09,0XFD,0X09,0XFD,
0X29,0XFD,0X08,0XFD,0X2A,0XF5,0X96,0XFF,0X4D,0XE5,0X2B,0XF5,0X29,0XFD,0X09,0XFD,
0X2B,0XF5,0X2E,0XE5,0X97,0XFF,0X2A,0XF5,0X08,0XFD,0X09,0XFD,0X09,0XFD,0X09,0XFD,
0X2B,0XF5,0X4E,0XE5,0XB9,0XFF,0XB9,0XFF,0X4D,0XE5,0X0A,0XFD,0X09,0XFD,0X09,0XFD,
0X2C,0XED,0XDC,0XFF,0XFC,0XFF,0X4C,0XF5,0X09,0XFD,0X2A,0XFD,0XE9,0XF4,0X09,0XFD,
0X09,0XFD,0XC9,0XFC,0X64,0XCB,0X44,0XD3,0X45,0XCB,0X25,0XCB,0X87,0XBB,0XDC,0XFF,
0XFF,0XF7,0X4A,0X22,0X98,0X75,0X99,0X55,0XFB,0X4D,0XDA,0X4D,0XFA,0X4D,0XD9,0X4D,
0XD8,0X65,0X49,0X2A,0XFE,0XFF,0XDB,0XFF,0X4F,0XE5,0X2B,0XF5,0XE9,0XFC,0X09,0XFD,
0X0B,0XED,0XBB,0XFF,0XBB,0XFF,0X2C,0XF5,0X29,0XFD,0X09,0XFD,0X2B,0XF5,0X2E,0XE5,
0X99,0XFF,0XB9,0XFF,0X2E,0XE5,0X2B,0XF5,0X29,0XFD,0XE8,0XFC,0X2B,0XFD,0X2D,0XED,
0X76,0XFF,0X4A,0XF5,0X29,0XFD,0X28,0XFD,0XE8,0XFC,0X44,0XC3,0X45,0XBB,0XB7,0XFF,
0X6E,0XE5,0X0A,0XFD,0X09,0XFD,0X09,0XFD,0X29,0XFD,0X08,0XFD,0XE9,0XFC,0X65,0XCB,
0X65,0XCB,0X44,0XD3,0X44,0XCB,0X65,0XD3,0X65,0XC3,0X97,0XFF,0X4D,0XED,0X2A,0XF5,
0X09,0XFD,0X09,0XFD,0X0B,0XF5,0X4E,0XE5,0XDB,0XFF,0X29,0X32,0X77,0X6D,0XD9,0X5D,
0XD9,0X4D,0XB9,0X4D,0XFA,0X4D,0X19,0X56,0XC8,0X51,0XE7,0X59,0XBA,0XFF,0X2E,0XE5,
0X0B,0XFD,0XE9,0XFC,0X0A,0XFD,0X0A,0XFD,0X6D,0XF5,0X54,0XFF,0X2A,0XFD,0X09,0XFD,
0X2A,0XFD,0XE9,0XFC,0XE9,0XFC,0XEB,0XF4,0X97,0XFF,0X4E,0XE5,0XE9,0XF4,0X29,0XFD,
0X08,0XFD,0XE9,0XF4,0X2B,0XED,0X95,0XFF,0X2A,0XF5,0X29,0XFD,0X29,0XFD,0X29,0XFD,
0X08,0XFD,0X2A,0XF5,0X96,0XFF,0X4D,0XE5,0X2B,0XF5,0X09,0XFD,0X09,0XFD,0X2B,0XF5,
0X2E,0XE5,0X97,0XFF,0X2A,0XF5,0X09,0XFD,0X09,0XFD,0X09,0XFD,0X09,0XFD,0X0B,0XFD,
0X2E,0XE5,0XB9,0XFF,0XB9,0XFF,0X4E,0XE5,0X0B,0XFD,0X09,0XFD,0X08,0XFD,0X2B,0XED,
0XFC,0XFF,0XDB,0XFF,0X2C,0XF5,0XE8,0XFC,0X09,0XFD,0X29,0XFD,0X08,0XFD,0X08,0XFD,
0X4C,0XFD,0X66,0XC3,0X67,0XC3,0X68,0XC3,0X88,0XBB,0XAA,0XAB,0XDD,0XFF,0XFF,0XF7,
0X6A,0X22,0X77,0X6D,0XDA,0X55,0XDA,0X4D,0XDA,0X4D,0XDA,0X4D,0XFA,0X4D,0XB7,0X5D,
0X49,0X2A,0XFE,0XFF,0XBB,0XFF,0X4E,0XE5,0X0A,0XF5,0X29,0XFD,0X08,0XFD,0X2B,0XFD,
0X99,0XFF,0X99,0XFF,0X2B,0XF5,0X29,0XFD,0X08,0XFD,0X2A,0XF5,0X4E,0XE5,0XB9,0XFF,
0XB9,0XFF,0X2D,0XE5,0X2B,0XF5,0XE8,0XF4,0X29,0XFD,0X2A,0XFD,0X2D,0XF5,0X76,0XFF,
0X0B,0XFD,0XE9,0XFC,0X29,0XFD,0X2A,0XFD,0X87,0XC3,0XA9,0XB3,0XB8,0XFF,0X4D,0XE5,
0X0A,0XFD,0XE9,0XFC,0X2A,0XFD,0XE9,0XFC,0X29,0XFD,0X09,0XFD,0X86,0XBB,0X67,0XB3,
0X47,0XB3,0XA8,0XC3,0X67,0XBB,0X67,0XB3,0X98,0XFF,0X4E,0XED,0X0A,0XF5,0X09,0XFD,
0X29,0XFD,0X0A,0XF5,0X4E,0XE5,0XDB,0XFF,0X29,0X32,0X78,0X75,0XB9,0X5D,0XD9,0X55,
0X1A,0X56,0XD9,0X4D,0XD9,0X45,0XA8,0X51,0XC7,0X51,0X9B,0XFF,0X2E,0XE5,0X0B,0XFD,
0XE9,0XFC,0XE9,0XFC,0X2A,0XFD,0X0A,0XF5,0X2B,0XF5,0X2A,0XFD,0XE9,0XFC,0XE8,0XFC,
0X09,0XFD,0X09,0XFD,0X2B,0XFD,0X98,0XFF,0X4E,0XE5,0X0B,0XF5,0X09,0XFD,0X29,0XFD,
0X09,0XFD,0X4B,0XFD,0X0A,0XF5,0X29,0XFD,0X29,0XFD,0X29,0XFD,0X29,0XFD,0X09,0XFD,
0X2A,0XF5,0X97,0XFF,0X4E,0XE5,0X2B,0XF5,0X09,0XFD,0X09,0XFD,0X2B,0XF5,0X2E,0XE5,
0X97,0XFF,0X2A,0XF5,0X09,0XFD,0X09,0XFD,0X09,0XFD,0X09,0XFD,0X0B,0XFD,0X2E,0XE5,
0XB9,0XFF,0XB9,0XFF,0X4E,0XE5,0X0B,0XF5,0X09,0XFD,0X08,0XFD,0X6C,0XF5,0XFB,0XFF,
0XFC,0XFF,0X2D,0XED,0X0A,0XFD,0X09,0XFD,0X08,0XFD,0X48,0XFD,0X09,0XFD,0X2C,0XED,
0X98,0XFF,0X7A,0XFF,0X9B,0XFF,0XBB,0XFF,0XDC,0XFF,0XDE,0XFF,0XFF,0XE7,0X6B,0X22,
0XB8,0X6D,0XB9,0X55,0XFA,0X4D,0XDA,0X4D,0XFA,0X55,0XF9,0X4D,0XB7,0X65,0X6A,0X32,
0X9D,0XFF,0XBC,0XFF,0X4F,0XDD,0X2B,0XFD,0XE8,0XFC,0XE8,0XFC,0X0A,0XFD,0X0D,0XED,
0X2D,0XED,0X0A,0XFD,0XE8,0XFC,0X28,0XFD,0X0A,0XF5,0X6F,0XE5,0XB9,0XFF,0XB8,0XFF,
0X4D,0XE5,0X4B,0XF5,0X29,0XFD,0X29,0XFD,0XC8,0XF4,0X2B,0XFD,0X0C,0XFD,0XEB,0XFC,
0X0B,0XFD,0X0B,0XFD,0X86,0XBB,0X9A,0XFF,0X9B,0XFF,0X99,0XFF,0X6E,0XE5,0X0A,0XF5,
0X09,0XFD,0X09,0XFD,0X09,0XFD,0X08,0XFD,0X0A,0XF5,0X98,0XFF,0XBA,0XFF,0X9B,0XFF,
0X5A,0XFF,0X9B,0XFF,0X9A,0XFF,0X99,0XFF,0X2E,0XED,0X0A,0XF5,0X09,0XFD,0X29,0XFD,
0X2A,0XF5,0X4E,0XE5,0XDB,0XFF,0X29,0X32,0X78,0X75,0XB9,0X5D,0XFA,0X4D,0XB9,0X45,
0X1A,0X4E,0XF9,0X45,0XE9,0X51,0XC7,0X51,0XBD,0XFF,0X89,0XAB,0XEC,0XFC,0XEA,0XFC,
0X09,0XFD,0X08,0XFD,0X29,0XFD,0X29,0XFD,0X2A,0XFD,0X09,0XFD,0X29,0XFD,0X09,0XFD,
0X29,0XFD,0X2B,0XF5,0X98,0XFF,0XCA,0XAB,0XA7,0XBB,0X0B,0XF5,0X09,0XFD,0XE8,0XFC,
0X09,0XFD,0X09,0XFD,0X09,0XFD,0X09,0XFD,0X29,0XFD,0X29,0XFD,0X09,0XFD,0X2B,0XF5,
0X98,0XFF,0X4E,0XE5,0X2A,0XF5,0X09,0XFD,0X09,0XFD,0X2B,0XFD,0X2E,0XE5,0X97,0XFF,
0X2A,0XF5,0X09,0XFD,0X09,0XFD,0X09,0XFD,0X09,0XFD,0X2A,0XF5,0X4D,0XE5,0XB8,0XFF,
0XD9,0XFF,0X4E,0XDD,0X2B,0XF5,0X09,0XFD,0X09,0XFD,0X4C,0XED,0XBB,0XFF,0XBC,0XFF,
0XA9,0XB3,0X0B,0XFD,0X0A,0XFD,0X49,0XFD,0X28,0XFD,0X09,0XF5,0X4D,0XED,0XBA,0XFF,
0XC6,0X49,0X08,0X42,0X27,0X3A,0X48,0X32,0X4A,0X2A,0XAC,0X2A,0XAC,0X12,0X98,0X65,
0XD9,0X55,0XF9,0X4D,0XF9,0X45,0X19,0X4E,0XF9,0X45,0XB7,0X65,0X49,0X32,0XDE,0XFF,
0XBD,0XFF,0X8A,0XAB,0X0B,0XFD,0XE9,0XFC,0X29,0XFD,0X09,0XFD,0X2B,0XFD,0X2B,0XFD,
0XE9,0XFC,0X09,0XFD,0X08,0XFD,0X2B,0XF5,0X88,0XAB,0XBA,0XFF,0XB9,0XFF,0X6E,0XE5,
0X0A,0XF5,0X09,0XFD,0X09,0XFD,0X4A,0XFD,0X0A,0XFD,0X2A,0XFD,0XCB,0XFC,0X66,0XC3,
0X88,0XAB,0XBB,0XFF,0XBD,0XFF,0X9D,0XFF,0XDB,0XFF,0XC9,0XAB,0X0B,0XF5,0X0A,0XFD,
0X2A,0XFD,0X09,0XFD,0X09,0XFD,0X2C,0XF5,0X9A,0XFF,0XE7,0X51,0XE7,0X49,0XE8,0X51,
0XC7,0X51,0XA5,0X59,0X9B,0XFF,0X2F,0XE5,0X0B,0XFD,0X09,0XFD,0X29,0XFD,0X2A,0XF5,
0X4E,0XE5,0XFC,0XFF,0X69,0X32,0X57,0X65,0XD9,0X5D,0XD9,0X4D,0XF9,0X4D,0XFA,0X4D,
0XF9,0X4D,0X09,0X3A,0XE8,0X41,0XDE,0XFF,0X9B,0XFF,0X67,0XC3,0X64,0XD3,0XE9,0XFC,
0X09,0XFD,0X08,0XFD,0X29,0XFD,0XE9,0XFC,0XE9,0XFC,0X09,0XFD,0X29,0XFD,0X08,0XFD,
0X2B,0XF5,0XBA,0XFF,0X9B,0XFF,0X99,0XFF,0X87,0XBB,0X0A,0XFD,0XE9,0XFC,0X09,0XFD,
0X09,0XFD,0X09,0XFD,0X09,0XFD,0X09,0XFD,0X09,0XFD,0X09,0XFD,0X2B,0XF5,0X98,0XFF,
0X2E,0XE5,0X0A,0XFD,0X09,0XFD,0XE9,0XFC,0X0B,0XFD,0X2E,0XED,0X97,0XFF,0X0A,0XFD,
0X09,0XFD,0X09,0XFD,0X09,0XFD,0X09,0XFD,0X0A,0XFD,0X2D,0XED,0XB8,0XFF,0XB9,0XFF,
0X4E,0XDD,0X2B,0XF5,0X09,0XFD,0X09,0XFD,0X4C,0XED,0XDC,0XFF,0XDD,0XFF,0X9A,0XFF,
0X87,0XBB,0X0B,0XFD,0X09,0XFD,0X08,0XFD,0X2A,0XFD,0X2D,0XE5,0XDB,0XFF,0X28,0X42,
0X16,0X85,0X56,0X7D,0X76,0X75,0X77,0X75,0X77,0X6D,0X98,0X65,0XD9,0X5D,0XD9,0X55,
0XF9,0X4D,0XF9,0X4D,0XF9,0X45,0X19,0X4E,0XB7,0X5D,0X4A,0X32,0XDF,0XFF,0XBE,0XFF,
0X9B,0XFF,0X67,0XC3,0X43,0XCB,0X09,0XFD,0XE9,0XFC,0X09,0XFD,0X09,0XFD,0XE9,0XFC,
0XE9,0XFC,0X09,0XFD,0X65,0XBB,0XBA,0XFF,0XBC,0XFF,0XD9,0XFF,0X4E,0XE5,0X0A,0XF5,
0X09,0XFD,0X09,0XFD,0X09,0XFD,0X09,0XFD,0XEA,0XFC,0X66,0XCB,0X78,0XFF,0X9B,0XFF,
0XDE,0XFF,0X09,0X4A,0X09,0X4A,0XDD,0XFF,0X9A,0XFF,0X87,0XC3,0X64,0XCB,0XE9,0XFC,
0X09,0XFD,0XE9,0XFC,0X0D,0XF5,0XBC,0XFF,0XE8,0X41,0X0A,0X3A,0XF5,0X94,0XE9,0X41,
0XC7,0X51,0XBC,0XFF,0X2F,0XE5,0X0B,0XFD,0X09,0XFD,0X09,0XFD,0X0A,0XF5,0X4E,0XE5,
0XFC,0XFF,0X49,0X2A,0X98,0X6D,0XD8,0X55,0XF9,0X4D,0X1A,0X4E,0XD9,0X45,0X1A,0X56,
0X6B,0X22,0X4A,0X2A,0XFF,0XFF,0XBC,0XFF,0XA9,0XB3,0X45,0XC3,0XEA,0XFC,0XE9,0XFC,
0XE9,0XFC,0XE9,0XFC,0XE9,0XFC,0XE9,0XFC,0XE9,0XFC,0XE9,0XFC,0X08,0XFD,0X2B,0XF5,
0XBB,0XFF,0XDE,0XFF,0XBB,0XFF,0XA9,0XB3,0X0C,0XFD,0XEA,0XFC,0XC9,0XFC,0XC9,0XFC,
0XC9,0XFC,0XE9,0XFC,0XE9,0XFC,0XE9,0XFC,0XE9,0XFC,0X0B,0XFD,0X97,0XFF,0X2E,0XED,
0X0B,0XFD,0XE9,0XFC,0XEA,0XFC,0XEB,0XFC,0X2E,0XED,0X77,0XFF,0X0B,0XFD,0XE9,0XFC,
0XEA,0XFC,0XEA,0XFC,0XC9,0XFC,0XEA,0XFC,0X0D,0XF5,0X78,0XFF,0X98,0XFF,0X4E,0XE5,
0X0B,0XFD,0XE9,0XFC,0XE9,0XFC,0X2C,0XED,0XDC,0XFF,0XFE,0XFF,0XBC,0XFF,0XA9,0XB3,
0X0C,0XFD,0XC9,0XFC,0XE9,0XFC,0XEA,0XFC,0X2E,0XED,0XDC,0XFF,0X49,0X3A,0X57,0X7D,
0X98,0X65,0XB9,0X5D,0XB9,0X5D,0XD9,0X5D,0XD9,0X55,0XD9,0X55,0XD9,0X55,0XF9,0X4D,
0X19,0X4E,0XF8,0X45,0X19,0X46,0XD8,0X5D,0X6B,0X22,0XFF,0XF7,0XDF,0XFF,0XBD,0XFF,
0X89,0XB3,0X86,0XC3,0XEA,0XFC,0XEA,0XFC,0XC9,0XFC,0XE9,0XFC,0XE9,0XFC,0XEA,0XFC,
0XEA,0XFC,0XA8,0XBB,0XBC,0XFF,0XDD,0XFF,0XBA,0XFF,0X2E,0XED,0XEA,0XFC,0X0A,0XFD,
0XEA,0XFC,0XEA,0XFC,0XC9,0XFC,0XEB,0XFC,0X89,0XB3,0XBB,0XFF,0XDD,0XFF,0XFF,0XFF,
0X2A,0X32,0X2A,0X32,0XFF,0XFF,0XBC,0XFF,0XA9,0XB3,0X66,0XC3,0XEA,0XFC,0XE9,0XFC,
0XE9,0XFC,0X0D,0XF5,0XBC,0XFF,0X09,0X42,0X2A,0X2A,0X16,0X8D,0X09,0X42,0XE6,0X51,
0XBB,0XFF,0X2E,0XED,0X0A,0XFD,0XE8,0XFC,0XE9,0XFC,0X0B,0XFD,0X2E,0XED,0XBC,0XFF,
0X49,0X32,0X97,0X6D,0XD8,0X55,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0X77,0X65,
0X77,0X7D,0X2A,0X32,0XDE,0XFF,0XBB,0XFF,0X99,0XFF,0X67,0XBB,0X66,0XC3,0X67,0XC3,
0X87,0XC3,0X86,0XC3,0X66,0XC3,0X66,0XC3,0X66,0XC3,0X85,0XC3,0XA8,0XB3,0XBD,0XFF,
0XBF,0XFF,0XDD,0XFF,0X9A,0XFF,0X88,0XB3,0X67,0XBB,0X66,0XCB,0X66,0XCB,0X66,0XC3,
0X46,0XC3,0XA7,0XCB,0X66,0XC3,0X86,0XC3,0X67,0XBB,0X99,0XFF,0XCA,0XAB,0X87,0XBB,
0X66,0XC3,0X67,0XC3,0XA8,0XBB,0X89,0XAB,0X78,0XFF,0XC8,0XBB,0X86,0XBB,0X66,0XBB,
0X67,0XC3,0X87,0XC3,0X67,0XBB,0X89,0XB3,0X9A,0XFF,0XBB,0XFF,0XCA,0XAB,0X87,0XBB,
0X66,0XC3,0X87,0XC3,0X89,0XB3,0XDD,0XFF,0XFF,0XFF,0XBD,0XFF,0XBB,0XFF,0X68,0XBB,
0X87,0XCB,0X45,0XC3,0X67,0XBB,0XCB,0XAB,0XFD,0XFF,0X28,0X32,0X77,0X75,0XB9,0X5D,
0XDA,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,
0XF9,0X4D,0X19,0X46,0XD8,0X4D,0X97,0X6D,0X6A,0X2A,0X2A,0X32,0XFF,0XFF,0XBC,0XFF,
0X78,0XFF,0XA8,0XC3,0X87,0XC3,0X66,0XC3,0X86,0XC3,0X46,0XC3,0X87,0XC3,0XA8,0XBB,
0X99,0XFF,0XDE,0XFF,0XBE,0XFF,0XBC,0XFF,0XCA,0XAB,0X67,0XB3,0X87,0XC3,0X87,0XC3,
0X66,0XC3,0X66,0XCB,0X88,0XBB,0X9B,0XFF,0XBD,0XFF,0XFF,0XFF,0X4A,0X32,0X37,0X75,
0X57,0X7D,0X29,0X32,0XDE,0XFF,0XBC,0XFF,0X99,0XFF,0X67,0XC3,0X66,0XCB,0X66,0XC3,
0XA9,0XB3,0X9D,0XFF,0X2A,0X4A,0X4B,0X2A,0X56,0X8D,0X09,0X42,0XC6,0X49,0XBC,0XFF,
0XA9,0XA3,0X86,0XBB,0XA6,0XC3,0XA6,0XC3,0X67,0XBB,0X89,0XAB,0XDD,0XFF,0X49,0X32,
0X77,0X6D,0XB8,0X55,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XFA,0X5D,0X78,0X65,
0X58,0X85,0X4A,0X32,0XFE,0XFF,0XFD,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,
0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBC,0XFF,0XBD,0XFF,0XDF,0XFF,0X29,0X4A,
0XE7,0X49,0XDD,0XFF,0XDC,0XFF,0XBC,0XFF,0XBC,0XFF,0X9B,0XFF,0XBC,0XFF,0XBC,0XFF,
0XBC,0XFF,0XBC,0XFF,0XBB,0XFF,0XBC,0XFF,0XDD,0XFF,0XDD,0XFF,0XBC,0XFF,0XDB,0XFF,
0XBC,0XFF,0XBC,0XFF,0XDD,0XFF,0XFD,0XFF,0XBB,0XFF,0XDB,0XFF,0XDC,0XFF,0XDC,0XFF,
0XBB,0XFF,0XDC,0XFF,0XBD,0XFF,0XDF,0XFF,0XFF,0XFF,0XBD,0XFF,0XDC,0XFF,0XBB,0XFF,
0XBB,0XFF,0XBD,0XFF,0XBF,0XFF,0XDF,0XFF,0X28,0X52,0XDD,0XFF,0XDC,0XFF,0XDC,0XFF,
0XDC,0XFF,0XDD,0XFF,0XBC,0XFF,0XFF,0XFF,0X6A,0X2A,0X98,0X6D,0XBA,0X55,0XFA,0X4D,
0X19,0X46,0X19,0X46,0X19,0X46,0XFA,0X45,0XFA,0X4D,0XFA,0X4D,0XD9,0X4D,0XD9,0X4D,
0XFA,0X4D,0XF9,0X4D,0XD8,0X5D,0XB8,0X6D,0X57,0X7D,0X4A,0X32,0XFF,0XFF,0XDD,0XFF,
0XBC,0XFF,0XBC,0XFF,0XBD,0XFF,0XDC,0XFF,0XDC,0XFF,0XDC,0XFF,0XDC,0XFF,0XDD,0XFF,
0XE8,0X49,0XDF,0XFF,0XBE,0XFF,0XFD,0XFF,0XDC,0XFF,0XDC,0XFF,0XBC,0XFF,0XDC,0XFF,
0XBB,0XFF,0XDC,0XFF,0XDE,0XFF,0XE8,0X41,0X6A,0X3A,0X36,0X7D,0XD9,0X65,0X98,0X5D,
0X78,0X7D,0X4A,0X32,0XDE,0XFF,0XDD,0XFF,0XBD,0XFF,0XBC,0XFF,0X9B,0XFF,0XDD,0XFF,
0XDF,0XFF,0X0A,0X3A,0X2A,0X2A,0X15,0X8D,0XE9,0X41,0XE8,0X51,0XDF,0XFF,0XDE,0XFF,
0XDC,0XFF,0XBB,0XFF,0XBB,0XFF,0XDC,0XFF,0XDD,0XFF,0XFF,0XFF,0X4A,0X2A,0X77,0X75,
0XB8,0X5D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XDA,0X4D,0XB9,0X55,0X99,0X6D,
0X6C,0X22,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XBF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XDF,0XFF,0XDF,0XFF,0XDF,0XFF,0XC8,0X41,0XE8,0X49,
0XDF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XDF,0XFF,0XDF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XDF,0XFF,
0XDF,0XFF,0XDF,0XFF,0XDF,0XFF,0XC8,0X49,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0X4B,0X22,0X78,0X6D,0XBA,0X55,0XFA,0X4D,0X19,0X46,
0X19,0X46,0X1A,0X46,0XFA,0X45,0XFA,0X4D,0XFA,0X4D,0XFA,0X4D,0X1A,0X56,0XFA,0X4D,
0XFA,0X4D,0X19,0X56,0X98,0X55,0XB8,0X6D,0X6B,0X1A,0XFF,0XF7,0XFF,0XFF,0XBF,0XFF,
0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XE9,0X51,
0XBF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,
0XFE,0XFF,0XFF,0XFF,0X2A,0X32,0X6B,0X1A,0X98,0X65,0XD9,0X4D,0XB9,0X4D,0XB9,0X65,
0X6C,0X22,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,
0X2A,0X42,0X4B,0X32,0X15,0X8D,0X0A,0X4A,0XC8,0X49,0XDF,0XFF,0XFF,0XFF,0XDF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0X4B,0X2A,0X77,0X75,0XB8,0X5D,
0XF9,0X4D,0XD9,0X4D,0XFA,0X4D,0XF9,0X4D,0XF9,0X4D,0XF9,0X4D,0XD9,0X55,0X77,0X6D,
0X29,0X3A,0XC8,0X49,0XC9,0X49,0XEA,0X51,0XC9,0X49,0XC9,0X51,0XC9,0X49,0XC9,0X49,
0XC8,0X49,0XE8,0X49,0XE9,0X49,0XC9,0X49,0XC9,0X49,0XC9,0X49,0XE9,0X49,0XC8,0X49,
0XE9,0X49,0XE9,0X49,0XC8,0X49,0XE8,0X49,0XE9,0X49,0XC8,0X41,0XC8,0X49,0XE8,0X49,
0XC8,0X49,0XC8,0X49,0XA8,0X49,0XE9,0X51,0XE8,0X49,0XC8,0X49,0XC8,0X49,0X09,0X52,
0XE9,0X49,0XC7,0X41,0XC7,0X49,0XC7,0X49,0XC8,0X49,0XE9,0X51,0XC9,0X49,0XA9,0X49,
0XA9,0X49,0XC9,0X51,0XC8,0X49,0XC8,0X49,0XC8,0X49,0XE9,0X49,0XA8,0X49,0XC9,0X49,
0XCA,0X51,0XA9,0X49,0XC9,0X51,0XC7,0X49,0X08,0X52,0XC7,0X49,0XC8,0X49,0XC8,0X49,
0XC8,0X51,0XE9,0X41,0X2A,0X2A,0X98,0X75,0XB9,0X5D,0XFA,0X4D,0X19,0X46,0X19,0X46,
0XF9,0X4D,0XFA,0X4D,0XDA,0X4D,0XFA,0X4D,0XFA,0X4D,0XF9,0X45,0XD9,0X45,0X1A,0X4E,
0XD9,0X45,0XF9,0X4D,0XD9,0X55,0X98,0X6D,0X2A,0X3A,0XC8,0X49,0XE9,0X51,0XC9,0X49,
0XC9,0X49,0XC8,0X49,0XE9,0X49,0XC8,0X49,0XC8,0X49,0XE9,0X51,0X88,0X51,0XA8,0X51,
0XC8,0X51,0XE9,0X51,0XC8,0X49,0XE8,0X51,0XC9,0X49,0XE9,0X51,0XA7,0X49,0XE8,0X41,
0X4A,0X3A,0X57,0X75,0X98,0X5D,0XFA,0X55,0XD9,0X4D,0XFA,0X4D,0XD9,0X55,0X77,0X6D,
0X2A,0X3A,0XC9,0X49,0XC9,0X49,0XE9,0X51,0XC8,0X49,0XC8,0X49,0XE9,0X51,0XC9,0X39,
0X2B,0X2A,0X16,0X8D,0XE9,0X41,0XC8,0X49,0XC8,0X49,0X09,0X4A,0XE8,0X41,0XE8,0X49,
0XE8,0X49,0XC8,0X49,0XEA,0X51,0XEA,0X41,0X4B,0X2A,0X78,0X75,0XB9,0X55,0XFA,0X4D,
0XFA,0X45,0XFA,0X45,0XFA,0X45,};
#line 14 "app\\game.c"
#line 1 ".\\app\\RES\\Pic\\flappybird_score_board.c"
const unsigned char gImage_flappybird_score_board[25600] = {  
0XD8,0X5D,0XD8,0X5D,0X98,0X65,0X57,0X6D,0X36,0X7D,0X14,0X95,0X06,0X42,0X25,0X52,
0X03,0X52,0X03,0X52,0X04,0X52,0X05,0X4A,0X04,0X4A,0X04,0X4A,0XE4,0X51,0XE4,0X51,
0XE4,0X51,0X04,0X52,0X24,0X4A,0X24,0X4A,0X04,0X52,0X04,0X52,0X04,0X4A,0X04,0X4A,
0X24,0X4A,0X04,0X4A,0X04,0X52,0X04,0X52,0X04,0X4A,0X04,0X4A,0X04,0X52,0X04,0X52,
0X04,0X52,0X04,0X52,0X04,0X52,0X04,0X52,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,
0X04,0X52,0X04,0X52,0X24,0X4A,0X04,0X4A,0X04,0X52,0XE4,0X51,0X04,0X52,0X04,0X52,
0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,
0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,
0X03,0X4A,0X03,0X4A,0X03,0X4A,0X03,0X4A,0X03,0X4A,0X03,0X4A,0X03,0X4A,0X03,0X4A,
0X03,0X4A,0X03,0X4A,0X03,0X4A,0X03,0X4A,0X03,0X4A,0X03,0X4A,0X03,0X4A,0X03,0X4A,
0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,
0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,
0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,
0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,
0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X52,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,
0X04,0X52,0X04,0X52,0X04,0X52,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,
0X24,0X4A,0X24,0X4A,0X24,0X4A,0X24,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,
0X04,0X4A,0X04,0X4A,0X24,0X4A,0X24,0X4A,0X24,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X52,
0XE4,0X51,0XE4,0X51,0X05,0X4A,0X05,0X4A,0X05,0X4A,0X04,0X4A,0XE4,0X51,0X04,0X52,
0X24,0X52,0X05,0X42,0X14,0X95,0X36,0X7D,0X57,0X6D,0X98,0X65,0XB8,0X5D,0XB9,0X5D,
0XB8,0X5D,0XB8,0X65,0X77,0X65,0XAB,0X1A,0X49,0X2A,0X27,0X32,0X39,0XE7,0X17,0XEF,
0X37,0XF7,0X37,0XF7,0X38,0XF7,0X38,0XF7,0X38,0XF7,0X38,0XF7,0X38,0XF7,0X38,0XF7,
0X37,0XF7,0X37,0XF7,0X57,0XEF,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X57,0XEF,
0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X57,0XEF,0X37,0XF7,0X37,0XF7,
0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X57,0XEF,0X57,0XEF,0X37,0XF7,
0X37,0XF7,0X37,0XF7,0X57,0XEF,0X57,0XEF,0X37,0XF7,0X17,0XF7,0X37,0XF7,0X37,0XF7,
0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,
0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,
0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,
0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,
0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,
0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,
0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,
0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,
0X57,0XEF,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,
0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,
0X37,0XF7,0X37,0XF7,0X57,0XEF,0X57,0XEF,0X37,0XF7,0X37,0XF7,0X37,0XF7,0X37,0XF7,
0X37,0XF7,0X37,0XF7,0X57,0XEF,0X57,0XEF,0X57,0XEF,0X37,0XF7,0X37,0XF7,0X37,0XF7,
0X37,0XF7,0X38,0XF7,0X38,0XF7,0X38,0XF7,0X38,0XF7,0X38,0XF7,0X37,0XF7,0X37,0XF7,
0X17,0XEF,0X39,0XEF,0X27,0X32,0X49,0X2A,0XAB,0X1A,0X97,0X65,0XB8,0X65,0XB8,0X5D,
0X98,0X65,0X77,0X65,0X55,0X75,0X89,0X2A,0X47,0X3A,0X45,0X42,0X57,0XEF,0X55,0XF7,
0X55,0XF7,0X55,0XF7,0X55,0XEF,0X55,0XEF,0X55,0XF7,0X35,0XF7,0X35,0XF7,0X35,0XF7,
0X35,0XF7,0X35,0XF7,0X35,0XF7,0X35,0XF7,0X35,0XF7,0X35,0XF7,0X35,0XF7,0X35,0XF7,
0X35,0XF7,0X35,0XF7,0X15,0XF7,0X35,0XF7,0X35,0XF7,0X35,0XF7,0X35,0XF7,0X35,0XF7,
0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X56,0XEF,0X56,0XEF,0X56,0XEF,0X36,0XEF,
0X36,0XF7,0X36,0XF7,0X56,0XEF,0X56,0XEF,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,
0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,
0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,
0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,
0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,
0X35,0XF7,0X35,0XF7,0X35,0XF7,0X35,0XF7,0X35,0XF7,0X35,0XF7,0X35,0XF7,0X35,0XF7,
0X35,0XF7,0X35,0XF7,0X35,0XF7,0X35,0XF7,0X35,0XF7,0X35,0XF7,0X35,0XF7,0X35,0XF7,
0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,
0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,
0X36,0XEF,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,
0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X36,0XF7,0X35,0XF7,
0X35,0XF7,0X35,0XF7,0X35,0XF7,0X35,0XF7,0X55,0XEF,0X35,0XF7,0X35,0XF7,0X35,0XF7,
0X35,0XF7,0X35,0XF7,0X55,0XEF,0X55,0XEF,0X55,0XEF,0X35,0XF7,0X35,0XF7,0X35,0XF7,
0X55,0XF7,0X35,0XF7,0X35,0XF7,0X35,0XF7,0X55,0XEF,0X55,0XEF,0X35,0XF7,0X55,0XEF,
0X55,0XF7,0X56,0XEF,0X45,0X42,0X47,0X3A,0X89,0X2A,0X55,0X75,0X77,0X65,0X98,0X65,
0X57,0X6D,0XAB,0X1A,0X69,0X2A,0X3A,0XD7,0X59,0XE7,0X16,0XEF,0XB3,0XDE,0XB2,0XDE,
0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,0XB1,0XDE,0XB1,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0X92,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,
0XB2,0XE6,0XB2,0XE6,0X92,0XE6,0X92,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB3,0XDE,0XB3,0XDE,0XB3,0XDE,0XB3,0XDE,0XB3,0XDE,0XB3,0XDE,0XB3,0XDE,0XB3,0XDE,
0XB3,0XE6,0XB3,0XE6,0XB3,0XE6,0XB3,0XE6,0XB3,0XE6,0XB3,0XE6,0XB3,0XE6,0XB3,0XE6,
0XB3,0XDE,0XB3,0XDE,0XB3,0XE6,0XB3,0XE6,0XB3,0XE6,0XB3,0XE6,0XB3,0XE6,0XB3,0XE6,
0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,
0XB2,0XE6,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,
0XD2,0XE6,0XD1,0XDE,0XD2,0XE6,0XB2,0XE6,0XB2,0XDE,0XD3,0XDE,0XD3,0XDE,0XD2,0XDE,
0XB1,0XDE,0XB2,0XDE,0X16,0XE7,0X59,0XE7,0X3A,0XD7,0X68,0X2A,0XAB,0X1A,0X57,0X6D,
0X16,0X7D,0X6A,0X2A,0X27,0X32,0XD7,0XD6,0X95,0XDE,0X94,0XE6,0XB3,0XE6,0XB3,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0X92,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0X92,0XE6,0X92,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0XB3,0XDE,0XB3,0XDE,
0XD2,0XDE,0XB2,0XDE,0XB4,0XE6,0XB5,0XDE,0XD7,0XD6,0X27,0X32,0X4A,0X2A,0X16,0X7D,
0XF4,0X8C,0X27,0X32,0X25,0X42,0X95,0XD6,0X94,0XE6,0X72,0XE6,0XB3,0XE6,0X92,0XDE,
0XB3,0XDE,0XB3,0XDE,0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD3,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XE6,0XB2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0XB3,0XDE,
0XB2,0XDE,0XB2,0XE6,0X72,0XE6,0X93,0XE6,0X94,0XD6,0X45,0X42,0X28,0X32,0XF4,0X8C,
0X26,0X42,0X39,0XE7,0X57,0XEF,0XB3,0XDE,0XD3,0XE6,0XB2,0XE6,0XB2,0XE6,0XB3,0XE6,
0X92,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0X92,0XE6,0X92,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XE6,
0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,
0XB2,0XE6,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XD3,0XE6,0XB2,0XDE,0X56,0XEF,0X39,0XEF,0X26,0X42,
0X04,0X4A,0XB5,0XDE,0XB3,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XE6,0X92,0XE6,0X93,0XE6,
0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XD1,0XDE,0XD1,0XDE,0XD2,0XDE,0XB2,0XDE,0XB3,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XD2,0XDE,
0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XE6,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XE6,
0XB2,0XE6,0XB2,0XE6,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,
0XB2,0XE6,0XB2,0XE6,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XE6,0XB2,0XE6,
0XB3,0XE6,0XB3,0XDE,0XD2,0XDE,0XD2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB2,0XE6,0XB3,0XE6,
0XB2,0XE6,0XB2,0XDE,0XB2,0XE6,0XB2,0XDE,0XB1,0XDE,0XB3,0XDE,0XB5,0XDE,0X04,0X4A,
0X03,0X52,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB3,0XDE,0X93,0XE6,0X93,0XE6,
0XB2,0XE6,0XB2,0XE6,0XB1,0XDE,0XD1,0XDE,0XD1,0XDE,0XD1,0XDE,0XB2,0XDE,0XB2,0XE6,
0XB2,0XE6,0XB1,0XE6,0XB1,0XDE,0XD2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD1,0XDE,
0XD2,0XDE,0XD1,0XDE,0XB1,0XDE,0XD1,0XDE,0XD2,0XE6,0XB1,0XDE,0XB1,0XDE,0XD2,0XE6,
0XD2,0XDE,0XB1,0XDE,0XB2,0XE6,0XB1,0XDE,0XD2,0XE6,0XB1,0XDE,0XB1,0XDE,0XB2,0XE6,
0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XE6,0XB2,0XDE,0XD1,0XDE,0XF2,0XE6,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB1,0XDE,0XD2,0XE6,0XB2,0XDE,0XB1,0XDE,0XD2,0XDE,0XD1,0XDE,0XB2,0XDE,0XB1,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB1,0XDE,0XD2,0XE6,0XB2,0XDE,0XB1,0XDE,0XD2,0XE6,0X91,0XDE,
0XD2,0XE6,0XB1,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XD2,0XE6,0X91,0XDE,0XD2,0XE6,
0XB2,0XDE,0XB1,0XDE,0XD2,0XDE,0XB1,0XDE,0XB1,0XDE,0XD2,0XDE,0XD2,0XE6,0XB2,0XE6,
0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XD1,0XDE,0XD1,0XDE,0XB1,0XDE,0XB2,0XE6,0XB2,0XE6,
0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0XB2,0XDE,0XD2,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X52,
0X03,0X52,0X95,0XDE,0XB3,0XDE,0XD3,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0XB3,0XE6,
0XB2,0XE6,0XB1,0XDE,0XD1,0XDE,0XD1,0XDE,0XD1,0XDE,0XD1,0XDE,0XB2,0XDE,0XB2,0XE6,
0XB2,0XE6,0XB2,0XE6,0XD2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XD2,0XDE,
0XB2,0XDE,0XF2,0XDE,0XD2,0XDE,0XB1,0XDE,0XD2,0XE6,0XB2,0XDE,0XD2,0XE6,0X91,0XDE,
0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XE6,0XB1,0XDE,0XB2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XD2,0XE6,0XB2,0XDE,0X91,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XDE,
0XB1,0XDE,0XD2,0XE6,0XB2,0XDE,0XB2,0XDE,0XD2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,
0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XE6,0XB2,0XE6,0XB2,0XDE,
0XD2,0XDE,0XD2,0XDE,0X91,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,
0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XD2,0XDE,0XD1,0XDE,0XD1,0XDE,0XB2,0XDE,0XB2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0XD2,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X52,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD1,0XDE,0XD1,0XDE,0XD1,0XDE,0XD1,0XDE,0XD1,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,
0XB2,0XE6,0X92,0XE6,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XE6,0XB2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0X92,0XE6,0XB2,0XE6,
0XB2,0XDE,0X91,0XDE,0XD2,0XE6,0XD2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XD2,0XE6,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XD2,0XE6,0XB2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB1,0XD6,0XF2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XE6,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XE6,
0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XD1,0XDE,0XD1,0XDE,0XD1,0XDE,0XD1,0XDE,
0XB1,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,
0XD1,0XDE,0XD1,0XDE,0XD1,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,
0X92,0XE6,0XB2,0XE6,0XB3,0XE6,0X92,0XE6,0XB3,0XE6,0X92,0XDE,0XB2,0XDE,0XD2,0XE6,
0XD3,0XE6,0X92,0XDE,0XD3,0XEE,0X92,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0X92,0XE6,
0XB3,0XE6,0XB2,0XE6,0X92,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0X91,0XDE,
0XD3,0XE6,0XB2,0XE6,0X92,0XDE,0XB2,0XE6,0X92,0XDE,0XD2,0XE6,0X92,0XDE,0XB2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD3,0XE6,0XB2,0XDE,0XB2,0XE6,0X92,0XE6,0X92,0XE6,0X92,0XE6,0X92,0XE6,0XB2,0XDE,
0X92,0XDE,0X91,0XDE,0XB2,0XE6,0XB2,0XE6,0X92,0XE6,0XB2,0XE6,0XD2,0XE6,0XB2,0XDE,
0XB2,0XE6,0X92,0XE6,0XB3,0XE6,0XB2,0XE6,0X92,0XDE,0X92,0XE6,0XB2,0XE6,0X92,0XE6,
0XB3,0XEE,0X91,0XDE,0XB2,0XE6,0X92,0XDE,0X92,0XE6,0XB2,0XEE,0X92,0XE6,0X92,0XE6,
0X92,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XD2,0XDE,0XD1,0XDE,0XD1,0XDE,0XD1,0XDE,
0XD1,0XDE,0XD1,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB3,0XDE,0X95,0XDE,0X05,0X4A,
0X05,0X4A,0X95,0XDE,0XB3,0XDE,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD1,0XDE,0XD1,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XE6,0XB2,0XE6,0X72,0XEE,0X92,0XF6,0X92,0XEE,0X92,0XE6,0XB2,0XE6,0XB2,0XE6,
0X71,0XE6,0X92,0XEE,0X71,0XE6,0X92,0XEE,0X92,0XE6,0X92,0XE6,0XB2,0XEE,0X91,0XE6,
0X92,0XEE,0XB2,0XEE,0X92,0XE6,0XB2,0XE6,0X92,0XEE,0X71,0XE6,0XB2,0XE6,0XD2,0XE6,
0X92,0XE6,0X92,0XE6,0XB2,0XEE,0XB2,0XEE,0X92,0XE6,0XB2,0XE6,0X92,0XEE,0X72,0XE6,
0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0X71,0XDE,0XB2,0XEE,0X51,0XEE,0X72,0XF6,0X52,0XF6,0X51,0XF6,0X92,0XE6,0X91,0XE6,
0X92,0XEE,0X72,0XEE,0X52,0XEE,0X31,0XEE,0X93,0XF6,0X71,0XEE,0X92,0XEE,0XB2,0XEE,
0X92,0XEE,0X71,0XEE,0X72,0XEE,0X92,0XEE,0X71,0XE6,0XB2,0XEE,0X72,0XF6,0X52,0XF6,
0X73,0XFE,0X51,0XEE,0XB3,0XF6,0X92,0XEE,0X92,0XF6,0X31,0XEE,0X93,0XFE,0X51,0XEE,
0X92,0XE6,0XB2,0XE6,0XB3,0XE6,0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD1,0XDE,
0XB1,0XDE,0XD1,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB3,0XDE,0X95,0XDE,0X05,0X4A,
0X05,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,
0XB2,0XDE,0X92,0XE6,0XCC,0XCC,0X6B,0XC4,0X52,0XF6,0X92,0XF6,0X91,0XE6,0XB2,0XEE,
0XEC,0XBC,0X72,0XF6,0X72,0XF6,0XAB,0XBC,0X0C,0XC5,0X0C,0XBD,0XCB,0XB4,0X92,0XEE,
0X72,0XEE,0XCB,0XBC,0X2C,0XBD,0XEB,0XB4,0XEB,0XBC,0XB2,0XF6,0X92,0XEE,0X91,0XE6,
0X71,0XE6,0XEC,0XBC,0XCB,0XBC,0XCB,0XBC,0X92,0XEE,0X51,0XEE,0X72,0XEE,0XEC,0XBC,
0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,
0XB2,0XE6,0X71,0XEE,0X72,0XF6,0XAB,0XC4,0X8B,0XCC,0XCC,0XC4,0XEC,0XBC,0X92,0XEE,
0X52,0XFE,0X32,0XFE,0XCC,0XCC,0XEC,0XC4,0X8A,0XBC,0X72,0XF6,0X51,0XEE,0XEC,0XBC,
0X0C,0XC5,0XCB,0XC4,0X32,0XFE,0X52,0XFE,0X92,0XF6,0XAB,0XBC,0XCC,0XCC,0X8B,0XC4,
0X8B,0XC4,0X52,0XFE,0X31,0XEE,0X72,0XF6,0XAB,0XCC,0X8B,0XCC,0X8B,0XCC,0XCC,0XC4,
0X92,0XE6,0XB2,0XDE,0XB3,0XE6,0XB3,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,
0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD1,0XDE,0XB3,0XDE,0X95,0XDE,0X05,0X4A,
0XE4,0X51,0X95,0XE6,0XB2,0XE6,0XD1,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,
0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XD2,0XDE,0XD1,0XDE,0XD1,0XDE,
0XB2,0XE6,0X72,0XEE,0X8C,0XD4,0X6C,0XDC,0X6B,0XCC,0X52,0XFE,0X72,0XF6,0XAB,0XBC,
0X8C,0XCC,0XF1,0XFD,0X32,0XFE,0XAC,0XCC,0X73,0XFE,0X93,0XFE,0X14,0XFF,0X72,0XF6,
0X52,0XF6,0X8B,0XBC,0XF4,0XFE,0XF4,0XFE,0XB3,0XFE,0XCC,0XC4,0X92,0XF6,0X72,0XEE,
0XEC,0XC4,0XD4,0XFE,0XF4,0XFE,0XF4,0XFE,0XCC,0XC4,0X72,0XF6,0X11,0XF6,0X0C,0XC5,
0X92,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,
0X72,0XE6,0XEC,0XBC,0X8B,0XBC,0XB3,0XFE,0XD4,0XFE,0XD4,0XFE,0XD3,0XFE,0X72,0XF6,
0X6B,0XD4,0X6C,0XD4,0XD4,0XFE,0XF4,0XFE,0X34,0XFF,0X92,0XEE,0XAB,0XC4,0XD4,0XFE,
0XF4,0XFE,0XD3,0XFE,0XAC,0XDC,0X6B,0XDC,0XF1,0XFD,0XAC,0XCC,0X8B,0XCC,0XB3,0XFE,
0XB3,0XFE,0XAB,0XCC,0X8B,0XCC,0XF1,0XFD,0X4C,0XE4,0X6C,0XE4,0X73,0XFE,0XF4,0XFE,
0XB1,0XE6,0XD2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD1,0XDE,0XB2,0XDE,0X95,0XE6,0XE5,0X51,
0XE4,0X51,0X95,0XE6,0XB3,0XE6,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,
0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XD2,0XDE,0XD1,0XDE,0XD1,0XDE,
0XB2,0XE6,0X72,0XF6,0X4B,0XD4,0X4C,0XE4,0X73,0XFE,0XAC,0XCC,0XCC,0XC4,0XB4,0XFE,
0X8C,0XD4,0XD1,0XFD,0XF1,0XFD,0X6B,0XCC,0XAC,0XD4,0XAC,0XCC,0XCC,0XC4,0X52,0XF6,
0X32,0XFE,0XAB,0XC4,0X72,0XFE,0X31,0XF6,0X52,0XFE,0XAB,0XC4,0X52,0XEE,0X72,0XF6,
0XAC,0XC4,0X32,0XFE,0X32,0XFE,0X52,0XFE,0X8B,0XC4,0X52,0XFE,0X32,0XFE,0XAB,0XC4,
0X92,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,
0X92,0XEE,0X14,0XFF,0XD4,0XFE,0XAC,0XC4,0X8B,0XBC,0XCC,0XC4,0X93,0XFE,0X31,0XF6,
0X6C,0XDC,0X6C,0XDC,0X52,0XFE,0X72,0XEE,0XB2,0XEE,0X92,0XEE,0X8B,0XC4,0X32,0XFE,
0X51,0XF6,0X52,0XFE,0X4B,0XDC,0X2B,0XE4,0XF1,0XFD,0X6B,0XCC,0X8B,0XD4,0X12,0XFE,
0X11,0XFE,0X8B,0XCC,0XAC,0XD4,0XD1,0XFD,0X0B,0XEC,0X2B,0XE4,0X8B,0XD4,0XCB,0XBC,
0XB1,0XE6,0XD1,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB3,0XDE,0X95,0XE6,0XE4,0X51,
0X04,0X52,0X95,0XDE,0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,
0X72,0XE6,0X72,0XF6,0X6C,0XDC,0X2B,0XDC,0X94,0XFE,0XAB,0XC4,0XAB,0XBC,0XD4,0XFE,
0X6B,0XD4,0XF1,0XFD,0X12,0XFE,0X8B,0XCC,0X8B,0XCC,0XAB,0XC4,0XCC,0XC4,0X52,0XF6,
0X52,0XFE,0XAC,0XCC,0X52,0XFE,0X52,0XF6,0X72,0XFE,0XCB,0XBC,0X72,0XEE,0X72,0XF6,
0X8B,0XCC,0XF1,0XFD,0X32,0XFE,0X12,0XFE,0X8B,0XCC,0X32,0XFE,0X11,0XFE,0XAC,0XCC,
0X92,0XEE,0X92,0XE6,0XB2,0XE6,0XB2,0XDE,0XD2,0XE6,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XE6,0X14,0XFF,0X15,0XFF,0XAB,0XC4,0XCC,0XCC,0XAB,0XC4,0X52,0XFE,0X12,0XFE,
0X4C,0XDC,0X8C,0XDC,0X11,0XF6,0X92,0XF6,0X72,0XE6,0X51,0XE6,0XCC,0XC4,0X52,0XFE,
0X52,0XFE,0X32,0XFE,0X2B,0XDC,0X2C,0XE4,0XD1,0XFD,0X6B,0XD4,0X4B,0XDC,0XD1,0XFD,
0XF1,0XFD,0X8C,0XD4,0X8C,0XCC,0XF2,0XFD,0XEB,0XEB,0X4C,0XF4,0X6B,0XCC,0XCB,0XBC,
0X92,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X52,
0X03,0X4A,0X95,0XDE,0XB3,0XDE,0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,
0X92,0XEE,0X72,0XFE,0X4B,0XD4,0X6C,0XDC,0X11,0XF6,0XF4,0XFE,0X15,0XFF,0X31,0XF6,
0XAC,0XD4,0XF1,0XFD,0X12,0XFE,0X6B,0XCC,0X94,0XFE,0X73,0XFE,0XB4,0XFE,0X32,0XFE,
0X11,0XFE,0X8C,0XCC,0X12,0XFE,0X52,0XFE,0X31,0XF6,0XAB,0XBC,0X92,0XF6,0X72,0XF6,
0XAC,0XCC,0X6B,0XCC,0X4B,0XCC,0X8C,0XCC,0X8B,0XCC,0X32,0XFE,0XF1,0XFD,0XAC,0XD4,
0X32,0XF6,0X92,0XF6,0X51,0XEE,0XB2,0XEE,0X92,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,
0X92,0XE6,0X93,0XF6,0X11,0XFE,0X73,0XFE,0X73,0XFE,0X53,0XFE,0X8B,0XCC,0X52,0XFE,
0X6B,0XCC,0X8C,0XD4,0X12,0XFE,0X52,0XFE,0X92,0XF6,0X72,0XF6,0XAB,0XC4,0X32,0XFE,
0X32,0XFE,0X11,0XFE,0X8C,0XDC,0X6C,0XDC,0XF1,0XFD,0X8C,0XD4,0X4B,0XDC,0X4B,0XDC,
0X6B,0XD4,0X93,0XFE,0X94,0XFE,0XF2,0XFD,0X0C,0XEC,0XCB,0XE3,0X33,0XFE,0X93,0XFE,
0X92,0XEE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X03,0X4A,0X95,0XDE,0XB3,0XDE,0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,
0X71,0XE6,0X72,0XFE,0X4B,0XD4,0X6C,0XDC,0X32,0XF6,0X92,0XEE,0X92,0XE6,0X92,0XEE,
0XCC,0XC4,0X32,0XFE,0X12,0XFE,0X6C,0XD4,0X2B,0XDC,0X0B,0XDC,0X8C,0XDC,0X12,0XFE,
0X32,0XFE,0X6B,0XD4,0X4C,0XDC,0X4B,0XD4,0X8C,0XD4,0XD4,0XFE,0X72,0XF6,0X72,0XEE,
0XAB,0XBC,0XD4,0XFE,0X15,0XFF,0XF4,0XFE,0XCC,0XCC,0X12,0XFE,0XF1,0XFD,0X4B,0XD4,
0X4B,0XD4,0X8C,0XD4,0X8B,0XCC,0X31,0XF6,0X72,0XEE,0XB2,0XE6,0XD2,0XDE,0XF2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0X92,0XE6,
0X72,0XF6,0X6B,0XCC,0X4C,0XE4,0X2C,0XEC,0X2B,0XE4,0X6C,0XDC,0XF5,0XFE,0X72,0XEE,
0XF4,0XFE,0XB4,0XFE,0X8C,0XD4,0X8C,0XD4,0X6B,0XCC,0X52,0XFE,0XB4,0XFE,0X8B,0XCC,
0X6B,0XD4,0XAC,0XD4,0XB4,0XFE,0XF4,0XFE,0X32,0XFE,0XAC,0XD4,0X4B,0XCC,0X94,0XFE,
0XB3,0XFE,0XAC,0XC4,0X8B,0XCC,0XF2,0XFD,0X0B,0XE4,0X4D,0XF4,0X0B,0XE4,0X4B,0XD4,
0X72,0XEE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X03,0X4A,0X95,0XDE,0XB3,0XDE,0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0X92,0XE6,0X51,0XEE,0XCC,0XCC,0XAC,0XCC,0X52,0XEE,0X92,0XE6,0XB2,0XE6,0X92,0XE6,
0XCB,0XBC,0X52,0XF6,0X52,0XFE,0X8C,0XCC,0X8C,0XDC,0X6C,0XDC,0XAC,0XD4,0X11,0XFE,
0XF1,0XFD,0XAC,0XD4,0X6C,0XD4,0X8C,0XD4,0X8B,0XCC,0XD4,0XFE,0X92,0XEE,0X72,0XE6,
0XEC,0XBC,0XF4,0XF6,0X54,0XF7,0X34,0XFF,0XCB,0XBC,0X32,0XF6,0X52,0XFE,0XAC,0XD4,
0X6B,0XD4,0X8C,0XD4,0XAC,0XCC,0X31,0XF6,0X92,0XEE,0X92,0XE6,0X91,0XDE,0XB2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,
0X72,0XEE,0XAC,0XCC,0X8C,0XE4,0X2B,0XE4,0X6C,0XDC,0X4B,0XCC,0XF4,0XFE,0X92,0XE6,
0X34,0XFF,0X14,0XFF,0X8B,0XC4,0X8B,0XCC,0XAC,0XCC,0X52,0XFE,0XD4,0XFE,0XAB,0XC4,
0XAC,0XD4,0X8B,0XC4,0XF4,0XFE,0X34,0XFF,0X72,0XF6,0XAC,0XC4,0XAC,0XCC,0X73,0XFE,
0XF4,0XFE,0XCB,0XBC,0X8B,0XBC,0X11,0XFE,0X4B,0XD4,0X4B,0XDC,0X4C,0XE4,0X8C,0XDC,
0X72,0XEE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X03,0X4A,0X95,0XDE,0XB3,0XDE,0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XE6,0XB2,0XE6,0XF4,0XF6,0X14,0XFF,0XB2,0XE6,0X91,0XDE,0XD2,0XE6,0XB2,0XE6,
0X14,0XF7,0XB2,0XEE,0X72,0XEE,0X14,0XFF,0XF4,0XFE,0XF4,0XFE,0XF4,0XFE,0X92,0XEE,
0X92,0XEE,0X14,0XFF,0X14,0XFF,0X15,0XFF,0XF4,0XFE,0XB2,0XEE,0X71,0XDE,0XD2,0XE6,
0X55,0XFF,0X92,0XE6,0XD2,0XE6,0X91,0XDE,0X54,0XF7,0XB2,0XEE,0X72,0XF6,0XF4,0XFE,
0X14,0XFF,0X14,0XFF,0XF3,0XF6,0XD3,0XEE,0XB2,0XE6,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0X35,0XFF,0XD3,0XFE,0XD4,0XFE,0XF4,0XFE,0X35,0XFF,0X91,0XDE,0XD2,0XE6,
0XB2,0XE6,0X92,0XE6,0X14,0XFF,0X14,0XFF,0X35,0XFF,0X72,0XEE,0X51,0XE6,0X35,0XFF,
0XF4,0XFE,0X14,0XFF,0XD3,0XEE,0X92,0XE6,0X92,0XE6,0X34,0XFF,0X35,0XFF,0X93,0XF6,
0X72,0XEE,0X35,0XFF,0X34,0XFF,0X71,0XE6,0X35,0XFF,0X15,0XFF,0XB4,0XFE,0XF4,0XFE,
0X92,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X03,0X4A,0X95,0XDE,0XB3,0XDE,0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XE6,0XB2,0XE6,0XD3,0XE6,0XD2,0XDE,0X91,0XDE,0XD3,0XE6,0XB2,0XE6,0X91,0XDE,
0XD3,0XE6,0X92,0XDE,0X92,0XDE,0XB2,0XE6,0XB2,0XDE,0XB2,0XE6,0XD2,0XE6,0XB2,0XDE,
0XB2,0XDE,0XD2,0XDE,0XB2,0XE6,0X92,0XE6,0X91,0XDE,0XB2,0XE6,0XB2,0XDE,0XD2,0XDE,
0X92,0XE6,0X92,0XDE,0XD2,0XE6,0XD2,0XE6,0X91,0XDE,0XB2,0XDE,0X71,0XE6,0XB2,0XEE,
0X71,0XDE,0X92,0XDE,0XF3,0XE6,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD2,0XDE,0XB1,0XD6,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XD2,0XE6,0XD2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XF3,0XE6,0X91,0XDE,0XB2,0XE6,0X92,0XE6,0XB2,0XE6,0X71,0XE6,
0XB2,0XEE,0X92,0XE6,0XB2,0XE6,0X92,0XDE,0XB2,0XDE,0XB2,0XDE,0X92,0XE6,0X92,0XE6,
0X72,0XE6,0XB2,0XE6,0XB2,0XE6,0XD2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XEE,0X92,0XE6,
0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X03,0X4A,0X95,0XDE,0XB3,0XDE,0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XE6,0X91,0XDE,0XD2,0XE6,0XB2,0XDE,0XB2,0XDE,0X92,0XDE,0X92,0XE6,0XB2,0XE6,
0XB2,0XE6,0X92,0XDE,0XD2,0XE6,0XB2,0XDE,0XB2,0XDE,0XD2,0XE6,0XB2,0XDE,0XB2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XF3,0XE6,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,
0X92,0XE6,0XB2,0XEE,0X92,0XE6,0X92,0XE6,0XD2,0XDE,0XB2,0XDE,0XD2,0XE6,0X92,0XDE,
0XD2,0XE6,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XD2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XF2,0XDE,0XB2,0XD6,0XB2,0XE6,0XB2,0XE6,0X91,0XDE,0XD2,0XE6,0X91,0XDE,0XD2,0XE6,
0XB2,0XDE,0XD2,0XDE,0X91,0XD6,0XD2,0XDE,0XD2,0XE6,0X92,0XE6,0XB2,0XDE,0XD2,0XE6,
0XB2,0XDE,0XB2,0XE6,0X91,0XDE,0XD2,0XE6,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,
0XB2,0XEE,0X92,0XE6,0X91,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X03,0X4A,0X95,0XDE,0XB3,0XDE,0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0X92,0XE6,0XB2,0XE6,
0XB2,0XE6,0XD2,0XE6,0XB2,0XDE,0XB2,0XDE,0XD2,0XE6,0X91,0XDE,0XD2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0X91,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,
0XB2,0XE6,0X92,0XE6,0X92,0XE6,0X92,0XE6,0XD2,0XE6,0XB2,0XDE,0XB2,0XDE,0XD2,0XE6,
0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XD2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XD3,0XE6,0XB2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,
0XD2,0XE6,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XD2,0XE6,0XB2,0XDE,0XB2,0XE6,0XB2,0XDE,
0X92,0XDE,0XB2,0XE6,0XD2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X03,0X4A,0X95,0XDE,0XB3,0XDE,0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,
0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,
0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XDE,0XB2,0XE6,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X03,0X4A,0X95,0XDE,0XB3,0XDE,0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,
0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0X92,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X03,0X4A,0X95,0XDE,0XB3,0XDE,0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,
0XB2,0XE6,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X03,0X4A,0X95,0XDE,0XB3,0XDE,0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XE6,0XB2,0XE6,
0XB2,0XE6,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,
0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X03,0X4A,0X95,0XDE,0XB3,0XDE,0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0X92,0XDE,0X92,0XDE,
0X92,0XE6,0X92,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0X92,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XE6,
0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X03,0X4A,0X95,0XDE,0XB3,0XDE,0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XE6,0X92,0XDE,0X91,0XDE,0X71,0XD6,
0X71,0XDE,0X71,0XDE,0X71,0XDE,0X71,0XD6,0X91,0XD6,0X91,0XDE,0XB2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X03,0X4A,0X95,0XDE,0XB3,0XDE,0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0X91,0XDE,0X91,0XD6,0X91,0XD6,
0X91,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X91,0XDE,0X92,0XDE,0XB2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XDE,
0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XD2,0XDE,0XD2,0XD6,
0XD2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,
0XD2,0XE6,0XD2,0XDE,0XB2,0XDE,0X91,0XDE,0X91,0XD6,0X91,0XD6,0X91,0XD6,0X91,0XD6,
0X91,0XD6,0X91,0XD6,0X91,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XDE,0X71,0XDE,
0X91,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XD6,
0XD2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0X92,0XDE,0XD2,0XE6,0XB2,0XDE,0XB2,0XDE,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0XB2,0XDE,0XD2,0XE6,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XD2,0XE6,0XB2,0XDE,0XB2,0XDE,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,
0XB2,0XE6,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0X92,0XDE,0X92,0XDE,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X92,0XDE,0X92,0XDE,0XB2,0XDE,0XD2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,
0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X91,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,
0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD2,0XE6,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0X92,0XDE,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X91,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,
0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XD2,0XE6,0XB2,0XDE,0XB2,0XDE,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,
0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,
0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XD2,0XE6,0XB2,0XDE,0XB2,0XDE,0X91,0XDE,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0XB2,0XDE,0X91,0XDE,0XD2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,
0X92,0XDE,0XD3,0XEE,0X92,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0X72,0XE6,
0XB2,0XEE,0X92,0XE6,0XB2,0XE6,0XD2,0XE6,0X92,0XDE,0XB2,0XE6,0XB2,0XE6,0X92,0XE6,
0X92,0XE6,0X92,0XE6,0XB2,0XEE,0XB2,0XE6,0X92,0XE6,0XB2,0XE6,0X92,0XE6,0X92,0XE6,
0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XD2,0XDE,0X91,0XDE,0X91,0XDE,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X91,0XDE,0X91,0XD6,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0X92,0XE6,
0X72,0XF6,0X31,0XF6,0X32,0XF6,0X72,0XFE,0X92,0XEE,0X92,0XE6,0X92,0XEE,0X92,0XF6,
0X52,0XF6,0X52,0XF6,0X52,0XEE,0X72,0XEE,0X92,0XE6,0X92,0XE6,0X92,0XEE,0X72,0XEE,
0X72,0XF6,0X72,0XF6,0X72,0XEE,0X51,0XEE,0X93,0XFE,0X72,0XF6,0X52,0XF6,0X93,0XF6,
0X92,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XD2,0XDE,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0X72,0XF6,
0X8C,0XDC,0X2B,0XE4,0X4C,0XEC,0X2B,0XDC,0X8B,0XCC,0X32,0XFE,0X32,0XFE,0X6B,0XD4,
0X8C,0XE4,0X2B,0XDC,0X6C,0XDC,0XAC,0XD4,0X92,0XF6,0X72,0XEE,0X31,0XF6,0XAC,0XCC,
0X8C,0XD4,0X8C,0XD4,0X6B,0XC4,0X32,0XFE,0X2B,0XD4,0X0B,0XDC,0X8C,0XDC,0X8B,0XCC,
0X72,0XEE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XD2,0XDE,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0XD2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0X92,0XE6,0X52,0XF6,
0X0B,0XE4,0X0C,0XF4,0X0B,0XEC,0X2C,0XE4,0X8C,0XD4,0X32,0XFE,0X12,0XFE,0X4B,0XD4,
0X0B,0XDC,0X2B,0XE4,0X6C,0XEC,0X6C,0XD4,0X52,0XF6,0X72,0XF6,0X72,0XFE,0X8C,0XCC,
0X6C,0XD4,0X8C,0XD4,0XAC,0XD4,0X12,0XFE,0X4C,0XDC,0X4C,0XE4,0X2B,0XDC,0X8C,0XD4,
0X72,0XEE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XD2,0XDE,0X71,0XD6,0X91,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X91,0XD6,0X71,0XD6,0XD2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0X92,0XE6,0X52,0XF6,
0X4C,0XE4,0X0C,0XEC,0X13,0XFE,0X53,0XFE,0X93,0XFE,0X8B,0XC4,0XF1,0XFD,0X8C,0XDC,
0X6C,0XDC,0X53,0XFE,0X32,0XFE,0X73,0XFE,0X11,0XFE,0XAC,0XC4,0X8B,0XC4,0X93,0XFE,
0X93,0XFE,0XD4,0XFE,0XF4,0XFE,0X52,0XF6,0XD4,0XFE,0XB4,0XFE,0X6B,0XCC,0XB4,0XFE,
0X92,0XEE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0X91,0XD6,0X91,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0X92,0XE6,0X52,0XF6,
0X2B,0XDC,0X4C,0XE4,0X6B,0XD4,0XAC,0XCC,0XAB,0XC4,0XD4,0XFE,0X12,0XFE,0X6B,0XD4,
0X4B,0XD4,0X8C,0XD4,0XAC,0XCC,0XCC,0XCC,0X52,0XF6,0XF4,0XFE,0XF4,0XFE,0XCC,0XC4,
0XEC,0XC4,0X72,0XF6,0X52,0XEE,0X72,0XEE,0X92,0XF6,0X72,0XF6,0X8B,0XC4,0X52,0XF6,
0X92,0XEE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0X71,0XD6,0X91,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0X72,0XF6,
0X4B,0XDC,0X6C,0XDC,0XB4,0XFE,0XF4,0XFE,0XB3,0XFE,0X8B,0XCC,0XF1,0XFD,0X6B,0XD4,
0X6C,0XD4,0X94,0XFE,0XF4,0XFE,0XF4,0XF6,0X72,0XEE,0X72,0XEE,0X72,0XEE,0XF4,0XFE,
0XB3,0XFE,0XAB,0XC4,0XCC,0XCC,0X52,0XF6,0X72,0XEE,0X72,0XEE,0XCC,0XC4,0X72,0XF6,
0X92,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0XD2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XE6,0X72,0XEE,
0X8C,0XD4,0X6B,0XD4,0XD4,0XFE,0X14,0XFF,0XB4,0XFE,0XAC,0XCC,0X32,0XFE,0X8C,0XD4,
0X8C,0XD4,0XB3,0XFE,0X14,0XFF,0X35,0XFF,0X92,0XEE,0X92,0XEE,0X72,0XEE,0X15,0XFF,
0XF4,0XFE,0X8B,0XBC,0XAB,0XC4,0X72,0XF6,0X92,0XEE,0X72,0XEE,0XCB,0XBC,0X92,0XEE,
0X92,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD2,0XE6,0XB2,0XDE,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X51,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X91,0XDE,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X91,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XE6,0X92,0XE6,0X72,0XEE,
0XAC,0XC4,0XCC,0XCC,0XAB,0XBC,0XAB,0XC4,0XAC,0XC4,0XB4,0XFE,0X73,0XFE,0XAB,0XC4,
0XCC,0XCC,0X8B,0XC4,0XAB,0XBC,0X0C,0XC5,0X52,0XEE,0XEC,0XBC,0XCB,0XBC,0XCB,0XBC,
0XCB,0XBC,0X14,0XFF,0X14,0XFF,0X71,0XE6,0XB2,0XEE,0X72,0XE6,0XEC,0XB4,0X92,0XEE,
0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0X34,0XEF,0X55,0XF7,0X71,0XD6,0X71,0XD6,0X92,0XDE,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X91,0XDE,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X92,0XDE,0X71,0XD6,
0X75,0XF7,0X34,0XEF,0XD2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0X92,0XE6,0X92,0XE6,
0XF4,0XFE,0XF4,0XFE,0XD4,0XFE,0XF4,0XFE,0XF4,0XFE,0X72,0XEE,0X72,0XEE,0X14,0XFF,
0XB3,0XFE,0XF4,0XFE,0XF4,0XFE,0X35,0XFF,0X72,0XEE,0XF4,0XFE,0XF4,0XFE,0XF4,0XFE,
0X14,0XFF,0X51,0XE6,0X92,0XE6,0XD2,0XE6,0XB2,0XE6,0XB2,0XE6,0X34,0XFF,0X92,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0X91,0XDE,0X75,0XF7,0X75,0XF7,0X51,0XD6,0X71,0XD6,0X71,0XD6,0X51,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X51,0XD6,
0X75,0XF7,0X34,0XEF,0XD2,0XE6,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XE6,
0X55,0XFF,0X35,0XFF,0X35,0XFF,0X14,0XFF,0X35,0XFF,0X92,0XE6,0XB2,0XE6,0X14,0XF7,
0X55,0XFF,0X35,0XFF,0X34,0XFF,0X34,0XFF,0X92,0XE6,0X75,0XFF,0X34,0XFF,0X34,0XFF,
0X34,0XF7,0XB2,0XE6,0X91,0XDE,0XB2,0XDE,0XB2,0XDE,0X91,0XDE,0X55,0XF7,0XD2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD3,0XE6,0XB2,0XDE,0XD2,0XE6,0XB2,0XDE,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X92,0XDE,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X92,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XD6,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,
0X92,0XDE,0X71,0XDE,0X92,0XE6,0X92,0XE6,0X92,0XDE,0XB2,0XDE,0XD2,0XE6,0X92,0XDE,
0XB2,0XE6,0X91,0XDE,0XD2,0XE6,0X92,0XDE,0XD2,0XE6,0XB2,0XDE,0X71,0XDE,0XB2,0XE6,
0X91,0XDE,0XD2,0XE6,0XD2,0XDE,0XD2,0XDE,0XB2,0XD6,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0X92,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0X54,0XEF,0XB2,0XDE,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X91,0XDE,0X71,0XD6,0X71,0XD6,0X91,0XDE,0X71,0XD6,0X91,0XDE,0X34,0XEF,
0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XE6,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XD2,0XE6,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XE6,
0XD2,0XE6,0X91,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XD2,0XE6,0XB2,0XDE,0XB2,0XDE,0XF3,0XE6,0X50,0XD6,0X71,0XD6,0X91,0XDE,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X91,0XDE,0X71,0XD6,0X71,0XD6,0X91,0XDE,0X50,0XD6,0X71,0XD6,0XF3,0XE6,
0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XE6,0XD2,0XE6,0XB2,0XE6,0X92,0XDE,0XD2,0XE6,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,
0XD2,0XDE,0XD2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XE6,0XB2,0XDE,
0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0X91,0XDE,0X91,0XD6,0XB2,0XDE,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X91,0XD6,0X91,0XDE,0XB2,0XDE,0X91,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0X92,0XDE,0X92,0XDE,0XB2,0XE6,0X92,0XDE,0XB2,0XE6,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XB2,0XDE,0XD2,0XE6,0XB2,0XDE,0XB2,0XDE,0XD2,0XE6,0X92,0XDE,0XB2,0XDE,0XB2,0XE6,
0XB2,0XDE,0XD2,0XE6,0XB2,0XDE,0XD2,0XE6,0XB2,0XDE,0XD2,0XE6,0XB2,0XE6,0X92,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XE6,0X54,0XF7,0X34,0XEF,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X91,0XD6,0X71,0XD6,0X91,0XD6,0X71,0XD6,0X91,0XDE,0X13,0XEF,0X54,0XF7,0XD2,0XE6,
0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,
0XB2,0XDE,0XB2,0XE6,0XB2,0XDE,0X92,0XDE,0XB2,0XE6,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,
0XB2,0XE6,0X91,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0X91,0XDE,0XD2,0XE6,0X54,0XF7,
0X91,0XDE,0X71,0XD6,0X71,0XD6,0X91,0XDE,0X51,0XD6,0X71,0XD6,0X91,0XDE,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X92,0XDE,0X51,0XD6,0X71,0XD6,
0X91,0XD6,0X71,0XD6,0X71,0XD6,0X91,0XDE,0X54,0XEF,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XE6,0XB2,0XDE,0XD2,0XDE,0X34,0XEF,
0X71,0XD6,0X71,0XD6,0X91,0XDE,0X71,0XD6,0X91,0XDE,0X92,0XDE,0X51,0XD6,0X51,0XD6,
0X92,0XDE,0X91,0XDE,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,
0X91,0XDE,0X71,0XD6,0X71,0XD6,0X91,0XDE,0X54,0XEF,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XE6,0XB2,0XDE,0XB2,0XDE,0XF3,0XE6,
0X54,0XF7,0X54,0XF7,0X75,0XF7,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X92,0XDE,
0X51,0XD6,0X71,0XD6,0X71,0XD6,0X71,0XD6,0X91,0XD6,0X50,0XD6,0X91,0XDE,0X91,0XD6,
0X71,0XD6,0X54,0XF7,0X75,0XF7,0X34,0XEF,0XD2,0XDE,0XF3,0XE6,0XD2,0XDE,0X91,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD2,0XDE,0XB2,0XDE,0X92,0XDE,0X55,0XF7,0X55,0XF7,0X55,0XF7,0X71,0XD6,0X71,0XD6,
0X71,0XD6,0X71,0XD6,0X71,0XD6,0X51,0XD6,0X71,0XD6,0X91,0XDE,0X34,0XEF,0X54,0XF7,
0X54,0XF7,0XD2,0XE6,0X91,0XDE,0XB2,0XDE,0XB2,0XDE,0X91,0XDE,0XB2,0XDE,0XD2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XD2,0XDE,0XD2,0XE6,0XB2,0XDE,0X92,0XDE,0XB2,0XDE,0X54,0XF7,0X55,0XF7,
0X54,0XF7,0X54,0XEF,0X54,0XF7,0X75,0XF7,0X54,0XF7,0X54,0XEF,0XD2,0XE6,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XD2,0XE6,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XD2,0XE6,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XF3,0XE6,0XB2,0XDE,0XB2,0XDE,0X54,0XF7,0X55,0XF7,
0X54,0XF7,0X54,0XEF,0X54,0XF7,0X75,0XF7,0X54,0XEF,0X54,0XEF,0XD2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X4A,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XD2,0XDE,0XD2,0XE6,0X91,0XDE,0XD2,0XDE,0XB2,0XDE,0XD3,0XE6,0X92,0XDE,
0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XE6,
0XD2,0XE6,0XB2,0XDE,0XD3,0XE6,0X92,0XDE,0XD3,0XE6,0XB2,0XDE,0XD2,0XE6,0XD2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X4A,
0X04,0X52,0X95,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,
0XB2,0XDE,0XD2,0XDE,0X92,0XDE,0XD2,0XE6,0XD2,0XE6,0XB2,0XDE,0X92,0XDE,0XD3,0XE6,
0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD3,0XE6,0XB2,0XDE,0XD2,0XE6,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XE6,0X92,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0X95,0XE6,0X04,0X52,
0XE4,0X51,0X75,0XE6,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,
0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,
0X92,0XDE,0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0X75,0XE6,0XE4,0X51,
0XE4,0X51,0X75,0XE6,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,
0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XD1,0XDE,0XB2,0XDE,0X75,0XE6,0XE4,0X51,
0XE5,0X51,0X96,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB1,0XDE,0XD1,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,0XB1,0XDE,
0XB1,0XDE,0XB1,0XDE,0XB2,0XDE,0XB2,0XDE,0XD1,0XDE,0XB2,0XDE,0X95,0XDE,0XE5,0X51,
0X05,0X4A,0X96,0XDE,0XB3,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD1,0XDE,0XD1,0XDE,0XD1,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,
0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XE6,0XB2,0XE6,0XB3,0XDE,0XB2,0XDE,0XD2,0XDE,0XD1,0XDE,0XD1,0XDE,0XD1,0XDE,
0XB1,0XDE,0XB1,0XDE,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB3,0XDE,0X95,0XDE,0X05,0X4A,
0X05,0X4A,0X96,0XDE,0XB3,0XDE,0XD3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD1,0XDE,0XD1,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XB2,0XDE,0XD2,0XDE,0XD1,0XDE,0XD1,0XDE,0XD1,0XDE,
0XB1,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XD3,0XDE,0XB3,0XDE,0X96,0XDE,0X05,0X4A,
0X04,0X52,0X95,0XDE,0XB3,0XDE,0XD3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XD1,0XDE,0XD1,0XDE,0XD1,0XDE,0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XE6,0X92,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XE6,0XB2,0XE6,0XB2,0XDE,0XD2,0XDE,0XD1,0XDE,0XD1,0XDE,0XD1,0XDE,0XB1,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0XD3,0XDE,0XB3,0XDE,0X95,0XDE,0X04,0X52,
0X03,0X52,0X94,0XE6,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XE6,
0XB1,0XE6,0XB1,0XDE,0XD1,0XDE,0XD1,0XDE,0XD1,0XDE,0XB2,0XDE,0XB2,0XE6,0XB2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XE6,0XB2,0XE6,0XD1,0XDE,0XD1,0XDE,0XD1,0XDE,0XD1,0XDE,0XB1,0XDE,0XB2,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0XD2,0XDE,0XB3,0XDE,0X95,0XE6,0XE4,0X51,
0X03,0X52,0X94,0XE6,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB2,0XE6,
0XB2,0XE6,0XB1,0XDE,0XD1,0XDE,0XD1,0XDE,0XD1,0XDE,0XD2,0XDE,0XB2,0XE6,0XB3,0XE6,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XD1,0XDE,0XD1,0XDE,0XD1,0XDE,0XD1,0XDE,0XB2,0XDE,0XB2,0XE6,
0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,0XB2,0XDE,0XD2,0XDE,0XB3,0XDE,0X94,0XE6,0X04,0X52,
0X05,0X52,0X75,0XDE,0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XE6,0X93,0XDE,
0XB3,0XE6,0XB2,0XDE,0XB2,0XDE,0XD1,0XDE,0XD1,0XDE,0XD2,0XDE,0XB2,0XDE,0XB3,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB3,0XDE,0XB3,0XDE,0XD2,0XDE,0XD2,0XDE,0XD1,0XDE,0XD2,0XDE,0XB3,0XDE,0XB3,0XE6,
0X92,0XDE,0XB3,0XE6,0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0X75,0XDE,0X04,0X52,
0X26,0X42,0X15,0XC6,0XD1,0XBD,0XD4,0XE6,0X93,0XDE,0X92,0XDE,0XB3,0XE6,0X92,0XDE,
0XB3,0XDE,0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB1,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB3,0XDE,0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,
0X92,0XDE,0XB3,0XE6,0X72,0XDE,0X92,0XDE,0XD3,0XE6,0XD1,0XBD,0X14,0XC6,0X26,0X42,
0X71,0X7C,0X48,0X3A,0X45,0X4A,0X95,0XDE,0X94,0XE6,0XB3,0XE6,0X92,0XDE,0XD3,0XE6,
0XB3,0XDE,0XB3,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB3,0XDE,
0XD3,0XE6,0X92,0XDE,0XB3,0XE6,0X94,0XE6,0X94,0XDE,0X24,0X42,0X48,0X3A,0X71,0X7C,
0XD4,0X74,0X69,0X2A,0X47,0X3A,0X76,0XD6,0XB5,0XE6,0X93,0XE6,0XB2,0XDE,0X92,0XD6,
0XD3,0XDE,0XD3,0XDE,0XB3,0XDE,0XB3,0XE6,0XB2,0XE6,0XB2,0XE6,0XD2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XD2,0XDE,
0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XD2,0XDE,0XB2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,0XD2,0XDE,
0XD2,0XDE,0XD2,0XDE,0XB2,0XE6,0XB2,0XE6,0XB3,0XE6,0XB3,0XDE,0XD3,0XDE,0XD3,0XDE,
0XB2,0XD6,0XB2,0XDE,0X93,0XE6,0XB5,0XE6,0X76,0XD6,0X46,0X3A,0X69,0X2A,0XB4,0X74,
0X15,0X65,0X6A,0X1A,0X47,0X2A,0X56,0XBE,0XD3,0XBD,0XD1,0XC5,0XF4,0XE6,0XD2,0XDE,
0XD2,0XD6,0XB2,0XD6,0XB3,0XDE,0X93,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB3,0XDE,0XB2,0XDE,0XB3,0XDE,0XB2,0XDE,0XB3,0XDE,0XB2,0XDE,
0XB3,0XDE,0XB2,0XDE,0XB3,0XDE,0XB2,0XDE,0XB3,0XDE,0XB2,0XDE,0XB3,0XDE,0XB2,0XDE,
0XB3,0XDE,0XB2,0XDE,0XB3,0XDE,0XB2,0XDE,0XB3,0XDE,0XB2,0XDE,0XB3,0XDE,0XB2,0XDE,
0XB3,0XDE,0XB2,0XDE,0XB3,0XDE,0XB2,0XDE,0XB3,0XDE,0XB2,0XDE,0XB3,0XDE,0XB2,0XDE,
0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XDE,0XB2,0XD6,0XD2,0XD6,
0XD2,0XDE,0XF3,0XE6,0XD1,0XC5,0XD3,0XBD,0X56,0XBE,0X48,0X2A,0X6A,0X1A,0XF5,0X64,
0X35,0X55,0X14,0X5D,0XD3,0X6C,0X48,0X2A,0X47,0X42,0X45,0X4A,0X12,0XC6,0X11,0XC6,
0X10,0XC6,0X11,0XC6,0XF1,0XC5,0XF2,0XCD,0XF1,0XCD,0XF1,0XCD,0XF1,0XCD,0XF1,0XCD,
0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,
0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,
0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,
0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,
0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,
0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,
0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,
0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,
0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,
0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,0X11,0XCE,0X10,0XCE,0X11,0XCE,0X11,0XCE,
0X11,0XCE,0X11,0XCE,0X11,0XCE,0X11,0XCE,0X11,0XCE,0X11,0XCE,0X11,0XCE,0X11,0XCE,
0X11,0XCE,0X11,0XCE,0X11,0XCE,0X11,0XCE,0X11,0XCE,0X11,0XCE,0X11,0XCE,0X11,0XCE,
0XF1,0XCD,0XF1,0XCD,0XF1,0XCD,0XF1,0XCD,0XF1,0XCD,0XF1,0XCD,0XF1,0XCD,0XF1,0XCD,
0XF1,0XCD,0XF1,0XCD,0XF1,0XCD,0XF1,0XCD,0XF1,0XCD,0XF1,0XCD,0XF1,0XCD,0XF1,0XCD,
0XF1,0XC5,0X11,0XC6,0XF1,0XC5,0X11,0XC6,0XF1,0XC5,0X11,0XC6,0XF1,0XC5,0X11,0XC6,
0XF1,0XC5,0X11,0XC6,0XF1,0XC5,0X11,0XC6,0XF1,0XC5,0X11,0XC6,0XF1,0XC5,0X11,0XC6,
0X10,0XCE,0X10,0XCE,0XF1,0XCD,0XF1,0XCD,0XF1,0XCD,0X11,0XC6,0X11,0XC6,0X10,0XC6,
0X10,0XC6,0X11,0XC6,0X45,0X4A,0X47,0X42,0X48,0X2A,0XD3,0X6C,0XF4,0X5C,0X15,0X55,
0X56,0X55,0X15,0X55,0XF5,0X5C,0X6B,0X1A,0X29,0X2A,0X28,0X3A,0XF4,0XBD,0X13,0XC6,
0XF2,0XC5,0XF2,0XC5,0XF3,0XC5,0XD3,0XCD,0XF3,0XCD,0XD3,0XCD,0XD3,0XCD,0XD3,0XCD,
0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,
0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,
0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,
0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,
0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,
0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,
0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,
0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,
0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,
0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF2,0XCD,0XF3,0XCD,0XF3,0XCD,
0XF3,0XCD,0XF3,0XCD,0XF3,0XCD,0XF3,0XCD,0XF3,0XCD,0XF3,0XCD,0XF3,0XCD,0XF3,0XCD,
0XF3,0XCD,0XF3,0XCD,0XF3,0XCD,0XF3,0XCD,0XF3,0XCD,0XF3,0XCD,0XF3,0XCD,0XF3,0XCD,
0XF3,0XCD,0XF3,0XCD,0XF3,0XCD,0XF3,0XCD,0XF3,0XCD,0XF3,0XCD,0XF3,0XCD,0XF3,0XCD,
0XF3,0XCD,0XF3,0XCD,0XF3,0XCD,0XF3,0XCD,0XF3,0XCD,0XF3,0XCD,0XF3,0XCD,0XF3,0XCD,
0XF3,0XC5,0XF3,0XC5,0XF3,0XC5,0XF3,0XC5,0XF3,0XC5,0XF3,0XC5,0XF3,0XC5,0XF3,0XC5,
0XF3,0XC5,0XF3,0XC5,0XF3,0XC5,0XF3,0XC5,0XF3,0XC5,0XF3,0XC5,0XF3,0XC5,0XF3,0XCD,
0XD3,0XCD,0XD3,0XCD,0XD3,0XCD,0XD3,0XCD,0XF3,0XC5,0XF3,0XC5,0XF2,0XCD,0XF2,0XC5,
0X12,0XC6,0XF3,0XBD,0X27,0X3A,0X29,0X2A,0X6A,0X22,0XF4,0X5C,0X14,0X55,0X35,0X55,
0X76,0X55,0X36,0X4D,0XD5,0X54,0XD5,0X64,0X73,0X6C,0X52,0X7C,0X06,0X42,0X25,0X4A,
0X04,0X4A,0X04,0X4A,0X05,0X4A,0X05,0X4A,0X05,0X4A,0X05,0X52,0XE5,0X51,0XE4,0X51,
0X04,0X52,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,
0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,
0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,
0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,
0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,
0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,
0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,
0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,
0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,
0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,
0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,
0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,
0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,
0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,
0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,
0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,0X04,0X4A,
0X05,0X52,0XE5,0X51,0XE5,0X49,0X05,0X4A,0X05,0X4A,0X04,0X4A,0X04,0X52,0X04,0X4A,
0X24,0X4A,0X05,0X42,0X51,0X7C,0X53,0X6C,0XD5,0X64,0XF4,0X54,0X35,0X55,0X76,0X55,
};
#line 15 "app\\game.c"

u16 FlappyBird_Frame[480][320] __attribute__((at(0x68000000)));
u16 FlappyBird_Bird_Buff[18][26] __attribute__((at(0X6804C000)));
u16 FlappyBird_column_Buff[3][259][39] __attribute__((at(0X6804E000))) ;
u16 FlappyBird_column_Buff_BG[3][259][39] __attribute__((at(0X6805D000)));



u8 hide[3] = {0,0,0};
static u16 l_x = 0, l_y = 0;
static u16 l_x_c[3] = {0},l_y_c[3] = {0};
u8 ud_mode = 0,bird_mode = 'u';
int bird_height = 119,bird_wide = 57;
u16 score_lock = 0;
u32 score = 0;
int difficult = 0;





void FlappyBird_Frame_DrawPoint( u16 x, u16 y, u16 color )
{
	FlappyBird_Frame[y][x] = color;
}


void FlappyBird_Frame_Fill( u16 x, u16 y, u16 xx, u16 yy, u16 color )
{
	u16 x_t,y_t;
	for( y_t = y; y_t <= yy; y_t++ )
	{
		for( x_t = x; x_t <= xx; x_t++)
		{
			FlappyBird_Frame_DrawPoint(x_t,y_t,color);
		}
	}
}

void FlappyBird_Frame_Clear( void )
{
	u16 x_t,y_t;
	for( y_t = 0; y_t < 320; y_t++ )
	{
		for( x_t = 0; x_t < 240; x_t++)
		{
			FlappyBird_Frame_DrawPoint(x_t,y_t,0x4df9);
		}
	}
}



void FlappyBird_Frame_DrawLine( u16 x1, u16 y1, u16 x2, u16 y2 , u16 color )
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 
	delta_x=x2-x1; 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1; 
	else if(delta_x==0)incx=0;
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )
	{  
		FlappyBird_Frame_DrawPoint(uRow,uCol,color);
		xerr+=delta_x ; 
		yerr+=delta_y ; 
		if(xerr>distance) 
		{ 
			xerr-=distance; 
			uRow+=incx; 
		} 
		if(yerr>distance) 
		{ 
			yerr-=distance; 
			uCol+=incy; 
		} 
	}  
} 

void FlappyBird_DrawGreenBar( u16 x, u16 y, u16 color )
{
	u8 i;
	FlappyBird_Frame_Fill(x,y,240,y+12,color);
	for( i = 0; i < 12; i++)
		FlappyBird_DrawParallelogram(x+i*20,y+12,0x75E6);
	
}

void FlappyBird_DrawGreenBar_Play( u16 x, u16 y,u8 Speed )
{
	u8 i;
	u16 temp,xx,yy;
	for( i = 0; i < Speed ; i++ )
	{
		for( yy = y; yy < y + 12; yy++ )
		{
			for( xx = x; xx < 239 ; xx++)
			{
				if(x == xx)
				{
					temp = FlappyBird_Frame[yy][xx];
				}
				FlappyBird_Frame[yy][xx] = FlappyBird_Frame[yy][xx+1];
			}
			FlappyBird_Frame[yy][xx] = temp;
		}
	}
}


void FlappyBird_DrawParallelogram( u16 x, u16 y, u16 color )
{
	u8 i;
	for( i = 0; i < 10; i++ )
		FlappyBird_Frame_DrawLine(x+i,y-1,x+10+i,y-12,color);
}

void FlappyBird_Frame_ShowChar(u16 x,u16 y,u8 num,u8 size,u8 mode,u16 color)
{  							  
    u8 temp,t1,t;
	u16 y0=y;
	u16 colortemp=color;      			     
	
	num=num-' ';
	if(!mode) 
	{
	    for(t=0;t<size;t++)
	    {   
			if(size==12)temp=asc2_1206[num][t];  
			else temp=asc2_1608[num][t];		 
	        for(t1=0;t1<8;t1++)
			{			    
		        if(temp&0x80)POINT_COLOR=colortemp;
				else POINT_COLOR=BACK_COLOR;
				FlappyBird_Frame_DrawPoint(x,y,POINT_COLOR);	
				temp<<=1;
				y++;
				if(x>=Lcd_Dev.width){POINT_COLOR=colortemp;return;}
				if((y-y0)==size)
				{
					y=y0;
					x++;
					if(x>=Lcd_Dev.width){POINT_COLOR=colortemp;return;}
					break;
				}
			}  	 
	    }    
	}else
	{
	    for(t=0;t<size;t++)
	    {   
			if(size==12)temp=asc2_1206[num][t];  
			else temp=asc2_1608[num][t];		 
	        for(t1=0;t1<8;t1++)
			{			    
		        if(temp&0x80)FlappyBird_Frame_DrawPoint(x,y,POINT_COLOR);
				temp<<=1;
				y++;
				if(x>=Lcd_Dev.height){POINT_COLOR=colortemp;return;}
				if((y-y0)==size)
				{
					y=y0;
					x++;
					if(x>=Lcd_Dev.width){POINT_COLOR=colortemp;return;}
					break;
				}
			}  	 
	    }     
	}
	POINT_COLOR=colortemp;	    	   	 	  
}

void FlappyBird_Frame_ShowString(u16 x,u16 y,u16 width,u16 height,u8 size,u8 *p,u16 color)
{         
	u8 x0=x;
	width+=x;
	height+=y;
    while((*p<='~')&&(*p>=' '))
    {       
        if(x>=width){x=x0;y+=size;}
        if(y>=height)break;
        FlappyBird_Frame_ShowChar(x,y,*p,size,0,color);
        x+=size/2;
        p++;
    }  
}

void FlappyBird_Frame_DrawBGPic( u16 x, u16 y )
{
	u16 color,xx = x,yy = y;
	
	for( yy = y; yy < 70 + y; yy++ )
	{
		for( xx = x; xx < 240; xx++ )
		{
			color = gImage_flappybird_background[((yy-y)*240+xx)*2+1];
			color <<= 8;
			color = color + gImage_flappybird_background[((yy-y)*240+xx)*2];
			FlappyBird_Frame_DrawPoint(xx,yy,color);
		}
	}
}

void FlappyBird_Frame_DrawLogo( u16 x, u16 y )
{
		u16 color,xx = x,yy = y;
	
	for( yy = y; yy < 36 + y; yy++ )
	{
		for( xx = x; xx < 135 + x; xx++ )
		{
			color = gImage_flappybird_Logo[((yy-y)*135+(xx-x))*2+1];
			color <<= 8;
			color = color + gImage_flappybird_Logo[((yy-y)*135+(xx-x))*2];
			FlappyBird_Frame_DrawPoint(xx,yy,color);
		}
	}
	
}

void FlappyBird_Frame_DrawButton( u16 x, u16 y , u8 mode )
{
		u16 color,xx = x,yy = y;
		int R,G,B;
		if(0 == mode)
		{
			for( yy = y; yy < 44 + y; yy++ )
			{
				for( xx = x; xx < 78 + x; xx++ )
				{
					color = gImage_flappybird_button_play[((yy-y)*78+(xx-x))*2+1];
					color <<= 8;
					color = color + gImage_flappybird_button_play[((yy-y)*78+(xx-x))*2];
					R = color & 0xf800;
					R >>= 11;
					G = color & 0x07e0;
					G >>= 5;
					B = color & 0x001f;
					if((B - R) > 7)
					{
						if((G - B) > 10 && (G - R) > 10 && (B - R) < 10)
						{
							FlappyBird_Frame_DrawPoint(xx,yy,color);
						}
					}
					else
						FlappyBird_Frame_DrawPoint(xx,yy,color);
				}
			}
		}
		else
		{
			for( yy = y; yy < 44 + y; yy++ )
			{
				for( xx = x; xx < 78 + x; xx++ )
				{
					color = gImage_flappybird_button_stop[((yy-y)*78+(xx-x))*2+1];
					color <<= 8;
					color = color + gImage_flappybird_button_stop[((yy-y)*78+(xx-x))*2];
					R = color & 0xf800;
					R >>= 11;
					G = color & 0x07e0;
					G >>= 5;
					B = color & 0x001f;
					if((B - R) > 7)
						;
					else
						FlappyBird_Frame_DrawPoint(xx,yy,color);
				}
			}
		}
}

void FlappyBird_Frame_DrawBird( u16 x, u16 y , u8 mode )
{
	u16 color,xx = x,yy = y;
	int R,G,B;
	switch(mode)
	{
		case 'u':
		{
			for( yy = y; yy < 18 + y; yy++ )
			{
				for( xx = x; xx < 26 + x; xx++ )
				{
					color = gImage_flappybird_bird_u[((yy-y)*26+(xx-x))*2+1];
					color <<= 8;
					color = color + gImage_flappybird_bird_u[((yy-y)*26+(xx-x))*2];
					R = color & 0xf800;
					R >>= 11;
					G = color & 0x07e0;
					G >>= 5;
					B = color & 0x001f;
					if((B - R) > 7)
						;
					else
						FlappyBird_Frame_DrawPoint(xx,yy,color);
				}
			}
		}
		break;
		case 'm':
		{
			for( yy = y; yy < 18 + y; yy++ )
			{
				for( xx = x; xx < 26 + x; xx++ )
				{
					color = gImage_flappybird_bird_m[((yy-y)*26+(xx-x))*2+1];
					color <<= 8;
					color = color + gImage_flappybird_bird_m[((yy-y)*26+(xx-x))*2];
					R = color & 0xf800;
					R >>= 11;
					G = color & 0x07e0;
					G >>= 5;
					B = color & 0x001f;
					if((B - R) > 7)
						;
					else
						FlappyBird_Frame_DrawPoint(xx,yy,color);
				}
			}
		}
		break;
		case 'd':
		{
			for( yy = y; yy < 18 + y; yy++ )
			{
				for( xx = x; xx < 26 + x; xx++ )
				{
					color = gImage_flappybird_bird_d[((yy-y)*26+(xx-x))*2+1];
					color <<= 8;
					color = color + gImage_flappybird_bird_d[((yy-y)*26+(xx-x))*2];
					R = color & 0xf800;
					R >>= 11;
					G = color & 0x07e0;
					G >>= 5;
					B = color & 0x001f;
					if((B - R) > 7)
						;
					else
						FlappyBird_Frame_DrawPoint(xx,yy,color);
				}
			}
		}
		break;
		case 'f':
		{
			for( yy = y; yy < 26 + y; yy++ )
			{
				for( xx = x; xx < 18 + x; xx++ )
				{
					color = gImage_flappybird_bird_m[((17-(xx-x))*26+(yy-y))*2+1];
					color <<= 8;
					color = color + gImage_flappybird_bird_m[((17-(xx-x))*26+(yy-y))*2];
					R = color & 0xf800;
					R >>= 11;
					G = color & 0x07e0;
					G >>= 5;
					B = color & 0x001f;
					if((B - R) > 7)
						;
					else
						FlappyBird_Frame_DrawPoint(xx,yy,color);
				}
			}
		}
		break;
		default:
			break;
	}
}

void FlappyBird_Frame_DrawBird_Play( u16 x, u16 y, u8 mode )
{
	u16 xx,yy;
	if( 0 != l_x || 0 != l_y )
	{
		for( yy = l_y; yy < l_y+18; yy++ )
		{
			for( xx = l_x; xx < l_x+26; xx++ )
			{
				FlappyBird_Frame_DrawPoint(xx,yy,FlappyBird_Bird_Buff[yy-l_y][xx-l_x]);
			}
		}
	}
	for( yy = y; yy < y+18; yy++ )
	{
		for( xx = x; xx < x+26; xx++ )
		{
			FlappyBird_Bird_Buff[yy-y][xx-x] = FlappyBird_Frame[yy][xx];
		}
	}
	FlappyBird_Frame_DrawBird(x,y,mode);
	l_x = x;
	l_y = y;
}

































u16 FlappyBird_Frame_MixColor(u16 color_1,u16 color_2,u8 percent)
{
	u16 R_1,R_2;
	u16 G_1,G_2;
	u16 B_1,B_2;
	u16 R_t,G_t,B_t;
	if( percent > 100 )
	{
		;
	}
	else
	{
		R_1 = color_1 & 0xf800;
		R_1 >>= 11;
		G_1 = color_1 & 0x07e0;
		G_1 >>= 5;
		B_1 = color_1 & 0x001f;
		
		R_2 = color_2 & 0xf800;
		R_2 >>= 11;
		G_2 = color_2 & 0x07e0;
		G_2 >>= 5;
		B_2 = color_2 & 0x001f;
		
		R_t = R_1*percent/100 + R_2*(100-percent)/100;
		G_t = G_1*percent/100 + G_2*(100-percent)/100;
		B_t = B_1*percent/100 + B_2*(100-percent)/100;
		
		return LCD_DecToRGB((u8)R_t,(u8)G_t,(u8)B_t);
	}
	return 0xfe00;
}

u16 FlappyBird_Frame_IntensityControl(u16 color,u8 percent,u8 mode) 
{
	switch(mode)
	{
		case 0:
			return FlappyBird_Frame_MixColor(color,0x0000,percent);
		case 1:
			return FlappyBird_Frame_MixColor(color,0xffff,percent);
		default:
			return 0xfe00;
	}
	
}

void FlappyBird_Frame_AllIntensityControl_1( u8 percent, u8 mode )
{
	u16 x,y;
	for( y = 0; y < 320; y++ )
	{
		for( x = 0; x < 240; x++ )
		{
			FlappyBird_Frame[y][x] = FlappyBird_Frame_IntensityControl(FlappyBird_Frame[y][x],percent,mode);
		}
	}
}

void FlappyBird_Frame_AllIntensityControl( u8 percent, u8 mode )
{
	((TIM_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x3400))->CCR2= percent*2;
}

void FlappyBird_Frame_DrawTag( u16 x, u16 y  )
{
		u16 color,xx = x,yy = y;
		int R,G,B;
		for( yy = y; yy < 84 + y; yy++ )
		{
			for( xx = x; xx < 95 + x; xx++ )
			{
				color = gImage_flappybird_tag[((yy-y)*95+(xx-x))*2+1];
				color <<= 8;
				color = color + gImage_flappybird_tag[((yy-y)*95+(xx-x))*2];
				R = color & 0xf800;
				R >>= 11;
				G = color & 0x07e0;
				G >>= 5;
				B = color & 0x001f;
				if((B - R) > 7)
					;
				else
					FlappyBird_Frame_DrawPoint(xx,yy,color);
			}
		}
}

void FlappyBird_Frame_DrawGetStart( u16 x, u16 y )
{
		u16 color,xx = x,yy = y;
		int R,G,B;
		for( yy = y; yy < 35 + y; yy++ )
		{
			for( xx = x; xx < 129 + x; xx++ )
			{
				color = gImage_flappybird_getstart[((yy-y)*129+(xx-x))*2+1];
				color <<= 8;
				color = color + gImage_flappybird_getstart[((yy-y)*129+(xx-x))*2];
				R = color & 0xf800;
				R >>= 11;
				G = color & 0x07e0;
				G >>= 5;
				B = color & 0x001f;
				if((B - R) > 7)
					;
				else
					FlappyBird_Frame_DrawPoint(xx,yy,color);
			}
		}
}

void FlappyBird_Frame_DrawColumn_bottom( u16 x, u16 y, u8 mode, u8 column )
{
	
		u16 color,xx = x,yy = y;
		int R,G,B;
		if( 0 == mode )
		{
			for( yy = y; yy < 21 + y; yy++ )
			{
				for( xx = x; xx < 39 + x; xx++ )
				{
					color = gImage_flappybird_column_bottom[((yy-y)*39+(xx-x))*2+1];
					color <<= 8;
					color = color + gImage_flappybird_column_bottom[((yy-y)*39+(xx-x))*2];
					R = color & 0xf800;
					R >>= 11;
					G = color & 0x07e0;
					G >>= 5;
					B = color & 0x001f;
					if((B - R) > 7)
						;
					else
						FlappyBird_column_Buff[column][yy][xx] = color;
				}
			}
		}
		else
		{
			for( yy = y + 20; yy >= y; yy-- )
			{
				for( xx = x; xx < 39 + x; xx++ )
				{
					color = gImage_flappybird_column_bottom[((yy-y)*39+(xx-x))*2+1];
					color <<= 8;
					color = color + gImage_flappybird_column_bottom[((yy-y)*39+(xx-x))*2];
					R = color & 0xf800;
					R >>= 11;
					G = color & 0x07e0;
					G >>= 5;
					B = color & 0x001f;
					if((B - R) > 7)
						;
					else
						FlappyBird_column_Buff[column][2*y+20-yy][xx] = color;
				}
			}
		}
}

void FlappyBird_Frame_DrawColumn_middle( u16 x, u16 y, u8 column )
{
	u16 color,xx = x,yy = y;
	int R,G,B;
	for( yy = y; yy < y + 7; yy++ )
	{
		for( xx = x; xx < 35 + x; xx++ )
		{
			color = gImage_flappybird_column_middle[((yy-y)*35+(xx-x))*2+1];
			color <<= 8;
			color = color + gImage_flappybird_column_middle[((yy-y)*35+(xx-x))*2];
			R = color & 0xf800;
			R >>= 11;
			G = color & 0x07e0;
			G >>= 5;
			B = color & 0x001f;
			if((B - R) > 7)
				;
			else
				FlappyBird_column_Buff[column][yy][xx] = color;
		}
	}
}

void FlappyBird_Frame_DrawColumn( int x, u16 y, u8 level, u8 column )
{
	u16 i,yy;
	int xx;
	for( i = 0; i < level; i++ )
	{
		FlappyBird_Frame_DrawColumn_middle(2,i*7,column);
	}
	
	FlappyBird_Frame_DrawColumn_bottom(0,i*7,0,column);
	
	FlappyBird_Frame_DrawColumn_bottom(0,i*7+89,1,column);
	
	for( i = 0; i < 21 - level; i++ )
	{
		FlappyBird_Frame_DrawColumn_middle(2,110+i*7+level*7,column);
	}
	if(x > -42 && x < 0)
	{
		for( yy = y; yy < 259 + y; yy++ )
		{
			for( xx = -x; xx < 39; xx++ )
			{
				if(FlappyBird_column_Buff[column][yy-y][xx]!=0xfe)
				{
					FlappyBird_Frame_DrawPoint(xx+x,yy,FlappyBird_column_Buff[column][yy-y][xx]);
				}
			}
		}
	}
	else
	{
		if(x > 201 && x < 239)
		{
			for( yy = y; yy < 259 + y; yy++ )
			{
				for( xx = x; xx < 240; xx++ )
				{
					if(FlappyBird_column_Buff[column][yy-y][xx-x]!=0xfe)
						FlappyBird_Frame_DrawPoint(xx,yy,FlappyBird_column_Buff[column][yy-y][xx-x]);
				}
			}
		}
		else if(x < 201 && x > 0)
		{
			for( yy = y; yy < 259 + y; yy++ )
			{
				for( xx = x; xx < 39 + x; xx++ )
				{
					FlappyBird_Frame_DrawPoint(xx,yy,FlappyBird_column_Buff[column][yy-y][xx-x]);
				}
			}
		}
	}

}

void FlappyBird_Frame_DrawColumn_Play( int x, u16 y, u8 level, u8 column )
{
	u16 xx,yy;
	if( hide[column] )
	{
		for( yy = l_y_c[column]; yy < l_y_c[column] + 259; yy++ )
		{
			for( xx = l_x_c[column]; xx < l_x_c[column] + 39; xx++ )
			{
				if( yy >= level*7+22 && yy <= level*7+88 )
				{
					;
				}
				else
					FlappyBird_Frame_DrawPoint(xx,yy,FlappyBird_column_Buff_BG[column][yy-l_y_c[column]][xx-l_x_c[column]]);
			}
		}
	}
	if( x > 201 )
	{
		for( yy = y; yy < y+259; yy++ )
		{
			for( xx = 201; xx < 240; xx++ )
			{
				FlappyBird_column_Buff_BG[column][yy-y][xx-201] = FlappyBird_Frame[yy][xx];
				FlappyBird_column_Buff[column][yy-y][xx-201] = 0xfe;
			}
		}
	}
	else
	{
		if( x < 0 )
		{
			for( yy = y; yy < y+259; yy++ )
			{
				for( xx = 0; xx < 39; xx++ )
				{
					FlappyBird_column_Buff_BG[column][yy-y][xx] = FlappyBird_Frame[yy][xx];
					FlappyBird_column_Buff[column][yy-y][xx] = 0xfe;
				}
			}
		}
		else
		{
			for( yy = y; yy < y+259; yy++ )
			{
				for( xx = x; xx < x+39; xx++ )
				{
					FlappyBird_column_Buff_BG[column][yy-y][xx-x] = FlappyBird_Frame[yy][xx];
					FlappyBird_column_Buff[column][yy-y][xx-x] = FlappyBird_Frame[yy][xx];
				}
			}
		}
	}
	FlappyBird_Frame_DrawColumn(x,y,level,column);
	if( x < 0 )
	{
		l_x_c[column] = 0;
	}
	else
	{
		if( x > 201 )
		{
			l_x_c[column] = 201;
		}
		else
		{
			l_x_c[column] = x;
		}
	}
	l_y_c[column] = y;
	hide[column] = 1;
}

void FlappyBird_Frame_DrawScoreBoard( u16 x, u16 y )
{
		u16 color,xx = x,yy = y;
		int R,G,B;
		for( yy = y; yy < 80 + y; yy++ )
		{
			for( xx = x; xx < 160 + x; xx++ )
			{
				color = gImage_flappybird_score_board[((yy-y)*160+(xx-x))*2+1];
				color <<= 8;
				color = color + gImage_flappybird_score_board[((yy-y)*160+(xx-x))*2];
				R = color & 0xf800;
				R >>= 11;
				G = color & 0x07e0;
				G >>= 5;
				B = color & 0x001f;
				if((B - R) > 7)
					;
				else
					FlappyBird_Frame_DrawPoint(xx,yy,color);
			}
		}
}

void FlappyBird_Frame_DrawGameOver( u16 x, u16 y )
{
		u16 color,xx = x,yy = y;
		int R,G,B;
		for( yy = y; yy < 29 + y; yy++ )
		{
			for( xx = x; xx < 135 + x; xx++ )
			{
				color = gImage_flappybird_gameover[((yy-y)*135+(xx-x))*2+1];
				color <<= 8;
				color = color + gImage_flappybird_gameover[((yy-y)*135+(xx-x))*2];
				R = color & 0xf800;
				R >>= 11;
				G = color & 0x07e0;
				G >>= 5;
				B = color & 0x001f;
				if((B - R) > 7)
					;
				else
					FlappyBird_Frame_DrawPoint(xx,yy,color);
			}
		}
}

void FlappyBird_Frame_DrawColumn_Play_times( int x, u16 y, u8 level, u8 column, u8 times )
{
	u8 i;
	
	for( i =0; i < times; i++ )
	{
		FlappyBird_Frame_DrawColumn_Play(x-i,y,level,column);
	}
}

void FlappyBird_Stage_Start( void )
{
	u8 x;
	u8 Start_Flag = 1;
	u8 Key;
	FlappyBird_Frame_Clear();
	FlappyBird_Frame_Fill(0,257,239,257,0x41E7);
	FlappyBird_Frame_Fill(0,258,239,258,0xEFB6);
	FlappyBird_Frame_Fill(0,259,239,259,0xE7F1);
	FlappyBird_Frame_Fill(0,273,239,273,0x5BE3);
	FlappyBird_Frame_Fill(0,274,239,274,0xCD49);
	FlappyBird_Frame_Fill(0,275,239,275,0xCD49);
	FlappyBird_Frame_Fill(0,276,239,320,0xDEB2);
	FlappyBird_DrawGreenBar(0,260,0x9F2B);
	FlappyBird_Frame_DrawBGPic(0,187);
	FlappyBird_Frame_DrawLogo(52,70);
	FlappyBird_Frame_DrawButton(32,213,0);
	FlappyBird_Frame_DrawButton(130,213,1);
	Font_Num_DrawNum_Multi(110,20,difficult,0);
	FlappyBird_DrawFrame();
	BACK_COLOR = 0xDEB2;
	FlappyBird_Frame_ShowString(55,276,200,16,16,"(c) Microbai 2014",0xFFFF);
	while(Start_Flag)
	{
		Key=key_scan();
		if( Key == KEY_UP )
		{
			difficult++;
			if( difficult > 6 )
			{
				difficult = 6;
			}
			Font_Num_DrawNum_Multi(110,20,difficult,0);
		}
		if( Key == KEY_DOWN )
		{
			difficult--;
			if( difficult < 0 )
			{
				difficult = 0;
			}
			Font_Num_DrawNum_Multi(110,20,difficult,0);
		}
		t_pad.scan(0); 		 
		if(t_pad.penStat&0x80)			
		{	
			lcdDisplayOn();
		 	if(t_pad.xoff<Lcd_Dev.width&&t_pad.yoff<Lcd_Dev.height)
			{
					if(t_pad.xoff >= 32 && t_pad.xoff <= 110 && t_pad.yoff <= 257  && t_pad.yoff >= 213)
					{
						FlappyBird_Frame_DrawBGPic(0,187);
						FlappyBird_Frame_DrawButton(32,215,0);
						FlappyBird_Frame_DrawButton(130,213,1);
						FlappyBird_DrawFrame();
						while(t_pad.penStat&0x80)
						{
							t_pad.scan(0);
							FlappyBird_DrawGreenBar_Play(0,260,3);
							FlappyBird_Frame_DrawBird_Play(106,bird_height,bird_mode);
							FlappyBird_DrawFrame();
							if( x%3 == 0 )
								switch(bird_height)
								{
									case 119: 
										bird_height = 120;
										bird_mode = 'd';
										ud_mode = 0;
										break;
									case 120: 
										if( 0 == ud_mode )
										{
											bird_height = 121;
											bird_mode = 'm';
										}
										else
										{
											bird_height = 119;
											bird_mode = 'u';
										}
										break;
									case 121: 
										if( 0 == ud_mode )
										{
											bird_height = 122;
											bird_mode = 'u';
										}
										else
										{
											bird_height = 120;
											bird_mode = 'd';
										}
										break;
									case 122: 
										if( 0 == ud_mode )
										{
											bird_height = 123;
											bird_mode = 'd';
										}
										else
										{
											bird_height = 121;
											bird_mode = 'm';
										}
										break;
									case 123: 
										bird_height = 122;
										bird_mode = 'u';
										ud_mode = 1;
										break;
								}
								x++;
								if(x == 100)
								{
									x = 0;
								}
						}
						FlappyBird_Frame_DrawBGPic(0,187);
						FlappyBird_Frame_DrawButton(32,213,0);
						FlappyBird_Frame_DrawButton(130,213,1);
						FlappyBird_Frame_Fill(0,257,239,257,0x41E7);
						FlappyBird_Frame_Fill(0,258,239,258,0xEFB6);
						FlappyBird_Frame_Fill(0,259,239,259,0xE7F1);
						FlappyBird_DrawFrame();
						for( x = 1; x <= 10; x++)
						{
							if( x%3 == 0 )
								switch(bird_height)
								{
									case 119: 
										bird_height = 120;
										bird_mode = 'd';
										ud_mode = 0;
										break;
									case 120: 
										if( 0 == ud_mode )
										{
											bird_height = 121;
											bird_mode = 'm';
										}
										else
										{
											bird_height = 119;
											bird_mode = 'u';
										}
										break;
									case 121: 
										if( 0 == ud_mode )
										{
											bird_height = 122;
											bird_mode = 'u';
										}
										else
										{
											bird_height = 120;
											bird_mode = 'd';
										}
										break;
									case 122: 
										if( 0 == ud_mode )
										{
											bird_height = 123;
											bird_mode = 'd';
										}
										else
										{
											bird_height = 121;
											bird_mode = 'm';
										}
										break;
									case 123: 
										bird_height = 122;
										bird_mode = 'u';
										ud_mode = 1;
										break;
								}
							FlappyBird_DrawGreenBar_Play(0,260,3);
							FlappyBird_Frame_DrawBird_Play(106,bird_height,bird_mode);
							FlappyBird_Frame_AllIntensityControl(100-x*10,0);
							FlappyBird_DrawFrame();
						}

						Start_Flag = 0;
					}
					if(t_pad.xoff >= 130 && t_pad.xoff <= 208 && t_pad.yoff <= 257  && t_pad.yoff >= 213)
					{
						FlappyBird_Frame_DrawBGPic(0,187);
						FlappyBird_Frame_DrawButton(32,213,0);
						FlappyBird_Frame_DrawButton(130,215,1);
						FlappyBird_DrawFrame();
						while(t_pad.penStat&0x80)
						{
							t_pad.scan(0);
							FlappyBird_DrawGreenBar_Play(0,260,3);
							FlappyBird_Frame_DrawBird_Play(106,bird_height,bird_mode);
							FlappyBird_DrawFrame();
							if( x%3 == 0 )
								switch(bird_height)
								{
									case 119: 
										bird_height = 120;
										bird_mode = 'd';
										ud_mode = 0;
										break;
									case 120: 
										if( 0 == ud_mode )
										{
											bird_height = 121;
											bird_mode = 'm';
										}
										else
										{
											bird_height = 119;
											bird_mode = 'u';
										}
										break;
									case 121: 
										if( 0 == ud_mode )
										{
											bird_height = 122;
											bird_mode = 'u';
										}
										else
										{
											bird_height = 120;
											bird_mode = 'd';
										}
										break;
									case 122: 
										if( 0 == ud_mode )
										{
											bird_height = 123;
											bird_mode = 'd';
										}
										else
										{
											bird_height = 121;
											bird_mode = 'm';
										}
										break;
									case 123: 
										bird_height = 122;
										bird_mode = 'u';
										ud_mode = 1;
										break;
								}
								x++;
								if(x == 100)
								{
									x = 0;
								}
						}
						FlappyBird_Frame_DrawBGPic(0,187);
						FlappyBird_Frame_DrawButton(32,213,0);
						FlappyBird_Frame_DrawButton(130,213,1);
						FlappyBird_Frame_Fill(0,257,239,257,0x41E7);
						FlappyBird_Frame_Fill(0,258,239,258,0xEFB6);
						FlappyBird_Frame_Fill(0,259,239,259,0xE7F1);

						FlappyBird_DrawFrame();
						lcdDisplayOff();
					}
			}
		}
		if( Start_Flag != 0 )
		{
			FlappyBird_DrawGreenBar_Play(0,260,3);
			FlappyBird_Frame_DrawBird_Play(106,bird_height,bird_mode);
			FlappyBird_DrawFrame();
			if( x%3 == 0 )
				switch(bird_height)
				{
					case 119: 
						bird_height = 120;
						bird_mode = 'd';
						ud_mode = 0;
						break;
					case 120: 
						if( 0 == ud_mode )
						{
							bird_height = 121;
							bird_mode = 'm';
						}
						else
						{
							bird_height = 119;
							bird_mode = 'u';
						}
						break;
					case 121: 
						if( 0 == ud_mode )
						{
							bird_height = 122;
							bird_mode = 'u';
						}
						else
						{
							bird_height = 120;
							bird_mode = 'd';
						}
						break;
					case 122: 
						if( 0 == ud_mode )
						{
							bird_height = 123;
							bird_mode = 'd';
						}
						else
						{
							bird_height = 121;
							bird_mode = 'm';
						}
						break;
					case 123: 
						bird_height = 122;
						bird_mode = 'u';
						ud_mode = 1;
						break;
				}
				x++;
				if(x == 100)
				{
					x = 0;
				}
			}
		}
}

void FlappyBird_Stage_PP( void )
{
	u8 x;
	u8 Start_Flag = 1;
	bird_height = bird_height+30;
	FlappyBird_Frame_Clear();
	FlappyBird_Frame_Fill(0,257,239,257,0x41E7);
	FlappyBird_Frame_Fill(0,258,239,258,0xEFB6);
	FlappyBird_Frame_Fill(0,259,239,259,0xE7F1);
	FlappyBird_Frame_Fill(0,273,239,273,0x5BE3);
	FlappyBird_Frame_Fill(0,274,239,274,0xCD49);
	FlappyBird_Frame_Fill(0,275,239,275,0xCD49);
	FlappyBird_Frame_Fill(0,276,239,320,0xDEB2);
	FlappyBird_DrawGreenBar(0,260,0x9F2B);
	FlappyBird_Frame_DrawBGPic(0,187);
	for( x = 1; x <= 10; x++)
	{
		if( x%3 == 0 )
			switch(bird_height)
			{
				case 149: 
					bird_height = 150;
					bird_mode = 'd';
					ud_mode = 0;
					break;
				case 150: 
					if( 0 == ud_mode )
					{
						bird_height = 151;
						bird_mode = 'm';
					}
					else
					{
						bird_height = 149;
						bird_mode = 'u';
					}
					break;
				case 151: 
					if( 0 == ud_mode )
					{
						bird_height = 152;
						bird_mode = 'u';
					}
					else
					{
						bird_height = 150;
						bird_mode = 'd';
					}
					break;
				case 152: 
					if( 0 == ud_mode )
					{
						bird_height = 153;
						bird_mode = 'd';
					}
					else
					{
						bird_height = 151;
						bird_mode = 'm';
					}
					break;
				case 153: 
					bird_height = 152;
					bird_mode = 'u';
					ud_mode = 1;
					break;
			}
		FlappyBird_DrawGreenBar_Play(0,260,3);
		FlappyBird_Frame_DrawBird_Play(57,bird_height,bird_mode);
		FlappyBird_Frame_DrawGetStart(52,70);
		FlappyBird_Frame_DrawTag(70,120);
		FlappyBird_Frame_AllIntensityControl(x*20,0);
		FlappyBird_DrawFrame();
	}
	while(Start_Flag)
	{
		t_pad.scan(0); 		 
		if(t_pad.penStat&0x80)			
		{	
			Start_Flag = 0;
		}
		if(x > 10)
		{
			x = 0;
		}
		if(Start_Flag)
		{
			if( x%3 == 0 )
				switch(bird_height)
				{
					case 149: 
						bird_height = 150;
						bird_mode = 'd';
						ud_mode = 0;
						break;
					case 150: 
						if( 0 == ud_mode )
						{
							bird_height = 151;
							bird_mode = 'm';
						}
						else
						{
							bird_height = 149;
							bird_mode = 'u';
						}
						break;
					case 151: 
						if( 0 == ud_mode )
						{
							bird_height = 152;
							bird_mode = 'u';
						}
						else
						{
							bird_height = 150;
							bird_mode = 'd';
						}
						break;
					case 152: 
						if( 0 == ud_mode )
						{
							bird_height = 153;
							bird_mode = 'd';
						}
						else
						{
							bird_height = 151;
							bird_mode = 'm';
						}
						break;
					case 153: 
						bird_height = 152;
						bird_mode = 'u';
						ud_mode = 1;
						break;
				}
			FlappyBird_DrawGreenBar_Play(0,260,3);
			FlappyBird_Frame_DrawBird_Play(57,bird_height,bird_mode);
			FlappyBird_DrawFrame();
			x++;
		}
	}
	for( x = 1; x <= 10; x++ )
	{
		FlappyBird_Frame_AllIntensityControl(100-x*10,0);
		_delay_ms(30);
	}
}

void FlappyBird_Stage_Play( void )
{
	int x;
	int x_c[3];
	u8 level;
	u16 adcx;
	
	u8 Dead_Flag = 1;
	clk_init(16);
	
	((FSMC_Bank1E_TypeDef *) (((u32)0xA0000000) + 0x0104))->BWTR[6]|=2<<0; 	 
	((FSMC_Bank1E_TypeDef *) (((u32)0xA0000000) + 0x0104))->BWTR[6]|=3<<8;	 	 
 	
	((FSMC_Bank1_TypeDef *) (((u32)0xA0000000) + 0x0000))->BTCR[5]|=6<<8;
	FlappyBird_Frame_Clear();
	FlappyBird_Frame_Fill(0,257,239,257,0x41E7);
	FlappyBird_Frame_Fill(0,258,239,258,0xEFB6);
	FlappyBird_Frame_Fill(0,259,239,259,0xE7F1);
	FlappyBird_Frame_Fill(0,273,239,273,0x5BE3);
	FlappyBird_Frame_Fill(0,274,239,274,0xCD49);
	FlappyBird_Frame_Fill(0,275,239,275,0xCD49);
	FlappyBird_Frame_Fill(0,276,239,320,0xDEB2);
	FlappyBird_DrawGreenBar(0,260,0x9F2B);
	FlappyBird_Frame_DrawBGPic(0,187);
	adcx=Get_Adc_Average(0x01,10);
	srand((unsigned)adcx);
	level = rand()%21; 
	FlappyBird_Frame_DrawColumn_Play_times(239,0,level,0,1);
	Font_Num_DrawNum_Multi(110,40,0,0);
	FlappyBird_DrawFrame();
	for( x = 1; x <= 10; x++ )
	{
		FlappyBird_Frame_AllIntensityControl(x*10,0);
		_delay_ms(30);
	}
	for( x = 0; x<= 10; x++)
	{
			if( x%3 == 0 )
			switch(bird_height)
			{
				case 149: 
					bird_height = 150;
					bird_mode = 'd';
					ud_mode = 0;
					break;
				case 150: 
					if( 0 == ud_mode )
					{
						bird_height = 151;
						bird_mode = 'm';
					}
					else
					{
						bird_height = 149;
						bird_mode = 'u';
					}
					break;
				case 151: 
					if( 0 == ud_mode )
					{
						bird_height = 152;
						bird_mode = 'u';
					}
					else
					{
						bird_height = 150;
						bird_mode = 'd';
					}
					break;
				case 152: 
					if( 0 == ud_mode )
					{
						bird_height = 153;
						bird_mode = 'd';
					}
					else
					{
						bird_height = 151;
						bird_mode = 'm';
					}
					break;
				case 153: 
					bird_height = 152;
					bird_mode = 'u';
					ud_mode = 1;
					break;
			}
		FlappyBird_DrawGreenBar_Play(0,260,3);
		FlappyBird_Frame_DrawBird_Play(57,bird_height,bird_mode);
		FlappyBird_DrawFrame();
		}
		x = 0;
		x_c[0] = 236;
		while(Dead_Flag)
		{
			t_pad.scan(0); 		 
			if(t_pad.penStat&0x80)			
			{
				FlappyBird_Frame_DrawBird_Play(57,bird_height,'u');
				bird_height = bird_height - 6 - difficult;
				bird_mode = 'd';
				FlappyBird_DrawGreenBar_Play(0,260,3);
				
				if(bird_height < 0)
				{
					bird_height = 0;
				}
				
				FlappyBird_Frame_DrawColumn_Play_times(x_c[0],0,level,0,1);
				if( x_c[0] >= 18 && x_c[0] <= 83)
				{
					if(!((bird_height > 7*level+22) && (bird_height < 7*level+71)))
					{
						Dead_Flag = 0;
					}
					else if( 0 == score_lock )
					{
						score_lock = 1;
						score ++;
					}
				}
				Font_Num_DrawNum_Multi(110,40,score,0);
				FlappyBird_DrawFrame();
				x_c[0] = x_c[0] - 3;
				if(x_c[0] < -42)
				{
					adcx=Get_Adc_Average(0x01,10);
					srand((unsigned)adcx);
					level = rand()%21; 
					x_c[0] = 239;
					score_lock = 0;
				}	
			}
			else if(bird_height < 239 && bird_mode != 'f' )
			{
				bird_mode = 'm';
				bird_height = bird_height + 6 + difficult;
				if(bird_height > 238)
				{
					bird_height = 231;
					bird_mode = 'f';
					Dead_Flag = 0;
				}
			}
			if(bird_mode != 'f')
			FlappyBird_DrawGreenBar_Play(0,260,3);
			FlappyBird_Frame_DrawBird_Play(bird_wide,bird_height,bird_mode);
			FlappyBird_Frame_DrawColumn_Play_times(x_c[0],0,level,0,1);
			if( x_c[0] >= 18 && x_c[0] <= 83)
			{
				if(!((bird_height > 7*level+22) && (bird_height < 7*level+71)))
				{
					Dead_Flag = 0;
				}
				else if( 0 == score_lock )
				{
					score_lock = 1;
					score ++;
				}
			}
			Font_Num_DrawNum_Multi(110,40,score,0);
			FlappyBird_DrawFrame();
			x_c[0] = x_c[0] - 3;
			if(x_c[0] < -42)
			{
				adcx=Get_Adc_Average(0x01,10);
				srand((unsigned)adcx);
				level = rand()%21; 
				x_c[0] = 239;
				score_lock = 0;
				
			}	
		}
	
}

void FlappyBird_Stage_Over( void )
{
	u16 i,best_score;
	u8 Over_Flag = 1,AT24CXX_FLAG;
	u8 x;
	clk_init(9);
	FSMC_SRAM_init();
	DMA_Init();
	FlappyBird_Frame_DrawGameOver(52,60);
	FlappyBird_Frame_DrawScoreBoard(40,110);
	FlappyBird_Frame_DrawButton(32,213,0);
	FlappyBird_Frame_DrawButton(130,213,1);
	FlappyBird_DrawFrame();
	AT24CXX_FLAG = (u8)AT24CXX_ReadLenByte(132,1);
	if( AT24CXX_FLAG != 0x55 )
	{
		AT24CXX_WriteLenByte(112,(u32)0x0000,2);
		AT24CXX_WriteLenByte(132,(u32)0x55,1);
	}
	best_score = (u16)AT24CXX_ReadLenByte(112,2);
	if( score > best_score )
	{
		AT24CXX_WriteLenByte(112,(u32)score,2);
	}
	
	for( i = 0; i <= ((score)> (best_score) ? (score):(best_score)); i++ )
	{
		if( i <= score )
		{
			Font_Num_DrawNum_Multi(175,132,i,1);
		}
		Font_Num_DrawNum_Multi(175,162,i,1);
		FlappyBird_DrawFrame();
		_delay_ms(30);
	}
	score = 0;
	
	while(Over_Flag)
	{
		t_pad.scan(0);
		if(t_pad.penStat&0x80)			
		{	
			lcdDisplayOn();
		 	if(t_pad.xoff<Lcd_Dev.width&&t_pad.yoff<Lcd_Dev.height)
			{
					if(t_pad.xoff >= 32 && t_pad.xoff <= 110 && t_pad.yoff <= 257  && t_pad.yoff >= 213)
					{
						FlappyBird_Frame_DrawButton(32,215,0);
						FlappyBird_Frame_DrawButton(130,213,1);
						while(t_pad.penStat&0x80)
						{
							t_pad.scan(0);
							for( x = 1; x <= 10; x++)
							{
								FlappyBird_Frame_AllIntensityControl(100-x*10,0);
								_delay_ms(30);
							}

							Over_Flag = 0;
						}
					}
					if(t_pad.xoff >= 130 && t_pad.xoff <= 208 && t_pad.yoff <= 257  && t_pad.yoff >= 213)
					{
						FlappyBird_Frame_Fill(0,257,239,257,0x41E7);
						FlappyBird_Frame_Fill(0,258,239,258,0xEFB6);
						FlappyBird_Frame_Fill(0,259,239,259,0xE7F1);
						FlappyBird_Frame_DrawButton(32,213,0);
						FlappyBird_Frame_DrawButton(130,215,1);
						FlappyBird_DrawFrame();
						while(t_pad.penStat&0x80)
						{
							t_pad.scan(0);
							lcdDisplayOff();
						}
					}
			}
		}
		FlappyBird_Frame_DrawButton(32,213,0);
		FlappyBird_Frame_DrawButton(130,213,1);
		ud_mode = 0;
		bird_mode = 'u';
		bird_height = 119;
		bird_wide = 57;
		score_lock = 0;
		l_x = 0;
		l_y = 0;
		hide[0] = 0;
		FlappyBird_DrawFrame();
	}
	
}




void FlappyBird_DrawFrame(void)
{
	u16 i,j,tmp;
	for(i = 0;i < 480;i++)
		for(j = 0;j < 320;j++)
		{
			tmp = FlappyBird_Frame[i][j]; 
			FlappyBird_Frame[i][j] = ((tmp&0x1f)<<11) + (tmp&0x07e0) + ((tmp&0xf800)>>11);
		}
	lcdSetWindow(0,0,320,480);
	DMA_Enable();
}


