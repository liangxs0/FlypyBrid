#line 1 "devices\\timer.c"
#line 1 "devices\\timer.h"
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

#line 4 "devices\\timer.h"









void TIM3_Int_Init(u16 arr,u16 psc);
void TIM3_PWM_Init(u16 arr,u16 psc);
void TIM5_Cap_Init(u16 arr,u16 psc);
void TIM4_PWM_Init(u16 arr,u16 psc);
void TIM6_Int_Init(u16 arr,u16 psc);
void LCD_PWM_Init(void);





#line 2 "devices\\timer.c"
#line 1 "devices\\led.h"







 



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




#line 13 "devices\\led.h"
#line 14 "devices\\led.h"








typedef enum{
	LED_ON, LED_OFF
} LED_STAT;

extern void led_init(void);

extern void led_setR(LED_STAT);
extern void led_setG(LED_STAT);
extern void led_setB(LED_STAT);
extern void led_col(void);


#line 3 "devices\\timer.c"
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


#line 4 "devices\\timer.c"


void TIM3_IRQHandler(void)
{ 		    		  			    
	if ((((TIM_TypeDef *) (((u32)0x40000000) + 0x0400))->SR & 0x0001)) 
	{
		((TIM_TypeDef *) (((u32)0x40000000) + 0x0400))->SR &= (~0x0001);  
		*((vu32*)(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1000))->ODR)&0xF0000000)+0x2000000 +(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1000))->ODR)&0xFFFFF)<<5) + (1<<2)))=! *((vu32*)(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1000))->ODR)&0xF0000000)+0x2000000 +(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1000))->ODR)&0xFFFFF)<<5) + (1<<2)));
	}
}





void TIM3_Int_Init(u16 arr,u16 psc)
{
	((RCC_TypeDef *) ((((u32)0x40000000) + 0x20000) + 0x1000))->APB1ENR |= 1<<1; 

	((TIM_TypeDef *) (((u32)0x40000000) + 0x0400))->ARR = arr; 
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0400))->PSC =psc; 
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0400))->CR1 &= ~(1<<8); 
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0400))->CR1 &= ~(1<<4) ;  
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0400))->DIER |= 0x0041;
	
	register_nvic(2,0,3,((u8)0x1D));
	
	
	
	
	
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0400))->CR1 |= 1<<0;
	
}




void TIM3_PWM_Init(u16 arr,u16 psc)
{
	((RCC_TypeDef *) ((((u32)0x40000000) + 0x20000) + 0x1000))->APB1ENR |= 1<<1; 
 	((RCC_TypeDef *) ((((u32)0x40000000) + 0x20000) + 0x1000))->APB2ENR |= 1<<0 | 1<<3;
	
	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->CRL &= 0xff0fffff;
	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->CRL |= 0x00b00000;
	
	((RCC_TypeDef *) ((((u32)0x40000000) + 0x20000) + 0x1000))->APB2ENR|=1<<0;     
	((AFIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0000))->MAPR&=0XFFFFF3FF; 
	((AFIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0000))->MAPR|=1<<11;      
	

	((TIM_TypeDef *) (((u32)0x40000000) + 0x0400))->ARR = arr; 
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0400))->PSC =psc; 
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0400))->CR1 &= ~(1<<8); 
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0400))->CR1 &= ~(1<<4) ;  
	 
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0400))->CCMR1 |= 7<<12; 
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0400))->CCMR1 |= 1<<11; 
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0400))->CCR2 = 0; 
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0400))->CCER |= (0<<4); 
	
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0400))->CR1 |= 0x0080;
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0400))->CR1 |= 0x0001;
}



void TIM5_Cap_Init(u16 arr,u16 psc)
{
	((RCC_TypeDef *) ((((u32)0x40000000) + 0x20000) + 0x1000))->APB1ENR |= 0x00000008;
	((RCC_TypeDef *) ((((u32)0x40000000) + 0x20000) + 0x1000))->APB2ENR |= 0x00000004;			


	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0800))->BRR = 1<<0;
	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0800))->CRL |= 0;
	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0800))->BRR = 1<<0;

	
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0C00))->ARR = arr;
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0C00))->PSC = psc;
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0C00))->CR1 &= ~(1<<8);
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0C00))->CR1 &= ~(1<<4);
	
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0C00))->CCMR1 |= 1<<0;
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0C00))->CCER &= ~(1<<1);
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0C00))->CCMR1 &= ~(0x3<<2);
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0C00))->CCMR1 &= ~(0xf<<4);

  
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0C00))->CR1 |= 1<<0;
  ((TIM_TypeDef *) (((u32)0x40000000) + 0x0C00))->DMAR |= 1<<0 | 1<<1;


	
	
	
	
	register_nvic(2,2,0,((u8)0x32));
	   
}





u8  TIM5CH1_CAPTURE_STA=0;	
u16	TIM5CH1_CAPTURE_VAL;	

void TIM5_IRQHandler(void)
{ 		    
	if((TIM5CH1_CAPTURE_STA&0X80)==0)
	{	  
		if ((((TIM_TypeDef *) (((u32)0x40000000) + 0x0C00))->SR & 1) != 0 && (((TIM_TypeDef *) (((u32)0x40000000) + 0x0C00))->DIER & 1) != 0)
		{
			if(TIM5CH1_CAPTURE_STA&0X40)
			{
				if((TIM5CH1_CAPTURE_STA&0X3F)==0X3F)
				{
					TIM5CH1_CAPTURE_STA|=0X80;
					TIM5CH1_CAPTURE_VAL=0XFFFF;
				}else TIM5CH1_CAPTURE_STA++;
			}	 
		}
	if ((((TIM_TypeDef *) (((u32)0x40000000) + 0x0C00))->SR & 1<<1) != 0 && (((TIM_TypeDef *) (((u32)0x40000000) + 0x0C00))->DIER & 1<<1) != 0)
		{	
			if(TIM5CH1_CAPTURE_STA&0X40)		
			{	  			
				TIM5CH1_CAPTURE_STA|=0X80;		
				TIM5CH1_CAPTURE_VAL= ((TIM_TypeDef *) (((u32)0x40000000) + 0x0C00))->CCR1;
		   	((TIM_TypeDef *) (((u32)0x40000000) + 0x0C00))->CCER &= ~(1<<1); 
			}else  								
			{
				TIM5CH1_CAPTURE_STA=0;			
				TIM5CH1_CAPTURE_VAL=0;
	 			((TIM_TypeDef *) (((u32)0x40000000) + 0x0C00))->CNT = 0;
				TIM5CH1_CAPTURE_STA|=0X40;		
		   	((TIM_TypeDef *) (((u32)0x40000000) + 0x0C00))->CCER |= 1<<1;		
			}
		}	     	    					   
 	}
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0C00))->SR = ~(1<<0 | 1<<1);
}




void TIM4_PWM_Init(u16 arr,u16 psc)
{

	((RCC_TypeDef *) ((((u32)0x40000000) + 0x20000) + 0x1000))->APB1ENR |= 1<<2;
	((RCC_TypeDef *) ((((u32)0x40000000) + 0x20000) + 0x1000))->APB2ENR |= 1<<3;
 
   
	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->CRL |= 0x2<<26;
	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->CRL |= 0x3<<24;
	
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0800))->ARR = arr; 
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0800))->PSC =psc; 
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0800))->CR1 &= ~(1<<8);
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0800))->CR1 &= ~(1<<4);
	
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0800))->CCMR1 |= 0x7<<4;
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0800))->CCMR1 |= 1<<3;
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0800))->CCR1 = 0;
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0800))->CCER |= 1<<1;
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0800))->CCER |= 1<<0;
	
}
u8 ov_frame; 	

void TIM6_IRQHandler(void)
{
	if (((TIM_TypeDef *) (((u32)0x40000000) + 0x1000))->SR & 1<<0) 
	{
			*((vu32*)(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1000))->ODR)&0xF0000000)+0x2000000 +(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1000))->ODR)&0xFFFFF)<<5) + (2<<2)))=! *((vu32*)(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1000))->ODR)&0xF0000000)+0x2000000 +(((vu32)(&((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1000))->ODR)&0xFFFFF)<<5) + (2<<2)));
			printf("frame:%dfps\r\n",ov_frame);	
			ov_frame=0;	
	} 
	((TIM_TypeDef *) (((u32)0x40000000) + 0x1000))->SR &= ~(1<<0);  
}





void TIM6_Int_Init(u16 arr,u16 psc)
{
	
	((RCC_TypeDef *) ((((u32)0x40000000) + 0x20000) + 0x1000))->APB1ENR |= 1<<4;

	((TIM_TypeDef *) (((u32)0x40000000) + 0x1000))->ARR = arr; 
	((TIM_TypeDef *) (((u32)0x40000000) + 0x1000))->PSC =psc; 
	((TIM_TypeDef *) (((u32)0x40000000) + 0x1000))->CR1 &= ~(1<<8);
	((TIM_TypeDef *) (((u32)0x40000000) + 0x1000))->CR1 &= ~(1<<4);

	((TIM_TypeDef *) (((u32)0x40000000) + 0x1000))->DIER |= 1<<0;
 
	((TIM_TypeDef *) (((u32)0x40000000) + 0x1000))->CR1 |= 1<<0; 
 	
  
	
	
	
	register_nvic(2,1,3,((u8)0x32));
}





void LCD_PWM_Init(void)
{		 					 
	
	((RCC_TypeDef *) ((((u32)0x40000000) + 0x20000) + 0x1000))->APB2ENR|=1<<13; 	
	((RCC_TypeDef *) ((((u32)0x40000000) + 0x20000) + 0x1000))->APB2ENR|=1<<3;    	
	  	
	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->CRL&=0XFFFFFFF0;	
	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->CRL|=0X0000000B;	
	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->ODR|=1<<0;		

	((TIM_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x3400))->ARR=110;			
	((TIM_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x3400))->PSC=0;			
	
	((TIM_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x3400))->CCMR1|=7<<12; 	
	((TIM_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x3400))->CCMR1|=1<<11; 	

	((TIM_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x3400))->CCER|=1<<6;   	
 	((TIM_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x3400))->CCER|=1<<7;   	
	((TIM_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x3400))->BDTR|=1<<15;   	

	((TIM_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x3400))->CR1=0x0080;   	
	((TIM_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x3400))->CR1|=0x01;    	
} 

