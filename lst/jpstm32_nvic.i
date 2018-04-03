#line 1 "core\\jpstm32_nvic.c"







 

#line 1 "core\\jpstm32_nvic.h"







 




#line 1 "core\\jpstm32_common.h"







 



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

 
 
 



 
#line 13 "core\\jpstm32_common.h"
#line 14 "core\\jpstm32_common.h"
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



 
#line 15 "core\\jpstm32_common.h"

 



 





typedef enum{
    RISING = 0,FALLING,BOTH_EDGE
}EXTI_TRIGGER_MODE;


typedef enum{
    IRQ = 0,EVENT,BOTH_INT_EVENT
}INT_EVENT_MODE;


typedef enum{
    PORTA = 0,PORTB,PORTC,PORTD,PORTE,PORTF,PORTG
}PORT;

#line 14 "core\\jpstm32_nvic.h"










 
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


#line 11 "core\\jpstm32_nvic.c"










 
void select_nvic_vector(u32 NVIC_VectTab, u32 Offset)
{
	
	
	((SCB_TypeDef *) (((u32)0xE000E000) + 0x0D00))->VTOR = NVIC_VectTab|(Offset & (u32)0x1FFFFF80);
}


bool nvic_set_enable(u8 Channel)
{
    int temp = 0;
    if((((Channel) == ((u8)0x00)) || ((Channel) == ((u8)0x01)) || ((Channel) == ((u8)0x02)) || ((Channel) == ((u8)0x03)) || ((Channel) == ((u8)0x04)) || ((Channel) == ((u8)0x05)) || ((Channel) == ((u8)0x06)) || ((Channel) == ((u8)0x07)) || ((Channel) == ((u8)0x08)) || ((Channel) == ((u8)0x09)) || ((Channel) == ((u8)0x0A)) || ((Channel) == ((u8)0x0B)) || ((Channel) == ((u8)0x0C)) || ((Channel) == ((u8)0x0D)) || ((Channel) == ((u8)0x0E)) || ((Channel) == ((u8)0x0F)) || ((Channel) == ((u8)0x10)) || ((Channel) == ((u8)0x11)) || ((Channel) == ((u8)0x12)) || ((Channel) == ((u8)0x13)) || ((Channel) == ((u8)0x14)) || ((Channel) == ((u8)0x15)) || ((Channel) == ((u8)0x16)) || ((Channel) == ((u8)0x17)) || ((Channel) == ((u8)0x18)) || ((Channel) == ((u8)0x19)) || ((Channel) == ((u8)0x1A)) || ((Channel) == ((u8)0x1B)) || ((Channel) == ((u8)0x1C)) || ((Channel) == ((u8)0x1D)) || ((Channel) == ((u8)0x1E)) || ((Channel) == ((u8)0x1F)) || ((Channel) == ((u8)0x20)) || ((Channel) == ((u8)0x21)) || ((Channel) == ((u8)0x22)) || ((Channel) == ((u8)0x23)) || ((Channel) == ((u8)0x24)) || ((Channel) == ((u8)0x25)) || ((Channel) == ((u8)0x26)) || ((Channel) == ((u8)0x27)) || ((Channel) == ((u8)0x28)) || ((Channel) == ((u8)0x29)) || ((Channel) == ((u8)0x2A)) || ((Channel) == ((u8)0x2B)) || ((Channel) == ((u8)0x2C)) || ((Channel) == ((u8)0x2D)) || ((Channel) == ((u8)0x2E)) || ((Channel) == ((u8)0x2F)) || ((Channel) == ((u8)0x30)) || ((Channel) == ((u8)0x31)) || ((Channel) == ((u8)0x32)) || ((Channel) == ((u8)0x33)) || ((Channel) == ((u8)0x34)) || ((Channel) == ((u8)0x35)) || ((Channel) == ((u8)0x36)) || ((Channel) == ((u8)0x37)) || ((Channel) == ((u8)0x38)) || ((Channel) == ((u8)0x39)) || ((Channel) == ((u8)0x3A)) || ((Channel) == ((u8)0x3B))) == FALSE)
    {
        return FALSE;
    }
    
    
    ((NVIC_TypeDef *) (((u32)0xE000E000) + 0x0100))->ISER[Channel/32] |= ENABLE << (Channel%32);
    
    
    temp = ((NVIC_TypeDef *) (((u32)0xE000E000) + 0x0100))->ISER[Channel/32];
    temp = temp & (ENABLE << (Channel%32));
    if( (((NVIC_TypeDef *) (((u32)0xE000E000) + 0x0100))->ISER[Channel/32] & (ENABLE << (Channel%32)) ) != FALSE )
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}


bool nvic_clear_enable(u8 Channel)
{
    if((((Channel) == ((u8)0x00)) || ((Channel) == ((u8)0x01)) || ((Channel) == ((u8)0x02)) || ((Channel) == ((u8)0x03)) || ((Channel) == ((u8)0x04)) || ((Channel) == ((u8)0x05)) || ((Channel) == ((u8)0x06)) || ((Channel) == ((u8)0x07)) || ((Channel) == ((u8)0x08)) || ((Channel) == ((u8)0x09)) || ((Channel) == ((u8)0x0A)) || ((Channel) == ((u8)0x0B)) || ((Channel) == ((u8)0x0C)) || ((Channel) == ((u8)0x0D)) || ((Channel) == ((u8)0x0E)) || ((Channel) == ((u8)0x0F)) || ((Channel) == ((u8)0x10)) || ((Channel) == ((u8)0x11)) || ((Channel) == ((u8)0x12)) || ((Channel) == ((u8)0x13)) || ((Channel) == ((u8)0x14)) || ((Channel) == ((u8)0x15)) || ((Channel) == ((u8)0x16)) || ((Channel) == ((u8)0x17)) || ((Channel) == ((u8)0x18)) || ((Channel) == ((u8)0x19)) || ((Channel) == ((u8)0x1A)) || ((Channel) == ((u8)0x1B)) || ((Channel) == ((u8)0x1C)) || ((Channel) == ((u8)0x1D)) || ((Channel) == ((u8)0x1E)) || ((Channel) == ((u8)0x1F)) || ((Channel) == ((u8)0x20)) || ((Channel) == ((u8)0x21)) || ((Channel) == ((u8)0x22)) || ((Channel) == ((u8)0x23)) || ((Channel) == ((u8)0x24)) || ((Channel) == ((u8)0x25)) || ((Channel) == ((u8)0x26)) || ((Channel) == ((u8)0x27)) || ((Channel) == ((u8)0x28)) || ((Channel) == ((u8)0x29)) || ((Channel) == ((u8)0x2A)) || ((Channel) == ((u8)0x2B)) || ((Channel) == ((u8)0x2C)) || ((Channel) == ((u8)0x2D)) || ((Channel) == ((u8)0x2E)) || ((Channel) == ((u8)0x2F)) || ((Channel) == ((u8)0x30)) || ((Channel) == ((u8)0x31)) || ((Channel) == ((u8)0x32)) || ((Channel) == ((u8)0x33)) || ((Channel) == ((u8)0x34)) || ((Channel) == ((u8)0x35)) || ((Channel) == ((u8)0x36)) || ((Channel) == ((u8)0x37)) || ((Channel) == ((u8)0x38)) || ((Channel) == ((u8)0x39)) || ((Channel) == ((u8)0x3A)) || ((Channel) == ((u8)0x3B))) == FALSE)
    {
        return FALSE;
    }
    
    
    ((NVIC_TypeDef *) (((u32)0xE000E000) + 0x0100))->ICER[Channel/32] |= ENABLE << (Channel%32);
    
    
    if( (((NVIC_TypeDef *) (((u32)0xE000E000) + 0x0100))->ICER[Channel/32] & (ENABLE << (Channel%32)) ) == DISABLE )
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}


bool nvic_set_pending(u8 Channel)
{
    if((((Channel) == ((u8)0x00)) || ((Channel) == ((u8)0x01)) || ((Channel) == ((u8)0x02)) || ((Channel) == ((u8)0x03)) || ((Channel) == ((u8)0x04)) || ((Channel) == ((u8)0x05)) || ((Channel) == ((u8)0x06)) || ((Channel) == ((u8)0x07)) || ((Channel) == ((u8)0x08)) || ((Channel) == ((u8)0x09)) || ((Channel) == ((u8)0x0A)) || ((Channel) == ((u8)0x0B)) || ((Channel) == ((u8)0x0C)) || ((Channel) == ((u8)0x0D)) || ((Channel) == ((u8)0x0E)) || ((Channel) == ((u8)0x0F)) || ((Channel) == ((u8)0x10)) || ((Channel) == ((u8)0x11)) || ((Channel) == ((u8)0x12)) || ((Channel) == ((u8)0x13)) || ((Channel) == ((u8)0x14)) || ((Channel) == ((u8)0x15)) || ((Channel) == ((u8)0x16)) || ((Channel) == ((u8)0x17)) || ((Channel) == ((u8)0x18)) || ((Channel) == ((u8)0x19)) || ((Channel) == ((u8)0x1A)) || ((Channel) == ((u8)0x1B)) || ((Channel) == ((u8)0x1C)) || ((Channel) == ((u8)0x1D)) || ((Channel) == ((u8)0x1E)) || ((Channel) == ((u8)0x1F)) || ((Channel) == ((u8)0x20)) || ((Channel) == ((u8)0x21)) || ((Channel) == ((u8)0x22)) || ((Channel) == ((u8)0x23)) || ((Channel) == ((u8)0x24)) || ((Channel) == ((u8)0x25)) || ((Channel) == ((u8)0x26)) || ((Channel) == ((u8)0x27)) || ((Channel) == ((u8)0x28)) || ((Channel) == ((u8)0x29)) || ((Channel) == ((u8)0x2A)) || ((Channel) == ((u8)0x2B)) || ((Channel) == ((u8)0x2C)) || ((Channel) == ((u8)0x2D)) || ((Channel) == ((u8)0x2E)) || ((Channel) == ((u8)0x2F)) || ((Channel) == ((u8)0x30)) || ((Channel) == ((u8)0x31)) || ((Channel) == ((u8)0x32)) || ((Channel) == ((u8)0x33)) || ((Channel) == ((u8)0x34)) || ((Channel) == ((u8)0x35)) || ((Channel) == ((u8)0x36)) || ((Channel) == ((u8)0x37)) || ((Channel) == ((u8)0x38)) || ((Channel) == ((u8)0x39)) || ((Channel) == ((u8)0x3A)) || ((Channel) == ((u8)0x3B))) == FALSE)
    {
        return FALSE;
    }
    
    
    ((NVIC_TypeDef *) (((u32)0xE000E000) + 0x0100))->ISPR[Channel/32] = ENABLE << (Channel%32);
    
    
    if( (((NVIC_TypeDef *) (((u32)0xE000E000) + 0x0100))->ISPR[Channel/32]&(ENABLE<<(Channel%32))) == ENABLE )
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

bool nvic_clear_pending(u8 Channel)
{
    if((((Channel) == ((u8)0x00)) || ((Channel) == ((u8)0x01)) || ((Channel) == ((u8)0x02)) || ((Channel) == ((u8)0x03)) || ((Channel) == ((u8)0x04)) || ((Channel) == ((u8)0x05)) || ((Channel) == ((u8)0x06)) || ((Channel) == ((u8)0x07)) || ((Channel) == ((u8)0x08)) || ((Channel) == ((u8)0x09)) || ((Channel) == ((u8)0x0A)) || ((Channel) == ((u8)0x0B)) || ((Channel) == ((u8)0x0C)) || ((Channel) == ((u8)0x0D)) || ((Channel) == ((u8)0x0E)) || ((Channel) == ((u8)0x0F)) || ((Channel) == ((u8)0x10)) || ((Channel) == ((u8)0x11)) || ((Channel) == ((u8)0x12)) || ((Channel) == ((u8)0x13)) || ((Channel) == ((u8)0x14)) || ((Channel) == ((u8)0x15)) || ((Channel) == ((u8)0x16)) || ((Channel) == ((u8)0x17)) || ((Channel) == ((u8)0x18)) || ((Channel) == ((u8)0x19)) || ((Channel) == ((u8)0x1A)) || ((Channel) == ((u8)0x1B)) || ((Channel) == ((u8)0x1C)) || ((Channel) == ((u8)0x1D)) || ((Channel) == ((u8)0x1E)) || ((Channel) == ((u8)0x1F)) || ((Channel) == ((u8)0x20)) || ((Channel) == ((u8)0x21)) || ((Channel) == ((u8)0x22)) || ((Channel) == ((u8)0x23)) || ((Channel) == ((u8)0x24)) || ((Channel) == ((u8)0x25)) || ((Channel) == ((u8)0x26)) || ((Channel) == ((u8)0x27)) || ((Channel) == ((u8)0x28)) || ((Channel) == ((u8)0x29)) || ((Channel) == ((u8)0x2A)) || ((Channel) == ((u8)0x2B)) || ((Channel) == ((u8)0x2C)) || ((Channel) == ((u8)0x2D)) || ((Channel) == ((u8)0x2E)) || ((Channel) == ((u8)0x2F)) || ((Channel) == ((u8)0x30)) || ((Channel) == ((u8)0x31)) || ((Channel) == ((u8)0x32)) || ((Channel) == ((u8)0x33)) || ((Channel) == ((u8)0x34)) || ((Channel) == ((u8)0x35)) || ((Channel) == ((u8)0x36)) || ((Channel) == ((u8)0x37)) || ((Channel) == ((u8)0x38)) || ((Channel) == ((u8)0x39)) || ((Channel) == ((u8)0x3A)) || ((Channel) == ((u8)0x3B))) == FALSE)
    {
        return FALSE;
    }
    
    
    ((NVIC_TypeDef *) (((u32)0xE000E000) + 0x0100))->ICPR[Channel/32] = ENABLE << (Channel%32);
    
    
    if( (((NVIC_TypeDef *) (((u32)0xE000E000) + 0x0100))->ICPR[Channel/32]&(ENABLE<<(Channel%32)) ) == DISABLE )
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}


bool nvic_get_active(u8 Channel)
{
    if((((Channel) == ((u8)0x00)) || ((Channel) == ((u8)0x01)) || ((Channel) == ((u8)0x02)) || ((Channel) == ((u8)0x03)) || ((Channel) == ((u8)0x04)) || ((Channel) == ((u8)0x05)) || ((Channel) == ((u8)0x06)) || ((Channel) == ((u8)0x07)) || ((Channel) == ((u8)0x08)) || ((Channel) == ((u8)0x09)) || ((Channel) == ((u8)0x0A)) || ((Channel) == ((u8)0x0B)) || ((Channel) == ((u8)0x0C)) || ((Channel) == ((u8)0x0D)) || ((Channel) == ((u8)0x0E)) || ((Channel) == ((u8)0x0F)) || ((Channel) == ((u8)0x10)) || ((Channel) == ((u8)0x11)) || ((Channel) == ((u8)0x12)) || ((Channel) == ((u8)0x13)) || ((Channel) == ((u8)0x14)) || ((Channel) == ((u8)0x15)) || ((Channel) == ((u8)0x16)) || ((Channel) == ((u8)0x17)) || ((Channel) == ((u8)0x18)) || ((Channel) == ((u8)0x19)) || ((Channel) == ((u8)0x1A)) || ((Channel) == ((u8)0x1B)) || ((Channel) == ((u8)0x1C)) || ((Channel) == ((u8)0x1D)) || ((Channel) == ((u8)0x1E)) || ((Channel) == ((u8)0x1F)) || ((Channel) == ((u8)0x20)) || ((Channel) == ((u8)0x21)) || ((Channel) == ((u8)0x22)) || ((Channel) == ((u8)0x23)) || ((Channel) == ((u8)0x24)) || ((Channel) == ((u8)0x25)) || ((Channel) == ((u8)0x26)) || ((Channel) == ((u8)0x27)) || ((Channel) == ((u8)0x28)) || ((Channel) == ((u8)0x29)) || ((Channel) == ((u8)0x2A)) || ((Channel) == ((u8)0x2B)) || ((Channel) == ((u8)0x2C)) || ((Channel) == ((u8)0x2D)) || ((Channel) == ((u8)0x2E)) || ((Channel) == ((u8)0x2F)) || ((Channel) == ((u8)0x30)) || ((Channel) == ((u8)0x31)) || ((Channel) == ((u8)0x32)) || ((Channel) == ((u8)0x33)) || ((Channel) == ((u8)0x34)) || ((Channel) == ((u8)0x35)) || ((Channel) == ((u8)0x36)) || ((Channel) == ((u8)0x37)) || ((Channel) == ((u8)0x38)) || ((Channel) == ((u8)0x39)) || ((Channel) == ((u8)0x3A)) || ((Channel) == ((u8)0x3B))) == FALSE)
    {
        return FALSE;
    }
    
    return( (((NVIC_TypeDef *) (((u32)0xE000E000) + 0x0100))->IABR[Channel/32] & (ENABLE<<(Channel%32))) ? TRUE:FALSE);
}

bool nvic_set_group(u8 group)
{
    u32 temp = 0;
    temp = ((SCB_TypeDef *) (((u32)0xE000E000) + 0x0D00))->AIRCR;  
	temp&=0X0000F8FF; 
    if(group > 4)
    {
        return FALSE;
    }
    temp |= 0x05FA0000;
    temp |= (group+3)<<8;
    ((SCB_TypeDef *) (((u32)0xE000E000) + 0x0D00))->AIRCR = temp;
    if( (((((SCB_TypeDef *) (((u32)0xE000E000) + 0x0D00))->AIRCR >> 8)&0x07)-3) == group )
    {
        return TRUE;
    }
    return FALSE;
}

bool nvic_set_priority(u8 Channel, u8 PreemptionPriority, u8 SubPriority)
{
    u8 group = 0;
    u8 priority = 0;
    group = (((((SCB_TypeDef *) (((u32)0xE000E000) + 0x0D00))->AIRCR >> 8)&0x07)-3);
    switch (group)
    {
        case 0:
            
            priority = (PreemptionPriority & 0xf);
            break;
        case 1:
            priority = ( (PreemptionPriority & 0x7)<<1 )|(SubPriority & 0x1);
            break;
        case 2:
            priority = ( (PreemptionPriority & 0x3)<<2 )|(SubPriority & 0x3);
            break;
        case 3:
            priority = ( (PreemptionPriority & 0x1)<<3 )|(SubPriority & 0x7);
            break;
    }    
    ((NVIC_TypeDef *) (((u32)0xE000E000) + 0x0100))->IPR[Channel>>2] &= ~( (0xf<<4) << ((Channel%4)<<3) );
    ((NVIC_TypeDef *) (((u32)0xE000E000) + 0x0100))->IPR[Channel>>2] |= ((priority<<4) << ((Channel%4)<<3) );
    return TRUE;
}
vu32 nvic_get_priority(void)
{
    return FALSE;
}

bool unregister_nvic(void)
{
    return FALSE;
}
















 
bool register_nvic(u8 nvic_Group, u8 nvic_PreemptionPriority, u8 nvic_SubPriority, u8 nvic_Channel)
{
    bool ret = TRUE;
    ret = nvic_set_group(nvic_Group);
    ret = nvic_set_priority(nvic_Channel, nvic_PreemptionPriority, nvic_SubPriority);
    
    return ret;
}


