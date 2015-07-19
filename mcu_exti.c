#include "mcu_exti.h"
#include "mcu_key.h"
#include "stm32f10x.h"
#include "common.h"

/********************************************************************************************************************
*
*
*
*
*********************************************************************************************************************/
T_MCU_EXTI	mcu_exti_map[7]=
{
	{GPIOA, GPIO_Pin_0	},		//	SYS_KEY			
	{GPIOB, GPIO_Pin_3	},		//	GSM_RING	
	{GPIOA, GPIO_Pin_8	},		//	GSR_INT2		
	{GPIOA, GPIO_Pin_11	},		//	GSR_INT1		    
	{GPIOA, GPIO_Pin_6	},		//	CHG_STS		

#ifdef USING_PWR_EXT	
	{GPIOB, GPIO_Pin_12	},		//	EXTPWR_DET ???
#endif

#ifdef USING_PWR_ACC
	#if (HW_VER == 17)
	{GPIOB, GPIO_Pin_9	},		
	#elif (HW_VER == 16)
	{GPIOB, GPIO_Pin_8	},		
	#endif
#endif
};

// 外部中断I/O口及中断输入开关初始化。
void EXTI_Configure(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	EXTI_InitTypeDef EXTI_InitStructure;

	/* config the extiline MCU_CLOCK_MODE and AFIO MCU_CLOCK_MODE */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	// PA0 --> SYS_KEY		// 开关机按键
  	GPIO_InitStructure.GPIO_Pin 	= mcu_exti_map[EXTI_SYS_KEY].GPIO_Pin;      					
  	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IPD;					
  	GPIO_Init(mcu_exti_map[EXTI_SYS_KEY].GPIOx, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0); 		
  	EXTI_InitStructure.EXTI_Line 	= EXTI_Line0;
  	EXTI_InitStructure.EXTI_Mode 	= EXTI_Mode_Interrupt;			
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; 					// 上升沿触发
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);

	// PB3 --> GSM_RING		// GSM 来电、来短信指示
  	GPIO_InitStructure.GPIO_Pin 	= mcu_exti_map[EXTI_GSM_RING].GPIO_Pin;              			
  	// GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IPU;					// 带内部上拉的中断模式?
  	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN_FLOATING;				// 带内部上拉的中断模式?
  	GPIO_Init(mcu_exti_map[EXTI_GSM_RING].GPIOx, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3); 
	EXTI_InitStructure.EXTI_Line 	= EXTI_Line3;
	EXTI_InitStructure.EXTI_Mode 	= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 				// 下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure); 	

#if 0	
	// PA8 --> GSR_INT2		// G-Sensor INT2
  	GPIO_InitStructure.GPIO_Pin 	= mcu_exti_map[EXTI_GSR_INT2].GPIO_Pin;              			
  	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN_FLOATING;	 		
  	GPIO_Init(mcu_exti_map[EXTI_GSR_INT2].GPIOx, &GPIO_InitStructure);			
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8); 
  	EXTI_InitStructure.EXTI_Line 	= EXTI_Line8;
  	EXTI_InitStructure.EXTI_Mode 	= EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 				// 下降沿触发
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	
#endif

#if 1
	// PA11 --> GSR_INT1	// G-Sensor INT1
  	GPIO_InitStructure.GPIO_Pin 	= mcu_exti_map[EXTI_GSR_INT1].GPIO_Pin;              		
  	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN_FLOATING;	 		 	 
  	GPIO_Init(mcu_exti_map[EXTI_GSR_INT1].GPIOx, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource11); 
  	EXTI_InitStructure.EXTI_Line 	= EXTI_Line11;
  	EXTI_InitStructure.EXTI_Mode 	= EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 				// 下降沿触发
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure); 	
#endif 

#if 0
	// CHG_STS充电状态指示中断容易波动，因此改用ACC电源插入/断开检测+电池电压测量检测是否充电及充电是否完成
	// PA6 --> CHG_STS		// 锂电池是否充满状态指示(未充满时为高电平、充满后为低电平，需要读取电平状态)
  	GPIO_InitStructure.GPIO_Pin 	= mcu_exti_map[EXTI_CHG_STS].GPIO_Pin;              			 		 
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IPU;	 		 
  	GPIO_Init(mcu_exti_map[EXTI_CHG_STS].GPIOx, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource6); 
	EXTI_InitStructure.EXTI_Line 	= EXTI_Line6;
	EXTI_InitStructure.EXTI_Mode 	= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 			// 双边沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;								// TBD
	EXTI_Init(&EXTI_InitStructure); 
#endif

#ifdef USING_PWR_ACC
	#if (HW_VER == 17)
	
	// PB9 --> ACCPWR_DET		// ACC接入/断开检测(接入时为高电平、断开时为低电平，需要读取电平状态)
  	GPIO_InitStructure.GPIO_Pin 	= mcu_exti_map[EXTI_ACCPWR_DET].GPIO_Pin;              			
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IPD;	 		 	// 设置成IPU或IN_FLOATING模式都不能正常产生ACC检测中断处理
  	GPIO_Init(mcu_exti_map[EXTI_ACCPWR_DET].GPIOx, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9); 
	EXTI_InitStructure.EXTI_Line 	= EXTI_Line9;
	EXTI_InitStructure.EXTI_Mode 	= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 			// 双边沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure); 
	
	#elif (HW_VER == 16)

	// PB8 --> ACCPWR_DET		// ACC接入/断开检测(接入时为高电平、断开时为低电平，需要读取电平状态)
  	GPIO_InitStructure.GPIO_Pin 	= mcu_exti_map[EXTI_ACCPWR_DET].GPIO_Pin;              			
  	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IPD;	 		 			// 设置成IPU或IN_FLOATING模式都不能正常产生ACC检测中断处理
   	GPIO_Init(mcu_exti_map[EXTI_ACCPWR_DET].GPIOx, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8); 
	EXTI_InitStructure.EXTI_Line 	= EXTI_Line8;
	EXTI_InitStructure.EXTI_Mode 	= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 			// 双边沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure); 
	#endif
#endif

#ifdef USING_PWR_EXT
	// PB12 --> EXTPWR_DET		// EXT接入/断开检测(接入时为高电平、断开时为低电平，需要读取电平状态)
  	GPIO_InitStructure.GPIO_Pin 	= mcu_exti_map[EXTI_EXTPWR_DET].GPIO_Pin;              			
  	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IPD;	 		 			// 设置成IPU或IN_FLOATING模式都不能正常产生ACC检测中断处理
  	GPIO_Init(mcu_exti_map[EXTI_EXTPWR_DET].GPIOx, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12); 
	EXTI_InitStructure.EXTI_Line 	= EXTI_Line12;
	EXTI_InitStructure.EXTI_Mode 	= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 			// 双边沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure); 	
#endif	

#if 0
	// Line 17 --> RTC Alarm interrupt(STM32 MCU将RTC中断作为外部中断处理，因而RTC中断能唤醒MCU)
	EXTI_InitStructure.EXTI_Line 	= EXTI_Line17;
	EXTI_InitStructure.EXTI_Mode 	= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;				// 上升沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure); 
#endif
}

