/*和系统软硬件相关的其他管理*/
#include "mcu_flash.h"
#include "log_power.h"
#include "log_time.h"
#include "log_queue.h"
#include "common.h"
#include "dev_gsm.h"
#include "mcu_adc.h"
#include "mcu_gpio.h"
#include "mcu_systick.h"
#include "mcu_exti.h"
#include "mcu_exti.h"
#include "mcu_usart.h"

#include "dev_mma845x.h"
#include "dev_gps.h"

#include "stm32f10x.h"

#include "stdio.h"
#include "string.h"
#include "stdlib.h"

BOOL			is_mcu_in_stopmode;		// MCU是否处于STOP模式的标志
BOOL			is_sys_in_darkmode = FALSE;	// 系统是否处于黑暗模式(系统初始化后LED灯不点亮)

// battery related
#ifdef USING_PWR_BAT
BOOL			is_bat_in_charging;			// 电池是否在充电的标志

unsigned int		de_batvol = 0.0;				// 电池电压(放大100倍表示，仅在电池脸上时有意义，电池未连上是)	
#endif

unsigned char	sts_power;					// 系统电源的状态

/********************************************************************************************************************
*
*
*
*
*********************************************************************************************************************/
// 系统电源管理功能初始化。
void power_init(void)
{		
	// mcu默认处于非stop模式
	is_mcu_in_stopmode = FALSE;		

	// 系统默认关闭静默模式(即初始化过程中及初始化后都可点亮led灯)
	is_sys_in_darkmode = FALSE;	

#ifdef USING_PWR_BAT
	// 测量电池电压
	de_batvol = batvol_measure();			

	printf("battery voltage = %1.2f.\r\n", de_batvol/(double)100);
#endif

	sts_power = 0x00;

	// ext电源和acc电源状态通过读取io状态设置。
#ifdef USING_PWR_EXT	
	sts_power |= (MCU_GPIO_READ(GPIO_EXTPWR_DET)<<BIT_STS_POWER_EXT);	
#endif

#ifdef USING_PWR_ACC
	sts_power |= (MCU_GPIO_READ(GPIO_ACCPWR_DET)<<BIT_STS_POWER_ACC);
#endif

	// 电池状态通过电池电压比较设置(只要USB或DC外接电源接入，电池电压测量的结果一般为4.10v左右，不管电池有无连接上)。
	if(de_batvol < MAX_ADCVOL_FOR_BAT)
	{
		sts_power |= (1<<BIT_STS_POWER_BAT);
	}
	// else: 不能判断电池是否连接上。
}

__asm void asm_mcu_reset(void)
{
	MOV R0, #1           	; 
 	MSR FAULTMASK, R0    	; // 清除FAULTMASK 禁止一切中断产生
 	LDR R0, =0xE000ED0C  	;
 	LDR R1, =0x05FA0004  	; 
 	STR R1, [R0]         	; // 系统软件复位   
 
deadloop
    B deadloop        		; // 死循环使程序运行不到下面的代码
}

void mcu_reset(void)
{
	asm_mcu_reset();
}

void mcu_shutdown(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;  
	
	led_switch(GPIO_LED_R, OFF);
	led_switch(GPIO_LED_G, OFF);
	led_switch(GPIO_LED_B, OFF);

	// 红灯每隔1秒闪烁3次
	led_flash1(GPIO_LED_R, 1, 3);	
	
#if 1
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_7 | GPIO_Pin_12 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN_FLOATING;	// 开漏输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	 
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN_FLOATING;	// 开漏输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, DISABLE);
#endif

	// 在Flash中特定区域写入特征数值以记录系统为正常关机
	bkp_u16_set(ENV_SYS_NORMAL_STARTUP_START, TAG_START_UP);

	// MCU进入待机模式
	mcu_standby();				
}

//系统进入待机（Standby）模式
void mcu_standby(void)
{			 
	RCC_APB2PeriphResetCmd(0X01FC,DISABLE);
	
	NVIC_SystemLPConfig(NVIC_LP_SLEEPDEEP,ENABLE);	   

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);	// 使能PWR和BKP外设时钟    

	PWR_WakeUpPinCmd(ENABLE);  							// 使能唤醒管脚功能
	
	PWR_EnterSTANDBYMode();	 				 			// 进入待机模式
}

//系统进入停止（Stop）模式。
void mcu_stop_enter(void)
{
	// printf("to put MCU in STOP mode.\r\n");
	
	is_mcu_in_stopmode = TRUE;
	
	// 将MCU至于STOP模式	
	PWR_EnterSTOPMode(PWR_Regulator_LowPower,PWR_STOPEntry_WFI);

	// printf("MCU wakeup from STOP mode.\r\n");
}

// 车载模式下检查外接电源是否介入、便携模式下检查电池是否充电中。
void check_power_status(void)
{
	
	if(GET_STS_POWER(BIT_STS_POWER_EXT) == ON)
	{
		// 绿灯灭、蓝灯亮，指示外接电源接通。
		led_switch(GPIO_LED_G, OFF);
		led_switch(GPIO_LED_B, ON);
	}
	else
	{
		// 绿灯亮、蓝灯灭，指示外接电源断开。
		led_switch(GPIO_LED_G, ON);
		led_switch(GPIO_LED_B, OFF);
	}
}

