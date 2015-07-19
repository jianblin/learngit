#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "common.h"

#include "log_time.h"
#include "log_queue.h"
#include "log_pos.h"
#include "log_time.h"
#include "log_motion.h"

#include "log_power.h"

#include "mcu_exti.h"
#include "mcu_adc.h"
#include "mcu_flash.h"
#include "mcu_usart.h"
#include "mcu_gpio.h"
#include "mcu_i2c.h"
#include "mcu_systick.h"
#include "mcu_systick.h"
#include "mcu_timer.h"

#include "mcu_key.h"
#include "mcu_usart.h"

#include "dev_mma845x.h"
#include "dev_gps.h"
#include "dev_gsm.h"

#include "stm32f10x.h"
#include "stm32f10x_it.h"

#include "example.h"

void mcu_init(void);
void dev_init(void);
void log_init(void);

void IWDG_Configure(void);
void NVIC_Configure(void);

BOOL is_sys_idle 	= FALSE;

void rcc_hsi_8mhz(void);
void rcc_hsi_64mhz(void);

int	 loop;

int	 mcu_reset_flag;		// MCU复位标志(也即复位原因)


#define USING_FUNC_ECHO

/********************************************************************************************************************
*
*
*
*
*********************************************************************************************************************/
int main(void)
{	
#if 0
	int					len = 0;	
	
	char				echo[128];
#endif
	unsigned char		send[12]="hello";
	mcu_reset_flag = 0x00;
		
	// 设置MCU中断向量表的偏移量(相对地址格式)
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, BLOCK_APPLICATION_START1-BASE_OF_ROM);

	is_sys_idle = FALSE;

	// 初始化MCU
	mcu_init();
			
	// 闪烁绿灯指示MCU初始化完成
	led_flash1(GPIO_LED_G, 1, 3);

	// 应用逻辑初始化
	log_init();	

	// 检测com命令，若有则添加命令请求。
	check_buf_com();

	// 检查全局命令请求队列并逐一处理命令。
	check_que_cmd(CHECK_MODE_CONTINUOUS);

	// GSM、GPS、G-Sensor等外设初始化
	dev_init();

	// 闪烁红灯指示外设初始化完成
	led_flash1(GPIO_LED_R, 1, 3);	

	// 使能系统复位看门狗
	WATCHDOG_ENABLE(mcu_clock_mode_8m);

	// 使能用户定时器
	USERTIMER_ENABLE(mcu_clock_mode_8m);

	is_sys_idle = TRUE;

	// 调用短信发送测试例程
	// example_send_sms();

#if 0
	len += sprintf(echo+len, "IMEI: %s; ", gsm_imei);

	if(strlen(gsm_imei) > 0)      
	{	
		len += sprintf(echo+len, "sw ver: %s; hw ver: %s; ", swversion, hwversion);

		send_command_virtual(CMD_SRC_COM, "cxsbgd#", "", "", (char*)echo+len);

#if 0
		gsm_send_sms(TEST_PHONE_NUMBER, echo);
#endif
	}
#endif

	example_ctrl_led();

	// 检查电源状态。
	check_power_status();

	// 调用gps定位测试例程
	// example_gps_pos();

	// example_con_server(); 

	// 初始化时自动打开周期定位模式(可改为从串口发命令打开)
	example_periodic_pos();

	printf("to enter main loop...\r\n");	

	// 进入系统主循环
	while(1)
	{				
		WATCHDOG_RELOAD();
		
#ifdef 	USING_FUNC_POWERSAVING
		// 无gsm ring中断事件要处理时才进入mcu stop模式
		if(is_gsmring_pending == FALSE)
		{		
			loop++;			
			
			if(loop > 100)
			{	
				// 系统空闲且mcu处于stop模式时，蓝等替代绿灯常亮
				led_switch(GPIO_LED_G, OFF);		// 强行熄灭绿灯，因为在事务处理完成后会熄灭红灯、点亮绿灯。

				// 进入stop模式时，蓝灯常亮
				led_switch(GPIO_LED_B, ON);		
					
				// MCU进入STOP模式
				mcu_stop_enter();

				// 退出stop模式时，熄灭蓝灯(此后会有很短的一段时间内三种颜色的led灯都是熄灭的，这段时间等于1-100计数的循环时间)
				led_switch(GPIO_LED_B, OFF);

				loop = 0;
			}			
		}
#endif		

		/*******************************************************************************************
		在主循环中检查各种队列(queue)或事件(event)，有则处理之、无则跳过，有队列或事件需要处理的时候，
		在处理前打开红灯以指示当前系统忙、在处理后关闭红灯以指示当前系统闲。
		********************************************************************************************/

		// 检测sms命令，若有则添加命令请求。
//		check_que_sms(CHECK_MODE_CONTINUOUS);

		// 检测com命令，若有则添加命令请求。
//		check_buf_com();

		// 检查全局命令请求队列并逐一处理命令。
//		check_que_cmd(CHECK_MODE_CONTINUOUS);

		// 检查系统事件标志，若有则处理之。
//		sys_check_event();

		// 检查系统信号量(仅定位请求)
//		sys_check_sem();
		gsm_send_data(GPRS_ID0, send, sizeof(send));
	}
}

// MCU内核初始化。
void mcu_init(void)
{		
	// MCU初始化	
#ifndef STARTUP_FROM_BOOTLOADER
	SystemInit();	
#endif		
	
	// RCC_MCOConfig(RCC_MCO_NoClock);//hsi

	// 检查MCU复位类型(实验发现，应在MCU硬件复位后尽早读取复位原因寄存器的值，否则此寄存器值可能会被清除)
	if(RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)			// 独立看门狗复位
	{
		RCC_ClearFlag();

		mcu_reset_flag = RCC_FLAG_IWDGRST;
	}
	else if(RCC_GetFlagStatus(RCC_FLAG_WWDGRST) != RESET)		// 窗口看门狗复位
	{
		RCC_ClearFlag();

		mcu_reset_flag = RCC_FLAG_WWDGRST;
	}
	else if(RCC_GetFlagStatus(RCC_FLAG_LPWRRST) != RESET)		// 低电复位
	{
		RCC_ClearFlag();

		mcu_reset_flag = RCC_FLAG_LPWRRST;

		// MCU低电时直接将MCU深度休眠
		mcu_standby();		
	}
	else if(RCC_GetFlagStatus(RCC_FLAG_SFTRST) != RESET)		// 软件复位(程序执行异常或程序更新后会产生软件复位)
	{
		RCC_ClearFlag();

		mcu_reset_flag = RCC_FLAG_SFTRST;
	}
	else if(RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)		// POR/PDR复位(断开电源后的复位类型为POR复位)
	{
		RCC_ClearFlag();

		mcu_reset_flag = RCC_FLAG_PORRST;
	}
	// 所有的复位，都会使PIN RSTF 置位，即除了NRST pin复位源外，其他引起的复位都能查到两个复位标志位（其中一个是PIN RSTF ），
	// 所以在做复位标志判断时要将NRST pin标志位放到最后判断。还有需要注意的是复位标志位需要软件清除。
	else if(RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)		// 引脚复位
	{
		RCC_ClearFlag();

		mcu_reset_flag = RCC_FLAG_PINRST;
	}
	else
	{
		mcu_reset_flag = 0x00;									// 按键开机复位类型不是上面罗列的标志，原因不详
	}	

	// 开启afio时钟 	
	RCC->APB2ENR |= 0x00000001; 			

	//关闭JTAG（GSM模块的PowerOn引脚和JATG相关引脚复用了），保留SWD。
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);				
	
	// 全局中断优先级设置
	NVIC_Configure();		

	// systick初始化
	systick_init_hsi8mhz();	

	// 串口初始化
	USART_Configure();		

	// 外部中断初始化
	EXTI_Configure();

	FLASH_Unlock();	
	
	// GPIO(控制口)初始化
	GPIO_Configure();

	// ADC初始化
	ADC_Configure();

	// (模拟)I2C初始化
	I2C_Configure();	 		

	// FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable); 			// 开启FLASH预读缓冲功能，加速FLASH的读取。所有程序中必须的用法.位置：RCC初始化子函数里面，时钟起振之后
	// FLASH_SetLatency(FLASH_Latency_2);                    			// Flash操作的延时

	// 仅在个人应用模式下才需要按键开机，其他应用模式下通电自动开机(以免人工介入)
#ifdef APPLICATION_IN_PERSON 	
	#ifdef USING_IO_KEY	
	
	// 检查MCU复位类型，如果是看门狗复位或软件复位，则跳过按键检查直接运行程序。
	if(mcu_reset_flag != RCC_FLAG_WWDGRST && mcu_reset_flag != RCC_FLAG_IWDGRST && mcu_reset_flag != RCC_FLAG_SFTRST)
	{
		if(key_scan()!=4)
		{		
			// MCU进入standby模式后，前面打开的systick中断不会产生，因为MCU始终都已经关闭
			mcu_standby();												// 检测是不是按键按下3秒（正常开机），如果不是，直接进入待机
		}
	}	

	#endif	
#endif	

	printf("\r\n\r\n");
	printf("============================\r\n");
	printf("Welcom to GPS Tracker -- STD\r\n");
	printf("============================\r\n");
	// printf("\r\nreset_flag = 0x%02x\r\n", mcu_reset_flag);	
	// dump_mcu_reset_flag(mcu_reset_flag);

	// 检查系统是否首次运行，若是，则将默认运行参数初始化到Flash中
	if(check_first_running() == OK)
	{	
		printf("Device is in first running.\r\n");		
		
		bkp_init();

		printf("Evironment variables initialized into Flash.\r\n");
	}
	else
	{	
		if(check_normal_startup() == OK)
		{
			printf("Device is in non-first running.\r\n");
			
			printf("Device is shutdown manually last time.\r\n");
		}
		else
		{
			printf("Device is in non-first running.\r\n");
			
			printf("Device is not shutdown manually last time.\r\n");
		}
	}

	// 不管系统是否首次启动，每次启动时都会从flash中恢复环境变量参数	
	bkp_recover();
	
	printf("Environment variables recoveried from Flash.\r\n");	
}

// 外设初始化。
void dev_init(void)
{	
#if 1
	int		ret;
#endif
	
	// 电源管理功能初始化
	power_init();
	printf("Power initialized.\r\n");

#ifdef USING_DEV_GSR
	// 加速度传感器初始化
	MMA845x_Init();		

	printf("G-Sensor initialized.\r\n"); 

	#ifdef TEST_GSR_ACQUIRE_DATA
	MMA845x_Acquiye_Data();
	#endif
#endif

#ifdef USING_DEV_GPS
	// GPS初始化
	gps_init();
#endif	

#if 1
	// GSM初始化(需考虑初始化失败情况下的故障恢复)
	ret = gsm_init();

	if(ret != OK)
	{
		// SIM未检测到错误
		if(ret == ER_GSM_INIT_SIM)									
		{
			led_flash1(GPIO_LED_R, 4, 5);				// 红灯持续快闪报警
		}
		// 注册网络失败
		else if(ret == ER_GSM_INIT_REGISTRATION)								
		{
			led_flash1(GPIO_LED_G, 4, 5);				// 红灯持续快闪报警
		}
		// 其他初始化时错误
		else
		{		
			led_flash1(GPIO_LED_B, 4, 5);				// 红灯慢闪报警
		}

		// 只要初始化不成功，都尝试一次恢复gsm
		if(gsm_recover() == OK)
		{
			led_flash1(GPIO_LED_G, 1, 10);				// 绿灯快闪指示
		}
		else
		{
			led_flash1(GPIO_LED_R, 1, 10);				// 红灯快闪报警
		}	
	}

	// gsm初始化成功后默认处于休眠状态
	gsm_sleep();		
#endif
}

// 应用逻辑初始化。
void log_init(void)
{	
	pos_init();

	que_init(); 

	time_init();

	motion_init();
}

/*
EXTI_PA0_SYS_KEY
EXTI_PB5_GSM_RING
EXTI_PA6_CHG_STS
EXTI_PA8_GSR_INT2
EXTI_PA11_GSR_INT1
EXTI_PB12_EXTPWR_DET
*/

// 设置各内部/外部中断的优先级(包括抢占优先级和响应优先级)和开关。
void NVIC_Configure(void)
{
 	NVIC_InitTypeDef 	NVIC_InitStructure;

	// 选择优先级分组第二组(STM32 MCU中4位优先级位的高2位用于指定抢占优先级、低2位用于指定响应优先级)
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	

   	// 设置rtcalarm中断的抢占优先级为0(不设置其响应优先级)
   	/*
	NVIC_SetPriority(SysTick_IRQn, n);
	n=0x00~0x03 设置Systick为抢占优先级0
	n=0x04~0x07 设置Systick为抢占优先级1
	n=0x08~0x0B 设置Systick为抢占优先级2
	n=0x0C~0x0F 设置Systick为抢占优先级3 
	*/
	// NVIC_SetPriority(SysTick_IRQn, 0);	

	// TIM3_IRQn(软件看门狗定时器，应设置最高中断优先级)
   	NVIC_InitStructure.NVIC_IRQChannel 	= TIM3_IRQn;       		
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 		= 1; 	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 0;	
   	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		   			
   	NVIC_Init(&NVIC_InitStructure); 			
	
	// USART3 <--> GSM
   	NVIC_InitStructure.NVIC_IRQChannel 	= USART3_IRQn;       		
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 		= 1; 	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 1;	
   	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		   			
   	NVIC_Init(&NVIC_InitStructure); 	

	// PB3 --> GSM_RING
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;			
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 		= 1; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// USART1 <--> PC
   	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;       		
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 		= 1; 	
   	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 3;	
   	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		   			
   	NVIC_Init(&NVIC_InitStructure);

	// USART2 <--> GPS
   	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;       		
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 		= 2; 	
   	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 0;	
   	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		   			
   	NVIC_Init(&NVIC_InitStructure); 	
	
#ifdef USING_PWR_EXT
	/*
	PB12 --> EXTPWR_DET
	*/
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;		
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 		= 2; 	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
#else
	/*
	PB12 --> EXTPWR_DET
	*/
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;		
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 		= 2; 	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);	
#endif

#ifdef USING_PWR_ACC
	/*
	PB9/PB8 --> ACCPWR_DET
	*/
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 		= 2; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
#else
	/*
	PB9/PB8 --> ACCPWR_DET
	*/
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 		= 2; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);	
#endif

#if 0
	// 设置RTC(Line 17)中断的抢占优先级为1、响应优先级为1
	NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;				
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 		= 2;	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 3;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					
	NVIC_Init(&NVIC_InitStructure);
#else
	// TIM4_IRQn(软件看门狗定时器，应设置最高中断优先级)
   	NVIC_InitStructure.NVIC_IRQChannel 	= TIM4_IRQn;       		
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 		= 2; 	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 3;	
   	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		   			
   	NVIC_Init(&NVIC_InitStructure); 
#endif

	// PA0 --> SYS_KEY
	// 按键中断处理程序中检测按键动作的耗时更长且实时性要求不高，因此应设置更低的中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 		= 3; 	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 0;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
}

/*
独立看门狗(IWDG)由MCU内部专用的40kHz的低速时钟驱动，即使主时钟发生故障它也仍然有效。
IWDG主要特点: 
>> 自由运行的递减计数器
>> 时钟由独立的RC振荡器提供(可在停止和待机模式下工作)
>> 看门狗被激活后，则在计数器计数至0x000时产生复位信号将MCU复位
*/
/*
void IWDG_Configure(void)
{
	// 写入0x5555,以解锁独立看门狗寄存器写入
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	// 狗狗时钟分频,40K/256=156HZ(6.4ms)
	IWDG_SetPrescaler(IWDG_Prescaler_256);

	// 喂狗时间 5s/6.4ms=781 .注意不能大于0xfff，即最大喂狗延迟=26s
	// IWDG_SetReload(0xFFF);
	IWDG_SetReload(0x30D);

	// 启动独立看门狗后首次喂狗以初始化
	WATCHDOG_RELOAD();
}
*/

// 初始化系统时钟(使用内部8MHz时钟，不开启PLL)。
void rcc_hsi_64mhz(void)
{
  // ErrorStatus HSEStartUpStatus;
  
  //将外设 RCC寄存器重设为缺省值
  RCC_DeInit();

	// 使能内部RC振荡器(8Mhz)
  RCC_HSICmd(ENABLE);
  
  while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);
  	
  if(1)
  {
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    
    FLASH_SetLatency(FLASH_Latency_2);
   
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
   
    RCC_PCLK2Config(RCC_HCLK_Div1);
  
    RCC_PCLK1Config(RCC_HCLK_Div2);
    
    //设置 PLL 时钟源及倍频系数
    RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);  
                
    //使能或者失能 PLL,这个参数可以取：ENABLE或者DISABLE
    RCC_PLLCmd(ENABLE);//如果PLL被用于系统时钟,那么它不能被失能
		
    //等待指定的 RCC 标志位设置成功 等待PLL初始化成功
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

    //设置系统时钟（SYSCLK） 设置PLL为系统时钟源
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
  
    //等待PLL成功用作于系统时钟的时钟源
    //  0x00：HSI 作为系统时钟 
    //  0x04：HSE作为系统时钟 
    //  0x08：PLL作为系统时钟  
    while(RCC_GetSYSCLKSource() != 0x08);
	}
	
 	// 重新初始化软件看门狗
	WATCHDOG_DISABLE();
	WATCHDOG_ENABLE(mcu_clock_mode_64m);

	// 重新初始化systick
	systick_init_hsi64mhz();

	// 系统主频提升后，串口需要重新初始化，否则波特率不对
	USART_Configure();
}

void rcc_hsi_8mhz(void)
{
	SystemInit();	

	// 重新初始化软件看门狗
	WATCHDOG_DISABLE();
	WATCHDOG_ENABLE(mcu_clock_mode_8m);

	// 重新初始化systick
	systick_init_hsi8mhz();

	// 改变系统主时钟后应重新初始化串口硬件
	USART_Configure();
}

// 进入系统事务处理时调用。
void transaction_enter(void)
{		
	// 有事务要处理时，熄灭绿灯、点亮红灯以指示系统忙。
	// led_switch(GPIO_LED_G, OFF);
	// led_switch(GPIO_LED_R, ON);	

	is_sys_idle = FALSE;
}

// 退出系统事务处理时调用。
void transaction_quit(void)
{
	// 事务处理完成后，熄灭红灯、点亮绿灯以指示系统空闲。
	// led_switch(GPIO_LED_R, OFF);
	// led_switch(GPIO_LED_G, ON);

	is_sys_idle = TRUE;
}

// 检查电池电压偏低等系统事件并做相应处理。
void sys_check_event(void)
{			
	// printf("enter <sys_check_event>.\r\n");
	
	// 检查是否打开GPS串口转发
	if(sw_gps_forward == ON)
	{
		if(is_gps_forwarding == FALSE)
		{
			transaction_enter();

			RxCnt2_rd = 0;
			RxCnt2_wr = 0;

			// 执行GPS串口转发前应重新打开USART2 Rx中断
			USART_Configure();

			// GPS模块强行上电
			gps_power_up();

			delay_100ms(10);

			// 执行GPS串口转发
			usart2_redir_usart1();	

			transaction_quit();
		}
	}	
}	 

// 检查系统信号量。
void sys_check_sem(void)
{
	int		to;

	char		reply[128];
	unsigned char		send[128]="hello";
	// 周期定位请求计数器非0则说明有定位请求需要处理。
	if(sem_periodic_pos > 0)
	{
		printf("sem_periodic_pos = %d\r\n", sem_periodic_pos);
		
		// 根据定位周期设置单次定位超时。
		if(cycle_periodic_pos > 60)
		{
			to =  60;
		}
		else
		{
			to = cycle_periodic_pos;
		}

		// gps定位，成功则提取定位结果。
		if(pos_via_gps(to) == OK)
		{
			sprintf(reply, "GPS pos info: lat=%3.6f, lon=%3.6f, alt=%3.6f, spd=%4.2f.\r\n", de_gpspos.lat, de_gpspos.lon, de_gpspos.alt, de_gpspos.spd);

			printf("%s", reply);

			// 提取当次gps定位结果后，从ram中删除之。
			clr_gpspos();

#if 1
			gsm_wakeup();

			// 通过短信回复gps定位结果。
//			gsm_send_sms(TEST_PHONE_NUMBER, reply);
			// 这里可改为通过gprs发送定位结果。。。
			gsm_send_data(GPRS_ID0, (unsigned char*)reply, sizeof(reply));

			gsm_sleep();
#endif		
		}		
		else
		{
			sprintf(reply, "failed to get gps fixed within %d seconds!", to);

			printf("%s", reply);

#if 1
			gsm_wakeup();

			// 通过短信回复gps定位失败。
//			gsm_send_sms(TEST_PHONE_NUMBER, reply);
			// 这里可改为通过gprs发送定位结果。。。
			gsm_send_data(GPRS_ID0, send, sizeof(send));
			gsm_sleep();
#endif			
		}

		// 周期定位请求计数器递减
		sem_periodic_pos--;
	}
}

