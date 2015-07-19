 #include "stm32f10x.h"
#include "mcu_systick.h"
#include "mcu_gpio.h"
#include "mcu_systick.h"
#include "mcu_rtc.h"
#include "stdio.h"
#include "common.h"

unsigned int rtcalarm;

void RTC_Configure(void);

/*
2013-05-04: RTC寄存器如果操作不当，可能会造成RTC计数器时钟变快(推测可能还是32K的RTC时钟可能被8M的内部RC替代了)，从而导致一系列的问题，如延时出错。
*/


// 初始化RTC时钟。
void rtc_init(void)
{
	systick = 0;

	// 检查备份寄存器中的RTC校准标志是否被设置，若未被设置，则校准RTC
	if(BKP_ReadBackupRegister(BKP_DR1) != 0xA5A5)
  	{
    	/* Backup data register value is not correct or not yet programmed (when
       	   the first time the program is executed) */
   
    	/* RTC Configuration */
    	RTC_Configure();
 
    	/* Adjust time by values entred by the user on the hyperterminal */
    	// Time_Adjust();

		// 校准RTC后在备份寄存器中写入校准标志
    	BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);    
  	}
  	else	// 若RTC已经被校准(备份寄存器在MCU关机但不断电情况下可保持内容)，则无需再校准
  	{
		//启用PWR和BKP的时钟（from APB1）
    	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
		
    	//后备域解锁，以为内RTC处于后备域，操作其相关寄存器时需要先解锁
    	PWR_BackupAccessCmd(ENABLE);
		
#ifdef RTCClockSource_LSI
  		
  		/* Enable LSI */ 
  		RCC_LSICmd(ENABLE);
		
  		/* Wait till LSI is ready */
  		while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
  		{
  		
  		}

  		/* Select LSI as RTC Clock Source */
  		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
		
#endif
		
		RCC_RTCCLKCmd(ENABLE);		
		
		//==========================================================
    	/* Check if the Power On Reset flag is set */
		#if 0
    	if(RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
    	{
      		//printf("\r\n\n Power On Reset occurred....");
    	}
    	/* Check if the Pin Reset flag is set */
    	else if(RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
    	{
      		//printf("\r\n\n External Reset occurred....");
    	}
		#endif

    	//printf("\r\n No need to configure RTC....");    
    	/* Wait for RTC registers synchronization */
    	RTC_WaitForSynchro();

    	/*允许RTC报警中断*/
	 	RTC_ITConfig(RTC_IT_ALR, ENABLE); 
    	
    	/* Wait until last write operation on RTC registers has finished */
    	RTC_WaitForLastTask();
  	}
	
  	/* Clear reset flags */
  	RCC_ClearFlag();

 	/*等待最后一条写指令完成*/
 	RTC_WaitForLastTask();

  	/* Display time in infinte loop */
  	//Time_Show();

	RTC_SetAlarm(systick+SYSTICK_PERIOD); 
}

// 配置RTC硬件。
void RTC_Configure(void)
{
  	/* Enable PWR and BKP clocks */
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
  
  	/* Allow access to BKP Domain */
  	PWR_BackupAccessCmd(ENABLE);
	
	RCC_RTCCLKCmd(ENABLE);

  	/* Reset Backup Domain */
  	BKP_DeInit();
  
  	/* Wait until last write operation on RTC registers has finished */
  	RTC_WaitForLastTask();	

#ifdef RTCClockSource_LSI

  	/* Enable LSI */ 
  	RCC_LSICmd(ENABLE);

  	/* Wait till LSI is ready */
  	while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
	
  	/* Select LSI as RTC Clock Source */
  	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);  

#elif defined	RTCClockSource_LSE  

  	/* Enable LSE */
  	RCC_LSEConfig(RCC_LSE_ON);

	 /* Wait till LSE is ready */
  	while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);

  	/* Select LSE as RTC Clock Source */
  	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);  
	
#endif

	// 上电后需要校准RTC时钟，也即准确计算出RTC的周期时长。
	RTC_Calibrate();

#ifdef RTCClockOutput_Enable  

  	/* Disable the Tamper Pin */
  	BKP_TamperPinCmd(DISABLE); /* To output RTCCLK/64 on Tamper pin, the tamper functionality must be disabled */
                               
  	/* Enable RTC Clock Output on Tamper Pin */
  	BKP_RTCCalibrationClockOutputCmd(ENABLE);
  
#endif 
  
  	/*允许RTC报警中断*/
	RTC_ITConfig(RTC_IT_ALR, ENABLE); 

  	/* Wait until last write operation on RTC registers has finished */
  	RTC_WaitForLastTask();
}

// 通过直接计算校准RTC时钟(使用0.01ms周期的systick测量再修正RTC预分频系数)。
void RTC_Calibrate(void)
{
	unsigned int systick_sta;
	unsigned int rtctick_lmt;
	unsigned int prescaler 	= 0;	

	// 在使用systick测量RTC tick的周期长度之前，初始化systick
	NVIC_SetPriority(SysTick_IRQn, 0);	
	
	SysTick_Init();
	
	// 开始rtcalarm计数并使能rtcalarm中断
	SysTick_Enable();

	// 先设置默认的RTC预分频系数(rtc tike = ~1ms)
	RTC_SetPrescaler(DEF_RTC_PRESCALER);

	/* Enable RTC Clock */
	RCC_RTCCLKCmd(ENABLE);

	/* Wait for RTC registers synchronization */
	RTC_WaitForSynchro();

	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();

	// 测量RTC单周期长度持续~1秒
	rtctick_lmt = systick+1000/SYSTICK_PERIOD;

	// 记录测量前的systick值
	systick_sta = systick;

	// 测量1000个周期的rtc tick(~1s)
	while(systick < rtctick_lmt);

	prescaler = (DEF_RTC_PRESCALER * 1000)/((systick-systick_sta)/100);

	// printf("prescaler = %d.\r\n", prescaler);	

	// 重新设置RTC预分频系数
	RTC_SetPrescaler(prescaler);

	/* Enable RTC Clock */
	RCC_RTCCLKCmd(ENABLE);

	/* Wait for RTC registers synchronization */
	RTC_WaitForSynchro();

	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();

	// 校准完RTC时钟后，停止rtcalarm计数并禁止rtcalarm中断
	SysTick_Disable();

	// printf("RTC Clock calibrated.\r\n");
}

