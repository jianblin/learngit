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
2013-05-04: RTC�Ĵ�������������������ܻ����RTC������ʱ�ӱ��(�Ʋ���ܻ���32K��RTCʱ�ӿ��ܱ�8M���ڲ�RC�����)���Ӷ�����һϵ�е����⣬����ʱ����
*/


// ��ʼ��RTCʱ�ӡ�
void rtc_init(void)
{
	systick = 0;

	// ��鱸�ݼĴ����е�RTCУ׼��־�Ƿ����ã���δ�����ã���У׼RTC
	if(BKP_ReadBackupRegister(BKP_DR1) != 0xA5A5)
  	{
    	/* Backup data register value is not correct or not yet programmed (when
       	   the first time the program is executed) */
   
    	/* RTC Configuration */
    	RTC_Configure();
 
    	/* Adjust time by values entred by the user on the hyperterminal */
    	// Time_Adjust();

		// У׼RTC���ڱ��ݼĴ�����д��У׼��־
    	BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);    
  	}
  	else	// ��RTC�Ѿ���У׼(���ݼĴ�����MCU�ػ������ϵ�����¿ɱ�������)����������У׼
  	{
		//����PWR��BKP��ʱ�ӣ�from APB1��
    	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
		
    	//�����������Ϊ��RTC���ں��򣬲�������ؼĴ���ʱ��Ҫ�Ƚ���
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

    	/*����RTC�����ж�*/
	 	RTC_ITConfig(RTC_IT_ALR, ENABLE); 
    	
    	/* Wait until last write operation on RTC registers has finished */
    	RTC_WaitForLastTask();
  	}
	
  	/* Clear reset flags */
  	RCC_ClearFlag();

 	/*�ȴ����һ��дָ�����*/
 	RTC_WaitForLastTask();

  	/* Display time in infinte loop */
  	//Time_Show();

	RTC_SetAlarm(systick+SYSTICK_PERIOD); 
}

// ����RTCӲ����
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

	// �ϵ����ҪУ׼RTCʱ�ӣ�Ҳ��׼ȷ�����RTC������ʱ����
	RTC_Calibrate();

#ifdef RTCClockOutput_Enable  

  	/* Disable the Tamper Pin */
  	BKP_TamperPinCmd(DISABLE); /* To output RTCCLK/64 on Tamper pin, the tamper functionality must be disabled */
                               
  	/* Enable RTC Clock Output on Tamper Pin */
  	BKP_RTCCalibrationClockOutputCmd(ENABLE);
  
#endif 
  
  	/*����RTC�����ж�*/
	RTC_ITConfig(RTC_IT_ALR, ENABLE); 

  	/* Wait until last write operation on RTC registers has finished */
  	RTC_WaitForLastTask();
}

// ͨ��ֱ�Ӽ���У׼RTCʱ��(ʹ��0.01ms���ڵ�systick����������RTCԤ��Ƶϵ��)��
void RTC_Calibrate(void)
{
	unsigned int systick_sta;
	unsigned int rtctick_lmt;
	unsigned int prescaler 	= 0;	

	// ��ʹ��systick����RTC tick�����ڳ���֮ǰ����ʼ��systick
	NVIC_SetPriority(SysTick_IRQn, 0);	
	
	SysTick_Init();
	
	// ��ʼrtcalarm������ʹ��rtcalarm�ж�
	SysTick_Enable();

	// ������Ĭ�ϵ�RTCԤ��Ƶϵ��(rtc tike = ~1ms)
	RTC_SetPrescaler(DEF_RTC_PRESCALER);

	/* Enable RTC Clock */
	RCC_RTCCLKCmd(ENABLE);

	/* Wait for RTC registers synchronization */
	RTC_WaitForSynchro();

	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();

	// ����RTC�����ڳ��ȳ���~1��
	rtctick_lmt = systick+1000/SYSTICK_PERIOD;

	// ��¼����ǰ��systickֵ
	systick_sta = systick;

	// ����1000�����ڵ�rtc tick(~1s)
	while(systick < rtctick_lmt);

	prescaler = (DEF_RTC_PRESCALER * 1000)/((systick-systick_sta)/100);

	// printf("prescaler = %d.\r\n", prescaler);	

	// ��������RTCԤ��Ƶϵ��
	RTC_SetPrescaler(prescaler);

	/* Enable RTC Clock */
	RCC_RTCCLKCmd(ENABLE);

	/* Wait for RTC registers synchronization */
	RTC_WaitForSynchro();

	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();

	// У׼��RTCʱ�Ӻ�ֹͣrtcalarm��������ֹrtcalarm�ж�
	SysTick_Disable();

	// printf("RTC Clock calibrated.\r\n");
}

