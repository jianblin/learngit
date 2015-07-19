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

int	 mcu_reset_flag;		// MCU��λ��־(Ҳ����λԭ��)


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
		
	// ����MCU�ж��������ƫ����(��Ե�ַ��ʽ)
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, BLOCK_APPLICATION_START1-BASE_OF_ROM);

	is_sys_idle = FALSE;

	// ��ʼ��MCU
	mcu_init();
			
	// ��˸�̵�ָʾMCU��ʼ�����
	led_flash1(GPIO_LED_G, 1, 3);

	// Ӧ���߼���ʼ��
	log_init();	

	// ���com��������������������
	check_buf_com();

	// ���ȫ������������в���һ�������
	check_que_cmd(CHECK_MODE_CONTINUOUS);

	// GSM��GPS��G-Sensor�������ʼ��
	dev_init();

	// ��˸���ָʾ�����ʼ�����
	led_flash1(GPIO_LED_R, 1, 3);	

	// ʹ��ϵͳ��λ���Ź�
	WATCHDOG_ENABLE(mcu_clock_mode_8m);

	// ʹ���û���ʱ��
	USERTIMER_ENABLE(mcu_clock_mode_8m);

	is_sys_idle = TRUE;

	// ���ö��ŷ��Ͳ�������
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

	// ����Դ״̬��
	check_power_status();

	// ����gps��λ��������
	// example_gps_pos();

	// example_con_server(); 

	// ��ʼ��ʱ�Զ������ڶ�λģʽ(�ɸ�Ϊ�Ӵ��ڷ������)
	example_periodic_pos();

	printf("to enter main loop...\r\n");	

	// ����ϵͳ��ѭ��
	while(1)
	{				
		WATCHDOG_RELOAD();
		
#ifdef 	USING_FUNC_POWERSAVING
		// ��gsm ring�ж��¼�Ҫ����ʱ�Ž���mcu stopģʽ
		if(is_gsmring_pending == FALSE)
		{		
			loop++;			
			
			if(loop > 100)
			{	
				// ϵͳ������mcu����stopģʽʱ����������̵Ƴ���
				led_switch(GPIO_LED_G, OFF);		// ǿ��Ϩ���̵ƣ���Ϊ����������ɺ��Ϩ���ơ������̵ơ�

				// ����stopģʽʱ�����Ƴ���
				led_switch(GPIO_LED_B, ON);		
					
				// MCU����STOPģʽ
				mcu_stop_enter();

				// �˳�stopģʽʱ��Ϩ������(�˺���к̵ܶ�һ��ʱ����������ɫ��led�ƶ���Ϩ��ģ����ʱ�����1-100������ѭ��ʱ��)
				led_switch(GPIO_LED_B, OFF);

				loop = 0;
			}			
		}
#endif		

		/*******************************************************************************************
		����ѭ���м����ֶ���(queue)���¼�(event)��������֮�������������ж��л��¼���Ҫ�����ʱ��
		�ڴ���ǰ�򿪺����ָʾ��ǰϵͳæ���ڴ����رպ����ָʾ��ǰϵͳ�С�
		********************************************************************************************/

		// ���sms��������������������
//		check_que_sms(CHECK_MODE_CONTINUOUS);

		// ���com��������������������
//		check_buf_com();

		// ���ȫ������������в���һ�������
//		check_que_cmd(CHECK_MODE_CONTINUOUS);

		// ���ϵͳ�¼���־����������֮��
//		sys_check_event();

		// ���ϵͳ�ź���(����λ����)
//		sys_check_sem();
		gsm_send_data(GPRS_ID0, send, sizeof(send));
	}
}

// MCU�ں˳�ʼ����
void mcu_init(void)
{		
	// MCU��ʼ��	
#ifndef STARTUP_FROM_BOOTLOADER
	SystemInit();	
#endif		
	
	// RCC_MCOConfig(RCC_MCO_NoClock);//hsi

	// ���MCU��λ����(ʵ�鷢�֣�Ӧ��MCUӲ����λ�����ȡ��λԭ��Ĵ�����ֵ������˼Ĵ���ֵ���ܻᱻ���)
	if(RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)			// �������Ź���λ
	{
		RCC_ClearFlag();

		mcu_reset_flag = RCC_FLAG_IWDGRST;
	}
	else if(RCC_GetFlagStatus(RCC_FLAG_WWDGRST) != RESET)		// ���ڿ��Ź���λ
	{
		RCC_ClearFlag();

		mcu_reset_flag = RCC_FLAG_WWDGRST;
	}
	else if(RCC_GetFlagStatus(RCC_FLAG_LPWRRST) != RESET)		// �͵縴λ
	{
		RCC_ClearFlag();

		mcu_reset_flag = RCC_FLAG_LPWRRST;

		// MCU�͵�ʱֱ�ӽ�MCU�������
		mcu_standby();		
	}
	else if(RCC_GetFlagStatus(RCC_FLAG_SFTRST) != RESET)		// �����λ(����ִ���쳣�������º����������λ)
	{
		RCC_ClearFlag();

		mcu_reset_flag = RCC_FLAG_SFTRST;
	}
	else if(RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)		// POR/PDR��λ(�Ͽ���Դ��ĸ�λ����ΪPOR��λ)
	{
		RCC_ClearFlag();

		mcu_reset_flag = RCC_FLAG_PORRST;
	}
	// ���еĸ�λ������ʹPIN RSTF ��λ��������NRST pin��λԴ�⣬��������ĸ�λ���ܲ鵽������λ��־λ������һ����PIN RSTF ����
	// ����������λ��־�ж�ʱҪ��NRST pin��־λ�ŵ�����жϡ�������Ҫע����Ǹ�λ��־λ��Ҫ��������
	else if(RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)		// ���Ÿ�λ
	{
		RCC_ClearFlag();

		mcu_reset_flag = RCC_FLAG_PINRST;
	}
	else
	{
		mcu_reset_flag = 0x00;									// ����������λ���Ͳ����������еı�־��ԭ����
	}	

	// ����afioʱ�� 	
	RCC->APB2ENR |= 0x00000001; 			

	//�ر�JTAG��GSMģ���PowerOn���ź�JATG������Ÿ����ˣ�������SWD��
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);				
	
	// ȫ���ж����ȼ�����
	NVIC_Configure();		

	// systick��ʼ��
	systick_init_hsi8mhz();	

	// ���ڳ�ʼ��
	USART_Configure();		

	// �ⲿ�жϳ�ʼ��
	EXTI_Configure();

	FLASH_Unlock();	
	
	// GPIO(���ƿ�)��ʼ��
	GPIO_Configure();

	// ADC��ʼ��
	ADC_Configure();

	// (ģ��)I2C��ʼ��
	I2C_Configure();	 		

	// FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable); 			// ����FLASHԤ�����幦�ܣ�����FLASH�Ķ�ȡ�����г����б�����÷�.λ�ã�RCC��ʼ���Ӻ������棬ʱ������֮��
	// FLASH_SetLatency(FLASH_Latency_2);                    			// Flash��������ʱ

	// ���ڸ���Ӧ��ģʽ�²���Ҫ��������������Ӧ��ģʽ��ͨ���Զ�����(�����˹�����)
#ifdef APPLICATION_IN_PERSON 	
	#ifdef USING_IO_KEY	
	
	// ���MCU��λ���ͣ�����ǿ��Ź���λ�������λ���������������ֱ�����г���
	if(mcu_reset_flag != RCC_FLAG_WWDGRST && mcu_reset_flag != RCC_FLAG_IWDGRST && mcu_reset_flag != RCC_FLAG_SFTRST)
	{
		if(key_scan()!=4)
		{		
			// MCU����standbyģʽ��ǰ��򿪵�systick�жϲ����������ΪMCUʼ�ն��Ѿ��ر�
			mcu_standby();												// ����ǲ��ǰ�������3�루������������������ǣ�ֱ�ӽ������
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

	// ���ϵͳ�Ƿ��״����У����ǣ���Ĭ�����в�����ʼ����Flash��
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

	// ����ϵͳ�Ƿ��״�������ÿ������ʱ�����flash�лָ�������������	
	bkp_recover();
	
	printf("Environment variables recoveried from Flash.\r\n");	
}

// �����ʼ����
void dev_init(void)
{	
#if 1
	int		ret;
#endif
	
	// ��Դ�����ܳ�ʼ��
	power_init();
	printf("Power initialized.\r\n");

#ifdef USING_DEV_GSR
	// ���ٶȴ�������ʼ��
	MMA845x_Init();		

	printf("G-Sensor initialized.\r\n"); 

	#ifdef TEST_GSR_ACQUIRE_DATA
	MMA845x_Acquiye_Data();
	#endif
#endif

#ifdef USING_DEV_GPS
	// GPS��ʼ��
	gps_init();
#endif	

#if 1
	// GSM��ʼ��(�迼�ǳ�ʼ��ʧ������µĹ��ϻָ�)
	ret = gsm_init();

	if(ret != OK)
	{
		// SIMδ��⵽����
		if(ret == ER_GSM_INIT_SIM)									
		{
			led_flash1(GPIO_LED_R, 4, 5);				// ��Ƴ�����������
		}
		// ע������ʧ��
		else if(ret == ER_GSM_INIT_REGISTRATION)								
		{
			led_flash1(GPIO_LED_G, 4, 5);				// ��Ƴ�����������
		}
		// ������ʼ��ʱ����
		else
		{		
			led_flash1(GPIO_LED_B, 4, 5);				// �����������
		}

		// ֻҪ��ʼ�����ɹ���������һ�λָ�gsm
		if(gsm_recover() == OK)
		{
			led_flash1(GPIO_LED_G, 1, 10);				// �̵ƿ���ָʾ
		}
		else
		{
			led_flash1(GPIO_LED_R, 1, 10);				// ��ƿ�������
		}	
	}

	// gsm��ʼ���ɹ���Ĭ�ϴ�������״̬
	gsm_sleep();		
#endif
}

// Ӧ���߼���ʼ����
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

// ���ø��ڲ�/�ⲿ�жϵ����ȼ�(������ռ���ȼ�����Ӧ���ȼ�)�Ϳ��ء�
void NVIC_Configure(void)
{
 	NVIC_InitTypeDef 	NVIC_InitStructure;

	// ѡ�����ȼ�����ڶ���(STM32 MCU��4λ���ȼ�λ�ĸ�2λ����ָ����ռ���ȼ�����2λ����ָ����Ӧ���ȼ�)
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	

   	// ����rtcalarm�жϵ���ռ���ȼ�Ϊ0(����������Ӧ���ȼ�)
   	/*
	NVIC_SetPriority(SysTick_IRQn, n);
	n=0x00~0x03 ����SystickΪ��ռ���ȼ�0
	n=0x04~0x07 ����SystickΪ��ռ���ȼ�1
	n=0x08~0x0B ����SystickΪ��ռ���ȼ�2
	n=0x0C~0x0F ����SystickΪ��ռ���ȼ�3 
	*/
	// NVIC_SetPriority(SysTick_IRQn, 0);	

	// TIM3_IRQn(������Ź���ʱ����Ӧ��������ж����ȼ�)
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
	// ����RTC(Line 17)�жϵ���ռ���ȼ�Ϊ1����Ӧ���ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;				
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 		= 2;	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 3;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					
	NVIC_Init(&NVIC_InitStructure);
#else
	// TIM4_IRQn(������Ź���ʱ����Ӧ��������ж����ȼ�)
   	NVIC_InitStructure.NVIC_IRQChannel 	= TIM4_IRQn;       		
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 		= 2; 	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 3;	
   	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		   			
   	NVIC_Init(&NVIC_InitStructure); 
#endif

	// PA0 --> SYS_KEY
	// �����жϴ�������м�ⰴ�������ĺ�ʱ������ʵʱ��Ҫ�󲻸ߣ����Ӧ���ø��͵��ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 		= 3; 	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 0;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
}

/*
�������Ź�(IWDG)��MCU�ڲ�ר�õ�40kHz�ĵ���ʱ����������ʹ��ʱ�ӷ���������Ҳ��Ȼ��Ч��
IWDG��Ҫ�ص�: 
>> �������еĵݼ�������
>> ʱ���ɶ�����RC�����ṩ(����ֹͣ�ʹ���ģʽ�¹���)
>> ���Ź�����������ڼ�����������0x000ʱ������λ�źŽ�MCU��λ
*/
/*
void IWDG_Configure(void)
{
	// д��0x5555,�Խ����������Ź��Ĵ���д��
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	// ����ʱ�ӷ�Ƶ,40K/256=156HZ(6.4ms)
	IWDG_SetPrescaler(IWDG_Prescaler_256);

	// ι��ʱ�� 5s/6.4ms=781 .ע�ⲻ�ܴ���0xfff�������ι���ӳ�=26s
	// IWDG_SetReload(0xFFF);
	IWDG_SetReload(0x30D);

	// �����������Ź����״�ι���Գ�ʼ��
	WATCHDOG_RELOAD();
}
*/

// ��ʼ��ϵͳʱ��(ʹ���ڲ�8MHzʱ�ӣ�������PLL)��
void rcc_hsi_64mhz(void)
{
  // ErrorStatus HSEStartUpStatus;
  
  //������ RCC�Ĵ�������Ϊȱʡֵ
  RCC_DeInit();

	// ʹ���ڲ�RC����(8Mhz)
  RCC_HSICmd(ENABLE);
  
  while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);
  	
  if(1)
  {
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    
    FLASH_SetLatency(FLASH_Latency_2);
   
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
   
    RCC_PCLK2Config(RCC_HCLK_Div1);
  
    RCC_PCLK1Config(RCC_HCLK_Div2);
    
    //���� PLL ʱ��Դ����Ƶϵ��
    RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);  
                
    //ʹ�ܻ���ʧ�� PLL,�����������ȡ��ENABLE����DISABLE
    RCC_PLLCmd(ENABLE);//���PLL������ϵͳʱ��,��ô�����ܱ�ʧ��
		
    //�ȴ�ָ���� RCC ��־λ���óɹ� �ȴ�PLL��ʼ���ɹ�
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

    //����ϵͳʱ�ӣ�SYSCLK�� ����PLLΪϵͳʱ��Դ
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
  
    //�ȴ�PLL�ɹ�������ϵͳʱ�ӵ�ʱ��Դ
    //  0x00��HSI ��Ϊϵͳʱ�� 
    //  0x04��HSE��Ϊϵͳʱ�� 
    //  0x08��PLL��Ϊϵͳʱ��  
    while(RCC_GetSYSCLKSource() != 0x08);
	}
	
 	// ���³�ʼ��������Ź�
	WATCHDOG_DISABLE();
	WATCHDOG_ENABLE(mcu_clock_mode_64m);

	// ���³�ʼ��systick
	systick_init_hsi64mhz();

	// ϵͳ��Ƶ�����󣬴�����Ҫ���³�ʼ�����������ʲ���
	USART_Configure();
}

void rcc_hsi_8mhz(void)
{
	SystemInit();	

	// ���³�ʼ��������Ź�
	WATCHDOG_DISABLE();
	WATCHDOG_ENABLE(mcu_clock_mode_8m);

	// ���³�ʼ��systick
	systick_init_hsi8mhz();

	// �ı�ϵͳ��ʱ�Ӻ�Ӧ���³�ʼ������Ӳ��
	USART_Configure();
}

// ����ϵͳ������ʱ���á�
void transaction_enter(void)
{		
	// ������Ҫ����ʱ��Ϩ���̵ơ����������ָʾϵͳæ��
	// led_switch(GPIO_LED_G, OFF);
	// led_switch(GPIO_LED_R, ON);	

	is_sys_idle = FALSE;
}

// �˳�ϵͳ������ʱ���á�
void transaction_quit(void)
{
	// ��������ɺ�Ϩ���ơ������̵���ָʾϵͳ���С�
	// led_switch(GPIO_LED_R, OFF);
	// led_switch(GPIO_LED_G, ON);

	is_sys_idle = TRUE;
}

// ����ص�ѹƫ�͵�ϵͳ�¼�������Ӧ����
void sys_check_event(void)
{			
	// printf("enter <sys_check_event>.\r\n");
	
	// ����Ƿ��GPS����ת��
	if(sw_gps_forward == ON)
	{
		if(is_gps_forwarding == FALSE)
		{
			transaction_enter();

			RxCnt2_rd = 0;
			RxCnt2_wr = 0;

			// ִ��GPS����ת��ǰӦ���´�USART2 Rx�ж�
			USART_Configure();

			// GPSģ��ǿ���ϵ�
			gps_power_up();

			delay_100ms(10);

			// ִ��GPS����ת��
			usart2_redir_usart1();	

			transaction_quit();
		}
	}	
}	 

// ���ϵͳ�ź�����
void sys_check_sem(void)
{
	int		to;

	char		reply[128];
	unsigned char		send[128]="hello";
	// ���ڶ�λ�����������0��˵���ж�λ������Ҫ����
	if(sem_periodic_pos > 0)
	{
		printf("sem_periodic_pos = %d\r\n", sem_periodic_pos);
		
		// ���ݶ�λ�������õ��ζ�λ��ʱ��
		if(cycle_periodic_pos > 60)
		{
			to =  60;
		}
		else
		{
			to = cycle_periodic_pos;
		}

		// gps��λ���ɹ�����ȡ��λ�����
		if(pos_via_gps(to) == OK)
		{
			sprintf(reply, "GPS pos info: lat=%3.6f, lon=%3.6f, alt=%3.6f, spd=%4.2f.\r\n", de_gpspos.lat, de_gpspos.lon, de_gpspos.alt, de_gpspos.spd);

			printf("%s", reply);

			// ��ȡ����gps��λ����󣬴�ram��ɾ��֮��
			clr_gpspos();

#if 1
			gsm_wakeup();

			// ͨ�����Żظ�gps��λ�����
//			gsm_send_sms(TEST_PHONE_NUMBER, reply);
			// ����ɸ�Ϊͨ��gprs���Ͷ�λ���������
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

			// ͨ�����Żظ�gps��λʧ�ܡ�
//			gsm_send_sms(TEST_PHONE_NUMBER, reply);
			// ����ɸ�Ϊͨ��gprs���Ͷ�λ���������
			gsm_send_data(GPRS_ID0, send, sizeof(send));
			gsm_sleep();
#endif			
		}

		// ���ڶ�λ����������ݼ�
		sem_periodic_pos--;
	}
}

