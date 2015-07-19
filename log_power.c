/*��ϵͳ��Ӳ����ص���������*/
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

BOOL			is_mcu_in_stopmode;		// MCU�Ƿ���STOPģʽ�ı�־
BOOL			is_sys_in_darkmode = FALSE;	// ϵͳ�Ƿ��ںڰ�ģʽ(ϵͳ��ʼ����LED�Ʋ�����)

// battery related
#ifdef USING_PWR_BAT
BOOL			is_bat_in_charging;			// ����Ƿ��ڳ��ı�־

unsigned int		de_batvol = 0.0;				// ��ص�ѹ(�Ŵ�100����ʾ�����ڵ������ʱ�����壬���δ������)	
#endif

unsigned char	sts_power;					// ϵͳ��Դ��״̬

/********************************************************************************************************************
*
*
*
*
*********************************************************************************************************************/
// ϵͳ��Դ�����ܳ�ʼ����
void power_init(void)
{		
	// mcuĬ�ϴ��ڷ�stopģʽ
	is_mcu_in_stopmode = FALSE;		

	// ϵͳĬ�Ϲرվ�Ĭģʽ(����ʼ�������м���ʼ���󶼿ɵ���led��)
	is_sys_in_darkmode = FALSE;	

#ifdef USING_PWR_BAT
	// ������ص�ѹ
	de_batvol = batvol_measure();			

	printf("battery voltage = %1.2f.\r\n", de_batvol/(double)100);
#endif

	sts_power = 0x00;

	// ext��Դ��acc��Դ״̬ͨ����ȡio״̬���á�
#ifdef USING_PWR_EXT	
	sts_power |= (MCU_GPIO_READ(GPIO_EXTPWR_DET)<<BIT_STS_POWER_EXT);	
#endif

#ifdef USING_PWR_ACC
	sts_power |= (MCU_GPIO_READ(GPIO_ACCPWR_DET)<<BIT_STS_POWER_ACC);
#endif

	// ���״̬ͨ����ص�ѹ�Ƚ�����(ֻҪUSB��DC��ӵ�Դ���룬��ص�ѹ�����Ľ��һ��Ϊ4.10v���ң����ܵ������������)��
	if(de_batvol < MAX_ADCVOL_FOR_BAT)
	{
		sts_power |= (1<<BIT_STS_POWER_BAT);
	}
	// else: �����жϵ���Ƿ������ϡ�
}

__asm void asm_mcu_reset(void)
{
	MOV R0, #1           	; 
 	MSR FAULTMASK, R0    	; // ���FAULTMASK ��ֹһ���жϲ���
 	LDR R0, =0xE000ED0C  	;
 	LDR R1, =0x05FA0004  	; 
 	STR R1, [R0]         	; // ϵͳ�����λ   
 
deadloop
    B deadloop        		; // ��ѭ��ʹ�������в�������Ĵ���
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

	// ���ÿ��1����˸3��
	led_flash1(GPIO_LED_R, 1, 3);	
	
#if 1
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_7 | GPIO_Pin_12 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN_FLOATING;	// ��©���
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	 
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN_FLOATING;	// ��©���
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, DISABLE);
#endif

	// ��Flash���ض�����д��������ֵ�Լ�¼ϵͳΪ�����ػ�
	bkp_u16_set(ENV_SYS_NORMAL_STARTUP_START, TAG_START_UP);

	// MCU�������ģʽ
	mcu_standby();				
}

//ϵͳ���������Standby��ģʽ
void mcu_standby(void)
{			 
	RCC_APB2PeriphResetCmd(0X01FC,DISABLE);
	
	NVIC_SystemLPConfig(NVIC_LP_SLEEPDEEP,ENABLE);	   

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);	// ʹ��PWR��BKP����ʱ��    

	PWR_WakeUpPinCmd(ENABLE);  							// ʹ�ܻ��ѹܽŹ���
	
	PWR_EnterSTANDBYMode();	 				 			// �������ģʽ
}

//ϵͳ����ֹͣ��Stop��ģʽ��
void mcu_stop_enter(void)
{
	// printf("to put MCU in STOP mode.\r\n");
	
	is_mcu_in_stopmode = TRUE;
	
	// ��MCU����STOPģʽ	
	PWR_EnterSTOPMode(PWR_Regulator_LowPower,PWR_STOPEntry_WFI);

	// printf("MCU wakeup from STOP mode.\r\n");
}

// ����ģʽ�¼����ӵ�Դ�Ƿ���롢��Яģʽ�¼�����Ƿ����С�
void check_power_status(void)
{
	
	if(GET_STS_POWER(BIT_STS_POWER_EXT) == ON)
	{
		// �̵�����������ָʾ��ӵ�Դ��ͨ��
		led_switch(GPIO_LED_G, OFF);
		led_switch(GPIO_LED_B, ON);
	}
	else
	{
		// �̵�����������ָʾ��ӵ�Դ�Ͽ���
		led_switch(GPIO_LED_G, ON);
		led_switch(GPIO_LED_B, OFF);
	}
}

