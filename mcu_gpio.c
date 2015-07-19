#include "mcu_exti.h"
#include "log_power.h"
#include "mcu_key.h"
#include "log_time.h"
#include "common.h"
#include "stdio.h"

#include "mcu_gpio.h"


T_LED_STATUS	last_led_status[3];		// �ϴε���led_switch��������LEDʱ��״̬(R��G��B��������)

static unsigned char led_sts_r, led_sts_g, led_sts_b;

/********************************************************************************************************************
*
*
*
*
*********************************************************************************************************************/
// output control pins definition
T_MCU_GPIO	mcu_gpio_map[10]=
{
	{GPIOA, GPIO_Pin_4	},	//	GSM_PWR		
	{GPIOA, GPIO_Pin_15	},	//	GSM_ONOFF		
	{GPIOA, GPIO_Pin_12	},	//	GSM_DTR		
	
	{GPIOB, GPIO_Pin_13	},	//	GPS_PWR		
	
	{GPIOB, GPIO_Pin_1	},	//	LED_R		    
	{GPIOB, GPIO_Pin_0	},	//	LED_G		    
	{GPIOA, GPIO_Pin_7	},	//	LED_B		    

	{GPIOB, GPIO_Pin_6	},	//	ADC_CT	

#ifdef USING_PWR_EXT
	{GPIOB, GPIO_Pin_12	},	//	EXTPWR_DET	
#endif

#if defined(USING_PWR_ACC)
	#if (HW_VER == 17)
	{GPIOB, GPIO_Pin_9	},	//	ACCPWR_DET
	#elif (HW_VER == 16)
	{GPIOB, GPIO_Pin_8	},	//	ACCPWR_DET
	#endif
#endif
};

void GPIO_Configure(void)
{
  	GPIO_InitTypeDef  GPIO_InitStructure;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

	// �����úõ�ƽ�������ã���������м䶯��
	MCU_GPIO_HIGH(GPIO_GSM_PWR);	
	MCU_GPIO_HIGH(GPIO_GSM_ONOFF);
	MCU_GPIO_HIGH(GPIO_GSM_DTR);

	MCU_GPIO_HIGH(GPIO_GPS_PWR);

	MCU_GPIO_HIGH(GPIO_LED_R);
	MCU_GPIO_HIGH(GPIO_LED_G);
	MCU_GPIO_HIGH(GPIO_LED_B);

	MCU_GPIO_LOW(GPIO_ADC_CT);

	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_7 | GPIO_Pin_12 | GPIO_Pin_15;

	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_OD;	// ��©���
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	 
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_OD;	// ��©���
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	// PB4��PB3��·��Ϊ����PB4Ӱ��PB3������PB4ΪIN_FLOATINGģʽ
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN_FLOATING;	// ��©���
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	// ��ʼ���ϴ�led������״̬����
	last_led_status[GPIO_LED_R-GPIO_LED_R].led = GPIO_LED_R;
	last_led_status[GPIO_LED_R-GPIO_LED_R].sw  = OFF;

	last_led_status[GPIO_LED_G-GPIO_LED_R].led = GPIO_LED_G;
	last_led_status[GPIO_LED_G-GPIO_LED_R].sw  = OFF;

	last_led_status[GPIO_LED_B-GPIO_LED_R].led = GPIO_LED_B;
	last_led_status[GPIO_LED_B-GPIO_LED_R].sw  = OFF;
}

// ��/�ر�ָ����LED�ơ�
void led_switch(int led, SWITCH sw)
{	
	// ÿ�β���ǰ��¼��Ʋ����Ĳ����������ܽ���ȥ�Ƿ�Ҫʵ�ʲ�����
	last_led_status[led-GPIO_LED_R].led 	= led;
	last_led_status[led-GPIO_LED_R].sw	= sw;

	// printf("led = %d, sw = %d\r\n", led, sw);
		
	// ϵͳ���ںڰ�ģʽʱ���������κ�LED�ƣ�ֻ�Ǽ�¼��Ʋ���������
	if(is_sys_in_darkmode == TRUE)
	{		
		return;
	}
	
	if(sw == ON)
	{
		MCU_GPIO_LOW(led);
	}
	else 
	{
		MCU_GPIO_HIGH(led);
	}
}

// ���浱ǰ��ɫled�Ƶ�����״̬��
void led_sts_save(void)
{
	led_sts_r = MCU_GPIO_READ(GPIO_LED_R);
	led_sts_g = MCU_GPIO_READ(GPIO_LED_G);
	led_sts_b = MCU_GPIO_READ(GPIO_LED_B);

	MCU_GPIO_HIGH(GPIO_LED_R);
	MCU_GPIO_HIGH(GPIO_LED_G);
	MCU_GPIO_HIGH(GPIO_LED_B);
}

// �ָ���ɫled�Ƶ�����״̬��
void led_sts_recovery(void)
{
	led_sts_r == 0?MCU_GPIO_LOW(GPIO_LED_R):MCU_GPIO_HIGH(GPIO_LED_R);
	led_sts_g == 0?MCU_GPIO_LOW(GPIO_LED_G):MCU_GPIO_HIGH(GPIO_LED_G);
	led_sts_b == 0?MCU_GPIO_LOW(GPIO_LED_B):MCU_GPIO_HIGH(GPIO_LED_B);
}

// (����led_switch����)��ָ���ĵ���led����ָ���ļ����˸ָ��������
void led_flash1(int led, int ms100, int times)
{	
	// ��˸ǰ������ɫled�Ƶ�����״̬��
	led_sts_save();
	
	while(times--)
	{
		led_switch(led, ON);

		delay_100ms((unsigned short)ms100);

		led_switch(led, OFF);

		delay_100ms((unsigned short)ms100);		
	}

	// ��˸��ָ���ɫled�Ƶ�����״̬��
	led_sts_recovery();
}

// ��ָ����ʱ�������ε�����Ϩ������LED�ơ�
void led_flash2(int led1, int led2, int led3, int ms100)
{
	// ��˸ǰ������ɫled�Ƶ�����״̬��
	led_sts_save();

	
	led_switch(led1, ON);

	delay_100ms((unsigned short)ms100);

	led_switch(led1, OFF);
	led_switch(led2, ON);

	delay_100ms((unsigned short)ms100);

	led_switch(led2, OFF);
	led_switch(led3, ON);

	delay_100ms((unsigned short)ms100);

	led_switch(led3, OFF);

	// ��˸��ָ���ɫled�Ƶ�����״̬��
	led_sts_recovery();
}

