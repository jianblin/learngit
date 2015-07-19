#ifndef __MCU_GPIO
#define __MCU_GPIO

#include "stm32f10x.h"
#include "common.h"

// MCU IO控制脚数据结构及操作宏
typedef struct
{
	GPIO_TypeDef* 	GPIOx;
	uint16_t 		GPIO_Pin;
}T_MCU_GPIO;

typedef struct
{
	unsigned char	led;		// LED编号
	SWITCH			sw;			// LED开关控制参数
}T_LED_STATUS;

#define	GPIO_GSM_PWR			0							
#define	GPIO_GSM_ONOFF			1							
#define	GPIO_GSM_DTR			2		

#define	GPIO_GPS_PWR			3		

#if 1
#define	GPIO_LED_R				4							
#define	GPIO_LED_G				5							
#define	GPIO_LED_B				6	
#else
#define	GPIO_LED_B				4							
#define	GPIO_LED_G				5							
#define	GPIO_LED_R				6	
#endif

#define	GPIO_ADC_CT				7

#define	GPIO_EXTPWR_DET			8
#define	GPIO_ACCPWR_DET			9

#define	GPIO_RELAY1				10
#define	GPIO_RELAY2				11

extern T_MCU_GPIO	mcu_gpio_map[10];

extern T_LED_STATUS	last_led_status[3];

#define MCU_GPIO_HIGH(gpio_no)	GPIO_SetBits(  mcu_gpio_map[gpio_no].GPIOx, mcu_gpio_map[gpio_no].GPIO_Pin)	// 设置指定GPIO口为高电平
#define MCU_GPIO_LOW(gpio_no)	GPIO_ResetBits(mcu_gpio_map[gpio_no].GPIOx, mcu_gpio_map[gpio_no].GPIO_Pin)	// 设置指定GPIO口为低电平
#define MCU_GPIO_READ(gpio_no)	GPIO_ReadInputDataBit(mcu_gpio_map[gpio_no].GPIOx, mcu_gpio_map[gpio_no].GPIO_Pin)	// 读取指定GPIO口的电平状态

void GPIO_Configure(void);
void gpio_stop(void);
void gpio_start(void);
void led_switch(int led, SWITCH sw);
void led_sts_save(void);
void led_sts_recovery(void);
void led_flash1(int led,  int ms100,   int times);
void led_flash2(int led1, int led2, int led3, int ms100);

#endif
