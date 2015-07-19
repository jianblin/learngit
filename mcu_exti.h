#ifndef __EXTI
#define __EXTI

#include "stm32f10x.h"

typedef struct
{
	GPIO_TypeDef* 	GPIOx;
	uint16_t 			GPIO_Pin;
}T_MCU_EXTI;

extern T_MCU_EXTI	mcu_exti_map[7];

#define EXTI_SYS_KEY		0		// 按键中断
#define EXTI_GSM_RING	1		// gsm模块ring中断
#define EXTI_GSR_INT2		2		// 加速度传感器输出中断2
#define EXTI_GSR_INT1		3		// 加速度传感器输出中断1

#define EXTI_CHG_STS		4		// 电池充电状态指示中断

#define EXTI_EXTPWR_DET	5		// 外部主电源接通/断开检测中断
#define EXTI_ACCPWR_DET	6		// ACC电源接通/断开检测中断

void EXTI_Configure(void);

#endif
