#ifndef __EXTI
#define __EXTI

#include "stm32f10x.h"

typedef struct
{
	GPIO_TypeDef* 	GPIOx;
	uint16_t 			GPIO_Pin;
}T_MCU_EXTI;

extern T_MCU_EXTI	mcu_exti_map[7];

#define EXTI_SYS_KEY		0		// �����ж�
#define EXTI_GSM_RING	1		// gsmģ��ring�ж�
#define EXTI_GSR_INT2		2		// ���ٶȴ���������ж�2
#define EXTI_GSR_INT1		3		// ���ٶȴ���������ж�1

#define EXTI_CHG_STS		4		// ��س��״ָ̬ʾ�ж�

#define EXTI_EXTPWR_DET	5		// �ⲿ����Դ��ͨ/�Ͽ�����ж�
#define EXTI_ACCPWR_DET	6		// ACC��Դ��ͨ/�Ͽ�����ж�

void EXTI_Configure(void);

#endif
