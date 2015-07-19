#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "stm32f10x.h"


#define SYSTICK_PERIOD			100						// 100 ms

#define SYSTICK_FREQUENCY		1000000L				// 1MHz

#define SYSTICK_MAX_VALUE		((SYSTICK_PERIOD*SYSTICK_FREQUENCY/1000))	// systick计数器位数为24，但是计数值可人为设定: (SYSTICK_PERIOD/1000)/(1/SYSTICK_FREQUENCY)
		
/* SysTick counter state */
#define SysTick_Counter_Disable        ((u32)0xFFFFFFFE)
#define SysTick_Counter_Enable         ((u32)0x00000001)
#define SysTick_Counter_Clear          ((u32)0x00000000)

/* CTRL TICKINT Mask */
#define CTRL_TICKINT_Set     		 ((u32)0x00000002)
#define CTRL_TICKINT_Reset  		 ((u32)0xFFFFFFFD)

/* SysTick Flag Mask */
#define FLAG_Mask           			  ((u8)0x1F)


extern volatile unsigned int systick;;

void systick_init_hsi8mhz(void);
void systick_init_hsi64mhz(void);

void SysTick_Enable(void);
void SysTick_Disable(void);

void SysTick_CLKSourceConfig(u32 SysTick_CLKSource);
void SysTick_SetReload(u32 Reload);
void SysTick_CounterCmd(u32 SysTick_Counter);
void SysTick_ITConfig(FunctionalState NewState);
u32 SysTick_GetCounter(void);
// FlagStatus SysTick_GetFlagStatus(u8 SysTick_FLAG);

#endif
