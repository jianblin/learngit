#ifndef __TIMER
#define __TIMER

#include "common.h"
#include "stm32f10x.h"

#define TIMER2					0
#define TIMER3					1
#define TIMER4					2

#define TIMER_MODE_ONESHOT	0
#define TIMER_MODE_CYCLIC		1

#define DEF_WATCHDOG_PERIOD	60000		// 60s³¬Ê±Òç³ö

#define DEF_USERTIMER_PERIOD	200			// 200ms

extern unsigned int usertimer_counter;

enum MCU_CLOCK_MODE{mcu_clock_mode_8m=0,mcu_clock_mode_64m=1};

void timer_enable(TIM_TypeDef* TIMx, unsigned int cycle_ms, enum MCU_CLOCK_MODE mcu_clock_mode);
void timer_disable(TIM_TypeDef* TIMx);
void timer_reload(TIM_TypeDef* TIMx);

#define WATCHDOG_ENABLE(mcu_clock_mode)		timer_enable(TIM3, DEF_WATCHDOG_PERIOD, mcu_clock_mode)
#define WATCHDOG_DISABLE()					timer_disable(TIM3)
#define WATCHDOG_RELOAD()						timer_reload(TIM3)

#define USERTIMER_ENABLE(mcu_clock_mode)		timer_enable(TIM4, DEF_USERTIMER_PERIOD, mcu_clock_mode)
#define USERTIMER_DISABLE()					timer_disable(TIM4)
#define USERTIMER_RELOAD()					timer_reload(TIM4)

#endif

