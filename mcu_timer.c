/*定时器管理*/
#include "mcu_timer.h"
#include "stdio.h"
#include "mcu_gpio.h"
#include "common.h"

unsigned int usertimer_counter;

/********************************************************************************************************************
*
*
*
*
*********************************************************************************************************************/
/*
// 以指定的模式、指定的定时间隔启动指定的timer。
// timer定时模式分为one-shot和cyclic，前者为一次性定时模式、后者为周期性定时模式。
// 目前1号定时器分辨率为10mS，2号、3号均为1秒，最大取值范围是54秒，要更长的定时，可在中断中加入一个32位的全局变量计数，
// 以实现更长定时
void timer_start(unsigned char id, unsigned int to, unsigned char mode)
{
	switch(id)
	{
//==================== 1号定时器分辨率设为10mS，to取值范围：1~5461 ====================//
		case TIMER2://1
			
			RCC->APB1ENR|=0x01; 					//使能TIM2时钟

			if(mode==TIMER_MODE_ONESHOT)
			{
				TIM2->CR1|=(1<<3);    				//单次计数
			}
			else 
			{
				TIM2->CR1&=~(1<<3);    				//周期计数
			}
			
			TIM2->CNT=0;							//清零计数器，重新计数

#ifdef USING_MCU_HSI_8MHZ		// 8Mhz		
			TIM2->PSC=8000-1; 						//预分频数

			TIM2->ARR=10*to;  						//设定计数器自动重装值
#else	// 64Mhz
			TIM2->PSC=64000-1; 						//预分频数

			TIM2->ARR=10*to;  						//设定计数器自动重装值			
#endif
			
			TIM2->SR&=~(1<<0);						//清除中断标志位，避免一启动中断马上进入中断
			
			TIM2->DIER|=1<<0;   					//允许TIM2溢出中断
			
			TIM2->CR1|=0x01;    					//使能定时器2

			break;
		
//==================== 2号定时器分辨率设为1S，to取值范围：1~54 ====================//
		case TIMER3://2
			
			RCC->APB1ENR|=0x02; 					//使能TIM3时钟 
					
			if(mode==TIMER_MODE_ONESHOT)
			{
				TIM3->CR1|=(1<<3);    				//单次计数
			}
			else 
			{
				TIM3->CR1&=~(1<<3);    				//周期计数
			}
			TIM3->CNT=0;									//清零计数器，重新计数

#ifdef USING_MCU_HSI_8MHZ		// 8Mhz	
			TIM3->PSC=8000-1; 						//预分频数
	
			TIM3->ARR=1000*to;  					//设定计数器自动重装值
#else		// 64Mhz
			TIM3->PSC=64000-1; 						//预分频数
	
			TIM3->ARR=1000*to;  					//设定计数器自动重装值			
#endif
			
			TIM3->SR&=~(1<<0);						//清除中断标志位，避免一启动中断马上进入中断
			
			TIM3->DIER|=1<<0;   //允许更新中断				
			TIM3->DIER|=1<<6;   //允许触发中断

			TIM3->SR&=~(1<<0);						//清除中断标志位，避免一启动中断马上进入中断

			TIM3->CR1|=0x01;    					//使能定时器3

			break;
//==================== 3号定时器分辨率设为1S，to取值范围：1~54 ====================//
		case TIMER4://3	
			
			RCC->APB1ENR|=0x04; 					//使能TIM4时钟 
			
			if(mode==TIMER_MODE_ONESHOT)
			{
				TIM4->CR1|=(1<<3);    				//单次计数
			}
			else 
			{
				TIM4->CR1&=~(1<<3);    				//周期计数
			}
			TIM4->CNT=0;							//清零计数器，重新计数
			
#ifdef USING_MCU_HSI_8MHZ		// 8Mhz	
			TIM4->PSC=8000-1; 						//预分频数
	
			TIM4->ARR=1000*to;  					//设定计数器自动重装值
#else		// 64Mhz
			TIM4->PSC=64000-1; 						//预分频数
	
			TIM4->ARR=1000*to;  					//设定计数器自动重装值			
#endif
			
			TIM4->SR&=~(1<<0);  					//清除中断标志位，避免一启动中断马上进入中断
			
			TIM4->DIER|=1<<0;   					//允许TIM4溢出中断

			TIM4->CR1|=0x01;    					//使能定时器4

			break;
		default:
			break;
	}
}

// 停止指定定时器的定时。
void timer_stop(unsigned char id)
{
	switch(id)
	{
		case TIMER2:				// 关闭1号定时器
			TIM2->CR1&=~0x01;    	// 停止定时器2计数
			
			RCC->APB1ENR&=~0x01; 	// 停止TIM2时钟 

			break;
		case TIMER3:				// 关闭2号定时器
			TIM3->CR1&=~0x01;    	// 停止定时器3计数
			
			RCC->APB1ENR&=~0x02; 	// 停止TIM3时钟

			break;
		case TIMER4:				// 关闭3号定时器
			TIM4->CR1&=~0x01;    	// 停止定时器4计数
			
			RCC->APB1ENR&=~0x04; 	// 停止TIM4时钟

			break;
		default:
			break;
	}
}

void timer_reload(unsigned char id)
{
	switch(id)
	{
		case TIMER2:				
			TIM2->CNT=0;			//清零计数器，重新计数

			break;
		case TIMER3:				// 关闭2号定时器
			TIM3->CNT=1000*5;		//清零计数器，重新计数

			break;
		case TIMER4:				// 关闭3号定时器
			TIM4->CNT=0;			//清零计数器，重新计数

			break;
		default:
			break;
	}
}
*/


// 使能timer(默认为循环计数模式)。
void timer_enable(TIM_TypeDef* TIMx, unsigned int cycle_ms, enum MCU_CLOCK_MODE mcu_clock_mode)
{
	//设置TIM类型
  	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	unsigned int 	prescaler;

  	//设置TIM时钟
  	if(TIMx == TIM2)
  	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
  	}
	else if(TIMx == TIM3)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);
	}
	else if(TIMx == TIM4)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
	}

 	TIM_DeInit(TIMx);

	// 设定超时时间
	if(mcu_clock_mode == mcu_clock_mode_8m)
	{
		prescaler = (8000 - 1);
	}
	else if(mcu_clock_mode == mcu_clock_mode_64m)
	{
		prescaler = (64000 - 1);
	}
	else
	{
		return;
	}

	TIM_TimeBaseStructure.TIM_Period		= cycle_ms;     		// 自动重装载寄存器周期的值(计数值)，累计 TIM_Period个频率后产生一个更新或者中断
	TIM_TimeBaseStructure.TIM_Prescaler		= prescaler;    		// 时钟预分频数，设置为1ms周期
	TIM_TimeBaseStructure.TIM_ClockDivision	= TIM_CKD_DIV1;   		// 采样分频
	TIM_TimeBaseStructure.TIM_CounterMode	= TIM_CounterMode_Up;   // 向上计数模式

	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

 	TIM_ClearFlag(TIMx, TIM_FLAG_Update);       					//清除溢出中断标志
 
 	//TIM_PrescalerConfig(TIMx,0x8C9F,TIM_PSCReloadMode_Immediate); 	//时钟分频系数36000，所以定时器时钟为2K
 	//TIM_ARRPreloadConfig(TIMx, DISABLE);       						//禁止ARR预装载缓冲器

 	TIM_ITConfig(TIMx,TIM_IT_Update,ENABLE);
 
 	TIM_Cmd(TIMx, ENABLE);   
}

// 关闭timer。
void timer_disable(TIM_TypeDef* TIMx)
{
	TIM_Cmd(TIMx, DISABLE);   
}

// 重载timer数值。
void timer_reload(TIM_TypeDef* TIMx)
{
	TIM_SetCounter(TIMx, 0);
}

