/*��ʱ������*/
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
// ��ָ����ģʽ��ָ���Ķ�ʱ�������ָ����timer��
// timer��ʱģʽ��Ϊone-shot��cyclic��ǰ��Ϊһ���Զ�ʱģʽ������Ϊ�����Զ�ʱģʽ��
// Ŀǰ1�Ŷ�ʱ���ֱ���Ϊ10mS��2�š�3�ž�Ϊ1�룬���ȡֵ��Χ��54�룬Ҫ�����Ķ�ʱ�������ж��м���һ��32λ��ȫ�ֱ���������
// ��ʵ�ָ�����ʱ
void timer_start(unsigned char id, unsigned int to, unsigned char mode)
{
	switch(id)
	{
//==================== 1�Ŷ�ʱ���ֱ�����Ϊ10mS��toȡֵ��Χ��1~5461 ====================//
		case TIMER2://1
			
			RCC->APB1ENR|=0x01; 					//ʹ��TIM2ʱ��

			if(mode==TIMER_MODE_ONESHOT)
			{
				TIM2->CR1|=(1<<3);    				//���μ���
			}
			else 
			{
				TIM2->CR1&=~(1<<3);    				//���ڼ���
			}
			
			TIM2->CNT=0;							//��������������¼���

#ifdef USING_MCU_HSI_8MHZ		// 8Mhz		
			TIM2->PSC=8000-1; 						//Ԥ��Ƶ��

			TIM2->ARR=10*to;  						//�趨�������Զ���װֵ
#else	// 64Mhz
			TIM2->PSC=64000-1; 						//Ԥ��Ƶ��

			TIM2->ARR=10*to;  						//�趨�������Զ���װֵ			
#endif
			
			TIM2->SR&=~(1<<0);						//����жϱ�־λ������һ�����ж����Ͻ����ж�
			
			TIM2->DIER|=1<<0;   					//����TIM2����ж�
			
			TIM2->CR1|=0x01;    					//ʹ�ܶ�ʱ��2

			break;
		
//==================== 2�Ŷ�ʱ���ֱ�����Ϊ1S��toȡֵ��Χ��1~54 ====================//
		case TIMER3://2
			
			RCC->APB1ENR|=0x02; 					//ʹ��TIM3ʱ�� 
					
			if(mode==TIMER_MODE_ONESHOT)
			{
				TIM3->CR1|=(1<<3);    				//���μ���
			}
			else 
			{
				TIM3->CR1&=~(1<<3);    				//���ڼ���
			}
			TIM3->CNT=0;									//��������������¼���

#ifdef USING_MCU_HSI_8MHZ		// 8Mhz	
			TIM3->PSC=8000-1; 						//Ԥ��Ƶ��
	
			TIM3->ARR=1000*to;  					//�趨�������Զ���װֵ
#else		// 64Mhz
			TIM3->PSC=64000-1; 						//Ԥ��Ƶ��
	
			TIM3->ARR=1000*to;  					//�趨�������Զ���װֵ			
#endif
			
			TIM3->SR&=~(1<<0);						//����жϱ�־λ������һ�����ж����Ͻ����ж�
			
			TIM3->DIER|=1<<0;   //��������ж�				
			TIM3->DIER|=1<<6;   //�������ж�

			TIM3->SR&=~(1<<0);						//����жϱ�־λ������һ�����ж����Ͻ����ж�

			TIM3->CR1|=0x01;    					//ʹ�ܶ�ʱ��3

			break;
//==================== 3�Ŷ�ʱ���ֱ�����Ϊ1S��toȡֵ��Χ��1~54 ====================//
		case TIMER4://3	
			
			RCC->APB1ENR|=0x04; 					//ʹ��TIM4ʱ�� 
			
			if(mode==TIMER_MODE_ONESHOT)
			{
				TIM4->CR1|=(1<<3);    				//���μ���
			}
			else 
			{
				TIM4->CR1&=~(1<<3);    				//���ڼ���
			}
			TIM4->CNT=0;							//��������������¼���
			
#ifdef USING_MCU_HSI_8MHZ		// 8Mhz	
			TIM4->PSC=8000-1; 						//Ԥ��Ƶ��
	
			TIM4->ARR=1000*to;  					//�趨�������Զ���װֵ
#else		// 64Mhz
			TIM4->PSC=64000-1; 						//Ԥ��Ƶ��
	
			TIM4->ARR=1000*to;  					//�趨�������Զ���װֵ			
#endif
			
			TIM4->SR&=~(1<<0);  					//����жϱ�־λ������һ�����ж����Ͻ����ж�
			
			TIM4->DIER|=1<<0;   					//����TIM4����ж�

			TIM4->CR1|=0x01;    					//ʹ�ܶ�ʱ��4

			break;
		default:
			break;
	}
}

// ָֹͣ����ʱ���Ķ�ʱ��
void timer_stop(unsigned char id)
{
	switch(id)
	{
		case TIMER2:				// �ر�1�Ŷ�ʱ��
			TIM2->CR1&=~0x01;    	// ֹͣ��ʱ��2����
			
			RCC->APB1ENR&=~0x01; 	// ֹͣTIM2ʱ�� 

			break;
		case TIMER3:				// �ر�2�Ŷ�ʱ��
			TIM3->CR1&=~0x01;    	// ֹͣ��ʱ��3����
			
			RCC->APB1ENR&=~0x02; 	// ֹͣTIM3ʱ��

			break;
		case TIMER4:				// �ر�3�Ŷ�ʱ��
			TIM4->CR1&=~0x01;    	// ֹͣ��ʱ��4����
			
			RCC->APB1ENR&=~0x04; 	// ֹͣTIM4ʱ��

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
			TIM2->CNT=0;			//��������������¼���

			break;
		case TIMER3:				// �ر�2�Ŷ�ʱ��
			TIM3->CNT=1000*5;		//��������������¼���

			break;
		case TIMER4:				// �ر�3�Ŷ�ʱ��
			TIM4->CNT=0;			//��������������¼���

			break;
		default:
			break;
	}
}
*/


// ʹ��timer(Ĭ��Ϊѭ������ģʽ)��
void timer_enable(TIM_TypeDef* TIMx, unsigned int cycle_ms, enum MCU_CLOCK_MODE mcu_clock_mode)
{
	//����TIM����
  	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	unsigned int 	prescaler;

  	//����TIMʱ��
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

	// �趨��ʱʱ��
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

	TIM_TimeBaseStructure.TIM_Period		= cycle_ms;     		// �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ)���ۼ� TIM_Period��Ƶ�ʺ����һ�����»����ж�
	TIM_TimeBaseStructure.TIM_Prescaler		= prescaler;    		// ʱ��Ԥ��Ƶ��������Ϊ1ms����
	TIM_TimeBaseStructure.TIM_ClockDivision	= TIM_CKD_DIV1;   		// ������Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode	= TIM_CounterMode_Up;   // ���ϼ���ģʽ

	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

 	TIM_ClearFlag(TIMx, TIM_FLAG_Update);       					//�������жϱ�־
 
 	//TIM_PrescalerConfig(TIMx,0x8C9F,TIM_PSCReloadMode_Immediate); 	//ʱ�ӷ�Ƶϵ��36000�����Զ�ʱ��ʱ��Ϊ2K
 	//TIM_ARRPreloadConfig(TIMx, DISABLE);       						//��ֹARRԤװ�ػ�����

 	TIM_ITConfig(TIMx,TIM_IT_Update,ENABLE);
 
 	TIM_Cmd(TIMx, ENABLE);   
}

// �ر�timer��
void timer_disable(TIM_TypeDef* TIMx)
{
	TIM_Cmd(TIMx, DISABLE);   
}

// ����timer��ֵ��
void timer_reload(TIM_TypeDef* TIMx)
{
	TIM_SetCounter(TIMx, 0);
}

