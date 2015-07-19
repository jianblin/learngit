#include "mcu_systick.h"
#include "stm32f10x.h"

volatile unsigned int systick;			// ϵͳʱ�䣨��ϵͳ�ϵ��

/********************************************************************************************************************
*
*
*
*
*********************************************************************************************************************/
/*
 * ��������SysTick_Init
 * ����  ������ϵͳ�δ�ʱ�� SysTick
 * ����  ����
 * ���  ����
 * ����  ���ⲿ���� 
 */
void systick_init_hsi8mhz(void)
{
	systick = 0;

	// ѡ��systickʱ��ԴΪHCLK����8(8MHz / 8 = 1MHz)
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);

	// ����systick���������(systick������Ϊ24λ����ѡ�����ʱ��Ƶ��Ϊ1MHz����˿ɶ�ʱ�����λʱ��Ϊ16s)
	// ע��: �����100�����ú�SYSTICK_PERIOD���棬��Ȼ���ߵ���ֵҲ��100������ʵ�鷢�ֻᵼ��ʵ�ʵ�systick����ƫ��ü��������������ڴ����Ż���ɵ�!!!
	SysTick_SetReload(1000*100-1);	

	// ʹ��systick����
	SysTick_CounterCmd(SysTick_Counter_Enable);

	// ��ʹ��systick����RTC tick�����ڳ���֮ǰ����ʼ��systick
	NVIC_SetPriority(SysTick_IRQn, 0);	

	// ʹ��systick����ж�
	SysTick_ITConfig(ENABLE);
}

void systick_init_hsi64mhz(void)
{
	systick = 0;

	// ѡ��systickʱ��ԴΪHCLK����8(8MHz / 8 = 1MHz)
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);

	// ����systick���������(systick������Ϊ24λ����ѡ�����ʱ��Ƶ��Ϊ1MHz����˿ɶ�ʱ�����λʱ��Ϊ16s)
	// ע��: �����100�����ú�SYSTICK_PERIOD���棬��Ȼ���ߵ���ֵҲ��100������ʵ�鷢�ֻᵼ��ʵ�ʵ�systick����ƫ��ü��������������ڴ����Ż���ɵ�!!!
	SysTick_SetReload(8000*100-1);	

	// ʹ��systick����
	SysTick_CounterCmd(SysTick_Counter_Enable);

	// ��ʹ��systick����RTC tick�����ڳ���֮ǰ����ʼ��systick
	NVIC_SetPriority(SysTick_IRQn, 0);	

	// ʹ��systick����ж�
	SysTick_ITConfig(ENABLE);
}

/*******************************************************************************
* Function Name  : SysTick_CLKSourceConfig
* Description    : Configures the SysTick MCU_CLOCK_MODE source.
* Input          : - SysTick_CLKSource: specifies the SysTick MCU_CLOCK_MODE source.
*                    This parameter can be one of the following values:
*                       - SysTick_CLKSource_HCLK_Div8: AHB MCU_CLOCK_MODE divided by 8
*                         selected as SysTick MCU_CLOCK_MODE source.
*                       - SysTick_CLKSource_HCLK: AHB MCU_CLOCK_MODE selected as
*                         SysTick MCU_CLOCK_MODE source.
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_CLKSourceConfig(u32 SysTick_CLKSource)
{
  /* Check the parameters */
  assert_param(IS_SYSTICK_CLK_SOURCE(SysTick_CLKSource));

  if (SysTick_CLKSource == SysTick_CLKSource_HCLK)
  {
    SysTick->CTRL |= SysTick_CLKSource_HCLK;
  }
  else
  {
    SysTick->CTRL &= SysTick_CLKSource_HCLK_Div8;
  }
}

/*******************************************************************************
* Function Name  : SysTick_SetReload
* Description    : Sets SysTick Reload value.
* Input          : - Reload: SysTick Reload new value.
*                    This parameter must be a number between 1 and 0xFFFFFF.
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_SetReload(u32 Reload)
{
  /* Check the parameters */
  assert_param(IS_SYSTICK_RELOAD(Reload));

  SysTick->LOAD = Reload;
}

/*******************************************************************************
* Function Name  : SysTick_CounterCmd
* Description    : Enables or disables the SysTick counter.
* Input          : - SysTick_Counter: new state of the SysTick counter.
*                    This parameter can be one of the following values:
*                       - SysTick_Counter_Disable: Disable counter
*                       - SysTick_Counter_Enable: Enable counter
*                       - SysTick_Counter_Clear: Clear counter value to 0
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_CounterCmd(u32 SysTick_Counter)
{
  /* Check the parameters */
  assert_param(IS_SYSTICK_COUNTER(SysTick_Counter));

  if (SysTick_Counter == SysTick_Counter_Clear)
  {
    SysTick->VAL = SysTick_Counter_Clear;
  }
  else
  {
    if (SysTick_Counter == SysTick_Counter_Enable)
    {
      SysTick->CTRL |= SysTick_Counter_Enable;
    }
    else
    {
      SysTick->CTRL &= SysTick_Counter_Disable;
    }
  }
}

/*******************************************************************************
* Function Name  : SysTick_ITConfig
* Description    : Enables or disables the SysTick Interrupt.
* Input          : - NewState: new state of the SysTick Interrupt.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_ITConfig(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    SysTick->CTRL |= CTRL_TICKINT_Set;
  }
  else
  {
    SysTick->CTRL &= CTRL_TICKINT_Reset;
  }
}

/*******************************************************************************
* Function Name  : SysTick_GetCounter
* Description    : Gets SysTick counter value.
* Input          : None
* Output         : None
* Return         : SysTick current value
*******************************************************************************/
u32 SysTick_GetCounter(void)
{
  return(SysTick->VAL);
}

#if 0

/*******************************************************************************
* Function Name  : SysTick_GetFlagStatus
* Description    : Checks whether the specified SysTick flag is set or not.
* Input          : - SysTick_FLAG: specifies the flag to check.
*                    This parameter can be one of the following values:
*                       - SysTick_FLAG_COUNT
*                       - SysTick_FLAG_SKEW
*                       - SysTick_FLAG_NOREF
* Output         : None
* Return         : None
*******************************************************************************/
FlagStatus SysTick_GetFlagStatus(u8 SysTick_FLAG)
{
  u32 tmp = 0;
  u32 statusreg = 0;
  FlagStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_SYSTICK_FLAG(SysTick_FLAG));

  /* Get the SysTick register index */
  tmp = SysTick_FLAG >> 5;

  if (tmp == 1) /* The flag to check is in CTRL register */
  {
    statusreg = SysTick->CTRL;
  }
  else          /* The flag to check is in CALIB register */
  {
    statusreg = SysTick->CALIB;
  }

  /* Get the flag position */
  tmp = SysTick_FLAG & FLAG_Mask;

  if ((statusreg & ((u32)1 << tmp)) != (u32)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

#endif

// ��ʼsystick������ʹ��systick�жϡ�
void SysTick_Enable(void)
{
	SysTick->CTRL = (1 << SYSTICK_CLKSOURCE) | (1<<SYSTICK_ENABLE) | (1<<SYSTICK_TICKINT);	
}

// ֹͣsystick��������ֹsystick�жϡ�
void SysTick_Disable(void)
{	
	 SysTick->CTRL = (1 << SYSTICK_CLKSOURCE) & ~(1<<SYSTICK_ENABLE) & ~(1<<SYSTICK_TICKINT);
}


