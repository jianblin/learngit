#include "mcu_adc.h"
#include "mcu_gpio.h"
#include "stdio.h"

void adc_start(void);
void adc_stop(void);


/********************************************************************************************************************
*
*
*
*
*********************************************************************************************************************/
void ADC_Configure(void)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	ADC_InitTypeDef 	ADC_InitStructure;		

	int	i = MAX_TIMES_ADC_CALIBRATION;	

    /* 使能 ADC1 and GPIOC MCU_CLOCK_MODE */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);              		// 72M/6=12,ADC最大时间不能超过14M

	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_5;		// ADC_BT
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AIN;	// 模拟输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);	 
								
	/* 配置ADC1, 不用DMA, 用软件自己触发 */
	ADC_InitStructure.ADC_Mode 					= ADC_Mode_Independent;	//ADC1工作模式:独立模式
	ADC_InitStructure.ADC_ScanConvMode 			= DISABLE;			 	//单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode 	= DISABLE;				//单次转换
	ADC_InitStructure.ADC_ExternalTrigConv 		= ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign 			= ADC_DataAlign_Right;	//ADC1数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel 			= 1;	  				//顺序进行规则转换的ADC通道的数目

	ADC_Init(ADC1, &ADC_InitStructure);		   		//根据ADC_InitStruct中指定的参数，初始化外设ADC1的寄存器

	/* ADC1 regular channel11 configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_239Cycles5);//ADC1,ADC通道11,规则采样顺序值为1,采样时间为239.5周期

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);		  //使能ADC1

	/* Enable ADC1 reset calibaration register */
	ADC_ResetCalibration(ADC1);						//重置ADC1的校准寄存器
	
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));		//获取ADC1重置校准寄存器的状态,设置状态则等待

	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);						//开始ADC1的校准状态
	
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));			//等待校准完成

	/* Start ADC1 Software Conversion */
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);			//使能ADC1的软件转换启动功能

	/**************************** ADC配置完成后，测量三次以便校准ADC ****************************/
	
	// 接通电池电压测量
	MCU_GPIO_LOW(GPIO_ADC_CT);	

	// 舍弃前三次ADC测量值
	while(i--)
	{
		//软件启动ADC转换
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);	  
		
		//等待转换结束
		while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC )); 

		ADC_GetConversionValue(ADC1);
	}

	// 初始化ADC后将其关闭，以节电
	adc_stop();
}

void adc_start(void)
{
	MCU_GPIO_LOW(GPIO_ADC_CT);		// 接通电池电压测量

	ADC_Cmd(ADC1, ENABLE);		 	// 使能ADC功能
}	

void adc_stop(void)
{
	MCU_GPIO_HIGH(GPIO_ADC_CT);		// 断开电池电压测量

	ADC_Cmd(ADC1, DISABLE);		 	// 关闭ADC功能,功耗降低0.9mA
}

// 测量电池电压(ADC初始化后的前三次测量值一般误差较大，因此应舍去)。
unsigned int batvol_measure(void)
{
	unsigned int	val = 0;
	unsigned int	sum = 0;
		
	unsigned int 	max = 0 ;
	unsigned int	min = 0x10000000;
	
	int				i;

	// 测量电池电压前打开ADC
	adc_start();

	for(i = 0;i < MAX_TIMES_BAT_MEASURE; i++)
	{		
		//软件启动ADC转换
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);	  
		
		//等待转换结束
		while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC )); 	

		//读取ADC值
		val = ADC_GetConversionValue(ADC1);	

		// printf("ADC value = %d\r\n", val);

		sum += val;

		// 测量过程中记录最大值和最小值，以便计算平均值时去掉最大值和最小值再取平均
		if(val > max)
		{
			max = val;
		}

		if(val < min)
		{
			min = val;
		}
	}

	// 累加和中去掉最大值和最小值再计算平均值
	sum = sum - max - min;	

	// 测量电池电压后关闭ADC
	adc_stop();

	return ((sum/(MAX_TIMES_BAT_MEASURE-2))*328/2048);	// 将计算结果放大100倍，从而保留3位有效数字

	// 仅电池，			测量电压:3.73V
	// 电池+外接电源，	测量电压:3.83V(随着电池逐渐充电，电压会逐渐提升)
	// 仅外接电源，		测量电压:4.17V
}

