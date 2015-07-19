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

    /* ʹ�� ADC1 and GPIOC MCU_CLOCK_MODE */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);              		// 72M/6=12,ADC���ʱ�䲻�ܳ���14M

	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_5;		// ADC_BT
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AIN;	// ģ������
	GPIO_Init(GPIOA, &GPIO_InitStructure);	 
								
	/* ����ADC1, ����DMA, ������Լ����� */
	ADC_InitStructure.ADC_Mode 					= ADC_Mode_Independent;	//ADC1����ģʽ:����ģʽ
	ADC_InitStructure.ADC_ScanConvMode 			= DISABLE;			 	//��ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode 	= DISABLE;				//����ת��
	ADC_InitStructure.ADC_ExternalTrigConv 		= ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign 			= ADC_DataAlign_Right;	//ADC1�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel 			= 1;	  				//˳����й���ת����ADCͨ������Ŀ

	ADC_Init(ADC1, &ADC_InitStructure);		   		//����ADC_InitStruct��ָ���Ĳ�������ʼ������ADC1�ļĴ���

	/* ADC1 regular channel11 configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_239Cycles5);//ADC1,ADCͨ��11,�������˳��ֵΪ1,����ʱ��Ϊ239.5����

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);		  //ʹ��ADC1

	/* Enable ADC1 reset calibaration register */
	ADC_ResetCalibration(ADC1);						//����ADC1��У׼�Ĵ���
	
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));		//��ȡADC1����У׼�Ĵ�����״̬,����״̬��ȴ�

	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);						//��ʼADC1��У׼״̬
	
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));			//�ȴ�У׼���

	/* Start ADC1 Software Conversion */
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);			//ʹ��ADC1�����ת����������

	/**************************** ADC������ɺ󣬲��������Ա�У׼ADC ****************************/
	
	// ��ͨ��ص�ѹ����
	MCU_GPIO_LOW(GPIO_ADC_CT);	

	// ����ǰ����ADC����ֵ
	while(i--)
	{
		//�������ADCת��
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);	  
		
		//�ȴ�ת������
		while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC )); 

		ADC_GetConversionValue(ADC1);
	}

	// ��ʼ��ADC����رգ��Խڵ�
	adc_stop();
}

void adc_start(void)
{
	MCU_GPIO_LOW(GPIO_ADC_CT);		// ��ͨ��ص�ѹ����

	ADC_Cmd(ADC1, ENABLE);		 	// ʹ��ADC����
}	

void adc_stop(void)
{
	MCU_GPIO_HIGH(GPIO_ADC_CT);		// �Ͽ���ص�ѹ����

	ADC_Cmd(ADC1, DISABLE);		 	// �ر�ADC����,���Ľ���0.9mA
}

// ������ص�ѹ(ADC��ʼ�����ǰ���β���ֵһ�����ϴ����Ӧ��ȥ)��
unsigned int batvol_measure(void)
{
	unsigned int	val = 0;
	unsigned int	sum = 0;
		
	unsigned int 	max = 0 ;
	unsigned int	min = 0x10000000;
	
	int				i;

	// ������ص�ѹǰ��ADC
	adc_start();

	for(i = 0;i < MAX_TIMES_BAT_MEASURE; i++)
	{		
		//�������ADCת��
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);	  
		
		//�ȴ�ת������
		while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC )); 	

		//��ȡADCֵ
		val = ADC_GetConversionValue(ADC1);	

		// printf("ADC value = %d\r\n", val);

		sum += val;

		// ���������м�¼���ֵ����Сֵ���Ա����ƽ��ֵʱȥ�����ֵ����Сֵ��ȡƽ��
		if(val > max)
		{
			max = val;
		}

		if(val < min)
		{
			min = val;
		}
	}

	// �ۼӺ���ȥ�����ֵ����Сֵ�ټ���ƽ��ֵ
	sum = sum - max - min;	

	// ������ص�ѹ��ر�ADC
	adc_stop();

	return ((sum/(MAX_TIMES_BAT_MEASURE-2))*328/2048);	// ���������Ŵ�100�����Ӷ�����3λ��Ч����

	// ����أ�			������ѹ:3.73V
	// ���+��ӵ�Դ��	������ѹ:3.83V(���ŵ���𽥳�磬��ѹ��������)
	// ����ӵ�Դ��		������ѹ:4.17V
}

