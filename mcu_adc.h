#ifndef __MCU_ADC
#define __MCU_ADC

#define MAX_TIMES_BAT_MEASURE		10		// ������ص�ѹֵʱ�Ĳ�������(����������ֵ����Сֵ��ȡƽ��ֵ)
#define MAX_TIMES_ADC_CALIBRATION	5		// ADC������ɺ�У׼�Ĵ���

void ADC_Configure(void);
unsigned int batvol_measure(void);

#endif

