#ifndef __MCU_ADC
#define __MCU_ADC

#define MAX_TIMES_BAT_MEASURE		10		// 测量电池电压值时的采样次数(最后舍弃最大值和最小值后取平均值)
#define MAX_TIMES_ADC_CALIBRATION	5		// ADC配置完成后校准的次数

void ADC_Configure(void);
unsigned int batvol_measure(void);

#endif

