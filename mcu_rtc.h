#ifndef __MCU_RTC__
#define __MCU_RTC__

#define RTCClockSource_LSI				// 使用STM32 MCU的内部低速RTC时钟(RC振荡器，~40KHz)

// #define RTCClockOutput_Enable

#define DEF_RTC_PRESCALER			40		// RTC默认频率为40KKHz,一般在30Khz~60KHz之间变动

#define RTC_PERIOD_ALARMINT		500		// RTC闹钟中断的周期长度(单位: ms)

#define RTC_PERIOD_COUNTER		1		// RTC计数器的周期(RTC时钟频率的倒数, 1ms)

#define MAX_RTC_COUNTER_VALUE		(4291367295u)	// RTC硬件计数器的计数值上限，=2^32-1-3600*1000，即在RTC硬件计数器即将达到32位寄存器计数上限之前1小时就将MCU强行复位
													// 以1ms的rtc时钟周期计算，32位rtc计数器最多可及时2^32-1 = 49.71天(注意要在数值敞亮的末尾加上u字母，以指示)

extern unsigned int rtcalarm;	

void rtc_init(void);
void RTC_Calibrate(void);

#endif
