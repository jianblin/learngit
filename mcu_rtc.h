#ifndef __MCU_RTC__
#define __MCU_RTC__

#define RTCClockSource_LSI				// ʹ��STM32 MCU���ڲ�����RTCʱ��(RC������~40KHz)

// #define RTCClockOutput_Enable

#define DEF_RTC_PRESCALER			40		// RTCĬ��Ƶ��Ϊ40KKHz,һ����30Khz~60KHz֮��䶯

#define RTC_PERIOD_ALARMINT		500		// RTC�����жϵ����ڳ���(��λ: ms)

#define RTC_PERIOD_COUNTER		1		// RTC������������(RTCʱ��Ƶ�ʵĵ���, 1ms)

#define MAX_RTC_COUNTER_VALUE		(4291367295u)	// RTCӲ���������ļ���ֵ���ޣ�=2^32-1-3600*1000������RTCӲ�������������ﵽ32λ�Ĵ�����������֮ǰ1Сʱ�ͽ�MCUǿ�и�λ
													// ��1ms��rtcʱ�����ڼ��㣬32λrtc���������ɼ�ʱ2^32-1 = 49.71��(ע��Ҫ����ֵ������ĩβ����u��ĸ����ָʾ)

extern unsigned int rtcalarm;	

void rtc_init(void);
void RTC_Calibrate(void);

#endif
