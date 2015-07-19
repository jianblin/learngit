#ifndef __ALARM
#define __ALARM

#include "common.h"
#include "dev_gsm.h"

#define BIT_ALARM_SOS				7		// sos报警标志位序号
#define BIT_ALARM_EMERGENCY		6		// 紧急报警标志位序号
#define BIT_ALARM_ANTITHEFT		5		// 防盗报警标志位序号
#define BIT_ALARM_EXTPWROFF		4		// 外部电源断开报警标志位序号
#define BIT_ALARM_LOWBATVOL		3		// 电源(任何)电压过低报警标志位序号
#define BIT_ALARM_OVERSPEED		2		// 超速报警标志位序号
#define BIT_ALARM_OVERDUE			1		// 欠费报警标志位序号

#define SWITCH_ALARM_ON(bit)		(sw_alarm |=  (1<<bit))		// 打开指定的报警功能
#define SWITCH_ALARM_OFF(bit)		(sw_alarm &= ~(1<<bit))		// 关闭指定的报警功能

#define GET_SWITCH_ALARM(bit)		((sw_alarm&(1<<bit))?ON:OFF)

#define SET_EVENT_ALARM(bit)		(event_alarm |=  (1<<bit))	// 设置指定的报警事件
#define CLR_EVENT_ALARM(bit)		(event_alarm &= ~(1<<bit))	// 清除指定的报警事件

#define GET_EVENT_ALARM(bit)		((event_alarm&(1<<bit))?ON:OFF)	// 清除指定的报警事件

extern unsigned char	sw_alarm;
extern unsigned char	event_alarm;

void event_init(void);
void sys_check_event(void);

#ifdef USING_FUNC_ALARM_LOWBATVOL

#define STS_ALARM_LOWBATVOL_SNEG		-1			// 低电报警功能关闭
#define STS_ALARM_LOWBATVOL_S0		0			// 待检测低电
#define STS_ALARM_LOWBATVOL_S1		1			// 已检测到低电、待检测持续低电
#define STS_ALARM_LOWBATVOL_S2		2			// 已检测到持续低电
#define STS_ALARM_LOWBATVOL_S3		3			// 已检测到常电、待检测持续常电

#define DEF_WINDOW_DETECT_CONTINUOUS_BATVOL		400		// 持续检测电池电压的窗口时间(电池电压测量周期为102秒，持续电池低电或常电检测的时间窗口应能保证检测三次电池电压)

extern STATUS			sts_alarm_lowbatvol;

extern unsigned int		swtimer_window_detect_continuous_lowbatvol;	
extern unsigned int		swtimer_window_detect_continuous_nmlbatvol;

#endif

#ifdef USING_FUNC_ALARM_EXTPWROFF

#define STS_ALARM_EXTPWROFF_SNEG		-1			// 断电报警功能关闭
#define STS_ALARM_EXTPWROFF_S0		0			// 待检测断电
#define STS_ALARM_EXTPWROFF_S1		1			// 已检测到断电

extern STATUS			sts_alarm_extpwroff;
																	
#endif

void send_alarm_msg(char* msg);

void alarm_init(void);

#endif

