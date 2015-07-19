#ifndef __ALARM
#define __ALARM

#include "common.h"
#include "dev_gsm.h"

#define BIT_ALARM_SOS				7		// sos������־λ���
#define BIT_ALARM_EMERGENCY		6		// ����������־λ���
#define BIT_ALARM_ANTITHEFT		5		// ����������־λ���
#define BIT_ALARM_EXTPWROFF		4		// �ⲿ��Դ�Ͽ�������־λ���
#define BIT_ALARM_LOWBATVOL		3		// ��Դ(�κ�)��ѹ���ͱ�����־λ���
#define BIT_ALARM_OVERSPEED		2		// ���ٱ�����־λ���
#define BIT_ALARM_OVERDUE			1		// Ƿ�ѱ�����־λ���

#define SWITCH_ALARM_ON(bit)		(sw_alarm |=  (1<<bit))		// ��ָ���ı�������
#define SWITCH_ALARM_OFF(bit)		(sw_alarm &= ~(1<<bit))		// �ر�ָ���ı�������

#define GET_SWITCH_ALARM(bit)		((sw_alarm&(1<<bit))?ON:OFF)

#define SET_EVENT_ALARM(bit)		(event_alarm |=  (1<<bit))	// ����ָ���ı����¼�
#define CLR_EVENT_ALARM(bit)		(event_alarm &= ~(1<<bit))	// ���ָ���ı����¼�

#define GET_EVENT_ALARM(bit)		((event_alarm&(1<<bit))?ON:OFF)	// ���ָ���ı����¼�

extern unsigned char	sw_alarm;
extern unsigned char	event_alarm;

void event_init(void);
void sys_check_event(void);

#ifdef USING_FUNC_ALARM_LOWBATVOL

#define STS_ALARM_LOWBATVOL_SNEG		-1			// �͵籨�����ܹر�
#define STS_ALARM_LOWBATVOL_S0		0			// �����͵�
#define STS_ALARM_LOWBATVOL_S1		1			// �Ѽ�⵽�͵硢���������͵�
#define STS_ALARM_LOWBATVOL_S2		2			// �Ѽ�⵽�����͵�
#define STS_ALARM_LOWBATVOL_S3		3			// �Ѽ�⵽���硢������������

#define DEF_WINDOW_DETECT_CONTINUOUS_BATVOL		400		// ��������ص�ѹ�Ĵ���ʱ��(��ص�ѹ��������Ϊ102�룬������ص͵�򳣵����ʱ�䴰��Ӧ�ܱ�֤������ε�ص�ѹ)

extern STATUS			sts_alarm_lowbatvol;

extern unsigned int		swtimer_window_detect_continuous_lowbatvol;	
extern unsigned int		swtimer_window_detect_continuous_nmlbatvol;

#endif

#ifdef USING_FUNC_ALARM_EXTPWROFF

#define STS_ALARM_EXTPWROFF_SNEG		-1			// �ϵ籨�����ܹر�
#define STS_ALARM_EXTPWROFF_S0		0			// �����ϵ�
#define STS_ALARM_EXTPWROFF_S1		1			// �Ѽ�⵽�ϵ�

extern STATUS			sts_alarm_extpwroff;
																	
#endif

void send_alarm_msg(char* msg);

void alarm_init(void);

#endif

