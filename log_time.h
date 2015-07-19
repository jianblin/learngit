#ifndef __TIME
#define __TIME

#include "common.h"
#include "stm32f10x.h"

#define DEF_TIME_ZONE		8			// Ĭ�ϴ��ڶ�8��(������ʱ��)

// �����ַ���������������ݳ���
#define MAX_LEN_TIMEMARK	22			// "2012-02-22,18:36:23"

//ϵͳʱ������Ȼʱ���ͬ����ʽ
#define SYNC_BY_NULL			0
#define SYNC_BY_GPS			1			// ͨ��GPS��λͬ����Ȼʱ��
#define SYNC_BY_SMS			2			// ͨ��SMS����ʱ��ͬ����Ȼʱ��

// ϵͳʱ�����Ȼʱ���ͬ���㶨�塣
typedef struct
{
	unsigned char	nattime[6];			// ��Ȼʱ��(��ǰ�������α����ꡢ�¡��ա�ʱ���֡��룬���������λ����)	
	unsigned int 	systick;					// ʱ��ͬ��ʱ��ϵͳ���ʱ��
	unsigned char	synmode;			// ͬ����ʽ��0 - ͨ��GPS��λͬ����1 - ͨ��SMS���Ž���ʱ��ͬ����2 - ͨ��SMS����ͬ��
}T_SYNPOINT;			

extern SWITCH				sw_sync_by_gps;
extern SWITCH				sw_sync_by_sms;

extern T_SYNPOINT			g_sync_point;	//

extern int					g_time_zone;	// time zone of device used, e.g. E8 for Beijing


void 	time_init(void);

void 	sync_systime(unsigned char* time, unsigned char mode);
void 	tick_to_nattime(unsigned int tick, unsigned char* nattime);

int 		longitude2timezone(float longitude);

void 	delay_100ms(u16 ms100);

#endif


