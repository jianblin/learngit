#ifndef __TIME
#define __TIME

#include "common.h"
#include "stm32f10x.h"

#define DEF_TIME_ZONE		8			// 默认处于东8区(即北京时间)

// 常用字符串变量的最大数据长度
#define MAX_LEN_TIMEMARK	22			// "2012-02-22,18:36:23"

//系统时间与自然时间的同步方式
#define SYNC_BY_NULL			0
#define SYNC_BY_GPS			1			// 通过GPS定位同步自然时间
#define SYNC_BY_SMS			2			// 通过SMS发送时间同步自然时间

// 系统时间和自然时间的同步点定义。
typedef struct
{
	unsigned char	nattime[6];			// 自然时间(从前往后依次保存年、月、日、时、分、秒，年仅保留两位整数)	
	unsigned int 	systick;					// 时间同步时的系统相对时间
	unsigned char	synmode;			// 同步方式：0 - 通过GPS定位同步，1 - 通过SMS短信接收时间同步，2 - 通过SMS设置同步
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


