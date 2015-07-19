#ifndef __POS
#define __POS

#include "dev_gsm.h"
#include "log_pos.h"
#include "log_queue.h" 

// 默认的超时时间(单位: 秒)
#define TO_GPSPOS_AUTO						60		// GPS定位过程（自主定位）
#define TO_GSMPOS							10		// GSM基站定位过程
#define TO_DOWNLOAD_IMAGE					60		// 远程程序更新

#define MAX_TIMES_ANALYZE_GPS_INFO			3		// 解析GPS定位信息的最大重试次数	
#define MAX_TIMES_FETCH_GPS_INFO				3		// 提取GPS定位信息的最大重试次数

#define MAX_LEN_POS_LATLON					20		
#define MAX_LEN_POS_ALTITUDE					10
#define MAX_LEN_POS_ACCURACY				10
#define MAX_LEN_POS_LBS						64		// LBS信息(GSM基站信息或WIFI AP信息)
#define MAX_LEN_POS_ADDRESS					256		// "广东省深圳市福田区景田北三街4,福田区中医院西北方64米,福景外国语学校正南方93米"

#define PRECISION_OF_LATLON					6		// 经纬度的小数部分精度



#define DEF_CYCLE_PERIODIC_POS					300

#define DEF_CYCLE_PERIODIC_POS_WHEN_STILL			300			// 静止状态下定位周期
#define DEF_CYCLE_PERIDOIC_POS_WHEN_MOTIONING	60			// 运动状态下定位周期


// GPS定位结果
typedef struct
{		
	// fix result
	float			lat;
	float			lon;

	float			alt;
	
	float			spd;
	float			cog;
	float			hdop;

	unsigned char	utc[6];
	
	BOOL			sts;
}T_POS_GPS;								

extern T_POS_GPS	de_gpspos;	

extern SWITCH		sw_periodic_pos;
extern unsigned int 	swtimer_periodic_pos;
extern unsigned int 	cycle_periodic_pos;
extern int			sem_periodic_pos;
	
void 	pos_init(void);
int 		pos_via_gps(int to);
void 	clr_gpspos(void);
#endif

