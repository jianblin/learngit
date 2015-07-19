#ifndef __POS
#define __POS

#include "dev_gsm.h"
#include "log_pos.h"
#include "log_queue.h" 

// Ĭ�ϵĳ�ʱʱ��(��λ: ��)
#define TO_GPSPOS_AUTO						60		// GPS��λ���̣�������λ��
#define TO_GSMPOS							10		// GSM��վ��λ����
#define TO_DOWNLOAD_IMAGE					60		// Զ�̳������

#define MAX_TIMES_ANALYZE_GPS_INFO			3		// ����GPS��λ��Ϣ��������Դ���	
#define MAX_TIMES_FETCH_GPS_INFO				3		// ��ȡGPS��λ��Ϣ��������Դ���

#define MAX_LEN_POS_LATLON					20		
#define MAX_LEN_POS_ALTITUDE					10
#define MAX_LEN_POS_ACCURACY				10
#define MAX_LEN_POS_LBS						64		// LBS��Ϣ(GSM��վ��Ϣ��WIFI AP��Ϣ)
#define MAX_LEN_POS_ADDRESS					256		// "�㶫ʡ�����и��������ﱱ����4,��������ҽԺ������64��,���������ѧУ���Ϸ�93��"

#define PRECISION_OF_LATLON					6		// ��γ�ȵ�С�����־���



#define DEF_CYCLE_PERIODIC_POS					300

#define DEF_CYCLE_PERIODIC_POS_WHEN_STILL			300			// ��ֹ״̬�¶�λ����
#define DEF_CYCLE_PERIDOIC_POS_WHEN_MOTIONING	60			// �˶�״̬�¶�λ����


// GPS��λ���
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

