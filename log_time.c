/*ʱ�����*/
#include "common.h"
#include "log_time.h"
#include "log_cmd.h"
#include "log_queue.h"
#include "mcu_systick.h"

#include "stdio.h"
#include "math.h"
#include "string.h"

#define IS_LEAP_YEAR(year)	((year % 400 == 0) || (year%4 == 0 && year%100 != 0)?1:0)	// ���ָ������Ƿ�Ϊ����

SWITCH				sw_sync_by_gps;					// �Ƿ���GPS��λʱ��ͬ���Ŀ��ر���
SWITCH				sw_sync_by_sms;					// �Ƿ���SMS����ʱ��ͬ���Ŀ��ر���

T_SYNPOINT			g_sync_point;						// ϵͳʱ�����Ȼʱ���ͬ����
int 					g_time_zone;						// �ն˵�ǰ������ʱ����Ĭ��Ϊ��8��

const unsigned char	days_per_month[2][12]= 
					{
						{31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},				// ��ͨ�꣺365��
						{31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}				// ��  �꣺366��
					};
					
const char*			weekday[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};	// ������Ϊÿ�ܵĵ�һ��



// ʱ�����ģ���ʼ��������
/********************************************************************************************************************
*
*
*
*
*********************************************************************************************************************/
void time_init(void)
{
	int i;

	// printf("\r\nto initialize time module.\r\n");
	
	// ��ʼ��ʱ��ͬ������
	sw_sync_by_gps = ON;				// Ĭ�ϴ�GPS��λʱ��ͬ�����Ա㼰ʱͬ��ϵͳʱ��(ͬ������Զ��ر�)
	sw_sync_by_sms = ON;				// Ĭ�ϴ�SMS����ʱ��ͬ�����Ա㼰ʱͬ��ϵͳʱ��(ͬ������Զ��ر�)
		
	// ��ʼ��ʱ��ͬ����Ϣ
	g_sync_point.systick = 0;
	g_sync_point.synmode	= SYNC_BY_NULL;

	for(i = 0; i < 6; i++)
	{
		g_sync_point.nattime[i] = 0;
	}
	
	// ��ʼ��ʱ��
	g_time_zone = DEF_TIME_ZONE;
}

// ����ָ���������ռ��㵱�յ�������(������Ϊÿ�ܵĵ�һ��)��
short get_weekday(short year, short month, short day)
{
	short week  = 0;
	
	week = year > 0 ? (5 + (year + 1) + (year - 1)/4 - (year - 1)/100 + (year - 1)/400) % 7 : (5 + year + year/4 - year/100 + year/400) % 7;
	
	week = month > 2 ? (week + 2*(month + 1) + 3*(month + 1)/5) % 7 : (week + 2*(month + 2) + 3*(month + 2)/5) % 7;
	
	if (((year%4 == 0 && year%100 != 0) || year%400 == 0) && month>2)
	{
		week = (week + 1) % 7;
	}
	
	week = (week + day) % 7;

	return week;
}

/*
�����Ϲ涨��ÿ��15�㻮Ϊһ��ʱ����ȫ��ɷ�Ϊ24��ʱ�����Ա���������Ϊ��׼��������7.5�㵽����7.5�㣬
��Ϊ��ʱ�������0ʱ����������ʱ���Զ������λ���Ϊ��һ������ʮ����;����ʱ�����������λ���Ϊ��һ��
����ʮ��������ʮ��������ʮ������ͬһ��ʱ����һ���Ϊ����ʮ��������180�㾭��Ϊ��׼���Ӷ���172.5��
������172.5�㣡���ں�ʱ�򣬶�ʮ�����ܱ���ʮ������24Сʱ����һ�죬��ʱ����ͬ��
*/
// ���ݾ��ȼ���������ʱ��(��������)��
int longitude2timezone(float longitude)
{
	int timezone = 0;
	
	if(longitude <= 7.5 && longitude >= -7.5)
	{
		timezone = 0;
	}
	else if(longitude >= (180-7.5) && longitude <= 180)		// ��12��
	{
		timezone = 12;
	}
	else if(longitude <= (-180+7.5) && longitude > -180)	// ��12��
	{
		timezone = 11;
	}
	else if(longitude > 0)
	{
		timezone += ceil((longitude-7.5)/15);				// ÿ��ʱ����Խ15�Ⱦ��ȷ�Χ
	}
	else if(longitude < 0)
	{
		timezone += ceil((longitude+7.5)/15);
	}

	return (int)timezone;
}

// ͨ��GPS��λ����Ž���ͬ��ϵͳʱ�䡣
void sync_systime(unsigned char* time, unsigned char mode)
{
	memcpy(g_sync_point.nattime, time, 6);

	g_sync_point.systick = systick;

	g_sync_point.synmode = mode;
}

// ��ָ��tickֵ(����ϵͳʱ��ͬ����)ת��Ϊ��Ȼʱ��()��
void tick_to_nattime(unsigned int tick, unsigned char* nattime)
{
	unsigned int 		diff;
	
	int 				carry;
	int				overflow;
	
	int				days;
	int				rest;		// ��ʱ���������ǰ����ʱ�������ȥ��ʣ������

	int				year;	 
	int				month;
	int				day;
	int				hour;
	int				minute;
	int				second;

	// ָ����tickֵ���ڻ����ͬ��ʱ��rtcalarmֵ������ʱ��ӷ���ʱ��ǰ�ƣ���
	if(tick >= g_sync_point.systick)
	{
		// ����ָ��tickֵ��ͬ��ʱrtcalarmֵ�Ĳ�ֵ����ת��Ϊ��-ʱ-��-��ĸ�ʽ��������ʱ��ӷ�	
		diff   = tick - g_sync_point.systick;		// ��λ��ms		
		second = diff * SYSTICK_PERIOD / 1000;			// ��λ��s
	
		// ��ͬ��ʱ����Ȼʱ������ϸ��ݲ�ֵ��ʱ��ӷ�����С���𵽸߼�����һ���ӷ���	
		// �������ӷ�
		minute = second / 60;
		second = second % 60;	
	
		second += g_sync_point.nattime[5];	// ��
		overflow   = second / 60;
		second -= overflow * 60;
		minute += overflow;		
		
		// �Է������ӷ�		
		hour    = minute / 60;
		minute  = minute % 60;
		
		minute  += g_sync_point.nattime[4];	// ��
		overflow = minute / 60;
		minute  -= overflow * 60;
		hour    += overflow;

		// ���ϵͳʱ����ͨ��GPS���Ƕ�λͬ���ģ���Ҫ����ʱ��У��Сʱ��ֵ(���ϵͳʱ����ͨ��SMS����ʱ��ͬ���ģ�����Ҫ)
		if(g_sync_point.synmode == SYNC_BY_GPS)
		{
			// printf("g_time_zone = %d\r\n", g_time_zone);
			
			hour+= g_time_zone;					// 2012-07-06 07:59 WJC added
		}
		
		// ��Сʱ���ӷ�			
		day      = hour / 24;
		hour     = hour % 24;	
		
		hour    += g_sync_point.nattime[3];	// ʱ
		overflow = hour / 24;
		hour    -= overflow * 24;
		day     += overflow;	
		
		// �������ӷ�
		day     += g_sync_point.nattime[2];	// ��	

		// �ȶ�����³�ʼ��
		year    = g_sync_point.nattime[0];	// ��
		month   = g_sync_point.nattime[1];	// ��
	
		// ���ѭ������days_per_month���飬�Ӷ�ʵ������Ϊ��λ��ʱ��ӷ���ʱ���֡���ӷ�����ɣ�
		while(1)
		{
			days = days_per_month[IS_LEAP_YEAR(year+2000)][month-1];
			
			if(day <= days)
			{
				nattime[5]	= (short)second;
				nattime[4] 	= (short)minute;
				nattime[3]  = (short)hour;
				nattime[2]  = (short)day;
				nattime[1]  = (short)month;
				nattime[0]  = (short)year;
				
				break;		
			}
			else
			{
				day -= days;
				
				month++;
				
				if(month > 12)
				{
					year++;
					
					month = 1;	
				}	
			}
		}
	}
	// ָ����tickֵС��ͬ��ʱ��rtcalarmֵ������ʱ�������ʱ����ݣ���
	else
	{
		// ����ָ��tickֵ��ͬ��ʱrtcalarmֵ�Ĳ�ֵ����ת��Ϊ��-ʱ-��-��ĸ�ʽ��������ʱ�����	
		diff   		= g_sync_point.systick - tick;		// ��λ��ms
		second 		= diff * SYSTICK_PERIOD / 1000;	// ��λ��s		
	
		// ��ͬ��ʱ����Ȼʱ������ϸ��ݲ�ֵ��ʱ��������ӵͼ������߼�����һ��������ע���λ��
		// ����������
		minute 		= second / 60;
		second 		= second % 60;									
		carry	    = (g_sync_point.nattime[5] >= second ? 0 : 1);		
		second 		= (g_sync_point.nattime[5] - second) + carry * 60;		
		minute     -= carry;
		
		// �Է���������		
		hour 		= minute / 60;
		minute 		= minute % 60;
		carry	    = (g_sync_point.nattime[4] >= minute ? 0 : 1);		
		minute 		= (g_sync_point.nattime[4] - minute) + carry * 60;		
		hour       -= carry;

		// ����ʱ��У��Сʱ��ֵ
		if(g_sync_point.synmode == SYNC_BY_GPS)
		{
			// printf("g_time_zone = %d\r\n", g_time_zone);
			
			hour += g_time_zone;						// 2012-07-06 07:59 WJC added
		}
		
		// ��Сʱ������		
		day 		= hour / 24;
		hour 		= hour % 24;	
		carry	    = (g_sync_point.nattime[3] >= hour ? 0 : 1);		
		hour 		= (g_sync_point.nattime[3] - hour) + carry * 24;		
		day        -=  carry;
		 	
		// ��ǰѭ������days_per_month���飬�Ӷ�ʵ������Ϊ��λ��ʱ�佣����ʱ���֡����������ɣ�
		year        = g_sync_point.nattime[0];
		month       = g_sync_point.nattime[1];
				
		rest        = g_sync_point.nattime[2];
	
		// ��ǰѭ������days_per_month���飬�Ӷ�ʵ��ʱ�����
		do
		{
			if(day < rest)			// dayС�ڵ�ǰrestʱ��ֱ���������󽫲�ֵ���ո���day
			{
				day = rest - day;
				
				nattime[5]	= (short)second;
				nattime[4] 	= (short)minute;
				nattime[3]  = (short)hour;
				nattime[2]  = (short)day;
				nattime[1]  = (short)month;
				nattime[0]  = (short)year;
				
				break;		
			}	
			else
			{
				day -= rest;		// day���ڵ�ǰrestʱ������ȡ�ϸ��µ����������ճ�����month��year��
				
				month--;
				
				if(month < 1)
				{
					year--;
					
					month = 12;	
				}			
				
				rest = days_per_month[IS_LEAP_YEAR(year+2000)][month-1];	
				
				if(day == 0)		// day����ǰһrestʱ����day���ո�ֵΪ��ǰrest������ǰ�·ݵ����һ�죩
				{
					day = rest;
					
					nattime[5]	= (short)second;
					nattime[4] 	= (short)minute;
					nattime[3]  = (short)hour;
					nattime[2]  = (short)day;
					nattime[1]  = (short)month;
					nattime[0]  = (short)year;	
					
					break;			
				}	
			}		
		}while(1);
	}

}

// ͨ��RTC Tick����ʵ��1mmsΪ��λ������ʱ(ÿ��rtc tickΪ1ms)��
void delay_100ms(u16 ms100)
{
	unsigned int limit = systick + ms100;

	while(systick < limit);
}

