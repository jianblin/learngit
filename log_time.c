/*时间管理*/
#include "common.h"
#include "log_time.h"
#include "log_cmd.h"
#include "log_queue.h"
#include "mcu_systick.h"

#include "stdio.h"
#include "math.h"
#include "string.h"

#define IS_LEAP_YEAR(year)	((year % 400 == 0) || (year%4 == 0 && year%100 != 0)?1:0)	// 检查指定年份是否为闰年

SWITCH				sw_sync_by_gps;					// 是否开启GPS定位时间同步的开关变量
SWITCH				sw_sync_by_sms;					// 是否开启SMS发送时间同步的开关变量

T_SYNPOINT			g_sync_point;						// 系统时间和自然时间的同步点
int 					g_time_zone;						// 终端当前所处的时区，默认为东8区

const unsigned char	days_per_month[2][12]= 
					{
						{31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},				// 普通年：365天
						{31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}				// 闰  年：366天
					};
					
const char*			weekday[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};	// 星期日为每周的第一天



// 时间管理模块初始化函数。
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
	
	// 初始化时间同步开关
	sw_sync_by_gps = ON;				// 默认打开GPS定位时间同步，以便及时同步系统时间(同步后会自动关闭)
	sw_sync_by_sms = ON;				// 默认打开SMS发送时间同步，以便及时同步系统时间(同步后会自动关闭)
		
	// 初始化时间同步信息
	g_sync_point.systick = 0;
	g_sync_point.synmode	= SYNC_BY_NULL;

	for(i = 0; i < 6; i++)
	{
		g_sync_point.nattime[i] = 0;
	}
	
	// 初始化时区
	g_time_zone = DEF_TIME_ZONE;
}

// 根据指定的年月日计算当日的星期数(星期日为每周的第一天)。
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
国际上规定，每隔15°划为一个时区，全球可分为24个时区。以本初子午线为基准，从西经7.5°到东经7.5°，
划为中时区（或叫0时区）。在中时区以东，依次划分为东一区至东十二区;在中时区以西，依次划分为西一区
至西十二区。东十二区和西十二区是同一个时区（一般称为东西十二区，以180°经线为基准，从东经172.5°
到西经172.5°！）在何时候，东十二区总比西十二区早24小时，即一天，但时间相同。
*/
// 根据经度计算所处的时区(有正负号)。
int longitude2timezone(float longitude)
{
	int timezone = 0;
	
	if(longitude <= 7.5 && longitude >= -7.5)
	{
		timezone = 0;
	}
	else if(longitude >= (180-7.5) && longitude <= 180)		// 东12区
	{
		timezone = 12;
	}
	else if(longitude <= (-180+7.5) && longitude > -180)	// 西12区
	{
		timezone = 11;
	}
	else if(longitude > 0)
	{
		timezone += ceil((longitude-7.5)/15);				// 每个时区跨越15度经度范围
	}
	else if(longitude < 0)
	{
		timezone += ceil((longitude+7.5)/15);
	}

	return (int)timezone;
}

// 通过GPS定位或短信接收同步系统时间。
void sync_systime(unsigned char* time, unsigned char mode)
{
	memcpy(g_sync_point.nattime, time, 6);

	g_sync_point.systick = systick;

	g_sync_point.synmode = mode;
}

// 将指定tick值(根据系统时间同步点)转换为自然时间()。
void tick_to_nattime(unsigned int tick, unsigned char* nattime)
{
	unsigned int 		diff;
	
	int 				carry;
	int				overflow;
	
	int				days;
	int				rest;		// 做时间减法而往前回溯时当月需减去的剩余天数

	int				year;	 
	int				month;
	int				day;
	int				hour;
	int				minute;
	int				second;

	// 指定的tick值大于或等于同步时的rtcalarm值，则做时间加法（时间前移）。
	if(tick >= g_sync_point.systick)
	{
		// 计算指定tick值和同步时rtcalarm值的差值，并转换为日-时-分-秒的格式，便于做时间加法	
		diff   = tick - g_sync_point.systick;		// 单位：ms		
		second = diff * SYSTICK_PERIOD / 1000;			// 单位：s
	
		// 在同步时的自然时间基础上根据差值做时间加法（从小级别到高级别逐一做加法）	
		// 对秒做加法
		minute = second / 60;
		second = second % 60;	
	
		second += g_sync_point.nattime[5];	// 秒
		overflow   = second / 60;
		second -= overflow * 60;
		minute += overflow;		
		
		// 对分钟做加法		
		hour    = minute / 60;
		minute  = minute % 60;
		
		minute  += g_sync_point.nattime[4];	// 分
		overflow = minute / 60;
		minute  -= overflow * 60;
		hour    += overflow;

		// 如果系统时间是通过GPS卫星定位同步的，需要根据时区校正小时数值(如果系统时间是通过SMS发送时间同步的，则不需要)
		if(g_sync_point.synmode == SYNC_BY_GPS)
		{
			// printf("g_time_zone = %d\r\n", g_time_zone);
			
			hour+= g_time_zone;					// 2012-07-06 07:59 WJC added
		}
		
		// 对小时做加法			
		day      = hour / 24;
		hour     = hour % 24;	
		
		hour    += g_sync_point.nattime[3];	// 时
		overflow = hour / 24;
		hour    -= overflow * 24;
		day     += overflow;	
		
		// 对天做加法
		day     += g_sync_point.nattime[2];	// 天	

		// 先对年和月初始化
		year    = g_sync_point.nattime[0];	// 年
		month   = g_sync_point.nattime[1];	// 月
	
		// 向后循环遍历days_per_month数组，从而实现以天为单位的时间加法（时、分、秒加法已完成）
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
	// 指定的tick值小于同步时的rtcalarm值，则做时间减法（时间回溯）。
	else
	{
		// 计算指定tick值和同步时rtcalarm值的差值，并转换为日-时-分-秒的格式，便于做时间减法	
		diff   		= g_sync_point.systick - tick;		// 单位：ms
		second 		= diff * SYSTICK_PERIOD / 1000;	// 单位：s		
	
		// 在同步时的自然时间基础上根据差值做时间减法（从低级别往高级别逐一做减法，注意借位）
		// 对秒做减法
		minute 		= second / 60;
		second 		= second % 60;									
		carry	    = (g_sync_point.nattime[5] >= second ? 0 : 1);		
		second 		= (g_sync_point.nattime[5] - second) + carry * 60;		
		minute     -= carry;
		
		// 对分钟做减法		
		hour 		= minute / 60;
		minute 		= minute % 60;
		carry	    = (g_sync_point.nattime[4] >= minute ? 0 : 1);		
		minute 		= (g_sync_point.nattime[4] - minute) + carry * 60;		
		hour       -= carry;

		// 根据时区校正小时数值
		if(g_sync_point.synmode == SYNC_BY_GPS)
		{
			// printf("g_time_zone = %d\r\n", g_time_zone);
			
			hour += g_time_zone;						// 2012-07-06 07:59 WJC added
		}
		
		// 对小时做减法		
		day 		= hour / 24;
		hour 		= hour % 24;	
		carry	    = (g_sync_point.nattime[3] >= hour ? 0 : 1);		
		hour 		= (g_sync_point.nattime[3] - hour) + carry * 24;		
		day        -=  carry;
		 	
		// 向前循环遍历days_per_month数组，从而实现以天为单位的时间剑法（时、分、秒减法已完成）
		year        = g_sync_point.nattime[0];
		month       = g_sync_point.nattime[1];
				
		rest        = g_sync_point.nattime[2];
	
		// 向前循环遍历days_per_month数组，从而实现时间减法
		do
		{
			if(day < rest)			// day小于当前rest时，直接做减法后将差值最终赋给day
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
				day -= rest;		// day等于当前rest时，继续取上个月的天数，并照常更新month、year。
				
				month--;
				
				if(month < 1)
				{
					year--;
					
					month = 12;	
				}			
				
				rest = days_per_month[IS_LEAP_YEAR(year+2000)][month-1];	
				
				if(day == 0)		// day等于前一rest时，将day最终赋值为当前rest（即当前月份的最后一天）
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

// 通过RTC Tick计数实以1mms为单位的现延时(每个rtc tick为1ms)。
void delay_100ms(u16 ms100)
{
	unsigned int limit = systick + ms100;

	while(systick < limit);
}

