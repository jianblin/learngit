/*定位管理*/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "stm32f10x.h"
#include "stm32f10x_it.h"

#include "common.h"
#include "log_time.h"
#include "mcu_usart.h"
#include "log_motion.h"
#include "log_cmd.h"
#include "log_pos.h"
#include "dev_gps.h"
#include "dev_gsm.h"
#include "mcu_systick.h"
#include "mcu_usart.h"
#include "mcu_gpio.h"
#include "mcu_timer.h"
#include "mcu_flash.h"
#include "log_power.h"

#pragma diag_suppress 870 					// 抑制某些编译警告


// 定位结果交换区域。
T_POS_GPS 	de_gpspos;						// 最新的GPS定位结果保存区域

SWITCH		sw_periodic_pos = OFF;			// 周期定位模式是否开启开关
unsigned int 	swtimer_periodic_pos;
unsigned int 	cycle_periodic_pos 	= DEF_CYCLE_PERIODIC_POS;		// unit: second
int			sem_periodic_pos 	= 10;

/********************************************************************************************************************
*
*
*
*
*********************************************************************************************************************/
void pos_init(void)
{	
	// 初始化定位结果交换区域
	clr_gpspos();
}

// 清空GPS定位结果数据交换区域（初始化和每次定位成功且数据被拷出后执行）。
void clr_gpspos(void)
{
	de_gpspos.lat 	= 0.0;
	de_gpspos.lon 	= 0.0;
	de_gpspos.alt 	= 0.0;
	de_gpspos.spd 	= 0.0;
	de_gpspos.cog 	= 0.0;
	de_gpspos.hdop	= 0.0;

	memset(de_gpspos.utc, 0x00, 6);

	de_gpspos.sts = FALSE;
}

// 检查GPS是否定位并返回定位结果(自动管理GPS电源开关和GPS定位超时时间)。
int pos_via_gps(int to)
{
	// unsigned int 	to;

	T_POS_GPS	 	best_gpspos;
	
	char				reply[128];
	
	int			 	i = 0;
	int				j = 0;

	int				ret1 = NG;
	int				ret2 = NG;

	int				ret = NG;	

	/****************************************** 电源管理 ****************************************/
	
	// gps定位前确保gps电源已经打开(gps定位过程负责电源管理，以增强gps功能的内聚性)。
	if(sts_gps_power == OFF)
	{
		gps_power_up();
	}
	
	to = to*1000;	

	printf("\r\nto wait gps fix for %d ms...\r\n", to);

	// 将以秒为单位的超时值转化为以systick为单位的超时值
	to  = systick+to/SYSTICK_PERIOD;	

	// printf("systick = %d, to = %d\r\n", systick, to);

	/************************************** 定位结果检测和提取 ************************************/
	
	// 检查GPS是否定位成功(一般定位成功后会以1Hz频率输出定位结果，通常GPS定位结果处于持续优化中，
	// 最开始的1-3个定位结果多数时候误差较大，因此对于GPS定位结果应做优化处理)。
FETCH_GPS_INFO:		
	while(systick < to)
	{
		WATCHDOG_RELOAD();

		// 若rmc消息和gsa消息都被就提取过，则
		if((is_rmc_received == TRUE) && (is_gsa_received == TRUE))
		{			
			// 解析RMC消息和GSA消息
			// printf("RMC=%s", gps_msg_rmc);
			// printf("GSA=%s", gps_msg_gsa);

			ret1 = gps_ana_rmc();
			ret2 = gps_ana_gsa();
			
			// 不管信息提取是否成功，解析完RMC和GSA消息后都将两种消息的提取标志都复位(从而在GPS串口中断处理程序中可以再次检测并提取RMC消息和GSA消息)
			is_rmc_received = FALSE;
			is_gsa_received = FALSE;

			memset(gps_msg_rmc, '\0', MAX_LEN_GPS_RMC);
			memset(gps_msg_gsa, '\0', MAX_LEN_GPS_GSA);

			i++;

			// RMC和GSA消息解析只要有一个出错，则放弃当次信息提取，准备下一次提取(前提是提取次数未超过上限)
			if((ret1 != OK) || (ret2 != OK))
			{
				// 解析出错次数不超过三次，继续尝试解析消息
				if(i < MAX_TIMES_ANALYZE_GPS_INFO)
				{
					printf("#%d failed to fetch complete GPS info...\r\n", i);
					
					goto FETCH_GPS_INFO;
				}
				else		// 连续三次解析出错，则停止消息解析
				{
					ret = NG;
					
					goto FINISH_FETCHING_GPS_INFO;	
				}
			}

			// 提取消息成功后，次数递增
			j++;

			// printf("gps fixed.\r\n");
			
#if 1
			// 打印当前定位结果
			printf("\r\n----------------------------------------------------------\r\n");
			printf("#%d GPS fix result:\r\n", 		j);		
			printf("de_gpspos.lat = %3.6f\r\n",  	de_gpspos.lat);
			printf("de_gpspos.lon = %3.6f\r\n",  	de_gpspos.lon);
			printf("de_gpspos.alt = %5.2f\r\n",  	de_gpspos.alt);
			printf("de_gpspos.spd = %3.2f\r\n",  	de_gpspos.spd);
			printf("de_gpspos.cog = %3.2f\r\n",  	de_gpspos.cog);
			printf("de_gpspos.hdop= %3.2f\r\n",  de_gpspos.hdop);
			printf("----------------------------------------------------------\r\n");
#endif	
			// 若当前为一次性定位模式，则取最优GPS定位结果
			if(j == 1)	// 将第一个定位结果直接拷贝到最优定位结果中
			{
				memcpy((unsigned char*)&best_gpspos, (unsigned char*)&de_gpspos, sizeof(T_POS_GPS));
			}
			else		// 将当前定位结果和最优定位结果做hdop值比较，若数值更小(即精度更高)则替代之
			{
				// 当前GSA消息若解析错误，hdop值为0，因此需检测这种异常情况
				if(de_gpspos.hdop > 0.01 && de_gpspos.hdop < best_gpspos.hdop)
				{			
					memcpy((unsigned char*)&best_gpspos, (unsigned char*)&de_gpspos, sizeof(T_POS_GPS));
				}
			}				

			// 如果当前定位结果的HDOP值<2或者当前定位结果次数为3或者距离定位超时事件所剩不足1秒，那么取当前最优定位结果作为本次定位结果
			// 有时定位后可读取经纬度数值但是HDOP信息为空，此时宜再读
			if((best_gpspos.hdop < 2.0 && best_gpspos.hdop > 0.01) || j >= MAX_TIMES_FETCH_GPS_INFO || (to - systick)*SYSTICK_PERIOD<1000)			
			{
				memcpy((unsigned char*)&de_gpspos, (unsigned char*)&best_gpspos, sizeof(T_POS_GPS));

				ret = OK;

				strcpy(reply, "got fixed for oneshot pos.\r\n");

				goto FINISH_FETCHING_GPS_INFO;
			}
		}

		// 由于GPS定位的过程一般耗时较长，为了不影响这期间对新到SMS的响应，这里插入短信命令检测、串口命令检测、TCP命令检测、GSM振铃检测、系统事件检测，
		// 定位请求队列和重发队列不检测，因为检测前者。
		sys_check_event();	
		
		check_buf_com();
		check_que_sms(CHECK_MODE_ONESHOT);
		// check_que_tcp(CHECK_MODE_ONESHOT);		
		
		check_que_cmd(CHECK_MODE_CONTINUOUS);	

		// gps定位过程中不检查定期请求信号量。。。
	}

	ret = NG;

	strcpy(reply, "failed to get gps fixed.\r\n");

	/****************************************** 电源管理 ****************************************/

FINISH_FETCHING_GPS_INFO:	

	// 尽在定位周期超过60秒时才关闭gps
	if(to >= 60)
	{
		gps_power_down();
	}
	
	printf("%s", reply);
		
	return ret;				
}

