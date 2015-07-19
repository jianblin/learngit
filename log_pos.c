/*��λ����*/
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

#pragma diag_suppress 870 					// ����ĳЩ���뾯��


// ��λ�����������
T_POS_GPS 	de_gpspos;						// ���µ�GPS��λ�����������

SWITCH		sw_periodic_pos = OFF;			// ���ڶ�λģʽ�Ƿ�������
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
	// ��ʼ����λ�����������
	clr_gpspos();
}

// ���GPS��λ������ݽ������򣨳�ʼ����ÿ�ζ�λ�ɹ������ݱ�������ִ�У���
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

// ���GPS�Ƿ�λ�����ض�λ���(�Զ�����GPS��Դ���غ�GPS��λ��ʱʱ��)��
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

	/****************************************** ��Դ���� ****************************************/
	
	// gps��λǰȷ��gps��Դ�Ѿ���(gps��λ���̸����Դ��������ǿgps���ܵ��ھ���)��
	if(sts_gps_power == OFF)
	{
		gps_power_up();
	}
	
	to = to*1000;	

	printf("\r\nto wait gps fix for %d ms...\r\n", to);

	// ������Ϊ��λ�ĳ�ʱֵת��Ϊ��systickΪ��λ�ĳ�ʱֵ
	to  = systick+to/SYSTICK_PERIOD;	

	// printf("systick = %d, to = %d\r\n", systick, to);

	/************************************** ��λ���������ȡ ************************************/
	
	// ���GPS�Ƿ�λ�ɹ�(һ�㶨λ�ɹ������1HzƵ�������λ�����ͨ��GPS��λ������ڳ����Ż��У�
	// �ʼ��1-3����λ�������ʱ�����ϴ���˶���GPS��λ���Ӧ���Ż�����)��
FETCH_GPS_INFO:		
	while(systick < to)
	{
		WATCHDOG_RELOAD();

		// ��rmc��Ϣ��gsa��Ϣ��������ȡ������
		if((is_rmc_received == TRUE) && (is_gsa_received == TRUE))
		{			
			// ����RMC��Ϣ��GSA��Ϣ
			// printf("RMC=%s", gps_msg_rmc);
			// printf("GSA=%s", gps_msg_gsa);

			ret1 = gps_ana_rmc();
			ret2 = gps_ana_gsa();
			
			// ������Ϣ��ȡ�Ƿ�ɹ���������RMC��GSA��Ϣ�󶼽�������Ϣ����ȡ��־����λ(�Ӷ���GPS�����жϴ�������п����ٴμ�Ⲣ��ȡRMC��Ϣ��GSA��Ϣ)
			is_rmc_received = FALSE;
			is_gsa_received = FALSE;

			memset(gps_msg_rmc, '\0', MAX_LEN_GPS_RMC);
			memset(gps_msg_gsa, '\0', MAX_LEN_GPS_GSA);

			i++;

			// RMC��GSA��Ϣ����ֻҪ��һ�����������������Ϣ��ȡ��׼����һ����ȡ(ǰ������ȡ����δ��������)
			if((ret1 != OK) || (ret2 != OK))
			{
				// ��������������������Σ��������Խ�����Ϣ
				if(i < MAX_TIMES_ANALYZE_GPS_INFO)
				{
					printf("#%d failed to fetch complete GPS info...\r\n", i);
					
					goto FETCH_GPS_INFO;
				}
				else		// �������ν���������ֹͣ��Ϣ����
				{
					ret = NG;
					
					goto FINISH_FETCHING_GPS_INFO;	
				}
			}

			// ��ȡ��Ϣ�ɹ��󣬴�������
			j++;

			// printf("gps fixed.\r\n");
			
#if 1
			// ��ӡ��ǰ��λ���
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
			// ����ǰΪһ���Զ�λģʽ����ȡ����GPS��λ���
			if(j == 1)	// ����һ����λ���ֱ�ӿ��������Ŷ�λ�����
			{
				memcpy((unsigned char*)&best_gpspos, (unsigned char*)&de_gpspos, sizeof(T_POS_GPS));
			}
			else		// ����ǰ��λ��������Ŷ�λ�����hdopֵ�Ƚϣ�����ֵ��С(�����ȸ���)�����֮
			{
				// ��ǰGSA��Ϣ����������hdopֵΪ0��������������쳣���
				if(de_gpspos.hdop > 0.01 && de_gpspos.hdop < best_gpspos.hdop)
				{			
					memcpy((unsigned char*)&best_gpspos, (unsigned char*)&de_gpspos, sizeof(T_POS_GPS));
				}
			}				

			// �����ǰ��λ�����HDOPֵ<2���ߵ�ǰ��λ�������Ϊ3���߾��붨λ��ʱ�¼���ʣ����1�룬��ôȡ��ǰ���Ŷ�λ�����Ϊ���ζ�λ���
			// ��ʱ��λ��ɶ�ȡ��γ����ֵ����HDOP��ϢΪ�գ���ʱ���ٶ�
			if((best_gpspos.hdop < 2.0 && best_gpspos.hdop > 0.01) || j >= MAX_TIMES_FETCH_GPS_INFO || (to - systick)*SYSTICK_PERIOD<1000)			
			{
				memcpy((unsigned char*)&de_gpspos, (unsigned char*)&best_gpspos, sizeof(T_POS_GPS));

				ret = OK;

				strcpy(reply, "got fixed for oneshot pos.\r\n");

				goto FINISH_FETCHING_GPS_INFO;
			}
		}

		// ����GPS��λ�Ĺ���һ���ʱ�ϳ���Ϊ�˲�Ӱ�����ڼ���µ�SMS����Ӧ�����������������⡢���������⡢TCP�����⡢GSM�����⡢ϵͳ�¼���⣬
		// ��λ������к��ط����в���⣬��Ϊ���ǰ�ߡ�
		sys_check_event();	
		
		check_buf_com();
		check_que_sms(CHECK_MODE_ONESHOT);
		// check_que_tcp(CHECK_MODE_ONESHOT);		
		
		check_que_cmd(CHECK_MODE_CONTINUOUS);	

		// gps��λ�����в���鶨�������ź���������
	}

	ret = NG;

	strcpy(reply, "failed to get gps fixed.\r\n");

	/****************************************** ��Դ���� ****************************************/

FINISH_FETCHING_GPS_INFO:	

	// ���ڶ�λ���ڳ���60��ʱ�Źر�gps
	if(to >= 60)
	{
		gps_power_down();
	}
	
	printf("%s", reply);
		
	return ret;				
}

