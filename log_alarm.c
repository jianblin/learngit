/*������������Ԯ��������*/
#include "log_pos.h"
#include "log_motion.h"
#include "log_alarm.h"
#include "protocol.h"

#include "dev_gps.h"

#include "mcu_usart.h"
#include "mcu_systick.h"
#include <string.h>

#ifdef USING_FUNC_ALARM_LOWBATVOL
STATUS			sts_alarm_lowbatvol;

unsigned int	swtimer_window_detect_continuous_lowbatvol;		// �͵�ͳ��綼����̬�������Ҫʱ�䴰�ڼ�⡣
unsigned int	swtimer_window_detect_continuous_nmlbatvol;
#endif

#ifdef USING_FUNC_ALARM_EXTPWROFF
STATUS			sts_alarm_extpwroff;									// �ϵ��¼�Ϊ˲̬����˲���Ҫʱ�䴰�ڼ�⡣																
#endif

// �ͱ�����ص�ȫ�ֿ��ر������¼���־
unsigned char	sw_alarm;
unsigned char	event_alarm;

#ifdef USING_FUNC_ALARM_SOS
// SOS��ȣ��뼰ʱ����
const char	alarm_info_sos[] = "\x53\x4F\x53\xE6\xB1\x82\xE6\x95\x91\xEF\xBC\x8C\xE8\xAF\xB7\xE5\x8F\x8A\xE6\x97\xB6\xE5\xA4\x84\xE7\x90\x86\xEF\xBC\x81";
#endif

// ��ص�ѹ���ͣ��뼰ʱ����س�磡
const char	alarm_info_lowbatvol[] = "\xE7\x94\xB5\xE6\xB1\xA0\xE7\x94\xB5\xE5\x8E\x8B\xE8\xBF\x87\xE4\xBD\x8E\xEF\xBC\x8C\xE8\xAF\xB7\xE5\x8F\x8A\xE6\x97\xB6\xE7\xBB\x99\xE7\x94\xB5\xE6\xB1\xA0\xE5\x85\x85\xE7\x94\xB5\xE3\x80\x82";	

#ifdef USING_FUNC_ALARM_EXTPWROFF
// �ⲿ��Դ�Ͽ��������Ƿ���Ϊ�ƻ�����˿�նϡ�
const char	alarm_info_extpwroff[] = "\xE5\xA4\x96\xE9\x83\xA8\xE7\x94\xB5\xE6\xBA\x90\xE6\x96\xAD\xE5\xBC\x80\xEF\xBC\x8C\xE8\xAF\xB7\xE6\xA3\x80\xE6\x9F\xA5\xE6\x98\xAF\xE5\x90\xA6\xE4\xBA\xBA\xE4\xB8\xBA\xE7\xA0\xB4\xE5\x9D\x8F\xE6\x88\x96\xE4\xBF\x9D\xE9\x99\xA9\xE4\xB8\x9D\xE7\x83\xA7\xE6\x96\xAD\xE3\x80\x82";
#endif

/********************************************************************************************************************
*
*
*
*
*********************************************************************************************************************/
void alarm_init()
{
	// sw_alarm ��bkp_recovery�б���ʼ����
	
	event_alarm = 0x00;
	
#ifdef USING_FUNC_ALARM_EXTPWROFF
	// Ĭ�ϴ򿪶ϵ籨��
	sts_alarm_extpwroff = STS_ALARM_EXTPWROFF_S0;
#endif
	
#ifdef USING_FUNC_ALARM_LOWBATVOL
	// Ĭ�ϴ򿪵͵籨��
	sts_alarm_lowbatvol = STS_ALARM_LOWBATVOL_S0;
#endif
}

// ���ͱ�����Ϣ����Ȩ�������롣
void send_alarm_msg(char* msg)
{
	int		i;
	
	gsm_wakeup();
		
	// ���͸���Ȩ��������
	for(i = 0; i < MAX_PN_OPERATON; i++)
	{		
		if(strlen((char*)pn_operation[i]) > 0)
		{
			gsm_send_sms((char*)pn_operation[i], msg, TRUE);
					
			delay_100ms(10);		// ���η��Ͷ��Ų���֮��Ӧ���1������
		}
	}

	gsm_sleep();
}


// ����ص�ѹƫ�͵�ϵͳ�¼�������Ӧ����
void sys_check_event(void)
{		
	// printf("enter <sys_check_event>.\r\n");
	
	// ����Ƿ��GPS����ת��
	if(sw_gps_forward == ON)
	{
		if(is_gps_forwarding == FALSE)
		{
			transaction_enter();

			RxCnt2_rd = 0;
			RxCnt2_wr = 0;

			// ִ��GPS����ת��ǰӦ���´�USART2 Rx�ж�
			USART_Configure();

			// GPSģ��ǿ���ϵ�
			gps_power_up();

			delay_100ms(10);

			// ִ��GPS����ת��
			usart2_redir_usart1();	

			transaction_quit();
		}
	}
}	 


