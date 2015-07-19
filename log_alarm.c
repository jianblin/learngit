/*防盗报警、救援报警管理*/
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

unsigned int	swtimer_window_detect_continuous_lowbatvol;		// 低电和常电都是稳态，因此需要时间窗口检测。
unsigned int	swtimer_window_detect_continuous_nmlbatvol;
#endif

#ifdef USING_FUNC_ALARM_EXTPWROFF
STATUS			sts_alarm_extpwroff;									// 断电事件为瞬态，因此不需要时间窗口检测。																
#endif

// 和报警相关的全局开关变量和事件标志
unsigned char	sw_alarm;
unsigned char	event_alarm;

#ifdef USING_FUNC_ALARM_SOS
// SOS求救，请及时处理！
const char	alarm_info_sos[] = "\x53\x4F\x53\xE6\xB1\x82\xE6\x95\x91\xEF\xBC\x8C\xE8\xAF\xB7\xE5\x8F\x8A\xE6\x97\xB6\xE5\xA4\x84\xE7\x90\x86\xEF\xBC\x81";
#endif

// 电池电压过低，请及时给电池充电！
const char	alarm_info_lowbatvol[] = "\xE7\x94\xB5\xE6\xB1\xA0\xE7\x94\xB5\xE5\x8E\x8B\xE8\xBF\x87\xE4\xBD\x8E\xEF\xBC\x8C\xE8\xAF\xB7\xE5\x8F\x8A\xE6\x97\xB6\xE7\xBB\x99\xE7\x94\xB5\xE6\xB1\xA0\xE5\x85\x85\xE7\x94\xB5\xE3\x80\x82";	

#ifdef USING_FUNC_ALARM_EXTPWROFF
// 外部电源断开，请检查是否人为破坏或保险丝烧断。
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
	// sw_alarm 在bkp_recovery中被初始化。
	
	event_alarm = 0x00;
	
#ifdef USING_FUNC_ALARM_EXTPWROFF
	// 默认打开断电报警
	sts_alarm_extpwroff = STS_ALARM_EXTPWROFF_S0;
#endif
	
#ifdef USING_FUNC_ALARM_LOWBATVOL
	// 默认打开低电报警
	sts_alarm_lowbatvol = STS_ALARM_LOWBATVOL_S0;
#endif
}

// 发送报警消息给授权操作号码。
void send_alarm_msg(char* msg)
{
	int		i;
	
	gsm_wakeup();
		
	// 发送给授权操作号码
	for(i = 0; i < MAX_PN_OPERATON; i++)
	{		
		if(strlen((char*)pn_operation[i]) > 0)
		{
			gsm_send_sms((char*)pn_operation[i], msg, TRUE);
					
			delay_100ms(10);		// 两次发送短信操作之间应间隔1秒以上
		}
	}

	gsm_sleep();
}


// 检查电池电压偏低等系统事件并做相应处理。
void sys_check_event(void)
{		
	// printf("enter <sys_check_event>.\r\n");
	
	// 检查是否打开GPS串口转发
	if(sw_gps_forward == ON)
	{
		if(is_gps_forwarding == FALSE)
		{
			transaction_enter();

			RxCnt2_rd = 0;
			RxCnt2_wr = 0;

			// 执行GPS串口转发前应重新打开USART2 Rx中断
			USART_Configure();

			// GPS模块强行上电
			gps_power_up();

			delay_100ms(10);

			// 执行GPS串口转发
			usart2_redir_usart1();	

			transaction_quit();
		}
	}
}	 


