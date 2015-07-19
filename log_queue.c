/*队列管理*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "stm32f10x.h"

#include "sms.h"

#include "dev_gps.h"
#include "dev_gsm.h"
#include "log_queue.h"
#include "log_time.h"
#include "mcu_flash.h"
#include "log_power.h"
#include "log_motion.h"
#include "log_pos.h"
#include "log_cmd.h"
#include "common.h"
#include "mcu_systick.h"
#include "mcu_gpio.h"
#include "mcu_usart.h"
#include "mcu_timer.h"

#include "stm32f10x_it.h"

// 定位请求队列相关

#ifdef USING_FUNC_CMD
#ifdef USING_DEV_GSM
// sms命令消息队列(完整的sms消息，需提取出sms命令)。
char			que_sms_cmd[MAX_NUM_SMS_RX][MAX_LEN_SMS_HEAD+MAX_LEN_CMD+1];

unsigned int 	cnt_sms_cmd_wr = 0;
unsigned int 	cnt_sms_cmd_rd = 0;
#endif

// 全局文本命令队列。
T_QUE_CMD	que_cmd[MAX_NUM_CMD_TXT];

unsigned int	cnt_cmd_wr = 0;
unsigned int	cnt_cmd_rd = 0;
#endif

void que_init(void)
{

}

#ifdef USING_FUNC_CMD
// 将一条命令记录加入到全局文本命令队列，需指定命令来源、命令字符串、命令的来源号码、命令结果的回复号码。
int que_cmd_wr(unsigned char cmd_src, char* cmd_str, char* pnf_src, char* pnf_dst)
{
	que_cmd[cnt_cmd_wr&(MAX_NUM_CMD_TXT-1)].cmd_src = cmd_src;

	strcpy(que_cmd[cnt_cmd_wr&(MAX_NUM_CMD_TXT-1)].cmd_str, cmd_str);
	strcpy(que_cmd[cnt_cmd_wr&(MAX_NUM_CMD_TXT-1)].pnf_src,  pnf_src);
	strcpy(que_cmd[cnt_cmd_wr&(MAX_NUM_CMD_TXT-1)].pnf_dst,  pnf_dst);

	cnt_cmd_wr++;

	printf("\r\ncommand inserted into queue and cnt_cmd_wr = %d\r\n", cnt_cmd_wr);	
	
	if((cnt_cmd_wr - cnt_cmd_rd) >= MAX_NUM_CMD_TXT)
	{
		printf("que_cmd[] full.\r\n");
	}	

	return OK;
}

// 根据指定检查模式检查文明命令队列有无待处理命令，有的话处理之。
void check_que_cmd(int check_mode)
{
	char	reply[MAX_LEN_CMD_REPLY+1];

	// printf("enter <check_que_cmd>.\r\n");

	strcpy(reply, "");
	
	if(check_mode == CHECK_MODE_CONTINUOUS)
	{
		while(cnt_cmd_rd < cnt_cmd_wr)
		{
			goto HANDLE_CMD_TXT;
		}
	}
	else if(check_mode == CHECK_MODE_ONESHOT)
	{
		if(cnt_cmd_rd < cnt_cmd_wr)
		{
			printf("to handle command when cnt_cmd_rd = %d, cnt_cmd_wr = %d\r\n", cnt_cmd_rd, cnt_cmd_wr);
			
HANDLE_CMD_TXT:
			handle_cmd(	que_cmd[cnt_cmd_rd&(MAX_NUM_CMD_TXT-1)].cmd_src, \
						que_cmd[cnt_cmd_rd&(MAX_NUM_CMD_TXT-1)].cmd_str, \
						que_cmd[cnt_cmd_rd&(MAX_NUM_CMD_TXT-1)].pnf_src, \
						que_cmd[cnt_cmd_rd&(MAX_NUM_CMD_TXT-1)].pnf_dst, \
						reply);

			#if 0
			printf("cmd_src: %d, cmd_str: %s, pnf_src: %s, pnf_dst: %s\r\n", 	que_cmd[cnt_cmd_rd&(MAX_NUM_CMD_TXT-1)].cmd_src, \
																	que_cmd[cnt_cmd_rd&(MAX_NUM_CMD_TXT-1)].cmd_str, \
																	que_cmd[cnt_cmd_rd&(MAX_NUM_CMD_TXT-1)].pnf_src, \
																	que_cmd[cnt_cmd_rd&(MAX_NUM_CMD_TXT-1)].pnf_dst);
			#endif
			
			// 针对不同来源的命令，采取相应的方式反馈命令处理结果。
			switch(que_cmd[cnt_cmd_rd&(MAX_NUM_CMD_TXT-1)].cmd_src)
			{
				case CMD_SRC_SMS:
#ifdef USING_DEV_GSM
					gsm_send_sms(que_cmd[cnt_cmd_rd&(MAX_NUM_CMD_TXT-1)].pnf_dst, reply);
#endif

					break;
				case CMD_SRC_DTMF:
#ifdef USING_DEV_GSM					
					// 检测到dtmf命令后延迟2秒再执行正常处理，以等待来电挂断。
					// 如果延迟过短，可能造成gsm硬件不稳定而不能正常查询imei、查询rssi等。
					delay_100ms(20);

					// printf("que_cmd[cnt_cmd_rd & (MAX_NUM_CMD_TXT-1)].pnf_dst = %s\r\n", que_cmd[cnt_cmd_rd & (MAX_NUM_CMD_TXT-1)].pnf_dst);
					// printf("que_cmd[(cnt_cmd_rd-1) & (MAX_NUM_CMD_TXT-1)].pnf_dst = %s\r\n", que_cmd[(cnt_cmd_rd-1) & (MAX_NUM_CMD_TXT-1)].pnf_dst);

					gsm_send_sms(que_cmd[cnt_cmd_rd&(MAX_NUM_CMD_TXT-1)].pnf_dst, reply);
#endif

					break;
				case CMD_SRC_COM:
					printf("%s\r\n", reply);

					break;
				default:
					break;
			}
			
			cnt_cmd_rd++;
		}
	}
}
#endif

