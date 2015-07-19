#ifndef __QUEUE
#define __QUEUE

#include "common.h"
#include "dev_gsm.h"
#include "log_time.h"

#include "mcu_flash.h"

void feedback_pos_result(unsigned char src, char* pn, char* reply);

#define CHECK_MODE_CONTINUOUS			0
#define CHECK_MODE_ONESHOT				1

#ifdef USING_FUNC_CMD
#ifdef USING_DEV_GSM
// SMS related.
#define MAX_NUM_SMS_RX					4	// 保存接收到的短信的最大条数(须为2的乘方)

extern char	que_sms_cmd[MAX_NUM_SMS_RX][MAX_LEN_SMS_HEAD+MAX_LEN_CMD+1];
extern unsigned int cnt_sms_cmd_wr;	
extern unsigned int cnt_sms_cmd_rd;	
#endif

extern T_QUE_CMD		que_cmd[MAX_NUM_CMD_TXT];
extern unsigned int	cnt_cmd_wr;
extern unsigned int	cnt_cmd_rd;

int 		que_cmd_wr(unsigned char cmd_src, char* cmd_str, char* pnf_src, char* pnf_dst);
void 	check_que_cmd(int check_mode);
#endif

void que_init(void);

#endif

