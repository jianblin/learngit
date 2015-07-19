#ifndef __LOG_INTERFACE
#define __LOG_INTERFACE

#include "dev_gsm.h"


#define MAX_NUM_CMD_DMN			6					// SMS/串口命令字符串的区域(包括命令本身和参数)最多个数
#define MAX_LEN_CMD_DMN			32					// SMS/串口命令每个区域的最大长度	

#define MAX_LEN_CMD					160					// 各种命令的最大长度，包括AT+CMGR输出的原始短信命令
														/*
															"REC UNREAD","8613923887347","","2013/07/02 01:05:30+32"
															0069006E0071002D006400650076002D0062006100740076006F006C0023
														*/

#define MAX_LEN_CMD_REPLY			400					// 命令处理结果的回复消息最大长度

#define TO_GET_IMAGE_VIA_COM			15					// 通过串口下载程序文件时的接收超时时间(单位: 秒)


#define LENGTH_OF_FILE_BLOCK			1024
#define LENGTH_OF_FRAME_HEADER		4					// 程序文件分包的包头长度

#define ERR_DESTRUCT_COMMAND_CHAOS				-1
#define ERR_DESTRUCT_COMMAND_DOMAIN_TOOLONG	-2
#define ERR_DESTRUCT_COMMAND_DOMAIN_TOOMANY	-3
#define ERR_DESTRUCT_COMMAND_NOENDER			-4

#define ERR_HANDLE_COMMAND_INVALID_PN			-1
#define ERR_HANDLE_COMMAND_INVALID_COMMAND	-2
#define ERR_HANDLE_COMMAND_PN_NO_RIGHT			-3
#define ERR_HANDLE_COMMAND_PN_NO_ENTRANCE		-4
#define ERR_HANDLE_COMMAND_NO_ITEM				-5

// 检查命令字符串是否合法的宏(可打印字符)
#define IS_TXT_COMMAND_CHARACTER(byte)	((byte > 0x20 && byte < 0x7F)?TRUE:FALSE)
#define IS_DIG_COMMAND_CHARACTER(byte)	(((byte >='0' && byte <= '9') || byte == '*' || byte == '#')?TRUE:FALSE)

// 命令来源(sms、com、tcp、dtmf)、命令和参数的合计个数、命令本体及参数字符串列表、命令发送方号码、命令处理结果的接收缓冲。
// 注: 由于命令可能来自不同接口，因此某些命令需要根据命令来源不同作相应处理。
typedef int (*function)(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);	

//////////////////////////////////////////////////////////////////////////////////////
#define MAX_LEN_CMD_DIG				4	

#define MAX_LEN_CMD_TXT				9	

typedef struct
{
	char			cmd[MAX_LEN_CMD_TXT+1];					// 命令字符串本体
	char			dtmf[MAX_LEN_CMD_DIG+1];				// 命令对应的数字编码字符串
	function		func;									// 命令处理函数的指针
	unsigned char	args_min;							// 命令参数的最小个数(不包括命令本体)
	unsigned char	args_max;							// 命令参数的最大个数(不包括命令本体)
}T_CMD_TXT;

#define MAX_NUM_CMD_TXT				8	

// 全局文本命令队列相关。
#define MAX_COMMAND_TXT_NUM			28

//////////////////////////////////////////////////////////////////////////////////////

#define CMD_SRC_SMS					0x01
#define CMD_SRC_COM					0x02
#define CMD_SRC_DTMF				0x04

typedef struct
{
	unsigned char	cmd_src;							// 命令来源
	char			cmd_str[MAX_LEN_CMD+1];					// 未分割的完整命令字符串

	char			pnf_src[MAX_LEN_PN+1];					// 命令发送方号码
	char			pnf_dst[MAX_LEN_PN+1];					// 命令结果反馈号码
}T_QUE_CMD;

//////////////////////////////////////////////////////////////////////////////////////

#define MAX_NUM_PN_OPERATION			4				// 每条添加或删除号码的SMS命令能操作的做多号码个数

ERR_NO destruct_command(char* command, int* argc, char (*argv)[MAX_LEN_CMD_DMN+1]);

#ifdef USING_DEV_GSM
/********************************************************************************************************/
#define MAX_PN_OPERATON					4				// 系统允许设定的最多授权号码数量

#define STD_LEN_OF_CMD_PWD				6				// 操作密码固定为6个字符长度	

#define DEF_CMD_PWD_SMS				"123456"
#define DEF_CMD_PWD_DTMF				"999999#"		// 数字命令密码需带命令结尾符号#
#endif

#define ERR_AUTH_NO_RIGHT				-1
#define ERR_AUTH_NO_ENTRANCE			-2

// 授权号码被分配的操作权限(不同的命令需要不同的操作权限)
#define RIGHT_NULL						0x00			// 无需任何权限

#define RIGHT_INQUIRE						0x01			// 查询
#define RIGHT_CONFIGURE					0x02			// 配置
#define RIGHT_OPERATION					0x04			// 操作(指针对号码的操作)
#define RIGHT_CONTROL					0x08			// 为了将来使用保留(如断油断电等控制)
#define RIGHT_SUPERUSER					0x80			// 超级用户

#ifdef USING_DEV_GSM
extern char	cmd_pwd_sms[STD_LEN_OF_CMD_PWD+1];

extern char	pn_operation[MAX_PN_OPERATON][MAX_LEN_PN+1];

extern const char sms_pos_motion[2][3][10];

ERR_NO 	check_pn_operation(char* pnf);
/********************************************************************************************************/
// pn相关		
int 		set_dtmf_password(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int 		set_sms_password(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int 		rev_sms_password(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);

int		add_pn_operation(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		clr_pn_operation(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		del_pn_operation(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		inq_pn_operation(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
#endif

int 		inq_alm_switch(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
		
//dev相关		
int		inq_dev_batvol(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		inq_dev_hwversion(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		inq_dev_cyclicpos(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		inq_dev_powersupply(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		inq_dev_swversion(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		inq_dev_time(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int 		set_dev_factory(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int 		swt_dev_led(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply);

// gps
int 		swt_gps_dump(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int 		swt_gps_ppos(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply);

#ifdef USING_DEV_GSM
//gsm相关		
int		inq_gsm_connection(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		inq_gsm_imei(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		inq_gsm_signal(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		inq_gsm_swversion(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		set_gsm_apn(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		set_gsm_telecode(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		set_gsm_debug(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		set_gsm_recover(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		set_gsm_reset(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
				
//con相关		
int		set_con_appserver(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		set_con_ephserver(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		set_con_mapserver(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
#endif

//sys相关		
int		set_dev_reset(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		set_dev_shutdown(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
		
//img相关		
int		upd_img_com(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);

ERR_NO 	handle_cmd(unsigned char cmd_src, char* cmd_str, char* pnf_src, char* pnf_dst, char* reply);

int 		send_command_virtual(unsigned char cmd_src, char* cmd_str, char* pnf_src, char* pnf_dst, char* reply);

int 		ana_buf_com(void);

#ifdef USING_DEV_GSM
int 		ana_que_sms(void);
void 	ana_str_dtmf(char* dtmf);
void 	ana_num_ring(int rings);

void 	check_que_sms(int check_mode);
#endif

void 	check_buf_com(void);

#endif
