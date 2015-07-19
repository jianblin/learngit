#ifndef __LOG_INTERFACE
#define __LOG_INTERFACE

#include "dev_gsm.h"


#define MAX_NUM_CMD_DMN			6					// SMS/���������ַ���������(���������Ͳ���)������
#define MAX_LEN_CMD_DMN			32					// SMS/��������ÿ���������󳤶�	

#define MAX_LEN_CMD					160					// �����������󳤶ȣ�����AT+CMGR�����ԭʼ��������
														/*
															"REC UNREAD","8613923887347","","2013/07/02 01:05:30+32"
															0069006E0071002D006400650076002D0062006100740076006F006C0023
														*/

#define MAX_LEN_CMD_REPLY			400					// ��������Ļظ���Ϣ��󳤶�

#define TO_GET_IMAGE_VIA_COM			15					// ͨ���������س����ļ�ʱ�Ľ��ճ�ʱʱ��(��λ: ��)


#define LENGTH_OF_FILE_BLOCK			1024
#define LENGTH_OF_FRAME_HEADER		4					// �����ļ��ְ��İ�ͷ����

#define ERR_DESTRUCT_COMMAND_CHAOS				-1
#define ERR_DESTRUCT_COMMAND_DOMAIN_TOOLONG	-2
#define ERR_DESTRUCT_COMMAND_DOMAIN_TOOMANY	-3
#define ERR_DESTRUCT_COMMAND_NOENDER			-4

#define ERR_HANDLE_COMMAND_INVALID_PN			-1
#define ERR_HANDLE_COMMAND_INVALID_COMMAND	-2
#define ERR_HANDLE_COMMAND_PN_NO_RIGHT			-3
#define ERR_HANDLE_COMMAND_PN_NO_ENTRANCE		-4
#define ERR_HANDLE_COMMAND_NO_ITEM				-5

// ��������ַ����Ƿ�Ϸ��ĺ�(�ɴ�ӡ�ַ�)
#define IS_TXT_COMMAND_CHARACTER(byte)	((byte > 0x20 && byte < 0x7F)?TRUE:FALSE)
#define IS_DIG_COMMAND_CHARACTER(byte)	(((byte >='0' && byte <= '9') || byte == '*' || byte == '#')?TRUE:FALSE)

// ������Դ(sms��com��tcp��dtmf)������Ͳ����ĺϼƸ���������弰�����ַ����б�����ͷ����롢��������Ľ��ջ��塣
// ע: ��������������Բ�ͬ�ӿڣ����ĳЩ������Ҫ����������Դ��ͬ����Ӧ����
typedef int (*function)(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);	

//////////////////////////////////////////////////////////////////////////////////////
#define MAX_LEN_CMD_DIG				4	

#define MAX_LEN_CMD_TXT				9	

typedef struct
{
	char			cmd[MAX_LEN_CMD_TXT+1];					// �����ַ�������
	char			dtmf[MAX_LEN_CMD_DIG+1];				// �����Ӧ�����ֱ����ַ���
	function		func;									// ���������ָ��
	unsigned char	args_min;							// �����������С����(�����������)
	unsigned char	args_max;							// ���������������(�����������)
}T_CMD_TXT;

#define MAX_NUM_CMD_TXT				8	

// ȫ���ı����������ء�
#define MAX_COMMAND_TXT_NUM			28

//////////////////////////////////////////////////////////////////////////////////////

#define CMD_SRC_SMS					0x01
#define CMD_SRC_COM					0x02
#define CMD_SRC_DTMF				0x04

typedef struct
{
	unsigned char	cmd_src;							// ������Դ
	char			cmd_str[MAX_LEN_CMD+1];					// δ�ָ�����������ַ���

	char			pnf_src[MAX_LEN_PN+1];					// ����ͷ�����
	char			pnf_dst[MAX_LEN_PN+1];					// ��������������
}T_QUE_CMD;

//////////////////////////////////////////////////////////////////////////////////////

#define MAX_NUM_PN_OPERATION			4				// ÿ����ӻ�ɾ�������SMS�����ܲ���������������

ERR_NO destruct_command(char* command, int* argc, char (*argv)[MAX_LEN_CMD_DMN+1]);

#ifdef USING_DEV_GSM
/********************************************************************************************************/
#define MAX_PN_OPERATON					4				// ϵͳ�����趨�������Ȩ��������

#define STD_LEN_OF_CMD_PWD				6				// ��������̶�Ϊ6���ַ�����	

#define DEF_CMD_PWD_SMS				"123456"
#define DEF_CMD_PWD_DTMF				"999999#"		// ��������������������β����#
#endif

#define ERR_AUTH_NO_RIGHT				-1
#define ERR_AUTH_NO_ENTRANCE			-2

// ��Ȩ���뱻����Ĳ���Ȩ��(��ͬ��������Ҫ��ͬ�Ĳ���Ȩ��)
#define RIGHT_NULL						0x00			// �����κ�Ȩ��

#define RIGHT_INQUIRE						0x01			// ��ѯ
#define RIGHT_CONFIGURE					0x02			// ����
#define RIGHT_OPERATION					0x04			// ����(ָ��Ժ���Ĳ���)
#define RIGHT_CONTROL					0x08			// Ϊ�˽���ʹ�ñ���(����Ͷϵ�ȿ���)
#define RIGHT_SUPERUSER					0x80			// �����û�

#ifdef USING_DEV_GSM
extern char	cmd_pwd_sms[STD_LEN_OF_CMD_PWD+1];

extern char	pn_operation[MAX_PN_OPERATON][MAX_LEN_PN+1];

extern const char sms_pos_motion[2][3][10];

ERR_NO 	check_pn_operation(char* pnf);
/********************************************************************************************************/
// pn���		
int 		set_dtmf_password(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int 		set_sms_password(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int 		rev_sms_password(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);

int		add_pn_operation(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		clr_pn_operation(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		del_pn_operation(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		inq_pn_operation(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
#endif

int 		inq_alm_switch(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
		
//dev���		
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
//gsm���		
int		inq_gsm_connection(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		inq_gsm_imei(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		inq_gsm_signal(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		inq_gsm_swversion(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		set_gsm_apn(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		set_gsm_telecode(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		set_gsm_debug(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		set_gsm_recover(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		set_gsm_reset(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
				
//con���		
int		set_con_appserver(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		set_con_ephserver(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		set_con_mapserver(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
#endif

//sys���		
int		set_dev_reset(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
int		set_dev_shutdown(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pnf, char* reply);
		
//img���		
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
