#ifndef __GSM
#define __GSM

// sms
#define MAX_LEN_SMS_TXTIME					32		// sms���ͷ����͵�ʱ�䣬����������ʱ�����Լ�ʱ����Ϣ

#define MAX_BYTE_SMS_PDU					140		// PDU SMS ��󳤶ȣ�M660+/M660_1120_D7S13000_V009: 140�ֽڣ����Ǹ���GSMЭ�鵥��������󳤶�Ϊ144�ֽڣ��й��ƶ�/�й���ͨ��70�����֣�
#define MAX_BYTE_SMS_TXT						140		// TXT SMS ��󳤶ȣ�M660+/M660_1120_D7S13000_V009: 159�ֽڣ����Ǹ���GSMЭ�鵥��������󳤶�Ϊ144�ֽڣ��й��ƶ�/�й���ͨ��70�����֣�
													// ���ڲ���7bit���뷢��Ӣ�Ķ���ʱ�Է���������ĩλ�ַ����룬��˸���8bit���롣

#ifdef USING_DEV_GSM

#include "common.h"
#include "log_cmd.h"
#include "stm32f10x.h"

// gsm global
#define MAX_LEN_GSM_TELECODE				8		// ���Ҵ������󳤶�
#define STD_LEN_GSM_IMEI						15		// gsm imei�ı�׼����

#define TO_GSM_REGISTRATION					15		// GSM�ϵ��ע�ᵽ����ĳ�ʱʱ��(һ��Ϊ10������)

#define MIN_RSSI_FOR_COMMUNICATION			5		// ά��gsm����ͨѶ��������rssiֵ
														
// �������ŵĲ�������(���������Ļ���Ӣ�ģ����������ŵ���󳤶�ΪMAX_LEN_CNC_SMS*MAX_NUM_CNC_SMS����˵������������ɷ���
// MAX_LEN_CNC_SMS*MAX_NUM_CNC_SMS*8/7��Ӣ���ַ����Զ�ɷ���MAX_LEN_CNC_SMS*MAX_NUM_CNC_SMS/2�������ַ�)
#define MAX_CHAR_CNC_SMS_CN					67						// �����������ſɷ��͵���������ַ���(����USC2���룬ÿ������ռ��2�ֽ�)
#define MAX_CHAR_CNC_SMS_EN					(MAX_CHAR_CNC_SMS_CN*2)	// �����������ſɷ��͵����Ӣ���ַ���(����8bit���룬ÿ����ĸռ��1�ֽ�)

#define MAX_LEN_CNC_SMS						(MAX_CHAR_CNC_SMS_CN*2)	// ÿ���������ŵ���󳤶�
#define MAX_NUM_CNC_SMS					4						// ϵͳ֧�ֵĵ����������������(������RAM�ռ�)

#define MAX_LEN_SMS_HEAD					60		// +CMGR:"REC UNREAD","13430981508","","2012/09/08 16:30:08+32"
#define DEF_LEN_SMS_UDH						7		// ���ͳ�����ʱUDH�Ĺ̶�����

#define TO_SMS_TX							10		// RSSI=15����ʱ�����ŷ���ʱ��ԼΪ8s

// gprs
#define MAX_LEN_GSM_APN						64		// gprs apn�����й��ƶ�Ϊ"cmnet"
#define MAX_LEN_GPRS_IP						20		// ip��ַ����󳤶ȣ���"128.128.128.128"
#define MAX_LEN_GPRS_DN						32		// ��������󳤶ȣ���"www.google.com"
#define MAX_LEN_GPRS_PORT					8		// �˿ڵ���󳤶ȣ���"8080"

#define TO_GPRS_DO_DNS						10		// ��������
#define TO_GPRS_SET_APN						2		// ����APN
#define TO_GPRS_SETUP_CONNECTION			10		// ����GPRS����
#define TO_GPRS_CLOSE_CONNECTION			2		// �Ͽ�GPRS����												
#define TO_GPRS_TRANSPARENT_ENTER			2		// ��͸��ģʽ
#define TO_GPRS_TRANSPARENT_EXIT				2		// �ر�͸��ģʽ
#define TO_GPRS_TX_FRAME						2		// ����һ֡����
#define TO_GPRS_RX_FRAME						15		// ����һ֡����(ʵ����Ե�֡��󳤶�Ϊ1500�ֽ�)

#define TO_GSM_DOWNLOAD_MODE				120		// ����gsmģ��̼��ĳ�ʱʱ��
#define TO_GET_IMAGE_VIA_TCP					90		// Զ�����س����ļ��ĳ�ʱʱ��

#define MAX_LEN_GPRS_FRAMERX				1428	// GSMģ����յ�GPRS���ݺ�Ӵ�������ĵ������ݰ���󳤶ȣ����ͷ�Ӧ�����ݷָ���ٷ��ͣ�������շ����ճ���
														// ������Ϊ1500�ֽڣ�������ʱ���1500ʱ����쳣��"data length over 1500 bytes,exception!"��

#define MAX_TIMES_GPRS_SEND_PACKET			1		// ����AT+SOCSEND�����GPRS����֡ʱ������ط�����(��Ϊ1�Σ����ⷢ�ͳ�����ʱĳ֡����ʧ�ܺ��ط����½��շ��ظ����ո�֡)
#define MAX_TIMES_GPRS_SEND_DATA			1		// ����ָ������(������)��������Դ���
#define MAX_TIMES_GPRS_SETUP_CONNECTION		2		// ����GPRS���ӵ�������Դ���

#define MAX_TIMES_GSM_GET_CELLID				2		// ��ѯgsm��վ��Ϣ��������
#define MAX_TIEMS_GSM_GET_RSSI				2		// ȷ��rssi�Ƿ�����ʱ��ѯrssi��������

#define MAX_TIMES_GSM_RECOVER				3		// �����ָ�gsm��������

#define MAX_LEN_GPRS_PACKET_SEND			1392		// AT+SOCSEND����η��͵�������ݳ��ȣ�MMA3328��496�ֽڣ�

// GPRS ID
#define MAX_NUM_TCP_CONN					4		// ϵͳ���õ�gprs��������ַ(GSMģ��֧�����3��GPRS��������)

#define	GPRS_ID0							0
#define	GPRS_ID1							1
#define	GPRS_ID2							2

// dtmf
#define TO_DTMF_INPUT_PASSWORD				15		// ��dtmf�����û�����dtmf����ĳ�ʱʱ��
#define DELAY_SET_MICMUTE_OFF				1		// dtmf����������ȷ�󵽹ر���ͷ����֮����ӳ�ʱ��
#define MAX_LEN_GSM_DTMF					64		// dtmf�ַ�������󳤶�(�������������)

// cell-id
#define MAX_NUM_GSM_CELL					7		// AT+POSI��ѯ������վ����
#define MAX_NUM_GSM_CELL_DMN				7		// gsm��վ��Ϣ����������
#define MAX_LEN_GSM_CELL_DMN				6		// gsm��վ��Ϣ�����󳤶�

// server
#define MAX_LEN_EPHSERVER_UASER				32
#define MAX_LEN_EPHSERVER_PASSWORD			16

// error code
// ���������(��GSMģ���޹�)
#define ER_GSM_PATTERN_NULL					0		// �����ģʽ�ַ�������Ϊ��

#define ER_GSM_PN_TOOLONG					-1
#define ER_GSM_PN_TOOSHORT					-2
#define ER_GSM_PN_CHAOS						-3		// pn�г��ַ������ַ�

#define ER_GSM_SMS_PDU_CHAOS				-4

// �ӿ���ش���(һ����GSMģ���������)
#define ER_GSM_UART_RECV_NOTHING			-10		// gsm���������(һ��Ϊ����Ӳ������)
#define ER_GSM_UART_RECV_TIMEOUT			-11		// gsm���ڽ��ճ�ʱ(�������������ָ��ʱ���ڽ��ղ����������ַ����ַ���)
#define ER_GSM_UART_RECV_CHAOS				-12		// gsm���ڽ��յ�����

// ������ش���(������GSMģ��������������)
#define ER_GSM_NETWORK_UNREGISTERED		-13		

#define ER_GSM_RSSI_UNKNOW					-15		// rssi = 99
#define ER_GSM_RSSI_TOOLOW					-16		// rssi < 4

#define ER_GSM_GPRS_LINK_TIMEOUT				-17		// gprs���ӳ�ʱ
#define ER_GSM_GPRS_SEND_TIMEOUT			-18		// gprs���ͳ�ʱ
#define ER_GSM_GPRS_RECV_TIMEOUT			-19		// gprs���ճ�ʱ

// GSM��ʼ������
#define ER_GSM_INIT_SIM						-20		// SIM��Ϊ��⵽
#define ER_GSM_INIT_REGISTRATION				-21		// ע������ʧ��
#define ER_GSM_INIT_SWVERSION				-22		// ��ѯ�̼��汾ʧ��
#define ER_GSM_INIT_IMEI1						-23		// ��ѯimeiʧ��
#define ER_GSM_INIT_IMEI2						-24		// ��ѯ�õ���imei���Ȳ���
#define ER_GSM_INIT_SMS_MODE					-25		// ����smsģʽΪtextʧ��
#define ER_GSM_INIT_SMS_SETTING				-26		// ����smsflag-ringʧ��
#define ER_GSM_INIT_SMS_DELETE				-27		// ɾ��smsʧ��
#define ER_GSM_INIT_CLIP						-28		// ������������Զ����ʧ��
#define ER_GSM_INIT_DATAFORMAT				-29		// ����tcp���ݽ��մ���ʧ��
#define ER_GSM_INIT_RSSI						-30		// ��ѯrssiʧ��

#define ER_GSM_RECOVER_FAILURE				-14		// gsm�ָ�ʧ��

// �ṹ�嶨��
typedef struct
{
	char				dn[MAX_LEN_GPRS_DN+1];			// Domain Name
	char				ip[MAX_LEN_GPRS_IP+1];			// IP Address
	char				port[MAX_LEN_GPRS_PORT+1];		// Port
}T_GPRS_CONNECTION;	// ����GPRS���������Ľṹ��

typedef struct
{
	char	mcc[16];	
	char	mnc[16];
	char	lac[16];
	char	cellid[16];
	char	rssi[4];
}T_INFO_GSMCELL;			// GSM��վ��Ϣ

// for future extension
typedef struct
{
	char	bssid[18];
	char	rssi[6];
}T_INFO_WIFIAP;			// WIFI AP��Ϣ

// ��������
extern T_GPRS_CONNECTION	tcp_conn[MAX_NUM_TCP_CONN];	

extern char	 			gsm_telecode[MAX_LEN_GSM_TELECODE+1];	
extern char				gsm_sca[MAX_LEN_PN+1];					
extern char				gsm_apn[MAX_LEN_GSM_APN+1];				

extern char 				de_gsm_cell[MAX_NUM_GSM_CELL][MAX_NUM_GSM_CELL_DMN][MAX_LEN_GSM_CELL_DMN+1];		
extern char				de_gsm_pnin[MAX_LEN_PN+1];				
extern char				de_gsm_dtmf[MAX_LEN_GSM_DTMF+1];	

extern char				gsm_imei[STD_LEN_GSM_IMEI+1];				

extern STATUS			sts_gsm_power;

extern BOOL				is_gsm_ready;	
extern BOOL				is_gsm_calling;

extern BOOL				is_gsmring_pending;	
extern int				cnt_gsmring_asserted;	

extern int				cnt_gsm_recovered;							

extern int				swtimer_input_dtmf;		
extern int				swtimer_set_micmute_off;
extern BOOL				is_dtmf_detection_enabled;				
extern STATUS			sts_dtmf_command;						
extern char				dtmf_password[MAX_LEN_GSM_DTMF+1];	
extern int				times_input_dtmf_password;		

#define DEF_PERIOD_CHECK_GSM_MODULE				30			// 30s

#define DEF_TIMES_RING_TO_ANSWER_CALL			1			// ����ʱ�Զ���ͨ������Ҫ�ﵽ���������
#define MAX_TIMES_INPUT_DTMF_COMMAND			3			// �����ͨ����������dtmf���������������

#define STS_DTMF_COMMAND_CALL_WAITING			0			// �ȴ�����
#define STS_DTMF_COMMAND_CALL_INCOMING			1			// ���յ�����
#define STS_DTMF_COMMAND_CALL_ACCEPTED			2			// �����ͨ
#define STS_DTMF_COMMAND_MIC_MUTE_ON			3			// ����ͷ����
#define STS_DTMF_COMMAND_AUTHORIZED			4			// dtmf����У��ͨ��
#define STS_DTMF_COMMAND_MIC_MUTE_OFF			5			// ��ͷ�����ر�


typedef enum
{
	SMS_LANGUAGE_ENGLISH = 0,
	SMS_LANGUAGE_CHINESE = 1
}T_SMS_LANGUAGE;				// ���ŵ���������

ERR_NO  gsm_init(void);
void gsm_exit(void);
int    gsm_recover(void);
void gsm_reset(void);
void gsm_onoff(void);
void gsm_power_up(void);
void gsm_power_down(void);
int    gsm_sleep(void);
void gsm_wakeup(void);

int  gsm_get_imei(char* imei, int len);
int  gsm_get_swversion(char* version, int len);
int  gsm_get_cellid(void);

int  gsm_check_sim(void);
int  gsm_check_reg(void);

int  gsm_call_out(char* pn);

int  gsm_send_sms(char* pn, char* ascii_utf8);
int  gsm_send_data(int id, unsigned char* data, unsigned int size);
ERR_NO  gsm_recv_data(int id, unsigned char* data, unsigned int to);

void gsm_gpio_off(void);
void gsm_gpio_on(void);

int gprs_soc_setup_dns(void);
int  gprs_soc_setup(int id);
int  gprs_soc_close(int id);
int  gprs_soc_status(int id);
unsigned int gprs_soc_tx(int id, unsigned char* data, unsigned int size);

int  gsm_sms_mode_txt(void);
int  gsm_sms_mode_pdu(void);

int  gsm_find_pattern(char* ptn, unsigned int to);
int  gsm_wait_output(unsigned int len, unsigned int to);
int  gsm_fetch_value(int* val, unsigned int to);
int  gsm_fetch_digits(char* digits, int len, unsigned int to);
int  gsm_fetch_string(char* str, int len, unsigned int to);
int  gsm_fetch_spliters(char ptn, char* str, int len, unsigned int to);
int  gsm_send_at(char* at, char* ptn, int to);
int  gsm_get_rssi(int* rssi);
int  gsm_check_rssi(void);

#endif

#endif

