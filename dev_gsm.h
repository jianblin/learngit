#ifndef __GSM
#define __GSM

// sms
#define MAX_LEN_SMS_TXTIME					32		// sms发送方发送的时间，包含年月日时分秒以及时区信息

#define MAX_BYTE_SMS_PDU					140		// PDU SMS 最大长度（M660+/M660_1120_D7S13000_V009: 140字节，但是根据GSM协议单条短信最大长度为144字节，中国移动/中国联通：70个汉字）
#define MAX_BYTE_SMS_TXT						140		// TXT SMS 最大长度（M660+/M660_1120_D7S13000_V009: 159字节，但是根据GSM协议单条短信最大长度为144字节，中国移动/中国联通：70个汉字）
													// 由于采用7bit编码发送英文短信时对方解码会出现末位字符乱码，因此改用8bit编码。

#ifdef USING_DEV_GSM

#include "common.h"
#include "log_cmd.h"
#include "stm32f10x.h"

// gsm global
#define MAX_LEN_GSM_TELECODE				8		// 国家代码的最大长度
#define STD_LEN_GSM_IMEI						15		// gsm imei的标准长度

#define TO_GSM_REGISTRATION					15		// GSM上电后注册到网络的超时时间(一般为10秒左右)

#define MIN_RSSI_FOR_COMMUNICATION			5		// 维持gsm正常通讯所需的最低rssi值
														
// 级联短信的参数设置(不管是中文还是英文，单条长短信的最大长度为MAX_LEN_CNC_SMS*MAX_NUM_CNC_SMS，因此单条长短信最多可发送
// MAX_LEN_CNC_SMS*MAX_NUM_CNC_SMS*8/7个英文字符、对多可发送MAX_LEN_CNC_SMS*MAX_NUM_CNC_SMS/2个中文字符)
#define MAX_CHAR_CNC_SMS_CN					67						// 单条级联短信可发送的最大中文字符数(采用USC2编码，每个汉字占用2字节)
#define MAX_CHAR_CNC_SMS_EN					(MAX_CHAR_CNC_SMS_CN*2)	// 单条级联短信可发送的最大英文字符数(采用8bit编码，每个字母占用1字节)

#define MAX_LEN_CNC_SMS						(MAX_CHAR_CNC_SMS_CN*2)	// 每条级联短信的最大长度
#define MAX_NUM_CNC_SMS					4						// 系统支持的单条长短信最大级联数(受限于RAM空间)

#define MAX_LEN_SMS_HEAD					60		// +CMGR:"REC UNREAD","13430981508","","2012/09/08 16:30:08+32"
#define DEF_LEN_SMS_UDH						7		// 发送长短信时UDH的固定长度

#define TO_SMS_TX							10		// RSSI=15左右时，短信发送时间约为8s

// gprs
#define MAX_LEN_GSM_APN						64		// gprs apn，如中国移动为"cmnet"
#define MAX_LEN_GPRS_IP						20		// ip地址的最大长度，如"128.128.128.128"
#define MAX_LEN_GPRS_DN						32		// 域名的最大长度，如"www.google.com"
#define MAX_LEN_GPRS_PORT					8		// 端口的最大长度，如"8080"

#define TO_GPRS_DO_DNS						10		// 域名解析
#define TO_GPRS_SET_APN						2		// 设置APN
#define TO_GPRS_SETUP_CONNECTION			10		// 建立GPRS连接
#define TO_GPRS_CLOSE_CONNECTION			2		// 断开GPRS连接												
#define TO_GPRS_TRANSPARENT_ENTER			2		// 打开透传模式
#define TO_GPRS_TRANSPARENT_EXIT				2		// 关闭透传模式
#define TO_GPRS_TX_FRAME						2		// 发送一帧数据
#define TO_GPRS_RX_FRAME						15		// 接收一帧数据(实验测试单帧最大长度为1500字节)

#define TO_GSM_DOWNLOAD_MODE				120		// 更新gsm模块固件的超时时间
#define TO_GET_IMAGE_VIA_TCP					90		// 远程下载程序文件的超时时间

#define MAX_LEN_GPRS_FRAMERX				1428	// GSM模块接收到GPRS数据后从串口输出的单个数据包最大长度（发送方应将数据分割后再发送，以免接收方接收出错）
														// （极限为1500字节，但是有时输出1500时会出异常："data length over 1500 bytes,exception!"）

#define MAX_TIMES_GPRS_SEND_PACKET			1		// 采用AT+SOCSEND命令发送GPRS数据帧时的最大重发次数(设为1次，以免发送长数据时某帧发送失败后重发导致接收方重复接收该帧)
#define MAX_TIMES_GPRS_SEND_DATA			1		// 发送指定数据(不定长)的最大重试次数
#define MAX_TIMES_GPRS_SETUP_CONNECTION		2		// 建立GPRS连接的最大重试次数

#define MAX_TIMES_GSM_GET_CELLID				2		// 查询gsm基站信息的最大次数
#define MAX_TIEMS_GSM_GET_RSSI				2		// 确认rssi是否正常时查询rssi的最大次数

#define MAX_TIMES_GSM_RECOVER				3		// 连续恢复gsm的最大次数

#define MAX_LEN_GPRS_PACKET_SEND			1392		// AT+SOCSEND命令单次发送的最大数据长度（MMA3328：496字节）

// GPRS ID
#define MAX_NUM_TCP_CONN					4		// 系统内置的gprs服务器地址(GSM模块支持最多3个GPRS并发连接)

#define	GPRS_ID0							0
#define	GPRS_ID1							1
#define	GPRS_ID2							2

// dtmf
#define TO_DTMF_INPUT_PASSWORD				15		// 打开dtmf检测后用户输入dtmf命令的超时时间
#define DELAY_SET_MICMUTE_OFF				1		// dtmf密码输入正确后到关闭咪头静音之间的延迟时间
#define MAX_LEN_GSM_DTMF					64		// dtmf字符串的最大长度(包括密码和命令)

// cell-id
#define MAX_NUM_GSM_CELL					7		// AT+POSI查询的最多基站数量
#define MAX_NUM_GSM_CELL_DMN				7		// gsm基站信息的最大域个数
#define MAX_LEN_GSM_CELL_DMN				6		// gsm基站信息域的最大长度

// server
#define MAX_LEN_EPHSERVER_UASER				32
#define MAX_LEN_EPHSERVER_PASSWORD			16

// error code
// 纯软件错误(和GSM模块无关)
#define ER_GSM_PATTERN_NULL					0		// 输入的模式字符串参数为空

#define ER_GSM_PN_TOOLONG					-1
#define ER_GSM_PN_TOOSHORT					-2
#define ER_GSM_PN_CHAOS						-3		// pn中出现非数字字符

#define ER_GSM_SMS_PDU_CHAOS				-4

// 接口相关错误(一般由GSM模块故障引起)
#define ER_GSM_UART_RECV_NOTHING			-10		// gsm串口无输出(一般为严重硬件错误)
#define ER_GSM_UART_RECV_TIMEOUT			-11		// gsm串口接收超时(串口有输出但在指定时间内接收不到期望的字符或字符串)
#define ER_GSM_UART_RECV_CHAOS				-12		// gsm串口接收到乱码

// 网络相关错误(可能由GSM模块或网络故障引起)
#define ER_GSM_NETWORK_UNREGISTERED		-13		

#define ER_GSM_RSSI_UNKNOW					-15		// rssi = 99
#define ER_GSM_RSSI_TOOLOW					-16		// rssi < 4

#define ER_GSM_GPRS_LINK_TIMEOUT				-17		// gprs连接超时
#define ER_GSM_GPRS_SEND_TIMEOUT			-18		// gprs发送超时
#define ER_GSM_GPRS_RECV_TIMEOUT			-19		// gprs接收超时

// GSM初始化错误
#define ER_GSM_INIT_SIM						-20		// SIM卡为检测到
#define ER_GSM_INIT_REGISTRATION				-21		// 注册网络失败
#define ER_GSM_INIT_SWVERSION				-22		// 查询固件版本失败
#define ER_GSM_INIT_IMEI1						-23		// 查询imei失败
#define ER_GSM_INIT_IMEI2						-24		// 查询得到的imei长度不对
#define ER_GSM_INIT_SMS_MODE					-25		// 设置sms模式为text失败
#define ER_GSM_INIT_SMS_SETTING				-26		// 设置smsflag-ring失败
#define ER_GSM_INIT_SMS_DELETE				-27		// 删除sms失败
#define ER_GSM_INIT_CLIP						-28		// 设置来电号码自动输出失败
#define ER_GSM_INIT_DATAFORMAT				-29		// 设置tcp数据接收传输失败
#define ER_GSM_INIT_RSSI						-30		// 查询rssi失败

#define ER_GSM_RECOVER_FAILURE				-14		// gsm恢复失败

// 结构体定义
typedef struct
{
	char				dn[MAX_LEN_GPRS_DN+1];			// Domain Name
	char				ip[MAX_LEN_GPRS_IP+1];			// IP Address
	char				port[MAX_LEN_GPRS_PORT+1];		// Port
}T_GPRS_CONNECTION;	// 保存GPRS连网参数的结构体

typedef struct
{
	char	mcc[16];	
	char	mnc[16];
	char	lac[16];
	char	cellid[16];
	char	rssi[4];
}T_INFO_GSMCELL;			// GSM基站信息

// for future extension
typedef struct
{
	char	bssid[18];
	char	rssi[6];
}T_INFO_WIFIAP;			// WIFI AP信息

// 变量声明
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

#define DEF_TIMES_RING_TO_ANSWER_CALL			1			// 来电时自动接通来电需要达到的振铃次数
#define MAX_TIMES_INPUT_DTMF_COMMAND			3			// 来电接通后，允许输入dtmf命令密码的最多次数

#define STS_DTMF_COMMAND_CALL_WAITING			0			// 等带来电
#define STS_DTMF_COMMAND_CALL_INCOMING			1			// 接收到来电
#define STS_DTMF_COMMAND_CALL_ACCEPTED			2			// 来电接通
#define STS_DTMF_COMMAND_MIC_MUTE_ON			3			// 打开咪头静音
#define STS_DTMF_COMMAND_AUTHORIZED			4			// dtmf密码校验通过
#define STS_DTMF_COMMAND_MIC_MUTE_OFF			5			// 咪头静音关闭


typedef enum
{
	SMS_LANGUAGE_ENGLISH = 0,
	SMS_LANGUAGE_CHINESE = 1
}T_SMS_LANGUAGE;				// 短信的语言类型

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

