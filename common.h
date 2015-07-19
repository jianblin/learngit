#ifndef _GLOBAL_H
#define _GLOBAL_H

#include "common.h"

#include "stm32f10x.h"
//this is added by jianblin


/*************************************************************************************************/
/************************ 注意: 以下参数不宜擅自修改，以免造成系统运行异常 ***********************/
/*************************************************************************************************/

/*************************************************************************************************
									系统/设备全局定义
**************************************************************************************************/
// 软硬件版本定义
#if 1
#define HW_VER			17						// 仅JB(PB9)留孔
#define VER_HW			"HW-GTD100-17"
#else	
#define HW_VER			16						// 留出JA(PB8)和JB(PB9两个孔)
#define VER_HW			"HW-GTD100-16"
#endif

#define	VER_SW			"SW-STD-V1.2"			// 1K+1K+63K+63K

// mcu主频设置   
#define USING_MCU_HSI_8MHZ						// MCU使用内部高速时钟(8MHz)，否则使用HSI+PLL(8MHz/2*16)

// 软件功能配置
#undef	USING_FUNC_PROTECTION					// 开启软件保护功能
#undef	USING_FUNC_POWERSAVING					// 开启系统节电功能
#define USING_FUNC_AUTHORIZATION				// 使用操作号码认证功能

#define USING_FUNC_CMD							// 使用命令接口功能

#define USING_FUNC_UPGRADE_COM					// 开启串口程序下载功能

// 外设配置
#define USING_DEV_GPS							// 使用gps硬件
#define  USING_DEV_GSM							// 使用gsm硬件
#define USING_DEV_GSR							// 使用g-sensor硬件

#define TESTING_GPS_SV							// 测试GPS

#undef  USING_IO_KEY								// 使用按键开关机

#ifdef 	USING_DEV_GSM
#define USING_GSM_MIC_MUTE						// 使用gsm模块的咪头静音功能
#endif

// 与终端载体有关的配置
	// 电源配置
	#define USING_PWR_EXT						// 使用外部电源

	#define USING_PWR_ACC						// 使用ACC电源(便携应用模式下，acc引脚必须强行接地，否则acc引脚很容易被发送短信、运动/静止检测等动作干扰)

	#define USING_PWR_BAT						// 电池为选配
	
// #define TEST_GSR_ACQUIRE_DATA

/*************************************************************************************************
									全局宏定义
**************************************************************************************************/

#define MAX_DECIMAL_DIGITS_FOR_INT	10			// 32位int型数据能表示的最大十进制数的位数

// 函数的返回结果
#define 	OK							1
#define 	NG							0

// SWITCH类型的变量赋值
#define	ON							1
#define	OFF							0

#ifdef USING_DEV_GSM
/*************************************************************************************************
									用户定义参数
**************************************************************************************************/
// 用于测试目的的短信接收手机号码，一般为用户手机号码
#define TEST_PHONE_NUMBER	"18038017670"

// 系统支持的三个GPRS连接对应服务器IP地址和端口后(默认使用GPRS_ID0连接配置参数，且为TCP端口)
#define GPRS_ID0_IP			"124.225.65.154"
#define GPRS_ID0_PORT		"80"

#define GPRS_ID1_IP			""
#define GPRS_ID1_PORT		""

#define GPRS_ID2_IP			""
#define GPRS_ID2_PORT		""
#endif

typedef unsigned char	SWITCH;	// 开关型变量类型
typedef int				STATUS;	// 状态型变量类型

typedef int				ERR_NO;	// 函数返回的错误类型

// 天、小时、分、秒时间类型。
typedef struct
{
	unsigned short day;
	unsigned short hour;
	unsigned short minute;
	unsigned short second;
}T_TIME_DHMS;

#define SYS_LANGUAGE_ENGLISH				0	// 英文
#define SYS_LANGUAGE_CHINESE				1	// 简体中文

extern unsigned char 	sys_language;
extern char				swversion[16+1];
extern char				hwversion[16+1];

extern const unsigned short crc16tab[256];

extern BOOL is_sys_idle;

#define MAX(v1, v2)		(v1>v2?v1:v2)
#define MIN(v1, v2)		(v1<v2?v1:v2)

#define MAX_LEN_PN							32		
#define STD_LEN_PN							11		// 中国地区手机号码标准长度为11位数字

// 是否为十六进制数字。
#define IS_HEX_DIGIT(ch)		(((ch>='0' && ch<='9') || (ch>='A' && ch<='F') || (ch>='a' && ch<='f'))?TRUE:FALSE)

int __atoi(char* asc);
float __absf(float v);
unsigned short __abs16(short val);

unsigned short crc16_ccitt(unsigned char* buf, int size);

void __strncpy(char* dst, char* src, int n);
int  __strcmp(char* dst, char* src);

int  fetch_digits(char* dst, char* src);
int  fetch_quotes(char* dst, char* src);
void dump_bin(unsigned char* data, int size, char* remark);

int fetch_domain_in_sms_record(char* dst, unsigned char* src);
int fetch_domain_in_dtmf_command(char* domain, char* dtmf);

BOOL is_all_digit(char* str);
BOOL is_all_ascii(char* str);
#ifdef USING_DEV_GSM
int format_pn(char* pn);
#endif

int is_float_digit(char* str);

int cal_char_num(char* str, char ch);

void second_to_dhms(unsigned int seconds, T_TIME_DHMS* dhms);

void transaction_enter(void);
void transaction_quit(void);


void sys_check_event(void);
void sys_check_sem(void);

#endif

