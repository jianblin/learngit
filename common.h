#ifndef _GLOBAL_H
#define _GLOBAL_H

#include "common.h"

#include "stm32f10x.h"
//this is added by jianblin


/*************************************************************************************************/
/************************ ע��: ���²������������޸ģ��������ϵͳ�����쳣 ***********************/
/*************************************************************************************************/

/*************************************************************************************************
									ϵͳ/�豸ȫ�ֶ���
**************************************************************************************************/
// ��Ӳ���汾����
#if 1
#define HW_VER			17						// ��JB(PB9)����
#define VER_HW			"HW-GTD100-17"
#else	
#define HW_VER			16						// ����JA(PB8)��JB(PB9������)
#define VER_HW			"HW-GTD100-16"
#endif

#define	VER_SW			"SW-STD-V1.2"			// 1K+1K+63K+63K

// mcu��Ƶ����   
#define USING_MCU_HSI_8MHZ						// MCUʹ���ڲ�����ʱ��(8MHz)������ʹ��HSI+PLL(8MHz/2*16)

// �����������
#undef	USING_FUNC_PROTECTION					// ���������������
#undef	USING_FUNC_POWERSAVING					// ����ϵͳ�ڵ繦��
#define USING_FUNC_AUTHORIZATION				// ʹ�ò���������֤����

#define USING_FUNC_CMD							// ʹ������ӿڹ���

#define USING_FUNC_UPGRADE_COM					// �������ڳ������ع���

// ��������
#define USING_DEV_GPS							// ʹ��gpsӲ��
#define  USING_DEV_GSM							// ʹ��gsmӲ��
#define USING_DEV_GSR							// ʹ��g-sensorӲ��

#define TESTING_GPS_SV							// ����GPS

#undef  USING_IO_KEY								// ʹ�ð������ػ�

#ifdef 	USING_DEV_GSM
#define USING_GSM_MIC_MUTE						// ʹ��gsmģ�����ͷ��������
#endif

// ���ն������йص�����
	// ��Դ����
	#define USING_PWR_EXT						// ʹ���ⲿ��Դ

	#define USING_PWR_ACC						// ʹ��ACC��Դ(��ЯӦ��ģʽ�£�acc���ű���ǿ�нӵأ�����acc���ź����ױ����Ͷ��š��˶�/��ֹ���ȶ�������)

	#define USING_PWR_BAT						// ���Ϊѡ��
	
// #define TEST_GSR_ACQUIRE_DATA

/*************************************************************************************************
									ȫ�ֺ궨��
**************************************************************************************************/

#define MAX_DECIMAL_DIGITS_FOR_INT	10			// 32λint�������ܱ�ʾ�����ʮ��������λ��

// �����ķ��ؽ��
#define 	OK							1
#define 	NG							0

// SWITCH���͵ı�����ֵ
#define	ON							1
#define	OFF							0

#ifdef USING_DEV_GSM
/*************************************************************************************************
									�û��������
**************************************************************************************************/
// ���ڲ���Ŀ�ĵĶ��Ž����ֻ����룬һ��Ϊ�û��ֻ�����
#define TEST_PHONE_NUMBER	"18038017670"

// ϵͳ֧�ֵ�����GPRS���Ӷ�Ӧ������IP��ַ�Ͷ˿ں�(Ĭ��ʹ��GPRS_ID0�������ò�������ΪTCP�˿�)
#define GPRS_ID0_IP			"124.225.65.154"
#define GPRS_ID0_PORT		"80"

#define GPRS_ID1_IP			""
#define GPRS_ID1_PORT		""

#define GPRS_ID2_IP			""
#define GPRS_ID2_PORT		""
#endif

typedef unsigned char	SWITCH;	// �����ͱ�������
typedef int				STATUS;	// ״̬�ͱ�������

typedef int				ERR_NO;	// �������صĴ�������

// �졢Сʱ���֡���ʱ�����͡�
typedef struct
{
	unsigned short day;
	unsigned short hour;
	unsigned short minute;
	unsigned short second;
}T_TIME_DHMS;

#define SYS_LANGUAGE_ENGLISH				0	// Ӣ��
#define SYS_LANGUAGE_CHINESE				1	// ��������

extern unsigned char 	sys_language;
extern char				swversion[16+1];
extern char				hwversion[16+1];

extern const unsigned short crc16tab[256];

extern BOOL is_sys_idle;

#define MAX(v1, v2)		(v1>v2?v1:v2)
#define MIN(v1, v2)		(v1<v2?v1:v2)

#define MAX_LEN_PN							32		
#define STD_LEN_PN							11		// �й������ֻ������׼����Ϊ11λ����

// �Ƿ�Ϊʮ���������֡�
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

