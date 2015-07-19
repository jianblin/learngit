/*GSMӲ��������Ϣ����*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "common.h"
#include "log_pos.h"
#include "dev_gsm.h"
#include "dev_gps.h"
#include "sms.h"
#include "log_time.h"

#include "log_cmd.h"
#include "log_power.h"

#include "dev_mma845x.h"

#include "stm32f10x.h"

#include "string.h"
#include "stdlib.h"

#include "sms.h"

#include "mcu_usart.h"
#include "mcu_gpio.h"
#include "mcu_flash.h"
#include "mcu_systick.h"
#include "mcu_timer.h"

#include "stm32f10x_it.h"

#ifdef USING_DEV_GSM

T_GPRS_CONNECTION	tcp_conn[MAX_NUM_TCP_CONN];				// �豸Ҫ���ʵ�tcp��������������

char	 				gsm_telecode[MAX_LEN_GSM_TELECODE+1];		// ����PDU����ʱ��Ҫ�绰����Ĺ���/��������(���й�Ϊ86��+�Ǳ���)
char					gsm_sca[MAX_LEN_PN+1];						// ��������
char					gsm_apn[MAX_LEN_GSM_APN+1];				// GSM APN

/*
	mcc[6];		
	mnc[6];		
	lac[6];		
	ci[6];   
	bsic[6]; 
	rxlev[6];
	ended[6];
*/
char 		de_gsm_cell[MAX_NUM_GSM_CELL][MAX_NUM_GSM_CELL_DMN][MAX_LEN_GSM_CELL_DMN+1];		// ���ڱ���GSM��վ��Ϣ��ȫ������(���7����վ��ÿ����վ�̶�7����ÿ����ĳ��Ȳ�����6�ֽ�)
char			de_gsm_pnin[MAX_LEN_PN+1];				// ��ǰ��⵽�����������Ϣ
char			de_gsm_dtmf[MAX_LEN_GSM_DTMF+1];		// ��ǰ��⵽��DTMF��Ϣ

char			gsm_imei[STD_LEN_GSM_IMEI+1];				// GSMģ���IMEI��

STATUS		sts_gsm_power = OFF;

BOOL		is_gsm_ready  = FALSE;					// GSMģ���Ƿ�������ʼ���ı�־
BOOL		is_gsm_calling = FALSE;					// gsm�Ƿ���ͨ��״̬(ͨ��״̬�²��ܽ���tcp���ӣ�Ҳ���ܷ���tcp����)

BOOL		is_gsmring_pending;						// gsmģ���ring�ж��¼��Ƿ������
int			cnt_gsmring_asserted;					// GSMģ��RING�жϲ����Ĵ���(GSMģ���ϵ����Զ�����һ��RING�ж�)

int			cnt_gsm_recovered;						// GSMģ����ϻָ��Ĵ���

// dtmf related
int			swtimer_input_dtmf;						// dtmf���򿪺�����û�����dtmf����ĳ�ʱ��ⶨʱ��
int			swtimer_set_micmute_off;				// �û�����dtmf������ȷ��ر���ͷ�������ӳٶ�ʱ��
BOOL		is_dtmf_detection_enabled;				// ��ǰDTMF����Ƿ��
STATUS		sts_dtmf_command;						// DTMF�����ĵ�ǰ״̬
char			dtmf_password[MAX_LEN_GSM_DTMF+1];		// DTMF��������
int			times_input_dtmf_password;				// �Ѿ�����DTMF��������Ĵ���

// ��ʼ��GSMģ��(Ӳ���ϵ硢���������ȳ�ʼ��)��
// ע: �����ȫ�Ͽ�GSM���ߣ�GSM��������ʼ����rssiֵΪ99����ʼ����ɺ��ٲ�ѯ��rssiΪ2-3��
//     �ڴ�����£����ն˷��Ͷ��ţ����ͷ��յ�����δ����Ļظ���
ERR_NO gsm_init(void)
{		
	char		str[64+1];

	int		rssi = -1;	

	int  ret;
	printf("to initialize GSM module.\r\n");

	// ��ʼ��gsmӲ��ǰ��is_gsm_ready����ΪFALSE
	is_gsm_ready 			= FALSE;

	// ��ʼ��gsmӲ��ǰ��is_gsm_calling����ΪFALSE
	is_gsm_calling 			= FALSE;

	is_gsmring_pending 		= FALSE;	
	cnt_gsmring_asserted 	= 0;	

	// ��ʼ��dtmf������ر���
	sts_dtmf_command 		= STS_DTMF_COMMAND_CALL_WAITING;
	times_input_dtmf_password 	= 0;
	
	// ��ʼ��GSM�����շ���ر����ͻ���
	memset(RxBuf3, RxBuf3Size, 0x00);
	RxCnt3_rd = 0;
	RxCnt3_wr = 0;

	memset(TxBuf3, TxBuf3Size, 0x00);
	TxCnt3_rd = 0;
	TxCnt3_wr = 0;

	strcpy(de_gsm_pnin, "");
	strcpy(de_gsm_dtmf, "");
	strcpy(gsm_sca, "");

	is_dtmf_detection_enabled = FALSE;

	strcpy(gsm_imei, "");

	// ��ǿ�йض�GSM��Դ���Ա���ϵͳ�쳣������δ���ڹػ�ǰ�ض�GSM��Դʱ���GSM����״̬
//	gsm_power_down();

	// ��GSM��Դ
//	gsm_power_up();	

	// ��GSMģ���ϵ�
//	gsm_onoff();  	

	// ��ʱ3��ȴ�GSMӲ����ʼ��
	delay_100ms(30);

	// �ȴ�GSMģ��������ע��
	printf("to detect SIM card...\r\n");	
	

//	ret=gsm_send_at("AT\r\n", "OK", 2);
//	 if(ret != OK)
//	 	printf("AT ERROR.......%d\r\n",ret);
//
//	//ATE0�رջ���
//	ret=gsm_send_at("ATE0\r\n", "OK", 8);
//	 if(ret != OK)
//	 	printf("ATE0 ERROR.......%d\r\n",ret);
	// ���SIM���Ƿ����
	if(gsm_check_sim() != OK)
	{
		printf("SIM card not detected.\r\n");

//		gsm_exit();
		
		return ER_GSM_INIT_SIM;		// sim��δ��⵽
	}

	// �ȴ�GSMģ��������ע��
	printf("to wait for GSM registration...\r\n");	
	
	ret=gsm_send_at("ATE0\r\n", "OK", 8);
	 if(ret != OK)
	 	printf("ATE0 ERROR.......%d\r\n",ret);
	// ����SIM���Ƿ���룬ģ���ϵ�󶼻����+EIND�����ַ�����
	// �Դ˼��ģ���Ƿ��������(M660+ģ���ע������ʱ��һ��̶�Ϊ10��)��	
	/*
	if(gsm_find_pattern("+EIND: 1\r\n", TO_GSM_REGISTRATION) != OK)
	{	
		printf("+EIND: 1 not found.\r\n");

		gsm_exit();		
		
		return ER_GSM_INIT_REGISTRATION;		// ����ע���쳣
	}	
	*/

#if 1
	// ��ѯGSM�̼��汾	
	if(gsm_get_swversion((char*)str, 64) == OK)
	{
		printf("GSM firmware version: %s.\r\n", str);
	}
	else
	{
		printf("failed to get GSM firmware version.\r\n");

//		gsm_exit();		
		
		return ER_GSM_INIT_SWVERSION;			// ��ѯGSM�̼��汾�쳣
	}
#endif	

#if 1
	// ��ѯIMEI��(GSMģ����ϵ���ѯIMEI����ʧ�ܣ�ԭ����)
	if(gsm_get_imei((char*)gsm_imei, STD_LEN_GSM_IMEI) != OK)
	{
		printf("failed to get GSM IMEI.\r\n");	

//		gsm_exit();

		return ER_GSM_INIT_IMEI1;
	}
	else
	{
		// ���imei�����Ƿ�Ϊ15λ
		if(strlen(gsm_imei) != 15)
		{
			printf("length of imei is not 15!\r\n");

			gsm_exit();

			return ER_GSM_INIT_IMEI2;
		}
		else
		{		
			printf("GSM IMEI: %s\r\n", gsm_imei);
		}
	}
#endif	
#if 0 
// SMS������ʼ��
	// ��GSMģ��Ķ���Ϣģʽ����Ϊ�ı�ģʽ
	if(gsm_sms_mode_txt()!= OK)
	{
		gsm_exit();

		printf("failed to set SMS mode to text.\r\n");
		
		return ER_GSM_INIT_SMS_MODE;
	}
#endif
	// ����Ϣ���ģʽ����Ĭ������(�Ա�GSMģ����յ�����ʱ�����+SMSFLAG��ʾ��Ϣ)�������������÷���������������ring�ж�
#if 0	
	if(gsm_send_at("AT+CNMI=1,1,0,0,0\r\n", "OK", 2) != OK)		// ��ֹ���յ��Ķ����Զ��������ʹ�ϲ�Ӧ�����������ǰ��RING����
	{
		printf("failed to set CNMI.\r\n");

		gsm_exit();
		
		return ER_GSM_INIT_SMS_SETTING;
	}
	// printf("enable outputing received SMS automatically.\r\n");
#endif	
#if 0
	// ɾ����ǰ���Ŵ洢���е����ж��ţ�������Ž����쳣
	if(gsm_send_at("AT+CMGD=\"DEL ALL\"\r\n", "OK", 5) != OK)
	{
		gsm_exit();

		printf("failed to delete all sms in current memory.\r\n");
		
		return ER_GSM_INIT_SMS_DELETE;
	}

// Call������ʼ��
	// ʹ����������Զ����
	if(gsm_send_at("at+clip=1\r\n", "OK", 2) != OK)
	{
		gsm_exit();

		printf("failed to enable phone number of incoming call output automatically.\r\n");
		
		return ER_GSM_INIT_CLIP;
	}
#endif
// GPRS������ʼ��
	// ���������շ���ΪASCIIģʽ(����ͳ�Ķ�����ģʽ)
	/*
	if(gsm_send_at("AT+DATAFORMAT=1,1\r\n", "OK", 2) != OK)
	{
		gsm_exit();

		printf("failed to set data mode to ASCII.\r\n");
		
		return ER_GSM_INIT_DATAFORMAT;
	}
	*/
	// ��ѯRSSI
	if(gsm_get_rssi((int*) &rssi) == OK)
	{
		printf("RSSI = %d\r\n", rssi);	
	}
	else
	{
		gsm_exit();

		printf("failed to get rssi.\r\n");
		
		return ER_GSM_INIT_RSSI;
	}

	cnt_gsm_recovered = 0;

	// ��ʼ��gsm�ɹ���is_gsm_ready����ΪTRUE
	is_gsm_ready = TRUE;
	ret = gprs_soc_setup_dns();
	if(ret!=OK)
	{
		printf("tcp setup error");
		return NG;
	}
	printf("GSM initialized successfully.\r\n");	
	
	return OK;
}

// ���������̹ض�GSMģ�顣
void gsm_exit(void)
{
	gsm_onoff();
				
	gsm_power_down();
}

/**********************************************************************************************************************
										 		GSMӲ��IO��������
***********************************************************************************************************************/

// ����GSMģ��(����SYSRST��50ms����)��
void gsm_reset(void)
{		
	/*
	GPIO_ResetBits(GPIOA,GPIO_Pin_15);	
	
	delay_100ms(1);						

	GPIO_SetBits(GPIOA,GPIO_Pin_15);

	delay_100ms(1);
	*/
}

/*
>> SIM������״̬�£�M660+�ϵ����Զ����������Ϣ:

+EIND: 128

+EUSIM: 0

+STKPCI: 0,"D081B6810301250082028182850B80795E5DDE884C592957308F0A01808F7B677E95EE50198F0A038077ED4FE17FA453D18F0A0480670065B063A883508F0A06804E1A52A17CBE90098F10078065E07EBF97F34E504FF14E5090E88F0E08800031003300394E9280547F518F0809808D224FE1901A8F0E0A8079FB52A84F1860E04E13533A8F0E0B8079FB52A875355B50554652A18F120C806211768400530049004D84254E1A53858F0E058000530049004D53614FE1606F"

+EIND: 2		// �����+STKPCI�����+EIND: 2֮����ܼ������3��

+EIND: 1		// �����+EIND: 2�����+EIND: 1֮����ܼ������5��(ʱ�䳤��ȡ����SIM�������ͺ���Ӧ�ĳɹ�ע�ᵽ�����ʱ��)

>> SIM��δ��״̬�£�M660+�ϵ����Զ����������Ϣ:

+EIND: 128

+EIND: 2

+EIND: 1		// ��ʼ����ɺ��Զ���������һ����Ϣ

*/
// ��GSMģ���ϵ�/�µ�(����/����POWERON����3s����)��
void gsm_onoff(void)
{	
	//MCU_GPIO_LOW(GPIO_GSM_ONOFF);
	
	//delay_100ms(30);	

	//MCU_GPIO_HIGH(GPIO_GSM_ONOFF);

	//printf("GSM On/OFF.\r\n");
}

// �򿪸�GSMģ��Ĺ��硣
void gsm_power_up(void)
{
	// GPIO_ResetBits(GPIOB,GPIO_Pin_13);
	MCU_GPIO_HIGH(GPIO_GSM_PWR);
	
	delay_100ms(3);	

	sts_gsm_power = ON;

	printf("GSM powered up.\r\n");
}

// �رո�GSMģ��Ĺ��硣
void gsm_power_down(void)
{
	// GPIO_SetBits(GPIOB,GPIO_Pin_13);
	MCU_GPIO_LOW(GPIO_GSM_PWR);
	
	delay_100ms(3);	

	sts_gsm_power = OFF;

	printf("GSM powered down.\r\n");
}

/*
��GSMģ������ߺͻ��Ѳ�������ȡ"Ĭ�����ߡ�����ʹ��ǰ���ѡ�����ʹ�ú����ߡ��������ղ�����"��ԭ��
------------------------------------------------------------------------------------------------
����ģ��������ģʽ�Ļ������̣�
1������ģ��� DTR ����Ϊ�ߵ�ƽ��ͨ��AT ָ�ģ������Ϊ�����������ģʽ���ο�ָ
�at+enpwrsave��
2����ģ��� DTR �����õͣ�Ӳ������ģ�����͹���״̬��ͨ��ģ�����2 �����ҽ���
�������ڴ���ģʽ�£�ģ��Ĵ����ǹرյģ�û����Ӧ�����е�Ҳ��ֹͣ��˸��
ģ��ֻ���ڿ���ʱ�Ż�������ģʽ����������ݽ���δ������������������
3��������������ݻ��ߺ��е�����ҵ�񣬿��Խ� DTR �øߣ�ģ�������˳�����ģʽ����
������ģʽ�����ڴ���ӦAT ָ�������ҵ������Ϻ��ⲿCPU �ٽ�DTR ��
�ͣ�ģ��������ģʽ��
4���ڴ���״̬�£����ģ���б���ҵ�񣬱������硢�����š��������������ݣ�ģ�����
���˳�����ģʽ����ͨ���������������Ϣ���ⲿCPU �ڼ�⵽������Ϣ�󣬽�����
��DTR �øߣ��ٴ������硢���ݵȡ���������Ϻ󣬽�DTR �õͣ�ʹģ��������ģ
ʽ���������ʱ��DTR û���øߣ��Ҵ���û����Ϣ����ģ�����2~30 �������Զ�����
����ģʽ��
*/

/*
at+enpwrsave=1
OK

at+enpwrsave=1
CME ERROR:<error>

at+enpwrsave?
+ENPWRSAVE:1
*/
// ��ģ�����ߡ�
int gsm_sleep(void)
{	
	// ����DTR����
//	MCU_GPIO_HIGH(GPIO_GSM_DTR);

	// ����ģ���������ģʽ
//	if(gsm_send_at("AT+ENPWRSAVE=1\r\n", "OK", 2) != OK)
//	{	
//		return NG;
//	}

	// ����DTR����
//	MCU_GPIO_LOW(GPIO_GSM_DTR);

	// ģ��ֻ���ڿ���ʱ�Ż�������ģʽ����������ݽ���δ������������������

//	printf("gsm sleep.\r\n");

	return OK;
}

// ǿ�л���GSMģ��(����ģ�������������ݴ������Ż�绰ǰ����)��
void gsm_wakeup(void)
{		// ��鵱ǰDTR�Ƿ����ͣ���������˵��֮ǰģ�鱻�ֶ���������״̬
//	if(MCU_GPIO_READ(GPIO_GSM_DTR)== 0)
//	{
		// ����DTR����
//		MCU_GPIO_HIGH(GPIO_GSM_DTR);
		
//		delay_100ms(20);

//		printf("gsm woke up.\r\n");
//	}
}

// ��M660+����ͨѶ�쳣(����GPRS���ӵ��漰����״���ĺ�ʱ����������)����£�
// ���Խ�ģ��ָ����ϵ��ĳ�ʼ״̬(��ϵͳ������⵽ģ������������ϵ��쳣��
// ����Ϊģ������ش���ϣ�Ȼ������Ӳ�����ϱ�������)��
int gsm_recover(void)
{
	printf("to recover GSM No.%d times...\r\n", cnt_gsm_recovered+1);
	
	if(gsm_init() == OK)
	{	
		printf("recover GSM module OK.\r\n");

		// �ָ�gsm�ɹ���cnt_gsm_recovered����Ϊ0
		cnt_gsm_recovered = 0;
			
		return OK;
	}
	// gsm��ʼ��ʧ�ܿ��������������źŲ��á�sim��δ��⵽��sim��Ƿ���Լ�gsmģ�鴮��ͨѶ���������
	else
	{
		printf("recover GSM module NG.\r\n");

		// cnt_gsm_recovered����������
		cnt_gsm_recovered++;

		// ��������ָ�gsm�Ĵ����Ƿ���
		if(cnt_gsm_recovered >= MAX_TIMES_GSM_RECOVER)
		{
			printf("failed to recovery gsm continuously %d times.\r\n", cnt_gsm_recovered);	
		}
		
		return NG;
	}
}

/**********************************************************************************************************************
											GSM���ڽ��ջ����������
***********************************************************************************************************************/

// ��GSM UART���ջ����С��ڸ�����ʱ���ڴӵ�ǰ��ָ��λ�ÿ�ʼ����ָ����Pattern��
ERR_NO gsm_find_pattern(char* ptn, unsigned int to)
{
	int 			i = 0;
    unsigned int 	len = 0;
	unsigned int	RxCnt3_wr_backup = RxCnt3_wr;
	
	to = systick+to*1000/SYSTICK_PERIOD;

	// �����������ĺϷ���
	len = strlen(ptn);
	
	if(len <= 0)
	{
		return ER_GSM_PATTERN_NULL;
	}

	while(systick < to)
	{  		
		// ����ģʽ�ַ����Ĺ��̿��ܺ�ʱ�ϳ�����˲��ҹ�����Ӧ��ʱι��
		WATCHDOG_RELOAD();

		// ��鴮����������
		check_buf_com();	
		// check_que_cmd(CHECK_MODE_CONTINUOUS);
		
        if((RxCnt3_wr - RxCnt3_rd) >= len)
        {   			
        	// Ϊ�������������������ֽ��ƶ��Ļ������ڱȽϷ�����δ����KMP�ȿ��������㷨��
        	// ʵ���϶���GSM���ջ���ļ��һ�㶼����Ŀ�ĵģ��������ַ������ֵ�λ�þ��뵱ǰ
        	// ���α�һ�㶼��Զ������ַ���ƥ��Ĺ��̺�ʱ���ޡ�
        	for(i = 0; i < len; i++)
            {
            	if(GSM_RX_RD(RxCnt3_rd+i) != ptn[i])
            	{
            		break;
            	}
            }
            
            if(i == len)
            {
            	RxCnt3_rd += i;		// ���α������Ӧ����
            	
            	return OK;
            }
            else
            {
            	RxCnt3_rd++;		// ����һ�ֽڴ���������ָ����Pattern
            }        
        }			    				 
	}

	if(RxCnt3_wr == RxCnt3_wr_backup)
	{
		return ER_GSM_UART_RECV_NOTHING;
	}
	else
	{
		return ER_GSM_UART_RECV_TIMEOUT;
	}
}

// �ȴ����ջ����ڽ��յ�ָ�����ȵ�δ�����ݣ��˺�����������Ҫ�ȴ����ܸ������ݵ�ʱ�򣬴Ӷ�Ϊ������������µ����ݣ���
ERR_NO gsm_wait_output(unsigned int len, unsigned int to)
{
	unsigned int	RxCnt3_wr_backup = RxCnt3_wr;
		
	to  = systick + to*1000/SYSTICK_PERIOD;

    while(systick < to)
    {
		WATCHDOG_RELOAD();

		// �ȴ�gsm�������ʱ��鴮��1�����������롣
		check_buf_com();
		// check_que_cmd(CHECK_MODE_CONTINUOUS);
		
        if((RxCnt3_wr - RxCnt3_rd) >= len)
        {
            return OK;   
        }
    }

	if(RxCnt3_wr == RxCnt3_wr_backup)
	{
		return ER_GSM_UART_RECV_NOTHING;
	}
	else
	{
		return ER_GSM_UART_RECV_TIMEOUT;
	}       
}

// �ӵ�ǰ���α�λ�ÿ�ʼ��ȡ�����ַ���������ת��Ϊ��ֵ�Է���(֧�ָ�����������ȡ)��
// Note: 
// 1, the character pointed by the passed 'start' must be digit;
/*
	AT+CSQ

	+CSQ: 20, 99
*/
ERR_NO gsm_fetch_value(int* val, unsigned int to)
{
	char	        str[MAX_DECIMAL_DIGITS_FOR_INT+1];
	int		        i = 0;
    char			ch;

	unsigned int	RxCnt3_wr_backup = RxCnt3_wr;
    
    to = systick+to*1000/SYSTICK_PERIOD;

	*val = 1;

	strcpy(str, "");
	
 	// �ȶ�λ�����ִ�����ʼ�ַ���'-'��'0~9'�� 
    while(systick < to)
    {    	
		WATCHDOG_RELOAD();
		
    	if(RxCnt3_rd < RxCnt3_wr)
    	{
	    	ch = GSM_RX_RD(RxCnt3_rd++);		
	    	
			if(ch == '-')
			{
				// δ���⵽�κ������ַ�ǰ��⵽'-'����Ϊ����ֵ����������
				if(i == 0)
				{
					*val = -1;
				}
				// �����⵽'-'����Ϊ�������ַ�ĩβ�ĺ�һ�ַ�
				else
				{
					str[i] = '\0';		// �������ַ������������ַ�����β����

					break;
				}
			}
			else if(ch >= '0' && ch <= '9')
			{
				// ���α���ʱ���ƶ�����պ�ָ�������ַ�
				str[i++] = ch;

				if(i >= MAX_DECIMAL_DIGITS_FOR_INT)
				{
					str[i] = '\0';		// �������ַ������������ַ�����β����

					break;
				}
			}
			else
			{
				// ���str���α��Ƿ����0�������ڣ���˵��str���Ѿ�����������ַ���
				// ��ǰ��⵽�������ַ��Ļ�˵�������ַ����Ѿ�������
				if(i > 0)
				{
					str[i] = '\0';		// �������ַ������������ַ�����β����

					break;
				}
				// else: �Թ���ǰ�ķ������ַ��������һ���ַ�
	    	}
	    }
    }

	if(RxCnt3_wr == RxCnt3_wr_backup)
	{
		return ER_GSM_UART_RECV_NOTHING;
	}
	else
	{
		// ����⵽�������ַ��ᴮת��Ϊ��ֵ(ע����ֵ��������)
	    if(strlen(str) > 0)
	    {
	    	*val *= __atoi(str);
	    	
	    	return OK;
	    }
	    else
	    {
	        return ER_GSM_UART_RECV_TIMEOUT;
	    }
	}    
}

// �ӵ�ǰ���α�λ�ÿ�ʼ��ȡ�����������ַ��������������е�'.'����
/*
	AT^GETLBS?

	MCC=460,MNC=0,LAC=9712,CELL_ID=3942,
	MAIN CELL Signal Strength:-72,
	NC_CELL_ID: 0, 0, 0,62194, 0, 0,
	Surround CELL Signal Strength:0,0,-72,-72,0,0
*/
ERR_NO gsm_fetch_digits(char* dig, int len, unsigned int to)
{
	unsigned int 	i = 0;
	char			ch; 

	unsigned int	systick_backup = systick;

	unsigned int	RxCnt3_wr_backup = RxCnt3_wr;
	
	to  = systick+to*1000/SYSTICK_PERIOD;
	
	// �ȶ�λ���׸������ַ�
	while(systick < to)
	{
		WATCHDOG_RELOAD();
		
		if(RxCnt3_rd < RxCnt3_wr)
		{
			ch = GSM_RX_RD(RxCnt3_rd);

			if(ch >= '0' && ch <= '9')
			{
				break;
			}
			else
			{
				RxCnt3_rd++;
			}
		}
	}

	// ����to
	to -= (systick-systick_backup);

	// ��ȡ�����������ַ�
    while(systick < to)
    {        
		WATCHDOG_RELOAD();
		
        if(RxCnt3_rd < RxCnt3_wr)   // GSM UART���ջ���Ķ�ָ�벻�ܳ���дָ�룬������Ҫ�ȴ�дָ�����
        {  
        	ch = GSM_RX_RD(RxCnt3_rd++); // RxBuf3[(RxCnt3_rd++) & (RxBuf3Size-1)];
			
            if(	(ch >= '0' && ch <= '9') || ch == '.')	// ֧�ָ���������ȡ���������м���'.'��
            {  
                dig[i++] = ch; 	

				if(i >= len)
				{
					dig[i] = '\0';	

					// ������Ŀ���ַ����˳�
					return OK;
				}
            }
			else
			{
				dig[i] = '\0';			

				if(i == 0)
				{
					return ER_GSM_UART_RECV_TIMEOUT;			
				}
				else
				{
					// �����������ַ��˳�
					return OK;	
				}
			}
        }
    }

	if(RxCnt3_wr == RxCnt3_wr_backup)
	{
		return ER_GSM_UART_RECV_NOTHING;
	}
	else
	{
		return ER_GSM_UART_RECV_TIMEOUT;
	}   
}

// �ӵ�ǰ���α�λ�ÿ�ʼ��ȡ��'\r\n'��β���ַ�����
ERR_NO gsm_fetch_string(char* str, int len, unsigned int to)
{
    unsigned int i = 0;

	unsigned int	RxCnt3_wr_backup = RxCnt3_wr;
    
    to  = systick+to*1000/SYSTICK_PERIOD;

    while(systick < to)
    {        
		WATCHDOG_RELOAD();
		
        if((RxCnt3_wr-RxCnt3_rd) > 1)   
        {  			
        	// ���յ����з��˳�     
            if(GSM_RX_RD(RxCnt3_rd+0) == '\r' && GSM_RX_RD(RxCnt3_rd+1) == '\n')
            {  
                str[i] = '\0';
            
                RxCnt3_rd += 2;		// ǰ�ƶ��α�
            
                return OK;				
            }
            else
            {
            	str[i++] = GSM_RX_RD(RxCnt3_rd++);

				// Ŀ���ַ����������˳�
				if(i >= len)
				{
					str[i] = '\0';

					return OK;
				}
            }
        }
    }

	if(RxCnt3_wr == RxCnt3_wr_backup)
	{
		return ER_GSM_UART_RECV_NOTHING;
	}
	else
	{
		return ER_GSM_UART_RECV_TIMEOUT;
	} 
}

/* 
�ύ��Ч�Ļ�վ��Ϣ��Google�������󣬴ӷ��������صĶ�λ��Ϣ:
d5
{"location":{"latitude":32.117301,"longitude":114.116606,"address":{"country":"China","country_code":"CN","region":"Henan","city":"Xinyang"},"accuracy":1625.0},"access_token":"2:OkivQMlLSpRHuPTx:Beb-gnvVNJBB3-vt"}
0
*/

// ��ȡָ���ָ����ڵ��ַ�(�ָ���������"��|��)��
ERR_NO gsm_fetch_spliters(char ptn, char* str, int len, unsigned int to)
{
	unsigned int 	i 			= 0;
	int				spliters 	= 0;

	unsigned int	RxCnt3_wr_backup = RxCnt3_wr;
	
	to  = systick+to*1000/SYSTICK_PERIOD;

	// ��ȡ��һ��˫����֮�󡢵ڶ���˫����֮ǰ���ַ�(ע������ʱ����Թ���쳣���)
	while(systick < to)
	{
		WATCHDOG_RELOAD();
		
		if(RxCnt3_rd < RxCnt3_wr)
		{			
			// ����AT+IPSTATUS=0�������Ӧ��Ϣ��״̬�ַ���û���Զ��Ŷ����Ի��з���β�����ҲҪ����ȡ����
			// +IPSTATUS:0,DISCONNECT
			if(GSM_RX_RD(RxCnt3_rd) == ptn || GSM_RX_RD(RxCnt3_rd) == '\r')
			{
				if(spliters == 0)
				{
					spliters++;
		
					RxCnt3_rd++;
				}
				else
				{
					// ������ָ����ڵ��ַ����˳�
					str[i++] = '\0';
					
					RxCnt3_rd++;
		
					return OK;
				}
			}
			else
			{
				if(spliters == 1)
				{
					str[i++] = GSM_RX_RD(RxCnt3_rd);

					// Ŀ���ַ����������˳�
					if(i >= len)
					{
						str[i++] = '\0';

						RxCnt3_rd++;

						return OK;
					}
				}
		
				RxCnt3_rd++;
			}
		}
	}

	if(RxCnt3_wr == RxCnt3_wr_backup)
	{
		return ER_GSM_UART_RECV_NOTHING;
	}
	else
	{
		return ER_GSM_UART_RECV_TIMEOUT;
	}
}

/**********************************************************************************************************************
										 		GSM����ͨѶ���ܺ���
***********************************************************************************************************************/

// ����ָ����AT�����鷵����Ϣ���Ƿ����ָ���������ַ������Ӷ��ж�AT����ͺ�ķ��������
// ע: ����ÿ��AT�����ִ��ʱ��򳤻�̣���˷���AT����ĳ�ʱʱ��ҲӦ��Ӧ�趨��
ERR_NO gsm_send_at(char* at, char* ptn, int to)
{	
	RxCnt3_rd = RxCnt3_wr;		// ����AT����ǰ�����α�ֱ���Ƶ�д�α�λ�ã��Ӷ�����֮���δ������

	// ͨ��GSM UART�ڷ���AT����
	usart3_send_int(at, strlen(at));	

	// �ڹ涨��ʱ���ڣ���GSM UART���ջ����ָ��λ�ÿ�ʼ����ָ����Pattern
	return gsm_find_pattern(ptn, to);
}

/*
AT+CSQ

+CSQ: 28, 99

OK
AT+CSQ

+CSQ: 28, 99

OK
*/
// ��ѯ��ǰ��GSM�ź�ǿ��(SIM��δ����ʱҲ��������ѯRSSI)��
ERR_NO gsm_get_rssi(int* rssi)
{   	
	ERR_NO		ret;
	
	printf("to get rssi.\r\n");

	ret = gsm_send_at("AT+CSQ\r\n", "+CSQ:", 3);	
	
	if(ret < 0)
	{	
		return ret;
    }

	return gsm_fetch_value(rssi, 3);	
}

/*
AT+CGSN
358511020024166		// ������Ϣ����+CGSN:��Ϣǰ׺����˸���+CGSN\r\n��Ϊ��ⷵ����Ϣ�������ַ���
OK
*/
// ��ѯGSMģ���IMEI��
ERR_NO gsm_get_imei(char* imei, int len)
{
	ERR_NO		ret;

	ret = gsm_send_at("AT+CGSN\r\n","", 2);
	if(ret < 0)
	{	
		return ret;
    }

	return gsm_fetch_digits(imei, STD_LEN_GSM_IMEI, 2);
}

// ��ѯGSMģ�������汾�š�
ERR_NO gsm_get_swversion(char* version, int len)
{
	ERR_NO 		ret;

	ret = gsm_send_at("AT+CGMR\r\n", "", 2);
	if(ret < 0)
	{	
		return ret;
    }

	return gsm_fetch_string(version, len, 2);
}

/* ʹ��AT+POSI�����ѯ�ܱ߻�վ����Ϣ(���7����SIM��δ����ʱҲ���������)
>> SIM������ʱ:

AT+POSI=1

+POSI: 1,460,01,517B,D2A3,38,42,0,460,01,517B,A872,30,38,0,460,01,517A,FFAC,0F,36,0,460,01,517A,6A7E,2F,34,0,460,01,517B,A871,19,31,0,460,01,517B,576D,36,29,0,460,01,517A,AAE8,2E,28,1

OK

1		// mode

// �Ʋ��վ�б��еĵ�һ��Ϊ��ǰע���վ���˻�վ��Ϣ����Ϊ��վ��λ����ѡ
,
460,	// mcc	
01,		// mnc
517B,	// lac
D2A3,	// ci
38,		// bsic
42,		// rxlev
0		// ended

>> SIM��δ����ʱ:

AT+POSI=1

+POSI: 1,460,01,2533,71E0,39,53,0,460,01,2533,741E,17,37,0,460,01,2533,7172,22,34,0,460,01,2533,735E,35,32,0,460,01,2533,7171,2E,30,0,460,01,2533,71F4,1B,27,0,460,01,2533,741B,00,25,1

OK

�����û���ҵ��κ�С������ֱ�ӷ���OK�������վ��ϢΪ��������������MCC��ENDED ֮��ѭ����
*/

/* ʹ��AT+CREG�����ѯ��ǰע���վ����Ϣ(SIM��δ����ʱ���ܼ��):
>> SIM������ʱ:

AT+CREG=2

OK
AT+CREG?

+CREG: 2, 1, "517B", "A872", 0	// ��һ����ʾ�Ļ�վ�ƺ����ǵ�ǰע��Ļ�վ

OK

+CREG: 1, "517B", "D2A3", 0		// �ڶ�����ʾ��ͷһ����վ���ǵ�ǰע��Ļ�վ?

+CREG: 1, "517B", "A872", 0

>> SIM��δ����ʱ:
AT+CREG=2

OK
AT+CREG?

+CREG: 2, 0

OK

>> ����ע���վ��Ϣ�Զ������ģ�������Ϊ��վ�л����Զ����ע���վ��Ϣ:

AT+CREG=2			// ʹ��ע���վ��Ϣ�Զ����

OK				
AT+CREG?

+CREG: 2, 1, "2796", "140C", 0

OK

+CREG: 1, "2796", "140D", 0		// ���¶��ǻ�վ�л�ʱ�Զ������ע���վ��Ϣ

+CREG: 1, "2796", "140C", 0

+CREG: 1, "2796", "140D", 0

+CREG: 1, "2796", "140C", 0

+CREG: 1, "2796", "140D", 0

+CREG: 1, "2796", "140C", 0

+CREG: 1, "2796", "140D", 0

*/

/*
AT+CSQ

+CSQ: 24, 99

OK
*/
// ���GSMģ���ܱ�7���ڽ���վ����Ϣ��
// ע: �ճ�ʼ��gsmģ��������ϲ�ѯgsm��վ��Ϣ��������
int gsm_get_cellid(void)
{
	int 	i= 0;
	int		j = 0;
	int		k = 0;
	int		to;

	char	ch;

	unsigned int 	RxCnt3_rd_backup;
	
	ERR_NO			ret;

	printf("to get gsm cell info...\r\n");

	ret = gsm_send_at("AT+POSI=1\r\n", "+POSI: ", 6);  //sim900a���޴�ָ��
	if(ret < 0)
	{		
		return ret;
	}

	// ��¼��վ��Ϣ�ַ�������ʼλ�á�
	RxCnt3_rd_backup = RxCnt3_rd;

	ret = gsm_find_pattern("\r\n", 2);
	if(ret < 0)
	{		
		return ret;
	}

	to = systick+2*1000/SYSTICK_PERIOD;

	// �ָ���վ��Ϣ�ַ�������ʼλ�á�
	RxCnt3_rd = RxCnt3_rd_backup;

	// ��ȡ������վ����Ϣ(ÿ����վ����7����)
	for(i = 0; i < MAX_NUM_GSM_CELL; i++)
	{
		for(j = 0;  j < MAX_NUM_GSM_CELL_DMN; j++)
		{
			for(k = 0; k < MAX_LEN_GSM_CELL_DMN; k++)
			{
				if(RxCnt3_rd < RxCnt3_wr)
				{
					ch = GSM_RX_RD(RxCnt3_rd++);

					if(IS_HEX_DIGIT(ch) == TRUE)
					{
						de_gsm_cell[i][j][k] = ch;
					}
					else if(ch == ',')
					{
						de_gsm_cell[i][j][k] = '\0';
						
						// �����굱�ڻ�վ�ĵ�������Ϣ
						break;
					}
					else if(ch == '\r')
					{
						de_gsm_cell[i][j][k] = '\0';

						// ȫ����վ��ȫ������Ϣ�������
						return OK;
					}
					else
					{
						// ���յ�����
						return ER_GSM_UART_RECV_CHAOS;
					}
				}

				// ������ȫ����վ��Ϣ�Ƿ�ʱ��
				if(systick > to)
				{
					// ���ճ�ʱ�˳�
					return ER_GSM_UART_RECV_TIMEOUT;
				}
			}

			printf("%s, ", de_gsm_cell[i][j]);
		}

		printf("\r\n");
	}

	return OK;
}

/*
AT+GETIP

+LOCALIP:10.58.143.117

AT+GETIP

ERROR

*/
// ��GPRS���ӽ����󣬲�ѯģ�������IP��ַ��
ERR_NO gsm_get_ip(char* ip, int len)
{
	ERR_NO			ret;

	ret = gsm_send_at("AT+CIFSR\r\n", "OK", 3); //SIM900A AT+CIFSR
	if(ret < 0)
	{
		return ret;
	}

	return gsm_fetch_digits(ip, len, 2);
}

/*
at+dns="www.china.com"
OK
+DNS:124.238.253.103
+DNS:OK

AT+DNS="agps.u-blox.com"

ERROR

AT+DNS="www.anttna.com"

OK
+DNS:27.54.228.98
+DNS:OK
*/
// ���ƶ�������������IP��ַ(Ӧ��APN���ú�ִ��)��
ERR_NO gsm_get_dns(char* ip, int len_ip, char* dn)
{
	char		at[128];
	ERR_NO		ret;

	printf("to inquire dns...\r\n");
	
	sprintf(at, "AT+CDNSGIP=\"%s\"\r\n", dn);  //sim900a  AT+CDNSGIP

	// �ȼ��rssiֵ�Ƿ�����
	ret = gsm_check_rssi();
	if(ret < 0)
	{
		return ret;
	}

	// OK��+DNS����ͬʱ����
	ret = gsm_send_at(at, "OK", TO_GPRS_DO_DNS);
	if(ret < 0)	// M660+��AT+DNS���ʱ�Ѿ����ò�Ĭ��Ϊ5�룬2012-04-19 23:23
	{
		return ret;
	}
	sprintf(at,"CDNSGIP:1\"%s\"\r\n",dn);
	ret = gsm_find_pattern(at, 1);
	if(ret < 0)	
	{
		return ret;
	}

	return gsm_fetch_digits(ip, len_ip, 1);
}

/*
>> ͨ�������з���

ATD13923887347

OK

SPEECH ON

ALERTING

CONNECT
AT+VTS=0

OK
AT+VTS=1

OK
AT+VTS=2

OK
AT+VTS=*

OK
AT+VTS=#

OK
AT+VTS=A

OK
AT+VTS=B

OK

SPEECH OFF

RELEAS

>> ��ͨ�������з���

AT+VTS=1

ERROR


>> ͨ�������н���DTMF

AT+DTMFDETECT=1

+DTMF:DETECT START OK

DTMF KEY(Rec): 0

DTMF KEY(Rec): 1

DTMF KEY(Rec): 2

DTMF KEY(Rec): 3

DTMF KEY(Rec): 4

DTMF KEY(Rec): 5

DTMF KEY(Rec): 6

DTMF KEY(Rec): 7

DTMF KEY(Rec): 8

DTMF KEY(Rec): 9

DTMF KEY(Rec): *

DTMF KEY(Rec): #

SPEECH OFF

RELEASE

NO CARRIER
*/
// ����ָ����DTMF�ַ�(֧��0,1,2,3,4,5,6,7,8,9,A,B,C,D,#,*������ͨ�������з���)��
ERR_NO gsm_send_dtmf(char dtmf)
{
	char		at[16];

	sprintf(at, "AT+VTS=%c\r\n", dtmf);

	return gsm_send_at(at, "OK", 3);
}

/*
>> SIM������ʱ:

AT+CCID

+CCID: 89860109520204253890

OK

>> SIM��δ����ʱ:

AT+CCID

ERROR

*/
// ͨ����ѯSIM ID�ż��SIM���Ƿ���롣
ERR_NO gsm_check_sim(void)
{
	return gsm_send_at("AT+CPIN?\r\n", "CPIN", 2);
}

/*
>> ע���

AT+CREG=2

OK
AT+CREG?

+CREG: 2, 1, "25F0", "0F8C", 0

OK

>> δע��


*/
// ��鵱ǰ������ע��״����
ERR_NO gsm_check_reg(void)
{
	char	str[4+1];
	int		state;
	
	ERR_NO	ret;
	
	// ��������ע�������ṩ���ڵ�ѶϢ��CELL ID��LOCAL ID��
	ret = gsm_send_at("AT+CREG=2\r\n", "OK", 3);
	if(ret < 0)
	{	
		return ret;
    }

	// ��ѯ����ע��״��
	ret = gsm_send_at("AT+CREG?\r\n", "+CREG:", 3);
	if(ret < 0)
	{	
		return ret;
    }

	ret = gsm_find_pattern(",", 1);
	if(ret < 0)
	{	
		return ret;
    }

	ret = gsm_fetch_digits(str, 4, 1);
	if(ret < 0)
	{	
		return ret;
    }

	state = atoi(str);
	if(state == 1 || state == 5)
	{
		return OK;	// ��ע��
	}
	else
	{
		return ER_GSM_NETWORK_UNREGISTERED;	// δע��
	}
}

// ���rssi�Ƿ�������һ���ڴ���绰�������š��������Լ���ѯdns(��Ҫ������)ǰִ���Ա�ȷ��ģ�鹤��״̬������
/*
------------------------------------
	signal 		rssi
------------------------------------
0 	<4��99 		<-107 dBm or unknown
1 	<10 		<-93dBm
2 	<16 		<-71 dBm
3 	<22 		<-69dBm
4 	<28 		<-57dBm
5 	>=28 		>=-57 dBm
------------------------------------
*/
ERR_NO gsm_check_rssi(void)
{
	int 	rssi;	
	int 	i;
	int		times = 1;

	int		cnt;

CHECK_GSM_RSSI:
	cnt = 3;
	
	// �������rssiֵ���������5�򷵻�ok
	for(i = 0; i < MAX_TIEMS_GSM_GET_RSSI; i++)
	{
// rssi���Ϊ99ʱ��ÿ��3����һ�Ρ���������Σ��Ա㾡���ܵȵ�gsmע������ɹ���
GET_GSM_RSSI:
		rssi = -1;

		if(gsm_get_rssi((int*)&rssi) == OK)
		{
			printf("#%d RSSI = %d\r\n", i+1, rssi);

			// rssi=99һ�����������δע�������
			if(rssi == 99)
			{
				cnt--;

				if(cnt >0)
				{
					delay_100ms(30);	// ���3��
					
					goto GET_GSM_RSSI;
				}
				else
				{
					return -1;			// ��δ�ɹ�ע������
				}
			}
			else if(rssi <= 4)
			{
				return -2;				// �ź�̫��
			}
			else if(rssi >= MIN_RSSI_FOR_COMMUNICATION)
			{
				return OK;
			}
		}
		// else: ��ѯrssiʧ�������ز�
	}

	if(times > 0)
	{
		gsm_recover();

		times--;

		goto CHECK_GSM_RSSI;
	}

	// �ָ�gsm���ѯrssi�Բ�����������NG
	printf("failed to check rssi after once trial of recovering gsm.\r\n");
	
	return -2;
}

/**********************************************************************************************************************
										 		GSMӦ�ýӿں���
***********************************************************************************************************************/

// ����绰��ָ�����롣
/*
>> ���жԷ�(�����з��֣��źŵ͵�4ʱ�Կɽ���ͨ��)

ATD13923887347	// ���жԷ�(ͨ�������ڳ���10���ڲŽ���)

OK

SPEECH ON

ALERTING

CONNECT

SPEECH OFF		// �Է��Ҷϵ绰

RELEASE

NO CARRIER	

>> �Է�����

RING

+CLIP: "13923887347",129,"",0,"",0

RING

+CLIP: "13923887347",129,"",0,"",0

RING

+CLIP: "13923887347",129,"",0,"",0

RING

+CLIP: "13923887347",129,"",0,"",0

DISCONNECT

RELEASE

*/
ERR_NO gsm_call_out(char* pn)
{
	char			at[32];
	ERR_NO			ret;

	// ��ʽ��gsm����
	if(format_pn(pn) != OK)
	{
		return NG;
	}

	// �ȼ��rssiֵ�Ƿ�����
	ret = gsm_check_rssi();
	if(ret < 0)
	{
		return ret;
	}

	sprintf(at, "ATD%s\r\n", pn);

	ret = gsm_send_at(at, "CONNECT", 10);
	if(ret < 0)
	{
		return ret;
	}

	// ����gsmͨ��״̬��־
	is_gsm_calling = TRUE;
	
	return OK;
}

ERR_NO gsm_call_end(void)
{
	char			at[32];
	ERR_NO			ret;
	
	strcpy(at, "ATH\n");

	ret = gsm_send_at(at, "RELEASE", 10);
	if(ret < 0)
	{
		return ret;
	}

	// ����gsmͨ��״̬��־
	is_gsm_calling = FALSE;
	
	return OK;
}

/********************************* SMS ��صĲ������� *************************************/
ERR_NO gsm_sms_mode_txt(void)
{
	return gsm_send_at("AT+CMGF=1\r\n", "OK", 5);
}

ERR_NO gsm_sms_mode_pdu(void)
{
	return gsm_send_at("AT+CMGF=0\r\n", "OK", 5);
}

// ��ָ����������PDU��ʽ��SMS���͸�ָ��Ŀ�귽��
// ascii_utf8	- 	��ASCII�����Ӣ�Ĵ����Ͷ������ݻ���UTF-8��������Ĵ����Ͷ������ݡ�
ERR_NO sms_snd_pdu(T_SMS_LANGUAGE language, char* pns, char* ascii_utf8)
{
	ERR_NO			ret;	
	
	char			pdu[320];					// pdu����󳤶�һ��С��320�ֽ�

	unsigned char	encoded[MAX_LEN_CNC_SMS*MAX_NUM_CNC_SMS];	// ���ض������Ķ�������(���Ĳ���UCS2���롢Ӣ�Ĳ���7bit����)
	unsigned char*	pencoded = (unsigned char*)encoded;	
	
	char			at[32];

	int				len = 0;	
	
	int				rest = 0;	
	unsigned int 	tobesent = 0; 
	unsigned int	len_seg = 0;		// ÿ��pdu���а����Ĵ��û����ݳ���(���ĺ�Ӣ�Ĳ�ͬ)

	unsigned int	feed;

	unsigned char	udh[7];				// uder data header
	unsigned int	len_udh = 0;		// udh����(����ʹ��6�ֽ�udh��Ӣ��ʹ��7�ֽ�udh)
	int				segment_total = 0;	// �����Ų�ַ��͵�������
	int				segment_count = 0;	// �����Ų�ַ��͵ĵ�ǰ���(��1��ʼ����)	

	// ����������ַ�����ԭʼ����(δ����ǰ)
	rest = strlen(ascii_utf8);

	printf("content to be sent in pdu:%s\r\n", ascii_utf8);

	// �Ƚ����������ݸ�������������Ϊ�ض��ĸ�ʽ
	if(language == SMS_LANGUAGE_CHINESE)
	{
		rest = pdu_encode16bit(encoded, ascii_utf8, rest);	

		// �������͵Ķ����������Ƿ������utf8����Ҳ��ascii������ַ���
		if(rest <= 0)
		{
			printf("error in encoding pdu.\r\n");
			
			return ER_GSM_SMS_PDU_CHAOS;
		}

		// printf("to send %d Chinese characters.\r\n", rest/2);

		len_udh = 6;
		len_seg = MAX_BYTE_SMS_PDU - len_udh;
	}
	else if(language == SMS_LANGUAGE_ENGLISH)
	{
		// printf("to send %d English characters.\r\n", rest);
		
		rest = pdu_encode8bit((char*)encoded, (char*)ascii_utf8, rest);	

		// �������Ͷ����������Ƿ������ASCII�ַ���
		if(rest <= 0)
		{
			return ER_GSM_SMS_PDU_CHAOS;
		}

		len_udh = 7;
		len_seg = MAX_BYTE_SMS_PDU - len_udh;
	}

	// �������Ҫ��ֵ�������(ÿ����ֺ�Ķ�����󳤶�ΪMAX_BYTE_SMS_PDU��ȥudh�ĳ���6�ֽ�)
	if(rest > MAX_BYTE_SMS_PDU)
	{
		if(rest%len_seg)
		{
			segment_total = rest/len_seg+1;
		}
		else
		{
			segment_total = rest/len_seg;
		}
	}
	else	// : ���Ͷ������ݳ���С�����pdu����ʱ�����Լ������ŷ�ʽ����
	{
		segment_total = 1;
	}

	// ���ڲ���α�����
	feed = systick;

	// ��ַ��ͳ�����
	while(rest > 0)
	{			
		// ����Ƿ���Ҫ�����Ų�ַ���
		if(segment_total > 1)
		{
			if(rest > len_seg)
			{
				tobesent = len_seg;
				rest -= tobesent;
				segment_count++;
			}
			else
			{
				tobesent = rest;
				rest -= tobesent;
				segment_count++;
			}

			// ����udh(����ʹ��6�ֽ�udh���Ա�ʣ���134�ֽڿ����������������֣�Ӣ��ʹ��7�ֽ�udh���Ա�udh�����udsһ������7�ֽ���)
			if(len_udh == 6)
			{				
				udh[0] = 0x05;
				udh[1] = 0x00;
				udh[2] = 0x03;
				udh[3] = feed&0xFF;			// serial number			
				udh[4] = segment_total;		
				udh[5] = segment_count;				// �������Ŵ�1��ʼ����
			}
			else if(len_udh == 7)
			{			
				udh[0] = 0x06;
				udh[1] = 0x08;
				udh[2] = 0x04;
				udh[3] = feed&0xFF;			// serial number			
				udh[4] = (feed>>2)&0xFF;		// serial number	
				udh[5] = segment_total;		
				udh[6] = segment_count;				// �������Ŵ�1��ʼ����
			}			

			// �������Ķ������ݴ����pdu�ַ���
			if(language == SMS_LANGUAGE_CHINESE)
			{
				len = pdu_construct((char*)pdu, pns, (char*)gsm_sca, SMS_PDU_ENCODING_UCS2, pencoded, tobesent, 1, (unsigned char*)udh, len_udh);

				pencoded += tobesent;
			}
			else if(language == SMS_LANGUAGE_ENGLISH)
			{
				len = pdu_construct((char*)pdu, pns, (char*)gsm_sca, SMS_PDU_ENCODING_8BIT, pencoded, tobesent, 1, (unsigned char*)udh, len_udh);	

				pencoded += tobesent;
			}
		}
		else
		{
			tobesent = rest;
			rest -= tobesent;
			
			if(language == SMS_LANGUAGE_CHINESE)
			{
				len = pdu_construct((char*)pdu, pns, (char*)gsm_sca, SMS_PDU_ENCODING_UCS2, pencoded, tobesent, 0, (unsigned char*)NULL, len_udh);

				pencoded += tobesent;
			}
			else if(language == SMS_LANGUAGE_ENGLISH)
			{
				len = pdu_construct((char*)pdu, pns, (char*)gsm_sca, SMS_PDU_ENCODING_8BIT, pencoded, tobesent, 0, (unsigned char*)NULL, len_udh);	

				pencoded += tobesent;
			}
		}

		// ���SMSģʽ���������Ƿ�ִ�гɹ��������ɹ����������Ϊ��ǰ����SMS PDU����ģʽ����ʱ�跢��0x1B��ֹ����ģʽ
		ret = gsm_send_at("AT+CMGF=0\r\n", "OK", 2);		
		if(ret < 0)
		{
			usart3_send_int("\x1B", 1);	

			printf("error in \"AT+CMGF=0\".\r\n");
			
			return ret;
		}
		
		// ����PDUģʽ���ŵ�����AT+CMGS=������Ҫ����������PDU���ĳ���(������SCA���ֵģ���������gsm_sca_len + gsm_sca_fmt + gsm_sca_str)
		// �˳�����ֵ������ȷ������SMS���ͻ�ʧ�ܡ�
		sprintf(at, "AT+CMGS=%d\r", len); 	// ������\r����\r\nȥ����AT���������MTKЭ��ջ��Bug
		
		// ����Ƿ����SMS PDU����ģʽ����δ���룬�������Ϊ��ǰ������SMS PDU����ģʽ����ʱ��Ҫ����0x1B��ֹ����ģʽ
		ret = gsm_send_at(at, ">", 2);
		if(ret < 0)	// �ȴ�SMS PDU����ָʾ��'>'
		{
			usart3_send_int("\x1B", 1);	

			printf("error in waiting for '>'.\r\n");
			 
			return ret;
		}
		
		// ���Ͷ���
		ret = gsm_send_at(pdu, "+CMGS:", TO_SMS_TX);
		if(ret < 0)	
		{
			usart3_send_int("\x1B", 1);	// cancel SMS inputing mode so to accept following AT command 

			printf("error in sending pdu.\r\n");
			
			return ret;
		}	
	}
	
	// ������SMS��ǿ�н�SMS�շ�ģʽ����ΪTXTģʽ�����ڽ���SMS����	
	ret = gsm_sms_mode_txt();
	if(ret < 0)
	{
		printf("error in setting sms to text mode.\r\n");

		return ret;
	}
	
	return OK;
}

// ��������������TXT��ʽ��SMS���͸�ָ��Ŀ�귽(ͨ���ʵ������ı��룬���ڴ���༭�������������ַ����ɷ�������SMS)��
ERR_NO sms_snd_txt(char* pns, char* ascii)
{
	ERR_NO	ret;
	char 	at[281];

	// ���SMSģʽ���������Ƿ�ִ�гɹ��������ɹ����������Ϊ��ǰ����SMS PDU����ģʽ����ʱ�跢��0x1B��ֹ����ģʽ
	ret = gsm_send_at("AT+CMGF=1\r\n", "OK", 2);
	if(ret < 0)
	{
		usart3_send_int("\x1B", 1);		// cancel SMS inputing mode so to accept following AT command 

		printf("error in setting sms to pdu mode.\r\n");

		return ret;
	}	

	// send "AT+CMGS"
	sprintf(at, "AT+CMGS=\"%s\"\r\n", pns);

	ret = gsm_send_at(at, ">", 2);
	if(ret < 0)
	{
		usart3_send_int("\x1B", 1);		// cancel SMS inputing mode so to accept following AT command 

		printf("error in waiting for '>'.\r\n");

		return ret;
	}

	// ����TXT��ʽ��SMS���ݣ�ע�������ݺ�����Ӷ�����1A�����з�\r\n������SMS���Ͳ���ɹ�
	sprintf((char*)at, "%s\x1A\r\n", ascii);
	
	// ���Ͷ���
	ret = gsm_send_at(at, "+CMGS:", TO_SMS_TX);
	if(ret < 0)
	{
		usart3_send_int("\x1B", 1);		// cancel SMS inputing mode so to accept following AT command 

		printf("error in sending pdu.\r\n");

		return ret;
	}

	return OK;
}

// ����ָ������Ϣ��ָ���ĺ��룬�����˶���Ϣ��ַ��ͺͷ���ʧ�ܺ��Զ��ط����ܡ�
// ע: ����Ĵ�������ϢҪô��ASCII���룬Ҫô��UTF-8���룬�Ա�ͳһ��Ϊ�ַ�������
// ʹ��gsm_send_sms�������οɷ��͵����������ΪMAX_CHAR_CNC_SMS_CN���塢����
// �ɷ��͵����Ӣ���������ܴ˺궨�����ƣ������ܳ���ɷ����ջ�ռ�����(���η���269�ַ�û����)��
ERR_NO gsm_send_sms(char* pn, char* ascii_utf8)
{	
	ERR_NO			ret;

	char				pns[MAX_LEN_PN+1];			// ��׼����gsm����(������������ǰ׺)
	
	int				len	 = 0;	// �����͵��ַ�����

	T_SMS_LANGUAGE	language = SMS_LANGUAGE_ENGLISH;

	// ͨ��״̬�¿��������Ͷ���!!!

	// �����ͺ���Ϊ�գ��򽫶����ض��򵽴������
	if(strlen(pn) <= 0)
	{	
		printf("pn is null and redirect to uart:%s", ascii_utf8);

		return NG;
	}

	// �����������Ƿ�Ϊ��
	len = strlen(ascii_utf8);
	
	if(len <= 0)
	{
		printf("reply is null.\r\n");
		
		return NG;
	}

	// ��ʽ��gsm����
	if(format_pn(pn) != OK)
	{
		return NG;
	}

	// ����gsm������ӹ�������ǰ׺(����pdu����ʱĬ���趨�����ʽ��Я����������ǰ׺)��
	strcpy(pns, gsm_telecode);
	strcat(pns, pn);

	// �������������Ƿ�Ϊ����
	if(is_all_ascii(ascii_utf8) == FALSE)
	{
		language = SMS_LANGUAGE_CHINESE;
	}

	// printf("sms length = %d and is written in %d.\r\n", len, language);

	// �������͵Ķ�Ϣ�ַ��Ƿ񳬹�������������(���Ķ������ݲ���UTF8�������롢Ӣ�Ķ������ݲ���ASCII�������룬ǰ��ÿ�ַ�ռ3�ֽڡ�����ÿ�ַ�ռ1�ֽ�)
	// ���͵ĵ���������󳤶�Ϊ4����������
	if(len > MAX_LEN_CNC_SMS*MAX_NUM_CNC_SMS)
	{
		printf("max. length of a Chinese or English SMS is %d .\r\n", MAX_LEN_CNC_SMS*MAX_NUM_CNC_SMS);

		return NG;
	}
		
	gsm_wakeup();

	// ���RSSIֵ�Ƿ�Ϊ-1�����ǣ����Ϊģ��GSM�Զ��ϵ��λ����ʱ��ָ�GSMģ��
#if 1
	// �ȼ��rssiֵ�Ƿ�����
	ret = gsm_check_rssi();

	if(ret < 0)
	{
		gsm_sleep();

		return NG;
	}
#endif

	// �̶�����pdu��ʽ���Ͷ���(Ӣ����8bit���롢������ucs2����)	
	ret = sms_snd_pdu(language, pns, ascii_utf8);

	gsm_sleep();
	
	// ���ͳ����Ҵ����Ƕ�Ϣ������������£����浱ǰ����׼���ط�		
	if(ret == ER_GSM_SMS_PDU_CHAOS)	// �����Ͷ�Ϣ�����а�����֧�ֵ��ַ�����
	{
		// ���ڰ�����֧�ֵ��ַ�����Ķ��ţ������ط�
		printf("unrecognized(neither ASCII nor UTF8 encoded) character found in SMS content!\r\n");
		
		return ret;
	}
	else if(ret < 0)
	{							
		printf("failed to send sms.\r\n");

		return ret;
	}
	else
	{
		printf("Sent sms to %s.\r\n", pns);

		return OK;
	}
}


/*
>> SIM������״̬�£�����TCP���������������ӳɹ��Ļ�������5���ڣ�
   �������ʧ�ܵĻ�������18����:
   
AT+NETAPN="CMNET","",""

OK
AT+TCPSETUP=0,74.125.71.105,80

OK
+TCPSETUP:0,OK
AT+IPSTATUS=0

+IPSTATUS:0,CONNECT,TCP,8192


AT+TCPSETUP=0,74.125.71.104,80

OK
+TCPSETUP:0,FAIL

>> SIM��δ��״̬�£�����TCP��������󣬻ἴ�̷���SIM��δ����Ĵ�����Ϣ:

AT+NETAPN="CMNET","",""

OK
AT+TCPSETUP=0,74.125.71.105,80

OK
+CME ERROR: NO SIM

+TCPSETUP:0,FAIL
*/
// ����ָ����GPRS���ӺŽ�����Ӧ��GPRS���ӡ�
// 1��M660+�����ֽ׶��Ե���?��쳣���󣬼�ĳ�ο��������״�GPRS���������Ļ���
//	  ���������������ĸ��ʺܴ󣬷�֮��Ȼ�����ĳ�ο�����������GPRS���Ӳ�������
//	  �Ϻõİ취�ǹر�ģ�鹩��Ȼ���ϵ�;
// 2��GPRS���ӹ���������������룬��GPRS����һ���ʧ�ܣ�
//    GPRS�Ѿ�����ʱ����������룬��GPRS���Ӳ���Ͽ�;
//    ͨ�������н���GPRS���ӣ���һ���ʧ��;

//modified by double lin
const u8 *modetbl[2]={"TCP","UDP"};//����ģʽ

int sim900a_tcpudp_test(u8 mode,u8* ipaddr,u8* port,u8 id)
{ 
	char	at[64];

	if(mode)printf("SIM900A UDP���Ӳ���\n");
	else printf("SIM900A TCP���Ӳ���\n"); 

	//sprintf(at,"AT+CIPSTATUS=?\r\n");
	//if(gprs_soc_status(id) != OK)
	{
		sprintf(at,"AT+CIPSTART=\"%s\",\"%s\",\"%s\"\r\n",modetbl[mode],ipaddr,port);
		if(gsm_send_at(at, "OK", 5) != OK)return NG;		//��������
	}
	return OK;
}
int gprs_soc_setup_dns(void)
{
	char	at[64];
	int ret;
	sprintf(at, "AT+CIPSHUT\r\n");	
	ret=gsm_send_at(at, "SHUT OK", 1) ;	 
	if(ret!= OK)		// to = 10 ��Ҫ����
	{
		printf("failed to CIPSHUT.\r\n");
		
		return NG;
	}


	sprintf(at, "AT+CIPMUX=0\r\n");	
	ret=gsm_send_at(at, "OK", 1);
	if(ret!= OK)		// to = 10 ��Ҫ����
	{
		printf("failed to CIPMUX.\r\n");
		
		return NG;
	}


	sprintf(at, "AT+CIPRXGET=1\r\n");	
	ret=gsm_send_at(at, "OK", 1);
	if(ret!= OK)		// to = 10 ��Ҫ����
	{
		printf("failed to CIPRXGET.\r\n");
		
		return NG;
	}


	sprintf(at, "AT+CIPQRCLOSE=1\r\n");	
	ret=gsm_send_at(at, "OK", 1);
	if(ret!= OK)		// to = 10 ��Ҫ����
	{
		printf("failed to CIPQRCLOSE.\r\n");
		
		return NG;
	}


	sprintf(at, "AT+CIPMODE=0\r\n");	
	ret=gsm_send_at(at, "OK", 1);
	if(ret!= OK)		// to = 10 ��Ҫ����
	{
		printf("failed to set CIPMODE.\r\n");
		
		return NG;
	}


	sprintf(at, "AT+CIPSTART=\"TCP\",\"bin172133.oicp.net\",8080\r\n");	
	ret=gsm_send_at(at, "OK", 1);
	if(ret!= OK)		// to = 10 ��Ҫ����
	{
		printf("failed to set CIPSTART.\r\n");
		
		return NG;
	}
	return OK;
}
//modified by double lin
int gprs_soc_setup(int id)
{
	char	at[64];
//	char	ptn[14];
//	char	cgclass='B';
	int		errors = 0;
//	const u8 *port="8086";	//�˿ڹ̶�Ϊ8086,����ĵ���8086�˿ڱ���������ռ�õ�ʱ��,���޸�Ϊ�������ж˿�
	u8 mode=0;				//0,TCP����;1,UDP����


	printf("to setup gprs connection to %s:%s...\r\n", (char*)tcp_conn[id].ip, (char*)tcp_conn[id].port);
	/// printf("to setup GPRS connection.\r\n");

	// ͨ��״̬�²��ܽ���gprs����
	if(is_gsm_calling == TRUE)
	{
		printf("gsm is in calling and can not send data!\r\n");
		
		return NG;
	}

	// �����������ӵ�GPRS���ò�����IP��ַ�Ƿ�Ϊ�գ�Ϊ�յĻ���ִ����������
	if(strlen(tcp_conn[id].ip) <= 0)
	{	
		if(strlen(tcp_conn[id].dn) > 0)
		{
			if(gsm_get_dns((char*)tcp_conn[id].ip, MAX_LEN_GPRS_IP, (char*)tcp_conn[id].dn) != OK)
			{
				printf("failed to convert %s into IP.\r\n", tcp_conn[id].dn);

				return NG;
			}
			else
			{
				printf("resolved ip is:%s\r\n", (char*)tcp_conn[id].ip);
			}
		}
		else
		{
			printf("both IP and domain name are empty.\r\n");
			
			return NG;
		}
	}
	//�ر�����
	sprintf(at, "AT+CIPCLOSE=1\r\n");	
	gsm_send_at(at, "CLOSE OK", 1);
	
	//�ر��ƶ�����
	sprintf(at, "AT+CIPSHUT\r\n");	
	gsm_send_at(at, "SHUT OK", 1) ;
	
	//�����ƶ�̨���
	sprintf(at, "AT+CGCLASS=\"B\"\r\n");	
	if(gsm_send_at(at, "OK", 10) != OK)		// to = 10 ��Ҫ����
	{
		printf("failed to set CGCLASS.\r\n");
		
		return NG;
	}
	//����PDP������
	sprintf(at, "AT+CGDCONT=1,\"IP\",\"CMNET\"\r\n");
	if(gsm_send_at(at, "OK", 10) != OK)		// to = 10 ��Ҫ����
	{
		printf("failed to set CGDCONT.\r\n");
		
		return NG;
	}
	//���ø���GPRSҵ��
	sprintf(at, "AT+CGATT=1\r\n");
	if(gsm_send_at(at, "OK", 5) != OK)		// to = 5 ��Ҫ����
	{
		printf("failed to set CGATT.\r\n");
		
		return NG;
	}
	//����ΪGPRS����ģʽ
	sprintf(at, "AT+CIPCSGP=1,\"CMNET\"\r\n");
	if(gsm_send_at(at, "OK", 5) != OK)		// to = 5 ��Ҫ����
	{
		printf("failed to set CIPCSGP.\r\n");
		
		return NG;
	}
	//���ý���������ʾIPͷ(�����ж�������Դ)
	sprintf(at, "AT+CIPHEAD=1\r\n");
	if(gsm_send_at(at, "OK", 5) != OK)		// to = 5 ��Ҫ����
	{
		printf("failed to set CIPHEAD.\r\n");
		
		return NG;
	}

//	return sim900a_tcpudp_test(mode,(u8 *)tcp_conn[id].ip,(u8 *)tcp_conn[id].port,id);
	if(sim900a_tcpudp_test(mode,(u8 *)tcp_conn[id].ip,(u8 *)tcp_conn[id].port,id) !=OK) return NG;
	// ����APN	"AT+NETAPN=%s,\"\",\"\"\r\n"
//	sprintf(at, "AT+NETAPN=\"%s\",\"\",\"\"\r\n", gsm_apn);	//SIM900A AT+CSTT
//	
//	if(gsm_send_at(at, "OK", TO_GPRS_SET_APN) != OK)		// to = 2
//	{
//		printf("failed to set APN.\r\n");
//		
//		return NG;
//	}

	// printf("APN set.\r\n");
		
//	sprintf(ptn, "+TCPSETUP:%d,OK", id);	// ��ǿ�жϣ�����������OK��FALL
	
	// ����GPRS����
//	sprintf(at, "AT+TCPSETUP=%d,%s,%s\r\n", id, tcp_conn[id].ip, tcp_conn[id].port); //SIM900A  AT+CIPSTART

	
	while(errors < (MAX_TIMES_GPRS_SETUP_CONNECTION+2) )//�ȴ�4 X 5 = 20s  modified by double lin
	{
		if(gprs_soc_status(id) != OK)
		{
			//printf("#%d: failed to setup GPRS connection to %s:%s.\r\n", errors+1, tcp_conn[id].ip, tcp_conn[id].port);

			errors++;

			//delay_100ms(10);
		}
		else
		{
			printf("GPRS conenction setup.\r\n");
			return OK;
			//break;
		}
	}

	return NG;
//
//	if(errors >= MAX_TIMES_GPRS_SETUP_CONNECTION)
//	{
//		return NG;	
//	}
//	else
//	{
//		// printf("successfully setup GPRS connection to %s:%s.\r\n", tcp_conn[id].ip, tcp_conn[id].port);
//
//		return OK;	
//	}	
}

/*
AT+NETAPN="CMNET","",""

OK
AT+TCPSETUP=0,74.125.71.105,80

OK
+TCPSETUP:0,OK
AT+IPSTATUS=0

+IPSTATUS:0,CONNECT,TCP,8192
AT+TCPCLOSE=0

+TCPCLOSE: 0,OK
AT+IPSTATUS=0

+IPSTATUS:0,DISCONNECT


AT+TCPCLOSE=0

+TCPCLOSE: ERROR
*/
// ����ָ����GPRS���ӺŹر���Ӧ��GPRS���ӡ�

//modified by double lin
int gprs_soc_close(int id)
{
	char	at[64];

	sprintf(at, "AT+CIPCLOSE=1\r\n");		//SIM900A AT+CIPCLOSE

	if(gsm_send_at(at, "CLOSE OK", 2) != OK)
	{
		return NG;
	}
	sprintf(at, "AT+CIPSHUT\r\n");
	if(gsm_send_at(at, "SHUT OK", 2) != OK)
	{
		return NG;
	}
	return OK;
}

/*
AT+IPSTATUS=0
+IPSTATUS:0,CONNECT,TCP,1500

AT+IPSTATUS=1
+IPSTATUS:1,DISCONNECT
*/
// ���ָ��GPRS���ӵ�����״̬��������ֵΪOK����Ϊ����״̬������Ϊδ����״̬��
// ע: GPRS�����Զ����ֳ�ʱʱ��Ϊ30���ӣ���GPRS���ӿ��г���30���ӣ�GSM��վ�Ὣ��Ͽ���

//modified by double lin
int  gprs_soc_status(int id)
{
	char	at[64];
	sprintf(at, "AT+CIPSTATUS\r\n");	//SIM900A AT+CIPSTATUS
	if(gsm_send_at(at, "OK", 5) != OK) return NG;
	else return OK;
	
	/*
	char	        at[16];
	char			ptn[16];
	char			sts[16+1];

	printf("to check GPRS #%d connection status.\r\n", id);

	sprintf(at, "AT+CIPSTATUS=%d\r\n", id);	//SIM900A AT+CIPSTATUS

	sprintf(ptn, "+IPSTATUS:%d",id);

    // �ȼ�ⷴ����Ϣ����Ϣͷ
    if(gsm_send_at((char*)at, (char*)ptn, 3) != OK)
    {
		return NG;
    }

	if(gsm_fetch_spliters(',', sts, 16, 1) != OK)
	{
		return NG;
	}

	printf("GPRS #%d Status = %s\r\n", id, sts);
	
	if(!__strcmp(sts, "CONNECT"))
	{
		// printf("gprs conenction is conencted.\r\n");
		
		return OK;
	}
	else
	{
		// printf("gprs conenction is disconencted.\r\n");
		
		return NG;
	}
	*/
}

/*
AT+TCPSETUP=0,183.12.149.225,8080

OK
+TCPSETUP:0,OK
AT+DATAFORMAT=1,1

OK
AT+TCPSEND=0,4

>						// ģ�鷢��1234���������յ�1234
OK
+TCPSEND:0,4

+TCPRECV: 0,4,4321		// ����������4321��ģ���յ�4321

+TCPRECV: 0,8,abcd1234	// ����������abcd1234��ģ���յ�abcd1234
*/
// �ڷ�͸��ģʽ�´�ָ����GPRS�����Ϸ��͸������ȵ����ݣ�
// ������ʵ�ʳɹ����͵����ݳ���(�����Զ��ָ�������ͻ���)��

void Delay_Ms(u16 myMs)
{
  u16 i;
  while(myMs--)
  {
    i=7200;
    while(i--);
  }
}


//modified by double lin
unsigned int gprs_soc_tx(int id, unsigned char* data, unsigned int size)
{
	unsigned char*	ptr 			= data;
	//strcat(ptr,"\r\n");
//	char 			tmp[MAX_LEN_GPRS_PACKET_SEND];
	int			  	n_tobesent 	= 0;		// ÿ�δ����͵��ֽ���
	int 			n_reported		= 0;		// ÿ�η��ͺ�GSMģ�鷴���ķ��ͳɹ��ֽ���
	int				n_sentout 		= 0;		// �ۼƷ��ͳɹ����ֽ���

	int				repeat  		= MAX_TIMES_GPRS_SEND_PACKET;    // repat times to sent the identical packet

	char			at[32];						// to accomodate the command header																		

//	char			ptn[16];	

	unsigned int 	to;

	// ���մ������ֽ���������ÿ֡���ݵĳ�ʱ�����ܷ��ͳ�ʱʱ�䡣
	to = ((size/MAX_LEN_GPRS_PACKET_SEND)+((size%MAX_LEN_GPRS_PACKET_SEND)?1:0))*(TO_GPRS_TX_FRAME+1);

	printf("to send %d bytes within %d seconds.\r\n", size, to);

	// ͨ��״̬�²��ܷ���gprs����
	if(is_gsm_calling == TRUE)
	{
		printf("gsm is in calling and can not send data!\r\n");
		
		return NG;
	}

	// ����GPRS���ݰ�֮ǰ���Թ���ǰ���յ���δ������������
	RxCnt3_rd = RxCnt3_wr;

	to = systick+to*1000/SYSTICK_PERIOD;

	// ��֡����GPRS����
	while(size > 0 && systick < to)
	{     
	    n_reported = 0;    

        // ÿ�η��͵����ݳ��Ȳ��ܳ�����AT+SOCSEND���������������ݳ���(=496�ֽ�)
		n_tobesent = size > MAX_LEN_GPRS_PACKET_SEND ? MAX_LEN_GPRS_PACKET_SEND : size;

		// ������Ϣͷ	:��\r\n
		sprintf(at, "AT+CIPSEND\r\n");	  //SIM900A CIPSEND

		if(gsm_send_at(at, ">", 5) != OK)//2
		{
			printf("send error\n");
			return n_sentout;
		}

		usart3_send_int((char*)ptr, n_tobesent);
		//sprintf(at, "0X1A\r\n");
		//if(gsm_send_at(at,"SEND OK", 1) != OK)//2
		//{
		//	return n_sentout;
		//}
		Delay_Ms(6);
		sprintf(at,"%s\r\n",(u8*)0X1A);
		if(gsm_send_at(at, "SEND OK", 4) != OK)//2
		{
			//sprintf(ptn, "SEND FAIL");	//��鷵���������Ƿ���SEND FAIL  �������˵������ʧ��			 
			//if(gsm_find_pattern(ptn, TO_GPRS_TX_FRAME) == OK)
			//{
			//	return n_sentout;
			//}

			return n_sentout;
		}
		

		// �ȴ����ٽ��յ�һ���ֽڵĺ�������
		/*
       	if(!gsm_wait_output(1, 2))
        {
        	return n_sentout;
        }

		// ��ⷵ�ص��ѳɹ����͵������ֽ���
		if(gsm_fetch_value((int*)&n_reported, 2) != OK)
		{
			return n_sentout;
		}
		*/
        // �Ƚ��������͵����ݳ��Ⱥ�ʵ�ʷ��͵����ݳ����Ƿ�һ��
        // ע: ���ڷ����ķ����ֽ����������������͵��ֽ���ʱ���ط�����Ϊ���������
        // ������PRS�����ڴ��������ͻȻ��ò��ȶ�ʱ(��GSM�ź�ͻȻ�жϻ�Է���������ʱ崻���)��
        // �ڴ�֮ǰ�������쳣�����ΪGSMӲ��/��������ش���ϣ����û��Ҫ�����ط�(�Ὣδ���ͳɹ�
        // �����ݼ��뵽ȫ�ֵ��ط����ݶ���������ط�)��
        n_reported = n_tobesent;
		if(n_reported == n_tobesent)
		{
			size        -= n_reported;		// ʣ�����ݵĳ��ȵݼ�
	        ptr         += n_reported;		// ��ָ�����
	        n_sentout   += n_reported;		// �ۼƷ����ֽ�������
            
            repeat       = MAX_TIMES_GPRS_SEND_PACKET; 	// Ϊ��һ������֡�ķ��������ط����������� 
		}
		// �ոշ��͵����ݳ��Ȳ����ڴ��������ݳ���ʱ�������ط���ǰ����֡���ط�������ϵͳ�趨
		else
		{		            
			if(!(--repeat))
			{
				return n_sentout;
			}
		}
	}

	return n_sentout;	
}

// ���������gprs_soc_tx()������װ�ɴ��ط����������㺯�����������Զ��ط�
// ����(�Ƿ��ط�ȡ�����ⲿ���ú���������)����·�Զ��������(��·��Ч���ؽ���
// �����ؽ���·)��

//modified by double lin
int gsm_send_data(int id, unsigned char* data, unsigned int size)
{
	volatile unsigned int systick1 = 0;
	volatile unsigned int systick2 = 0;

	int			ret;
	int			errors = 0;

#if 1
	// �ȼ��rssiֵ�Ƿ�����
	ret = gsm_check_rssi();
	if(ret < 0)
	{
		return -1;
	}
#endif

#if 1
	// ���ָ����GPRS�����Ƿ���Ч������Ч������֮
	ret = gprs_soc_status(id);
#endif

	// printf("step out gprs_soc_status().\r\n");

	// ��ָ��GPRS���ӵ�ǰ���ڶϿ�״̬����������(���Ӻ����ڲ�����������������Σ����ζ�ʧ�ܲŷ��س�����)
	if(ret != OK)
	{
		printf("ָ��GPRS���ӵ�ǰ���ڶϿ�״̬����������\n");	
		systick1 = systick;
#if 0
		ret = gprs_soc_setup(id);
#endif
#if 1
		ret = gprs_soc_setup_dns();
#endif
		systick2 = systick;		

		// ����������ʧ�ܣ��򽫴���������д��Flash���Դ��ط�
		if(ret != OK)
		{
			printf("failed to set up GPRS connection within %d ms.\r\n", (systick2-systick1)*SYSTICK_PERIOD);
			
			return -2;		// ����ʧ��
		}	
		else
		{
			printf("it took %5d ms to set up GPRS connection.\r\n", (systick2-systick1)*SYSTICK_PERIOD);
		}
	}

	// �����������ݣ����ͳ���(��ȫ���ͳɹ�����ɹ�)��������������Ρ�
	while(errors < MAX_TIMES_GPRS_SEND_DATA)
	{			
		systick1 = systick;
		ret = (int)gprs_soc_tx(id, data, size);
		systick2 = systick;
		
		if(ret != size)
		{		
			errors++;	

			// ���η���֮����һ��ʱ��
			delay_100ms(5);
		}
		else
		{
			printf("it took %5d ms to send out %5d bytes data.\r\n", (systick2-systick1)*SYSTICK_PERIOD, size);

			// ���ط��͵����ݳ���(ʵ��Ϊ����)
			return size;
		}
	}

	return -3;			// ����ʧ��
}

// ��ָ����GPRS�����Ͻ������ݡ�
// һ���ڷ������ݺ�ŵ��ý������ݵĺ�����
// +TCPRECV: 0,8,abcd1234	// ����������abcd1234��ģ���յ�abcd1234

//modified by  lei
ERR_NO gsm_recv_data(int id, unsigned char* data, unsigned int to)
{
//	char		str[16+1];
	int		size = 0;
	int		i;
	int*     length=NULL;
	char	pattern[10];

	assert_param(to < 5);

	printf("to receive data on GPRS...\r\n");

	sprintf(pattern, "+IPD,");

	RxCnt3_rd = RxCnt3_wr;

#if 0
	// �ȵȴ�����10�ֽ�����(TCP�������ݰ��İ�ͷ"+TCPRECV: "��ռȥ��10�ֽڳ���)
	if(gsm_wait_output(10, to) != OK)
	{
		printf("failed to receive 10 bytes within %d seconds.\r\n", to);

		return -1;
	}
#endif

	// �ȴ����շ�������
	if(gsm_wait_output(7, to) != OK)
	{
		printf("faield to wait for 7 bytes within 10s.\n");

		return -1;
	}

	// �ȴ����ղ���λ��TCP�������ݰ��İ�ͷ��
	if(gsm_find_pattern(pattern, 5) != OK)
	{
		printf("failed to find %s.\r\n", pattern);
	
		return -2;
	}

	// ��ȡTCP���ݰ������ݳ�����Ϣ�
	/*
	if(gsm_fetch_spliters(',', str, 16, 2) != OK)
	{
		printf("failed to find ',' pair.\r\n");
		
		return -3;
	}
	*/
	gsm_fetch_value(length, 2);
//	size = __atoi(str);
	size = *length;

	printf("%d bytes received.\r\n", size);

	if(size <= 0 || size > MAX_LEN_GPRS_FRAMERX)
	{
		printf("length of tcp packet is invalid.\r\n");
		
		return -4;
	}	

	// usart3_redir_usart1();

	// ��ȡTCP���ݰ����������ݡ�
	RxCnt3_rd++;//����":"
	i = 0;
	
	while(i < size)
	{
		if(RxCnt3_rd < RxCnt3_wr)
		{
			data[i++] = GSM_RX_RD(RxCnt3_rd++);
		}
	}

	printf("\r\n");

	// ���ؽ��յ������ݵĳ���
	return size;		
}

#endif

