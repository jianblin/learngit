#include "example.h"
#include "common.h"
#include "log_cmd.h"
#include "log_pos.h"
#include "log_queue.h"

#include "mcu_systick.h"

#include "stdio.h"
#include "string.h"

// �������Ļ�Ӣ�Ķ��ţ���main��������Ӧ������
void example_send_sms(void)
{
#if 0
	// "��Ӣ�Ļ�϶���/Chinese and English mixed sms."
	gsm_send_sms(TEST_PHONE_NUMBER, "\xE4\xB8\xAD\xE8\x8B\xB1\xE6\x96\x87\xE6\xB7\xB7\xE5\x90\x88\xE7\x9F\xAD\xE4\xBF\xA1\x2F\x43\x68\x69\x6E\x65\x73\x65\x20\x61\x6E\x64\x20\x45\x6E\x67\x6C\x69\x73\x68\x20\x6D\x69\x78\x65\x64\x20\x73\x6D\x73\x2E");
#endif
}

// ִ��gps��λ������λ����Զ��ŷ�ʽ���ظ�ָ���ֻ����롣
void example_gps_pos(void)
{
	char	reply[MAX_LEN_CMD_REPLY+1];
	
	if(pos_via_gps(60) == OK)
	{		
		sprintf(reply, "lat=%3.6f, lon=%3.6f, spd=%4.2f\r\n", de_gpspos.lat, de_gpspos.lon, de_gpspos.spd);

#if 0	
		gsm_send_sms(TEST_PHONE_NUMBER, (char*)reply);
#endif

		printf("gps fixed.\r\n");
	}
	else
	{
		printf("gps not fixed.\r\n");
	}
}

// �����������������ɫLED��
void example_ctrl_led(void)
{
	char	reply[MAX_LEN_CMD_REPLY+1];

	send_command_virtual(CMD_SRC_COM, "kgled,0,1#", "", "", (char*)reply);

	// gsm_send_sms(TEST_PHONE_NUMBER, reply);
}

// ����ASCII��Ϣ�����Է��������ԡ�
// ע: ��������ʹ��gsm_recv_data()������
void example_con_server(void)
{
//	int		ret;
//	char		message[] ="hello, the world!";

#if 0
	// ÿ��ʹ��gsmģ��ǰǿ�ƻ��ѡ�
	gsm_wakeup();

	ret = gsm_send_data(GPRS_ID0,  (unsigned char*)message, strlen(message)+1);

	if(ret != (strlen(message)+1))
	{
		printf("failed to send message to server.\r\n");
	}
	else
	{
		printf("successfully sned message to server.\r\n");
	}

	// ÿ��ʹ����gsm��ǿ�������Խڵ硣
	gsm_sleep();	
#endif
}

// ���ڶ�λ��
void example_periodic_pos(void)
{
	int				len = 0;		
	char				echo[128];

	// ���ʹ����ڶ�λģʽ�����⴮�����
	send_command_virtual(CMD_SRC_COM, "zqdw,1#", "", "", (char*)echo+len);
}
