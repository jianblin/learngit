/* GPSӲ������NMEA��Ϣ����*/
#include "dev_gps.h"
#include "log_pos.h"
#include "mcu_usart.h"
#include "mcu_gpio.h"
#include "mcu_systick.h"
#include "string.h"
#include "stdlib.h"
#include "log_time.h"
#include "log_motion.h"

#ifdef USING_DEV_GPS

STATUS	sts_gps_power;		// GPS����״̬

BOOL	is_gps_fixed;		// GPS�Ƿ��Ѿ���λ

BOOL	is_rmc_received;	// rmc��Ϣ�Ƿ��������յ�
BOOL	is_gsa_received;		// gsa��Ϣ�Ƿ��������յ�

char	gps_msg_rmc[MAX_LEN_GPS_RMC+1];	// $GPRMC��Ϣ���������ȡ�γ�ȡ��ٶȡ����εȣ�ʵ����󳤶�ԼΪ71�ֽ�
char	gps_msg_gsa[MAX_LEN_GPS_GSA+1];	// $GPGSA��Ϣ����������DOP��Ϣ����λģʽ�����ڶ�λ�����Ǳ��(���12��)�ȣ�ʵ����󳤶�Ϊ64�ֽ�

// �ر�GGA��Ϣ���
const unsigned char 	gps_cmd_turnoff_gga[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24};

// �ر�GLL��Ϣ���
const unsigned char 	gps_cmd_turnoff_gll[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};

// �ر�GSV��Ϣ���
const unsigned char 	gps_cmd_turnoff_gsv[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};

// �ر�VTG��Ϣ���
const unsigned char 	gps_cmd_turnoff_vtg[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47};

// ������
const unsigned char	gps_cmd_coldstart[12] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0xFF, 0x02, 0x00, 0x0E, 0x61};

void clr_gpspos(void);

void gps_send_cmd(unsigned char* cmd, unsigned int size);

// gps��ʼ����
void gps_init(void)
{	
	// �ϵ���ʼ��GPSʱ��GPSǿ�жϵ��Ա�ȷ���ϵ�ʱGPS������ȷ�ĵ�Դ����״̬
	gps_power_down();	

	// ��ʼ��gps���ڽ�����ر����ͻ���
	RxCnt2_rd = 0;
	RxCnt2_wr = 0;

	TxCnt2_rd = 0;
	TxCnt2_wr = 0;

	// ��ʼ��rmc��Ϣ��gsa��Ϣ������ر���
	strcpy(gps_msg_rmc, "");
	strcpy(gps_msg_gsa, "");

	is_rmc_received 	= FALSE;
	is_gsa_received 	= FALSE;

	is_gps_fixed 	 	= FALSE;

	// ���gps��λ��Ϣ���ջ���
	clr_gpspos();	

	printf("GPS initialized.\r\n");
}

// ��gpsоƬ���硣
void gps_power_up(void)
{	
	MCU_GPIO_LOW(GPIO_GPS_PWR);

	// ��gps��Դ��Ӧ�ȴ�һ��ʱ���Ա�gpsоƬ�����ʼ�����У��������ȥ�ĵ�һ������ᷢ��ʧ�ܡ�
	delay_100ms(5);

	sts_gps_power = ON;

	// �ر�rmc��gsa֮�������Ĭ����Ϣ������Լ���gps�����ж�������
	gps_send_cmd((unsigned char*)gps_cmd_turnoff_gga, 16);

	#ifndef TESTING_GPS_SV
	gps_send_cmd((unsigned char*)gps_cmd_turnoff_gsv, 16);
	#endif
	
	gps_send_cmd((unsigned char*)gps_cmd_turnoff_gll, 16);
	gps_send_cmd((unsigned char*)gps_cmd_turnoff_vtg, 16);	

	printf("GPS powered up.\r\n");
}

// �ر�gpsоƬ���硣
void gps_power_down(void)
{	
	MCU_GPIO_HIGH(GPIO_GPS_PWR);

	delay_100ms(3);

	sts_gps_power = OFF;

	printf("GPS powered down.\r\n");
}

// �������gpsоƬ��
void gps_send_cmd(unsigned char* cmd, unsigned int size)
{
	usart2_tx_fill((unsigned char*)cmd, size);

	usart2_tx_start();	

	printf("gps command sent with %d bytes.\r\n", size);
}

// ���������������GPS(�����RAM�б������ʷ��������λʱ�ӵ�)��
void gps_coldstart(void)
{
	unsigned char	gps_cmd_coldstart[12]={0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0xFF, 0x02, 0x00, 0x0E, 0x61};
	
	usart2_tx_fill((unsigned char*)gps_cmd_coldstart, 12);

	usart2_tx_start();	

	printf("cold start gps.\r\n");
}

// ��GPS�Ѷ�λ����½���RMC��Ϣ��
// ����: $GPRMC,172149.00,A,2234.03418,N,11402.72149,E,0.275,17.16,250113,,,A*55
int gps_ana_rmc(void)
{	
	char	str[16];

	int 	i = 0;
	int		j = 0;

	int		len = 0;
	
	float	val = 0.0;	

	len = strlen(gps_msg_rmc);

	// �����Ϣ�ַ����ж��ŵĸ����Ƿ�Ϲ�
	if(cal_char_num(gps_msg_rmc, ',') != 12)
	{
		return NG;
	}
	
	// �Թ���һ������
	for(i = 0; i < len; i++)
	{
		if(gps_msg_rmc[i] == ',')
		{
			break;
		}
	}

	// ָ�򶺺ŵ���һ���ַ�
	i++;

	// utc time
	__strncpy(str, gps_msg_rmc+i, 2);
	i += 2;
	if(is_float_digit(str) != OK)
	{
		return NG;
	}	
	de_gpspos.utc[3] = (unsigned char)atoi(str);

	__strncpy(str, gps_msg_rmc+i, 2);
	i += 2;
	if(is_float_digit(str) != OK)
	{
		return NG;
	}
	de_gpspos.utc[4] = (unsigned char)atoi(str);

	__strncpy(str, gps_msg_rmc+i, 5);
	i += 5;
	if(is_float_digit(str) != OK)
	{
		return NG;
	}
	de_gpspos.utc[5]= atof(str);

	while(gps_msg_rmc[i++] != ',');
	
	// fix status
	__strncpy(str, gps_msg_rmc+i, 1);
	i += 1;
	de_gpspos.sts = (!strcmp(str, "A")?TRUE:FALSE);

	while(gps_msg_rmc[i++] != ',');

	// latitude
	__strncpy(str, gps_msg_rmc+i, 2);
	i += 2;
	if(is_float_digit(str) != OK)
	{
		return NG;
	}
	val = (float)atoi(str);

	__strncpy(str, gps_msg_rmc+i, 8);
	i += 8;
	if(is_float_digit(str) != OK)
	{
		return NG;
	}
	val += atof(str)/60;

	de_gpspos.lat = val;

	while(gps_msg_rmc[i++] != ',');

	// N/S(��γΪ������γΪ��)
	__strncpy(str, gps_msg_rmc+i, 1);
	i += 1;
	de_gpspos.lat *= (!strcmp(str, "N")?1:-1);

	while(gps_msg_rmc[i++] != ',');

	// longitude
	__strncpy(str, gps_msg_rmc+i, 3);
	i += 3;
	if(is_float_digit(str) != OK)
	{
		return NG;
	}
	val = (float)atoi(str);

	__strncpy(str, gps_msg_rmc+i, 8);
	i += 8;
	if(is_float_digit(str) != OK)
	{
		return NG;
	}
	val += atof(str)/60;

	de_gpspos.lon = val;

	while(gps_msg_rmc[i++] != ',');

	// ����GPS��λ�õ��ľ��ȼ����ն�������ʱ��
	// g_time_zone = longitude2timezone(de_gpspos.lon);

	// E/W(����Ϊ��������Ϊ��)
	__strncpy(str, gps_msg_rmc+i, 1);
	i += 1;
	de_gpspos.lon *= (!strcmp(str, "E")?1:-1);

	while(gps_msg_rmc[i++] != ',');

	// speed
	j = 0;
	while(gps_msg_rmc[i] != ',')
	{
		str[j++] = gps_msg_rmc[i++];
	}
	str[j++] = '\0';

	if(is_float_digit(str) != OK)
	{
		return NG;
	}
	de_gpspos.spd = 1.852*atof(str);	// RMC��Ϣ�е��ٶȵ�λΪ�ڣ���Ҫת��Ϊkm/h��λ
	
	while(gps_msg_rmc[i++] != ',');

	// course
	j = 0;
	while(gps_msg_rmc[i] != ',')
	{
		str[j++] = gps_msg_rmc[i++];
	}
	str[j++] = '\0';
	if(is_float_digit(str) != OK)
	{
		return NG;
	}

	de_gpspos.cog = atof(str);
	
	while(gps_msg_rmc[i++] != ',');

	// date
	__strncpy(str, gps_msg_rmc+i, 2);
	i += 2;
	if(is_float_digit(str) != OK)
	{
		return NG;
	}
	de_gpspos.utc[2] = (unsigned char)atoi(str);

	__strncpy(str, gps_msg_rmc+i, 2);
	i += 2;
	if(is_float_digit(str) != OK)
	{
		return NG;
	}
	de_gpspos.utc[1]= (unsigned char)atoi(str);

	__strncpy(str, gps_msg_rmc+i, 2);
	i += 2;
	if(is_float_digit(str) != OK)
	{
		return NG;
	}
	de_gpspos.utc[0] = atof(str);	

	// ����Ƿ���Ҫ����GPSʱ��ͬ��ϵͳʱ��
	if(sw_sync_by_gps == ON)
	{
		// ��utcʱ��ͬ��ϵͳʱ��
		sync_systime(de_gpspos.utc, SYNC_BY_GPS);	
	}

	// ������RMC��Ϣ����ʱ����de_gpspos�ṹ���е�sts��λ��Ӧ�ȵ�GSA��Ϣ���յ�����ȡ��HDOP��Ϣ������λ

	return OK;
}

// GPS�Ѿ���λ�����GSA��Ϣ(Ŀǰ����ȡHDOP��Ϣ)��������ȡ������ִ�н��(�ɹ���ʧ��)��
// ����: $GPGSA,A,3,23,29,07,08,09,18,26,28,,,,,1.94,1.18,1.54*0D
int gps_ana_gsa(void)
{
	int 	i = 0;
	int		j = 0;
	int		k = 0;
	
	int		commas = 0;
	
	char	str[12];
	
	int 	len = strlen(gps_msg_gsa);

	// �����Ϣ�ַ����ж��ŵĸ���
	if(cal_char_num(gps_msg_gsa, ',') != 17)
	{
		return NG;
	}

	// ��ָ���ַ���ĩβ�ַ�
	i = len-1;

	while(i--)
	{
		// ���*λ��
		if(gps_msg_gsa[i] == '*')
		{
			break;
		}
	}

	// ��δ��⵽*������NG
	if(i <= 0)
	{
		de_gpspos.hdop = 0.00;
		
		return NG;
	}

	// ��⵹��2�����ŵ�λ�ò���¼
	while(i--)
	{
		if(gps_msg_gsa[i] == ',')
		{
			commas++;

			if(commas == 1)
			{
				j = i;
			}
			else if(commas == 2)
			{
				k = i;

				break;
			}
		}
	}

	// ��δ��⵽2�����Ż�2������֮��ľ��볬��5������NG
	if((i <= 0) || ((j-k) > 5))
	{
		de_gpspos.hdop = 0.00;
		
		return NG;
	}

	// ��ȡ��������֮����ַ�
	i = 0;
	k += 1;
	
	while(k < j)
	{
		str[i++] = gps_msg_gsa[k++];
	}

	str[i++] = '\0';

	len = strlen(str);

	// ���HDOP���Ƿ�Ϊ��
	if(len <= 0)
	{
		// printf("HDOP domain is empty.\r\n");

		de_gpspos.hdop = 0.00;
		
		return NG;
	}

	// ��HDOP�ַ���ת��Ϊ������
	de_gpspos.hdop = atof(str);

	// ��ȡ��GSA��Ϣ�е�HDOP��Ϣ��de_gpspos�ṹ���еĶ�Ϊ��־��λ
	de_gpspos.sts = TRUE;

	return OK;
}

#endif
