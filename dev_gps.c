/* GPS硬件管理及NMEA消息解析*/
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

STATUS	sts_gps_power;		// GPS供电状态

BOOL	is_gps_fixed;		// GPS是否已经定位

BOOL	is_rmc_received;	// rmc消息是否完整接收到
BOOL	is_gsa_received;		// gsa消息是否完整接收到

char	gps_msg_rmc[MAX_LEN_GPS_RMC+1];	// $GPRMC消息，包含经度、纬度、速度、海拔等，实际最大长度约为71字节
char	gps_msg_gsa[MAX_LEN_GPS_GSA+1];	// $GPGSA消息，包含三种DOP信息、定位模式、用于定位的卫星编号(最多12颗)等，实际最大长度为64字节

// 关闭GGA消息输出
const unsigned char 	gps_cmd_turnoff_gga[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24};

// 关闭GLL消息输出
const unsigned char 	gps_cmd_turnoff_gll[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};

// 关闭GSV消息输出
const unsigned char 	gps_cmd_turnoff_gsv[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};

// 关闭VTG消息输出
const unsigned char 	gps_cmd_turnoff_vtg[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47};

// 冷启动
const unsigned char	gps_cmd_coldstart[12] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0xFF, 0x02, 0x00, 0x0E, 0x61};

void clr_gpspos(void);

void gps_send_cmd(unsigned char* cmd, unsigned int size);

// gps初始化。
void gps_init(void)
{	
	// 上电后初始化GPS时将GPS强行断电以便确保上电时GPS处于明确的电源管理状态
	gps_power_down();	

	// 初始化gps串口接收相关变量和缓冲
	RxCnt2_rd = 0;
	RxCnt2_wr = 0;

	TxCnt2_rd = 0;
	TxCnt2_wr = 0;

	// 初始化rmc消息和gsa消息接收相关变量
	strcpy(gps_msg_rmc, "");
	strcpy(gps_msg_gsa, "");

	is_rmc_received 	= FALSE;
	is_gsa_received 	= FALSE;

	is_gps_fixed 	 	= FALSE;

	// 清空gps定位信息接收缓冲
	clr_gpspos();	

	printf("GPS initialized.\r\n");
}

// 打开gps芯片供电。
void gps_power_up(void)
{	
	MCU_GPIO_LOW(GPIO_GPS_PWR);

	// 打开gps电源后应等待一段时间以便gps芯片软件初始化运行，否则接下去的第一个命令会发送失败。
	delay_100ms(5);

	sts_gps_power = ON;

	// 关闭rmc和gsa之外的其他默认消息输出，以减少gps串口中断数量。
	gps_send_cmd((unsigned char*)gps_cmd_turnoff_gga, 16);

	#ifndef TESTING_GPS_SV
	gps_send_cmd((unsigned char*)gps_cmd_turnoff_gsv, 16);
	#endif
	
	gps_send_cmd((unsigned char*)gps_cmd_turnoff_gll, 16);
	gps_send_cmd((unsigned char*)gps_cmd_turnoff_vtg, 16);	

	printf("GPS powered up.\r\n");
}

// 关闭gps芯片供电。
void gps_power_down(void)
{	
	MCU_GPIO_HIGH(GPIO_GPS_PWR);

	delay_100ms(3);

	sts_gps_power = OFF;

	printf("GPS powered down.\r\n");
}

// 发命令给gps芯片。
void gps_send_cmd(unsigned char* cmd, unsigned int size)
{
	usart2_tx_fill((unsigned char*)cmd, size);

	usart2_tx_start();	

	printf("gps command sent with %d bytes.\r\n", size);
}

// 发送冷启动命令给GPS(会清除RAM中保存的历史星历并复位时钟等)。
void gps_coldstart(void)
{
	unsigned char	gps_cmd_coldstart[12]={0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0xFF, 0x02, 0x00, 0x0E, 0x61};
	
	usart2_tx_fill((unsigned char*)gps_cmd_coldstart, 12);

	usart2_tx_start();	

	printf("cold start gps.\r\n");
}

// 在GPS已定位情况下解析RMC消息。
// 样例: $GPRMC,172149.00,A,2234.03418,N,11402.72149,E,0.275,17.16,250113,,,A*55
int gps_ana_rmc(void)
{	
	char	str[16];

	int 	i = 0;
	int		j = 0;

	int		len = 0;
	
	float	val = 0.0;	

	len = strlen(gps_msg_rmc);

	// 检查消息字符串中逗号的个数是否合规
	if(cal_char_num(gps_msg_rmc, ',') != 12)
	{
		return NG;
	}
	
	// 略过第一个逗号
	for(i = 0; i < len; i++)
	{
		if(gps_msg_rmc[i] == ',')
		{
			break;
		}
	}

	// 指向逗号的下一个字符
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

	// N/S(北纬为正、南纬为负)
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

	// 根据GPS定位得到的经度计算终端所处的时区
	// g_time_zone = longitude2timezone(de_gpspos.lon);

	// E/W(东经为正、西经为负)
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
	de_gpspos.spd = 1.852*atof(str);	// RMC消息中的速度单位为节，需要转换为km/h单位
	
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

	// 检查是否需要根据GPS时间同步系统时间
	if(sw_sync_by_gps == ON)
	{
		// 用utc时间同步系统时间
		sync_systime(de_gpspos.utc, SYNC_BY_GPS);	
	}

	// 解析完RMC消息后暂时不将de_gpspos结构体中的sts置位，应等到GSA消息接收到并提取出HDOP信息后再置位

	return OK;
}

// GPS已经定位后解析GSA消息(目前仅提取HDOP信息)并返回提取操作的执行结果(成功或失败)。
// 样例: $GPGSA,A,3,23,29,07,08,09,18,26,28,,,,,1.94,1.18,1.54*0D
int gps_ana_gsa(void)
{
	int 	i = 0;
	int		j = 0;
	int		k = 0;
	
	int		commas = 0;
	
	char	str[12];
	
	int 	len = strlen(gps_msg_gsa);

	// 检查消息字符串中逗号的个数
	if(cal_char_num(gps_msg_gsa, ',') != 17)
	{
		return NG;
	}

	// 先指向字符串末尾字符
	i = len-1;

	while(i--)
	{
		// 检测*位置
		if(gps_msg_gsa[i] == '*')
		{
			break;
		}
	}

	// 若未检测到*，返回NG
	if(i <= 0)
	{
		de_gpspos.hdop = 0.00;
		
		return NG;
	}

	// 检测倒数2个逗号的位置并记录
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

	// 若未检测到2个逗号或2个逗号之间的距离超过5，返回NG
	if((i <= 0) || ((j-k) > 5))
	{
		de_gpspos.hdop = 0.00;
		
		return NG;
	}

	// 提取两个逗号之间的字符
	i = 0;
	k += 1;
	
	while(k < j)
	{
		str[i++] = gps_msg_gsa[k++];
	}

	str[i++] = '\0';

	len = strlen(str);

	// 检查HDOP域是否为空
	if(len <= 0)
	{
		// printf("HDOP domain is empty.\r\n");

		de_gpspos.hdop = 0.00;
		
		return NG;
	}

	// 将HDOP字符串转换为浮点数
	de_gpspos.hdop = atof(str);

	// 提取完GSA消息中的HDOP信息后将de_gpspos结构体中的定为标志置位
	de_gpspos.sts = TRUE;

	return OK;
}

#endif
