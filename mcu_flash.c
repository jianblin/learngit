  /*Flash读写管理*/
#include "mcu_flash.h"
#include "mcu_systick.h"
#include "common.h"
#include "dev_gsm.h"
#include "log_time.h"
#include "log_power.h"
#include "log_queue.h"
#include "log_pos.h"

#include "log_motion.h"

#include "string.h"
#include "stdio.h"

extern int mcu_reset_flag;

// 将指定长度的字节流数据写入到指定的Flash起始地址处。
// size		待写入字节流的实际长度
/********************************************************************************************************************
*
*
*
*
*********************************************************************************************************************/
void flash_bytestream_write(unsigned int flash_start, unsigned char* data_ptr, unsigned int data_len)
{
	int 			i;
	unsigned int	data_len2 = 0;	

	// 检查记录的长度是否为奇数
	data_len2 = data_len&1?(data_len-1):data_len;

	// 写入最大偶数个字节
	for(i = 0; i < data_len2; i += 2)
	{
		// 字节流数据类似大端存储格式，但是ARM内核采用小端格式读写数据，因此写入字节流数据时可采用双字节写入，
		// 但是读出字节流时除了要以双字节读出，还要对双字节数据做字节倒序处理。
		FLASH_ProgramHalfWord(flash_start+i, *((unsigned short*)data_ptr));	

		data_ptr += 2;
	}

	// 若记录长度为奇数，末尾字节补0x00(读出时舍去)后写入
	if(data_len2 < data_len)
	{
		FLASH_ProgramHalfWord(flash_start+i, (*(unsigned char*)data_ptr<<8) | 0x00);
	}
}

// 从指定Flash起始地址处读取指定长度的字节流数据到指定的缓存中。
// data_len			待读出字节流的长度
void flash_bytestream_read(unsigned int flash_start, unsigned char* data_ptr, unsigned int data_len)
{	
	int				i;
	unsigned int 	data_len2 = 0;
	unsigned short	halfword = 0x0000;
	
	// 检查待读出的字节流长度是否为奇数
	data_len2 = data_len&1?data_len-1:data_len;

	// 读取最大偶数个字节
	for(i = 0; i < data_len2; i += 2)
	{
		halfword = *(unsigned short*)(flash_start+i);

		// 对读出的双字节数据做字节倒序处理
		*data_ptr++ = halfword&0xFF;
		*data_ptr++ = (halfword>>8)&0xFF;
	}

	// 如果待读出的字节流长度为奇数，则读取最后一个字节
	if(data_len2 < data_len)
	{	
		halfword = *(unsigned short*)(flash_start+i);
		
		*data_ptr  = halfword&0xFF;
	}
}

// 设置指定的字符型备份参数。
void bkp_str_set(unsigned int bkp_start, unsigned int bkp_size, char* str)
{
	unsigned char	buf[ENV_SYSTEM_PARAMETER_SIZE];
	unsigned char*	ptr = (unsigned char*)buf;

	// 将Flash中的全部系统变量一次性读出到RAM
	flash_bytestream_read(BLCOK_ENVIRONMENT_START, (unsigned char*)ptr, ENV_SYSTEM_PARAMETER_SIZE);

	if(strlen(str) > (bkp_size-1))
	{
		// 截短字符串
		str[bkp_size-1] = '\0';		
	}

	// 修改ptr中的相应系统变量(strcpy函数会自动在字符串末未写入字符串结尾符号)
	strcpy((char*)(ptr+bkp_start), str);

	// 擦除当前页(系统变量存储在Flash中一个完整页里，从页的起始地址开始存储)
	FLASH_ErasePage(BLCOK_ENVIRONMENT_START);

	// 将RAM中更新的数据整体写回Flash
	flash_bytestream_write(BLCOK_ENVIRONMENT_START, (unsigned char*)ptr, ENV_SYSTEM_PARAMETER_SIZE);
}

// 读出指定的字符型备份参数?
// 注: 外部分配的字符串空间应足够保存读出的字节流， 否则会造成字符串溢出。
void bkp_str_get(unsigned int bkp_start, unsigned int bkp_size, char* str)
{
	flash_bytestream_read(BLCOK_ENVIRONMENT_START+bkp_start, (unsigned char*)str, bkp_size);
}

// 设置指定的32位整数型备份参数。
void bkp_u32_set(unsigned int bkp_start, unsigned int val)
{
	unsigned char	buf[ENV_SYSTEM_PARAMETER_SIZE];
	unsigned char*	ptr = (unsigned char*)buf;

	// 将Flash中的全部系统变量一次性读出到RAM
	flash_bytestream_read(BLCOK_ENVIRONMENT_START, (unsigned char*)ptr, ENV_SYSTEM_PARAMETER_SIZE);
	
	*(unsigned short*)(ptr+bkp_start+0) = val&0xFFFF;	
	*(unsigned short*)(ptr+bkp_start+2) = (val>>16)&0xFFFF;

	// 擦除当前页(系统变量存储在Flash中一个完整页里，从页的起始地址开始存储)
	FLASH_ErasePage(BLCOK_ENVIRONMENT_START);

	// 将RAM中更新的数据整体写回Flash
	flash_bytestream_write(BLCOK_ENVIRONMENT_START, (unsigned char*)ptr, ENV_SYSTEM_PARAMETER_SIZE);	
}

// 读出指定的16位整数型备份参数。
unsigned int bkp_u32_get(unsigned int bkp_start)
{
	unsigned int val = 0;
	
	val  =  *(unsigned short*)(BLCOK_ENVIRONMENT_START+bkp_start+0);
	val |= (*(unsigned short*)(BLCOK_ENVIRONMENT_START+bkp_start+2)<<16); 

	return val;
}

// 设置指定的整数型备份参数。
void bkp_u16_set(unsigned int bkp_start, unsigned short val)
{
	unsigned char	buf[ENV_SYSTEM_PARAMETER_SIZE];
	unsigned char*	ptr = (unsigned char*)buf;

	// 将Flash中的全部系统变量一次性读出到RAM
	flash_bytestream_read(BLCOK_ENVIRONMENT_START, (unsigned char*)ptr, ENV_SYSTEM_PARAMETER_SIZE);
	
	*(unsigned short*)(ptr+bkp_start+0) = val;

	// 擦除当前页(系统变量存储在Flash中一个完整页里，从页的起始地址开始存储)
	FLASH_ErasePage(BLCOK_ENVIRONMENT_START);

	// 将RAM中更新的数据整体写回Flash
	flash_bytestream_write(BLCOK_ENVIRONMENT_START, (unsigned char*)ptr, ENV_SYSTEM_PARAMETER_SIZE);	
}

// 读出指定的整数型备份参数。
unsigned short bkp_u16_get(unsigned int bkp_start)
{
	unsigned short val = 0;
	
	val  =  *(unsigned short*)(BLCOK_ENVIRONMENT_START+bkp_start);

	return val;
}

// 程序烧写后系统首次运行时设置默认的环境变量。
// 系统首次运行后用户可通过短信或USART接口命令修改环境变量，被修改的环境变量会被及时更新到Flash中，
// 下次开机运行时会系统会从Flash中自动恢复最新的环境变量。
void bkp_init(void)
{
	int		i;
	char	buf[ENV_SYSTEM_PARAMETER_SIZE];	

	// 初始化buf。
	memset(buf, '\0', ENV_SYSTEM_PARAMETER_SIZE);

	// 魔术数
	*(unsigned short*)(buf+ENV_SYS_FIRST_RUNNING_START) 		= TAG_FIRST_RUNNING;

	// 清除系统正常关机的特征数值
	*(unsigned short*)(buf+ENV_SYS_NORMAL_STARTUP_START) 		= 0x0000;	

#if 1	
	// MCU ID校验和(正常情况下在烧写程序时由PC写入，这里由终端MCU写入仅为测试用)
	*(unsigned int*)(buf+ENV_MCU_ID_START+0) 					= 0x87233113+MCUID_MASK;
	*(unsigned int*)(buf+ENV_MCU_ID_START+4) 					= 0x57486749+MCUID_MASK;
	*(unsigned int*)(buf+ENV_MCU_ID_START+8) 					= 0x066eff52+MCUID_MASK;		  

#ifdef USING_DEV_GSM
	// GSM IMEI
	sprintf(buf+ENV_GSM_IMEI_START, "%s", "868106001968904");
#endif
#endif	

	// 系统语言版本
	*(unsigned short*)(buf+ENV_SYS_LANGUAGE_START) 				= SYS_LANGUAGE_CHINESE;			// 默认设置为英文版本

	// 设备默认是否处于静默模式
	*(unsigned short*)(buf+ENV_DEV_DARKMODE_START) 				= FALSE;							// 默认不处于静默模式

	// 运动检测的数值门限
	*(unsigned short*)(buf+ENV_ERR_DETECT_MOTION_START) 		= DEF_DETECT_MOTION_ERR;				// 默认开启GPRS服务

	// 系统版本号
	sprintf(buf+ENV_VERSION_SOFTWARE_START, 	"%s", 	VER_SW);
	sprintf(buf+ENV_VERSION_HARDWARE_START, 	"%s", 	VER_HW);

#ifdef USING_DEV_GSM
	// GPRS网络参数
	sprintf(buf+ENV_GPRS_APN_START, 			"%s", 	"cmnet");										// 对于中国移动上网卡，APN可以是cmnet或cnwap
	sprintf(buf+ENV_GSM_TELECODE_START, 		"%s", 	"86");

	// GPRS #0连接参数初始化
	sprintf(buf+ENV_GPRS0_DN_START, 			"%s", 	"");
	sprintf(buf+ENV_GPRS0_IP_START, 			"%s", 	GPRS_ID0_IP);
	sprintf(buf+ENV_GPRS0_PORT_START, 			"%s", 	GPRS_ID0_PORT);

	// GPRS #1连接参数初始化
	sprintf(buf+ENV_GPRS1_DN_START, 			"%s", 	"");
	sprintf(buf+ENV_GPRS1_IP_START, 			"%s", 	GPRS_ID1_IP);
	sprintf(buf+ENV_GPRS1_PORT_START, 			"%s", 	GPRS_ID1_PORT);

	// GPRS #2连接参数初始化
	sprintf(buf+ENV_GPRS2_DN_START, 			"%s", 	"");
	sprintf(buf+ENV_GPRS2_IP_START, 			"%s", 	GPRS_ID2_IP);
	sprintf(buf+ENV_GPRS2_PORT_START, 			"%s", 	GPRS_ID2_PORT);

	// 设置默认的授权操作密码
	sprintf(buf+ENV_CMD_PWD_SMS_START, DEF_CMD_PWD_SMS);	
	// 系统中会统一手机号码为包含国家代码前缀的格式，因此保存在Flash中的默认手机号码都统一包含国家代码前缀

	sprintf(buf+ENV_CMD_PWD_DTMF_START, DEF_CMD_PWD_DTMF);	
	
	// 授权操作号码列表(指定的gsm号码应为不带国家区号的纯号码)
	for(i = 0; i < MAX_PN_OPERATON; i++)
	{		
		if(i == 0)
		{				
			sprintf(buf+ENV_PN_OPERATION_START+i*ENV_PN_OPERATION_SIZE, TEST_PHONE_NUMBER);		
		}	
		else
		{
			sprintf(buf+ENV_PN_OPERATION_START+i*ENV_PN_OPERATION_SIZE, "");
		}
	}
#endif

	// 擦除当前页(系统变量存储在Flash中一个完整页里，从页的起始地址开始存储)
	FLASH_ErasePage(BLCOK_ENVIRONMENT_START);

	// 将RAM中更新的数据整体写回Flash
	flash_bytestream_write(BLCOK_ENVIRONMENT_START, (unsigned char*)buf, ENV_SYSTEM_PARAMETER_SIZE);
}

// 系统启动后从Flash中恢复运行参数。
// 不管是正常关机后的重启还是看门狗复位后的重启，重启后都自动恢复Flash中保存的最新环境变量。
void bkp_recover(void)
{
	int		i;
	char	buf[ENV_SYSTEM_PARAMETER_SIZE];

	// 初始化buf。
	memset(buf, '\0', ENV_SYSTEM_PARAMETER_SIZE);

	printf("ENV_SYSTEM_PARAMETER_SIZE = %d\r\n", ENV_SYSTEM_PARAMETER_SIZE);
	
	flash_bytestream_read(BLCOK_ENVIRONMENT_START, (unsigned char*)buf, ENV_SYSTEM_PARAMETER_SIZE);

	// 系统语言版本
	sys_language				= *(unsigned short*)(buf+ENV_SYS_LANGUAGE_START);	

	// 设备软硬件版本
	strcpy(swversion, 	(char*)(buf+ENV_VERSION_SOFTWARE_START));
	strcpy(hwversion, 	(char*)(buf+ENV_VERSION_HARDWARE_START));

#if 1
	printf("sw_ver: %s\r\n", swversion);
	printf("hw_ver: %s\r\n", hwversion);
#endif

	// 系统语言版本
	is_sys_in_darkmode		= (BOOL)(*(unsigned short*)(buf+ENV_DEV_DARKMODE_START));	

	// 运动检测的数值门限
	err_detect_motion			= *(unsigned short*)(buf+ENV_ERR_DETECT_MOTION_START);	

#ifdef USING_DEV_GSM	
	// GPRS/GSM运行参数
	strcpy(gsm_telecode, 	(char*)(buf+ENV_GSM_TELECODE_START));

	// GPRS网络参数
	strcpy(gsm_apn, (char*)(buf+ENV_GPRS_APN_START));


	// 地图服务器的连接参数(TBD)
	strcpy(tcp_conn[GPRS_ID0].dn,   (char*)(buf+ENV_GPRS0_DN_START));
	strcpy(tcp_conn[GPRS_ID0].ip,   (char*)(buf+ENV_GPRS0_IP_START));
	strcpy(tcp_conn[GPRS_ID0].port, (char*)(buf+ENV_GPRS0_PORT_START));
	
	// 应用服务器的连接参数
	strcpy(tcp_conn[GPRS_ID1].dn,   (char*)(buf+ENV_GPRS1_DN_START));
	strcpy(tcp_conn[GPRS_ID1].ip,   (char*)(buf+ENV_GPRS1_IP_START));
	strcpy(tcp_conn[GPRS_ID1].port, (char*)(buf+ENV_GPRS1_PORT_START));	
		
	// 星历服务器的连接参数
	strcpy(tcp_conn[GPRS_ID2].ip,   (char*)(buf+ENV_GPRS2_IP_START));
	strcpy(tcp_conn[GPRS_ID2].port, (char*)(buf+ENV_GPRS2_PORT_START));

	// 恢复DTMF操作密码
	strcpy(dtmf_password, (char*)(buf+ENV_CMD_PWD_DTMF_START));

	// 恢复授权操作密码
	strcpy(cmd_pwd_sms, (char*)(buf+ENV_CMD_PWD_SMS_START));

	// 授权操作号码列表
	for(i = 0; i < MAX_PN_OPERATON; i++)
	{
		strcpy(pn_operation[i], (char*)(buf+ENV_PN_OPERATION_START+i*ENV_PN_OPERATION_SIZE));

		// 格式化gsm号码，若不成功，则不恢复该手机号码
		if(format_pn(pn_operation[i]) != OK)
		{
			i--;
		}

		// printf("pn_operation[%d] = %s\r\n", i, pn_operation[i]);
	}
#endif

	// 每次启动后清除系统正常关机的特征数值
	bkp_u16_set(ENV_SYS_NORMAL_STARTUP_START, 0x0000);
}


// 检查Flash中的魔术数区域是否设置了魔术数，若未设置，则系统为首次运行。
int check_first_running(void)
{	
	if(bkp_u16_get(ENV_SYS_FIRST_RUNNING_START) == TAG_FIRST_RUNNING)
	{
		return NG;
	}
	else
	{
		return OK;
	}
}

// 检查系统是否为关机后正常启动。
int check_normal_startup(void)
{
	if(bkp_u16_get(ENV_SYS_NORMAL_STARTUP_START) != TAG_START_UP)
	{
		return NG;
	}
	else
	{
		return OK;
	}
}

#if 0
// 从寄存器读取MCU ID(高字在前、低字在后)。
void read_mcuid_from_register(unsigned int* mcuid)
{
	mcuid[0] = *(volatile unsigned int*)(0x1FFFF7F0);	
	mcuid[1] = *(volatile unsigned int*)(0x1FFFF7EC);
	mcuid[2] = *(volatile unsigned int*)(0x1FFFF7E8);
}

// 从flash读取MCU ID(高字在前、低字在后)。
void read_mcuid_from_flash(unsigned int* mcuid)
{
	mcuid[0] = bkp_u32_get(ENV_MCU_ID_START+0)-MCUID_MASK;
	mcuid[1] = bkp_u32_get(ENV_MCU_ID_START+4)-MCUID_MASK;
	mcuid[2] = bkp_u32_get(ENV_MCU_ID_START+8)-MCUID_MASK;	
}

// 检查flash中保存的MCU ID是否和MCU读出的ID一致。
int check_mcuid(void)
{
	unsigned int mcuid1[3];
	unsigned int mcuid2[3];

	read_mcuid_from_register((unsigned int*)&mcuid1);
	// printf("mcu id = 0x%08x%08x%08x\r\n", mcuid1[0],mcuid1[1],mcuid1[2]);

	read_mcuid_from_flash((unsigned int*)&mcuid2);
	// printf("mcu id = 0x%08x%08x%08x\r\n", mcuid2[0],mcuid2[1],mcuid2[2]);

	if((mcuid1[0] != mcuid2[0]) || (mcuid1[1] != mcuid2[1]) || (mcuid1[2] != mcuid2[2]))
	{
		printf("MCU ID validation failed.\r\n");

		return NG;
	}
	else
	{
		printf("MCU ID validation past.\r\n");

		return OK;
	}
}
#endif

