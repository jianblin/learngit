#ifndef __FLASH
#define __FLASH

/*
128K Flash的空间分配:

0x8000000 --> 1K	IAP Bootloader
0x8000400 --> 1K	Environment Variables
0x8000800 --> 63K	App Program	
0x8010400 --> 63K	Retx sms/gprs or new app image
*/

#define MCUID_MASK							40123

#define FLASH_PAGE_SIZE						1024		// STM32 Flash的页大小(擦除Flash时须以页为单位)
#define FLASH_ACCESS_WIDTH					2			// Flash读写宽度(2字节，注意Nand Flash不允许覆盖写)

#define BASE_OF_ROM							0x08000000
#define BASE_OF_RAM							0x20000000

#define STARTUP_FROM_BOOTLOADER

#ifndef STARTUP_FROM_BOOTLOADER				// 应用程序自行启动，无需bootloader引导
	// | 63K App1 | 2K env | 63K app2| 

	#define BLOCK_APPLICATION_START1			(BASE_OF_ROM)			// 0x8000000
	#define BLOCK_APPLICATION_SPACE			(FLASH_PAGE_SIZE*63)	// 63K(程序区块1和程序区块2空间大小相同)

	#define BLCOK_ENVIRONMENT_START			(BLOCK_APPLICATION_START1+BLOCK_APPLICATION_SPACE)
	#define BLCOK_ENVIRONMENT_SPACE			(FLASH_PAGE_SIZE*2)		// 2K

	#define BLOCK_APPLICATION_START2			(BLCOK_ENVIRONMENT_START+BLCOK_ENVIRONMENT_SPACE)

#else										// 应用程序由bootloader引导再启动
	// | 1K boot  | 1k env | 63K app1 | 63K app2|

	// flash各个区块用途定义
	#define BLOCK_BOOTLOADER_START			(BASE_OF_ROM)
	#define BLOCK_BOOTLOADER_SPACE			(FLASH_PAGE_SIZE*1)		// 1K
	
	#define BLCOK_ENVIRONMENT_START			(BLOCK_BOOTLOADER_START+BLOCK_BOOTLOADER_SPACE)
	#define BLCOK_ENVIRONMENT_SPACE			(FLASH_PAGE_SIZE*1)		// 1K

	#define BLOCK_APPLICATION_SPACE			(63*1024)				// 63K(程序区块1和程序区块2空间大小相同)
	#define BLOCK_APPLICATION_START1			(BLCOK_ENVIRONMENT_START+BLCOK_ENVIRONMENT_SPACE)
	#define BLOCK_APPLICATION_START2			(BLOCK_APPLICATION_START1+BLOCK_APPLICATION_SPACE)

#endif

#define TAG_FIRST_RUNNING					0x1234		// 检测系统是否为烧写程序后首次运行的特征数值
#define TAG_START_UP							0x9876		// 检测系统上次是否为正常关机的特征数值

// 应用程序是否被更新的标志(0x00: 未更新，0x01: 已更新)
#define ENV_APP_UPDATED_START				0		
#define ENV_APP_UPDATED_SIZE					2

/****************************************** MCU ID校验和 *******************************************/	
#define ENV_SYS_FIRST_RUNNING_START			(ENV_APP_UPDATED_START+ENV_APP_UPDATED_SIZE)				// 记录系统是否为首次运行的魔术数	
#define ENV_SYS_FIRST_RUNNING_SIZE			2		

#define ENV_SYS_NORMAL_STARTUP_START			(ENV_SYS_FIRST_RUNNING_START+ENV_SYS_FIRST_RUNNING_SIZE)	// 记录系统是否为正常关机后启动	
#define ENV_SYS_NORMAL_STARTUP_SIZE			2		

/****************************************** MCU ID校验和 *******************************************/																									 
#define ENV_MCU_ID_START						(ENV_SYS_NORMAL_STARTUP_START+ENV_SYS_NORMAL_STARTUP_SIZE)		// 加密后的MCU ID校验和(96位)	
#define ENV_MCU_ID_SIZE						12		

#define ENV_GSM_IMEI_START					(ENV_MCU_ID_START+ENV_MCU_ID_SIZE)							// GSM IMEI	
#define ENV_GSM_IMEI_SIZE						20			

#define ENV_SYS_LANGUAGE_START				(ENV_GSM_IMEI_START+ENV_GSM_IMEI_SIZE)						// 系统语言版本号
#define ENV_SYS_LANGUAGE_SIZE				2 			

#define ENV_DEV_DARKMODE_START				(ENV_SYS_LANGUAGE_START+ENV_SYS_LANGUAGE_SIZE)						// 系统语言版本号
#define ENV_DEV_DARKMODE_SIZE				2 		

/**************************************** G-Sensor相关配置参数 *************************************/
#define ENV_ERR_DETECT_MOTION_START		   	(ENV_DEV_DARKMODE_START+ENV_DEV_DARKMODE_SIZE)					// 运动检测的数值门限
#define ENV_ERR_DETECT_MOTION_SIZE			2	 	

/*#################################################################################################
								    字符串类型的备份参数
###################################################################################################*/

/******************************************** 系统版本号 *******************************************/				// 系统软件版本号			
#define ENV_VERSION_SOFTWARE_START			(ENV_ERR_DETECT_MOTION_START+ENV_ERR_DETECT_MOTION_SIZE)
#define ENV_VERSION_SOFTWARE_SIZE			16		
					
#define ENV_VERSION_HARDWARE_START			(ENV_VERSION_SOFTWARE_START+ENV_VERSION_SOFTWARE_SIZE)					// 系统硬件版本号
#define ENV_VERSION_HARDWARE_SIZE			16	 	

/****************************************** GPRS网络参数 ******************************************/
#define ENV_GPRS_APN_START					(ENV_VERSION_HARDWARE_START+ENV_VERSION_HARDWARE_SIZE)					// GPRS接入点名称
#define ENV_GPRS_APN_SIZE					64		

#define ENV_GSM_TELECODE_START				(ENV_GPRS_APN_START+ENV_GPRS_APN_SIZE)									// GSM号码的国家代码(如中国为86)
#define ENV_GSM_TELECODE_SIZE				4		

/************************************** GPRS #0-2的连接参数 **************************************/
#define ENV_GPRS0_DN_START					(ENV_GSM_TELECODE_START+ENV_GSM_TELECODE_SIZE)						// 应用服务器的域名
#define ENV_GPRS0_DN_SIZE					20	 	

#define ENV_GPRS0_IP_START					(ENV_GPRS0_DN_START+ENV_GPRS0_DN_SIZE)							// 应用服务器的IP地址
#define ENV_GPRS0_IP_SIZE						16		

#define ENV_GPRS0_PORT_START					(ENV_GPRS0_IP_START+ENV_GPRS0_IP_SIZE)							// 应用服务器端口号
#define ENV_GPRS0_PORT_SIZE					8 		
//////////////
#define ENV_GPRS1_DN_START					(ENV_GPRS0_PORT_START+ENV_GPRS0_PORT_SIZE)						// 地图服务器的域名
#define ENV_GPRS1_DN_SIZE					20		

#define ENV_GPRS1_IP_START					(ENV_GPRS1_DN_START+ENV_GPRS1_DN_SIZE)							// 地图服务器的IP地址
#define ENV_GPRS1_IP_SIZE						16		

#define ENV_GPRS1_PORT_START					(ENV_GPRS1_IP_START+ENV_GPRS1_IP_SIZE)							// 地图服务器端口号
#define ENV_GPRS1_PORT_SIZE					8		
//////////////
#define ENV_GPRS2_DN_START					(ENV_GPRS1_PORT_START+ENV_GPRS1_PORT_SIZE)						// 星历服务器的域名
#define ENV_GPRS2_DN_SIZE					20		

#define ENV_GPRS2_IP_START					(ENV_GPRS2_DN_START+ENV_GPRS2_DN_SIZE)							// 星历服务器的IP地址
#define ENV_GPRS2_IP_SIZE						16		

#define ENV_GPRS2_PORT_START					(ENV_GPRS2_IP_START+ENV_GPRS2_IP_SIZE)							// 星历服务器的端口号
#define ENV_GPRS2_PORT_SIZE					8	

/***************************************** 授权操作密码 ********************************************/
#define ENV_CMD_PWD_SMS_START				(ENV_GPRS2_PORT_START+ENV_GPRS2_PORT_SIZE)			
#define ENV_CMD_PWD_SMS_SIZE				(STD_LEN_OF_CMD_PWD+2)		

#define  ENV_CMD_PWD_DTMF_START			(ENV_CMD_PWD_SMS_START+ENV_CMD_PWD_SMS_SIZE)			
#define ENV_CMD_PWD_DTMF_SIZE				(STD_LEN_OF_CMD_PWD+2)	

/**************************************** 授权操作号码列表 *****************************************/
// 注意字符串环境变量的存储长度应保证能存储字符串结尾符号'\0'，否则前后相邻的字符串会因为没有字符串结尾符号而被误认为是一个字符串。
#define ENV_PN_OPERATION_START				(ENV_CMD_PWD_DTMF_START+ENV_CMD_PWD_DTMF_SIZE)				
#define ENV_PN_OPERATION_SIZE				(MAX_LEN_PN+1)			

#define ENV_SYSTEM_PARAMETER_SIZE			(ENV_PN_OPERATION_START+MAX_PN_OPERATON*ENV_PN_OPERATION_SIZE)

void 			flash_bytestream_write(unsigned int flash_start, unsigned char* data_ptr, unsigned int data_len);
void 			flash_bytestream_read(unsigned int flash_start, unsigned char* data_ptr, unsigned int data_len);

void 			bkp_str_set(unsigned int bkp_start, unsigned int bkp_size, char* var);
void	 			bkp_str_get(unsigned int bkp_start, unsigned int bkp_size, char* var);

void 			bkp_u32_set(unsigned int bkp_start, unsigned int   val);
unsigned int 		bkp_u32_get(unsigned int bkp_start);

void 			bkp_u16_set(unsigned int bkp_start, unsigned short val);
unsigned short 	bkp_u16_get(unsigned int bkp_start);

void 			bkp_init(void);
void 			bkp_recover(void);

int 				check_first_running(void);
int 				check_normal_startup(void);

#endif
