#ifndef __FLASH
#define __FLASH

/*
128K Flash�Ŀռ����:

0x8000000 --> 1K	IAP Bootloader
0x8000400 --> 1K	Environment Variables
0x8000800 --> 63K	App Program	
0x8010400 --> 63K	Retx sms/gprs or new app image
*/

#define MCUID_MASK							40123

#define FLASH_PAGE_SIZE						1024		// STM32 Flash��ҳ��С(����Flashʱ����ҳΪ��λ)
#define FLASH_ACCESS_WIDTH					2			// Flash��д���(2�ֽڣ�ע��Nand Flash��������д)

#define BASE_OF_ROM							0x08000000
#define BASE_OF_RAM							0x20000000

#define STARTUP_FROM_BOOTLOADER

#ifndef STARTUP_FROM_BOOTLOADER				// Ӧ�ó�����������������bootloader����
	// | 63K App1 | 2K env | 63K app2| 

	#define BLOCK_APPLICATION_START1			(BASE_OF_ROM)			// 0x8000000
	#define BLOCK_APPLICATION_SPACE			(FLASH_PAGE_SIZE*63)	// 63K(��������1�ͳ�������2�ռ��С��ͬ)

	#define BLCOK_ENVIRONMENT_START			(BLOCK_APPLICATION_START1+BLOCK_APPLICATION_SPACE)
	#define BLCOK_ENVIRONMENT_SPACE			(FLASH_PAGE_SIZE*2)		// 2K

	#define BLOCK_APPLICATION_START2			(BLCOK_ENVIRONMENT_START+BLCOK_ENVIRONMENT_SPACE)

#else										// Ӧ�ó�����bootloader����������
	// | 1K boot  | 1k env | 63K app1 | 63K app2|

	// flash����������;����
	#define BLOCK_BOOTLOADER_START			(BASE_OF_ROM)
	#define BLOCK_BOOTLOADER_SPACE			(FLASH_PAGE_SIZE*1)		// 1K
	
	#define BLCOK_ENVIRONMENT_START			(BLOCK_BOOTLOADER_START+BLOCK_BOOTLOADER_SPACE)
	#define BLCOK_ENVIRONMENT_SPACE			(FLASH_PAGE_SIZE*1)		// 1K

	#define BLOCK_APPLICATION_SPACE			(63*1024)				// 63K(��������1�ͳ�������2�ռ��С��ͬ)
	#define BLOCK_APPLICATION_START1			(BLCOK_ENVIRONMENT_START+BLCOK_ENVIRONMENT_SPACE)
	#define BLOCK_APPLICATION_START2			(BLOCK_APPLICATION_START1+BLOCK_APPLICATION_SPACE)

#endif

#define TAG_FIRST_RUNNING					0x1234		// ���ϵͳ�Ƿ�Ϊ��д������״����е�������ֵ
#define TAG_START_UP							0x9876		// ���ϵͳ�ϴ��Ƿ�Ϊ�����ػ���������ֵ

// Ӧ�ó����Ƿ񱻸��µı�־(0x00: δ���£�0x01: �Ѹ���)
#define ENV_APP_UPDATED_START				0		
#define ENV_APP_UPDATED_SIZE					2

/****************************************** MCU IDУ��� *******************************************/	
#define ENV_SYS_FIRST_RUNNING_START			(ENV_APP_UPDATED_START+ENV_APP_UPDATED_SIZE)				// ��¼ϵͳ�Ƿ�Ϊ�״����е�ħ����	
#define ENV_SYS_FIRST_RUNNING_SIZE			2		

#define ENV_SYS_NORMAL_STARTUP_START			(ENV_SYS_FIRST_RUNNING_START+ENV_SYS_FIRST_RUNNING_SIZE)	// ��¼ϵͳ�Ƿ�Ϊ�����ػ�������	
#define ENV_SYS_NORMAL_STARTUP_SIZE			2		

/****************************************** MCU IDУ��� *******************************************/																									 
#define ENV_MCU_ID_START						(ENV_SYS_NORMAL_STARTUP_START+ENV_SYS_NORMAL_STARTUP_SIZE)		// ���ܺ��MCU IDУ���(96λ)	
#define ENV_MCU_ID_SIZE						12		

#define ENV_GSM_IMEI_START					(ENV_MCU_ID_START+ENV_MCU_ID_SIZE)							// GSM IMEI	
#define ENV_GSM_IMEI_SIZE						20			

#define ENV_SYS_LANGUAGE_START				(ENV_GSM_IMEI_START+ENV_GSM_IMEI_SIZE)						// ϵͳ���԰汾��
#define ENV_SYS_LANGUAGE_SIZE				2 			

#define ENV_DEV_DARKMODE_START				(ENV_SYS_LANGUAGE_START+ENV_SYS_LANGUAGE_SIZE)						// ϵͳ���԰汾��
#define ENV_DEV_DARKMODE_SIZE				2 		

/**************************************** G-Sensor������ò��� *************************************/
#define ENV_ERR_DETECT_MOTION_START		   	(ENV_DEV_DARKMODE_START+ENV_DEV_DARKMODE_SIZE)					// �˶�������ֵ����
#define ENV_ERR_DETECT_MOTION_SIZE			2	 	

/*#################################################################################################
								    �ַ������͵ı��ݲ���
###################################################################################################*/

/******************************************** ϵͳ�汾�� *******************************************/				// ϵͳ����汾��			
#define ENV_VERSION_SOFTWARE_START			(ENV_ERR_DETECT_MOTION_START+ENV_ERR_DETECT_MOTION_SIZE)
#define ENV_VERSION_SOFTWARE_SIZE			16		
					
#define ENV_VERSION_HARDWARE_START			(ENV_VERSION_SOFTWARE_START+ENV_VERSION_SOFTWARE_SIZE)					// ϵͳӲ���汾��
#define ENV_VERSION_HARDWARE_SIZE			16	 	

/****************************************** GPRS������� ******************************************/
#define ENV_GPRS_APN_START					(ENV_VERSION_HARDWARE_START+ENV_VERSION_HARDWARE_SIZE)					// GPRS���������
#define ENV_GPRS_APN_SIZE					64		

#define ENV_GSM_TELECODE_START				(ENV_GPRS_APN_START+ENV_GPRS_APN_SIZE)									// GSM����Ĺ��Ҵ���(���й�Ϊ86)
#define ENV_GSM_TELECODE_SIZE				4		

/************************************** GPRS #0-2�����Ӳ��� **************************************/
#define ENV_GPRS0_DN_START					(ENV_GSM_TELECODE_START+ENV_GSM_TELECODE_SIZE)						// Ӧ�÷�����������
#define ENV_GPRS0_DN_SIZE					20	 	

#define ENV_GPRS0_IP_START					(ENV_GPRS0_DN_START+ENV_GPRS0_DN_SIZE)							// Ӧ�÷�������IP��ַ
#define ENV_GPRS0_IP_SIZE						16		

#define ENV_GPRS0_PORT_START					(ENV_GPRS0_IP_START+ENV_GPRS0_IP_SIZE)							// Ӧ�÷������˿ں�
#define ENV_GPRS0_PORT_SIZE					8 		
//////////////
#define ENV_GPRS1_DN_START					(ENV_GPRS0_PORT_START+ENV_GPRS0_PORT_SIZE)						// ��ͼ������������
#define ENV_GPRS1_DN_SIZE					20		

#define ENV_GPRS1_IP_START					(ENV_GPRS1_DN_START+ENV_GPRS1_DN_SIZE)							// ��ͼ��������IP��ַ
#define ENV_GPRS1_IP_SIZE						16		

#define ENV_GPRS1_PORT_START					(ENV_GPRS1_IP_START+ENV_GPRS1_IP_SIZE)							// ��ͼ�������˿ں�
#define ENV_GPRS1_PORT_SIZE					8		
//////////////
#define ENV_GPRS2_DN_START					(ENV_GPRS1_PORT_START+ENV_GPRS1_PORT_SIZE)						// ����������������
#define ENV_GPRS2_DN_SIZE					20		

#define ENV_GPRS2_IP_START					(ENV_GPRS2_DN_START+ENV_GPRS2_DN_SIZE)							// ������������IP��ַ
#define ENV_GPRS2_IP_SIZE						16		

#define ENV_GPRS2_PORT_START					(ENV_GPRS2_IP_START+ENV_GPRS2_IP_SIZE)							// �����������Ķ˿ں�
#define ENV_GPRS2_PORT_SIZE					8	

/***************************************** ��Ȩ�������� ********************************************/
#define ENV_CMD_PWD_SMS_START				(ENV_GPRS2_PORT_START+ENV_GPRS2_PORT_SIZE)			
#define ENV_CMD_PWD_SMS_SIZE				(STD_LEN_OF_CMD_PWD+2)		

#define  ENV_CMD_PWD_DTMF_START			(ENV_CMD_PWD_SMS_START+ENV_CMD_PWD_SMS_SIZE)			
#define ENV_CMD_PWD_DTMF_SIZE				(STD_LEN_OF_CMD_PWD+2)	

/**************************************** ��Ȩ���������б� *****************************************/
// ע���ַ������������Ĵ洢����Ӧ��֤�ܴ洢�ַ�����β����'\0'������ǰ�����ڵ��ַ�������Ϊû���ַ�����β���Ŷ�������Ϊ��һ���ַ�����
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
