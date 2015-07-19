  /*Flash��д����*/
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

// ��ָ�����ȵ��ֽ�������д�뵽ָ����Flash��ʼ��ַ����
// size		��д���ֽ�����ʵ�ʳ���
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

	// ����¼�ĳ����Ƿ�Ϊ����
	data_len2 = data_len&1?(data_len-1):data_len;

	// д�����ż�����ֽ�
	for(i = 0; i < data_len2; i += 2)
	{
		// �ֽ����������ƴ�˴洢��ʽ������ARM�ں˲���С�˸�ʽ��д���ݣ����д���ֽ�������ʱ�ɲ���˫�ֽ�д�룬
		// ���Ƕ����ֽ���ʱ����Ҫ��˫�ֽڶ�������Ҫ��˫�ֽ��������ֽڵ�����
		FLASH_ProgramHalfWord(flash_start+i, *((unsigned short*)data_ptr));	

		data_ptr += 2;
	}

	// ����¼����Ϊ������ĩβ�ֽڲ�0x00(����ʱ��ȥ)��д��
	if(data_len2 < data_len)
	{
		FLASH_ProgramHalfWord(flash_start+i, (*(unsigned char*)data_ptr<<8) | 0x00);
	}
}

// ��ָ��Flash��ʼ��ַ����ȡָ�����ȵ��ֽ������ݵ�ָ���Ļ����С�
// data_len			�������ֽ����ĳ���
void flash_bytestream_read(unsigned int flash_start, unsigned char* data_ptr, unsigned int data_len)
{	
	int				i;
	unsigned int 	data_len2 = 0;
	unsigned short	halfword = 0x0000;
	
	// �����������ֽ��������Ƿ�Ϊ����
	data_len2 = data_len&1?data_len-1:data_len;

	// ��ȡ���ż�����ֽ�
	for(i = 0; i < data_len2; i += 2)
	{
		halfword = *(unsigned short*)(flash_start+i);

		// �Զ�����˫�ֽ��������ֽڵ�����
		*data_ptr++ = halfword&0xFF;
		*data_ptr++ = (halfword>>8)&0xFF;
	}

	// ������������ֽ�������Ϊ���������ȡ���һ���ֽ�
	if(data_len2 < data_len)
	{	
		halfword = *(unsigned short*)(flash_start+i);
		
		*data_ptr  = halfword&0xFF;
	}
}

// ����ָ�����ַ��ͱ��ݲ�����
void bkp_str_set(unsigned int bkp_start, unsigned int bkp_size, char* str)
{
	unsigned char	buf[ENV_SYSTEM_PARAMETER_SIZE];
	unsigned char*	ptr = (unsigned char*)buf;

	// ��Flash�е�ȫ��ϵͳ����һ���Զ�����RAM
	flash_bytestream_read(BLCOK_ENVIRONMENT_START, (unsigned char*)ptr, ENV_SYSTEM_PARAMETER_SIZE);

	if(strlen(str) > (bkp_size-1))
	{
		// �ض��ַ���
		str[bkp_size-1] = '\0';		
	}

	// �޸�ptr�е���Ӧϵͳ����(strcpy�������Զ����ַ���ĩδд���ַ�����β����)
	strcpy((char*)(ptr+bkp_start), str);

	// ������ǰҳ(ϵͳ�����洢��Flash��һ������ҳ���ҳ����ʼ��ַ��ʼ�洢)
	FLASH_ErasePage(BLCOK_ENVIRONMENT_START);

	// ��RAM�и��µ���������д��Flash
	flash_bytestream_write(BLCOK_ENVIRONMENT_START, (unsigned char*)ptr, ENV_SYSTEM_PARAMETER_SIZE);
}

// ����ָ�����ַ��ͱ��ݲ���?
// ע: �ⲿ������ַ����ռ�Ӧ�㹻����������ֽ����� ���������ַ��������
void bkp_str_get(unsigned int bkp_start, unsigned int bkp_size, char* str)
{
	flash_bytestream_read(BLCOK_ENVIRONMENT_START+bkp_start, (unsigned char*)str, bkp_size);
}

// ����ָ����32λ�����ͱ��ݲ�����
void bkp_u32_set(unsigned int bkp_start, unsigned int val)
{
	unsigned char	buf[ENV_SYSTEM_PARAMETER_SIZE];
	unsigned char*	ptr = (unsigned char*)buf;

	// ��Flash�е�ȫ��ϵͳ����һ���Զ�����RAM
	flash_bytestream_read(BLCOK_ENVIRONMENT_START, (unsigned char*)ptr, ENV_SYSTEM_PARAMETER_SIZE);
	
	*(unsigned short*)(ptr+bkp_start+0) = val&0xFFFF;	
	*(unsigned short*)(ptr+bkp_start+2) = (val>>16)&0xFFFF;

	// ������ǰҳ(ϵͳ�����洢��Flash��һ������ҳ���ҳ����ʼ��ַ��ʼ�洢)
	FLASH_ErasePage(BLCOK_ENVIRONMENT_START);

	// ��RAM�и��µ���������д��Flash
	flash_bytestream_write(BLCOK_ENVIRONMENT_START, (unsigned char*)ptr, ENV_SYSTEM_PARAMETER_SIZE);	
}

// ����ָ����16λ�����ͱ��ݲ�����
unsigned int bkp_u32_get(unsigned int bkp_start)
{
	unsigned int val = 0;
	
	val  =  *(unsigned short*)(BLCOK_ENVIRONMENT_START+bkp_start+0);
	val |= (*(unsigned short*)(BLCOK_ENVIRONMENT_START+bkp_start+2)<<16); 

	return val;
}

// ����ָ���������ͱ��ݲ�����
void bkp_u16_set(unsigned int bkp_start, unsigned short val)
{
	unsigned char	buf[ENV_SYSTEM_PARAMETER_SIZE];
	unsigned char*	ptr = (unsigned char*)buf;

	// ��Flash�е�ȫ��ϵͳ����һ���Զ�����RAM
	flash_bytestream_read(BLCOK_ENVIRONMENT_START, (unsigned char*)ptr, ENV_SYSTEM_PARAMETER_SIZE);
	
	*(unsigned short*)(ptr+bkp_start+0) = val;

	// ������ǰҳ(ϵͳ�����洢��Flash��һ������ҳ���ҳ����ʼ��ַ��ʼ�洢)
	FLASH_ErasePage(BLCOK_ENVIRONMENT_START);

	// ��RAM�и��µ���������д��Flash
	flash_bytestream_write(BLCOK_ENVIRONMENT_START, (unsigned char*)ptr, ENV_SYSTEM_PARAMETER_SIZE);	
}

// ����ָ���������ͱ��ݲ�����
unsigned short bkp_u16_get(unsigned int bkp_start)
{
	unsigned short val = 0;
	
	val  =  *(unsigned short*)(BLCOK_ENVIRONMENT_START+bkp_start);

	return val;
}

// ������д��ϵͳ�״�����ʱ����Ĭ�ϵĻ���������
// ϵͳ�״����к��û���ͨ�����Ż�USART�ӿ������޸Ļ������������޸ĵĻ��������ᱻ��ʱ���µ�Flash�У�
// �´ο�������ʱ��ϵͳ���Flash���Զ��ָ����µĻ���������
void bkp_init(void)
{
	int		i;
	char	buf[ENV_SYSTEM_PARAMETER_SIZE];	

	// ��ʼ��buf��
	memset(buf, '\0', ENV_SYSTEM_PARAMETER_SIZE);

	// ħ����
	*(unsigned short*)(buf+ENV_SYS_FIRST_RUNNING_START) 		= TAG_FIRST_RUNNING;

	// ���ϵͳ�����ػ���������ֵ
	*(unsigned short*)(buf+ENV_SYS_NORMAL_STARTUP_START) 		= 0x0000;	

#if 1	
	// MCU IDУ���(�������������д����ʱ��PCд�룬�������ն�MCUд���Ϊ������)
	*(unsigned int*)(buf+ENV_MCU_ID_START+0) 					= 0x87233113+MCUID_MASK;
	*(unsigned int*)(buf+ENV_MCU_ID_START+4) 					= 0x57486749+MCUID_MASK;
	*(unsigned int*)(buf+ENV_MCU_ID_START+8) 					= 0x066eff52+MCUID_MASK;		  

#ifdef USING_DEV_GSM
	// GSM IMEI
	sprintf(buf+ENV_GSM_IMEI_START, "%s", "868106001968904");
#endif
#endif	

	// ϵͳ���԰汾
	*(unsigned short*)(buf+ENV_SYS_LANGUAGE_START) 				= SYS_LANGUAGE_CHINESE;			// Ĭ������ΪӢ�İ汾

	// �豸Ĭ���Ƿ��ھ�Ĭģʽ
	*(unsigned short*)(buf+ENV_DEV_DARKMODE_START) 				= FALSE;							// Ĭ�ϲ����ھ�Ĭģʽ

	// �˶�������ֵ����
	*(unsigned short*)(buf+ENV_ERR_DETECT_MOTION_START) 		= DEF_DETECT_MOTION_ERR;				// Ĭ�Ͽ���GPRS����

	// ϵͳ�汾��
	sprintf(buf+ENV_VERSION_SOFTWARE_START, 	"%s", 	VER_SW);
	sprintf(buf+ENV_VERSION_HARDWARE_START, 	"%s", 	VER_HW);

#ifdef USING_DEV_GSM
	// GPRS�������
	sprintf(buf+ENV_GPRS_APN_START, 			"%s", 	"cmnet");										// �����й��ƶ���������APN������cmnet��cnwap
	sprintf(buf+ENV_GSM_TELECODE_START, 		"%s", 	"86");

	// GPRS #0���Ӳ�����ʼ��
	sprintf(buf+ENV_GPRS0_DN_START, 			"%s", 	"");
	sprintf(buf+ENV_GPRS0_IP_START, 			"%s", 	GPRS_ID0_IP);
	sprintf(buf+ENV_GPRS0_PORT_START, 			"%s", 	GPRS_ID0_PORT);

	// GPRS #1���Ӳ�����ʼ��
	sprintf(buf+ENV_GPRS1_DN_START, 			"%s", 	"");
	sprintf(buf+ENV_GPRS1_IP_START, 			"%s", 	GPRS_ID1_IP);
	sprintf(buf+ENV_GPRS1_PORT_START, 			"%s", 	GPRS_ID1_PORT);

	// GPRS #2���Ӳ�����ʼ��
	sprintf(buf+ENV_GPRS2_DN_START, 			"%s", 	"");
	sprintf(buf+ENV_GPRS2_IP_START, 			"%s", 	GPRS_ID2_IP);
	sprintf(buf+ENV_GPRS2_PORT_START, 			"%s", 	GPRS_ID2_PORT);

	// ����Ĭ�ϵ���Ȩ��������
	sprintf(buf+ENV_CMD_PWD_SMS_START, DEF_CMD_PWD_SMS);	
	// ϵͳ�л�ͳһ�ֻ�����Ϊ�������Ҵ���ǰ׺�ĸ�ʽ����˱�����Flash�е�Ĭ���ֻ����붼ͳһ�������Ҵ���ǰ׺

	sprintf(buf+ENV_CMD_PWD_DTMF_START, DEF_CMD_PWD_DTMF);	
	
	// ��Ȩ���������б�(ָ����gsm����ӦΪ�����������ŵĴ�����)
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

	// ������ǰҳ(ϵͳ�����洢��Flash��һ������ҳ���ҳ����ʼ��ַ��ʼ�洢)
	FLASH_ErasePage(BLCOK_ENVIRONMENT_START);

	// ��RAM�и��µ���������д��Flash
	flash_bytestream_write(BLCOK_ENVIRONMENT_START, (unsigned char*)buf, ENV_SYSTEM_PARAMETER_SIZE);
}

// ϵͳ�������Flash�лָ����в�����
// �����������ػ�����������ǿ��Ź���λ����������������Զ��ָ�Flash�б�������»���������
void bkp_recover(void)
{
	int		i;
	char	buf[ENV_SYSTEM_PARAMETER_SIZE];

	// ��ʼ��buf��
	memset(buf, '\0', ENV_SYSTEM_PARAMETER_SIZE);

	printf("ENV_SYSTEM_PARAMETER_SIZE = %d\r\n", ENV_SYSTEM_PARAMETER_SIZE);
	
	flash_bytestream_read(BLCOK_ENVIRONMENT_START, (unsigned char*)buf, ENV_SYSTEM_PARAMETER_SIZE);

	// ϵͳ���԰汾
	sys_language				= *(unsigned short*)(buf+ENV_SYS_LANGUAGE_START);	

	// �豸��Ӳ���汾
	strcpy(swversion, 	(char*)(buf+ENV_VERSION_SOFTWARE_START));
	strcpy(hwversion, 	(char*)(buf+ENV_VERSION_HARDWARE_START));

#if 1
	printf("sw_ver: %s\r\n", swversion);
	printf("hw_ver: %s\r\n", hwversion);
#endif

	// ϵͳ���԰汾
	is_sys_in_darkmode		= (BOOL)(*(unsigned short*)(buf+ENV_DEV_DARKMODE_START));	

	// �˶�������ֵ����
	err_detect_motion			= *(unsigned short*)(buf+ENV_ERR_DETECT_MOTION_START);	

#ifdef USING_DEV_GSM	
	// GPRS/GSM���в���
	strcpy(gsm_telecode, 	(char*)(buf+ENV_GSM_TELECODE_START));

	// GPRS�������
	strcpy(gsm_apn, (char*)(buf+ENV_GPRS_APN_START));


	// ��ͼ�����������Ӳ���(TBD)
	strcpy(tcp_conn[GPRS_ID0].dn,   (char*)(buf+ENV_GPRS0_DN_START));
	strcpy(tcp_conn[GPRS_ID0].ip,   (char*)(buf+ENV_GPRS0_IP_START));
	strcpy(tcp_conn[GPRS_ID0].port, (char*)(buf+ENV_GPRS0_PORT_START));
	
	// Ӧ�÷����������Ӳ���
	strcpy(tcp_conn[GPRS_ID1].dn,   (char*)(buf+ENV_GPRS1_DN_START));
	strcpy(tcp_conn[GPRS_ID1].ip,   (char*)(buf+ENV_GPRS1_IP_START));
	strcpy(tcp_conn[GPRS_ID1].port, (char*)(buf+ENV_GPRS1_PORT_START));	
		
	// ���������������Ӳ���
	strcpy(tcp_conn[GPRS_ID2].ip,   (char*)(buf+ENV_GPRS2_IP_START));
	strcpy(tcp_conn[GPRS_ID2].port, (char*)(buf+ENV_GPRS2_PORT_START));

	// �ָ�DTMF��������
	strcpy(dtmf_password, (char*)(buf+ENV_CMD_PWD_DTMF_START));

	// �ָ���Ȩ��������
	strcpy(cmd_pwd_sms, (char*)(buf+ENV_CMD_PWD_SMS_START));

	// ��Ȩ���������б�
	for(i = 0; i < MAX_PN_OPERATON; i++)
	{
		strcpy(pn_operation[i], (char*)(buf+ENV_PN_OPERATION_START+i*ENV_PN_OPERATION_SIZE));

		// ��ʽ��gsm���룬�����ɹ����򲻻ָ����ֻ�����
		if(format_pn(pn_operation[i]) != OK)
		{
			i--;
		}

		// printf("pn_operation[%d] = %s\r\n", i, pn_operation[i]);
	}
#endif

	// ÿ�����������ϵͳ�����ػ���������ֵ
	bkp_u16_set(ENV_SYS_NORMAL_STARTUP_START, 0x0000);
}


// ���Flash�е�ħ���������Ƿ�������ħ��������δ���ã���ϵͳΪ�״����С�
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

// ���ϵͳ�Ƿ�Ϊ�ػ�������������
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
// �ӼĴ�����ȡMCU ID(������ǰ�������ں�)��
void read_mcuid_from_register(unsigned int* mcuid)
{
	mcuid[0] = *(volatile unsigned int*)(0x1FFFF7F0);	
	mcuid[1] = *(volatile unsigned int*)(0x1FFFF7EC);
	mcuid[2] = *(volatile unsigned int*)(0x1FFFF7E8);
}

// ��flash��ȡMCU ID(������ǰ�������ں�)��
void read_mcuid_from_flash(unsigned int* mcuid)
{
	mcuid[0] = bkp_u32_get(ENV_MCU_ID_START+0)-MCUID_MASK;
	mcuid[1] = bkp_u32_get(ENV_MCU_ID_START+4)-MCUID_MASK;
	mcuid[2] = bkp_u32_get(ENV_MCU_ID_START+8)-MCUID_MASK;	
}

// ���flash�б����MCU ID�Ƿ��MCU������IDһ�¡�
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

