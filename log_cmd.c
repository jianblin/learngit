#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "common.h"
#include "log_cmd.h"

#include "log_queue.h"
#include "log_pos.h"
#include "log_motion.h"

#include "mcu_systick.h"
#include "mcu_adc.h"
#include "log_power.h"
#include "mcu_flash.h"

#include "mcu_usart.h"
#include "mcu_gpio.h"
#include "mcu_timer.h"

#include "dev_gsm.h"
#include "sms.h"
#include "dev_gps.h"

#include "dev_mma845x.h"

#include "stm32f10x_it.h"


void rcc_hsi_64mhz(void);
void rcc_hsi_8mhz(void);

#ifdef USING_DEV_GSM
char pn_operation[MAX_PN_OPERATON][MAX_LEN_PN+1];	// ��Ȩ����+Ȩ���б�

char	cmd_pwd_sms[STD_LEN_OF_CMD_PWD+1];				// ���������������
char	cmd_pwd_dtmf[STD_LEN_OF_CMD_PWD+1];			// ���������������
#endif

// �ر� / ��
const char info_switch[2][7] = {"\xE5\x85\xB3\xE9\x97\xAD", "\xE6\x89\x93\xE5\xBC\x80"};

// �����Ƿ���
const char info_invalid_argument[] = "\xE5\x8F\x82\xE6\x95\xB0\xE9\x9D\x9E\xE6\xB3\x95\xEF\xBC\x81";

// ��
const char info_fullstop[] = "\xE3\x80\x82";

// ��
const char info_commma[] = "\xEF\xBC\x8C";

// ���֧��
const char info_unsupported_command[] = "\xE5\x91\xBD\xE4\xBB\xA4\xE4\xB8\x8D\xE6\x94\xAF\xE6\x8C\x81\xEF\xBC\x81";

// ������������
const char info_incorrect_argc[] = "\xE5\x8F\x82\xE6\x95\xB0\xE4\xB8\xAA\xE6\x95\xB0\xE9\x94\x99\xE8\xAF\xAF\xEF\xBC\x81";

#define MAX_INDEX_CMD_WOTHOUT_PWD			6

// ȫ�������б�
const T_CMD_TXT	cmd_list[MAX_COMMAND_TXT_NUM]=
{
#ifdef USING_DEV_GSM
	// ��������(���ŷ����������ʱ�������ȱ���Ȩ�����ṩ��������������뼴��)
	{"szdxmm", 		"1001",		set_sms_password, 	3,	3},		// ���ö�������
	{"czdxmm",		"1002",		rev_sms_password, 	0,	0},		// ���ö�������
	{"szszmm", 		"1003",		set_dtmf_password, 3,	3},		// ������������
		
	{"tjczhm", 		"1004",		add_pn_operation, 	2,	2}, 		// ��Ӳ�������
	{"qkczhm", 		"1005",		clr_pn_operation, 	0,	0}, 		// ��ղ�������
	{"scczhm", 		"1006",		del_pn_operation, 	2,	2,}, 		// ɾ����������
	{"cxczhm", 		"1007",		inq_pn_operation,	0,	0}, 		// ��ѯ��������
#endif

	// dev���	 
	{"hfccsz", 		"1020",		set_dev_factory, 	0,	0}, 		// �ָ���������
	{"sbcq", 			"1021",		set_dev_reset, 		0,	0}, 		// �豸����
	{"sbgj", 			"",			set_dev_shutdown, 	0,	0}, 		// �豸�ػ�
	{"cxdcdy", 		"1024",		inq_dev_batvol, 		0,	0}, 		// ��ѯ��ص�ѹ
	{"cxsbgd", 		"1025",		inq_dev_powersupply, 	0,	0}, 	// ��ѯ�豸����
	{"cxsbsj", 		"1026",		inq_dev_time, 		0,	0}, 		// ��ѯ�豸ʱ��
	{"cxyjbb", 		"",			inq_dev_hwversion, 	0,	0}, 		// ��ѯӲ���汾
	{"cxrjbb", 		"",			inq_dev_swversion, 	0,	0}, 		// ��ѯ����汾

	// I/O���
	{"kgled", 			"1028",		swt_dev_led,	 	2,	2},		// ����LED
		
	// gsm���
#ifdef USING_DEV_GSM	
	{"cximei", 		"1030",		inq_gsm_imei, 		0,	0}, 		// ��ѯIMEI
	{"cxwlxh", 		"1031",		inq_gsm_signal, 		0,	0}, 		// ��ѯ�����ź�
	{"cxgjbb", 		"",			inq_gsm_swversion, 	0,	0}, 		// ��ѯ�̼��汾
	{"cxwllj", 		"",			inq_gsm_connection,	0,	0}, 		// ��ѯ��������
	{"szapn", 		"",			set_gsm_apn, 		1,	1}, 			// ����APN
	{"szgjqh", 		"1035",		set_gsm_telecode, 	1,	1}, 		// ���ù�������
	{"tsgsm", 		"",			set_gsm_debug, 		1,	1}, 		// ����gsm
	{"hfgsm", 		"",			set_gsm_recover, 	0,	0}, 			// �ָ�gsm
	{"cqgsm", 		"",			set_gsm_reset, 		0,	0}, 		// ����gsm
#endif

	// gps���
	#ifdef USING_DEV_GPS
	{"tsgps", 			"1042",		swt_gps_dump, 		1,	1}, 		// ����gps
	{"zqdw", 			"1043",		swt_gps_ppos, 		1,	1}, 		// �������ڶ�λģʽ	
	#endif

	// img���
	{"xzcx", 			"1045",		upd_img_com, 		2,	2}, 		// ���س���
};

/********************************************************************************************************************
											�����ͨ�ú���
*********************************************************************************************************************/

// �������ַ����Զ���Ϊ��������ֳɼ��������Ա��һ��������
ERR_NO destruct_command(char* command, int* argc, char (*argv)[MAX_LEN_CMD_DMN+1])
{
	int		i = 0;
	int		j = 0;
	int		k = 0;

	BOOL	ended = FALSE;		// �����ַ����Ƿ���#�����ı�־
	
	int		len = strlen(command);

	// printf("command in destruct_command(): %s\r\n", command);
		
	for(i = 0; i < len; i++)
	{		
		if(IS_TXT_COMMAND_CHARACTER(command[i]) == FALSE)
		{
			return -1;
		}

		// ��ȡ����������
		if(command[i] == ',')			// ����ǰ�ַ�Ϊ���ţ���ǰ���������
		{
			if(k > MAX_LEN_CMD_DMN)
			{
				return -3;
			}
			
			// ָ����һ��������
			argv[j][k++] = '\0';	

			j++;
			
			k = 0;				

			if(j >= MAX_NUM_CMD_DMN)
			{
				return -4;
			}
		}
		// ����ǰ�ַ�Ϊ���ţ��������������
		else if(command[i] == '#')
		{
			if(k > MAX_LEN_CMD_DMN)
			{
				return -3;
			}
			
			argv[j][k++] = '\0';

			j++;

			ended = TRUE;

			break;
		}
		// �������ַ��Ϸ�����ǰ�ַ��ȷǶ���Ҳ�Ǿ���ʱ����Ϊ��ͨ��ĸ����˽����������
		else
		{
			if(k > MAX_LEN_CMD_DMN)
			{
				return -3;
			}
			
			argv[j][k++] = command[i];
		}
	}	

	// ��������Ƿ���#�Ž�β�������ѭ������else if(command[i] == '#')�����˳���Ҳ����i < len�����˳���
	if(ended == FALSE)
	{		
		return -5;
	}

	// �������������ַ�������ĸ���
	*argc = j;
	
	return OK;
}

// �������б��а��ַ��������������š�
int	seek_cmd_by_str(char* cmd_str)
{
	int		i;

	for(i = 0; i < MAX_COMMAND_TXT_NUM; i++)
	{
		if(!__strcmp(cmd_str, (char*)(cmd_list[i].cmd)))
		{
			return i;
		}
	}

	return -1;
}

// �������б��а����ִ������������š�
int	seek_cmd_by_dig(char* cmd_dig)
{
	int		i;

	for(i = 0; i < MAX_COMMAND_TXT_NUM; i++)
	{
		if(!__strcmp(cmd_dig, (char*)(cmd_list[i].dtmf)))
		{
			return i;
		}
	}

	return -1;
}

// ��ȫ���ı������б��в������ִ��(�������ͨ��reply���巵��)��
ERR_NO handle_cmd(unsigned char cmd_src, char* cmd_str, char* pnf_src, char* pnf_dst, char* reply)
{	
	int		argc = 0;
	char	argv[MAX_NUM_CMD_DMN][MAX_LEN_CMD_DMN+1];

	int		i = -1;

	ERR_NO	ret;

	// printf("cmd_src: %d, cmd_str: %s, pnf_src: %s, pnf_dst: %s\r\n", cmd_src, cmd_str, pnf_src, pnf_dst);

	// �������ַ����ָ�ɸ���������
	ret = destruct_command(cmd_str, (int*)&argc, (char (*)[MAX_LEN_CMD_DMN+1])argv);
	if(ret != OK)
	{
		// �����ʽ����
		strcpy(reply, "\xE5\x91\xBD\xE4\xBB\xA4\xE6\xA0\xBC\xE5\xBC\x8F\xE9\x94\x99\xE8\xAF\xAF\xEF\xBC\x81");		

		return ERR_HANDLE_COMMAND_INVALID_COMMAND;
	}

	for(i = 0; i < argc; i++)
	{
		printf("argc = %d, argv[%d] = %s\r\n", argc, i, argv[i]);
	}	

	// ����������Դ��ͬ����ȡ��ͬ��ʽƥ�������ȡ��ͬ��ʽ�������������
	switch(cmd_src)
	{
		case CMD_SRC_SMS:
			printf("command from SMS.\r\n");
			
			// �Ȳ���������š�
			i = seek_cmd_by_str(argv[0]);
				
			if(i >= 0)
			{	
				// ��������Ƿ�Ͳ���������أ�����صĻ���������Ƿ��ѱ���Ȩ��
				if(i > MAX_INDEX_CMD_WOTHOUT_PWD)
				{
					// �ȶ�����ͺ����Ȩ��
					if(check_pn_operation(pnf_src) != OK)
					{
						//   δ��Ȩ��
						sprintf(reply, "%s\xE6\x9C\xAA\xE6\x8E\x88\xE6\x9D\x83\xEF\xBC\x81", pnf_src);
						
						return NG;
					}
				}

				// �����������Ƿ����������ܵ�������������
				if(argc > (cmd_list[i].args_max+1) || argc < (cmd_list[i].args_min+1))
				{
					// ������������
					strcpy(reply, info_incorrect_argc);
						
					return NG;
				}
				
				// �����������(���������������ظ���Ϣ)��
				ret = cmd_list[i].func(cmd_src, argc, argv, pnf_src, (char*)reply);

				return ret;
			}
			else
			{
				// ���֧�֣�
				sprintf(reply, "%s%s", argv[0], info_unsupported_command);
				
				return NG;
			}
		case CMD_SRC_COM:
			printf("command from COM.\r\n");
			
			// ����������š�
			i = seek_cmd_by_str(argv[0]);

			// ���ڴ��ڷ��͵��������������Ƿ���ȨҲ�޷����(�޷��ͺ���һ˵)��
				
			if(i >= 0)
			{
				// �����������Ƿ����������ܵ�������������
				if(argc > (cmd_list[i].args_max+1) || argc < (cmd_list[i].args_min+1))
				{
					// ������������
					strcpy(reply, info_incorrect_argc);
						
					return NG;
				}
				
				// �����������(���������������ظ���Ϣ)��
				ret = cmd_list[i].func(cmd_src, argc, argv, "", (char*)reply);

				return ret;
			}
			else
			{
				sprintf(reply, "%s%s", argv[0], info_unsupported_command);
				
				return NG;
			}
		case CMD_SRC_DTMF:
			printf("command from DTMF.\r\n");
			
			// ����������š�
			i = seek_cmd_by_dig(argv[0]);
	
			if(i >= 0)
			{
				printf("cmd = %s, dtmf = %s, args_min = %d, args_max = %d.\r\n", cmd_list[i].cmd, cmd_list[i].dtmf, cmd_list[i].args_min, cmd_list[i].args_max);
				
				// ͬһ���dmtf������ȶ����������β�����һ���������롣
				if(argc > (cmd_list[i].args_max+2) || argc < (cmd_list[i].args_min+2))
				{				
					// ������������
					strcpy(reply, info_incorrect_argc);
						
					return NG;
				}
				else
				{
					if(strlen(argv[argc-1]) == STD_LEN_PN)
					{
						// ��dtmf�����ַ���ĩβ�Ļظ�������ȡ��������䵽dtmf���������pnf_dst�����С�
						// strcpy(que_cmd[cnt_cmd_rd & (MAX_NUM_CMD_TXT-1)].pnf_dst, argv[argc-1]);
						strcpy(pnf_dst, argv[argc-1]);

						// �������������
						ret = cmd_list[i].func(cmd_src, argc, argv, argv[argc-1], (char*)reply);
						
						return ret;						
					}
					else
					{
						// ������11λ�ֻ�������Ϊ����ִ�н�����غ��룡
						strcpy(reply, "\xE8\xAF\xB7\xE8\xBE\x93\xE5\x85\xA5\x31\x31\xE4\xBD\x8D\xE6\x89\x8B\xE6\x9C\xBA\xE5\x8F\xB7\xE7\xA0\x81\xE4\xBD\x9C\xE4\xB8\xBA\xE5\x91\xBD\xE4\xBB\xA4\xE6\x89\xA7\xE8\xA1\x8C\xE7\xBB\x93\xE6\x9E\x9C\xE8\xBF\x94\xE5\x9B\x9E\xE5\x8F\xB7\xE7\xA0\x81\xEF\xBC\x81");

						return NG;
					}
				}					
			}
			else
			{
				sprintf(reply, "%s%s", argv[0], info_unsupported_command);
				
				return NG;
			}
		default:
			break;
	}
	
	return ERR_HANDLE_COMMAND_NO_ITEM;				
}

// �����ڷ���������(ֱ�Ӳ������ڽ��ն��У��޶����������)��
int ana_buf_com(void)
{
	char	command[MAX_BYTE_SMS_PDU+1];

	int 	i = 0;
	
	unsigned char	byte = 0x00;

	unsigned int	to;
	
	// ����Ƿ����µĴ���������յ�(��"#\n"��β���ַ���)
	while(cnt_usart1_cmd > 0)
	{	
		printf("%d com command pending.", cnt_usart1_cmd);
		
		i = 0;

#if 1
		// ���˵���ǰ��Ĳ��ɴ�ӡ�ַ���
		to  = systick + 2*1000/SYSTICK_PERIOD;

		while(systick < to)
		{
			byte = COM1_RX_RD(RxCnt1_rd);
			
			if(IS_TXT_COMMAND_CHARACTER(byte) == TRUE)
			{		
				break;
			}
			else
			{
				RxCnt1_rd++;
			}
		}

		// ����Ƿ�ʱ(����Ӧ���ᳬʱ)
		if(systick >= to)
		{
			// �Թ������Ѽ�⵽������
			cnt_usart1_cmd = 0;
			RxCnt1_rd = RxCnt1_wr;

			printf("receiving data time out.\r\n");
			
			return NG;
		}
#endif	

		// ���Ӷ�command�α�i�ļ�⣬�����û��󽫴��ļ����������ʱ��MCU����
		while(RxCnt1_rd < RxCnt1_wr && i < MAX_BYTE_SMS_PDU)
		{						
			command[i++] = COM1_RX_RD(RxCnt1_rd++);

			// ����������������ַ��Ƿ�Ϊ"#\n"���ǵĻ���˵�������������β
			if(command[i-3] == '#' && command[i-2] == '\r' && command[i-1] == '\n')
			{
				// ����ַ�����β����
				command[i-2] = '\0';

				printf("\r\ncom command:%s", command);

				// ������������뵽ȫ���ı�������С�
				que_cmd_wr(CMD_SRC_COM, command, "", "");
				
				// UART1�����Ҵ���������������ݼ�
				cnt_usart1_cmd--;
				
				break;
			}
		}
	}
	
	return OK;
}

// �������Զ��ŵ�����(ֻ����һ������)��
/*
	1��֧��textģʽ�º�pduģʽ��ucs2���Ż�ascci���ŵĽ�����ϵͳʱ��ͬ������֧�����Ķ�������;
	2��֧�ֶ��������ݴ���ͷ���: �������Ķ�����������#�Ž�β;
*/
int ana_que_sms(void)
{
	char			msg[256];
	
	char			txtime1[32];				// �ַ�����ʽ�ķ���ʱ����Ϣ(����������ʱ���뼰ʱ��)
	unsigned char	txtime2[7];					// ��ֵ  ��ʽ�ķ���ʱ����Ϣ(����������ʱ���뼰ʱ��)

	char			str[8];	

	char			pn[MAX_LEN_PN+1];					// ԭʼ��gsm����(���ܰ����򲻰�����������ǰ׺)

	char*			sms_cont_ptr = NULL;

	T_SMS_DELIVER	deliver;					// ����PDUģʽ�½��յ���PDU����������

	int				i = 0;
	int				j = 0;
	int				k = 0;

	int 			len = 0;

	/************************************** ����GSMģ�����������SMS��Ϣ ****************************************/

	// �Ӷ������������ȡ���������
	strcpy(msg, (char*)&que_sms_cmd[cnt_sms_cmd_rd++ & (MAX_NUM_SMS_RX-1)]);

	printf("cnt_sms_cmd_rd = %d\r\n", cnt_sms_cmd_rd);

	printf("raw sms:%s\r\n", msg);

	// BOOL	ended = FALSE;				// �����β�ַ�#�Ƿ��ҵ�

	for(i = 0; i < 7; i++)
	{
		txtime2[i] = 0x00;
	}
	
	i = 0;

	// ��λ����һ�����Ŵ�
	while(msg[i++] != ',');	
	
	// ��鵱ǰGSMģ�������Ķ���ģʽ��pduģʽ��ֻ�����pdu�ַ�����
	// ����textģʽ�¿������ascii�ַ�����ucs2����Ķ������ݣ����������Դ���
	if(msg[i] == '"')				// ����textģʽ
	{			
		// ��ȡ���ͺ���
		len = fetch_quotes(pn, msg+i);
		i += len;

		// �Թ�2�����ţ�ָ����ʱ���ַ����ĵ�һ��˫����
		while(msg[i++] != ',');
		while(msg[i++] != ',');

		// ��ȡ����ʱ���ַ���(�Ժ����)
		len = fetch_quotes(txtime1, msg+i);
		i += len;

		// ��λ���������ݵ���ʼλ�ô�
		for(;;)
		{
			if(msg[i] == '\r' && msg[i+1] == '\n')
			{
				i += 2;	
				
				break;
			}
			else
			{
				i++;	
			}
		}		

		// ��¼�������ݵ���ʼ��ַ(�������ݹ̶�Ϊascii������ַ���)
		sms_cont_ptr = (char*)msg+i;	
	
		// ��ȡ�м�������ꡢ�¡���
		for(i = 0, j = 0, k = 0; i < len && k < 3; )
		{
			str[j++] = txtime1[i++];

			// �����������ַ�����ͣ��ȡ
			if(txtime1[i] < '0' || txtime1[i] > '9')
			{
				i++;	// �Թ��������ַ�				
				
				str[j++] = '\0';					

				if(k == 0)	// ��
				{
					txtime2[k++] = (unsigned short)__atoi((char*)str)-2000;
				}
				else		// �¡���
				{
					txtime2[k++] = (short)__atoi((char*)str);
				}
	
				j = 0;				
			}
		}

		// ��ȡ��ð�Ÿ�����ʱ���֡���(����V015�汾�̼�)
		for( ; i < len && k < 6; )
		{			
			str[j++] = txtime1[i++];

			// �����������ַ�����ͣ��ȡ
			if(txtime1[i] < '0' || txtime1[i] > '9')
			{				
				i++;	// �Թ��������ַ�
				
				str[j++] = '\0';	
	
				txtime2[k++] = (unsigned char)__atoi((char*)str);		// ��ǰ�������ν�������ʱ���뱣�浽����ʱ��������
	
				j = 0;				
			}
		}

		// ��ȡ�������ŵ�ʱ��
		txtime2[k++] = (unsigned char)(__atoi((char*)txtime1+i)%24);				// ���ŷ���ʱ���е�ʱ��ֵ���ܳ���24			

		// printf("time mark of SMS: %04d-%02d-%02d %02d:%02d:%02d %2d time zone\n", txtime2[0], txtime2[1], txtime2[2], txtime2[3], txtime2[4], txtime2[5], txtime2[6]);

		// �������Ķ����������ַ��Ƿ�ΪСдӢ����ĸ�����ǵĻ���Ϊpdu�ַ���(���Ļ�Ӣ��)
		if(!(sms_cont_ptr[0] >= 'a' && sms_cont_ptr[0] <= 'z'))
		{
			// ��pdu�ַ���ת��Ϊ�������ֽ���
			len = pdu_atoh((unsigned char*)sms_cont_ptr, sms_cont_ptr, strlen(sms_cont_ptr));

			// ���pdu�ַ�������(ucs2��ʽ����)�Ƿ�ΪӢ��
			if(sms_cont_ptr[0] == 0x00 && (sms_cont_ptr[1] >= 'a' && sms_cont_ptr[1] <= 'z'))
			{				
				len /= 2;

				// ��ucs2����ת��Ϊascii����
				for(i = 0; i < len; i++)
				{
					*(sms_cont_ptr+i) = *(sms_cont_ptr+(i<<1)+1);
				}

				*(sms_cont_ptr+i) = '\0';
			}
			// else: ��pdu�ַ�������Ϊ���ģ����账��
		}		
	}
	else if(msg[1] == ',')			// ����pduģʽ
	{		
		// ��λ��pdu������ʼ��ַ��
		for(;;)
		{
			if(msg[i] == '\r' && msg[i+1] == '\n')
			{
				i += 2;
				
				break;
			}
			else
			{
				i++;
			}
		}

		sms_cont_ptr = msg+i;
		// // printf("sms_cont_ptr = %s", sms_cont_ptr);

		// ����PDU��
		memset((unsigned char*)&deliver, sizeof(T_SMS_DELIVER), 0x00);

		// ���������ݽ��벢��ý�������ݵ��ַ�����
		len = pdu_deconstruct((T_SMS_DELIVER*)&deliver, sms_cont_ptr);

		// ���淢�ͺ���
		strcpy(pn, deliver.tp_oas);	
		
		if(len > 0)
		{
			// ��¼���ű������ʼ��ַ(��������Ϊascii������ַ�����ucs2������ֽ���)
			sms_cont_ptr = (char*)deliver.tp_uds;

			// ��ucs2�����Ӣ�Ķ������ݽ�����轫��ת��Ϊascii�ַ������ɽ���
			if((deliver.tp_dcs&0x0C) == SMS_PDU_ENCODING_UCS2)
			{
				if(sms_cont_ptr[0] == 0x00 && (sms_cont_ptr[1] >= 'a' && sms_cont_ptr[1] <= 'z'))
				{					
					for(i = 0; i < len; i++)
					{
						*(sms_cont_ptr+i) = *(sms_cont_ptr+(i<<1)+1);
					}

					*(sms_cont_ptr+i) = '\0';
				}
				// else: ��ucs2��������Ķ������ݽ����Ҳ��������
			}
			// else: ����7bit��8bit����Ķ������ݾ�pdu�����ᱻת����ascii�ַ�������ֱ�ӽ���
		}

		/************* ��ȡ����ʱ�� ************/		
		for(i = 0; i < 14; i += 2)
		{
			str[0] = deliver.tp_scts[i+0];
			str[1] = deliver.tp_scts[i+1];
			str[2] = '\0';

			if(i == 0)
			{
				// ��ȡ��(ԭʼ��ֵ������4λ��ݵĺ�2λ)
				txtime2[i>>1] = __atoi(str);
			}
			else if(i < 12)
			{
				// ��ȡ�¡��ա�ʱ���֡���
				txtime2[i>>1] = __atoi(str);
			}
			else// i >= 12
			{
				// ��ȡʱ��
				txtime2[i>>1] = __atoi(str)%24;			// ���ŷ���ʱ����Ϣ�е�ʱ��ֵ���ܳ���24
			}
		}	
	}
	else
	{
		return NG;
	}

	// ����Ƿ���Ҫͨ��SMS����ʱ�����ϵͳʱ��
	if(sw_sync_by_sms == ON)
	{	
		// ���֮ǰ�Ƿ�ͨ��GPSͬ����ϵͳʱ�䣬ͬ�����Ļ�����ͨ��SMS����ʱ��ͬ��(���Ȳ���)
		if(!(g_sync_point.synmode == SYNC_BY_GPS && g_sync_point.systick > 0))
		{
			// ͨ��SMS����ʱ��ͬ��ϵͳʱ��
			sync_systime(txtime2, SYNC_BY_SMS);

			// ����SMS����ʱ���е�ʱ����Ϣ
			g_time_zone = txtime2[6];

			// ��SMS����ʱ�����ʱ��ͬ����󣬼��ɹر�sw_synsms����Ƶ��ͬ��ϵͳʱ��
			sw_sync_by_sms = OFF;
		}
	}

	// �����������Ƿ�Ϊ��ascii�ַ�.
	if(is_all_ascii(sms_cont_ptr) == FALSE)
	{
		return NG;
	}

	// ��ʽ��gsm���롣
	if(	format_pn(pn) == NG)
	{
		return NG;
	}

	// ��������Ķ���������뵽ȫ���ı�������С�
	que_cmd_wr(CMD_SRC_SMS, sms_cont_ptr, pn, pn);

	return OK;
}

#ifdef USING_DEV_GSM
// ��������dtmf���������(�����Զ���ͨ���û�������dtmf���봮������У��ͨ�����û�������dtmf������������Ҫ��������dtmf��)��
void ana_str_dtmf(char* dtmf)
{
	char	at[32];

	// int		argc = 0;
	// char	argv[MAX_NUM_CMD_DMN][MAX_LEN_CMD_DMN+1];

	int		i;
	int		len = -1;

	// ����ͷ�����򿪺�ſ�ʼ���dtmf����У������
	if(sts_dtmf_command == STS_DTMF_COMMAND_MIC_MUTE_ON)
	{
		if(!__strcmp(dtmf, dtmf_password))
		{
			sts_dtmf_command = STS_DTMF_COMMAND_AUTHORIZED;

			// ֹͣdtmf�������볬ʱ��ⶨʱ��
			swtimer_input_dtmf = 0;

			// ������ȷ����󣬷���dtmf����(�û��ֻ�һ�㲻����ʾdtmf����ֵ�����ǻ�����dtmf������)����ʾ�û�����������ȷ��������dtmf����
			strcpy(at, "AT+VTS=1\n");			
			usart3_send_poll(at, strlen(at));	

			// delay_100ms(1);

			// ������ȷ����󣬷���dtmf����(�û��ֻ�һ�㲻����ʾdtmf����ֵ�����ǻ�����dtmf������)����ʾ�û�����������ȷ��������dtmf����
			strcpy(at, "AT+VTS=1\n");			
			usart3_send_poll(at, strlen(at));	

			// delay_100ms(1);

			// �����ر���ͷ�����ӳٶ�ʱ��
			printf("systick = %d, delay %d second for turning off mic mute.\r\n", systick, DELAY_SET_MICMUTE_OFF);
			
			swtimer_set_micmute_off = DELAY_SET_MICMUTE_OFF*1000/SYSTICK_PERIOD;
		}
		else
		{
			times_input_dtmf_password++;

			// �����������δ����ն��Զ��Ҷϵ绰
			if(times_input_dtmf_password >= MAX_TIMES_INPUT_DTMF_COMMAND)
			{
				// ֹͣdtmf�������볬ʱ��ⶨʱ��
				swtimer_input_dtmf = 0;
					
				// �Ҷϵ绰
				strcpy(at, "ATH\n");
				usart3_send_poll(at, strlen(at));	

				// ����gsmͨ��״̬��־
				is_gsm_calling = FALSE;

				times_input_dtmf_password = 0;
				sts_dtmf_command = STS_DTMF_COMMAND_CALL_WAITING;
			}
		}
	}
	// ��ͷ�����رպ�ʼ��������ַ���
	else if(sts_dtmf_command == STS_DTMF_COMMAND_MIC_MUTE_OFF)
	{
		// �����������Ҷ�����
		// accept_dtmf_command();
		
		// ����У��ͨ��ָʾ��(�Է��ֻ����ܿ������ַ���ֻ������������)
		//////////////////////////////////////////////////////
		strcpy(at, "AT+VTS=1\n");
				
		usart3_send_poll(at, strlen(at));	

		// �Ҷϵ绰
		strcpy(at, "ATH\n");
		usart3_send_poll(at, strlen(at));	

		// �Ҷϵ绰��dtmf����״̬����
		sts_dtmf_command = STS_DTMF_COMMAND_CALL_WAITING;

		// ����gsmͨ��״̬��־
		is_gsm_calling = FALSE;

		times_input_dtmf_password = 0;
		//////////////////////////////////////////////////////
		
		printf("to detect dtmf command...\r\n");

		len = strlen(dtmf);

		// ��dtmf����е��Ǻ��滻�ɶ��ţ��Լ����ı������ʽ��
		for(i = 0; i < len; i++)
		{
			if(dtmf[i] == '*')
			{
				dtmf[i] = ',';
			}
		}

		printf("systick = %d, converted dtmf = %s\r\n", systick, dtmf);

		// ����ʽת�����dtmf������뵽ȫ���ı�������С�
		que_cmd_wr(CMD_SRC_DTMF, dtmf, "", "");
	}	

	// DTMF��⼰�����������ȵ�RELEASE��Ϣ���յ�����ܽ�RING�жϱ�־��λ!!!
}

// ���������������(����)��ע����ҪSIM����ͨ������ʾ���ܡ�
void ana_num_ring(int rings)
{
	// ��⵽������Ϣ�ҵ绰�ҶϺ󣬸��������������Ӧ�Ĵ���
	switch(rings)
	{
		case 1:	// ���������һ�㲻��ȷ����⵽������룬��˲��ʺ���Ҫ��Ϊ��Ҫ������������¼�����	
					
			break;
		case 2:	// TBD		

			break;
		case 3:	// TBD

			break;
		case 4:	// TBD

			break;			
		default:
			break;
	}
}
#endif

// ͨ����Ϊ��������ķ�ʽ���������ָ��������Դ�������ַ���(��#)��������Դ���롢����Ŀ����롢��������Ľ��ջ��档
int send_command_virtual(unsigned char cmd_src, char* cmd_str, char* pnf_src, char* pnf_dst, char* reply)
{
	int		ret;

	ret = handle_cmd(cmd_src, cmd_str, pnf_src, pnf_dst, reply);
	
	return ret;
}

/********************************************** ����������д����� ****************************************************/

// ��鴮��1�Ƿ���յ�����еĻ�����
void check_buf_com(void)
{
	// printf("enter <check_buf_com>.\r\n");
	
	transaction_enter();

	ana_buf_com();

	transaction_quit();
}

#ifdef USING_DEV_GSM

/********************************************** ����������д����� ****************************************************/

// ��ָ���Ĵ���(һ���Լ���һֱ���)���SMS���ն��У�����SMS�Ļ������֮��
// ע: �˺������ڽ���GSM���ڽ����жϴ��������յ�SMSԭʼ���ݣ�׼ȷ��˵�ǽ���+CMT
// ��Ϣ����Ϣ������Ϣͷ����Ϣ�������֣���Ϣͷ�е����ݸ�ʽ���ն���Ƴ��̶��塣SMS��
// �û����ն˽�������Ҫ�����������Ϣ�������ݵ����Ҳ�϶ࡣ
void check_que_sms(int check_mode)
{	
	// printf("enter <check_que_sms>.\r\n");
	
	if(check_mode == CHECK_MODE_CONTINUOUS)
	{
		// ���������д�����Ķ���
		while(cnt_sms_cmd_rd < cnt_sms_cmd_wr)
		{
			// printf("%d sms command pending.\r\n", (cnt_sms_cmd_wr-cnt_sms_cmd_rd));
			
			transaction_enter();
			
			ana_que_sms();		

			transaction_quit();
		}
	}
	else if(check_mode == CHECK_MODE_ONESHOT)
	{
		// ������һ������������
		if(cnt_sms_cmd_rd < cnt_sms_cmd_wr)
		{			
			// printf("%d sms command pending.\r\n", (cnt_sms_cmd_wr-cnt_sms_cmd_rd));;
			
			transaction_enter();
			
			ana_que_sms();

			transaction_quit();
		}
	}
}

// ���ָ�������Ƿ�����Ȩ���������б��С�
ERR_NO check_pn_operation(char* pn)
{
	int 		i;

	int			len = strlen(pn);

	// ��������com��tcp����������Ȩ
	if(len <= 0 || len == 1)
	{
		printf("no need to be authorized for command from com or TCP.\r\n");
		
		return OK;
	}	
	// �������Զ��ŵ���������Ȩ���������б����Ƿ���ڸú���
	else
	{
		for(i = 0;i < MAX_PN_OPERATON; i++)
		{
			// �ȼ����������Ƿ�����Ȩ�����б���
			if(!__strcmp(pn_operation[i], pn))
			{			
				printf("%s authorized.\r\n", pn);
				
				return OK;
			}		
		}

		printf("%s NOT authorized.\r\n", pn);

		return ERR_AUTH_NO_ENTRANCE;				// δ�ҵ�ƥ�����
	}
}

/********************************************************************************************************************
												�����������
*********************************************************************************************************************/
// ���ö�������������롣
// szdxmm,old_password,new_password,new_password#
int set_sms_password(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	int len1, len2, len3;

	len1 = strlen(argv[1]);
	len2 = strlen(argv[2]);
	len3 = strlen(argv[3]);
	
	if(len1 != STD_LEN_OF_CMD_PWD || len2 != STD_LEN_OF_CMD_PWD || len3 != STD_LEN_OF_CMD_PWD)
	{
		// ���볤�ȹ̶�Ϊ6�ַ���
		strcpy(reply, "\xE5\xAF\x86\xE7\xA0\x81\xE9\x95\xBF\xE5\xBA\xA6\xE5\x9B\xBA\xE5\xAE\x9A\xE4\xB8\xBA\x36\xE5\xAD\x97\xE7\xAC\xA6\xEF\xBC\x81");

		return NG;
	}

	if(len2 != len3)
	{
		// �����벻һ�£�
		strcpy(reply, "\xE6\x96\xB0\xE5\xAF\x86\xE7\xA0\x81\xE4\xB8\x8D\xE4\xB8\x80\xE8\x87\xB4\xEF\xBC\x81");

		return NG;
	}

	if(strcmp(cmd_pwd_sms, argv[1]))
	{
		// ���������
		strcpy(reply, "\xE6\x97\xA7\xE5\xAF\x86\xE7\xA0\x81\xE9\x94\x99\xE8\xAF\xAF\xEF\xBC\x81");

		return NG;
	}

	// ����ram�е�����
	strcpy(cmd_pwd_sms, argv[2]);

	// ����Flash�еĲ���
	bkp_str_set(ENV_CMD_PWD_SMS_START, ENV_CMD_PWD_SMS_SIZE, argv[2]);

	// �������óɹ���
	strcpy(reply, "\xE5\xAF\x86\xE7\xA0\x81\xE8\xAE\xBE\xE7\xBD\xAE\xE6\x88\x90\xE5\x8A\x9F\xEF\xBC\x81");

	return OK;
}

// ǿ�������豸��������ΪĬ�ϳ�ʼ���롣
// czczmm,imei#
int rev_sms_password(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	if(strcmp(gsm_imei, argv[1]))
	{
		// IMEI����
		strcpy(reply, "\x49\x4D\x45\x49\xE9\x94\x99\xE8\xAF\xAF\xEF\xBC\x81");

		return NG;
	}

	strcpy(cmd_pwd_sms, DEF_CMD_PWD_SMS);

	bkp_str_set(ENV_CMD_PWD_SMS_START, ENV_CMD_PWD_SMS_SIZE, DEF_CMD_PWD_SMS);

	// �������óɹ���
	strcpy(reply, "\xE5\xAF\x86\xE7\xA0\x81\xE9\x87\x8D\xE7\xBD\xAE\xE6\x88\x90\xE5\x8A\x9F\xEF\xBC\x81");

	return OK;
}

// ����dtmf����������롣
// szszmm,cmd_pwd_sms,new_password,new_password#
int set_dtmf_password(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	int len1, len2, len3;

	len1 = strlen(argv[1]);
	len2 = strlen(argv[2]);
	len3 = strlen(argv[3]);
	
	if(len1 != STD_LEN_OF_CMD_PWD || len2 != STD_LEN_OF_CMD_PWD || len3 != STD_LEN_OF_CMD_PWD)
	{
		// ���볤�ȹ̶�Ϊ6�ַ���
		strcpy(reply, "\xE5\xAF\x86\xE7\xA0\x81\xE9\x95\xBF\xE5\xBA\xA6\xE5\x9B\xBA\xE5\xAE\x9A\xE4\xB8\xBA\x36\xE5\xAD\x97\xE7\xAC\xA6\xEF\xBC\x81");

		return NG;
	}

	if(len2 != len3)
	{
		// �����벻һ�£�
		strcpy(reply, "\xE6\x96\xB0\xE5\xAF\x86\xE7\xA0\x81\xE4\xB8\x8D\xE4\xB8\x80\xE8\x87\xB4\xEF\xBC\x81");

		return NG;
	}

	if(strcmp(cmd_pwd_sms, argv[1]))
	{
		// ������������������
		strcpy(reply, "\xE7\x9F\xAD\xE4\xBF\xA1\xE5\x91\xBD\xE4\xBB\xA4\xE6\x93\x8D\xE4\xBD\x9C\xE5\xAF\x86\xE7\xA0\x81\xE9\x94\x99\xE8\xAF\xAF\xEF\xBC\x81");

		return NG;
	}

	// ����ram�е�����
	strcpy(cmd_pwd_dtmf, argv[2]);

	// ����Flash�еĲ���
	bkp_str_set(ENV_CMD_PWD_DTMF_START, ENV_CMD_PWD_DTMF_SIZE, argv[2]);

	// �������óɹ���
	strcpy(reply, "\xE5\xAF\x86\xE7\xA0\x81\xE8\xAE\xBE\xE7\xBD\xAE\xE6\x88\x90\xE5\x8A\x9F\xEF\xBC\x81");

	return OK;
}

// ��ָ���ĺ�����ӵ����������б�
// tjczhm,pn,cmd_pwd_sms#
int add_pn_operation(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	int 	i;

	// ������롣
	if(strcmp(cmd_pwd_sms, argv[2]))
	{
		// �������
		strcpy(reply, "\xE5\xAF\x86\xE7\xA0\x81\xE9\x94\x99\xE8\xAF\xAF\xEF\xBC\x81");

		return NG;
	}
	
	// ������Ӻ����ʽ����
	if(format_pn(argv[1]) != OK)
	{
		// ����Ƿ���
		strcpy(reply, "\xE5\x8F\xB7\xE7\xA0\x81\xE9\x9D\x9E\xE6\xB3\x95\xEF\xBC\x81");
		
		return NG;
	}	

	// ������Ȩ�����Ƿ��Ѵ���
	for(i = 0; i < MAX_PN_OPERATON; i++)
	{
		if(!__strcmp(pn_operation[i], argv[1]))
		{
			// ����Ȩ��
			sprintf(reply, "%s\xE5\xB7\xB2\xE6\x8E\x88\xE6\x9D\x83\xEF\xBC\x81", argv[1]);

			return OK;
		}
	}

	// �����Ȩ�����б��Ƿ��п������
	for(i = 0; i < MAX_PN_OPERATON; i++)
	{
		if(strlen(pn_operation[i]) == 0)
		{
			strcpy(pn_operation[i], argv[1]);
			
			bkp_str_set(ENV_PN_OPERATION_START+i*ENV_PN_OPERATION_SIZE, ENV_PN_OPERATION_SIZE, (char*)argv[1]);

			// ����Ȩ��
			sprintf(reply, "%s\xE8\xA2\xAB\xE6\x8E\x88\xE6\x9D\x83\xEF\xBC\x81", argv[1]);

			return OK;
		}
	}

	// ��Ȩ��������
	sprintf(reply, "\xE6\x8E\x88\xE6\x9D\x83\xE5\x8F\xB7\xE7\xA0\x81\xE6\xBB\xA1\xEF\xBC\x81");

	return NG;
}

// �Ӳ��������б���ɾ��ָ�����롣
// scczhm,pn,cmd_pwd_sms#
int del_pn_operation(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	int 	i;

	// ������롣
	if(strcmp(cmd_pwd_sms, argv[2]))
	{
		// �������
		strcpy(reply, "\xE5\xAF\x86\xE7\xA0\x81\xE9\x94\x99\xE8\xAF\xAF\xEF\xBC\x81");

		return NG;
	}
	
	// ������Ӻ����ʽ����
	if(format_pn(argv[1]) != OK)
	{
		// ����Ƿ���
		strcpy(reply, "\xE5\x8F\xB7\xE7\xA0\x81\xE9\x9D\x9E\xE6\xB3\x95\xEF\xBC\x81");
		
		return NG;
	}	

	for(i = 0; i < MAX_PN_OPERATON; i++)
	{
		if(!strcmp(pn_operation[i], argv[1]))
		{
			strcpy(pn_operation[i], "");

			bkp_str_set(ENV_PN_OPERATION_START+i*ENV_PN_OPERATION_SIZE, ENV_PN_OPERATION_SIZE, "");

			// ��ɾ����
			sprintf(reply, "%s\xE8\xA2\xAB\xE5\x88\xA0\xE9\x99\xA4\xEF\xBC\x81", argv[1]);

			return OK;
		}
	}

	// δ�ҵ���
	sprintf(reply, "%s\xE6\x9C\xAA\xE6\x89\xBE\xE5\x88\xB0\xEF\xBC\x81", argv[1]);

	return OK;
}

// ������������б�(����������û�)��
// qkczhm,cmd_pwd_sms#
int clr_pn_operation(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	int 	i;

	// ������롣
	if(strcmp(cmd_pwd_sms, argv[1]))
	{
		// �������
		strcpy(reply, "\xE5\xAF\x86\xE7\xA0\x81\xE9\x94\x99\xE8\xAF\xAF\xEF\xBC\x81");

		return NG;
	}

	for(i = 0; i < MAX_PN_OPERATON; i++)
	{
		strcpy(pn_operation[i], "");

		bkp_str_set(ENV_PN_OPERATION_START+i*ENV_PN_OPERATION_SIZE, ENV_PN_OPERATION_SIZE, "");		
	}
	
	// ��Ȩ���뱻��գ�
	sprintf(reply, "\xE6\x8E\x88\xE6\x9D\x83\xE5\x8F\xB7\xE7\xA0\x81\xE8\xA2\xAB\xE6\xB8\x85\xE7\xA9\xBA\xEF\xBC\x81");		
	
	return OK;
}

// ��ѯ��ͨ��Ȩ�����б�
// cxczhm,cmd_pwd_sms#
int inq_pn_operation(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	int 	i;
	char	pn_exist[(MAX_LEN_PN+1)*MAX_PN_OPERATON];
	int 	len  = 0;

	// ������롣
	if(strcmp(cmd_pwd_sms, argv[1]))
	{
		// �������
		strcpy(reply, "\xE5\xAF\x86\xE7\xA0\x81\xE9\x94\x99\xE8\xAF\xAF\xEF\xBC\x81");

		return NG;
	}

	strcpy(pn_exist, "");

	for(i = 0; i < MAX_PN_OPERATON; i++)
	{
		if(strlen(pn_operation[i]) > 0)
		{
			len += sprintf(pn_exist+len, "%s",  pn_operation[i]);
		}
	}

	// ��Ȩ���룺   �� 
	sprintf(reply, "\xE6\x8E\x88\xE6\x9D\x83\xE5\x8F\xB7\xE7\xA0\x81\xEF\xBC\x9A%s\xE3\x80\x82", pn_exist);

	return OK;
}
#endif

/********************************************************************************************************************
												GPS��������
*********************************************************************************************************************/
#ifdef USING_DEV_GPS

// ���ݽ������������Ӧ���������
int swt_gps_dump(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{	
	if(!__strcmp(argv[1], "1"))
	{
		sw_gps_forward = ON;

		strcpy(reply, "gps dump set to ON.");	
	}
	else if(!__strcmp(argv[1], "0"))
	{
		sw_gps_forward = OFF;

		strcpy(reply, "gps dump set to OFF.");	
	}
	
	return OK;
}

// 2014-04-12 ���ڶ�λģʽ�������
int swt_gps_ppos(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{	
	if(!__strcmp(argv[1], "1"))
	{
		sw_periodic_pos = ON;

		// ������ʱ��λtimer
		swtimer_periodic_pos = cycle_periodic_pos*1000/SYSTICK_PERIOD;

		strcpy(reply, "periodic pos mode set to ON.");	
	}
	else if(!__strcmp(argv[1], "0"))
	{
		sw_periodic_pos = OFF;

		// ���յ��ر����ڶ�λ������������Ե���δ��������ڶ�λ����
		sem_periodic_pos = 0;

		strcpy(reply, "periodic pos mode set to OFF.");	
	}
	
	return OK;
}

#endif

#ifdef USING_DEV_GSM
/********************************************************************************************************************
												GSM/GPRS��������
*********************************************************************************************************************/

// ����/�˳�GSM����ģʽ(GSM����ת��)��
int set_gsm_debug(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{	
	if(strlen(pn) > 0)
	{
		strcpy(reply, "gsm can be set to debug mode only via com.");
		
		return NG;
	}
	
 	if(!strcmp(argv[1], "1"))
 	{
		printf("Enter GSM debug mode:");

		gsm_wakeup();
		
		usart3_redir_usart1(); 

		return OK;
 	}
	else if(!strcmp(argv[1], "0"))
	{
		is_gsm_forwarding = FALSE;

		printf("Quit from GSM debug mod.");

		return OK;
	}
	else
	{
		strcpy(reply, "unsupported argument.");

		return NG;
	}
}

int set_gsm_reset(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	// printf("to reset GSM...");

	gsm_reset();

	strcpy(reply, "gsm reseted.");	

	return OK;
}

int set_gsm_recover(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	// printf("to recover GSM...");

	if(gsm_recover() == OK)
	{
		strcpy(reply, "gps recovedied.");	
		
		return OK;
	}
	else
	{
		strcpy(reply, "failed to recovery gsm.");	
		
		return NG;
	}
}

// ����GPRS APN��
int set_gsm_apn(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{	
	// ����RAM�еĲ���
	strcpy(gsm_apn, argv[1]);

	// ����Flash�еĲ���
	bkp_str_set(ENV_GPRS_APN_START, ENV_GPRS_APN_SIZE, argv[1]);

	// GPRS APN��Ϊ��
	sprintf(reply, "\x47\x50\x52\x53\x20\x41\x50\x4E\xE8\xAE\xBE\xE4\xB8\xBA\xEF\xBC\x9A%s\xE3\x80\x82", argv[1]);
	
	return OK;
}

int set_gsm_telecode(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	// ����RAM�еĲ���
	strcpy(gsm_telecode, argv[1]);

	// ����Flash�еĲ���
	bkp_str_set(ENV_GSM_TELECODE_START, ENV_GSM_TELECODE_SIZE, argv[1]);

	// ����������Ϊ��  ��
	sprintf(reply, "\xE5\x9B\xBD\xE5\xAE\xB6\xE5\x8C\xBA\xE5\x8F\xB7\xE8\xAE\xBE\xE4\xB8\xBA\xEF\xBC\x9A%s\xE3\x80\x82", argv[1]);

	return OK;
}

// ��ѯGSMģ���IMEI��
int inq_gsm_imei(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	gsm_wakeup();

	if(strlen(gsm_imei) <= 0)
	{
		// ��ѯIMEIʧ�ܡ�
		strcpy(reply, "\xE6\x9F\xA5\xE8\xAF\xA2\x49\x4D\x45\x49\xE5\xA4\xB1\xE8\xB4\xA5\xE3\x80\x82");

		gsm_sleep();
		
		return NG;
	}

	gsm_sleep();

	// GSM IMEI��   ��
	sprintf(reply, "\x47\x53\x4D\x20\x49\x4D\x45\x49\xEF\xBC\x9A%s\xE3\x80\x82", gsm_imei);

	return OK;
}

// ��ѯGSMģ������汾�š�
int inq_gsm_swversion(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	char	version[64+1];

	gsm_wakeup();

	if(gsm_get_swversion((char*)version, 64) != OK)
	{
		strcpy(reply, "failed to inquire GSM sofware version.");

		gsm_sleep();
		
		return NG;
	}

	gsm_sleep();

	// �����������
	sprintf(reply, "GSM SW version is: %s.", version);

	return OK;
}

// ��ѯGSMģ��������ź�ǿ�ȡ�
int inq_gsm_signal(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	int		rssi;

	gsm_wakeup();

	if(gsm_get_rssi((int*)&rssi) != OK)
	{
		// ��ѯRSSIʧ�ܡ�
		strcpy(reply, "\xE6\x9F\xA5\xE8\xAF\xA2\x52\x53\x53\x49\xE5\xA4\xB1\xE8\xB4\xA5\xE3\x80\x82");	

		gsm_sleep();
		
		return NG;
	}	

	gsm_sleep();

	// GSM�ź�ǿ��=   ��
	sprintf(reply, "\x47\x53\x4D\xE4\xBF\xA1\xE5\x8F\xB7\xE5\xBC\xBA\xE5\xBA\xA6\x3D%d\xE3\x80\x82", rssi);
	
	return OK;
}

// ��ѯ����GPRS���ӵ�״̬��
int inq_gsm_connection(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	gsm_wakeup();
	
	if(gprs_soc_status(GPRS_ID1)== OK)
	{
		strcpy(reply, "connected to APP Server, ");
	}
	else
	{
		strcpy(reply, "disconnected to APP Server, ");
	}

	if(gprs_soc_status(GPRS_ID0)== OK)
	{
		strcat(reply, "connected to MAP Server, ");
	}
	else
	{
		strcat(reply, "disconnected to MAP Server, ");
	}

	if(gprs_soc_status(GPRS_ID2)== OK)
	{
		strcat(reply, "connected to EPH Server.");
	}
	else
	{
		strcat(reply, "disconnected to EPH Server.");
	}

	gsm_sleep();
	
	return OK;
}
#endif

/********************************************************************************************************************
												�豸��������
*********************************************************************************************************************/

// ���ն˻ָ��������á�
int set_dev_factory(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	// ���ħ�������Ա��������ܰ���Ĭ�����ó�ʼ����������
	bkp_u16_set(ENV_SYS_FIRST_RUNNING_START, 0xFFFF);

	printf("to reset device.\r\n");

	if(cmd_src == CMD_SRC_SMS || cmd_src == CMD_SRC_DTMF)
	{
		gsm_wakeup();

		// �ָ��������ã�׼�������豸������
		gsm_send_sms(pn, "\xE6\x81\xA2\xE5\xA4\x8D\xE5\x87\xBA\xE5\x8E\x82\xE8\xAE\xBE\xE7\xBD\xAE\xEF\xBC\x8C\xE5\x87\x86\xE5\xA4\x87\xE9\x87\x8D\xE5\x90\xAF\xE8\xAE\xBE\xE5\xA4\x87\xE3\x80\x82\xE3\x80\x82\xE3\x80\x82");
	}

	// �����λmcu
	mcu_reset();

	return OK;
}

int set_dev_shutdown(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{	
	printf("to shutdown system...");

	if(cmd_src == CMD_SRC_SMS || cmd_src == CMD_SRC_DTMF)
	{
		gsm_wakeup();

		// �豸���ػ���
		gsm_send_sms(pn, "\xE8\xAE\xBE\xE5\xA4\x87\xE5\xB0\x86\xE5\x85\xB3\xE6\x9C\xBA\xE3\x80\x82");
	}

	gsm_exit();

	gps_power_down();

	MMA845x_Standby();

	mcu_shutdown();

	return OK;
 }

int set_dev_reset(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	printf("to reset system...");

	if(cmd_src == CMD_SRC_SMS || cmd_src == CMD_SRC_DTMF)
	{
		gsm_wakeup();

		// �豸��������
		gsm_send_sms(pn, "\xE8\xAE\xBE\xE5\xA4\x87\xE5\xB0\x86\xE9\x87\x8D\xE5\x90\xAF\xE3\x80\x82");
	}
	
	gsm_exit();

	gps_power_down();

	MMA845x_Standby();

	mcu_reset();

	return OK;
}

const char reply_sw_dev_led1[3][7]=
{
	"\xE7\xBA\xA2\xE8\x89\xB2\0",		// R: ��ɫ
	"\xE7\xBB\xBF\xE8\x89\xB2\0",		// G: ��ɫ
	"\xE8\x93\x9D\xE8\x89\xB2\0"		// B: ��ɫ
};

const char reply_sw_dev_led2[2][10]=
{
	"\xE7\x86\x84\xE7\x81\xAD\xE3\x80\x82\0",		// Ϩ��
	"\xE7\x82\xB9\xE4\xBA\xAE\xE3\x80\x82\0",		// ������
};

// ��/�ر�ָ����ɫ��LED�ơ�
int swt_dev_led(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	int		led_no;	// 0 / 1 / 2, R / G / B
	SWITCH	sw;		// 0 / 1, OFF /  ON

	led_no 	= atoi(argv[1]);
	sw 		= atoi(argv[2]);
	
	// ����RAM�еĲ���
	if(sw == ON)
	{
		MCU_GPIO_LOW(GPIO_LED_R+led_no);
	}
	else if(sw == OFF)
	{
		MCU_GPIO_HIGH(GPIO_LED_R+led_no);
	}
	else
	{
		// �����Ƿ���
		strcpy(reply, info_invalid_argument);

		return NG;
	}

	sprintf(reply, "%sLED%s", (char*)(reply_sw_dev_led1[led_no]), (char*)(reply_sw_dev_led2[sw]));
	
	return OK;
}

int inq_dev_hwversion(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	char	version[16];

	bkp_str_get(ENV_VERSION_HARDWARE_START, ENV_VERSION_HARDWARE_SIZE, version);

	// Ӳ���汾��  ��
	sprintf(reply, "\xE7\xA1\xAC\xE4\xBB\xB6\xE7\x89\x88\xE6\x9C\xAC\xEF\xBC\x9A%s\xE3\x80\x82", version);

	return OK;
}

int inq_dev_swversion(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	char	version[16];

	bkp_str_get(ENV_VERSION_SOFTWARE_START, ENV_VERSION_SOFTWARE_SIZE, version);

	// ����汾��  ��
	sprintf(reply, "\xE8\xBD\xAF\xE4\xBB\xB6\xE7\x89\x88\xE6\x9C\xAC\xEF\xBC\x9A%s\xE3\x80\x82", version);

	return OK;
}

// ��ѯ��ص�ѹ��
int inq_dev_batvol(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	// ��ص�ѹ��   ��
	sprintf(reply, "\xE7\x94\xB5\xE6\xB1\xA0\xE7\x94\xB5\xE5\x8E\x8B\xEF\xBC\x9A%1.2f V\xE3\x80\x82", batvol_measure()/(float)100);

	return OK;
}

// ��ѯ�豸����״����
int inq_dev_powersupply(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	int		len = 0;

	strcpy(reply, "");

#ifdef USING_PWR_ACC
	// ����acc��Դ״̬
	if(MCU_GPIO_READ(GPIO_ACCPWR_DET) == 1)
	{
		sts_power |= (1<<BIT_STS_POWER_ACC);
	}
	else
	{
		sts_power &= ~(1<<BIT_STS_POWER_ACC);
	}
#endif

#ifdef USING_PWR_EXT
	// ����ext��Դ״̬
	if(MCU_GPIO_READ(GPIO_EXTPWR_DET) == 1)
	{
		sts_power |= (1<<BIT_STS_POWER_EXT);
	}
	else
	{
		sts_power &= ~(1<<BIT_STS_POWER_EXT);
	}
#endif

	// ������ص�ѹ
	de_batvol = batvol_measure();	
	
#ifdef USING_PWR_EXT
	len += sprintf(reply+len, "\xE5\xA4\x96\xE6\x8E\xA5\xE7\x94\xB5\xE6\xBA\x90\xEF\xBC\x9A%s\xEF\xBC\x9B", info_switch[GET_STS_POWER(BIT_STS_POWER_EXT)]);
#endif

#ifdef USING_PWR_ACC
	len += sprintf(reply+len, "\x41\x43\x43\xE7\x94\xB5\xE6\xBA\x90\xEF\xBC\x9A%s\xEF\xBC\x9B", info_switch[GET_STS_POWER(BIT_STS_POWER_ACC)]);
#endif

#ifdef USING_PWR_BAT
	if(de_batvol < MAX_ADCVOL_FOR_BAT)
	{
		// ��ص�ѹ��  V��
		len += sprintf(reply+len, "\xE7\x94\xB5\xE6\xB1\xA0\xE7\x94\xB5\xE5\x8E\x8B\xEF\xBC\x9A%1.2f V\xE3\x80\x82", de_batvol/(double)100);

		sts_power |= (1<<BIT_STS_POWER_BAT);
	}
	else
	{
		// ��ؿ���δ�ӡ�
		len += sprintf(reply+len, "\xE7\x94\xB5\xE6\xB1\xA0\xE5\x8F\xAF\xE8\x83\xBD\xE6\x9C\xAA\xE6\x8E\xA5\xE3\x80\x82");

		sts_power &= ~(1<<BIT_STS_POWER_BAT);
	}	
#endif
	
	return OK;
}

// ��ѯ�豸�ĵ�ǰʱ��(�����豸ʱ������Ȼʱ��ͬ����Ż᷵��ʱ����Ϣ)��
int inq_dev_time(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	unsigned int	seconds;

	T_TIME_DHMS		dhms;

	unsigned char	systime[6];
	
	int				len = 0;

	// ����ϵͳ���еĳ���ʱ��(��MCU�ϵ縴λ��ʼ��ʱ)
	seconds		= systick*SYSTICK_PERIOD/1000;

	// ������ת��Ϊ�졢Сʱ���֡���ĸ�ʽ
	second_to_dhms(seconds, (T_TIME_DHMS*)&dhms);

	// ����ʱ�䣺  ��   Сʱ  ��  �룻
	len += sprintf(reply+len, "\xE8\xBF\x90\xE8\xA1\x8C\xE6\x97\xB6\xE9\x97\xB4\xEF\xBC\x9A%02d\xE5\xA4\xA9 %02d\xE5\xB0\x8F\xE6\x97\xB6 %02d\xE5\x88\x86\xE9\x92\x9F %02d\xE7\xA7\x92\xEF\xBC\x9B", dhms.day, dhms.hour, dhms.minute, dhms.second);

	// ����ϵͳά���ľ���ʱ��(��������Ȼʱ��ͬ�����������)
	if(g_sync_point.synmode == SYNC_BY_NULL)
	{
		// ϵͳʱ�䣺δͬ����
		len += sprintf(reply+len, "\xE7\xB3\xBB\xE7\xBB\x9F\xE6\x97\xB6\xE9\x97\xB4\xEF\xBC\x9A\xE6\x9C\xAA\xE5\x90\x8C\xE6\xAD\xA5\xE3\x80\x82");
	}
	else
	{		
		tick_to_nattime(systick, systime);

		// ϵͳʱ�䣺  ��
		len += sprintf(reply+len, "\xE7\xB3\xBB\xE7\xBB\x9F\xE6\x97\xB6\xE9\x97\xB4\xEF\xBC\x9A%04d-%02d-%02d %02d:%02d:%02d\xE3\x80\x82", 2000+systime[0], systime[1], systime[2], systime[3], systime[4], systime[5]);	
	}

	return OK;
}


/********************************************************************************************************************
												������������
*********************************************************************************************************************/
#ifdef USING_FUNC_UPGRADE_COM

// ͨ���������س���
// ע: ���ڱ��ݳ���������ط����ݹ���ͬһ�洢�ռ䣬����ѭ���м�����������ط�ʱ���ȡ�ط������б���Ķ�д�α���ֵ��
// ���Ϊ�˲�ʹ�Ķ�ȡ�ط����ݶ�д�α����������ѭ����Ӧ�ڽ��ճ���󽫱��ݳ���ռ��������ʼ���ط���ر�����
// 
// ��ʽ: upd-img-com,�����ļ��ֽ���,�����ļ�CRCУ����#
//       upd-img-com,59660,0xb9a9#
int get_image_file_via_com(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	int i = 0;
	int j = 0;

	unsigned char	frame[FLASH_PAGE_SIZE];
	unsigned char	pages = 0;
	unsigned int 	size = 0;

	unsigned char	byte;
	unsigned short	crc16 = 0x0000;

	// ��ȡbin�ļ����Ⱥ�У��ֵ��Ϣ
	unsigned int 	flen = (unsigned int)__atoi(argv[1]);
	unsigned short	fcrc = (unsigned short)__atoi(argv[2]);
	
	unsigned int 	to = 0;

	// unsigned short	readback = 0x00;

	RxCnt1_rd = RxCnt1_wr;	

	led_switch(GPIO_LED_G, OFF);
	led_switch(GPIO_LED_B, OFF);

	led_switch(GPIO_LED_R, OFF);

	printf("flen=%d\r\n", flen);
	printf("fcrc=0x%04x\r\n", fcrc);
	
	if(flen > BLOCK_APPLICATION_SPACE)
	{
		printf("the max. length of image file is %d.", BLOCK_APPLICATION_SPACE);

		return NG;
	}

	// ��ϵͳ��Ƶ������8/2*16=64MHz,�Ա�ӿ�����ļ�ʱдFlash���ٶ�
	rcc_hsi_64mhz();

	// �ر��˶��;�ֹ��⣬��������ļ�ʱ���Ŵ��ڽ���
	detect_motion(OFF);
	detect_still(OFF);

	printf("system main MCU_CLOCK_MODE is raised to 64MHz.\r\n");
	
	printf("please wait until image area is completely erased!!!\r\n");	
	
	// �������³���ǰ���Flash�еĳ�����±�־
	bkp_u16_set(ENV_APP_UPDATED_START, 0x00);
	
	// ���Ȳ����������ݳ�������
	for(i = 0; i < BLOCK_APPLICATION_SPACE; i+= FLASH_PAGE_SIZE)
	{
		FLASH_ErasePage(BLOCK_APPLICATION_START2+i);

		// �������ݳ�������ʱ��˸���ָʾ��
		if(!((i/FLASH_PAGE_SIZE)&1))			// ������һ��Flash��ʱ������ơ������ڶ���Flash��ʱϨ���ƣ����ѭ����
		{
			led_switch(GPIO_LED_R, ON);	
		}
		else
		{
			led_switch(GPIO_LED_R, OFF);
		}

		WATCHDOG_RELOAD();
	}

	led_switch(GPIO_LED_R, OFF);
	
	printf("image area is completely erased...\r\n");	
	
	printf("please send image file within 15s, or updating will abort!!!\r\n");	
	
	// 15�����û��������ļ����������
	to = systick+15*1000/SYSTICK_PERIOD;	
	
	while((systick < to))
	{
		if(RxCnt1_rd < RxCnt1_wr)
		{
			break;
		}
	
		// ����ι�������������Flashʱ�������Ź����������
		WATCHDOG_RELOAD();
	}
	
	if(systick >= to)
	{
		printf("updating image file aborted as 15s past.\r\n");
		
		goto ERROR_RECEIVE_COM;
	}
	
	to = systick;

	// �Ӵ���1���ջ�������������ļ�����
	while(j < flen)
	{
		// �����������ֽڵĽ���ʱ�䳬��1�룬����Ϊ�����Ѿ�����
		if((systick - to) >= 1000/SYSTICK_PERIOD)
		{
				printf("received %d data is less than %d bytes.\r\n", j-1, flen);
			
				goto ERROR_RECEIVE_COM;
		}

		if(RxCnt1_rd < RxCnt1_wr)
		{
			// ����ÿ���ֽ�ʱ��¼��ǰ��rtcalarn��ֵ
			to = systick;
			
			byte = COM1_RX_RD(RxCnt1_rd++);

			// ����CRCֵ(����ת������ֽ�)
			crc16 = (crc16<<8) ^ crc16tab[((crc16>>8) ^ byte)&0x00FF];

			// ���ܴ���: ���2λ�����2λ�Ե����м�4λ��2λΪ��λ�Ե�
			byte = ((byte&0xC0)>>6) | ((byte&0x0F)<<6) | ((byte&0x30)>>2) | ((byte&0x0C)<<2);
			
			frame[j++ & (FLASH_PAGE_SIZE-1)] = byte;

			// ÿ���յ�1024�ֽھ�д��flash��һҳ(ĩβ���ݿ��ܲ���1024�ֽ�)
			if(!(j & (FLASH_PAGE_SIZE-1)))
			{				
				size = FLASH_PAGE_SIZE;
			
				flash_bytestream_write(BLOCK_APPLICATION_START2+pages*FLASH_PAGE_SIZE, frame, size);			

				pages++;

				if(pages & 1)
				{
					led_switch(GPIO_LED_G, ON);
				}
				else
				{
					led_switch(GPIO_LED_G, OFF);
				}
			}
			else if(j >= flen)
			{				
				// ׼��д��ʣ���ֽ�(����һҳ)
				size = flen-pages*FLASH_PAGE_SIZE;
				
				flash_bytestream_write(BLOCK_APPLICATION_START2+pages*FLASH_PAGE_SIZE, frame, size);			

				pages++;

				if(pages & 1)
				{
					led_switch(GPIO_LED_G, ON);
				}
				else
				{
					led_switch(GPIO_LED_G, OFF);
				}
			}

			WATCHDOG_RELOAD();
		}
	}

	printf("\nto check CRC with crc16 = 0x%02x.\r\n", crc16);	

	// ������õ���CRC�Ƿ���ļ�����ʱЯ����CRC���
	if(crc16 != fcrc)
	{
		printf("error when checking CRC: fcrc = 0x%04x but crc = 0x%04x.\r\n", fcrc, crc16);	

		goto ERROR_RECEIVE_COM;
	}

	// ���ó��򱻸��µı�־
	bkp_u16_set(ENV_APP_UPDATED_START, 0x01);

	printf("\nsuccessfully receive image file with %d bytes.\r\n", j);

	rcc_hsi_8mhz();	

	// ���´��˶��;�ֹ���
	detect_motion(ON);
	detect_still(ON);
	return OK;

ERROR_RECEIVE_COM:
	
	// ������ճ����򽫱��ݳ���������ȫ����(���ط����ݹ���ͬһ�洢�ռ�)��������ѭ���м���ط������α�ʱ��ȡ��ֵ����
	for(i = 0; i < BLOCK_APPLICATION_SPACE; i+= FLASH_PAGE_SIZE)
	{
		FLASH_ErasePage(BLOCK_APPLICATION_START2+i);
	}

	// ����COM����ʧ�ܺ�Ӧ��ϵͳ��ʱ���л���HSI 8MHz������MCU����Ӱ��GPS����
	rcc_hsi_8mhz();	

	// ���´��˶��;�ֹ���
	detect_motion(ON);
	detect_still(ON);

	return NG;
}

/*
	ͨ���������س���Ĺ���:
	----------------------
	1����keil�б�����������Ϊsolidcom.bin��ԭʼ�����ļ���
	2������gps_bin_handler.exe���߽�solidcom.bin�ļ�ת��Ϊ���ܵĳ����ļ������ܺ���ļ���������upd-img-com,59660,0x7dab#��
	3���ڴ��ڵ��������з�������: upd-img-com,59660,0x7dab#������59660Ϊ�����ļ��ֽ�����0x7dab΢�����ļ���CRC(CRC16)У��ֵ��
	4���ڴ��ڵ��������з���������������ַ���(������#)�����ĳ����ļ�����d:\upd-img-com,59660,0x7dab#��
	5���ն������ʼ����flash�еı��ݳ���������ҳ���ճ����ļ����ݲ���ҳд��flash��
	6���ն��������������ļ���Ƚ�ʵ�ʽ��յ����ֽ�������Ҫ���յ��ֽ����Ƿ���ȣ������ٱȽ�ʵ�ʽ��յ����ݵ�CRCУ��ֵ��ԭʼCRCУ��ֵ�Ƿ���ȣ�
	   ������ȣ����ն�����Զ�ִ�������λ����λ��ִ�������صĳ��򣬷���ǰ������¹���ʧ�ܣ��ն�����ָ����³���ǰ��״̬��
*/
int upd_img_com(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	// �ر�USART3 RX�жϣ�������Ŵ������س���
	USART_ITConfig(USART3, USART_IT_RXNE, DISABLE); 
	
	if(get_image_file_via_com(cmd_src, argc, argv, pn, reply) == OK)
	{
		printf("download image file successfully!\r\n");		

		gsm_exit();

		gps_power_down();

		MMA845x_Standby();

		printf("to reset MCU...\r\n");

		// WATCHDOG_RELOAD();
		WATCHDOG_DISABLE();	
	
		// ��������
		mcu_reset();
		
		return OK;
	}
	else
	{
		// �ָ�SUART3 RX�ж�
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); 

		printf("failed to receive image.");
		
		return NG;	
	}
}

#endif


