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
char pn_operation[MAX_PN_OPERATON][MAX_LEN_PN+1];	// 授权号码+权限列表

char	cmd_pwd_sms[STD_LEN_OF_CMD_PWD+1];				// 短信命令操作密码
char	cmd_pwd_dtmf[STD_LEN_OF_CMD_PWD+1];			// 数字命令操作密码
#endif

// 关闭 / 打开
const char info_switch[2][7] = {"\xE5\x85\xB3\xE9\x97\xAD", "\xE6\x89\x93\xE5\xBC\x80"};

// 参数非法！
const char info_invalid_argument[] = "\xE5\x8F\x82\xE6\x95\xB0\xE9\x9D\x9E\xE6\xB3\x95\xEF\xBC\x81";

// 。
const char info_fullstop[] = "\xE3\x80\x82";

// ，
const char info_commma[] = "\xEF\xBC\x8C";

// 命令不支持
const char info_unsupported_command[] = "\xE5\x91\xBD\xE4\xBB\xA4\xE4\xB8\x8D\xE6\x94\xAF\xE6\x8C\x81\xEF\xBC\x81";

// 参数个数错误！
const char info_incorrect_argc[] = "\xE5\x8F\x82\xE6\x95\xB0\xE4\xB8\xAA\xE6\x95\xB0\xE9\x94\x99\xE8\xAF\xAF\xEF\xBC\x81";

#define MAX_INDEX_CMD_WOTHOUT_PWD			6

// 全局命令列表
const T_CMD_TXT	cmd_list[MAX_COMMAND_TXT_NUM]=
{
#ifdef USING_DEV_GSM
	// 操作号码(短信发送相关命令时无需事先被授权，仅提供短信命令操作密码即可)
	{"szdxmm", 		"1001",		set_sms_password, 	3,	3},		// 设置短信密码
	{"czdxmm",		"1002",		rev_sms_password, 	0,	0},		// 重置短信密码
	{"szszmm", 		"1003",		set_dtmf_password, 3,	3},		// 设置数字密码
		
	{"tjczhm", 		"1004",		add_pn_operation, 	2,	2}, 		// 添加操作号码
	{"qkczhm", 		"1005",		clr_pn_operation, 	0,	0}, 		// 清空操作号码
	{"scczhm", 		"1006",		del_pn_operation, 	2,	2,}, 		// 删除操作号码
	{"cxczhm", 		"1007",		inq_pn_operation,	0,	0}, 		// 查询操作号码
#endif

	// dev相关	 
	{"hfccsz", 		"1020",		set_dev_factory, 	0,	0}, 		// 恢复出厂设置
	{"sbcq", 			"1021",		set_dev_reset, 		0,	0}, 		// 设备重启
	{"sbgj", 			"",			set_dev_shutdown, 	0,	0}, 		// 设备关机
	{"cxdcdy", 		"1024",		inq_dev_batvol, 		0,	0}, 		// 查询电池电压
	{"cxsbgd", 		"1025",		inq_dev_powersupply, 	0,	0}, 	// 查询设备供电
	{"cxsbsj", 		"1026",		inq_dev_time, 		0,	0}, 		// 查询设备时间
	{"cxyjbb", 		"",			inq_dev_hwversion, 	0,	0}, 		// 查询硬件版本
	{"cxrjbb", 		"",			inq_dev_swversion, 	0,	0}, 		// 查询软件版本

	// I/O相关
	{"kgled", 			"1028",		swt_dev_led,	 	2,	2},		// 开关LED
		
	// gsm相关
#ifdef USING_DEV_GSM	
	{"cximei", 		"1030",		inq_gsm_imei, 		0,	0}, 		// 查询IMEI
	{"cxwlxh", 		"1031",		inq_gsm_signal, 		0,	0}, 		// 查询网络信号
	{"cxgjbb", 		"",			inq_gsm_swversion, 	0,	0}, 		// 查询固件版本
	{"cxwllj", 		"",			inq_gsm_connection,	0,	0}, 		// 查询网络连接
	{"szapn", 		"",			set_gsm_apn, 		1,	1}, 			// 设置APN
	{"szgjqh", 		"1035",		set_gsm_telecode, 	1,	1}, 		// 设置国家区号
	{"tsgsm", 		"",			set_gsm_debug, 		1,	1}, 		// 调试gsm
	{"hfgsm", 		"",			set_gsm_recover, 	0,	0}, 			// 恢复gsm
	{"cqgsm", 		"",			set_gsm_reset, 		0,	0}, 		// 重启gsm
#endif

	// gps相关
	#ifdef USING_DEV_GPS
	{"tsgps", 			"1042",		swt_gps_dump, 		1,	1}, 		// 调试gps
	{"zqdw", 			"1043",		swt_gps_ppos, 		1,	1}, 		// 开关周期定位模式	
	#endif

	// img相关
	{"xzcx", 			"1045",		upd_img_com, 		2,	2}, 		// 下载程序
};

/********************************************************************************************************************
											命令处理通用函数
*********************************************************************************************************************/

// 将命令字符串以逗号为间隔符划分成几个区域，以便进一步解析。
ERR_NO destruct_command(char* command, int* argc, char (*argv)[MAX_LEN_CMD_DMN+1])
{
	int		i = 0;
	int		j = 0;
	int		k = 0;

	BOOL	ended = FALSE;		// 命令字符串是否以#结束的标志
	
	int		len = strlen(command);

	// printf("command in destruct_command(): %s\r\n", command);
		
	for(i = 0; i < len; i++)
	{		
		if(IS_TXT_COMMAND_CHARACTER(command[i]) == FALSE)
		{
			return -1;
		}

		// 提取各个命令域
		if(command[i] == ',')			// 若当前字符为逗号，则当前命令域结束
		{
			if(k > MAX_LEN_CMD_DMN)
			{
				return -3;
			}
			
			// 指向下一个命令域
			argv[j][k++] = '\0';	

			j++;
			
			k = 0;				

			if(j >= MAX_NUM_CMD_DMN)
			{
				return -4;
			}
		}
		// 若当前字符为井号，则整条命令结束
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
		// 若命令字符合法，则当前字符既非逗号也非井号时，则为普通字母，因此将其逐个保存
		else
		{
			if(k > MAX_LEN_CMD_DMN)
			{
				return -3;
			}
			
			argv[j][k++] = command[i];
		}
	}	

	// 检查命令是否以#号结尾（上面的循环可因else if(command[i] == '#')条件退出，也可因i < len条件退出）
	if(ended == FALSE)
	{		
		return -5;
	}

	// 设置整个命令字符串中域的个数
	*argc = j;
	
	return OK;
}

// 在命令列表中按字符串查找命令的序号。
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

// 在命令列表中按数字串查找命令的序号。
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

// 在全局文本命令列表中查找命令并执行(命令处理结果通过reply缓冲返回)。
ERR_NO handle_cmd(unsigned char cmd_src, char* cmd_str, char* pnf_src, char* pnf_dst, char* reply)
{	
	int		argc = 0;
	char	argv[MAX_NUM_CMD_DMN][MAX_LEN_CMD_DMN+1];

	int		i = -1;

	ERR_NO	ret;

	// printf("cmd_src: %d, cmd_str: %s, pnf_src: %s, pnf_dst: %s\r\n", cmd_src, cmd_str, pnf_src, pnf_dst);

	// 将命令字符串分割成各个命令域。
	ret = destruct_command(cmd_str, (int*)&argc, (char (*)[MAX_LEN_CMD_DMN+1])argv);
	if(ret != OK)
	{
		// 命令格式错误！
		strcpy(reply, "\xE5\x91\xBD\xE4\xBB\xA4\xE6\xA0\xBC\xE5\xBC\x8F\xE9\x94\x99\xE8\xAF\xAF\xEF\xBC\x81");		

		return ERR_HANDLE_COMMAND_INVALID_COMMAND;
	}

	for(i = 0; i < argc; i++)
	{
		printf("argc = %d, argv[%d] = %s\r\n", argc, i, argv[i]);
	}	

	// 根据命令来源不同，采取不同方式匹配命令并采取不同方式返回命令处理结果。
	switch(cmd_src)
	{
		case CMD_SRC_SMS:
			printf("command from SMS.\r\n");
			
			// 先查找命令序号。
			i = seek_cmd_by_str(argv[0]);
				
			if(i >= 0)
			{	
				// 检查命令是否和操作号码相关，不相关的话需检查号码是否已被授权。
				if(i > MAX_INDEX_CMD_WOTHOUT_PWD)
				{
					// 先对命令发送号码鉴权。
					if(check_pn_operation(pnf_src) != OK)
					{
						//   未授权！
						sprintf(reply, "%s\xE6\x9C\xAA\xE6\x8E\x88\xE6\x9D\x83\xEF\xBC\x81", pnf_src);
						
						return NG;
					}
				}

				// 检查参数个数是否过该命令接受的最大参数个数。
				if(argc > (cmd_list[i].args_max+1) || argc < (cmd_list[i].args_min+1))
				{
					// 参数个数错误！
					strcpy(reply, info_incorrect_argc);
						
					return NG;
				}
				
				// 调用命令处理函数(命令处理函数本身会填充回复信息)。
				ret = cmd_list[i].func(cmd_src, argc, argv, pnf_src, (char*)reply);

				return ret;
			}
			else
			{
				// 命令不支持！
				sprintf(reply, "%s%s", argv[0], info_unsupported_command);
				
				return NG;
			}
		case CMD_SRC_COM:
			printf("command from COM.\r\n");
			
			// 查找命令序号。
			i = seek_cmd_by_str(argv[0]);

			// 对于串口发送的命令，无需检查号码是否授权也无法检查(无发送号码一说)。
				
			if(i >= 0)
			{
				// 检查参数个数是否过该命令接受的最大参数个数。
				if(argc > (cmd_list[i].args_max+1) || argc < (cmd_list[i].args_min+1))
				{
					// 参数个数错误！
					strcpy(reply, info_incorrect_argc);
						
					return NG;
				}
				
				// 调用命令处理函数(命令处理函数本身会填充回复信息)。
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
			
			// 查找命令序号。
			i = seek_cmd_by_dig(argv[0]);
	
			if(i >= 0)
			{
				printf("cmd = %s, dtmf = %s, args_min = %d, args_max = %d.\r\n", cmd_list[i].cmd, cmd_list[i].dtmf, cmd_list[i].args_min, cmd_list[i].args_max);
				
				// 同一命令，dmtf命令相比短信命令会在尾部多出一个反馈号码。
				if(argc > (cmd_list[i].args_max+2) || argc < (cmd_list[i].args_min+2))
				{				
					// 参数个数错误！
					strcpy(reply, info_incorrect_argc);
						
					return NG;
				}
				else
				{
					if(strlen(argv[argc-1]) == STD_LEN_PN)
					{
						// 将dtmf命令字符串末尾的回复号码提取出来并填充到dtmf命令请求的pnf_dst数组中。
						// strcpy(que_cmd[cnt_cmd_rd & (MAX_NUM_CMD_TXT-1)].pnf_dst, argv[argc-1]);
						strcpy(pnf_dst, argv[argc-1]);

						// 调用命令处理函数。
						ret = cmd_list[i].func(cmd_src, argc, argv, argv[argc-1], (char*)reply);
						
						return ret;						
					}
					else
					{
						// 请输入11位手机号码作为命令执行结果返回号码！
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

// 处理串口发来的命令(直接操作串口接收队列，无额外命令队列)。
int ana_buf_com(void)
{
	char	command[MAX_BYTE_SMS_PDU+1];

	int 	i = 0;
	
	unsigned char	byte = 0x00;

	unsigned int	to;
	
	// 检查是否有新的串口命令接收到(以"#\n"结尾的字符串)
	while(cnt_usart1_cmd > 0)
	{	
		printf("%d com command pending.", cnt_usart1_cmd);
		
		i = 0;

#if 1
		// 过滤掉令前面的不可打印字符。
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

		// 检查是否超时(正常应不会超时)
		if(systick >= to)
		{
			// 略过所有已检测到的命令
			cnt_usart1_cmd = 0;
			RxCnt1_rd = RxCnt1_wr;

			printf("receiving data time out.\r\n");
			
			return NG;
		}
#endif	

		// 增加对command游标i的检测，以免用户误将大文件当成命令发送时将MCU搞死
		while(RxCnt1_rd < RxCnt1_wr && i < MAX_BYTE_SMS_PDU)
		{						
			command[i++] = COM1_RX_RD(RxCnt1_rd++);

			// 检查最近保存的三个字符是否为"#\n"，是的话则说明遇到了命令结尾
			if(command[i-3] == '#' && command[i-2] == '\r' && command[i-1] == '\n')
			{
				// 添加字符串结尾符号
				command[i-2] = '\0';

				printf("\r\ncom command:%s", command);

				// 将串口命令加入到全局文本命令队列。
				que_cmd_wr(CMD_SRC_COM, command, "", "");
				
				// UART1接收且待处理的命令条数递减
				cnt_usart1_cmd--;
				
				break;
			}
		}
	}
	
	return OK;
}

// 处理来自短信的命令(只处理一条命令)。
/*
	1，支持text模式下和pdu模式下ucs2短信或ascci短信的解析和系统时间同步，不支持中文短信命令;
	2，支持短信命令容错处理和反馈: 发送中文短信命令、命令不以#号结尾;
*/
int ana_que_sms(void)
{
	char			msg[256];
	
	char			txtime1[32];				// 字符串形式的发送时间信息(包含年月日时分秒及时区)
	unsigned char	txtime2[7];					// 数值  形式的发送时间信息(包含年月日时分秒及时区)

	char			str[8];	

	char			pn[MAX_LEN_PN+1];					// 原始的gsm号码(可能包含或不包含国家区号前缀)

	char*			sms_cont_ptr = NULL;

	T_SMS_DELIVER	deliver;					// 保存PDU模式下接收到的PDU包解析内容

	int				i = 0;
	int				j = 0;
	int				k = 0;

	int 			len = 0;

	/************************************** 解析GSM模块输出的完整SMS消息 ****************************************/

	// 从短信命令缓冲中提取待处理短信
	strcpy(msg, (char*)&que_sms_cmd[cnt_sms_cmd_rd++ & (MAX_NUM_SMS_RX-1)]);

	printf("cnt_sms_cmd_rd = %d\r\n", cnt_sms_cmd_rd);

	printf("raw sms:%s\r\n", msg);

	// BOOL	ended = FALSE;				// 命令结尾字符#是否找到

	for(i = 0; i < 7; i++)
	{
		txtime2[i] = 0x00;
	}
	
	i = 0;

	// 定位到第一个逗号处
	while(msg[i++] != ',');	
	
	// 检查当前GSM模块所处的短信模式，pdu模式下只会输出pdu字符串，
	// 但是text模式下可能输出ascii字符串或ucs2编码的短信内容，因此需区别对待。
	if(msg[i] == '"')				// 处于text模式
	{			
		// 提取发送号码
		len = fetch_quotes(pn, msg+i);
		i += len;

		// 略过2个逗号，指向发送时间字符串的第一个双引号
		while(msg[i++] != ',');
		while(msg[i++] != ',');

		// 提取发送时间字符串(稍后解析)
		len = fetch_quotes(txtime1, msg+i);
		i += len;

		// 定位到短信内容的起始位置处
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

		// 记录短信内容的起始地址(短信内容固定为ascii编码的字符串)
		sms_cont_ptr = (char*)msg+i;	
	
		// 提取有间隔符的年、月、日
		for(i = 0, j = 0, k = 0; i < len && k < 3; )
		{
			str[j++] = txtime1[i++];

			// 遇到非数字字符则暂停提取
			if(txtime1[i] < '0' || txtime1[i] > '9')
			{
				i++;	// 略过非数字字符				
				
				str[j++] = '\0';					

				if(k == 0)	// 年
				{
					txtime2[k++] = (unsigned short)__atoi((char*)str)-2000;
				}
				else		// 月、日
				{
					txtime2[k++] = (short)__atoi((char*)str);
				}
	
				j = 0;				
			}
		}

		// 提取以冒号隔开的时、分、秒(基于V015版本固件)
		for( ; i < len && k < 6; )
		{			
			str[j++] = txtime1[i++];

			// 遇到非数字字符则暂停提取
			if(txtime1[i] < '0' || txtime1[i] > '9')
			{				
				i++;	// 略过非数字字符
				
				str[j++] = '\0';	
	
				txtime2[k++] = (unsigned char)__atoi((char*)str);		// 从前往后依次将年月日时分秒保存到接收时间数组中
	
				j = 0;				
			}
		}

		// 提取有正负号的时区
		txtime2[k++] = (unsigned char)(__atoi((char*)txtime1+i)%24);				// 短信发送时间中的时区值可能超过24			

		// printf("time mark of SMS: %04d-%02d-%02d %02d:%02d:%02d %2d time zone\n", txtime2[0], txtime2[1], txtime2[2], txtime2[3], txtime2[4], txtime2[5], txtime2[6]);

		// 检查输出的短信内容首字符是否为小写英文字母，不是的话多为pdu字符串(中文或英文)
		if(!(sms_cont_ptr[0] >= 'a' && sms_cont_ptr[0] <= 'z'))
		{
			// 将pdu字符串转换为二进制字节流
			len = pdu_atoh((unsigned char*)sms_cont_ptr, sms_cont_ptr, strlen(sms_cont_ptr));

			// 检查pdu字符串内容(ucs2格式编码)是否为英文
			if(sms_cont_ptr[0] == 0x00 && (sms_cont_ptr[1] >= 'a' && sms_cont_ptr[1] <= 'z'))
			{				
				len /= 2;

				// 将ucs2编码转换为ascii编码
				for(i = 0; i < len; i++)
				{
					*(sms_cont_ptr+i) = *(sms_cont_ptr+(i<<1)+1);
				}

				*(sms_cont_ptr+i) = '\0';
			}
			// else: 若pdu字符串内容为中文，则不予处理
		}		
	}
	else if(msg[1] == ',')			// 处于pdu模式
	{		
		// 定位到pdu包的起始地址处
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

		// 解码PDU包
		memset((unsigned char*)&deliver, sizeof(T_SMS_DELIVER), 0x00);

		// 将短信内容解码并获得解码后内容的字符个数
		len = pdu_deconstruct((T_SMS_DELIVER*)&deliver, sms_cont_ptr);

		// 保存发送号码
		strcpy(pn, deliver.tp_oas);	
		
		if(len > 0)
		{
			// 记录短信本体的起始地址(短信内容为ascii编码的字符串或ucs2编码的字节流)
			sms_cont_ptr = (char*)deliver.tp_uds;

			// 以ucs2编码的英文短信内容解码后需将其转化为ascii字符串方可解析
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
				// else: 以ucs2编码的中文短信内容解码后也不做解析
			}
			// else: 采用7bit或8bit编码的短信内容经pdu解码后会被转化成ascii字符串，可直接解析
		}

		/************* 提取发送时间 ************/		
		for(i = 0; i < 14; i += 2)
		{
			str[0] = deliver.tp_scts[i+0];
			str[1] = deliver.tp_scts[i+1];
			str[2] = '\0';

			if(i == 0)
			{
				// 提取年(原始数值仅保留4位年份的后2位)
				txtime2[i>>1] = __atoi(str);
			}
			else if(i < 12)
			{
				// 提取月、日、时、分、秒
				txtime2[i>>1] = __atoi(str);
			}
			else// i >= 12
			{
				// 提取时区
				txtime2[i>>1] = __atoi(str)%24;			// 短信发送时间信息中的时区值可能超过24
			}
		}	
	}
	else
	{
		return NG;
	}

	// 检查是否需要通过SMS发送时间更新系统时间
	if(sw_sync_by_sms == ON)
	{	
		// 检查之前是否通过GPS同步过系统时间，同步过的话不再通过SMS发送时间同步(精度不高)
		if(!(g_sync_point.synmode == SYNC_BY_GPS && g_sync_point.systick > 0))
		{
			// 通过SMS发送时间同步系统时间
			sync_systime(txtime2, SYNC_BY_SMS);

			// 保存SMS发送时间中的时区信息
			g_time_zone = txtime2[6];

			// 用SMS发送时间更新时间同步点后，即可关闭sw_synsms以免频繁同步系统时间
			sw_sync_by_sms = OFF;
		}
	}

	// 检查短信内容是否为纯ascii字符.
	if(is_all_ascii(sms_cont_ptr) == FALSE)
	{
		return NG;
	}

	// 格式化gsm号码。
	if(	format_pn(pn) == NG)
	{
		return NG;
	}

	// 将解析后的短信命令加入到全局文本命令队列。
	que_cmd_wr(CMD_SRC_SMS, sms_cont_ptr, pn, pn);

	return OK;
}

#ifdef USING_DEV_GSM
// 除了来自dtmf输入的命令(来电自动接通后用户先输入dtmf密码串，密码校验通过后用户再输入dtmf命令串，因此正常要处理两次dtmf串)。
void ana_str_dtmf(char* dtmf)
{
	char	at[32];

	// int		argc = 0;
	// char	argv[MAX_NUM_CMD_DMN][MAX_LEN_CMD_DMN+1];

	int		i;
	int		len = -1;

	// 在咪头静音打开后才开始检测dtmf命令校验密码
	if(sts_dtmf_command == STS_DTMF_COMMAND_MIC_MUTE_ON)
	{
		if(!__strcmp(dtmf, dtmf_password))
		{
			sts_dtmf_command = STS_DTMF_COMMAND_AUTHORIZED;

			// 停止dtmf密码输入超时检测定时器
			swtimer_input_dtmf = 0;

			// 密码正确输入后，发送dtmf回码(用户手机一般不会显示dtmf编码值，但是会听到dtmf按键音)以提示用户秘密输入正确，可输入dtmf命令
			strcpy(at, "AT+VTS=1\n");			
			usart3_send_poll(at, strlen(at));	

			// delay_100ms(1);

			// 密码正确输入后，发送dtmf回码(用户手机一般不会显示dtmf编码值，但是会听到dtmf按键音)以提示用户秘密输入正确，可输入dtmf命令
			strcpy(at, "AT+VTS=1\n");			
			usart3_send_poll(at, strlen(at));	

			// delay_100ms(1);

			// 启动关闭咪头静音延迟定时器
			printf("systick = %d, delay %d second for turning off mic mute.\r\n", systick, DELAY_SET_MICMUTE_OFF);
			
			swtimer_set_micmute_off = DELAY_SET_MICMUTE_OFF*1000/SYSTICK_PERIOD;
		}
		else
		{
			times_input_dtmf_password++;

			// 输入密码三次错误，终端自动挂断电话
			if(times_input_dtmf_password >= MAX_TIMES_INPUT_DTMF_COMMAND)
			{
				// 停止dtmf密码输入超时检测定时器
				swtimer_input_dtmf = 0;
					
				// 挂断电话
				strcpy(at, "ATH\n");
				usart3_send_poll(at, strlen(at));	

				// 重置gsm通话状态标志
				is_gsm_calling = FALSE;

				times_input_dtmf_password = 0;
				sts_dtmf_command = STS_DTMF_COMMAND_CALL_WAITING;
			}
		}
	}
	// 咪头静音关闭后开始检测命令字符串
	else if(sts_dtmf_command == STS_DTMF_COMMAND_MIC_MUTE_OFF)
	{
		// 返回嘀音并挂断来电
		// accept_dtmf_command();
		
		// 返回校验通过指示码(对方手机可能看不到字符，只能听到按键音)
		//////////////////////////////////////////////////////
		strcpy(at, "AT+VTS=1\n");
				
		usart3_send_poll(at, strlen(at));	

		// 挂断电话
		strcpy(at, "ATH\n");
		usart3_send_poll(at, strlen(at));	

		// 挂断电话后将dtmf命令状态重置
		sts_dtmf_command = STS_DTMF_COMMAND_CALL_WAITING;

		// 重置gsm通话状态标志
		is_gsm_calling = FALSE;

		times_input_dtmf_password = 0;
		//////////////////////////////////////////////////////
		
		printf("to detect dtmf command...\r\n");

		len = strlen(dtmf);

		// 将dtmf命令串中的星号替换成逗号，以兼容文本命令格式。
		for(i = 0; i < len; i++)
		{
			if(dtmf[i] == '*')
			{
				dtmf[i] = ',';
			}
		}

		printf("systick = %d, converted dtmf = %s\r\n", systick, dtmf);

		// 将格式转换后的dtmf命令加入到全局文本命令队列。
		que_cmd_wr(CMD_SRC_DTMF, dtmf, "", "");
	}	

	// DTMF检测及处理结束后，需等到RELEASE消息接收到后才能将RING中断标志复位!!!
}

// 处理来电振铃次数(命令)，注意需要SIM卡开通来电显示功能。
void ana_num_ring(int rings)
{
	// 检测到振铃消息且电话挂断后，根据振铃次数做相应的处理
	switch(rings)
	{
		case 1:	// 单次振铃后一般不能确保检测到来电号码，因此不适合需要作为需要检测来电号码的事件触发	
					
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

// 通过人为构造命令的方式发送命令，需指定明星来源、命令字符串(含#)、命令来源号码、命令目标号码、命令处理结果的接收缓存。
int send_command_virtual(unsigned char cmd_src, char* cmd_str, char* pnf_src, char* pnf_dst, char* reply)
{
	int		ret;

	ret = handle_cmd(cmd_src, cmd_str, pnf_src, pnf_dst, reply);
	
	return ret;
}

/********************************************** 串口命令队列处理函数 ****************************************************/

// 检查串口1是否接收到命令，有的话则处理。
void check_buf_com(void)
{
	// printf("enter <check_buf_com>.\r\n");
	
	transaction_enter();

	ana_buf_com();

	transaction_quit();
}

#ifdef USING_DEV_GSM

/********************************************** 短信命令队列处理函数 ****************************************************/

// 以指定的次数(一次性检查或一直检查)检查SMS接收队列，有新SMS的话则解析之。
// 注: 此函数用于解析GSM串口接收中断处理程序接收的SMS原始数据，准确的说是解析+CMT
// 消息，消息包含消息头和消息体两部分，消息头中的内容格式由终端设计厂商定义。SMS是
// 用户和终端交互的重要渠道，因此消息体中内容的类别也较多。
void check_que_sms(int check_mode)
{	
	// printf("enter <check_que_sms>.\r\n");
	
	if(check_mode == CHECK_MODE_CONTINUOUS)
	{
		// 处理完所有待处理的短信
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
		// 仅处理一待处理条短信
		if(cnt_sms_cmd_rd < cnt_sms_cmd_wr)
		{			
			// printf("%d sms command pending.\r\n", (cnt_sms_cmd_wr-cnt_sms_cmd_rd));;
			
			transaction_enter();
			
			ana_que_sms();

			transaction_quit();
		}
	}
}

// 检查指定号码是否在授权操作号码列表中。
ERR_NO check_pn_operation(char* pn)
{
	int 		i;

	int			len = strlen(pn);

	// 对于来自com或tcp的命令，无需鉴权
	if(len <= 0 || len == 1)
	{
		printf("no need to be authorized for command from com or TCP.\r\n");
		
		return OK;
	}	
	// 对于来自短信的命令，检查授权操作号码列表中是否存在该号码
	else
	{
		for(i = 0;i < MAX_PN_OPERATON; i++)
		{
			// 先检查操作号码是否在授权号码列表中
			if(!__strcmp(pn_operation[i], pn))
			{			
				printf("%s authorized.\r\n", pn);
				
				return OK;
			}		
		}

		printf("%s NOT authorized.\r\n", pn);

		return ERR_AUTH_NO_ENTRANCE;				// 未找到匹配号码
	}
}

/********************************************************************************************************************
												号码操作命令
*********************************************************************************************************************/
// 设置短信命令操作密码。
// szdxmm,old_password,new_password,new_password#
int set_sms_password(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	int len1, len2, len3;

	len1 = strlen(argv[1]);
	len2 = strlen(argv[2]);
	len3 = strlen(argv[3]);
	
	if(len1 != STD_LEN_OF_CMD_PWD || len2 != STD_LEN_OF_CMD_PWD || len3 != STD_LEN_OF_CMD_PWD)
	{
		// 密码长度固定为6字符！
		strcpy(reply, "\xE5\xAF\x86\xE7\xA0\x81\xE9\x95\xBF\xE5\xBA\xA6\xE5\x9B\xBA\xE5\xAE\x9A\xE4\xB8\xBA\x36\xE5\xAD\x97\xE7\xAC\xA6\xEF\xBC\x81");

		return NG;
	}

	if(len2 != len3)
	{
		// 新密码不一致！
		strcpy(reply, "\xE6\x96\xB0\xE5\xAF\x86\xE7\xA0\x81\xE4\xB8\x8D\xE4\xB8\x80\xE8\x87\xB4\xEF\xBC\x81");

		return NG;
	}

	if(strcmp(cmd_pwd_sms, argv[1]))
	{
		// 旧密码错误！
		strcpy(reply, "\xE6\x97\xA7\xE5\xAF\x86\xE7\xA0\x81\xE9\x94\x99\xE8\xAF\xAF\xEF\xBC\x81");

		return NG;
	}

	// 更新ram中的密码
	strcpy(cmd_pwd_sms, argv[2]);

	// 更新Flash中的参数
	bkp_str_set(ENV_CMD_PWD_SMS_START, ENV_CMD_PWD_SMS_SIZE, argv[2]);

	// 密码设置成功！
	strcpy(reply, "\xE5\xAF\x86\xE7\xA0\x81\xE8\xAE\xBE\xE7\xBD\xAE\xE6\x88\x90\xE5\x8A\x9F\xEF\xBC\x81");

	return OK;
}

// 强行重置设备操作密码为默认初始密码。
// czczmm,imei#
int rev_sms_password(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	if(strcmp(gsm_imei, argv[1]))
	{
		// IMEI错误！
		strcpy(reply, "\x49\x4D\x45\x49\xE9\x94\x99\xE8\xAF\xAF\xEF\xBC\x81");

		return NG;
	}

	strcpy(cmd_pwd_sms, DEF_CMD_PWD_SMS);

	bkp_str_set(ENV_CMD_PWD_SMS_START, ENV_CMD_PWD_SMS_SIZE, DEF_CMD_PWD_SMS);

	// 密码重置成功！
	strcpy(reply, "\xE5\xAF\x86\xE7\xA0\x81\xE9\x87\x8D\xE7\xBD\xAE\xE6\x88\x90\xE5\x8A\x9F\xEF\xBC\x81");

	return OK;
}

// 设置dtmf命令操作密码。
// szszmm,cmd_pwd_sms,new_password,new_password#
int set_dtmf_password(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	int len1, len2, len3;

	len1 = strlen(argv[1]);
	len2 = strlen(argv[2]);
	len3 = strlen(argv[3]);
	
	if(len1 != STD_LEN_OF_CMD_PWD || len2 != STD_LEN_OF_CMD_PWD || len3 != STD_LEN_OF_CMD_PWD)
	{
		// 密码长度固定为6字符！
		strcpy(reply, "\xE5\xAF\x86\xE7\xA0\x81\xE9\x95\xBF\xE5\xBA\xA6\xE5\x9B\xBA\xE5\xAE\x9A\xE4\xB8\xBA\x36\xE5\xAD\x97\xE7\xAC\xA6\xEF\xBC\x81");

		return NG;
	}

	if(len2 != len3)
	{
		// 新密码不一致！
		strcpy(reply, "\xE6\x96\xB0\xE5\xAF\x86\xE7\xA0\x81\xE4\xB8\x8D\xE4\xB8\x80\xE8\x87\xB4\xEF\xBC\x81");

		return NG;
	}

	if(strcmp(cmd_pwd_sms, argv[1]))
	{
		// 短信命令操作密码错误！
		strcpy(reply, "\xE7\x9F\xAD\xE4\xBF\xA1\xE5\x91\xBD\xE4\xBB\xA4\xE6\x93\x8D\xE4\xBD\x9C\xE5\xAF\x86\xE7\xA0\x81\xE9\x94\x99\xE8\xAF\xAF\xEF\xBC\x81");

		return NG;
	}

	// 更新ram中的密码
	strcpy(cmd_pwd_dtmf, argv[2]);

	// 更新Flash中的参数
	bkp_str_set(ENV_CMD_PWD_DTMF_START, ENV_CMD_PWD_DTMF_SIZE, argv[2]);

	// 密码设置成功！
	strcpy(reply, "\xE5\xAF\x86\xE7\xA0\x81\xE8\xAE\xBE\xE7\xBD\xAE\xE6\x88\x90\xE5\x8A\x9F\xEF\xBC\x81");

	return OK;
}

// 将指定的号码添加到操作号码列表。
// tjczhm,pn,cmd_pwd_sms#
int add_pn_operation(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	int 	i;

	// 检查密码。
	if(strcmp(cmd_pwd_sms, argv[2]))
	{
		// 密码错误！
		strcpy(reply, "\xE5\xAF\x86\xE7\xA0\x81\xE9\x94\x99\xE8\xAF\xAF\xEF\xBC\x81");

		return NG;
	}
	
	// 将待添加号码格式化。
	if(format_pn(argv[1]) != OK)
	{
		// 号码非法！
		strcpy(reply, "\xE5\x8F\xB7\xE7\xA0\x81\xE9\x9D\x9E\xE6\xB3\x95\xEF\xBC\x81");
		
		return NG;
	}	

	// 检查待授权号码是否已存在
	for(i = 0; i < MAX_PN_OPERATON; i++)
	{
		if(!__strcmp(pn_operation[i], argv[1]))
		{
			// 已授权！
			sprintf(reply, "%s\xE5\xB7\xB2\xE6\x8E\x88\xE6\x9D\x83\xEF\xBC\x81", argv[1]);

			return OK;
		}
	}

	// 检查授权号码列表是否有空入口项
	for(i = 0; i < MAX_PN_OPERATON; i++)
	{
		if(strlen(pn_operation[i]) == 0)
		{
			strcpy(pn_operation[i], argv[1]);
			
			bkp_str_set(ENV_PN_OPERATION_START+i*ENV_PN_OPERATION_SIZE, ENV_PN_OPERATION_SIZE, (char*)argv[1]);

			// 被授权！
			sprintf(reply, "%s\xE8\xA2\xAB\xE6\x8E\x88\xE6\x9D\x83\xEF\xBC\x81", argv[1]);

			return OK;
		}
	}

	// 授权号码满！
	sprintf(reply, "\xE6\x8E\x88\xE6\x9D\x83\xE5\x8F\xB7\xE7\xA0\x81\xE6\xBB\xA1\xEF\xBC\x81");

	return NG;
}

// 从操作号码列表中删除指定号码。
// scczhm,pn,cmd_pwd_sms#
int del_pn_operation(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	int 	i;

	// 检查密码。
	if(strcmp(cmd_pwd_sms, argv[2]))
	{
		// 密码错误！
		strcpy(reply, "\xE5\xAF\x86\xE7\xA0\x81\xE9\x94\x99\xE8\xAF\xAF\xEF\xBC\x81");

		return NG;
	}
	
	// 将待添加号码格式化。
	if(format_pn(argv[1]) != OK)
	{
		// 号码非法！
		strcpy(reply, "\xE5\x8F\xB7\xE7\xA0\x81\xE9\x9D\x9E\xE6\xB3\x95\xEF\xBC\x81");
		
		return NG;
	}	

	for(i = 0; i < MAX_PN_OPERATON; i++)
	{
		if(!strcmp(pn_operation[i], argv[1]))
		{
			strcpy(pn_operation[i], "");

			bkp_str_set(ENV_PN_OPERATION_START+i*ENV_PN_OPERATION_SIZE, ENV_PN_OPERATION_SIZE, "");

			// 被删除！
			sprintf(reply, "%s\xE8\xA2\xAB\xE5\x88\xA0\xE9\x99\xA4\xEF\xBC\x81", argv[1]);

			return OK;
		}
	}

	// 未找到！
	sprintf(reply, "%s\xE6\x9C\xAA\xE6\x89\xBE\xE5\x88\xB0\xEF\xBC\x81", argv[1]);

	return OK;
}

// 清除操作号码列表(不清除超级用户)。
// qkczhm,cmd_pwd_sms#
int clr_pn_operation(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	int 	i;

	// 检查密码。
	if(strcmp(cmd_pwd_sms, argv[1]))
	{
		// 密码错误！
		strcpy(reply, "\xE5\xAF\x86\xE7\xA0\x81\xE9\x94\x99\xE8\xAF\xAF\xEF\xBC\x81");

		return NG;
	}

	for(i = 0; i < MAX_PN_OPERATON; i++)
	{
		strcpy(pn_operation[i], "");

		bkp_str_set(ENV_PN_OPERATION_START+i*ENV_PN_OPERATION_SIZE, ENV_PN_OPERATION_SIZE, "");		
	}
	
	// 授权号码被清空！
	sprintf(reply, "\xE6\x8E\x88\xE6\x9D\x83\xE5\x8F\xB7\xE7\xA0\x81\xE8\xA2\xAB\xE6\xB8\x85\xE7\xA9\xBA\xEF\xBC\x81");		
	
	return OK;
}

// 查询普通授权号码列表。
// cxczhm,cmd_pwd_sms#
int inq_pn_operation(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	int 	i;
	char	pn_exist[(MAX_LEN_PN+1)*MAX_PN_OPERATON];
	int 	len  = 0;

	// 检查密码。
	if(strcmp(cmd_pwd_sms, argv[1]))
	{
		// 密码错误！
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

	// 授权号码：   。 
	sprintf(reply, "\xE6\x8E\x88\xE6\x9D\x83\xE5\x8F\xB7\xE7\xA0\x81\xEF\xBC\x9A%s\xE3\x80\x82", pn_exist);

	return OK;
}
#endif

/********************************************************************************************************************
												GPS操作命令
*********************************************************************************************************************/
#ifdef USING_DEV_GPS

// 根据解析结果调用相应的命令处理函数
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

// 2014-04-12 周期定位模式开关命令。
int swt_gps_ppos(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{	
	if(!__strcmp(argv[1], "1"))
	{
		sw_periodic_pos = ON;

		// 启动定时定位timer
		swtimer_periodic_pos = cycle_periodic_pos*1000/SYSTICK_PERIOD;

		strcpy(reply, "periodic pos mode set to ON.");	
	}
	else if(!__strcmp(argv[1], "0"))
	{
		sw_periodic_pos = OFF;

		// 接收到关闭周期定位命令后，立即忽略掉尚未处理的周期定位请求。
		sem_periodic_pos = 0;

		strcpy(reply, "periodic pos mode set to OFF.");	
	}
	
	return OK;
}

#endif

#ifdef USING_DEV_GSM
/********************************************************************************************************************
												GSM/GPRS操作命令
*********************************************************************************************************************/

// 进入/退出GSM调试模式(GSM串口转发)。
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

// 设置GPRS APN。
int set_gsm_apn(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{	
	// 更新RAM中的参数
	strcpy(gsm_apn, argv[1]);

	// 更新Flash中的参数
	bkp_str_set(ENV_GPRS_APN_START, ENV_GPRS_APN_SIZE, argv[1]);

	// GPRS APN设为：
	sprintf(reply, "\x47\x50\x52\x53\x20\x41\x50\x4E\xE8\xAE\xBE\xE4\xB8\xBA\xEF\xBC\x9A%s\xE3\x80\x82", argv[1]);
	
	return OK;
}

int set_gsm_telecode(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	// 更新RAM中的参数
	strcpy(gsm_telecode, argv[1]);

	// 更新Flash中的参数
	bkp_str_set(ENV_GSM_TELECODE_START, ENV_GSM_TELECODE_SIZE, argv[1]);

	// 国家区号设为：  。
	sprintf(reply, "\xE5\x9B\xBD\xE5\xAE\xB6\xE5\x8C\xBA\xE5\x8F\xB7\xE8\xAE\xBE\xE4\xB8\xBA\xEF\xBC\x9A%s\xE3\x80\x82", argv[1]);

	return OK;
}

// 查询GSM模块的IMEI。
int inq_gsm_imei(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	gsm_wakeup();

	if(strlen(gsm_imei) <= 0)
	{
		// 查询IMEI失败。
		strcpy(reply, "\xE6\x9F\xA5\xE8\xAF\xA2\x49\x4D\x45\x49\xE5\xA4\xB1\xE8\xB4\xA5\xE3\x80\x82");

		gsm_sleep();
		
		return NG;
	}

	gsm_sleep();

	// GSM IMEI：   。
	sprintf(reply, "\x47\x53\x4D\x20\x49\x4D\x45\x49\xEF\xBC\x9A%s\xE3\x80\x82", gsm_imei);

	return OK;
}

// 查询GSM模块软件版本号。
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

	// 反馈操作结果
	sprintf(reply, "GSM SW version is: %s.", version);

	return OK;
}

// 查询GSM模块的网络信号强度。
int inq_gsm_signal(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	int		rssi;

	gsm_wakeup();

	if(gsm_get_rssi((int*)&rssi) != OK)
	{
		// 查询RSSI失败。
		strcpy(reply, "\xE6\x9F\xA5\xE8\xAF\xA2\x52\x53\x53\x49\xE5\xA4\xB1\xE8\xB4\xA5\xE3\x80\x82");	

		gsm_sleep();
		
		return NG;
	}	

	gsm_sleep();

	// GSM信号强度=   。
	sprintf(reply, "\x47\x53\x4D\xE4\xBF\xA1\xE5\x8F\xB7\xE5\xBC\xBA\xE5\xBA\xA6\x3D%d\xE3\x80\x82", rssi);
	
	return OK;
}

// 查询常用GPRS连接的状态。
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
												设备操作命令
*********************************************************************************************************************/

// 将终端恢复出厂设置。
int set_dev_factory(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	// 清除魔术数，以便重启后能按照默认设置初始化环境变量
	bkp_u16_set(ENV_SYS_FIRST_RUNNING_START, 0xFFFF);

	printf("to reset device.\r\n");

	if(cmd_src == CMD_SRC_SMS || cmd_src == CMD_SRC_DTMF)
	{
		gsm_wakeup();

		// 恢复出厂设置，准备重启设备。。。
		gsm_send_sms(pn, "\xE6\x81\xA2\xE5\xA4\x8D\xE5\x87\xBA\xE5\x8E\x82\xE8\xAE\xBE\xE7\xBD\xAE\xEF\xBC\x8C\xE5\x87\x86\xE5\xA4\x87\xE9\x87\x8D\xE5\x90\xAF\xE8\xAE\xBE\xE5\xA4\x87\xE3\x80\x82\xE3\x80\x82\xE3\x80\x82");
	}

	// 软件复位mcu
	mcu_reset();

	return OK;
}

int set_dev_shutdown(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{	
	printf("to shutdown system...");

	if(cmd_src == CMD_SRC_SMS || cmd_src == CMD_SRC_DTMF)
	{
		gsm_wakeup();

		// 设备将关机。
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

		// 设备将重启。
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
	"\xE7\xBA\xA2\xE8\x89\xB2\0",		// R: 红色
	"\xE7\xBB\xBF\xE8\x89\xB2\0",		// G: 绿色
	"\xE8\x93\x9D\xE8\x89\xB2\0"		// B: 蓝色
};

const char reply_sw_dev_led2[2][10]=
{
	"\xE7\x86\x84\xE7\x81\xAD\xE3\x80\x82\0",		// 熄灭。
	"\xE7\x82\xB9\xE4\xBA\xAE\xE3\x80\x82\0",		// 点亮。
};

// 打开/关闭指定颜色的LED灯。
int swt_dev_led(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	int		led_no;	// 0 / 1 / 2, R / G / B
	SWITCH	sw;		// 0 / 1, OFF /  ON

	led_no 	= atoi(argv[1]);
	sw 		= atoi(argv[2]);
	
	// 更新RAM中的参数
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
		// 参数非法！
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

	// 硬件版本：  。
	sprintf(reply, "\xE7\xA1\xAC\xE4\xBB\xB6\xE7\x89\x88\xE6\x9C\xAC\xEF\xBC\x9A%s\xE3\x80\x82", version);

	return OK;
}

int inq_dev_swversion(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	char	version[16];

	bkp_str_get(ENV_VERSION_SOFTWARE_START, ENV_VERSION_SOFTWARE_SIZE, version);

	// 软件版本：  。
	sprintf(reply, "\xE8\xBD\xAF\xE4\xBB\xB6\xE7\x89\x88\xE6\x9C\xAC\xEF\xBC\x9A%s\xE3\x80\x82", version);

	return OK;
}

// 查询电池电压。
int inq_dev_batvol(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	// 电池电压：   。
	sprintf(reply, "\xE7\x94\xB5\xE6\xB1\xA0\xE7\x94\xB5\xE5\x8E\x8B\xEF\xBC\x9A%1.2f V\xE3\x80\x82", batvol_measure()/(float)100);

	return OK;
}

// 查询设备供电状况。
int inq_dev_powersupply(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	int		len = 0;

	strcpy(reply, "");

#ifdef USING_PWR_ACC
	// 更新acc电源状态
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
	// 更新ext电源状态
	if(MCU_GPIO_READ(GPIO_EXTPWR_DET) == 1)
	{
		sts_power |= (1<<BIT_STS_POWER_EXT);
	}
	else
	{
		sts_power &= ~(1<<BIT_STS_POWER_EXT);
	}
#endif

	// 测量电池电压
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
		// 电池电压：  V。
		len += sprintf(reply+len, "\xE7\x94\xB5\xE6\xB1\xA0\xE7\x94\xB5\xE5\x8E\x8B\xEF\xBC\x9A%1.2f V\xE3\x80\x82", de_batvol/(double)100);

		sts_power |= (1<<BIT_STS_POWER_BAT);
	}
	else
	{
		// 电池可能未接。
		len += sprintf(reply+len, "\xE7\x94\xB5\xE6\xB1\xA0\xE5\x8F\xAF\xE8\x83\xBD\xE6\x9C\xAA\xE6\x8E\xA5\xE3\x80\x82");

		sts_power &= ~(1<<BIT_STS_POWER_BAT);
	}	
#endif
	
	return OK;
}

// 查询设备的当前时间(仅在设备时间与自然时间同步后才会返回时间信息)。
int inq_dev_time(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	unsigned int	seconds;

	T_TIME_DHMS		dhms;

	unsigned char	systime[6];
	
	int				len = 0;

	// 计算系统运行的持续时间(自MCU上电复位开始计时)
	seconds		= systick*SYSTICK_PERIOD/1000;

	// 将秒数转换为天、小时、分、秒的格式
	second_to_dhms(seconds, (T_TIME_DHMS*)&dhms);

	// 运行时间：  天   小时  分  秒；
	len += sprintf(reply+len, "\xE8\xBF\x90\xE8\xA1\x8C\xE6\x97\xB6\xE9\x97\xB4\xEF\xBC\x9A%02d\xE5\xA4\xA9 %02d\xE5\xB0\x8F\xE6\x97\xB6 %02d\xE5\x88\x86\xE9\x92\x9F %02d\xE7\xA7\x92\xEF\xBC\x9B", dhms.day, dhms.hour, dhms.minute, dhms.second);

	// 计算系统维护的绝对时间(仅在与自然时间同步后才有意义)
	if(g_sync_point.synmode == SYNC_BY_NULL)
	{
		// 系统时间：未同步。
		len += sprintf(reply+len, "\xE7\xB3\xBB\xE7\xBB\x9F\xE6\x97\xB6\xE9\x97\xB4\xEF\xBC\x9A\xE6\x9C\xAA\xE5\x90\x8C\xE6\xAD\xA5\xE3\x80\x82");
	}
	else
	{		
		tick_to_nattime(systick, systime);

		// 系统时间：  。
		len += sprintf(reply+len, "\xE7\xB3\xBB\xE7\xBB\x9F\xE6\x97\xB6\xE9\x97\xB4\xEF\xBC\x9A%04d-%02d-%02d %02d:%02d:%02d\xE3\x80\x82", 2000+systime[0], systime[1], systime[2], systime[3], systime[4], systime[5]);	
	}

	return OK;
}


/********************************************************************************************************************
												程序下载命令
*********************************************************************************************************************/
#ifdef USING_FUNC_UPGRADE_COM

// 通过串口下载程序。
// 注: 由于备份程序区域和重发数据共享同一存储空间，而主循环中检查有无数据重发时会读取重发区域中保存的读写游标数值，
// 因此为了不使的读取重发数据读写游标出错而造成死循环，应在接收出错后将备份程序空间擦除并初始化重发相关变量。
// 
// 格式: upd-img-com,程序文件字节数,程序文件CRC校验码#
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

	// 获取bin文件长度和校验值信息
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

	// 将系统主频提升到8/2*16=64MHz,以便加快接收文件时写Flash的速度
	rcc_hsi_64mhz();

	// 关闭运动和静止检测，以免接收文件时干扰串口接收
	detect_motion(OFF);
	detect_still(OFF);

	printf("system main MCU_CLOCK_MODE is raised to 64MHz.\r\n");
	
	printf("please wait until image area is completely erased!!!\r\n");	
	
	// 下载最新程序前清除Flash中的程序更新标志
	bkp_u16_set(ENV_APP_UPDATED_START, 0x00);
	
	// 事先擦除整个备份程序区域
	for(i = 0; i < BLOCK_APPLICATION_SPACE; i+= FLASH_PAGE_SIZE)
	{
		FLASH_ErasePage(BLOCK_APPLICATION_START2+i);

		// 擦除备份程序区域时闪烁红灯指示。
		if(!((i/FLASH_PAGE_SIZE)&1))			// 擦除第一个Flash块时点亮红灯、擦除第二个Flash块时熄灭红灯，如此循环。
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
	
	// 15秒内用户不下载文件则放弃下载
	to = systick+15*1000/SYSTICK_PERIOD;	
	
	while((systick < to))
	{
		if(RxCnt1_rd < RxCnt1_wr)
		{
			break;
		}
	
		// 加入喂狗操作以免擦除Flash时触发看门狗溢出而重启
		WATCHDOG_RELOAD();
	}
	
	if(systick >= to)
	{
		printf("updating image file aborted as 15s past.\r\n");
		
		goto ERROR_RECEIVE_COM;
	}
	
	to = systick;

	// 从串口1接收缓冲中逐个拷贝文件数据
	while(j < flen)
	{
		// 若相邻两个字节的接收时间超过1秒，则认为发送已经结束
		if((systick - to) >= 1000/SYSTICK_PERIOD)
		{
				printf("received %d data is less than %d bytes.\r\n", j-1, flen);
			
				goto ERROR_RECEIVE_COM;
		}

		if(RxCnt1_rd < RxCnt1_wr)
		{
			// 接收每个字节时记录当前的rtcalarn数值
			to = systick;
			
			byte = COM1_RX_RD(RxCnt1_rd++);

			// 计算CRC值(基于转换后的字节)
			crc16 = (crc16<<8) ^ crc16tab[((crc16>>8) ^ byte)&0x00FF];

			// 解密处理: 最高2位和最低2位对调、中间4位以2位为单位对调
			byte = ((byte&0xC0)>>6) | ((byte&0x0F)<<6) | ((byte&0x30)>>2) | ((byte&0x0C)<<2);
			
			frame[j++ & (FLASH_PAGE_SIZE-1)] = byte;

			// 每接收到1024字节就写入flash中一页(末尾数据可能不满1024字节)
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
				// 准备写入剩余字节(不足一页)
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

	// 检查计算得到的CRC是否和文件发送时携带的CRC相等
	if(crc16 != fcrc)
	{
		printf("error when checking CRC: fcrc = 0x%04x but crc = 0x%04x.\r\n", fcrc, crc16);	

		goto ERROR_RECEIVE_COM;
	}

	// 设置程序被更新的标志
	bkp_u16_set(ENV_APP_UPDATED_START, 0x01);

	printf("\nsuccessfully receive image file with %d bytes.\r\n", j);

	rcc_hsi_8mhz();	

	// 重新打开运动和静止检测
	detect_motion(ON);
	detect_still(ON);
	return OK;

ERROR_RECEIVE_COM:
	
	// 如果接收出错，则将备份程序区域完全擦除(和重发数据共享同一存储空间)，以免主循环中检查重发数据游标时读取数值出错。
	for(i = 0; i < BLOCK_APPLICATION_SPACE; i+= FLASH_PAGE_SIZE)
	{
		FLASH_ErasePage(BLOCK_APPLICATION_START2+i);
	}

	// 接收COM数据失败后应将系统主时钟切换回HSI 8MHz，以免MCU工作影响GPS收星
	rcc_hsi_8mhz();	

	// 重新打开运动和静止检测
	detect_motion(ON);
	detect_still(ON);

	return NG;
}

/*
	通过串口下载程序的过程:
	----------------------
	1，在keil中编译生成名称为solidcom.bin的原始程序文件；
	2，利用gps_bin_handler.exe工具将solidcom.bin文件转换为加密的程序文件，加密后的文件名称形如upd-img-com,59660,0x7dab#；
	3，在串口调试助手中发送命令: upd-img-com,59660,0x7dab#，其中59660为程序文件字节数、0x7dab微程序文件的CRC(CRC16)校验值；
	4，在串口调试助手中发送命令发送以命令字符串(不包括#)命名的程序文件，如d:\upd-img-com,59660,0x7dab#；
	5，终端软件开始擦除flash中的备份程序区域并逐页接收程序文件数据并逐页写入flash；
	6，终端软件接收完程序文件后比较实际接收到的字节数和需要接收的字节数是否相等，此外再比较实际接收到数据的CRC校验值和原始CRC校验值是否相等，
	   若都相等，则终端软件自动执行软件复位，复位后执行新下载的程序，否则当前程序更新过程失败，终端软件恢复更新程序前的状态；
*/
int upd_img_com(unsigned char cmd_src, int argc, char (*argv)[MAX_LEN_CMD_DMN+1], char* pn, char* reply)
{
	// 关闭USART3 RX中断，以免干扰串口下载程序
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
	
		// 重启程序
		mcu_reset();
		
		return OK;
	}
	else
	{
		// 恢复SUART3 RX中断
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); 

		printf("failed to receive image.");
		
		return NG;	
	}
}

#endif


