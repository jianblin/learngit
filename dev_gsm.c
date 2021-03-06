/*GSM硬件管理及消息处理*/
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

T_GPRS_CONNECTION	tcp_conn[MAX_NUM_TCP_CONN];				// 设备要访问的tcp服务器参数数组

char	 				gsm_telecode[MAX_LEN_GSM_TELECODE+1];		// 发送PDU短信时需要电话号码的国家/地区区号(如中国为86，+非必需)
char					gsm_sca[MAX_LEN_PN+1];						// 短信中心
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
char 		de_gsm_cell[MAX_NUM_GSM_CELL][MAX_NUM_GSM_CELL_DMN][MAX_LEN_GSM_CELL_DMN+1];		// 用于保存GSM基站信息的全局数组(最多7个基站、每个基站固定7个域、每个域的长度不超过6字节)
char			de_gsm_pnin[MAX_LEN_PN+1];				// 当前检测到的来电号码信息
char			de_gsm_dtmf[MAX_LEN_GSM_DTMF+1];		// 当前检测到的DTMF信息

char			gsm_imei[STD_LEN_GSM_IMEI+1];				// GSM模块的IMEI串

STATUS		sts_gsm_power = OFF;

BOOL		is_gsm_ready  = FALSE;					// GSM模块是否正常初始化的标志
BOOL		is_gsm_calling = FALSE;					// gsm是否处于通话状态(通话状态下不能建立tcp连接，也不能发送tcp数据)

BOOL		is_gsmring_pending;						// gsm模块的ring中断事件是否待处理
int			cnt_gsmring_asserted;					// GSM模块RING中断产生的次数(GSM模块上电后会自动产生一个RING中断)

int			cnt_gsm_recovered;						// GSM模块故障恢复的次数

// dtmf related
int			swtimer_input_dtmf;						// dtmf检测打开后接收用户输入dtmf密码的超时检测定时器
int			swtimer_set_micmute_off;				// 用户输入dtmf密码正确后关闭咪头静音的延迟定时器
BOOL		is_dtmf_detection_enabled;				// 当前DTMF检测是否打开
STATUS		sts_dtmf_command;						// DTMF命令处理的当前状态
char			dtmf_password[MAX_LEN_GSM_DTMF+1];		// DTMF交互密码
int			times_input_dtmf_password;				// 已经输入DTMF交互密码的次数

// 初始化GSM模块(硬件上电、工作参数等初始化)。
// 注: 如果完全断开GSM天线，GSM能正常初始化但rssi值为99，初始化完成后再查询，rssi为2-3，
//     在此情况下，给终端发送短信，发送方收到短信未到达的回复。
ERR_NO gsm_init(void)
{		
	char		str[64+1];

	int		rssi = -1;	

	int  ret;
	printf("to initialize GSM module.\r\n");

	// 初始化gsm硬件前将is_gsm_ready设置为FALSE
	is_gsm_ready 			= FALSE;

	// 初始化gsm硬件前将is_gsm_calling设置为FALSE
	is_gsm_calling 			= FALSE;

	is_gsmring_pending 		= FALSE;	
	cnt_gsmring_asserted 	= 0;	

	// 初始化dtmf命令相关变量
	sts_dtmf_command 		= STS_DTMF_COMMAND_CALL_WAITING;
	times_input_dtmf_password 	= 0;
	
	// 初始化GSM串口收发相关变量和缓冲
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

	// 先强行关断GSM电源，以便在系统异常重启而未能在关机前关断GSM电源时清除GSM工作状态
//	gsm_power_down();

	// 打开GSM电源
//	gsm_power_up();	

	// 给GSM模块上电
//	gsm_onoff();  	

	// 延时3秒等待GSM硬件初始化
	delay_100ms(30);

	// 等待GSM模块搜网、注册
	printf("to detect SIM card...\r\n");	
	

//	ret=gsm_send_at("AT\r\n", "OK", 2);
//	 if(ret != OK)
//	 	printf("AT ERROR.......%d\r\n",ret);
//
//	//ATE0关闭回显
//	ret=gsm_send_at("ATE0\r\n", "OK", 8);
//	 if(ret != OK)
//	 	printf("ATE0 ERROR.......%d\r\n",ret);
	// 检查SIM卡是否插上
	if(gsm_check_sim() != OK)
	{
		printf("SIM card not detected.\r\n");

//		gsm_exit();
		
		return ER_GSM_INIT_SIM;		// sim卡未检测到
	}

	// 等待GSM模块搜网、注册
	printf("to wait for GSM registration...\r\n");	
	
	ret=gsm_send_at("ATE0\r\n", "OK", 8);
	 if(ret != OK)
	 	printf("ATE0 ERROR.......%d\r\n",ret);
	// 不管SIM卡是否插入，模块上电后都会输出+EIND特征字符串，
	// 以此检测模块是否基本正常(M660+模块的注册网络时间一般固定为10秒)。	
	/*
	if(gsm_find_pattern("+EIND: 1\r\n", TO_GSM_REGISTRATION) != OK)
	{	
		printf("+EIND: 1 not found.\r\n");

		gsm_exit();		
		
		return ER_GSM_INIT_REGISTRATION;		// 网络注册异常
	}	
	*/

#if 1
	// 查询GSM固件版本	
	if(gsm_get_swversion((char*)str, 64) == OK)
	{
		printf("GSM firmware version: %s.\r\n", str);
	}
	else
	{
		printf("failed to get GSM firmware version.\r\n");

//		gsm_exit();		
		
		return ER_GSM_INIT_SWVERSION;			// 查询GSM固件版本异常
	}
#endif	

#if 1
	// 查询IMEI串(GSM模块刚上电后查询IMEI经常失败，原因不详)
	if(gsm_get_imei((char*)gsm_imei, STD_LEN_GSM_IMEI) != OK)
	{
		printf("failed to get GSM IMEI.\r\n");	

//		gsm_exit();

		return ER_GSM_INIT_IMEI1;
	}
	else
	{
		// 检查imei长度是否为15位
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
// SMS环境初始化
	// 将GSM模块的短消息模式设置为文本模式
	if(gsm_sms_mode_txt()!= OK)
	{
		gsm_exit();

		printf("failed to set SMS mode to text.\r\n");
		
		return ER_GSM_INIT_SMS_MODE;
	}
#endif
	// 短消息输出模式采用默认设置(以便GSM模块接收到短信时能输出+SMSFLAG提示消息)，采用其他设置反而不能正常产生ring中断
#if 0	
	if(gsm_send_at("AT+CNMI=1,1,0,0,0\r\n", "OK", 2) != OK)		// 禁止接收到的短信自动输出，以使上层应用在输出短信前低RING引脚
	{
		printf("failed to set CNMI.\r\n");

		gsm_exit();
		
		return ER_GSM_INIT_SMS_SETTING;
	}
	// printf("enable outputing received SMS automatically.\r\n");
#endif	
#if 0
	// 删除当前短信存储器中的所有短信，以免短信接收异常
	if(gsm_send_at("AT+CMGD=\"DEL ALL\"\r\n", "OK", 5) != OK)
	{
		gsm_exit();

		printf("failed to delete all sms in current memory.\r\n");
		
		return ER_GSM_INIT_SMS_DELETE;
	}

// Call环境初始化
	// 使能来电号码自动输出
	if(gsm_send_at("at+clip=1\r\n", "OK", 2) != OK)
	{
		gsm_exit();

		printf("failed to enable phone number of incoming call output automatically.\r\n");
		
		return ER_GSM_INIT_CLIP;
	}
#endif
// GPRS环境初始化
	// 设置数据收发都为ASCII模式(即传统的二进制模式)
	/*
	if(gsm_send_at("AT+DATAFORMAT=1,1\r\n", "OK", 2) != OK)
	{
		gsm_exit();

		printf("failed to set data mode to ASCII.\r\n");
		
		return ER_GSM_INIT_DATAFORMAT;
	}
	*/
	// 查询RSSI
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

	// 初始化gsm成功后将is_gsm_ready设置为TRUE
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

// 按正常流程关断GSM模块。
void gsm_exit(void)
{
	gsm_onoff();
				
	gsm_power_down();
}

/**********************************************************************************************************************
										 		GSM硬件IO操作函数
***********************************************************************************************************************/

// 重启GSM模块(拉低SYSRST脚50ms以上)。
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
>> SIM卡插入状态下，M660+上电后会自动输出如下消息:

+EIND: 128

+EUSIM: 0

+STKPCI: 0,"D081B6810301250082028182850B80795E5DDE884C592957308F0A01808F7B677E95EE50198F0A038077ED4FE17FA453D18F0A0480670065B063A883508F0A06804E1A52A17CBE90098F10078065E07EBF97F34E504FF14E5090E88F0E08800031003300394E9280547F518F0809808D224FE1901A8F0E0A8079FB52A84F1860E04E13533A8F0E0B8079FB52A875355B50554652A18F120C806211768400530049004D84254E1A53858F0E058000530049004D53614FE1606F"

+EIND: 2		// 从输出+STKPCI到输出+EIND: 2之间可能间隔长达3秒

+EIND: 1		// 从输出+EIND: 2到输出+EIND: 1之间可能间隔长达5秒(时间长度取决于SIM卡的类型和相应的成功注册到网络的时间)

>> SIM卡未插状态下，M660+上电后会自动输出如下消息:

+EIND: 128

+EIND: 2

+EIND: 1		// 初始化完成后自动输出的最后一条消息

*/
// 给GSM模块上电/下电(拉低/拉高POWERON引脚3s以上)。
void gsm_onoff(void)
{	
	//MCU_GPIO_LOW(GPIO_GSM_ONOFF);
	
	//delay_100ms(30);	

	//MCU_GPIO_HIGH(GPIO_GSM_ONOFF);

	//printf("GSM On/OFF.\r\n");
}

// 打开给GSM模块的供电。
void gsm_power_up(void)
{
	// GPIO_ResetBits(GPIOB,GPIO_Pin_13);
	MCU_GPIO_HIGH(GPIO_GSM_PWR);
	
	delay_100ms(3);	

	sts_gsm_power = ON;

	printf("GSM powered up.\r\n");
}

// 关闭给GSM模块的供电。
void gsm_power_down(void)
{
	// GPIO_SetBits(GPIOB,GPIO_Pin_13);
	MCU_GPIO_LOW(GPIO_GSM_PWR);
	
	delay_100ms(3);	

	sts_gsm_power = OFF;

	printf("GSM powered down.\r\n");
}

/*
对GSM模块的休眠和唤醒操作，采取"默认休眠、主动使用前唤醒、主动使用后休眠、被动接收不处理"的原则。
------------------------------------------------------------------------------------------------
控制模块进入待机模式的基本流程：
1、保持模块的 DTR 输入为高电平，通过AT 指令将模块设置为允许进入休眠模式，参考指
令：at+enpwrsave。
2、将模块的 DTR 输入置低，硬件控制模块进入低功耗状态。通常模块会在2 秒左右进入
待机。在待机模式下，模块的串口是关闭的，没有响应。运行灯也会停止闪烁。
模块只有在空闲时才会进入待机模式，如果有数据交互未结束，不会进入待机。
3、如果本端有数据或者呼叫等主叫业务，可以将 DTR 置高，模块立即退出待机模式，进
入正常模式，串口打开响应AT 指令。在主叫业务处理完毕后，外部CPU 再将DTR 置
低，模块进入待机模式。
4、在待机状态下，如果模块有被叫业务，比如来电、来短信、服务器来的数据，模块会立
刻退出待机模式，并通过串口输出来电信息，外部CPU 在检测到串口信息后，建议先
将DTR 置高，再处理来电、数据等。待处理完毕后，将DTR 置低，使模块进入待机模
式。如果来电时，DTR 没有置高，且串口没有信息，则模块会在2~30 秒左右自动进入
待机模式。
*/

/*
at+enpwrsave=1
OK

at+enpwrsave=1
CME ERROR:<error>

at+enpwrsave?
+ENPWRSAVE:1
*/
// 将模块休眠。
int gsm_sleep(void)
{	
	// 拉高DTR引脚
//	MCU_GPIO_HIGH(GPIO_GSM_DTR);

	// 允许模块进入休眠模式
//	if(gsm_send_at("AT+ENPWRSAVE=1\r\n", "OK", 2) != OK)
//	{	
//		return NG;
//	}

	// 拉低DTR引脚
//	MCU_GPIO_LOW(GPIO_GSM_DTR);

	// 模块只有在空闲时才会进入待机模式，如果有数据交互未结束，不会进入待机。

//	printf("gsm sleep.\r\n");

	return OK;
}

// 强行唤醒GSM模块(仅在模块主动发起数据传输或短信或电话前调用)。
void gsm_wakeup(void)
{		// 检查当前DTR是否被拉低，被拉低则说明之前模块被手动置于休眠状态
//	if(MCU_GPIO_READ(GPIO_GSM_DTR)== 0)
//	{
		// 拉高DTR引脚
//		MCU_GPIO_HIGH(GPIO_GSM_DTR);
		
//		delay_100ms(20);

//		printf("gsm woke up.\r\n");
//	}
}

// 在M660+串口通讯异常(多由GPRS连接等涉及网络状况的耗时命令所引起)情况下，
// 尝试将模块恢复到上电后的初始状态(若系统连续检测到模块出现三次以上的异常，
// 则认为模块出现重大故障，然后启动硬件故障报警机制)。
int gsm_recover(void)
{
	printf("to recover GSM No.%d times...\r\n", cnt_gsm_recovered+1);
	
	if(gsm_init() == OK)
	{	
		printf("recover GSM module OK.\r\n");

		// 恢复gsm成功后将cnt_gsm_recovered重置为0
		cnt_gsm_recovered = 0;
			
		return OK;
	}
	// gsm初始化失败可能是由于网络信号不好、sim卡未检测到、sim卡欠费以及gsm模块串口通讯故障引起的
	else
	{
		printf("recover GSM module NG.\r\n");

		// cnt_gsm_recovered计数器递增
		cnt_gsm_recovered++;

		// 检查连续恢复gsm的次数是否超限
		if(cnt_gsm_recovered >= MAX_TIMES_GSM_RECOVER)
		{
			printf("failed to recovery gsm continuously %d times.\r\n", cnt_gsm_recovered);	
		}
		
		return NG;
	}
}

/**********************************************************************************************************************
											GSM串口接收缓冲操作函数
***********************************************************************************************************************/

// 在GSM UART接收缓冲中、在给定的时间内从当前读指针位置开始搜索指定的Pattern。
ERR_NO gsm_find_pattern(char* ptn, unsigned int to)
{
	int 			i = 0;
    unsigned int 	len = 0;
	unsigned int	RxCnt3_wr_backup = RxCnt3_wr;
	
	to = systick+to*1000/SYSTICK_PERIOD;

	// 检查输入参数的合法性
	len = strlen(ptn);
	
	if(len <= 0)
	{
		return ER_GSM_PATTERN_NULL;
	}

	while(systick < to)
	{  		
		// 查找模式字符串的过程可能耗时较长，因此查找过程中应及时喂狗
		WATCHDOG_RELOAD();

		// 检查串口命令输入
		check_buf_com();	
		// check_que_cmd(CHECK_MODE_CONTINUOUS);
		
        if((RxCnt3_wr - RxCnt3_rd) >= len)
        {   			
        	// 为简化起见，这里采用了逐字节移动的滑动窗口比较法，而未采用KMP等快速搜索算法，
        	// 实际上对于GSM接收缓冲的检测一般都是有目的的，即特征字符串出现的位置距离当前
        	// 读游标一般都不远，因而字符串匹配的过程耗时有限。
        	for(i = 0; i < len; i++)
            {
            	if(GSM_RX_RD(RxCnt3_rd+i) != ptn[i])
            	{
            		break;
            	}
            }
            
            if(i == len)
            {
            	RxCnt3_rd += i;		// 读游标后移相应长度
            	
            	return OK;
            }
            else
            {
            	RxCnt3_rd++;		// 从下一字节处重新搜索指定的Pattern
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

// 等待接收缓冲内接收到指定长度的未读数据（此函数多用于需要等待接受更多数据的时候，从而为后续处理积累新的数据）。
ERR_NO gsm_wait_output(unsigned int len, unsigned int to)
{
	unsigned int	RxCnt3_wr_backup = RxCnt3_wr;
		
	to  = systick + to*1000/SYSTICK_PERIOD;

    while(systick < to)
    {
		WATCHDOG_RELOAD();

		// 等待gsm串口输出时检查串口1有无命令输入。
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

// 从当前读游标位置开始提取数字字符串并将其转化为数值以返回(支持负数整数的提取)。
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
	
 	// 先定位到数字串的起始字符（'-'或'0~9'） 
    while(systick < to)
    {    	
		WATCHDOG_RELOAD();
		
    	if(RxCnt3_rd < RxCnt3_wr)
    	{
	    	ch = GSM_RX_RD(RxCnt3_rd++);		
	    	
			if(ch == '-')
			{
				// 未见测到任何数字字符前检测到'-'可认为是数值的正负符号
				if(i == 0)
				{
					*val = -1;
				}
				// 否则检测到'-'可认为是数字字符末尾的后一字符
				else
				{
					str[i] = '\0';		// 在数字字符数组后面添加字符串结尾符号

					break;
				}
			}
			else if(ch >= '0' && ch <= '9')
			{
				// 读游标暂时不移动，因刚好指向数字字符
				str[i++] = ch;

				if(i >= MAX_DECIMAL_DIGITS_FOR_INT)
				{
					str[i] = '\0';		// 在数字字符数组后面添加字符串结尾符号

					break;
				}
			}
			else
			{
				// 检查str的游标是否大于0，若大于，则说明str中已经填充了数字字符、
				// 当前检测到非数字字符的话说明数字字符串已经结束。
				if(i > 0)
				{
					str[i] = '\0';		// 在数字字符数组后面添加字符串结尾符号

					break;
				}
				// else: 略过当前的非数字字符，检测下一个字符
	    	}
	    }
    }

	if(RxCnt3_wr == RxCnt3_wr_backup)
	{
		return ER_GSM_UART_RECV_NOTHING;
	}
	else
	{
		// 将检测到的数字字符会串转化为数值(注意数值的正负号)
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

// 从当前读游标位置开始提取连续的数字字符（包括浮点数中的'.'）。
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
	
	// 先定位到首个数字字符
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

	// 更新to
	to -= (systick-systick_backup);

	// 提取连续的数字字符
    while(systick < to)
    {        
		WATCHDOG_RELOAD();
		
        if(RxCnt3_rd < RxCnt3_wr)   // GSM UART接收缓冲的读指针不能超过写指针，否则需要等待写指针更新
        {  
        	ch = GSM_RX_RD(RxCnt3_rd++); // RxBuf3[(RxCnt3_rd++) & (RxBuf3Size-1)];
			
            if(	(ch >= '0' && ch <= '9') || ch == '.')	// 支持浮点数的提取（不区分有几个'.'）
            {  
                dig[i++] = ch; 	

				if(i >= len)
				{
					dig[i] = '\0';	

					// 接收满目标字符串退出
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
					// 接收完数字字符退出
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

// 从当前读游标位置开始提取以'\r\n'结尾的字符串。
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
        	// 接收到换行符退出     
            if(GSM_RX_RD(RxCnt3_rd+0) == '\r' && GSM_RX_RD(RxCnt3_rd+1) == '\n')
            {  
                str[i] = '\0';
            
                RxCnt3_rd += 2;		// 前移读游标
            
                return OK;				
            }
            else
            {
            	str[i++] = GSM_RX_RD(RxCnt3_rd++);

				// 目标字符串接收满退出
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
提交有效的基站信息给Google服务器后，从服务器返回的定位信息:
d5
{"location":{"latitude":32.117301,"longitude":114.116606,"address":{"country":"China","country_code":"CN","region":"Henan","city":"Xinyang"},"accuracy":1625.0},"access_token":"2:OkivQMlLSpRHuPTx:Beb-gnvVNJBB3-vt"}
0
*/

// 提取指定分隔符内的字符(分隔符可能是"或|等)。
ERR_NO gsm_fetch_spliters(char ptn, char* str, int len, unsigned int to)
{
	unsigned int 	i 			= 0;
	int				spliters 	= 0;

	unsigned int	RxCnt3_wr_backup = RxCnt3_wr;
	
	to  = systick+to*1000/SYSTICK_PERIOD;

	// 提取第一个双引号之后、第二个双引号之前的字符(注意做超时检测以规避异常情况)
	while(systick < to)
	{
		WATCHDOG_RELOAD();
		
		if(RxCnt3_rd < RxCnt3_wr)
		{			
			// 类似AT+IPSTATUS=0命令的响应信息中状态字符串没有以逗号而是以换行符结尾，因此也要能提取出来
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
					// 接收完分隔符内的字符串退出
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

					// 目标字符串接收满退出
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
										 		GSM串口通讯功能函数
***********************************************************************************************************************/

// 发送指定的AT命令并检查返回信息中是否出现指定的特征字符串，从而判断AT命令发送后的反馈结果。
// 注: 由于每个AT命令的执行时间或长或短，因此发送AT命令的超时时间也应相应设定。
ERR_NO gsm_send_at(char* at, char* ptn, int to)
{	
	RxCnt3_rd = RxCnt3_wr;		// 发送AT命令前将读游标直接移到写游标位置，从而忽略之间的未读数据

	// 通过GSM UART口发送AT命令
	usart3_send_int(at, strlen(at));	

	// 在规定的时间内，从GSM UART接收缓冲的指定位置开始搜索指定的Pattern
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
// 查询当前的GSM信号强度(SIM卡未插入时也能正常查询RSSI)。
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
358511020024166		// 返回消息中无+CGSN:消息前缀，因此改用+CGSN\r\n作为检测返回消息的特征字符串
OK
*/
// 查询GSM模块的IMEI。
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

// 查询GSM模块的软件版本号。
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

/* 使用AT+POSI命令查询周边基站的信息(最多7个，SIM卡未插入时也能正常检测)
>> SIM卡插入时:

AT+POSI=1

+POSI: 1,460,01,517B,D2A3,38,42,0,460,01,517B,A872,30,38,0,460,01,517A,FFAC,0F,36,0,460,01,517A,6A7E,2F,34,0,460,01,517B,A871,19,31,0,460,01,517B,576D,36,29,0,460,01,517A,AAE8,2E,28,1

OK

1		// mode

// 推测基站列表中的第一个为当前注册基站，此基站信息可作为基站定位的首选
,
460,	// mcc	
01,		// mnc
517B,	// lac
D2A3,	// ci
38,		// bsic
42,		// rxlev
0		// ended

>> SIM卡未插入时:

AT+POSI=1

+POSI: 1,460,01,2533,71E0,39,53,0,460,01,2533,741E,17,37,0,460,01,2533,7172,22,34,0,460,01,2533,735E,35,32,0,460,01,2533,7171,2E,30,0,460,01,2533,71F4,1B,27,0,460,01,2533,741B,00,25,1

OK

如果还没有找到任何小区，则直接返回OK，如果基站信息为多条返回数据在MCC和ENDED 之间循环。
*/

/* 使用AT+CREG命令查询当前注册基站的信息(SIM卡未插入时不能检测):
>> SIM卡插入时:

AT+CREG=2

OK
AT+CREG?

+CREG: 2, 1, "517B", "A872", 0	// 第一次显示的基站似乎并非当前注册的基站

OK

+CREG: 1, "517B", "D2A3", 0		// 第二次显示的头一个基站才是当前注册的基站?

+CREG: 1, "517B", "A872", 0

>> SIM卡未插入时:
AT+CREG=2

OK
AT+CREG?

+CREG: 2, 0

OK

>> 设置注册基站信息自动输出后，模块可能因为基站切换而自动输出注册基站消息:

AT+CREG=2			// 使能注册基站信息自动输出

OK				
AT+CREG?

+CREG: 2, 1, "2796", "140C", 0

OK

+CREG: 1, "2796", "140D", 0		// 以下都是基站切换时自动输出的注册基站信息

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
// 检测GSM模块周边7个邻近基站的信息。
// 注: 刚初始化gsm模块后不宜马上查询gsm基站信息，多会出错。
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

	ret = gsm_send_at("AT+POSI=1\r\n", "+POSI: ", 6);  //sim900a中无此指令
	if(ret < 0)
	{		
		return ret;
	}

	// 记录基站信息字符串的起始位置。
	RxCnt3_rd_backup = RxCnt3_rd;

	ret = gsm_find_pattern("\r\n", 2);
	if(ret < 0)
	{		
		return ret;
	}

	to = systick+2*1000/SYSTICK_PERIOD;

	// 恢复基站信息字符串的起始位置。
	RxCnt3_rd = RxCnt3_rd_backup;

	// 提取各个基站的信息(每个基站包含7个域)
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
						
						// 接收完当期基站的当期域信息
						break;
					}
					else if(ch == '\r')
					{
						de_gsm_cell[i][j][k] = '\0';

						// 全部基站的全部域信息接收完成
						return OK;
					}
					else
					{
						// 接收到乱码
						return ER_GSM_UART_RECV_CHAOS;
					}
				}

				// 检查接收全部基站信息是否超时。
				if(systick > to)
				{
					// 接收超时退出
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
// 在GPRS连接建立后，查询模块自身的IP地址。
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
// 将制定的域名解析成IP地址(应在APN设置后执行)。
ERR_NO gsm_get_dns(char* ip, int len_ip, char* dn)
{
	char		at[128];
	ERR_NO		ret;

	printf("to inquire dns...\r\n");
	
	sprintf(at, "AT+CDNSGIP=\"%s\"\r\n", dn);  //sim900a  AT+CDNSGIP

	// 先检查rssi值是否正常
	ret = gsm_check_rssi();
	if(ret < 0)
	{
		return ret;
	}

	// OK和+DNS几乎同时出现
	ret = gsm_send_at(at, "OK", TO_GPRS_DO_DNS);
	if(ret < 0)	// M660+的AT+DNS命令超时已经设置并默认为5秒，2012-04-19 23:23
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
>> 通话过程中发送

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

>> 非通话过程中发送

AT+VTS=1

ERROR


>> 通话过程中接收DTMF

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
// 发送指定的DTMF字符(支持0,1,2,3,4,5,6,7,8,9,A,B,C,D,#,*，须在通话过程中发送)。
ERR_NO gsm_send_dtmf(char dtmf)
{
	char		at[16];

	sprintf(at, "AT+VTS=%c\r\n", dtmf);

	return gsm_send_at(at, "OK", 3);
}

/*
>> SIM卡插入时:

AT+CCID

+CCID: 89860109520204253890

OK

>> SIM卡未插入时:

AT+CCID

ERROR

*/
// 通过查询SIM ID号检测SIM卡是否插入。
ERR_NO gsm_check_sim(void)
{
	return gsm_send_at("AT+CPIN?\r\n", "CPIN", 2);
}

/*
>> 注册后

AT+CREG=2

OK
AT+CREG?

+CREG: 2, 1, "25F0", "0F8C", 0

OK

>> 未注册


*/
// 检查当前的网络注册状况。
ERR_NO gsm_check_reg(void)
{
	char	str[4+1];
	int		state;
	
	ERR_NO	ret;
	
	// 允许网络注册主动提供所在地讯息（CELL ID、LOCAL ID）
	ret = gsm_send_at("AT+CREG=2\r\n", "OK", 3);
	if(ret < 0)
	{	
		return ret;
    }

	// 查询网络注册状况
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
		return OK;	// 已注册
	}
	else
	{
		return ER_GSM_NETWORK_UNREGISTERED;	// 未注册
	}
}

// 检查rssi是否正常，一般在打出电话、发短信、发数据以及查询dns(需要发数据)前执行以便确认模块工作状态正常。
/*
------------------------------------
	signal 		rssi
------------------------------------
0 	<4或99 		<-107 dBm or unknown
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
	
	// 检测三次rssi值，如果大于5则返回ok
	for(i = 0; i < MAX_TIEMS_GSM_GET_RSSI; i++)
	{
// rssi检测为99时，每隔3秒检测一次、最多检测三次，以便尽可能等到gsm注册网络成功。
GET_GSM_RSSI:
		rssi = -1;

		if(gsm_get_rssi((int*)&rssi) == OK)
		{
			printf("#%d RSSI = %d\r\n", i+1, rssi);

			// rssi=99一般出现在网络未注册情况下
			if(rssi == 99)
			{
				cnt--;

				if(cnt >0)
				{
					delay_100ms(30);	// 间隔3秒
					
					goto GET_GSM_RSSI;
				}
				else
				{
					return -1;			// 尚未成功注册网络
				}
			}
			else if(rssi <= 4)
			{
				return -2;				// 信号太弱
			}
			else if(rssi >= MIN_RSSI_FOR_COMMUNICATION)
			{
				return OK;
			}
		}
		// else: 查询rssi失败则尝试重查
	}

	if(times > 0)
	{
		gsm_recover();

		times--;

		goto CHECK_GSM_RSSI;
	}

	// 恢复gsm后查询rssi仍不正常，返回NG
	printf("failed to check rssi after once trial of recovering gsm.\r\n");
	
	return -2;
}

/**********************************************************************************************************************
										 		GSM应用接口函数
***********************************************************************************************************************/

// 拨打电话到指定号码。
/*
>> 呼叫对方(测试中发现，信号低到4时仍可建立通话)

ATD13923887347	// 呼叫对方(通话可能在长达10秒内才建立)

OK

SPEECH ON

ALERTING

CONNECT

SPEECH OFF		// 对方挂断电话

RELEASE

NO CARRIER	

>> 对方来电

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

	// 格式化gsm号码
	if(format_pn(pn) != OK)
	{
		return NG;
	}

	// 先检查rssi值是否正常
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

	// 设置gsm通话状态标志
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

	// 重置gsm通话状态标志
	is_gsm_calling = FALSE;
	
	return OK;
}

/********************************* SMS 相关的操作函数 *************************************/
ERR_NO gsm_sms_mode_txt(void)
{
	return gsm_send_at("AT+CMGF=1\r\n", "OK", 5);
}

ERR_NO gsm_sms_mode_pdu(void)
{
	return gsm_send_at("AT+CMGF=0\r\n", "OK", 5);
}

// 将指定的内容以PDU格式的SMS发送给指定目标方。
// ascii_utf8	- 	以ASCII编码的英文待发送短信内容或以UTF-8编码的中文待发送短信内容。
ERR_NO sms_snd_pdu(T_SMS_LANGUAGE language, char* pns, char* ascii_utf8)
{
	ERR_NO			ret;	
	
	char			pdu[320];					// pdu的最大长度一般小于320字节

	unsigned char	encoded[MAX_LEN_CNC_SMS*MAX_NUM_CNC_SMS];	// 经特定编码后的短信内容(中文采用UCS2编码、英文采用7bit编码)
	unsigned char*	pencoded = (unsigned char*)encoded;	
	
	char			at[32];

	int				len = 0;	
	
	int				rest = 0;	
	unsigned int 	tobesent = 0; 
	unsigned int	len_seg = 0;		// 每个pdu包中包含的纯用户数据长度(中文和英文不同)

	unsigned int	feed;

	unsigned char	udh[7];				// uder data header
	unsigned int	len_udh = 0;		// udh长度(中文使用6字节udh、英文使用7字节udh)
	int				segment_total = 0;	// 长短信拆分发送的总条数
	int				segment_count = 0;	// 长短信差分法送的当前序号(从1开始计数)	

	// 计算待发送字符串的原始长度(未编码前)
	rest = strlen(ascii_utf8);

	printf("content to be sent in pdu:%s\r\n", ascii_utf8);

	// 先将待发送内容根据语言类别编码为特定的格式
	if(language == SMS_LANGUAGE_CHINESE)
	{
		rest = pdu_encode16bit(encoded, ascii_utf8, rest);	

		// 检查待发送的短信内容中是否包含非utf8编码也非ascii编码的字符。
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

		// 检查待发送短信内容中是否包含非ASCII字符。
		if(rest <= 0)
		{
			return ER_GSM_SMS_PDU_CHAOS;
		}

		len_udh = 7;
		len_seg = MAX_BYTE_SMS_PDU - len_udh;
	}

	// 计算短信要拆分的总条数(每条拆分后的短信最大长度为MAX_BYTE_SMS_PDU减去udh的长度6字节)
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
	else	// : 发送短信内容长度小于最大pdu长度时，不以级联短信方式发送
	{
		segment_total = 1;
	}

	// 用于产生伪随机数
	feed = systick;

	// 拆分发送长短信
	while(rest > 0)
	{			
		// 检查是否需要将短信拆分发送
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

			// 构造udh(中文使用6字节udh，以便剩余的134字节可以容纳整数个汉字，英文使用7字节udh，以便udh本身和uds一样构成7字节组)
			if(len_udh == 6)
			{				
				udh[0] = 0x05;
				udh[1] = 0x00;
				udh[2] = 0x03;
				udh[3] = feed&0xFF;			// serial number			
				udh[4] = segment_total;		
				udh[5] = segment_count;				// 分条短信从1开始计数
			}
			else if(len_udh == 7)
			{			
				udh[0] = 0x06;
				udh[1] = 0x08;
				udh[2] = 0x04;
				udh[3] = feed&0xFF;			// serial number			
				udh[4] = (feed>>2)&0xFF;		// serial number	
				udh[5] = segment_total;		
				udh[6] = segment_count;				// 分条短信从1开始计数
			}			

			// 将编码后的短信内容打包成pdu字符串
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

		// 检查SMS模式设置命令是否执行成功，若不成功，则可能因为当前处于SMS PDU输入模式，此时需发送0x1B终止输入模式
		ret = gsm_send_at("AT+CMGF=0\r\n", "OK", 2);		
		if(ret < 0)
		{
			usart3_send_int("\x1B", 1);	

			printf("error in \"AT+CMGF=0\".\r\n");
			
			return ret;
		}
		
		// 发送PDU模式短信的命令AT+CMGS=后面需要紧跟待发送PDU包的长度(不包括SCA部分的，即不包括gsm_sca_len + gsm_sca_fmt + gsm_sca_str)
		// 此长度数值必须正确，否则SMS发送会失败。
		sprintf(at, "AT+CMGS=%d\r", len); 	// 这里用\r而非\r\n去结束AT命令，可能是MTK协议栈的Bug
		
		// 检查是否进入SMS PDU输入模式，若未进入，则可能因为当前正处于SMS PDU输入模式，此时需要发送0x1B终止输入模式
		ret = gsm_send_at(at, ">", 2);
		if(ret < 0)	// 等待SMS PDU输入指示符'>'
		{
			usart3_send_int("\x1B", 1);	

			printf("error in waiting for '>'.\r\n");
			 
			return ret;
		}
		
		// 发送短信
		ret = gsm_send_at(pdu, "+CMGS:", TO_SMS_TX);
		if(ret < 0)	
		{
			usart3_send_int("\x1B", 1);	// cancel SMS inputing mode so to accept following AT command 

			printf("error in sending pdu.\r\n");
			
			return ret;
		}	
	}
	
	// 发送完SMS后，强行将SMS收发模式设置为TXT模式，便于接收SMS命令	
	ret = gsm_sms_mode_txt();
	if(ret < 0)
	{
		printf("error in setting sms to text mode.\r\n");

		return ret;
	}
	
	return OK;
}

// 将给定的内容以TXT格式的SMS发送给指定目标方(通过适当的中文编码，如在代码编辑器中输入中文字符，可发送中文SMS)。
ERR_NO sms_snd_txt(char* pns, char* ascii)
{
	ERR_NO	ret;
	char 	at[281];

	// 检查SMS模式设置命令是否执行成功，若不成功，则可能因为当前处于SMS PDU输入模式，此时需发送0x1B终止输入模式
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

	// 输入TXT格式的SMS内容，注意在内容后面添加二进制1A即换行符\r\n，否则SMS发送不会成功
	sprintf((char*)at, "%s\x1A\r\n", ascii);
	
	// 发送短信
	ret = gsm_send_at(at, "+CMGS:", TO_SMS_TX);
	if(ret < 0)
	{
		usart3_send_int("\x1B", 1);		// cancel SMS inputing mode so to accept following AT command 

		printf("error in sending pdu.\r\n");

		return ret;
	}

	return OK;
}

// 发送指定的消息到指定的号码，集成了短消息拆分发送和发送失败后自动重发功能。
// 注: 输入的待发送消息要么以ASCII编码，要么以UTF-8编码，以便统一作为字符串处理。
// 使用gsm_send_sms函数单次可发送的最大汉字数量为MAX_CHAR_CNC_SMS_CN定义、单次
// 可发送的最大英文数量不受此宏定义限制，但是受程序可分配的栈空间限制(单次发送269字符没问题)。
ERR_NO gsm_send_sms(char* pn, char* ascii_utf8)
{	
	ERR_NO			ret;

	char				pns[MAX_LEN_PN+1];			// 标准化的gsm号码(包含国家区号前缀)
	
	int				len	 = 0;	// 待发送的字符数量

	T_SMS_LANGUAGE	language = SMS_LANGUAGE_ENGLISH;

	// 通话状态下可正常发送短信!!!

	// 若发送号码为空，则将短信重定向到串口输出
	if(strlen(pn) <= 0)
	{	
		printf("pn is null and redirect to uart:%s", ascii_utf8);

		return NG;
	}

	// 检查待发内容是否为空
	len = strlen(ascii_utf8);
	
	if(len <= 0)
	{
		printf("reply is null.\r\n");
		
		return NG;
	}

	// 格式化gsm号码
	if(format_pn(pn) != OK)
	{
		return NG;
	}

	// 将纯gsm号码添加国家区号前缀(发送pdu短信时默认设定号码格式需携带国家区号前缀)。
	strcpy(pns, gsm_telecode);
	strcat(pns, pn);

	// 检查待发送内容是否为中文
	if(is_all_ascii(ascii_utf8) == FALSE)
	{
		language = SMS_LANGUAGE_CHINESE;
	}

	// printf("sms length = %d and is written in %d.\r\n", len, language);

	// 检查待发送的短息字符是否超过允许的最大数量(中文短信内容采用UTF8编码输入、英文短信内容采用ASCII编码输入，前者每字符占3字节、后者每字符占1字节)
	// 发送的单条短信最大长度为4条级联短信
	if(len > MAX_LEN_CNC_SMS*MAX_NUM_CNC_SMS)
	{
		printf("max. length of a Chinese or English SMS is %d .\r\n", MAX_LEN_CNC_SMS*MAX_NUM_CNC_SMS);

		return NG;
	}
		
	gsm_wakeup();

	// 检查RSSI值是否为-1，若是，则多为模块GSM自动断电或复位，此时需恢复GSM模块
#if 1
	// 先检查rssi值是否正常
	ret = gsm_check_rssi();

	if(ret < 0)
	{
		gsm_sleep();

		return NG;
	}
#endif

	// 固定采用pdu格式发送短信(英文以8bit编码、中文以ucs2编码)	
	ret = sms_snd_pdu(language, pns, ascii_utf8);

	gsm_sleep();
	
	// 发送出错且错误不是短息内容乱码情况下，缓存当前短信准备重发		
	if(ret == ER_GSM_SMS_PDU_CHAOS)	// 待发送短息内容中包含不支持的字符编码
	{
		// 对于包含不支持的字符编码的短信，不予重发
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
>> SIM卡插入状态下，输入TCP连接命令后，如果连接成功的话，多在5秒内，
   如果连接失败的话，多在18秒内:
   
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

>> SIM卡未插状态下，输入TCP连接命令后，会即刻返回SIM卡未插入的错误消息:

AT+NETAPN="CMNET","",""

OK
AT+TCPSETUP=0,74.125.71.105,80

OK
+CME ERROR: NO SIM

+TCPSETUP:0,FAIL
*/
// 根据指定的GPRS连接号建立相应的GPRS连接。
// 1，M660+常出现阶段性的正?蛞斐Ｏ窒螅茨炒慰螅羰状蜧PRS连接正常的话，
//	  则随后的连接正常的概率很大，反之亦然。因此某次开机后若发现GPRS连接不正常，
//	  较好的办法是关闭模块供电然后上电;
// 2，GPRS连接过程中若有来电呼入，则GPRS连接一般会失败，
//    GPRS已经连接时若有来电呼入，则GPRS连接不会断开;
//    通话过程中建立GPRS连接，则一般会失败;

//modified by double lin
const u8 *modetbl[2]={"TCP","UDP"};//连接模式

int sim900a_tcpudp_test(u8 mode,u8* ipaddr,u8* port,u8 id)
{ 
	char	at[64];

	if(mode)printf("SIM900A UDP连接测试\n");
	else printf("SIM900A TCP连接测试\n"); 

	//sprintf(at,"AT+CIPSTATUS=?\r\n");
	//if(gprs_soc_status(id) != OK)
	{
		sprintf(at,"AT+CIPSTART=\"%s\",\"%s\",\"%s\"\r\n",modetbl[mode],ipaddr,port);
		if(gsm_send_at(at, "OK", 5) != OK)return NG;		//发起连接
	}
	return OK;
}
int gprs_soc_setup_dns(void)
{
	char	at[64];
	int ret;
	sprintf(at, "AT+CIPSHUT\r\n");	
	ret=gsm_send_at(at, "SHUT OK", 1) ;	 
	if(ret!= OK)		// to = 10 需要测试
	{
		printf("failed to CIPSHUT.\r\n");
		
		return NG;
	}


	sprintf(at, "AT+CIPMUX=0\r\n");	
	ret=gsm_send_at(at, "OK", 1);
	if(ret!= OK)		// to = 10 需要测试
	{
		printf("failed to CIPMUX.\r\n");
		
		return NG;
	}


	sprintf(at, "AT+CIPRXGET=1\r\n");	
	ret=gsm_send_at(at, "OK", 1);
	if(ret!= OK)		// to = 10 需要测试
	{
		printf("failed to CIPRXGET.\r\n");
		
		return NG;
	}


	sprintf(at, "AT+CIPQRCLOSE=1\r\n");	
	ret=gsm_send_at(at, "OK", 1);
	if(ret!= OK)		// to = 10 需要测试
	{
		printf("failed to CIPQRCLOSE.\r\n");
		
		return NG;
	}


	sprintf(at, "AT+CIPMODE=0\r\n");	
	ret=gsm_send_at(at, "OK", 1);
	if(ret!= OK)		// to = 10 需要测试
	{
		printf("failed to set CIPMODE.\r\n");
		
		return NG;
	}


	sprintf(at, "AT+CIPSTART=\"TCP\",\"bin172133.oicp.net\",8080\r\n");	
	ret=gsm_send_at(at, "OK", 1);
	if(ret!= OK)		// to = 10 需要测试
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
//	const u8 *port="8086";	//端口固定为8086,当你的电脑8086端口被其他程序占用的时候,请修改为其他空闲端口
	u8 mode=0;				//0,TCP连接;1,UDP连接


	printf("to setup gprs connection to %s:%s...\r\n", (char*)tcp_conn[id].ip, (char*)tcp_conn[id].port);
	/// printf("to setup GPRS connection.\r\n");

	// 通话状态下不能建立gprs连接
	if(is_gsm_calling == TRUE)
	{
		printf("gsm is in calling and can not send data!\r\n");
		
		return NG;
	}

	// 检查待建立连接的GPRS配置参数中IP地址是否为空，为空的话先执行域名解析
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
	//关闭连接
	sprintf(at, "AT+CIPCLOSE=1\r\n");	
	gsm_send_at(at, "CLOSE OK", 1);
	
	//关闭移动场景
	sprintf(at, "AT+CIPSHUT\r\n");	
	gsm_send_at(at, "SHUT OK", 1) ;
	
	//设置移动台类别
	sprintf(at, "AT+CGCLASS=\"B\"\r\n");	
	if(gsm_send_at(at, "OK", 10) != OK)		// to = 10 需要测试
	{
		printf("failed to set CGCLASS.\r\n");
		
		return NG;
	}
	//设置PDP上下文
	sprintf(at, "AT+CGDCONT=1,\"IP\",\"CMNET\"\r\n");
	if(gsm_send_at(at, "OK", 10) != OK)		// to = 10 需要测试
	{
		printf("failed to set CGDCONT.\r\n");
		
		return NG;
	}
	//设置附着GPRS业务
	sprintf(at, "AT+CGATT=1\r\n");
	if(gsm_send_at(at, "OK", 5) != OK)		// to = 5 需要测试
	{
		printf("failed to set CGATT.\r\n");
		
		return NG;
	}
	//设置为GPRS连接模式
	sprintf(at, "AT+CIPCSGP=1,\"CMNET\"\r\n");
	if(gsm_send_at(at, "OK", 5) != OK)		// to = 5 需要测试
	{
		printf("failed to set CIPCSGP.\r\n");
		
		return NG;
	}
	//设置接收数据显示IP头(方便判断数据来源)
	sprintf(at, "AT+CIPHEAD=1\r\n");
	if(gsm_send_at(at, "OK", 5) != OK)		// to = 5 需要测试
	{
		printf("failed to set CIPHEAD.\r\n");
		
		return NG;
	}

//	return sim900a_tcpudp_test(mode,(u8 *)tcp_conn[id].ip,(u8 *)tcp_conn[id].port,id);
	if(sim900a_tcpudp_test(mode,(u8 *)tcp_conn[id].ip,(u8 *)tcp_conn[id].port,id) !=OK) return NG;
	// 设置APN	"AT+NETAPN=%s,\"\",\"\"\r\n"
//	sprintf(at, "AT+NETAPN=\"%s\",\"\",\"\"\r\n", gsm_apn);	//SIM900A AT+CSTT
//	
//	if(gsm_send_at(at, "OK", TO_GPRS_SET_APN) != OK)		// to = 2
//	{
//		printf("failed to set APN.\r\n");
//		
//		return NG;
//	}

	// printf("APN set.\r\n");
		
//	sprintf(ptn, "+TCPSETUP:%d,OK", id);	// 加强判断，令它能区别OK与FALL
	
	// 建立GPRS连接
//	sprintf(at, "AT+TCPSETUP=%d,%s,%s\r\n", id, tcp_conn[id].ip, tcp_conn[id].port); //SIM900A  AT+CIPSTART

	
	while(errors < (MAX_TIMES_GPRS_SETUP_CONNECTION+2) )//等待4 X 5 = 20s  modified by double lin
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
// 根据指定的GPRS连接号关闭相应的GPRS连接。

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
// 检查指定GPRS连接的连接状态，若返回值为OK，则为连接状态，否则为未连接状态。
// 注: GPRS连接自动保持超时时间为30分钟，即GPRS连接空闲超过30分钟，GSM基站会将其断开。

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

    // 先检测反馈消息的消息头
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

>						// 模块发送1234，服务器收到1234
OK
+TCPSEND:0,4

+TCPRECV: 0,4,4321		// 服务器发送4321，模块收到4321

+TCPRECV: 0,8,abcd1234	// 服务器发送abcd1234，模块收到abcd1234
*/
// 在非透传模式下从指定的GPRS连接上发送给定长度的数据，
// 并返回实际成功发送的数据长度(即从自动分割分批发送机制)。

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
	int			  	n_tobesent 	= 0;		// 每次待发送的字节数
	int 			n_reported		= 0;		// 每次发送后GSM模块反馈的发送成功字节数
	int				n_sentout 		= 0;		// 累计发送成功的字节数

	int				repeat  		= MAX_TIMES_GPRS_SEND_PACKET;    // repat times to sent the identical packet

	char			at[32];						// to accomodate the command header																		

//	char			ptn[16];	

	unsigned int 	to;

	// 按照待发送字节数及发送每帧数据的超时计算总发送超时时间。
	to = ((size/MAX_LEN_GPRS_PACKET_SEND)+((size%MAX_LEN_GPRS_PACKET_SEND)?1:0))*(TO_GPRS_TX_FRAME+1);

	printf("to send %d bytes within %d seconds.\r\n", size, to);

	// 通话状态下不能发送gprs数据
	if(is_gsm_calling == TRUE)
	{
		printf("gsm is in calling and can not send data!\r\n");
		
		return NG;
	}

	// 发送GPRS数据包之前，略过此前接收到但未读的所有数据
	RxCnt3_rd = RxCnt3_wr;

	to = systick+to*1000/SYSTICK_PERIOD;

	// 逐帧发送GPRS数据
	while(size > 0 && systick < to)
	{     
	    n_reported = 0;    

        // 每次发送的数据长度不能超过过AT+SOCSEND命令允许的最大数据长度(=496字节)
		n_tobesent = size > MAX_LEN_GPRS_PACKET_SEND ? MAX_LEN_GPRS_PACKET_SEND : size;

		// 构造消息头	:加\r\n
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
			//sprintf(ptn, "SEND FAIL");	//检查返回数据中是否有SEND FAIL  如果有则说明发送失败			 
			//if(gsm_find_pattern(ptn, TO_GPRS_TX_FRAME) == OK)
			//{
			//	return n_sentout;
			//}

			return n_sentout;
		}
		

		// 等待至少接收到一个字节的后续数据
		/*
       	if(!gsm_wait_output(1, 2))
        {
        	return n_sentout;
        }

		// 检测返回的已成功发送的数据字节数
		if(gsm_fetch_value((int*)&n_reported, 2) != OK)
		{
			return n_sentout;
		}
		*/
        // 比较期望发送的数据长度和实际发送的数据长度是否一致
        // 注: 仅在反馈的发送字节数不等于期望发送的字节数时才重发，因为这种情况多
        // 出现在PRS连接在传输过程中突然变得不稳定时(如GSM信号突然中断或对方服务器暂时宕机等)，
        // 在此之前若出现异常，则多为GSM硬件/软件出现重大故障，因此没必要即刻重发(会将未发送成功
        // 的数据加入到全局的重发数据队列在择机重发)。
        n_reported = n_tobesent;
		if(n_reported == n_tobesent)
		{
			size        -= n_reported;		// 剩余数据的长度递减
	        ptr         += n_reported;		// 读指针后移
	        n_sentout   += n_reported;		// 累计发送字节数递增
            
            repeat       = MAX_TIMES_GPRS_SEND_PACKET; 	// 为下一个数据帧的发送重置重发次数计数器 
		}
		// 刚刚发送的数据长度不等于待发送数据长度时，尝试重发当前数据帧，重发次数由系统设定
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

// 将驱动层的gprs_soc_tx()函数封装成带重发处理的适配层函数，并集成自动重发
// 机制(是否重发取决于外部调用函数的设置)和链路自动管理机制(链路有效则不重建，
// 否则重建链路)。

//modified by double lin
int gsm_send_data(int id, unsigned char* data, unsigned int size)
{
	volatile unsigned int systick1 = 0;
	volatile unsigned int systick2 = 0;

	int			ret;
	int			errors = 0;

#if 1
	// 先检查rssi值是否正常
	ret = gsm_check_rssi();
	if(ret < 0)
	{
		return -1;
	}
#endif

#if 1
	// 检查指定的GPRS连接是否有效，若无效，则建立之
	ret = gprs_soc_status(id);
#endif

	// printf("step out gprs_soc_status().\r\n");

	// 若指定GPRS连接当前处于断开状态，则尝试连接(连接函数内部会最多连续重试三次，三次都失败才返回出错结果)
	if(ret != OK)
	{
		printf("指定GPRS连接当前处于断开状态，尝试连接\n");	
		systick1 = systick;
#if 0
		ret = gprs_soc_setup(id);
#endif
#if 1
		ret = gprs_soc_setup_dns();
#endif
		systick2 = systick;		

		// 若建立连接失败，则将待发送数据写入Flash中以待重发
		if(ret != OK)
		{
			printf("failed to set up GPRS connection within %d ms.\r\n", (systick2-systick1)*SYSTICK_PERIOD);
			
			return -2;		// 连接失败
		}	
		else
		{
			printf("it took %5d ms to set up GPRS connection.\r\n", (systick2-systick1)*SYSTICK_PERIOD);
		}
	}

	// 发送完整数据，发送出错(完全发送成功才算成功)最多连续重试三次。
	while(errors < MAX_TIMES_GPRS_SEND_DATA)
	{			
		systick1 = systick;
		ret = (int)gprs_soc_tx(id, data, size);
		systick2 = systick;
		
		if(ret != size)
		{		
			errors++;	

			// 两次发送之间间隔一段时间
			delay_100ms(5);
		}
		else
		{
			printf("it took %5d ms to send out %5d bytes data.\r\n", (systick2-systick1)*SYSTICK_PERIOD, size);

			// 返回发送的数据长度(实际为拷贝)
			return size;
		}
	}

	return -3;			// 发送失败
}

// 在指定的GPRS连接上接收数据。
// 一般在发送数据后才调用接收数据的函数。
// +TCPRECV: 0,8,abcd1234	// 服务器发送abcd1234，模块收到abcd1234

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
	// 先等待接收10字节数据(TCP接收数据包的包头"+TCPRECV: "就占去了10字节长度)
	if(gsm_wait_output(10, to) != OK)
	{
		printf("failed to receive 10 bytes within %d seconds.\r\n", to);

		return -1;
	}
#endif

	// 等待接收返回数据
	if(gsm_wait_output(7, to) != OK)
	{
		printf("faield to wait for 7 bytes within 10s.\n");

		return -1;
	}

	// 等待接收并定位到TCP接收数据包的包头。
	if(gsm_find_pattern(pattern, 5) != OK)
	{
		printf("failed to find %s.\r\n", pattern);
	
		return -2;
	}

	// 提取TCP数据包的数据长度信息�
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

	// 提取TCP数据包的数据内容。
	RxCnt3_rd++;//跳过":"
	i = 0;
	
	while(i < size)
	{
		if(RxCnt3_rd < RxCnt3_wr)
		{
			data[i++] = GSM_RX_RD(RxCnt3_rd++);
		}
	}

	printf("\r\n");

	// 返回接收到的数据的长度
	return size;		
}

#endif

