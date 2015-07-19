/**
  ******************************************************************************
  * @file    Project/Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.0.0
  * @date    04/06/2009
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x.h"

#include "string.h"

#include "common.h"
#include "sms.h"

#include "log_motion.h"
#include "log_queue.h"
#include "log_cmd.h"
#include "log_pos.h"

#include "dev_gsm.h"
#include "dev_mma845x.h"
#include "dev_gps.h"

#include "mcu_systick.h"
#include "mcu_systick.h"
#include "mcu_i2c.h"
#include "mcu_flash.h"
#include "log_power.h"

#include "mcu_key.h"
#include "mcu_adc.h"
#include "mcu_systick.h"
#include "mcu_usart.h"
#include "mcu_exti.h"
#include "mcu_gpio.h"
#include "mcu_timer.h"

extern volatile unsigned int TimingDelay;

extern SWITCH	sw_gps_forward;

//各种加速度传感器中断响应处理
#ifdef TEST_GSENSOR_BASIC_FUNCTION

int cnt_singletap 	= 0;
int cnt_doubletap 	= 0;

int cnt_shake 		= 0;

int cnt_motion	 	= 0;
int cnt_freefall 		= 0;

int cnt_roll 			= 0;

int cnt_dataready	= 0;
#endif

static int		cnt_gsensor_still  = 0;			// 静止检测中断次数计数器
static int		cnt_gsensor_motion = 0;			// 运动检测中断次数计数器

void mcu_shutdown(void);

// -------------------------------------------------------
// 特征字符串	 状态编号   原始字符串	消息类型
// -------------------------------------------------------
// CLIP				30		// 来电号码输出
// SMSFLAG			31		// 短信输出
// KEY				32		// DTMF检测输出
// RELEASE			33		// 电话挂断
// RING				34		// 来电振铃
// CMGR				35		// 读取短信
// TCPRECV			36		// TCP数据接收
// -------------------------------------------------------

#define MAX_STATE_NUM_GSM		27			// GSM特征字符串状态机的最大状态值

const unsigned char ptn_map_gsm[MAX_STATE_NUM_GSM+1][16]=
{
	// input_gsm: A~Y
	{0, 1, 0, 0, 0, 0, 10, 0, 0, 0, 0, 12, 4, 22, 0, 0},	// state 0
	{0, 0, 0, 0, 0, 0, 0, 2, 20, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 30, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 0, 0, 0},
	{0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0},
	{9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 31, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 11, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 32},
	{0, 0, 13, 0, 0, 18, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 14, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 17, 0, 0, 0},
	{0, 0, 33, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 19, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 34, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 21, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 0, 0, 0, 0},
	{0, 23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 24, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25, 0, 0, 0, 0},
	{0, 0, 26, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 36, 0},		// state 27
};

const unsigned char ptn_idx_gsm[25] = {0, 99, 1, 99, 2, 3, 4, 99, 5, 99, 6, 7, 8, 9, 99, 10, 99, 11, 12, 13, 99, 14, 99, 99, 15};


// GPS串口特征字符串检测状态迁移表(仅检测$GPRMC和$GPGGA特征字符串)
#define MAX_STATE_NUM_GPS			8		// GPS特征字符串状态迁移表的最多状态数

const unsigned char ptn_map_gps[MAX_STATE_NUM_GPS][8]=
{
//   $ A C G M P R S
	{1,0,0,0,0,0,0,0},	// state 0
	{0,0,0,2,0,0,0,0},	// state 1
	{0,0,0,0,0,3,0,0},	// state 2
	{0,0,0,6,0,0,4,0},	// state 3
	{0,0,0,0,5,0,0,0},	// state 4
	{0,0,8,0,0,0,0,0},	// state 5
	{0,0,0,0,0,0,0,7},	// state 6
	{0,9,0,0,0,0,0,0},	// state 7
};

// 建立'A' - 'S'之间字符在gps_ptn_map表中的索引号(非特征字符对应0，但不意味其在gps_ptn_map表中的索引号为0)，
// 从而便于输入当前带检测字符快速得到其在gps_ptn_map表中的索引号。
const unsigned char ptn_idx_gps[19] = {1,0,2,0,0,0,3,0,0,0,0,0,4,0,0,5,0,6,7};

#ifdef USING_PWR_ACC
void isr_accpwr_detected(void);
#endif

#ifdef USING_PWR_EXT
void isr_extpwr_detected(void);
#endif

// void isr_chgsts_asserted(void);
#ifdef USING_DEV_GSM
void isr_gsmring_asserted(void);
#endif

void isr_batvol_to_measure(void);
void isr_gsrval_to_measure(void);

__asm int __get_msp(void)         //获取sp的地址,主程序使用的MSP因此返回MSP
{
	MRS     R0,MSP              ; main function is use msp,so return msp here
    BX      LR
}

__asm int __get_psp(void)         //获取sp的地址,主程序使用的MSP因此返回MSP
{
	MRS     R0,PSP              ; main function is use msp,so return msp here
    BX      LR
}

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval : None
  */
void NMI_Handler(void)
{
	while(1)
	{
		printf("NMI_Handler\r\n");

		mcu_reset();
	}
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval : None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
		printf("HardFault_Handler\r\n");

		mcu_reset();
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval : None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  		printf("MemManage_Handler\r\n");

		mcu_reset();
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval : None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  		printf("BusFault_Handler\r\n");

		mcu_reset();
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval : None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  		printf("UsageFault_Handler\r\n");

		mcu_reset();
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval : None
  */
void SVC_Handler(void)
{
	while(1)
	{
		printf("SVC_Handler\r\n");

		mcu_reset();
	}
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval : None
  */
void DebugMon_Handler(void)
{
	while(1)
	{
		printf("DebugMon_Handler\r\n");

		mcu_reset();
	}
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval : None
  */
void PendSV_Handler(void)
{
	while(1)
	{
		printf("PendSV_Handler\r\n");

		mcu_reset();
	}
}

/*******************************************************************************
* Function Name  : USART1_IRQHandler <--> PC
* Description    : This function handles USART1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
unsigned char		sts_usart1_rx  = 0;
unsigned int		cnt_usart1_cmd = 0;

/*
1，系统初始化后，usart1接收缓冲为空；
2，用户通过串口发送数据时，数据会被保存在usart1接收缓冲中，接收数据过程中检测到命令结尾符号"#\r\n"则串口命令计数器递增；
3，主循环中检测串口命令计数器，若大于0，则将usart1接收缓冲的当前读游标开始到"#\r\n"结尾之间的字符串截取下来做命令解析；
4，命令解析时先通过分隔符逗号将整个命令字符串分割为各个命令域，第一个命令域为命令本体；
5，先在全局命令列表中检查当前命令本体是否有匹配项，有的话继续检查命令参数个数和格式是否合法，都合法的执行命令，否则不执行；
注: 
*/
void USART1_IRQHandler(void)
{
	unsigned char	ch;
	
	 // handle USART1 Rx interrupt
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
 	{
	 	ch = USART_ReceiveData(USART1) & 0xFF;
		
        /* Read the latest byte from the receive data register */
        COM1_RX_WR(RxCnt1_wr++, ch);

		// 仅在非GPS转发也非GSM转发状态下才检查USART1命令
		// if(sw_gps_forward == OFF && is_gsm_forwarding == OFF)
		{
			// 接收usart1数据过程中不停检测串口命令的结尾标志"#\r\n"，检测到的话九江串口命令计数器递增。
			if(ch == '#')
			{
				if(sts_usart1_rx == 0)
				{
					sts_usart1_rx = 1;
				}
				else
				{
					sts_usart1_rx = 0;
				}
			}
			else if(ch == '\r')
			{			
				if(sts_usart1_rx == 1)
				{
					sts_usart1_rx = 2;
				}
				else
				{
					sts_usart1_rx = 0;
				}
			}
			else if(ch == '\n')
			{		
				if(sts_usart1_rx == 2)
				{
					sts_usart1_rx = 0;

					cnt_usart1_cmd++;
				}
				else
				{
					sts_usart1_rx = 0;
				}
			}
			else
			{
				sts_usart1_rx = 0;
			}
		}
		
        /* Clear the USART1 Receive interrupt */
       	USART_ClearITPendingBit(USART1, USART_IT_RXNE);  
 	 }

  	// handle USART1 Tx interrupt
  	if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
  	{ 
  		/* Write one byte to the transmit data register */
    	USART_SendData(USART1, TxBuf1[TxCnt1_rd++ & (TxBuf1Size-1)]);                    
	
    	/* Clear the USART1 transmit interrupt */
    	USART_ClearITPendingBit(USART1, USART_IT_TXE); 	  

		// disable USART1 Tx interrupt once all data in TxBuf1 have been sent out.
  		if(TxCnt1_rd == TxCnt1_wr)
		{
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
		} 
  	}  
}

/* 
不同定位状态下u-blox 6010在uCenter软件窗口输出的一组完整消息流
（消息前面的时间信息为uCenter自动添加，非消息本身所有）

>> 未见测到UTC时间（更未定位）时
??:??:??  $GPRMC,,V,,,,,,,,,,N*53
??:??:??  $GPVTG,,,,,,,,,N*30
??:??:??  $GPGGA,,,,,,0,00,99.99,,,,,,*48
??:??:??  $GPGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*30
??:??:??  $GPGSV,1,1,00*79
??:??:??  $GPGLL,,,,,,V,N*64

>> 检测到UTC时间但未定位时（UCT时间检测可早于定位成功）
14:12:03  $GPRMC,141203.00,V,,,,,,,230212,,,N*78
14:12:03  $GPVTG,,,,,,,,,N*30
14:12:03  $GPGGA,141203.00,,,,,0,03,30.33,,,,,,*63
14:12:03  $GPGSA,A,1,30,23,16,,,,,,,,,,30.35,30.33,1.00*02
14:12:03  $GPGSV,1,1,04,16,53,354,39,23,47,306,30,24,24,041,,30,38,029,43*71
14:12:03  $GPGLL,,,,,141203.00,V,N*4F

>> 定位后
14:18:29  $GPRMC,141829.00,A,2234.05540,N,11402.73255,E,1.229,,230212,,,A*7C
14:18:29  $GPVTG,,T,,M,1.229,N,2.275,K,A*29
14:18:29  $GPGGA,141829.00,2234.05540,N,11402.73255,E,1,03,15.34,63.8,M,-2.4,M,,*4E
14:18:29  $GPGSA,A,2,30,23,16,,,,,,,,,,15.37,15.34,1.00*04
14:18:29  $GPGSV,3,1,09,03,78,182,,06,77,091,15,11,01,197,,16,52,358,35*79
14:18:29  $GPGSV,3,2,09,23,49,303,32,24,22,043,,30,37,032,39,31,31,093,*7B
14:18:29  $GPGSV,3,3,09,32,15,214,*42
14:18:29  $GPGLL,2234.05540,N,11402.73255,E,141829.00,A,A*6D
14:18:29  $GPZDA,141829.00,23,02,2012,00,00*63
08:36:18  $GPRMC,083618.00,A,2233.99086,N,11402.66757,E,0.025,,250113,,,A*7A
08:36:18  $GPVTG,,T,,M,0.025,N,0.047,K,A*27
08:36:18  $GPGGA,083618.00,2233.99086,N,11402.66757,E,1,06,1.29,6.6,M,-2.4,M,,*42
08:36:18  $GPGSA,A,3,18,22,24,21,12,25,,,,,,,3.60,1.29,3.36*03
08:36:18  $GPGSV,2,1,08,12,35,110,35,15,,,32,18,71,008,45,21,44,208,37*4F
08:36:18  $GPGSV,2,2,08,22,40,326,27,24,39,035,38,25,30,153,34,27,,,18*4E
08:36:18  $GPGLL,2233.99086,N,11402.66757,E,083618.00,A,A*60
*/
/*******************************************************************************
* Function Name  : USART2_IRQHandler  <--> GPS
* Description    : This function handles USART2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static unsigned char	state_gps = 0;
static unsigned char	input_gps = 0;

static BOOL				found_rmc = FALSE;	// RMC消息头检测标志
static BOOL				found_gsa = FALSE;	// GGA消息头检测标志

static int 				cnt_msg_rmc = 0;
static int 				cnt_msg_gsa = 0;

static int				commas_rmc = 0;	

// 2012-03-05 特征字符串检测状态机模拟测试过。
void USART2_IRQHandler(void)
{								   	
	char			ch;			
	
	// usart2接收中断(每次接收一个字节)
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
 	{		
		// 保存当前接收到的数据 
		ch = USART_ReceiveData(USART2) & 0xFF;
	
		// 将当前接收到的数据写入GPS接收缓冲
		GPS_RX_WR(RxCnt2_wr++, ch);

		// printf("%c", ch);

		/************************ 检测GPS消息头 ********************************/		
		
		// 取得当前输入字符在gps_ptn_map表中的索引号(若为非特征字符，则无索引号)
		if(ch == '$')
		{
			input_gps = 0;
			
			state_gps = ptn_map_gps[state_gps][input_gps];	// '$'的索引号为0
		}
		else if(ch >= 'A' && ch <= 'S')							// 仅检测'A' - 'S'之间的字符
		{
			// 检查获得的索引号是否有意义(索引号为0的则无意义，即非特征字符)
			input_gps = ptn_idx_gps[ch - 'A'];
			
			if(input_gps > 0)
			{						
				state_gps = ptn_map_gps[state_gps][input_gps];
			}
			else // input_gps == 0
			{
				state_gps = 0;								// 状态复位	
			}
		}
		else
		{
			state_gps = 0;									// 状态复位
		}

		// 检查当前状态是否为终态
		if(state_gps > (MAX_STATE_NUM_GPS-1))
		{
			switch(state_gps)
			{
				case 8:			// $GPRMC检测到(每组NMEA消息中都会有一条RMC消息、RMC消息头被检测到的频率是1Hz)
					// 仅在上一条接收到的rmc消息被处理完后才接收新的rmc消息，以实现接收和处理的同步
					if(is_rmc_received == FALSE)
					{
						found_rmc 		= TRUE;		
						cnt_msg_rmc 	= 0;
						
						commas_rmc 	= 0;
					}
					
					break;
				case 9:			// $GPGSA检测到(每组NMEA消息中都会有一条GSA消息、GSA消息头被检测到的频率是1Hz)
					// 仅在上一条接收到的gsa消息被处理完后才接收新的gsa消息，以实现接收和处理的同步
					if(is_gsa_received == FALSE)
					{
						found_gsa 		= TRUE;							
						cnt_msg_gsa 	= 0;
					}

					break;
				default:
					break;
			}

			state_gps = 0;							// 状态机进入终态的话即刻复位
		}

		/************************ 提取并部分解析RMC/GSA消息 ********************************/	
		// 可采用状态机实现消息的接收，从而回避使用多个状态变量。

		// 检测到rmc消息头后准备接收rmc消息(从$GPRMC的C字符开始接收、直至换行符号)
		if(found_rmc == TRUE)
		{	
			// 仅在上一条rmc消息被处理后才开始接收新的rmc消息
			if(is_rmc_received == FALSE)
			{
				gps_msg_rmc[cnt_msg_rmc++] = ch;

				// 检测到换行符或提取的字符数超过gps_msg_rmc数组的最大长度，则停止提取
				if(((GPS_RX_RD(RxCnt2_wr-2) == '\r') && (GPS_RX_RD(RxCnt2_wr-1) == '\n')) || (cnt_msg_rmc >= MAX_LEN_GPS_RMC))
				{
					// 添加字符串结尾符号(在换行符之后)
					gps_msg_rmc[cnt_msg_rmc++] = '\0';	

					// 提取完一条RMC消息后，将RMC检测标志复位，以便重新提取下一条RMC消息
					found_rmc 		= FALSE;
					cnt_msg_rmc 	= 0;

					// 如果当前GPS已经定位，则设置RMC消息提取完成标志(之前提取的RMC消息由于是在GPS未定位时提取的，因此统统忽略)
					if(is_gps_fixed == TRUE)
					{
						is_rmc_received = TRUE;

						// 接收完一条rmc消息后，解析此消息需要时间，因此可能造成上次接收完但仍在处理的rmc消息被新的接收过程覆盖，这里需要同步
					}
				}	
			}			

			// 检测到RMC消息头后，紧接着检测定位状态
			if(ch == ',')
			{
				commas_rmc++;

				// 检测到GPRMC消息头后的第三个逗号，则提取此逗号之前的一个字符(即定位状态标志: A - 已定位，V - 未定位)
				if(commas_rmc == 3)
				{	
					if(GPS_RX_RD(RxCnt2_wr-2) == 'A')
					{			
						// 设置GPS定位标志
						is_gps_fixed = TRUE;
					}
					else
					{
						is_gps_fixed = FALSE;
					}
					// else: exception...
				}
			}	
		}

		// 不管当前是否定位， 在检测到GSA消息头后都逐个提取此消息(从GSA的末尾字符A开始提取)
		if(found_gsa == TRUE)
		{			
			// 从GSA消息头之后的第一个逗号(包含逗号本身)开始提取GSA消息
			if(is_gsa_received == FALSE)
			{
				gps_msg_gsa[cnt_msg_gsa++] = ch;

				if((GPS_RX_RD(RxCnt2_wr-2) == '\r' && GPS_RX_RD(RxCnt2_wr-1) == '\n') || (cnt_msg_gsa >= MAX_LEN_GPS_GSA))
				{
					// 添加字符串结尾符号(在换行符之后)
					gps_msg_gsa[cnt_msg_gsa++] = '\0';	

					// 提取完一条GSA消息后，将GSA检测标志复位，以便重新提取下一条GSA消息
					found_gsa = FALSE;
					cnt_msg_gsa = 0;
					
					// 如果当前RMC消息已经被提取完，则设置GSA消息提取完成标志(在一组NMEA消息中RMC消息最靠前)
					if(is_rmc_received == TRUE)
					{
						is_gsa_received = TRUE;						
					}
				}	
			}
		}

		#if 0
		if(USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET) 
		{  
			USART_ClearFlag(USART2, USART_FLAG_ORE);
		} 
		#endif

   		/* Clear the USART2 Receive interrupt */
    	USART_ClearITPendingBit(USART2, USART_IT_RXNE);
 	}

  	// handle USART2 Tx interrupt
	if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
	{ 		
		if(TxCnt2_rd < TxCnt2_wr)
		{
			/* Write one byte to the transmit data register */
		   	USART_SendData(USART2, TxBuf2[TxCnt2_rd++ & (TxBuf2Size-1)]);                    
			
		   	/* Clear the USART2 transmit interrupt */
		   	USART_ClearITPendingBit(USART2, USART_IT_TXE);			
		}
		// disable USART2 Tx interrupt once all data in TxBuf3 have been sent out.
		// or Tx interrupt will happen always, which will prevent normal procrss getting chance to run.
		else 	
		{
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
		}
	}  
}

/*******************************************************************************
* Function Name  : USART3_IRQHandler <--> GSM
* Description    : This function handles USART3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static int	state_gsm = 0;	// 特征字符串检测状态机的当前状态
static int	input_gsm = 0;	// 特征字符串检测状态机的当前输入字符

static BOOL	found_smsflag 	= FALSE;	// 是否检测到SMSFLAG特征字符串的标志(模块接收到短信时输出)
static BOOL	found_cmgr 		= FALSE;	// 是否检测到CMGR特征字符串的标志(发送读短信命令后模块输出)
static BOOL	found_clip 		= FALSE;	// 初次检测到振铃事件后就应及时执行来电号码检测的标志
static BOOL	found_key 		= FALSE;	// 通话期间检测到的DTMF信号的标志
static BOOL	found_release 	= FALSE;	// 通话挂断的标志
static BOOL	found_tcprecv	= FALSE;	// TCP数据包接收标识


// 和tcp命令相关
// static BOOL		to_recv_data = FALSE;
// static int		tcp_conn_id  = 0;
// static int		tcp_comma_cnt = 0;

static int	cnt_pnin = 0;	// pnin字符串的游标
static int	cnt_dtmf = 0;	// dtmf字符串的游标
static int	cnt_sms  = 0;	// cmt字符串的游标

static int	offset_dtmf 	= 0; // 

	   int 	rings  = 0;		// 当前振铃次数	
static int 	clips  = 0;		// 当前CLIP消息次数	
static int	quotes = 0;		// 检测到引号的次数
static int	enters = 0;		// 检测到换行符号的次数

static BOOL	flg_fetch_pnin = FALSE;	// 是否开始提取来电号码
static BOOL	flg_fetch_dtmf = FALSE;	// 是否开始提取DTMF字符
static BOOL	flg_fetch_cmgr  = FALSE;	// 是否开始提取CMT字符
static BOOL	flg_fetch_smsflag  = FALSE;	// 开始提取短信索引号的标志

#define MAX_LEN_GSM_SMSFLAG		32

static char buf_smsflag[MAX_LEN_GSM_SMSFLAG+1];
static int	cnt_smsflag = 0;	// cmt字符串的游标

void USART3_IRQHandler(void)
{  
#ifdef USING_DEV_GSM
	unsigned char	byte = 0x00;
	char			at[32];

	// char			string[16];

	int				len_smsflag = 0;

	int				i = 0;

	// unsigned char* 	ptr = NULL;

	// handle USART3 Rx interrupt
 	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
 	{				
    	/* Read the latest byte from the receive data register */
		byte = USART_ReceiveData(USART3) & 0xFF;

		// printf("usart3 byte = %d\r\n", byte);

		// 将接收到的1个字节数据写入GSM接收缓冲
		GSM_RX_WR(RxCnt3_wr++, byte);			

		/*************************************************
		 1, 检测特征字符串
		**************************************************/

		// 修正输入字符为状态迁移表的索引值
		if(byte >= 'A' && byte <= 'Y')	// 感兴趣的字符
		{
			input_gsm = ptn_idx_gsm[byte - 'A'];

			if(input_gsm != 99)
			{
				state_gsm = ptn_map_gsm[state_gsm][input_gsm];	
			}
			else
			{
				state_gsm = 0;
			}
		}
		else							// 不感兴趣的字符
		{				
			// 状态机复位
			state_gsm = 0;	
		}

		// 数值大于MAX_STATE_NUM_GSM的状态为终态
		if(state_gsm > (MAX_STATE_NUM_GSM))
		{
			switch(state_gsm)
			{
				case 30:  	// CLIP，来电号码显示
					found_clip = TRUE;		

					// 可出现连续多个CLIP消息
					clips++;		

					break;
				case 31:	// SMSFLAG，短消息接收指示
					found_smsflag = TRUE;
					
					break;
				case 32:	// KEY，dtmf按键检测
					found_key = TRUE;

					break;
				case 33:	// RELEASE，通话挂断
					found_release = TRUE;		
					
					break;
				case 34:  	// RING，来电振铃
					// 可出现连续多个RING消息	
					rings++;		

					printf("#%d ringing...\r\n", rings);

					// 在电话挂断前检测到4次振铃消息，则自动接听并打开DTMF检测
					if(rings >= DEF_TIMES_RING_TO_ANSWER_CALL)
					{
						// printf("rings = %d\r\n", rings);						

						if(sts_dtmf_command == STS_DTMF_COMMAND_CALL_WAITING)
						{
							sts_dtmf_command = STS_DTMF_COMMAND_CALL_INCOMING;
							
							times_input_dtmf_password = 0;
							
							// 自动接听(使用轮询方式发送自动接听命令)
							strcpy(at, "ATA\r\n");
							usart3_send_poll(at, strlen(at));		

							// 设置gsm通话状态标志
							is_gsm_calling = TRUE;

							// 发送ATA命令后应适当延迟(最少延迟500ms，否则下面的AT+CMUT命令发送后无法生效)
							delay_100ms(5);		

							sts_dtmf_command = STS_DTMF_COMMAND_CALL_ACCEPTED;

							#if 1
							// 马上设置静音，以免被未授权号码监听
							strcpy(at, "AT+CMUT=1\r\n");
							usart3_send_poll(at, strlen(at));	

							delay_100ms(3);
							#endif
							
							sts_dtmf_command = STS_DTMF_COMMAND_MIC_MUTE_ON;

							// 来电接听后启动dtmf输入超时检测定时器
							swtimer_input_dtmf = TO_DTMF_INPUT_PASSWORD*1000/SYSTICK_PERIOD;
						}

						// 再检查是否需要打开DTMF检测(GSM启动后一旦打开DTMF检测则一直维持检测状态，直至GSM关机)
						if(is_dtmf_detection_enabled == FALSE)
						{
							// 打开DTMF检测(使用轮询方式发送)
							strcpy(at, "AT+DTMFDETECT=1\r\n");
							usart3_send_poll(at, strlen(at));	

							// printf("send AT+DTMFDETECT=1 to GSM by polling.\r\n");

							// 一旦打开DTMF检测，此后无需再打开，除非模块重启
							is_dtmf_detection_enabled = TRUE;
						}	
					}

					break;
				case 35:	// CMGR，读取保存的短信
					found_cmgr = TRUE;

					break;
				case 36:	// TCPRECV，接收到tcp数据包
					found_tcprecv = TRUE;		

					found_tcprecv = found_tcprecv;		// 避免编译警告
			
					break;
				default:
					break;			
			}

			state_gsm = 0;		// 状态机复位
		}

		/*************************************************
		 2, 若检测到特征字符串，做相应的处理
		**************************************************/

		/*
			RING

			+CLIP: "13923887347",129,"",0,"",0

			RING

			+CLIP: "13923887347",129,"",0,"",0

			RING

			+CLIP: "13923887347",129,"",0,"",0

			DISCONNECT

			RELEASE
		*/
		
		// 检测到CLIP消息时，及时提取来电号码信息(第一次就提取，随后不再提取)
		if(found_clip == TRUE)
		{
			// 检测到第一个CLIP消息时就开始提取来电号码
			if(clips == 1)
			{
				// 来电号码信息包含在引号内
				if(byte == '"')	  
				{
					quotes++;

					// 检测到第一个引号时开始拷贝来电号码字符串的字符
					if(quotes == 1)
					{
						// 开始提取来电号码信息
						flg_fetch_pnin = TRUE;
					}
					else if(quotes == 2)
					{
FINISH_RECEIVING_GSM_PNIN:						
						// 给来电号码字符数组添加字符串结尾符号
						de_gsm_pnin[cnt_pnin++] = '\0';		// 如果来电号码为空，则de_gsm_pnin字符串数组中不会有字符填入					

						// 将来电号码格式化
						if(format_pn((char*)de_gsm_pnin) != OK)
						{
							// 将来电号码字符数组初始化为空字符串
							strcpy(de_gsm_pnin, "");
						}
						else
						{						
							strcpy(de_gsm_pnin, de_gsm_pnin);
						}
						
						printf("de_gsm_pnin = %s\r\n", de_gsm_pnin);

						// 引号计数器清零
						quotes = 0;					

						// PNIN字符串游标复位
						cnt_pnin = 0;

						// CLIP检测标志复位
						found_clip = FALSE;

						// 停止提取来电号码信息
						flg_fetch_pnin = FALSE;
					}
				}	
				else
				{
					if(flg_fetch_pnin == TRUE)
					{							
						// 逐个数字提取来电号码信息
						de_gsm_pnin[cnt_pnin++] = byte;

						// 检查提取的来电号码字符串是否越界
						if(cnt_pnin >= MAX_LEN_PN)
						{
							goto FINISH_RECEIVING_GSM_PNIN;
						}
					}
				}
			}
			else
			{
				// 忽略第一个之后的CLIP消息
				found_clip = FALSE;	
			}
		}

		/*
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
		// 检测到KEY消息时，提取KEY字符
		else if(found_key == TRUE)
		{
			// printf("KEY found.\r\n");
			
			// 检测到冒号时准备提取按键编号值
			if(byte == ':')
			{
				flg_fetch_dtmf = TRUE;
			}
			else
			{
				if(flg_fetch_dtmf == TRUE)
				{
					offset_dtmf++;

					// 冒号后的第二个字符为按键编码值
					if(offset_dtmf == 2)
					{
						de_gsm_dtmf[cnt_dtmf++] = byte;	

						// printf("%c", byte);

						// 偏移量计数器清零
						offset_dtmf = 0;

						// DTMF提取标志复位
						flg_fetch_dtmf = FALSE;

						// 一个DTMF字符提取完后，将DTMF检测标志复位
						found_key = FALSE;

						// 检查按键编码值是否为#或者DTMF字符串长度越界
						if(byte == '#' || (cnt_dtmf >= MAX_LEN_GSM_DTMF))
						{
							// DTMF字符数组添加字符串结尾符号
							de_gsm_dtmf[cnt_dtmf++] = '\0';

							printf("DTMF string:%s\r\n", de_gsm_dtmf);
							
							ana_str_dtmf((char*)de_gsm_dtmf);

							// DTMF字符串游标复位
							cnt_dtmf = 0;
						}
					}
				}
			}
		}

		// 检测到电话挂断消息(RING、CLIP以及DTMF消息都只会在RELEASE消息出现之前出现)
		else if(found_release == TRUE)
		{
			// 重置gsm通话状态标志
			is_gsm_calling = FALSE;
			
			printf("rigns = %d\r\n", rings);

			// 对方挂断电话后，dtmf命令处理恢复初始状态(己方自动挂断电话后也会将dtmf命令处理恢复初始状态)
			sts_dtmf_command = STS_DTMF_COMMAND_CALL_WAITING;

			// 清空来电显示号码字符串
			strcpy(de_gsm_pnin, "");
 			
			if(rings > 0)		// incoming call
			{				
				ana_num_ring(rings);

				// RING计数器清零
				rings = 0;

				// CLIP计数器清零
				clips = 0;		// CLIP消息和RING消息伴生伴灭

				// 挂断电话(而非接收完一条DTMF命令字)将is_gsmring_resolved设置为无效
				is_gsmring_pending = FALSE;
			}
			else	// outgoing call
			{
				// TBD
			}

			found_release = FALSE;

			// 将GSM模块休眠(将gsm_sleep()函数展开，以回避中断处理程序中不能已中断方式发送AT命令的限制)
			strcpy(at, "AT+ENPWRSAVE=1\r\n");
			usart3_send_poll(at, strlen(at));

			// 拉低DTR引脚
			MCU_GPIO_LOW(GPIO_GSM_DTR);   // GSM DTR	
		}

		/************************************************************************************  
		V015以前的版本，设置短信自动输出时，消息如下。

		Text模式下输出的消息:
			+CMT: "8613923887347", ,"2012/04/13 17:40:02+32"
			test!			// Unicode Big Endian编码为: 0074 0065 0073 0074 0021 

			+CMT: "8613923887347", ,"2012/04/13 17:44:43+32"
			4F60597DFF01	// 中文: 你好！(以Unicode Big Endian模式输出，每个汉字占用2字节)

			+CMT: "8613923887347", ,"2012/04/13 17:47:14+32"
			007400654F60597D00730074FF01	// 中文: te你好st！(以Unicode Big Endian模式输出，每个汉字占用2字节)

		PDU 模式下自动输出的接收到短信息: 
		+CMT: ,28
		0891683110200005F0240BA13134202377F00000216052229431230A4967B4057D4E59B111			// INQ-POS,1#
		 
		+CMT: ,29
		0891683110200005F0240BA13134202377F00008216052229463230A54084F5C61095FEBFF01		// 合作愉快！
		-------------------------------------------------------------------------------------	
		V015开始的版本，禁止短信自动输出，而自动输出如下消息:

		+SMSFLAG: "SM", 27		// 2012-12-32, M660+ V015 FW		
		*************************************************************************************/
		// M660+接收到短信时会先拉低RING引脚，延时~600ms后再自动输出+SMSFLAG消息，外部程序检测到
		// SMSFLAG消息后先提取其中的短信编号，再发送AT+CMGR命令读取最新接收到的短信，最后再检测CMGR消息。
		// 注: 通话过程中有短信发来也能正常接收并提取、分析。
		else if(found_smsflag == TRUE)
		{	
			if(flg_fetch_smsflag == FALSE)
			{
				if(byte == ':')
				{
					flg_fetch_smsflag= TRUE;		// 指示可以开始提取SMSFLAG消息

					strcpy(buf_smsflag, "");		// 初始化buf_smsflag
					cnt_smsflag = 0;

					return;
				}
			}
			else				
			{
				// 逐个字节接收SMSFLAG消息
				buf_smsflag[cnt_smsflag++] = byte;

				// 接收的字符数量超过smsflag的最大长度，则停止接收
				if(cnt_smsflag >= MAX_LEN_GSM_SMSFLAG)
				{
					goto FINISH_RECEIVING_SMSFLAG;
				}

				// 接收到换行符时也停止接收
				if((GSM_RX_RD(RxCnt3_wr-2) == '\r' && GSM_RX_RD(RxCnt3_wr-1) == '\n'))
				{
					enters++;

					// 检测到第一个换行符时，SMSFLAG消息接收完毕
					if(enters == 1)
					{			
FINISH_RECEIVING_SMSFLAG:						
						// 添加字符串结尾符号
						buf_smsflag[cnt_smsflag++] = '\0';	

						// 定位到SMSFLAG消息中的短信编号起始字符处			
						len_smsflag = strlen(buf_smsflag);

						for(i = 0; i < len_smsflag; i++)
						{
							if(buf_smsflag[i] == ',')
							{	
								break;
							}
						}

						// 跳过逗号
						i++;

						// 仅在定位到逗号时才进一步发送读取短信的命令
						if(i < cnt_smsflag)
						{
							// 以轮询方式发送AT+CMGR读短信命令
							sprintf(at, "AT+CMGR=%s\r\n", (char*)buf_smsflag+i);
							
							usart3_send_poll(at, strlen(at));
						}
						// else: 不发送读取短信的命令，终端接下去也不会接收到输出的短信

						// SMSFLAG消息提取相关的计数器清零
						cnt_smsflag 	= 0;								 
						enters 			= 0;
						quotes 			= 0;

						// 清除SMSFLAG消息提取标志
						flg_fetch_smsflag = FALSE;

						// 复位SMSFLAG特征字符串检测标志
						found_smsflag 	  = FALSE;
						
						return;
						
						// 读短信命令发送后，MCU随即应该会检测到+CMGR消息......
					}				  			
				}	
			}
		}

		// 检测到SMSFLAG后随即发送AT+CMGR命令读取刚接收到的短信，然后程序会检测到CMGR特征字符串
		else if(found_cmgr == TRUE)
		{				
			if(flg_fetch_cmgr == FALSE)
			{
				if(byte == ':')
				{	
					if((cnt_sms_cmd_wr - cnt_sms_cmd_rd) < MAX_NUM_SMS_RX)	
					{
						flg_fetch_cmgr = TRUE;
						
						strcpy((char*)que_sms_cmd[cnt_sms_cmd_wr & (MAX_NUM_SMS_RX-1)], "");
						cnt_sms = 0;
					}
					else
					{
						// 命令接收缓冲若满，则放弃接收新的命令
						found_cmgr = FALSE;
						
						printf("sms queue is full!!!\r\n");
					}

					return;
				}
			}
			else
			{					
				// 将接收的CMGR消息保存到系统的SMS接收环形队列中，为了避免数据拷贝，直接在ISR里将数据逐字节写入队列中。
				que_sms_cmd[cnt_sms_cmd_wr&(MAX_NUM_SMS_RX-1)][cnt_sms++] = byte;

				// 接收到的命令长度超过命令允许的最大长度时，停止接收命令，2013-05-21 added
				if(cnt_sms > MAX_LEN_CMD)
				{
					goto FINISH_RECEIVING_CMGR;
				}
				
				// 检测到第二个换行符时，也停止接收命令
				if(GSM_RX_RD(RxCnt3_wr-2) == '\r' && GSM_RX_RD(RxCnt3_wr-1) == '\n')
				{
					enters++;

					if(enters == 2)
					{	
FINISH_RECEIVING_CMGR:						
						// 添加字符串结尾符号
						que_sms_cmd[cnt_sms_cmd_wr&(MAX_NUM_SMS_RX-1)][cnt_sms++] = '\0';

						// 更新SMS接收环形队列写游标
						cnt_sms_cmd_wr++;

						printf("cnt_sms_cmd_wr = %d\r\n", cnt_sms_cmd_wr);

						// 接收完短信后，清除GSM模块RING中断事件待处理标志
						is_gsmring_pending = FALSE;

						// CMT消息提取相关的计数器清零
						cnt_sms	= 0;								 
						enters 	= 0;
						quotes 	= 0;

						// 清除CMGR消息提取标志
						flg_fetch_cmgr 	= FALSE;

						// 复位CMGR特征字符串检测标志
						found_cmgr 		= FALSE;

						// 删除当前接收到的短信，以免存储器中短信满(上限为50条)
						sprintf(at, "AT+CMGD=%s\r\n", (char*)buf_smsflag+cnt_smsflag);
						usart3_send_poll(at, strlen(at));	

						delay_100ms(1);

						// 让GSM模块休眠
						strcpy(at, "AT+ENPWRSAVE=1\r\n");
						usart3_send_poll(at, strlen(at));

						MCU_GPIO_LOW(GPIO_GSM_DTR);
					}	
				}
			}
		}		
#endif

	/* Clear the USART3 Receive interrupt */
    	USART_ClearITPendingBit(USART3, USART_IT_RXNE);	
	}
	
  	// handle USART3 Tx interrupt
  	if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
  	{ 
  		if(TxCnt3_rd < TxCnt3_wr)
  		{
	 		/* Write one byte to the transmit data register */
	    	USART_SendData(USART3, TxBuf3[TxCnt3_rd++ & (TxBuf3Size-1)]);                    
		
	    	/* Clear the USART3 transmit interrupt */
	    	USART_ClearITPendingBit(USART3, USART_IT_TXE); 	  
  		}
		// disable USART3 Tx interrupt once all data in TxBuf3 have been sent out.
		// or Tx interrupt will happen always, which will prevent normal procrss getting chance to run.
		else 	
		{
			USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
		}
	}
	
	// 溢出-如果发生溢出需要先读SR,再读DR寄存器则可清除不断入中断的问题[牛人说要这样]
   	if(USART_GetFlagStatus(USART3,USART_FLAG_ORE)==SET)
    {
    	USART_ClearFlag(USART3,USART_FLAG_ORE); //读SR其实就是清除标志

		USART_ReceiveData(USART3); 				//读DR        
   	}  
}

// 上次读取的加速度传感器三轴加速度数值(以用于和当次读取的三轴加速度数值比较从而判断是否发生运动)
static short 	regx_pre = 0;
static short 	regy_pre = 0;
static short 	regz_pre = 0;

static short	regx;
static short	regy;
static short	regz;

extern unsigned short swtimer_is_mcu_idle;
		
// RTC 闹钟中断。
void RTCAlarm_IRQHandler(void)
{
	// 清EXTI_Line17挂起位
	EXTI_ClearITPendingBit(EXTI_Line17);
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval : None
  */
void SysTick_Handler(void)
{	
	char	at[32];
	
	systick++;	

#if 0
	if(systick&1)
	{
		led_switch(GPIO_LED_R, ON);
	}
	else
	{
		led_switch(GPIO_LED_R, OFF);
	}
#endif

	// 如果系统采用systick而非rtcalarm中断作为周期tick中断，由于主循环中有不时喂狗的操作(其他执行时间较长的过程中也有喂狗操作)，因此无需在systick中断处理函数里面喂狗，
	// 这样可以避免优先级比systick中断更低的中断处理函数中的死循环不能触发看门狗溢出操作。
	// 如果采用rtcalarm中断作为系统的周期tick中断且mcu要进入stop模式，那么mcu一旦进入srop模式，主循环中无法喂狗，因此必须在rtcalarm中断处理程序中定期喂狗，以免看门狗溢出。
	if(is_sys_idle == TRUE)
	{
		
	}

	/*************************************** 检测dtmf命令输入是否超时 *************************************************/
	if(swtimer_input_dtmf > 0)
	{
		swtimer_input_dtmf--;

		if(swtimer_input_dtmf == 0)
		{
			strcpy(at, "ATH\n");
			usart3_send_poll(at, strlen(at));	

			// 重置gsm通话状态标志
			is_gsm_calling = FALSE;

			sts_dtmf_command = STS_DTMF_COMMAND_CALL_WAITING;
		}
	}

	// 用户打电话监听时，输入dtmf密码正确后关闭咪头静音的延迟定时器溢出
	if(swtimer_set_micmute_off > 0)
	{
		swtimer_set_micmute_off--;

		if(swtimer_set_micmute_off == 0)
		{
			// 关闭咪头静音并设置咪头音量最大以方便监听
			if(sts_dtmf_command == STS_DTMF_COMMAND_AUTHORIZED)
			{
				// printf("to turn off mic mute.\r\n");
				
				strcpy(at, "AT+CMUT=0\r\n");
				usart3_send_poll(at, strlen(at));	

				strcpy(at, "AT+MICL=6\r\n");
				usart3_send_poll(at, strlen(at));	

				sts_dtmf_command = STS_DTMF_COMMAND_MIC_MUTE_OFF;
			}
		}
	}

	// 定时定位timer
	if(swtimer_periodic_pos > 0)
	{
		swtimer_periodic_pos--;

		if(swtimer_periodic_pos == 0)
		{
			// 周期定位请求计数器
			sem_periodic_pos++;
		
			// 仅在周期定位模式打开时重启周期定位timer。	
			if(sw_periodic_pos == ON)
			{
				// 重新启动定时定位timer
				swtimer_periodic_pos = cycle_periodic_pos*1000/SYSTICK_PERIOD;
			}
		}	
	}
}

static int cnt_key_triple = 0;		// 三击的按键次数
static int cnt_key_double = 0;		// 双击的按键次数
static int cnt_key_single = 0;		// 单击的按键次数

/* I/O线中断，中断线为PA0 */
void EXTI0_IRQHandler(void)
{
	u16 ret;
	
	if(EXTI_GetITStatus(EXTI_Line0) != RESET) 			// 确保是否产生了EXTI Line中断
  	{				
 		ret = key_scan();	

		printf("ret of key_scan() = %d\r\n", ret);
		
		switch(ret)
		{
			// 单机和双击分支增加对GPS和GSM初始化状态的检测，以略过长按开机时按键抖动误触发单击或双击事件
			case 1:	 
				cnt_key_single++;			
				
				break;
			case 2:			// SOS
				cnt_key_double++;

				// dump_stack();

				break;				
			case 3:	 //三击

				cnt_key_triple++;

				/*
				printf("GPIO_LED_B = %d, sw1 = %d\r\n", last_led_status[GPIO_LED_B-GPIO_LED_B].led, last_led_status[GPIO_LED_B-GPIO_LED_B].sw);
				printf("GPIO_LED_G = %d, sw1 = %d\r\n", last_led_status[GPIO_LED_G-GPIO_LED_B].led, last_led_status[GPIO_LED_G-GPIO_LED_B].sw);
				printf("GPIO_LED_R = %d, sw1 = %d\r\n", last_led_status[GPIO_LED_R-GPIO_LED_B].led, last_led_status[GPIO_LED_R-GPIO_LED_B].sw);
				*/
				
				if(cnt_key_triple & 1)
				{
					// 将系统设置为黑暗模式
					is_sys_in_darkmode = TRUE;		
					
					// 直接调用GPIO控制函数熄灭三种颜色的led灯(调用led_switch函数熄灭的话会破坏last_led_status数组中保存的各个led灯在进入黑暗模式前的真实亮灭状态)
					MCU_GPIO_HIGH(GPIO_LED_B);
					MCU_GPIO_HIGH(GPIO_LED_G);
					MCU_GPIO_HIGH(GPIO_LED_R);
				}
				else
				{
					// 先恢复为系统正常模式
					is_sys_in_darkmode = FALSE;		
					
					// 再调用led_switch函数恢复各个led灯的亮灭状态
					led_switch(last_led_status[GPIO_LED_R-GPIO_LED_R].led, last_led_status[GPIO_LED_R-GPIO_LED_R].sw);
					led_switch(last_led_status[GPIO_LED_G-GPIO_LED_R].led, last_led_status[GPIO_LED_G-GPIO_LED_R].sw);
					led_switch(last_led_status[GPIO_LED_B-GPIO_LED_R].led, last_led_status[GPIO_LED_B-GPIO_LED_R].sw);
				}
				
				break;
			case 4:	 //长按		
				mcu_shutdown();	
				
				break;
			default:
				break;
		}
  	}

	EXTI_ClearITPendingBit(EXTI_Line0);     			// 清除中断标志位 
}

void EXTI3_IRQHandler(void)
{
	// PB3: GSM_RING
	if(EXTI_GetITStatus(EXTI_Line3) != RESET) 	
	{	
		isr_gsmring_asserted();
		
		EXTI_ClearITPendingBit(EXTI_Line3);		// 清除中断标志位  
	}
}

// PB4 不宜作为外部中断输入脚!!!

// 中断编号为9-5的外部中断的处理函数。
void EXTI9_5_IRQHandler(void) // EXTI Line 15..10
{
#if 0
	// PA6: CHG_STS
	if(EXTI_GetITStatus(EXTI_Line6) != RESET) 	
	{	
		isr_chgsts_asserted();
		
		EXTI_ClearITPendingBit(EXTI_Line6);		// 清除中断标志位  
	}
#endif

#ifdef USING_PWR_ACC
	#if (HW_VER == 17)	
	// EXTI_ACCPWR_DET
	if(EXTI_GetITStatus(EXTI_Line9) != RESET)
	{
		isr_accpwr_detected();
		
		EXTI_ClearITPendingBit(EXTI_Line9);     	//清除中断标志位
	}		

	#elif (HW_VER == 16)

	// EXTI_ACCPWR_DET
	if(EXTI_GetITStatus(EXTI_Line8) != RESET)
	{
		isr_accpwr_detected();
		
		EXTI_ClearITPendingBit(EXTI_Line8);     	//清除中断标志位
	}		
	#endif
#endif

#if 0
	// PA8: GSR_INT2
	if(EXTI_GetITStatus(EXTI_Line8) != RESET) 	
  	{			
		// isr_gsr_int2();
			
		EXTI_ClearITPendingBit(EXTI_Line8);		// 清除中断标志位  
	}
#endif
}

int cnt_int_acc = 0;

// void isr_gsr_int1(void);

// 中断编号为15-10的外部中断的处理函数。
void EXTI15_10_IRQHandler(void)
{
#if 1
	// PA11: GSR_INT1
	if(EXTI_GetITStatus(EXTI_Line11) != RESET) 				
  	{	
		// isr_gsr_int1();
		
		EXTI_ClearITPendingBit(EXTI_Line11);		// 清除中断标志位 
  	} 
#endif

#ifdef USING_PWR_EXT
	// EXTI_EXTPWR_DET
	if(EXTI_GetITStatus(EXTI_Line12) != RESET)
	{
		isr_extpwr_detected();
		
		EXTI_ClearITPendingBit(EXTI_Line12);     	//清除中断标志位
	}
#endif
}

//定时器2中断处理函数(防盗报警之静止检测延迟timer)。
void TIM2_IRQHandler(void)
{
	TIM2->SR&=~(1<<0);//清除中断标志位
}

// 软件看门狗定时器溢出中断处理函数。
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET )
  	{
  		TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);

		// 软件看门狗溢出时，软件复位MCU

		printf("Software Watchdog handler.\r\n");
		
		mcu_reset();
	}
	
	TIM3->SR&=~(1<<0);//清除中断标志位
}

//定时器4中断处理函数(周期定位timer)
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4 , TIM_IT_Update) != RESET )
  	{
  		TIM_ClearITPendingBit(TIM4 , TIM_FLAG_Update);

		usertimer_counter++;

		// printf("usertimer_counter = %d\r\n", usertimer_counter);
		
#ifdef USING_DEV_GSR
		// 每隔200*2=400ms测量加速度传感器三轴加速度数值
		if(usertimer_counter&1)
		{
			isr_gsrval_to_measure();
		}
#endif

		// 每隔512*200/1000=102秒(约1.7分钟)测量一次电池电压
		if(!(usertimer_counter & 511))
		{
			isr_batvol_to_measure();
		}
	}
	
	TIM4->SR&=~(1<<0);//清除中断标志位
}

#if 0
void isr_gsr_int1(void)
{
	unsigned char 		IntSourceMFF;
	unsigned char 	SysMod;

	IntSourceMFF = I2C_RegRead(REG_INT_SOURCE);
	
		#if 1	// 不使用加速度传感器内置的静止检测功能，而改由外部程序实现静止检测。
		// 读取加速度传感器的当前系统模式: standby, wake, sleep
		
		
		// ASLP/WAKEUP interrupt
		if(IntSourceMFF & 0x80)					
		{	
			if(cnt_gsensor_still & 1)
			{
				led_switch(GPIO_LED_R, ON);
			}
			else
			{
				led_switch(GPIO_LED_R, OFF);
			}
		
			// // printf("REG_INT_SOURCE & 0x20 = 0x%02x", I2C_RegRead(REG_INT_SOURCE) & 0x20);
			
			// 读系统模式寄存器查询当前所处模式
			SysMod=I2C_RegRead(REG_SYSMOD);				
			
			if(SysMod==0x02)					// 当前处于sleep模式，即wake mode --> sleep mode
			{					
				cnt_gsensor_still++;

				// 加速度传感器上电初始化后会自动产生一次静止检测中断，而不管运动检测是否打开、静止
				// 检测功能是否开启，因此需要忽略加速度传感器上电初始化后的第一个静止检测中断。
				if(cnt_gsensor_still > 1)
				{
					event_handler_still();
				}
			}
			else if(SysMod==0x01)				// 当前处于wake模式， 即sleep mode --> wake Mode
			{
				// TBD				
				printf("auto wakeup asserted.\r\n");
			}			
		}
		#endif

		#if 1
		// LNDPRT interrupt
		if(IntSourceMFF & 0x10)					
		{
			I2C_RegRead(REG_PL_STATUS);			// 读取Portrait/Landscape状态寄存器，清中断

			// event_handler_roll();
		}
		#endif

		#if 1
		// PULSE interrupt(singletap or doubletap)
		if(IntSourceMFF & 0x08)					
		{
			// 读取轻击中断源寄存器中的DPE位以判断产生中断的是单击还是双击事件，顺带清除清中断	
			if(I2C_RegRead(REG_PULSE_SRC)&0x08)	// double tap
			{					
				// event_handler_doubletap();	
			}
			else								// single tap
			{							
				// event_handler_singletap();
			}
		}
		#endif

		// data ready interrupt
		if(IntSourceMFF & 0x01)		
		{
#ifdef TEST_GSR_ACQUIRE_DATA
			gsensor_data[gsensor_data_wr & (GSENSOR_DATA_LENGTH-1)][0] = gsr_read_valx();
			gsensor_data[gsensor_data_wr & (GSENSOR_DATA_LENGTH-1)][1] = gsr_read_valy();
			gsensor_data[gsensor_data_wr & (GSENSOR_DATA_LENGTH-1)][2] = gsr_read_valz();

			gsensor_data_wr++;
#endif			
		}
}

void isr_gsr_int2(void)
{
	unsigned char 		IntSourceMFF, OAE;	
		
	cnt_gsensor_motion++;

		// 加速度传感器上电初始化后有时会自动产生一次运动检测中断，
		// 因此忽略加速度传感器上电后的第一个运动检测中断。
		if(cnt_gsensor_motion > 1)
		{
			// 检测到运动时将静止检测定时器复位
			swtimer_still_detection = DEF_SWTIMER_STILL_DETECTION;

			// 读取加速度传感器的中断源
			IntSourceMFF = I2C_RegRead(REG_INT_SOURCE);
	
			// TRANS interrupt
			if(IntSourceMFF & 0x20)						// 上限检测
			{	
				if(I2C_RegRead(REG_TRANSIENT_CFG)&0x01)	// HPF OFF				
				{				
					event_handler_motion();
				}
				else									// HPF ON 
				{
					// TBD
				}	

				// I2C_RegRead(REG_TRANSIENT_SRC);	// 读取震动中断源寄存器，清中断		
			}	
			else if(IntSourceMFF & 0x04)				// FF_MT interrupt，下限检测
			{
				// 读取当前的OAE位设置，以便判断检测对象是自由落体事件还是运动事件
				OAE = I2C_RegRead(REG_FF_MT_CFG);

				if(!(OAE & 0x40))			// Freefall
				{
					event_handler_motion();
				}
				else						// Motion
				{
					// TBD
				}

				// 读位移中断源寄存器以便清中断	
				// I2C_RegRead(REG_FF_MT_SRC);	
			}	
		}
}
#endif

#ifdef USING_PWR_EXT
// 外部主电源接通/断开检测中断。
void isr_extpwr_detected(void)
{	
	/**************************************** 断电报警相关 ******************************************/
	if(MCU_GPIO_READ(GPIO_EXTPWR_DET) == ON)	
	{
		sts_power |= (1<<BIT_STS_POWER_EXT);

		// 绿灯灭、蓝灯亮、指示外接电源接通。
		led_switch(GPIO_LED_G, OFF);
		led_switch(GPIO_LED_B, ON);

		printf("external power on.\r\n");
	}
	else
	{
		sts_power &= ~(1<<BIT_STS_POWER_EXT);

		// 绿灯亮、蓝灯灭，指示外接电源断开。
		led_switch(GPIO_LED_G, ON);
		led_switch(GPIO_LED_B, OFF);

		printf("exteral power off.\r\n");
	}
}
#endif

#ifdef USING_PWR_ACC
static unsigned int 	tm_acc_changed = 0;	// 上次acc电源改变(断开或接通)的时间戳

unsigned int			cnt_acc_changed = 0;

#define MAX_INTERVAL_ACC_CHANGED		10		// acc电源变化发生的最大时间间隔(由于acc断开时的电容放电缓慢，12V acc电压情况下，
												// 断开acc电源后一般需2-3秒程序才能检测到acc断开、16V acc电压情况下，断开acc电源后
												// 一般需3-4秒才会出发acc检测中断)

// 检查上次acc变化到当次acc变化是否在短时间内发生，是的话认为变化有效(一般在测试acc电源是否连接正确时通过快速开关acc电源产生)。
void check_acc_changed(void)
{
	// 开机后首次被动检测到acc变化时，只记录时间而不比较。
	if(tm_acc_changed == 0)
	{
		tm_acc_changed = systick;
	}
	else
	{
		// 检查上次acc变化到当前这次acc变化之间的时间间隔是否小于预设门限，是的话认为变化有效，并闪烁红灯。
		if((systick - tm_acc_changed) < MAX_INTERVAL_ACC_CHANGED*1000/SYSTICK_PERIOD)
		{
			// acc没变化一次，相关计数器递增，然后根据计数器奇偶性点亮或熄灭红灯，由此方便用户
			// 在测试acc接线是否正确的过程中通过观察红灯的亮灭变化来判断acc接线是否正确，同时用
			// 户也可通过来回产生acc变化将红灯熄灭，以免影响后续指示灯的变化。
			cnt_acc_changed++;

			if(cnt_acc_changed & 1)
			{
				led_switch(GPIO_LED_R, ON);
			}
			else
			{
				led_switch(GPIO_LED_R, OFF);
			}
		}

		// 更新tm_acc_changed
		tm_acc_changed = systick;
	}
}

// acc电源接通/断开检测中断。
// 注: 电源转接板上acc电源线上的分压电阻为120k时，12v acc输入情况下，acc检测引脚上的电压为3.12v左右、12v acc断开情况下，acc检测引脚上的电压为0.04v。
void isr_accpwr_detected(void)
{
	if(MCU_GPIO_READ(GPIO_ACCPWR_DET) == ON)	// ACC电源接通后产生
	{		
		sts_power |= (1<<BIT_STS_POWER_ACC);

		printf("acc power on.\r\n");
	}
	else										// ACC电源断开后
	{
		sts_power &= ~(1<<BIT_STS_POWER_ACC);

		printf("acc power off.\r\n");
	}

	// 检查acc连接/断开的变化是否在短时间内发生。
	check_acc_changed();
}
#endif

#ifdef USING_DEV_GSM
// 处理GSM_RING中断事件。
void isr_gsmring_asserted(void)
{
	// GSM模块RING中断处理函数内不能出现类似printf这样耗时的语句，否则会影响紧接着的GSM
	// 串口数据接收，即会出现数据部分丢失的问题(115200bps波特率下，一个字节的长度为70us)!!!

	cnt_gsmring_asserted++;

#if 1
	printf("cnt_gsmring_asserted = %d\r\n", cnt_gsmring_asserted);
#else
	if(cnt_gsmring_asserted & 1)
	{
		led_switch(GPIO_LED_R, ON);
	}
	else
	{
		led_switch(GPIO_LED_R, OFF);
	}
#endif

	// 仅GSM模块初始化后产生的GSM_RING中断才被处理
	if(is_gsm_ready == TRUE)
	{
		// 设置GSM模块RING中断事件待处理标志
		is_gsmring_pending = FALSE;
	}	
}
#endif

// 定期测量加速度传感器三轴的加速度数值。
// 终端正面朝上静止时的三轴加速度数值: x value=0.08, y value=0.10, z value=0.99
// 终端正面朝下静止时的三轴加速度数值: x value=0.08, y value=0.06, z value=-1.03
void isr_gsrval_to_measure(void)
{
	// printf("to measure G-Sensor 3-axis value when systick = %d.\r\n", systick);

	// printf("x value=%.2f, y value=%.2f, z value=%.2f\r\n", gsr_read_valx(), gsr_read_valy(), gsr_read_valz());
	
	// 检测运动事件
	if(sw_detect_motion == ON)
	{
		// printf("to detect motion event.\r\n");
		
		// 检查加速度传感器的三轴加速度数值变化是否超过门限
		regx = gsr_read_reg(AXIS_X);
		regy = gsr_read_reg(AXIS_Y);
		regz = gsr_read_reg(AXIS_Z);

		// printf("regx = %d, regy = %d, regz = %d\r\n", regx, regy, regz);
		
		if(!(regx_pre == 0 && regy_pre == 0 && regz_pre == 0))
		{
			// 三轴中有一轴的加速度数值变化超过门限就认为运动发生
			if((__abs16(regx-regx_pre) > err_detect_motion) || (__abs16(regy-regy_pre) > err_detect_motion) || (__abs16(regz-regz_pre) > err_detect_motion))
			{
				cnt_gsensor_motion++;

				// printf("cnt_gsensor_motion = %d\r\n", cnt_gsensor_motion);

				if(sw_detect_still == ON)
				{
					// 检测到运动时将静止检测定时器复位
					swtimer_still_detection = DEF_SWTIMER_STILL_DETECTION;
				}	

				event_handler_motion();	
			}
		}

		// 将当前三轴加速度数值保存为上次三轴加速度数值
		regx_pre = regx;
		regy_pre = regy;
		regz_pre = regz;
	}		

	// 检查静止事件
	if(sw_detect_still == ON)
	{				
		// printf("to detect still event.\r\n");
		
		// 静止检测定时器递减
		if(swtimer_still_detection > 0)
		{
			// printf("swtimer_still_detection = %d\r\n", swtimer_still_detection);

			// 每400ms检测一次三轴加速度数值
			swtimer_still_detection -= DEF_USERTIMER_PERIOD*2;

			// 若静止检测定时器溢出，则检测到静止
			if(swtimer_still_detection <= 0)
			{
				cnt_gsensor_still++;

				event_handler_still();
			}
		}
	}

}

// 定期测量电池的电压。
void isr_batvol_to_measure(void)
{
	de_batvol = batvol_measure();

	if(de_batvol < MAX_ADCVOL_FOR_BAT)
	{
		sts_power |= (1<<BIT_STS_POWER_BAT);
	}
	else
	{
		sts_power &= ~(1<<BIT_STS_POWER_BAT);
	}
	
	// 外部电源断开时检测电池是否电压过低
	// if(!(sts_power & (1<<BIT_STS_POWER_EXT)))
	if(1)
	{
		// 电池电压过低
		if(de_batvol < MIN_BATVOL_FOR_WORKING)
		{	
			
		} 
		// 电池电压处于临界状态
		else if(de_batvol < NML_BATVOL_FOR_WORKING)
		{

		}
		// 电池电压正常
		else
		{	
			
		}		
	}	
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval : None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
