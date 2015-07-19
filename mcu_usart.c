#include "mcu_usart.h"
#include "mcu_systick.h"
#include "mcu_i2c.h"
#include "mcu_usart.h"
#include "log_power.h"
#include "mcu_flash.h"
#include "mcu_gpio.h"
#include "mcu_timer.h"

#include "log_motion.h"
#include "log_pos.h"
#include "log_power.h"
#include "log_cmd.h"

#include "dev_mma845x.h"
#include "dev_gsm.h"
#include "dev_gps.h"


#include "stm32f10x_it.h"

#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// for USART1
unsigned char 			TxBuf1[TxBuf1Size];	   
unsigned char 			RxBuf1[RxBuf1Size];

volatile unsigned int 		TxCnt1_wr ; 
volatile unsigned int 		TxCnt1_rd ;
volatile unsigned int 		RxCnt1_wr ; 
volatile unsigned int 		RxCnt1_rd ;

// for USART2(包含接收和发送两个环形缓冲，用于和GPS通讯)
unsigned char 			TxBuf2[TxBuf2Size];	   
unsigned char 			RxBuf2[RxBuf2Size];

volatile unsigned int 		TxCnt2_wr ; 
volatile unsigned int 		TxCnt2_rd ;
volatile unsigned int 		RxCnt2_wr ; 
volatile unsigned int 		RxCnt2_rd ;

// for USART3(包含接收和发送两个环形缓冲，用于和GSM通讯)
unsigned char 			TxBuf3[TxBuf3Size];	   
unsigned char 			RxBuf3[RxBuf3Size];

volatile unsigned int 		TxCnt3_wr ; 
volatile unsigned int 		TxCnt3_rd ;
volatile unsigned int 		RxCnt3_wr ; 
volatile unsigned int 		RxCnt3_rd ;

BOOL					usart3_tx_busy 			= FALSE;		// 串口3发送是否使用中的标志

SWITCH					sw_gps_forward 		= OFF;			// GPS串口转发模式是否开启的标志
BOOL					is_gps_forwarding		= FALSE;
BOOL					is_gsm_forwarding 		= FALSE;		// GSM串口转发模式是否开启的标志
BOOL					is_gsm_downloading 	= FALSE;		// GSM串口下载模式是否开启的标志

/********************************************************************************************************************
*
*
*
*
*********************************************************************************************************************/
/*
 * 函数名：USART_Configure
 * 描述  ：USART1 GPIO 配置,工作模式配置。115200 8-N-1
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void USART_Configure(void)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	USART_InitTypeDef 	USART_InitStructure;
	
	/* config USART1, USART2, USART3 MCU_CLOCK_MODE */
	// RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB1Periph_USART3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

// USART1			
	GPIO_InitStructure.GPIO_Pin 			= GPIO_Pin_9;		// USART1 TX
	GPIO_InitStructure.GPIO_Mode 			= GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed 			= GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    
	
	GPIO_InitStructure.GPIO_Pin 			= GPIO_Pin_10;		// USART1 RX
	GPIO_InitStructure.GPIO_Mode 			= GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate 		= 115200;					// 为了支持GPS串口数据输出重定向到USART1，USART1波特率应大于9600
	USART_InitStructure.USART_WordLength 	= USART_WordLength_8b;
	USART_InitStructure.USART_StopBits 		= USART_StopBits_1;
	USART_InitStructure.USART_Parity 		= USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode 			= USART_Mode_Rx | USART_Mode_Tx;	
	USART_Init(USART1, &USART_InitStructure);

	// 使能串口1接收中断并打开串口1硬件
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	  				
	USART_Cmd(USART1, ENABLE);

	/* 如下语句解决第1个字节无法正确发送出去的问题 */
	USART_ClearFlag(USART1, USART_FLAG_TC);     // 清标志

// USART2	
	GPIO_InitStructure.GPIO_Pin 			= GPIO_Pin_2;		// USART2 TX
	GPIO_InitStructure.GPIO_Mode 			= GPIO_Mode_AF_PP;	
	GPIO_InitStructure.GPIO_Speed 			= GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin 			= GPIO_Pin_3;		// USART2 RX
	GPIO_InitStructure.GPIO_Mode 			= GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate 		= 9600;
	USART_InitStructure.USART_WordLength 	= USART_WordLength_8b;
	USART_InitStructure.USART_StopBits 		= USART_StopBits_1;
	USART_InitStructure.USART_Parity 		= USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode 			= USART_Mode_Rx | USART_Mode_Tx;	
	USART_Init(USART2, &USART_InitStructure);	

	// 使能串口2接收中断并打开串口2硬件
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 		
	USART_Cmd(USART2, ENABLE);

	/* 如下语句解决第1个字节无法正确发送出去的问题 */
	USART_ClearFlag(USART2, USART_FLAG_TC);     // 清标志

	// USART3	
	GPIO_InitStructure.GPIO_Pin 			= GPIO_Pin_10;		// UART3 TX
	// GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF_PP;
	//GPIO_InitStructure.GPIO_Mode 		 	= GPIO_Mode_AF_OD;
	// GPIO_InitStructure.GPIO_Speed 		= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 			= GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed 			= GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin 			= GPIO_Pin_11;		// UART3 RX
	GPIO_InitStructure.GPIO_Mode 			= GPIO_Mode_IN_FLOATING;
//	GPIO_InitStructure.GPIO_Speed 			= GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate 		= 115200;
	USART_InitStructure.USART_WordLength 	= USART_WordLength_8b;
	USART_InitStructure.USART_StopBits 		= USART_StopBits_1;
	USART_InitStructure.USART_Parity 		= USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode 			= USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	// 使能串口3接收中断并打开串口3硬件
  	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); 
	USART_Cmd(USART3, ENABLE);

	/* 如下语句解决第1个字节无法正确发送出去的问题 */
	USART_ClearFlag(USART3, USART_FLAG_TC);     // 清标志
}

/*
 * 函数名：fputc
 * 描述  ：重定向c库函数printf到USART1
 * 输入  ：无
 * 输出  ：无
 * 调用  ：由printf调用
 */
int fputc(int ch, FILE *f)
{
/* 将Printf内容发往串口 */
  USART_SendData(USART1, (unsigned char) ch);

  while (!(USART1->SR & USART_FLAG_TXE)); 

  return (ch);
}

/*******************************************************************************
	函数名：fgetc
	输  入:
	输  出:
	功能说明：
	重定义getc函数，这样可以使用scanf函数从串口1输入数据
*/
int fgetc(FILE *f)
{
	/* 等待串口1输入数据 */
	while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

	return USART_ReceiveData(USART1);
}

/**************************************************************************************************************/

// 将指定长度的数据填充到USART1发送缓冲（暂不发送）。
void usart1_tx_fill(char* data, unsigned int size)
{
	unsigned int cnt = 0;

	while(cnt < size)
	{
		TxBuf1[(TxCnt1_wr++) & (TxBuf1Size - 1)]	= data[cnt++];	
	}
}

// 将USART1发送缓冲中的数据发送出去。
void usart1_tx_start(void)
{
	unsigned int to = systick+2*1000/SYSTICK_PERIOD;
	
	/* Enable the USART Transmoit interrupt: this interrupt is generated when the 
  		USART1 transmit data register is empty */ 
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);	  				
	USART_Cmd(USART1, ENABLE);

	// 如果不等到数据发送完就退出，数据发送会出错
	while(TxCnt1_rd < TxCnt1_wr)
	{
		if(systick >= to)
		{
			return;
		}
	}
}

// 将指定长度的数据填充到USART2发送缓冲（暂不发送）。
void usart2_tx_fill(unsigned char* data, unsigned int size)
{
	unsigned int cnt = 0;

	while(cnt < size)
	{
		TxBuf2[(TxCnt2_wr++) & (TxBuf2Size - 1)]	= data[cnt++];	
	}
}

// 开始将USART2发送缓冲中的数据发送出去。
void usart2_tx_start(void)
{
	unsigned int to = systick+2*1000/SYSTICK_PERIOD;

	/* Enable the USART Transmoit interrupt: this interrupt is generated when the USART2 transmit data register is empty */  
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);	  				
	USART_Cmd(USART2, ENABLE);

	while(TxCnt2_rd < TxCnt2_wr)
	{
		if(systick >= to)
		{
			return;
		}
	}
}

// 以轮询的方式从USART发送一个字节。
void usart2_send_byte(unsigned char byte)
{
	unsigned int to = systick+400/SYSTICK_PERIOD;
	
	USART_SendData(USART2, (unsigned char) byte);
	
  	while (!(USART2->SR & USART_FLAG_TXE))
  	{
		if(systick >= to)
		{
			return;
		}
	}
}

// 开始将USART3发送缓冲中的数据发送出去。
void usart3_tx_start(void)
{
	unsigned int to = systick+2*1000/SYSTICK_PERIOD;

	/* Enable the USART Transmoit interrupt: this interrupt is generated when the 
  	 USART3 transmit data register is empty */  
	USART_ITConfig(USART3, USART_IT_TXE, ENABLE);	  				
	USART_Cmd(USART3, ENABLE);

	while(TxCnt3_rd < TxCnt3_wr)
	{
		if(systick >= to)
		{
			return;
		}
	}
}

// 使用中断方式直接从USART3口发送指定的数据(不可在中断处理程序中调用)。
void usart3_send_int(char* data, unsigned int size)
{
	unsigned int cnt = 0;

	// usart3发送超时取决于发送缓冲大小和串口波特率。
	unsigned int to = systick+3*1000/SYSTICK_PERIOD;

	// 检查当前USART3是否在发送数据(可能使用轮询方式在发送，如来电振铃时发送自动接听命令)。
	while(usart3_tx_busy == TRUE)
	{
		if(systick >= to)
		{
			return;
		}
	}

	while(cnt < size)
	{
		TxBuf3[(TxCnt3_wr++) & (TxBuf3Size - 1)]	= data[cnt++];	

		if(systick >= to)
		{
			return;
		}
	}

	USART_ITConfig(USART3, USART_IT_TXE, ENABLE);	  				
	USART_Cmd(USART3, ENABLE);
}

// 使用轮询方式直接从USART3口发送指定的数据(可在中断处理程序中调用)。
void usart3_send_poll(char* data, unsigned int size)
{
	unsigned int i;

	unsigned int to;

	usart3_tx_busy = TRUE;

	for(i = 0; i < size; i++)
	{
		to = systick+400/SYSTICK_PERIOD;

		USART_SendData(USART3, (unsigned char)data[i]);

	  	while (!(USART3->SR & USART_FLAG_TXE))
	  	{
			if(systick >= to)
			{
				return;
			}
		}
	}

	usart3_tx_busy = FALSE;
}


// 将USART2接收到的数据通过USART1发送出去并将USART1接收到的数据转发给USART2。
void usart2_redir_usart1(void)
{	
	unsigned char	byte;
	
	RxCnt2_rd = RxCnt2_wr;	
	RxCnt1_rd = RxCnt1_wr;

	// 关闭运动检测
	// detect_motion(OFF);
	// detect_still(OFF);

	printf("to redirect USART2 Rx to USART1 Tx...\r\n");

	is_gps_forwarding = TRUE;
	
	while(1)
	{
		// 将GPS UART Tx通过USART2 Rx转发给USART1 Tx
		if(RxCnt2_rd < RxCnt2_wr)
		{
			TxBuf1[(TxCnt1_wr++)&(TxBuf1Size-1)] = RxBuf2[(RxCnt2_rd++)&(RxBuf2Size-1)];

			// 由于GPS UART Tx输出数据位NMEA消息，因此采取逐行转发方式(换行符号"\r\n")
			if(RxBuf2[(RxCnt2_rd-2)&(RxBuf2Size-1)] == '\r' && RxBuf2[(RxCnt2_rd-1)&(RxBuf2Size-1)] == '\n')
			{				
				usart1_tx_start();
			}
		}	

		// 检查是否有以#号结尾的系统命令输入
		check_buf_com();	
		check_que_cmd(CHECK_MODE_CONTINUOUS);	

		// 检查系统事件
		sys_check_event();	

		WATCHDOG_RELOAD();
				
		// 将USART1 Rx通过USART2 Tx转发给GPS UART Rx
		if(RxCnt1_rd < RxCnt1_wr)
		{
			byte = RxBuf1[(RxCnt1_rd++)&(RxBuf1Size-1)];

			// 将USART1 Rx上接收到数据使用轮询方式通过USART2 Tx转发数据给GPS UART Rx
			usart2_send_byte(byte);
		}

		// 由于GPS转发会占用USART1的Tx和Rx，因此GPS串口转发时不检测USART1上的非GPS命令(如系统命令)，此时用户也不应在USART1上发送非GPS命令。
		// 用户如果要在USART1上发送系统命令，应先停止GPS串口转发(三击按键)，然后才能在USART1上发送系统命令,也能从USART1上看到系统反馈消息。

		// 检查是否双击按键触发停止GPS串口转发的事件
		if(sw_gps_forward == OFF)
		{
			// 检测到双击按键事件时断开GPS电源并退出GPS串口转发过程
			gps_power_down();	

			is_gps_forwarding = FALSE;

			// 重新打开运动和静止检测
			// detect_motion(ON);
			// detect_still(ON);

			printf("GPS forwarding terminated.\r\n");
			
			return;
		}	

		// GPS串口转发期间喂狗，以免看门狗溢出复位
		WATCHDOG_RELOAD();
	}
}

// 将USART3接收到的数据通过USART1转发出去并将USART1接收到的数据转发给USART3。
void usart3_redir_usart1(void)
{	
	is_gsm_forwarding = TRUE;
	
	// USART3 = 115200bps, USART1 = 19200bps
	while(1)
	{
		// Tx forward
		if(RxCnt3_rd < RxCnt3_wr)
		{
			TxBuf1[(TxCnt1_wr++)%(TxBuf1Size-1)] = RxBuf3[(RxCnt3_rd++)%(RxBuf3Size-1)];
			
			// 逐行转发
			if(RxBuf3[(RxCnt3_rd-2)%(RxBuf3Size-1)] == '\r' && RxBuf3[(RxCnt3_rd-1)%(RxBuf3Size-1)] == '\n')			
			{ 
				usart1_tx_start();
			}
		}	

		// 检查是否有以#号结尾的系统命令输入
		check_buf_com();
		check_que_cmd(CHECK_MODE_CONTINUOUS);	

		// Rx forward
		if(RxCnt1_rd < RxCnt1_wr)
		{
			if(COM1_RX_RD(RxCnt1_wr-1) == '\n' && COM1_RX_RD(RxCnt1_wr-2) == '\r' && COM1_RX_RD(RxCnt1_wr-3) != '#')
			{
				while(RxCnt1_rd < RxCnt1_wr)
				{
					TxBuf3[(TxCnt3_wr++)%(TxBuf3Size-1)] = RxBuf1[(RxCnt1_rd++)%(RxBuf1Size-1)];
				}

				usart3_tx_start();
			}		
		}

		// 检查GSM串口转发功能是否被关闭
		if(is_gsm_forwarding == FALSE)
		{
			return;
		}

		// GSM串口转发期间喂狗，以免看门狗溢出复位
		WATCHDOG_RELOAD();
	}	
}

