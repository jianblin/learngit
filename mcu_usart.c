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

// for USART2(�������պͷ����������λ��壬���ں�GPSͨѶ)
unsigned char 			TxBuf2[TxBuf2Size];	   
unsigned char 			RxBuf2[RxBuf2Size];

volatile unsigned int 		TxCnt2_wr ; 
volatile unsigned int 		TxCnt2_rd ;
volatile unsigned int 		RxCnt2_wr ; 
volatile unsigned int 		RxCnt2_rd ;

// for USART3(�������պͷ����������λ��壬���ں�GSMͨѶ)
unsigned char 			TxBuf3[TxBuf3Size];	   
unsigned char 			RxBuf3[RxBuf3Size];

volatile unsigned int 		TxCnt3_wr ; 
volatile unsigned int 		TxCnt3_rd ;
volatile unsigned int 		RxCnt3_wr ; 
volatile unsigned int 		RxCnt3_rd ;

BOOL					usart3_tx_busy 			= FALSE;		// ����3�����Ƿ�ʹ���еı�־

SWITCH					sw_gps_forward 		= OFF;			// GPS����ת��ģʽ�Ƿ����ı�־
BOOL					is_gps_forwarding		= FALSE;
BOOL					is_gsm_forwarding 		= FALSE;		// GSM����ת��ģʽ�Ƿ����ı�־
BOOL					is_gsm_downloading 	= FALSE;		// GSM��������ģʽ�Ƿ����ı�־

/********************************************************************************************************************
*
*
*
*
*********************************************************************************************************************/
/*
 * ��������USART_Configure
 * ����  ��USART1 GPIO ����,����ģʽ���á�115200 8-N-1
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
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

	USART_InitStructure.USART_BaudRate 		= 115200;					// Ϊ��֧��GPS������������ض���USART1��USART1������Ӧ����9600
	USART_InitStructure.USART_WordLength 	= USART_WordLength_8b;
	USART_InitStructure.USART_StopBits 		= USART_StopBits_1;
	USART_InitStructure.USART_Parity 		= USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode 			= USART_Mode_Rx | USART_Mode_Tx;	
	USART_Init(USART1, &USART_InitStructure);

	// ʹ�ܴ���1�����жϲ��򿪴���1Ӳ��
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	  				
	USART_Cmd(USART1, ENABLE);

	/* �����������1���ֽ��޷���ȷ���ͳ�ȥ������ */
	USART_ClearFlag(USART1, USART_FLAG_TC);     // ���־

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

	// ʹ�ܴ���2�����жϲ��򿪴���2Ӳ��
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 		
	USART_Cmd(USART2, ENABLE);

	/* �����������1���ֽ��޷���ȷ���ͳ�ȥ������ */
	USART_ClearFlag(USART2, USART_FLAG_TC);     // ���־

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

	// ʹ�ܴ���3�����жϲ��򿪴���3Ӳ��
  	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); 
	USART_Cmd(USART3, ENABLE);

	/* �����������1���ֽ��޷���ȷ���ͳ�ȥ������ */
	USART_ClearFlag(USART3, USART_FLAG_TC);     // ���־
}

/*
 * ��������fputc
 * ����  ���ض���c�⺯��printf��USART1
 * ����  ����
 * ���  ����
 * ����  ����printf����
 */
int fputc(int ch, FILE *f)
{
/* ��Printf���ݷ������� */
  USART_SendData(USART1, (unsigned char) ch);

  while (!(USART1->SR & USART_FLAG_TXE)); 

  return (ch);
}

/*******************************************************************************
	��������fgetc
	��  ��:
	��  ��:
	����˵����
	�ض���getc��������������ʹ��scanf�����Ӵ���1��������
*/
int fgetc(FILE *f)
{
	/* �ȴ�����1�������� */
	while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

	return USART_ReceiveData(USART1);
}

/**************************************************************************************************************/

// ��ָ�����ȵ�������䵽USART1���ͻ��壨�ݲ����ͣ���
void usart1_tx_fill(char* data, unsigned int size)
{
	unsigned int cnt = 0;

	while(cnt < size)
	{
		TxBuf1[(TxCnt1_wr++) & (TxBuf1Size - 1)]	= data[cnt++];	
	}
}

// ��USART1���ͻ����е����ݷ��ͳ�ȥ��
void usart1_tx_start(void)
{
	unsigned int to = systick+2*1000/SYSTICK_PERIOD;
	
	/* Enable the USART Transmoit interrupt: this interrupt is generated when the 
  		USART1 transmit data register is empty */ 
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);	  				
	USART_Cmd(USART1, ENABLE);

	// ������ȵ����ݷ�������˳������ݷ��ͻ����
	while(TxCnt1_rd < TxCnt1_wr)
	{
		if(systick >= to)
		{
			return;
		}
	}
}

// ��ָ�����ȵ�������䵽USART2���ͻ��壨�ݲ����ͣ���
void usart2_tx_fill(unsigned char* data, unsigned int size)
{
	unsigned int cnt = 0;

	while(cnt < size)
	{
		TxBuf2[(TxCnt2_wr++) & (TxBuf2Size - 1)]	= data[cnt++];	
	}
}

// ��ʼ��USART2���ͻ����е����ݷ��ͳ�ȥ��
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

// ����ѯ�ķ�ʽ��USART����һ���ֽڡ�
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

// ��ʼ��USART3���ͻ����е����ݷ��ͳ�ȥ��
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

// ʹ���жϷ�ʽֱ�Ӵ�USART3�ڷ���ָ��������(�������жϴ�������е���)��
void usart3_send_int(char* data, unsigned int size)
{
	unsigned int cnt = 0;

	// usart3���ͳ�ʱȡ���ڷ��ͻ����С�ʹ��ڲ����ʡ�
	unsigned int to = systick+3*1000/SYSTICK_PERIOD;

	// ��鵱ǰUSART3�Ƿ��ڷ�������(����ʹ����ѯ��ʽ�ڷ��ͣ�����������ʱ�����Զ���������)��
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

// ʹ����ѯ��ʽֱ�Ӵ�USART3�ڷ���ָ��������(�����жϴ�������е���)��
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


// ��USART2���յ�������ͨ��USART1���ͳ�ȥ����USART1���յ�������ת����USART2��
void usart2_redir_usart1(void)
{	
	unsigned char	byte;
	
	RxCnt2_rd = RxCnt2_wr;	
	RxCnt1_rd = RxCnt1_wr;

	// �ر��˶����
	// detect_motion(OFF);
	// detect_still(OFF);

	printf("to redirect USART2 Rx to USART1 Tx...\r\n");

	is_gps_forwarding = TRUE;
	
	while(1)
	{
		// ��GPS UART Txͨ��USART2 Rxת����USART1 Tx
		if(RxCnt2_rd < RxCnt2_wr)
		{
			TxBuf1[(TxCnt1_wr++)&(TxBuf1Size-1)] = RxBuf2[(RxCnt2_rd++)&(RxBuf2Size-1)];

			// ����GPS UART Tx�������λNMEA��Ϣ����˲�ȡ����ת����ʽ(���з���"\r\n")
			if(RxBuf2[(RxCnt2_rd-2)&(RxBuf2Size-1)] == '\r' && RxBuf2[(RxCnt2_rd-1)&(RxBuf2Size-1)] == '\n')
			{				
				usart1_tx_start();
			}
		}	

		// ����Ƿ�����#�Ž�β��ϵͳ��������
		check_buf_com();	
		check_que_cmd(CHECK_MODE_CONTINUOUS);	

		// ���ϵͳ�¼�
		sys_check_event();	

		WATCHDOG_RELOAD();
				
		// ��USART1 Rxͨ��USART2 Txת����GPS UART Rx
		if(RxCnt1_rd < RxCnt1_wr)
		{
			byte = RxBuf1[(RxCnt1_rd++)&(RxBuf1Size-1)];

			// ��USART1 Rx�Ͻ��յ�����ʹ����ѯ��ʽͨ��USART2 Txת�����ݸ�GPS UART Rx
			usart2_send_byte(byte);
		}

		// ����GPSת����ռ��USART1��Tx��Rx�����GPS����ת��ʱ�����USART1�ϵķ�GPS����(��ϵͳ����)����ʱ�û�Ҳ��Ӧ��USART1�Ϸ��ͷ�GPS���
		// �û����Ҫ��USART1�Ϸ���ϵͳ���Ӧ��ֹͣGPS����ת��(��������)��Ȼ�������USART1�Ϸ���ϵͳ����,Ҳ�ܴ�USART1�Ͽ���ϵͳ������Ϣ��

		// ����Ƿ�˫����������ֹͣGPS����ת�����¼�
		if(sw_gps_forward == OFF)
		{
			// ��⵽˫�������¼�ʱ�Ͽ�GPS��Դ���˳�GPS����ת������
			gps_power_down();	

			is_gps_forwarding = FALSE;

			// ���´��˶��;�ֹ���
			// detect_motion(ON);
			// detect_still(ON);

			printf("GPS forwarding terminated.\r\n");
			
			return;
		}	

		// GPS����ת���ڼ�ι�������⿴�Ź������λ
		WATCHDOG_RELOAD();
	}
}

// ��USART3���յ�������ͨ��USART1ת����ȥ����USART1���յ�������ת����USART3��
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
			
			// ����ת��
			if(RxBuf3[(RxCnt3_rd-2)%(RxBuf3Size-1)] == '\r' && RxBuf3[(RxCnt3_rd-1)%(RxBuf3Size-1)] == '\n')			
			{ 
				usart1_tx_start();
			}
		}	

		// ����Ƿ�����#�Ž�β��ϵͳ��������
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

		// ���GSM����ת�������Ƿ񱻹ر�
		if(is_gsm_forwarding == FALSE)
		{
			return;
		}

		// GSM����ת���ڼ�ι�������⿴�Ź������λ
		WATCHDOG_RELOAD();
	}	
}

