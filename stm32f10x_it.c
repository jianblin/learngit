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

//���ּ��ٶȴ������ж���Ӧ����
#ifdef TEST_GSENSOR_BASIC_FUNCTION

int cnt_singletap 	= 0;
int cnt_doubletap 	= 0;

int cnt_shake 		= 0;

int cnt_motion	 	= 0;
int cnt_freefall 		= 0;

int cnt_roll 			= 0;

int cnt_dataready	= 0;
#endif

static int		cnt_gsensor_still  = 0;			// ��ֹ����жϴ���������
static int		cnt_gsensor_motion = 0;			// �˶�����жϴ���������

void mcu_shutdown(void);

// -------------------------------------------------------
// �����ַ���	 ״̬���   ԭʼ�ַ���	��Ϣ����
// -------------------------------------------------------
// CLIP				30		// ����������
// SMSFLAG			31		// �������
// KEY				32		// DTMF������
// RELEASE			33		// �绰�Ҷ�
// RING				34		// ��������
// CMGR				35		// ��ȡ����
// TCPRECV			36		// TCP���ݽ���
// -------------------------------------------------------

#define MAX_STATE_NUM_GSM		27			// GSM�����ַ���״̬�������״ֵ̬

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


// GPS���������ַ������״̬Ǩ�Ʊ�(�����$GPRMC��$GPGGA�����ַ���)
#define MAX_STATE_NUM_GPS			8		// GPS�����ַ���״̬Ǩ�Ʊ�����״̬��

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

// ����'A' - 'S'֮���ַ���gps_ptn_map���е�������(�������ַ���Ӧ0��������ζ����gps_ptn_map���е�������Ϊ0)��
// �Ӷ��������뵱ǰ������ַ����ٵõ�����gps_ptn_map���е������š�
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

__asm int __get_msp(void)         //��ȡsp�ĵ�ַ,������ʹ�õ�MSP��˷���MSP
{
	MRS     R0,MSP              ; main function is use msp,so return msp here
    BX      LR
}

__asm int __get_psp(void)         //��ȡsp�ĵ�ַ,������ʹ�õ�MSP��˷���MSP
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
1��ϵͳ��ʼ����usart1���ջ���Ϊ�գ�
2���û�ͨ�����ڷ�������ʱ�����ݻᱻ������usart1���ջ����У��������ݹ����м�⵽�����β����"#\r\n"�򴮿����������������
3����ѭ���м�⴮�������������������0����usart1���ջ���ĵ�ǰ���α꿪ʼ��"#\r\n"��β֮����ַ�����ȡ���������������
4���������ʱ��ͨ���ָ������Ž����������ַ����ָ�Ϊ���������򣬵�һ��������Ϊ����壻
5������ȫ�������б��м�鵱ǰ������Ƿ���ƥ����еĻ��������������������͸�ʽ�Ƿ�Ϸ������Ϸ���ִ���������ִ�У�
ע: 
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

		// ���ڷ�GPSת��Ҳ��GSMת��״̬�²ż��USART1����
		// if(sw_gps_forward == OFF && is_gsm_forwarding == OFF)
		{
			// ����usart1���ݹ����в�ͣ��⴮������Ľ�β��־"#\r\n"����⵽�Ļ��Ž��������������������
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
��ͬ��λ״̬��u-blox 6010��uCenter������������һ��������Ϣ��
����Ϣǰ���ʱ����ϢΪuCenter�Զ���ӣ�����Ϣ�������У�

>> δ���⵽UTCʱ�䣨��δ��λ��ʱ
??:??:??  $GPRMC,,V,,,,,,,,,,N*53
??:??:??  $GPVTG,,,,,,,,,N*30
??:??:??  $GPGGA,,,,,,0,00,99.99,,,,,,*48
??:??:??  $GPGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*30
??:??:??  $GPGSV,1,1,00*79
??:??:??  $GPGLL,,,,,,V,N*64

>> ��⵽UTCʱ�䵫δ��λʱ��UCTʱ��������ڶ�λ�ɹ���
14:12:03  $GPRMC,141203.00,V,,,,,,,230212,,,N*78
14:12:03  $GPVTG,,,,,,,,,N*30
14:12:03  $GPGGA,141203.00,,,,,0,03,30.33,,,,,,*63
14:12:03  $GPGSA,A,1,30,23,16,,,,,,,,,,30.35,30.33,1.00*02
14:12:03  $GPGSV,1,1,04,16,53,354,39,23,47,306,30,24,24,041,,30,38,029,43*71
14:12:03  $GPGLL,,,,,141203.00,V,N*4F

>> ��λ��
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

static BOOL				found_rmc = FALSE;	// RMC��Ϣͷ����־
static BOOL				found_gsa = FALSE;	// GGA��Ϣͷ����־

static int 				cnt_msg_rmc = 0;
static int 				cnt_msg_gsa = 0;

static int				commas_rmc = 0;	

// 2012-03-05 �����ַ������״̬��ģ����Թ���
void USART2_IRQHandler(void)
{								   	
	char			ch;			
	
	// usart2�����ж�(ÿ�ν���һ���ֽ�)
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
 	{		
		// ���浱ǰ���յ������� 
		ch = USART_ReceiveData(USART2) & 0xFF;
	
		// ����ǰ���յ�������д��GPS���ջ���
		GPS_RX_WR(RxCnt2_wr++, ch);

		// printf("%c", ch);

		/************************ ���GPS��Ϣͷ ********************************/		
		
		// ȡ�õ�ǰ�����ַ���gps_ptn_map���е�������(��Ϊ�������ַ�������������)
		if(ch == '$')
		{
			input_gps = 0;
			
			state_gps = ptn_map_gps[state_gps][input_gps];	// '$'��������Ϊ0
		}
		else if(ch >= 'A' && ch <= 'S')							// �����'A' - 'S'֮����ַ�
		{
			// ����õ��������Ƿ�������(������Ϊ0���������壬���������ַ�)
			input_gps = ptn_idx_gps[ch - 'A'];
			
			if(input_gps > 0)
			{						
				state_gps = ptn_map_gps[state_gps][input_gps];
			}
			else // input_gps == 0
			{
				state_gps = 0;								// ״̬��λ	
			}
		}
		else
		{
			state_gps = 0;									// ״̬��λ
		}

		// ��鵱ǰ״̬�Ƿ�Ϊ��̬
		if(state_gps > (MAX_STATE_NUM_GPS-1))
		{
			switch(state_gps)
			{
				case 8:			// $GPRMC��⵽(ÿ��NMEA��Ϣ�ж�����һ��RMC��Ϣ��RMC��Ϣͷ����⵽��Ƶ����1Hz)
					// ������һ�����յ���rmc��Ϣ���������Ž����µ�rmc��Ϣ����ʵ�ֽ��պʹ����ͬ��
					if(is_rmc_received == FALSE)
					{
						found_rmc 		= TRUE;		
						cnt_msg_rmc 	= 0;
						
						commas_rmc 	= 0;
					}
					
					break;
				case 9:			// $GPGSA��⵽(ÿ��NMEA��Ϣ�ж�����һ��GSA��Ϣ��GSA��Ϣͷ����⵽��Ƶ����1Hz)
					// ������һ�����յ���gsa��Ϣ���������Ž����µ�gsa��Ϣ����ʵ�ֽ��պʹ����ͬ��
					if(is_gsa_received == FALSE)
					{
						found_gsa 		= TRUE;							
						cnt_msg_gsa 	= 0;
					}

					break;
				default:
					break;
			}

			state_gps = 0;							// ״̬��������̬�Ļ����̸�λ
		}

		/************************ ��ȡ�����ֽ���RMC/GSA��Ϣ ********************************/	
		// �ɲ���״̬��ʵ����Ϣ�Ľ��գ��Ӷ��ر�ʹ�ö��״̬������

		// ��⵽rmc��Ϣͷ��׼������rmc��Ϣ(��$GPRMC��C�ַ���ʼ���ա�ֱ�����з���)
		if(found_rmc == TRUE)
		{	
			// ������һ��rmc��Ϣ�������ſ�ʼ�����µ�rmc��Ϣ
			if(is_rmc_received == FALSE)
			{
				gps_msg_rmc[cnt_msg_rmc++] = ch;

				// ��⵽���з�����ȡ���ַ�������gps_msg_rmc�������󳤶ȣ���ֹͣ��ȡ
				if(((GPS_RX_RD(RxCnt2_wr-2) == '\r') && (GPS_RX_RD(RxCnt2_wr-1) == '\n')) || (cnt_msg_rmc >= MAX_LEN_GPS_RMC))
				{
					// ����ַ�����β����(�ڻ��з�֮��)
					gps_msg_rmc[cnt_msg_rmc++] = '\0';	

					// ��ȡ��һ��RMC��Ϣ�󣬽�RMC����־��λ���Ա�������ȡ��һ��RMC��Ϣ
					found_rmc 		= FALSE;
					cnt_msg_rmc 	= 0;

					// �����ǰGPS�Ѿ���λ��������RMC��Ϣ��ȡ��ɱ�־(֮ǰ��ȡ��RMC��Ϣ��������GPSδ��λʱ��ȡ�ģ����ͳͳ����)
					if(is_gps_fixed == TRUE)
					{
						is_rmc_received = TRUE;

						// ������һ��rmc��Ϣ�󣬽�������Ϣ��Ҫʱ�䣬��˿�������ϴν����굫���ڴ����rmc��Ϣ���µĽ��չ��̸��ǣ�������Ҫͬ��
					}
				}	
			}			

			// ��⵽RMC��Ϣͷ�󣬽����ż�ⶨλ״̬
			if(ch == ',')
			{
				commas_rmc++;

				// ��⵽GPRMC��Ϣͷ��ĵ��������ţ�����ȡ�˶���֮ǰ��һ���ַ�(����λ״̬��־: A - �Ѷ�λ��V - δ��λ)
				if(commas_rmc == 3)
				{	
					if(GPS_RX_RD(RxCnt2_wr-2) == 'A')
					{			
						// ����GPS��λ��־
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

		// ���ܵ�ǰ�Ƿ�λ�� �ڼ�⵽GSA��Ϣͷ�������ȡ����Ϣ(��GSA��ĩβ�ַ�A��ʼ��ȡ)
		if(found_gsa == TRUE)
		{			
			// ��GSA��Ϣͷ֮��ĵ�һ������(�������ű���)��ʼ��ȡGSA��Ϣ
			if(is_gsa_received == FALSE)
			{
				gps_msg_gsa[cnt_msg_gsa++] = ch;

				if((GPS_RX_RD(RxCnt2_wr-2) == '\r' && GPS_RX_RD(RxCnt2_wr-1) == '\n') || (cnt_msg_gsa >= MAX_LEN_GPS_GSA))
				{
					// ����ַ�����β����(�ڻ��з�֮��)
					gps_msg_gsa[cnt_msg_gsa++] = '\0';	

					// ��ȡ��һ��GSA��Ϣ�󣬽�GSA����־��λ���Ա�������ȡ��һ��GSA��Ϣ
					found_gsa = FALSE;
					cnt_msg_gsa = 0;
					
					// �����ǰRMC��Ϣ�Ѿ�����ȡ�꣬������GSA��Ϣ��ȡ��ɱ�־(��һ��NMEA��Ϣ��RMC��Ϣ�ǰ)
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
static int	state_gsm = 0;	// �����ַ������״̬���ĵ�ǰ״̬
static int	input_gsm = 0;	// �����ַ������״̬���ĵ�ǰ�����ַ�

static BOOL	found_smsflag 	= FALSE;	// �Ƿ��⵽SMSFLAG�����ַ����ı�־(ģ����յ�����ʱ���)
static BOOL	found_cmgr 		= FALSE;	// �Ƿ��⵽CMGR�����ַ����ı�־(���Ͷ����������ģ�����)
static BOOL	found_clip 		= FALSE;	// ���μ�⵽�����¼����Ӧ��ʱִ�����������ı�־
static BOOL	found_key 		= FALSE;	// ͨ���ڼ��⵽��DTMF�źŵı�־
static BOOL	found_release 	= FALSE;	// ͨ���Ҷϵı�־
static BOOL	found_tcprecv	= FALSE;	// TCP���ݰ����ձ�ʶ


// ��tcp�������
// static BOOL		to_recv_data = FALSE;
// static int		tcp_conn_id  = 0;
// static int		tcp_comma_cnt = 0;

static int	cnt_pnin = 0;	// pnin�ַ������α�
static int	cnt_dtmf = 0;	// dtmf�ַ������α�
static int	cnt_sms  = 0;	// cmt�ַ������α�

static int	offset_dtmf 	= 0; // 

	   int 	rings  = 0;		// ��ǰ�������	
static int 	clips  = 0;		// ��ǰCLIP��Ϣ����	
static int	quotes = 0;		// ��⵽���ŵĴ���
static int	enters = 0;		// ��⵽���з��ŵĴ���

static BOOL	flg_fetch_pnin = FALSE;	// �Ƿ�ʼ��ȡ�������
static BOOL	flg_fetch_dtmf = FALSE;	// �Ƿ�ʼ��ȡDTMF�ַ�
static BOOL	flg_fetch_cmgr  = FALSE;	// �Ƿ�ʼ��ȡCMT�ַ�
static BOOL	flg_fetch_smsflag  = FALSE;	// ��ʼ��ȡ���������ŵı�־

#define MAX_LEN_GSM_SMSFLAG		32

static char buf_smsflag[MAX_LEN_GSM_SMSFLAG+1];
static int	cnt_smsflag = 0;	// cmt�ַ������α�

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

		// �����յ���1���ֽ�����д��GSM���ջ���
		GSM_RX_WR(RxCnt3_wr++, byte);			

		/*************************************************
		 1, ��������ַ���
		**************************************************/

		// ���������ַ�Ϊ״̬Ǩ�Ʊ������ֵ
		if(byte >= 'A' && byte <= 'Y')	// ����Ȥ���ַ�
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
		else							// ������Ȥ���ַ�
		{				
			// ״̬����λ
			state_gsm = 0;	
		}

		// ��ֵ����MAX_STATE_NUM_GSM��״̬Ϊ��̬
		if(state_gsm > (MAX_STATE_NUM_GSM))
		{
			switch(state_gsm)
			{
				case 30:  	// CLIP�����������ʾ
					found_clip = TRUE;		

					// �ɳ����������CLIP��Ϣ
					clips++;		

					break;
				case 31:	// SMSFLAG������Ϣ����ָʾ
					found_smsflag = TRUE;
					
					break;
				case 32:	// KEY��dtmf�������
					found_key = TRUE;

					break;
				case 33:	// RELEASE��ͨ���Ҷ�
					found_release = TRUE;		
					
					break;
				case 34:  	// RING����������
					// �ɳ����������RING��Ϣ	
					rings++;		

					printf("#%d ringing...\r\n", rings);

					// �ڵ绰�Ҷ�ǰ��⵽4��������Ϣ�����Զ���������DTMF���
					if(rings >= DEF_TIMES_RING_TO_ANSWER_CALL)
					{
						// printf("rings = %d\r\n", rings);						

						if(sts_dtmf_command == STS_DTMF_COMMAND_CALL_WAITING)
						{
							sts_dtmf_command = STS_DTMF_COMMAND_CALL_INCOMING;
							
							times_input_dtmf_password = 0;
							
							// �Զ�����(ʹ����ѯ��ʽ�����Զ���������)
							strcpy(at, "ATA\r\n");
							usart3_send_poll(at, strlen(at));		

							// ����gsmͨ��״̬��־
							is_gsm_calling = TRUE;

							// ����ATA�����Ӧ�ʵ��ӳ�(�����ӳ�500ms�����������AT+CMUT����ͺ��޷���Ч)
							delay_100ms(5);		

							sts_dtmf_command = STS_DTMF_COMMAND_CALL_ACCEPTED;

							#if 1
							// �������þ��������ⱻδ��Ȩ�������
							strcpy(at, "AT+CMUT=1\r\n");
							usart3_send_poll(at, strlen(at));	

							delay_100ms(3);
							#endif
							
							sts_dtmf_command = STS_DTMF_COMMAND_MIC_MUTE_ON;

							// �������������dtmf���볬ʱ��ⶨʱ��
							swtimer_input_dtmf = TO_DTMF_INPUT_PASSWORD*1000/SYSTICK_PERIOD;
						}

						// �ټ���Ƿ���Ҫ��DTMF���(GSM������һ����DTMF�����һֱά�ּ��״̬��ֱ��GSM�ػ�)
						if(is_dtmf_detection_enabled == FALSE)
						{
							// ��DTMF���(ʹ����ѯ��ʽ����)
							strcpy(at, "AT+DTMFDETECT=1\r\n");
							usart3_send_poll(at, strlen(at));	

							// printf("send AT+DTMFDETECT=1 to GSM by polling.\r\n");

							// һ����DTMF��⣬�˺������ٴ򿪣�����ģ������
							is_dtmf_detection_enabled = TRUE;
						}	
					}

					break;
				case 35:	// CMGR����ȡ����Ķ���
					found_cmgr = TRUE;

					break;
				case 36:	// TCPRECV�����յ�tcp���ݰ�
					found_tcprecv = TRUE;		

					found_tcprecv = found_tcprecv;		// ������뾯��
			
					break;
				default:
					break;			
			}

			state_gsm = 0;		// ״̬����λ
		}

		/*************************************************
		 2, ����⵽�����ַ���������Ӧ�Ĵ���
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
		
		// ��⵽CLIP��Ϣʱ����ʱ��ȡ���������Ϣ(��һ�ξ���ȡ���������ȡ)
		if(found_clip == TRUE)
		{
			// ��⵽��һ��CLIP��Ϣʱ�Ϳ�ʼ��ȡ�������
			if(clips == 1)
			{
				// ���������Ϣ������������
				if(byte == '"')	  
				{
					quotes++;

					// ��⵽��һ������ʱ��ʼ������������ַ������ַ�
					if(quotes == 1)
					{
						// ��ʼ��ȡ���������Ϣ
						flg_fetch_pnin = TRUE;
					}
					else if(quotes == 2)
					{
FINISH_RECEIVING_GSM_PNIN:						
						// ����������ַ���������ַ�����β����
						de_gsm_pnin[cnt_pnin++] = '\0';		// ����������Ϊ�գ���de_gsm_pnin�ַ��������в������ַ�����					

						// ����������ʽ��
						if(format_pn((char*)de_gsm_pnin) != OK)
						{
							// ����������ַ������ʼ��Ϊ���ַ���
							strcpy(de_gsm_pnin, "");
						}
						else
						{						
							strcpy(de_gsm_pnin, de_gsm_pnin);
						}
						
						printf("de_gsm_pnin = %s\r\n", de_gsm_pnin);

						// ���ż���������
						quotes = 0;					

						// PNIN�ַ����α긴λ
						cnt_pnin = 0;

						// CLIP����־��λ
						found_clip = FALSE;

						// ֹͣ��ȡ���������Ϣ
						flg_fetch_pnin = FALSE;
					}
				}	
				else
				{
					if(flg_fetch_pnin == TRUE)
					{							
						// ���������ȡ���������Ϣ
						de_gsm_pnin[cnt_pnin++] = byte;

						// �����ȡ����������ַ����Ƿ�Խ��
						if(cnt_pnin >= MAX_LEN_PN)
						{
							goto FINISH_RECEIVING_GSM_PNIN;
						}
					}
				}
			}
			else
			{
				// ���Ե�һ��֮���CLIP��Ϣ
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
		// ��⵽KEY��Ϣʱ����ȡKEY�ַ�
		else if(found_key == TRUE)
		{
			// printf("KEY found.\r\n");
			
			// ��⵽ð��ʱ׼����ȡ�������ֵ
			if(byte == ':')
			{
				flg_fetch_dtmf = TRUE;
			}
			else
			{
				if(flg_fetch_dtmf == TRUE)
				{
					offset_dtmf++;

					// ð�ź�ĵڶ����ַ�Ϊ��������ֵ
					if(offset_dtmf == 2)
					{
						de_gsm_dtmf[cnt_dtmf++] = byte;	

						// printf("%c", byte);

						// ƫ��������������
						offset_dtmf = 0;

						// DTMF��ȡ��־��λ
						flg_fetch_dtmf = FALSE;

						// һ��DTMF�ַ���ȡ��󣬽�DTMF����־��λ
						found_key = FALSE;

						// ��鰴������ֵ�Ƿ�Ϊ#����DTMF�ַ�������Խ��
						if(byte == '#' || (cnt_dtmf >= MAX_LEN_GSM_DTMF))
						{
							// DTMF�ַ���������ַ�����β����
							de_gsm_dtmf[cnt_dtmf++] = '\0';

							printf("DTMF string:%s\r\n", de_gsm_dtmf);
							
							ana_str_dtmf((char*)de_gsm_dtmf);

							// DTMF�ַ����α긴λ
							cnt_dtmf = 0;
						}
					}
				}
			}
		}

		// ��⵽�绰�Ҷ���Ϣ(RING��CLIP�Լ�DTMF��Ϣ��ֻ����RELEASE��Ϣ����֮ǰ����)
		else if(found_release == TRUE)
		{
			// ����gsmͨ��״̬��־
			is_gsm_calling = FALSE;
			
			printf("rigns = %d\r\n", rings);

			// �Է��Ҷϵ绰��dtmf�����ָ���ʼ״̬(�����Զ��Ҷϵ绰��Ҳ�Ὣdtmf�����ָ���ʼ״̬)
			sts_dtmf_command = STS_DTMF_COMMAND_CALL_WAITING;

			// ���������ʾ�����ַ���
			strcpy(de_gsm_pnin, "");
 			
			if(rings > 0)		// incoming call
			{				
				ana_num_ring(rings);

				// RING����������
				rings = 0;

				// CLIP����������
				clips = 0;		// CLIP��Ϣ��RING��Ϣ��������

				// �Ҷϵ绰(���ǽ�����һ��DTMF������)��is_gsmring_resolved����Ϊ��Ч
				is_gsmring_pending = FALSE;
			}
			else	// outgoing call
			{
				// TBD
			}

			found_release = FALSE;

			// ��GSMģ������(��gsm_sleep()����չ�����Իر��жϴ�������в������жϷ�ʽ����AT���������)
			strcpy(at, "AT+ENPWRSAVE=1\r\n");
			usart3_send_poll(at, strlen(at));

			// ����DTR����
			MCU_GPIO_LOW(GPIO_GSM_DTR);   // GSM DTR	
		}

		/************************************************************************************  
		V015��ǰ�İ汾�����ö����Զ����ʱ����Ϣ���¡�

		Textģʽ���������Ϣ:
			+CMT: "8613923887347", ,"2012/04/13 17:40:02+32"
			test!			// Unicode Big Endian����Ϊ: 0074 0065 0073 0074 0021 

			+CMT: "8613923887347", ,"2012/04/13 17:44:43+32"
			4F60597DFF01	// ����: ��ã�(��Unicode Big Endianģʽ�����ÿ������ռ��2�ֽ�)

			+CMT: "8613923887347", ,"2012/04/13 17:47:14+32"
			007400654F60597D00730074FF01	// ����: te���st��(��Unicode Big Endianģʽ�����ÿ������ռ��2�ֽ�)

		PDU ģʽ���Զ�����Ľ��յ�����Ϣ: 
		+CMT: ,28
		0891683110200005F0240BA13134202377F00000216052229431230A4967B4057D4E59B111			// INQ-POS,1#
		 
		+CMT: ,29
		0891683110200005F0240BA13134202377F00008216052229463230A54084F5C61095FEBFF01		// ������죡
		-------------------------------------------------------------------------------------	
		V015��ʼ�İ汾����ֹ�����Զ���������Զ����������Ϣ:

		+SMSFLAG: "SM", 27		// 2012-12-32, M660+ V015 FW		
		*************************************************************************************/
		// M660+���յ�����ʱ��������RING���ţ���ʱ~600ms�����Զ����+SMSFLAG��Ϣ���ⲿ�����⵽
		// SMSFLAG��Ϣ������ȡ���еĶ��ű�ţ��ٷ���AT+CMGR�����ȡ���½��յ��Ķ��ţ�����ټ��CMGR��Ϣ��
		// ע: ͨ���������ж��ŷ���Ҳ���������ղ���ȡ��������
		else if(found_smsflag == TRUE)
		{	
			if(flg_fetch_smsflag == FALSE)
			{
				if(byte == ':')
				{
					flg_fetch_smsflag= TRUE;		// ָʾ���Կ�ʼ��ȡSMSFLAG��Ϣ

					strcpy(buf_smsflag, "");		// ��ʼ��buf_smsflag
					cnt_smsflag = 0;

					return;
				}
			}
			else				
			{
				// ����ֽڽ���SMSFLAG��Ϣ
				buf_smsflag[cnt_smsflag++] = byte;

				// ���յ��ַ���������smsflag����󳤶ȣ���ֹͣ����
				if(cnt_smsflag >= MAX_LEN_GSM_SMSFLAG)
				{
					goto FINISH_RECEIVING_SMSFLAG;
				}

				// ���յ����з�ʱҲֹͣ����
				if((GSM_RX_RD(RxCnt3_wr-2) == '\r' && GSM_RX_RD(RxCnt3_wr-1) == '\n'))
				{
					enters++;

					// ��⵽��һ�����з�ʱ��SMSFLAG��Ϣ�������
					if(enters == 1)
					{			
FINISH_RECEIVING_SMSFLAG:						
						// ����ַ�����β����
						buf_smsflag[cnt_smsflag++] = '\0';	

						// ��λ��SMSFLAG��Ϣ�еĶ��ű����ʼ�ַ���			
						len_smsflag = strlen(buf_smsflag);

						for(i = 0; i < len_smsflag; i++)
						{
							if(buf_smsflag[i] == ',')
							{	
								break;
							}
						}

						// ��������
						i++;

						// ���ڶ�λ������ʱ�Ž�һ�����Ͷ�ȡ���ŵ�����
						if(i < cnt_smsflag)
						{
							// ����ѯ��ʽ����AT+CMGR����������
							sprintf(at, "AT+CMGR=%s\r\n", (char*)buf_smsflag+i);
							
							usart3_send_poll(at, strlen(at));
						}
						// else: �����Ͷ�ȡ���ŵ�����ն˽���ȥҲ������յ�����Ķ���

						// SMSFLAG��Ϣ��ȡ��صļ���������
						cnt_smsflag 	= 0;								 
						enters 			= 0;
						quotes 			= 0;

						// ���SMSFLAG��Ϣ��ȡ��־
						flg_fetch_smsflag = FALSE;

						// ��λSMSFLAG�����ַ�������־
						found_smsflag 	  = FALSE;
						
						return;
						
						// ����������ͺ�MCU�漴Ӧ�û��⵽+CMGR��Ϣ......
					}				  			
				}	
			}
		}

		// ��⵽SMSFLAG���漴����AT+CMGR�����ȡ�ս��յ��Ķ��ţ�Ȼ�������⵽CMGR�����ַ���
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
						// ������ջ�������������������µ�����
						found_cmgr = FALSE;
						
						printf("sms queue is full!!!\r\n");
					}

					return;
				}
			}
			else
			{					
				// �����յ�CMGR��Ϣ���浽ϵͳ��SMS���ջ��ζ����У�Ϊ�˱������ݿ�����ֱ����ISR�ｫ�������ֽ�д������С�
				que_sms_cmd[cnt_sms_cmd_wr&(MAX_NUM_SMS_RX-1)][cnt_sms++] = byte;

				// ���յ�������ȳ��������������󳤶�ʱ��ֹͣ�������2013-05-21 added
				if(cnt_sms > MAX_LEN_CMD)
				{
					goto FINISH_RECEIVING_CMGR;
				}
				
				// ��⵽�ڶ������з�ʱ��Ҳֹͣ��������
				if(GSM_RX_RD(RxCnt3_wr-2) == '\r' && GSM_RX_RD(RxCnt3_wr-1) == '\n')
				{
					enters++;

					if(enters == 2)
					{	
FINISH_RECEIVING_CMGR:						
						// ����ַ�����β����
						que_sms_cmd[cnt_sms_cmd_wr&(MAX_NUM_SMS_RX-1)][cnt_sms++] = '\0';

						// ����SMS���ջ��ζ���д�α�
						cnt_sms_cmd_wr++;

						printf("cnt_sms_cmd_wr = %d\r\n", cnt_sms_cmd_wr);

						// ��������ź����GSMģ��RING�ж��¼��������־
						is_gsmring_pending = FALSE;

						// CMT��Ϣ��ȡ��صļ���������
						cnt_sms	= 0;								 
						enters 	= 0;
						quotes 	= 0;

						// ���CMGR��Ϣ��ȡ��־
						flg_fetch_cmgr 	= FALSE;

						// ��λCMGR�����ַ�������־
						found_cmgr 		= FALSE;

						// ɾ����ǰ���յ��Ķ��ţ�����洢���ж�����(����Ϊ50��)
						sprintf(at, "AT+CMGD=%s\r\n", (char*)buf_smsflag+cnt_smsflag);
						usart3_send_poll(at, strlen(at));	

						delay_100ms(1);

						// ��GSMģ������
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
	
	// ���-������������Ҫ�ȶ�SR,�ٶ�DR�Ĵ����������������жϵ�����[ţ��˵Ҫ����]
   	if(USART_GetFlagStatus(USART3,USART_FLAG_ORE)==SET)
    {
    	USART_ClearFlag(USART3,USART_FLAG_ORE); //��SR��ʵ���������־

		USART_ReceiveData(USART3); 				//��DR        
   	}  
}

// �ϴζ�ȡ�ļ��ٶȴ�����������ٶ���ֵ(�����ں͵��ζ�ȡ��������ٶ���ֵ�ȽϴӶ��ж��Ƿ����˶�)
static short 	regx_pre = 0;
static short 	regy_pre = 0;
static short 	regz_pre = 0;

static short	regx;
static short	regy;
static short	regz;

extern unsigned short swtimer_is_mcu_idle;
		
// RTC �����жϡ�
void RTCAlarm_IRQHandler(void)
{
	// ��EXTI_Line17����λ
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

	// ���ϵͳ����systick����rtcalarm�ж���Ϊ����tick�жϣ�������ѭ�����в�ʱι���Ĳ���(����ִ��ʱ��ϳ��Ĺ�����Ҳ��ι������)�����������systick�жϴ���������ι����
	// �������Ա������ȼ���systick�жϸ��͵��жϴ������е���ѭ�����ܴ������Ź����������
	// �������rtcalarm�ж���Ϊϵͳ������tick�ж���mcuҪ����stopģʽ����ômcuһ������sropģʽ����ѭ�����޷�ι������˱�����rtcalarm�жϴ�������ж���ι�������⿴�Ź������
	if(is_sys_idle == TRUE)
	{
		
	}

	/*************************************** ���dtmf���������Ƿ�ʱ *************************************************/
	if(swtimer_input_dtmf > 0)
	{
		swtimer_input_dtmf--;

		if(swtimer_input_dtmf == 0)
		{
			strcpy(at, "ATH\n");
			usart3_send_poll(at, strlen(at));	

			// ����gsmͨ��״̬��־
			is_gsm_calling = FALSE;

			sts_dtmf_command = STS_DTMF_COMMAND_CALL_WAITING;
		}
	}

	// �û���绰����ʱ������dtmf������ȷ��ر���ͷ�������ӳٶ�ʱ�����
	if(swtimer_set_micmute_off > 0)
	{
		swtimer_set_micmute_off--;

		if(swtimer_set_micmute_off == 0)
		{
			// �ر���ͷ������������ͷ��������Է������
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

	// ��ʱ��λtimer
	if(swtimer_periodic_pos > 0)
	{
		swtimer_periodic_pos--;

		if(swtimer_periodic_pos == 0)
		{
			// ���ڶ�λ���������
			sem_periodic_pos++;
		
			// �������ڶ�λģʽ��ʱ�������ڶ�λtimer��	
			if(sw_periodic_pos == ON)
			{
				// ����������ʱ��λtimer
				swtimer_periodic_pos = cycle_periodic_pos*1000/SYSTICK_PERIOD;
			}
		}	
	}
}

static int cnt_key_triple = 0;		// �����İ�������
static int cnt_key_double = 0;		// ˫���İ�������
static int cnt_key_single = 0;		// �����İ�������

/* I/O���жϣ��ж���ΪPA0 */
void EXTI0_IRQHandler(void)
{
	u16 ret;
	
	if(EXTI_GetITStatus(EXTI_Line0) != RESET) 			// ȷ���Ƿ������EXTI Line�ж�
  	{				
 		ret = key_scan();	

		printf("ret of key_scan() = %d\r\n", ret);
		
		switch(ret)
		{
			// ������˫����֧���Ӷ�GPS��GSM��ʼ��״̬�ļ�⣬���Թ���������ʱ���������󴥷�������˫���¼�
			case 1:	 
				cnt_key_single++;			
				
				break;
			case 2:			// SOS
				cnt_key_double++;

				// dump_stack();

				break;				
			case 3:	 //����

				cnt_key_triple++;

				/*
				printf("GPIO_LED_B = %d, sw1 = %d\r\n", last_led_status[GPIO_LED_B-GPIO_LED_B].led, last_led_status[GPIO_LED_B-GPIO_LED_B].sw);
				printf("GPIO_LED_G = %d, sw1 = %d\r\n", last_led_status[GPIO_LED_G-GPIO_LED_B].led, last_led_status[GPIO_LED_G-GPIO_LED_B].sw);
				printf("GPIO_LED_R = %d, sw1 = %d\r\n", last_led_status[GPIO_LED_R-GPIO_LED_B].led, last_led_status[GPIO_LED_R-GPIO_LED_B].sw);
				*/
				
				if(cnt_key_triple & 1)
				{
					// ��ϵͳ����Ϊ�ڰ�ģʽ
					is_sys_in_darkmode = TRUE;		
					
					// ֱ�ӵ���GPIO���ƺ���Ϩ��������ɫ��led��(����led_switch����Ϩ��Ļ����ƻ�last_led_status�����б���ĸ���led���ڽ���ڰ�ģʽǰ����ʵ����״̬)
					MCU_GPIO_HIGH(GPIO_LED_B);
					MCU_GPIO_HIGH(GPIO_LED_G);
					MCU_GPIO_HIGH(GPIO_LED_R);
				}
				else
				{
					// �Ȼָ�Ϊϵͳ����ģʽ
					is_sys_in_darkmode = FALSE;		
					
					// �ٵ���led_switch�����ָ�����led�Ƶ�����״̬
					led_switch(last_led_status[GPIO_LED_R-GPIO_LED_R].led, last_led_status[GPIO_LED_R-GPIO_LED_R].sw);
					led_switch(last_led_status[GPIO_LED_G-GPIO_LED_R].led, last_led_status[GPIO_LED_G-GPIO_LED_R].sw);
					led_switch(last_led_status[GPIO_LED_B-GPIO_LED_R].led, last_led_status[GPIO_LED_B-GPIO_LED_R].sw);
				}
				
				break;
			case 4:	 //����		
				mcu_shutdown();	
				
				break;
			default:
				break;
		}
  	}

	EXTI_ClearITPendingBit(EXTI_Line0);     			// ����жϱ�־λ 
}

void EXTI3_IRQHandler(void)
{
	// PB3: GSM_RING
	if(EXTI_GetITStatus(EXTI_Line3) != RESET) 	
	{	
		isr_gsmring_asserted();
		
		EXTI_ClearITPendingBit(EXTI_Line3);		// ����жϱ�־λ  
	}
}

// PB4 ������Ϊ�ⲿ�ж������!!!

// �жϱ��Ϊ9-5���ⲿ�жϵĴ�������
void EXTI9_5_IRQHandler(void) // EXTI Line 15..10
{
#if 0
	// PA6: CHG_STS
	if(EXTI_GetITStatus(EXTI_Line6) != RESET) 	
	{	
		isr_chgsts_asserted();
		
		EXTI_ClearITPendingBit(EXTI_Line6);		// ����жϱ�־λ  
	}
#endif

#ifdef USING_PWR_ACC
	#if (HW_VER == 17)	
	// EXTI_ACCPWR_DET
	if(EXTI_GetITStatus(EXTI_Line9) != RESET)
	{
		isr_accpwr_detected();
		
		EXTI_ClearITPendingBit(EXTI_Line9);     	//����жϱ�־λ
	}		

	#elif (HW_VER == 16)

	// EXTI_ACCPWR_DET
	if(EXTI_GetITStatus(EXTI_Line8) != RESET)
	{
		isr_accpwr_detected();
		
		EXTI_ClearITPendingBit(EXTI_Line8);     	//����жϱ�־λ
	}		
	#endif
#endif

#if 0
	// PA8: GSR_INT2
	if(EXTI_GetITStatus(EXTI_Line8) != RESET) 	
  	{			
		// isr_gsr_int2();
			
		EXTI_ClearITPendingBit(EXTI_Line8);		// ����жϱ�־λ  
	}
#endif
}

int cnt_int_acc = 0;

// void isr_gsr_int1(void);

// �жϱ��Ϊ15-10���ⲿ�жϵĴ�������
void EXTI15_10_IRQHandler(void)
{
#if 1
	// PA11: GSR_INT1
	if(EXTI_GetITStatus(EXTI_Line11) != RESET) 				
  	{	
		// isr_gsr_int1();
		
		EXTI_ClearITPendingBit(EXTI_Line11);		// ����жϱ�־λ 
  	} 
#endif

#ifdef USING_PWR_EXT
	// EXTI_EXTPWR_DET
	if(EXTI_GetITStatus(EXTI_Line12) != RESET)
	{
		isr_extpwr_detected();
		
		EXTI_ClearITPendingBit(EXTI_Line12);     	//����жϱ�־λ
	}
#endif
}

//��ʱ��2�жϴ�����(��������֮��ֹ����ӳ�timer)��
void TIM2_IRQHandler(void)
{
	TIM2->SR&=~(1<<0);//����жϱ�־λ
}

// ������Ź���ʱ������жϴ�������
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET )
  	{
  		TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);

		// ������Ź����ʱ�������λMCU

		printf("Software Watchdog handler.\r\n");
		
		mcu_reset();
	}
	
	TIM3->SR&=~(1<<0);//����жϱ�־λ
}

//��ʱ��4�жϴ�����(���ڶ�λtimer)
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4 , TIM_IT_Update) != RESET )
  	{
  		TIM_ClearITPendingBit(TIM4 , TIM_FLAG_Update);

		usertimer_counter++;

		// printf("usertimer_counter = %d\r\n", usertimer_counter);
		
#ifdef USING_DEV_GSR
		// ÿ��200*2=400ms�������ٶȴ�����������ٶ���ֵ
		if(usertimer_counter&1)
		{
			isr_gsrval_to_measure();
		}
#endif

		// ÿ��512*200/1000=102��(Լ1.7����)����һ�ε�ص�ѹ
		if(!(usertimer_counter & 511))
		{
			isr_batvol_to_measure();
		}
	}
	
	TIM4->SR&=~(1<<0);//����жϱ�־λ
}

#if 0
void isr_gsr_int1(void)
{
	unsigned char 		IntSourceMFF;
	unsigned char 	SysMod;

	IntSourceMFF = I2C_RegRead(REG_INT_SOURCE);
	
		#if 1	// ��ʹ�ü��ٶȴ��������õľ�ֹ��⹦�ܣ��������ⲿ����ʵ�־�ֹ��⡣
		// ��ȡ���ٶȴ������ĵ�ǰϵͳģʽ: standby, wake, sleep
		
		
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
			
			// ��ϵͳģʽ�Ĵ�����ѯ��ǰ����ģʽ
			SysMod=I2C_RegRead(REG_SYSMOD);				
			
			if(SysMod==0x02)					// ��ǰ����sleepģʽ����wake mode --> sleep mode
			{					
				cnt_gsensor_still++;

				// ���ٶȴ������ϵ��ʼ������Զ�����һ�ξ�ֹ����жϣ��������˶�����Ƿ�򿪡���ֹ
				// ��⹦���Ƿ����������Ҫ���Լ��ٶȴ������ϵ��ʼ����ĵ�һ����ֹ����жϡ�
				if(cnt_gsensor_still > 1)
				{
					event_handler_still();
				}
			}
			else if(SysMod==0x01)				// ��ǰ����wakeģʽ�� ��sleep mode --> wake Mode
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
			I2C_RegRead(REG_PL_STATUS);			// ��ȡPortrait/Landscape״̬�Ĵ��������ж�

			// event_handler_roll();
		}
		#endif

		#if 1
		// PULSE interrupt(singletap or doubletap)
		if(IntSourceMFF & 0x08)					
		{
			// ��ȡ����ж�Դ�Ĵ����е�DPEλ���жϲ����жϵ��ǵ�������˫���¼���˳��������ж�	
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

		// ���ٶȴ������ϵ��ʼ������ʱ���Զ�����һ���˶�����жϣ�
		// ��˺��Լ��ٶȴ������ϵ��ĵ�һ���˶�����жϡ�
		if(cnt_gsensor_motion > 1)
		{
			// ��⵽�˶�ʱ����ֹ��ⶨʱ����λ
			swtimer_still_detection = DEF_SWTIMER_STILL_DETECTION;

			// ��ȡ���ٶȴ��������ж�Դ
			IntSourceMFF = I2C_RegRead(REG_INT_SOURCE);
	
			// TRANS interrupt
			if(IntSourceMFF & 0x20)						// ���޼��
			{	
				if(I2C_RegRead(REG_TRANSIENT_CFG)&0x01)	// HPF OFF				
				{				
					event_handler_motion();
				}
				else									// HPF ON 
				{
					// TBD
				}	

				// I2C_RegRead(REG_TRANSIENT_SRC);	// ��ȡ���ж�Դ�Ĵ��������ж�		
			}	
			else if(IntSourceMFF & 0x04)				// FF_MT interrupt�����޼��
			{
				// ��ȡ��ǰ��OAEλ���ã��Ա��жϼ����������������¼������˶��¼�
				OAE = I2C_RegRead(REG_FF_MT_CFG);

				if(!(OAE & 0x40))			// Freefall
				{
					event_handler_motion();
				}
				else						// Motion
				{
					// TBD
				}

				// ��λ���ж�Դ�Ĵ����Ա����ж�	
				// I2C_RegRead(REG_FF_MT_SRC);	
			}	
		}
}
#endif

#ifdef USING_PWR_EXT
// �ⲿ����Դ��ͨ/�Ͽ�����жϡ�
void isr_extpwr_detected(void)
{	
	/**************************************** �ϵ籨����� ******************************************/
	if(MCU_GPIO_READ(GPIO_EXTPWR_DET) == ON)	
	{
		sts_power |= (1<<BIT_STS_POWER_EXT);

		// �̵�����������ָʾ��ӵ�Դ��ͨ��
		led_switch(GPIO_LED_G, OFF);
		led_switch(GPIO_LED_B, ON);

		printf("external power on.\r\n");
	}
	else
	{
		sts_power &= ~(1<<BIT_STS_POWER_EXT);

		// �̵�����������ָʾ��ӵ�Դ�Ͽ���
		led_switch(GPIO_LED_G, ON);
		led_switch(GPIO_LED_B, OFF);

		printf("exteral power off.\r\n");
	}
}
#endif

#ifdef USING_PWR_ACC
static unsigned int 	tm_acc_changed = 0;	// �ϴ�acc��Դ�ı�(�Ͽ����ͨ)��ʱ���

unsigned int			cnt_acc_changed = 0;

#define MAX_INTERVAL_ACC_CHANGED		10		// acc��Դ�仯���������ʱ����(����acc�Ͽ�ʱ�ĵ��ݷŵ绺����12V acc��ѹ����£�
												// �Ͽ�acc��Դ��һ����2-3�������ܼ�⵽acc�Ͽ���16V acc��ѹ����£��Ͽ�acc��Դ��
												// һ����3-4��Ż����acc����ж�)

// ����ϴ�acc�仯������acc�仯�Ƿ��ڶ�ʱ���ڷ������ǵĻ���Ϊ�仯��Ч(һ���ڲ���acc��Դ�Ƿ�������ȷʱͨ�����ٿ���acc��Դ����)��
void check_acc_changed(void)
{
	// �������״α�����⵽acc�仯ʱ��ֻ��¼ʱ������Ƚϡ�
	if(tm_acc_changed == 0)
	{
		tm_acc_changed = systick;
	}
	else
	{
		// ����ϴ�acc�仯����ǰ���acc�仯֮���ʱ�����Ƿ�С��Ԥ�����ޣ��ǵĻ���Ϊ�仯��Ч������˸��ơ�
		if((systick - tm_acc_changed) < MAX_INTERVAL_ACC_CHANGED*1000/SYSTICK_PERIOD)
		{
			// accû�仯һ�Σ���ؼ�����������Ȼ����ݼ�������ż�Ե�����Ϩ���ƣ��ɴ˷����û�
			// �ڲ���acc�����Ƿ���ȷ�Ĺ�����ͨ���۲��Ƶ�����仯���ж�acc�����Ƿ���ȷ��ͬʱ��
			// ��Ҳ��ͨ�����ز���acc�仯�����Ϩ������Ӱ�����ָʾ�Ƶı仯��
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

		// ����tm_acc_changed
		tm_acc_changed = systick;
	}
}

// acc��Դ��ͨ/�Ͽ�����жϡ�
// ע: ��Դת�Ӱ���acc��Դ���ϵķ�ѹ����Ϊ120kʱ��12v acc��������£�acc��������ϵĵ�ѹΪ3.12v���ҡ�12v acc�Ͽ�����£�acc��������ϵĵ�ѹΪ0.04v��
void isr_accpwr_detected(void)
{
	if(MCU_GPIO_READ(GPIO_ACCPWR_DET) == ON)	// ACC��Դ��ͨ�����
	{		
		sts_power |= (1<<BIT_STS_POWER_ACC);

		printf("acc power on.\r\n");
	}
	else										// ACC��Դ�Ͽ���
	{
		sts_power &= ~(1<<BIT_STS_POWER_ACC);

		printf("acc power off.\r\n");
	}

	// ���acc����/�Ͽ��ı仯�Ƿ��ڶ�ʱ���ڷ�����
	check_acc_changed();
}
#endif

#ifdef USING_DEV_GSM
// ����GSM_RING�ж��¼���
void isr_gsmring_asserted(void)
{
	// GSMģ��RING�жϴ������ڲ��ܳ�������printf������ʱ����䣬�����Ӱ������ŵ�GSM
	// �������ݽ��գ�����������ݲ��ֶ�ʧ������(115200bps�������£�һ���ֽڵĳ���Ϊ70us)!!!

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

	// ��GSMģ���ʼ���������GSM_RING�жϲű�����
	if(is_gsm_ready == TRUE)
	{
		// ����GSMģ��RING�ж��¼��������־
		is_gsmring_pending = FALSE;
	}	
}
#endif

// ���ڲ������ٶȴ���������ļ��ٶ���ֵ��
// �ն����泯�Ͼ�ֹʱ��������ٶ���ֵ: x value=0.08, y value=0.10, z value=0.99
// �ն����泯�¾�ֹʱ��������ٶ���ֵ: x value=0.08, y value=0.06, z value=-1.03
void isr_gsrval_to_measure(void)
{
	// printf("to measure G-Sensor 3-axis value when systick = %d.\r\n", systick);

	// printf("x value=%.2f, y value=%.2f, z value=%.2f\r\n", gsr_read_valx(), gsr_read_valy(), gsr_read_valz());
	
	// ����˶��¼�
	if(sw_detect_motion == ON)
	{
		// printf("to detect motion event.\r\n");
		
		// �����ٶȴ�������������ٶ���ֵ�仯�Ƿ񳬹�����
		regx = gsr_read_reg(AXIS_X);
		regy = gsr_read_reg(AXIS_Y);
		regz = gsr_read_reg(AXIS_Z);

		// printf("regx = %d, regy = %d, regz = %d\r\n", regx, regy, regz);
		
		if(!(regx_pre == 0 && regy_pre == 0 && regz_pre == 0))
		{
			// ��������һ��ļ��ٶ���ֵ�仯�������޾���Ϊ�˶�����
			if((__abs16(regx-regx_pre) > err_detect_motion) || (__abs16(regy-regy_pre) > err_detect_motion) || (__abs16(regz-regz_pre) > err_detect_motion))
			{
				cnt_gsensor_motion++;

				// printf("cnt_gsensor_motion = %d\r\n", cnt_gsensor_motion);

				if(sw_detect_still == ON)
				{
					// ��⵽�˶�ʱ����ֹ��ⶨʱ����λ
					swtimer_still_detection = DEF_SWTIMER_STILL_DETECTION;
				}	

				event_handler_motion();	
			}
		}

		// ����ǰ������ٶ���ֵ����Ϊ�ϴ�������ٶ���ֵ
		regx_pre = regx;
		regy_pre = regy;
		regz_pre = regz;
	}		

	// ��龲ֹ�¼�
	if(sw_detect_still == ON)
	{				
		// printf("to detect still event.\r\n");
		
		// ��ֹ��ⶨʱ���ݼ�
		if(swtimer_still_detection > 0)
		{
			// printf("swtimer_still_detection = %d\r\n", swtimer_still_detection);

			// ÿ400ms���һ��������ٶ���ֵ
			swtimer_still_detection -= DEF_USERTIMER_PERIOD*2;

			// ����ֹ��ⶨʱ����������⵽��ֹ
			if(swtimer_still_detection <= 0)
			{
				cnt_gsensor_still++;

				event_handler_still();
			}
		}
	}

}

// ���ڲ�����صĵ�ѹ��
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
	
	// �ⲿ��Դ�Ͽ�ʱ������Ƿ��ѹ����
	// if(!(sts_power & (1<<BIT_STS_POWER_EXT)))
	if(1)
	{
		// ��ص�ѹ����
		if(de_batvol < MIN_BATVOL_FOR_WORKING)
		{	
			
		} 
		// ��ص�ѹ�����ٽ�״̬
		else if(de_batvol < NML_BATVOL_FOR_WORKING)
		{

		}
		// ��ص�ѹ����
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
