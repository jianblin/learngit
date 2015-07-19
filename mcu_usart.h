#ifndef __USART1_H
#define	__USART1_H

#include "stm32f10x.h"
#include "common.h"
#include <stdio.h>

#define countof(a)   (sizeof(a) / sizeof(*(a)))		

// for USART1
#define TxBuf1Size	 		256			// 在usart1波特率为115200、GPS串口波特率为9600情况下，256字节的usart1发送缓冲即可满足转发要求	
#define RxBuf1Size   			2048		// usart1一般作为输入系统命令用，因此接收缓冲长度可设置为较短

extern unsigned char 			TxBuf1[TxBuf1Size];	   
extern unsigned char 			RxBuf1[RxBuf1Size];

extern volatile unsigned int 	TxCnt1_wr; 
extern volatile unsigned int 	TxCnt1_rd;
extern volatile unsigned int 	RxCnt1_wr; 
extern volatile unsigned int 	RxCnt1_rd;

#define COM1_RX_RD(n)		(RxBuf1[(n)&(RxBuf1Size-1)])
#define COM1_RX_WR(n, v)		(RxBuf1[(n)&(RxBuf1Size-1)] = v)

#define COM1_TX_RD(n)		(TxBuf1[(n)&(TxBuf1Size-1)])
#define COM1_TX_WR(n, v)		(TxBuf1[(n)&(TxBuf1Size-1)] = v)

void 	usart1_init(void);
void 	usart1_tx_fill(char* data, unsigned int size);
void	usart1_tx_start(void);

// for USART2
#define TxBuf2Size	 		1024		// fixed
#define RxBuf2Size   			1024		// adjusted

extern unsigned char 			TxBuf2[TxBuf2Size];	   
extern unsigned char 			RxBuf2[RxBuf2Size];

extern volatile unsigned int 	TxCnt2_wr; 
extern volatile unsigned int 	TxCnt2_rd;
extern volatile unsigned int 	RxCnt2_wr; 
extern volatile unsigned int 	RxCnt2_rd;

extern BOOL						usart3_tx_busy;

#define GPS_RX_RD(n)			(RxBuf2[(n)&(RxBuf2Size-1)])
#define GPS_RX_WR(n, v)		(RxBuf2[(n)&(RxBuf2Size-1)] = v)

#define GPS_TX_RD(n)			(TxBuf2[(n)&(TxBuf2Size-1)])
#define GPS_TX_WR(n, v)		(TxBuf2[(n)&(TxBuf2Size-1)] = v)

void 	usart2_init(void);
void 	usart2_tx_fill(unsigned char* data, unsigned int size);
void 	usart2_tx_start(void);

//  for USART3
#define TxBuf3Size	 		1024		// fixed
#define RxBuf3Size   			2048		// 由于GPRS下载的星历数据一般在3K字节左右，因此GPRS接收缓冲的长度应不小于此长度

extern unsigned char 			TxBuf3[TxBuf3Size];	   
extern unsigned char 			RxBuf3[RxBuf3Size];

extern volatile unsigned int 	TxCnt3_wr; 
extern volatile unsigned int 	TxCnt3_rd;
extern volatile unsigned int 	RxCnt3_wr; 
extern volatile unsigned int 	RxCnt3_rd;

#define GSM_RX_RD(n)		(RxBuf3[(n) & (RxBuf3Size-1)])
#define GSM_RX_WR(n, v)		(RxBuf3[(n) & (RxBuf3Size-1)] = v)

#define GSM_TX_RD(n)			(TxBuf3[(n) & (TxBuf3Size-1)])
#define GSM_TX_WR(n, v)		(TxBuf3[(n) & (TxBuf3Size-1)] = v)

extern SWITCH				sw_gps_forward;
extern BOOL					is_gps_forwarding;
extern BOOL					is_gsm_forwarding;
extern BOOL					is_gsm_downloading;

void 	usart1_tx_fill(char* data, unsigned int size);
void 	usart1_tx_start(void);

void 	usart2_tx_fill(unsigned char* data, unsigned int size);
void 	usart2_tx_start(void);
void 	usart2_send_byte(unsigned char byte);
// void 	usart2_debug_mode(void);

void 	usart3_send_int(char* data, unsigned int size);
void 	usart3_send_poll(char* data, unsigned int size);
void 	usart3_tx_start(void);

void 	usart2_redir_usart1(void);
void 	usart3_redir_usart1(void);

void 	usart_stop(void);
void 	usart_start(void);

void 	USART_Configure(void);
#endif /* __USART1_H */
