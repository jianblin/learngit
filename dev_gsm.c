/*GSMÓ²¼ş¹ÜÀí¼°ÏûÏ¢´¦Àí*/
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

T_GPRS_CONNECTION	tcp_conn[MAX_NUM_TCP_CONN];				// Éè±¸Òª·ÃÎÊµÄtcp·şÎñÆ÷²ÎÊıÊı×é

char	 				gsm_telecode[MAX_LEN_GSM_TELECODE+1];		// ·¢ËÍPDU¶ÌĞÅÊ±ĞèÒªµç»°ºÅÂëµÄ¹ú¼Ò/µØÇøÇøºÅ(ÈçÖĞ¹úÎª86£¬+·Ç±ØĞè)
char					gsm_sca[MAX_LEN_PN+1];						// ¶ÌĞÅÖĞĞÄ
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
char 		de_gsm_cell[MAX_NUM_GSM_CELL][MAX_NUM_GSM_CELL_DMN][MAX_LEN_GSM_CELL_DMN+1];		// ÓÃÓÚ±£´æGSM»ùÕ¾ĞÅÏ¢µÄÈ«¾ÖÊı×é(×î¶à7¸ö»ùÕ¾¡¢Ã¿¸ö»ùÕ¾¹Ì¶¨7¸öÓò¡¢Ã¿¸öÓòµÄ³¤¶È²»³¬¹ı6×Ö½Ú)
char			de_gsm_pnin[MAX_LEN_PN+1];				// µ±Ç°¼ì²âµ½µÄÀ´µçºÅÂëĞÅÏ¢
char			de_gsm_dtmf[MAX_LEN_GSM_DTMF+1];		// µ±Ç°¼ì²âµ½µÄDTMFĞÅÏ¢

char			gsm_imei[STD_LEN_GSM_IMEI+1];				// GSMÄ£¿éµÄIMEI´®

STATUS		sts_gsm_power = OFF;

BOOL		is_gsm_ready  = FALSE;					// GSMÄ£¿éÊÇ·ñÕı³£³õÊ¼»¯µÄ±êÖ¾
BOOL		is_gsm_calling = FALSE;					// gsmÊÇ·ñ´¦ÓÚÍ¨»°×´Ì¬(Í¨»°×´Ì¬ÏÂ²»ÄÜ½¨Á¢tcpÁ¬½Ó£¬Ò²²»ÄÜ·¢ËÍtcpÊı¾İ)

BOOL		is_gsmring_pending;						// gsmÄ£¿éµÄringÖĞ¶ÏÊÂ¼şÊÇ·ñ´ı´¦Àí
int			cnt_gsmring_asserted;					// GSMÄ£¿éRINGÖĞ¶Ï²úÉúµÄ´ÎÊı(GSMÄ£¿éÉÏµçºó»á×Ô¶¯²úÉúÒ»¸öRINGÖĞ¶Ï)

int			cnt_gsm_recovered;						// GSMÄ£¿é¹ÊÕÏ»Ö¸´µÄ´ÎÊı

// dtmf related
int			swtimer_input_dtmf;						// dtmf¼ì²â´ò¿ªºó½ÓÊÕÓÃ»§ÊäÈëdtmfÃÜÂëµÄ³¬Ê±¼ì²â¶¨Ê±Æ÷
int			swtimer_set_micmute_off;				// ÓÃ»§ÊäÈëdtmfÃÜÂëÕıÈ·ºó¹Ø±ÕßäÍ·¾²ÒôµÄÑÓ³Ù¶¨Ê±Æ÷
BOOL		is_dtmf_detection_enabled;				// µ±Ç°DTMF¼ì²âÊÇ·ñ´ò¿ª
STATUS		sts_dtmf_command;						// DTMFÃüÁî´¦ÀíµÄµ±Ç°×´Ì¬
char			dtmf_password[MAX_LEN_GSM_DTMF+1];		// DTMF½»»¥ÃÜÂë
int			times_input_dtmf_password;				// ÒÑ¾­ÊäÈëDTMF½»»¥ÃÜÂëµÄ´ÎÊı

// ³õÊ¼»¯GSMÄ£¿é(Ó²¼şÉÏµç¡¢¹¤×÷²ÎÊıµÈ³õÊ¼»¯)¡£
// ×¢: Èç¹ûÍêÈ«¶Ï¿ªGSMÌìÏß£¬GSMÄÜÕı³£³õÊ¼»¯µ«rssiÖµÎª99£¬³õÊ¼»¯Íê³ÉºóÔÙ²éÑ¯£¬rssiÎª2-3£¬
//     ÔÚ´ËÇé¿öÏÂ£¬¸øÖÕ¶Ë·¢ËÍ¶ÌĞÅ£¬·¢ËÍ·½ÊÕµ½¶ÌĞÅÎ´µ½´ïµÄ»Ø¸´¡£
ERR_NO gsm_init(void)
{		
	char		str[64+1];

	int		rssi = -1;	

	int  ret;
	printf("to initialize GSM module.\r\n");

	// ³õÊ¼»¯gsmÓ²¼şÇ°½«is_gsm_readyÉèÖÃÎªFALSE
	is_gsm_ready 			= FALSE;

	// ³õÊ¼»¯gsmÓ²¼şÇ°½«is_gsm_callingÉèÖÃÎªFALSE
	is_gsm_calling 			= FALSE;

	is_gsmring_pending 		= FALSE;	
	cnt_gsmring_asserted 	= 0;	

	// ³õÊ¼»¯dtmfÃüÁîÏà¹Ø±äÁ¿
	sts_dtmf_command 		= STS_DTMF_COMMAND_CALL_WAITING;
	times_input_dtmf_password 	= 0;
	
	// ³õÊ¼»¯GSM´®¿ÚÊÕ·¢Ïà¹Ø±äÁ¿ºÍ»º³å
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

	// ÏÈÇ¿ĞĞ¹Ø¶ÏGSMµçÔ´£¬ÒÔ±ãÔÚÏµÍ³Òì³£ÖØÆô¶øÎ´ÄÜÔÚ¹Ø»úÇ°¹Ø¶ÏGSMµçÔ´Ê±Çå³ıGSM¹¤×÷×´Ì¬
//	gsm_power_down();

	// ´ò¿ªGSMµçÔ´
//	gsm_power_up();	

	// ¸øGSMÄ£¿éÉÏµç
//	gsm_onoff();  	

	// ÑÓÊ±3ÃëµÈ´ıGSMÓ²¼ş³õÊ¼»¯
	delay_100ms(30);

	// µÈ´ıGSMÄ£¿éËÑÍø¡¢×¢²á
	printf("to detect SIM card...\r\n");	
	

//	ret=gsm_send_at("AT\r\n", "OK", 2);
//	 if(ret != OK)
//	 	printf("AT ERROR.......%d\r\n",ret);
//
//	//ATE0¹Ø±Õ»ØÏÔ
//	ret=gsm_send_at("ATE0\r\n", "OK", 8);
//	 if(ret != OK)
//	 	printf("ATE0 ERROR.......%d\r\n",ret);
	// ¼ì²éSIM¿¨ÊÇ·ñ²åÉÏ
	if(gsm_check_sim() != OK)
	{
		printf("SIM card not detected.\r\n");

//		gsm_exit();
		
		return ER_GSM_INIT_SIM;		// sim¿¨Î´¼ì²âµ½
	}

	// µÈ´ıGSMÄ£¿éËÑÍø¡¢×¢²á
	printf("to wait for GSM registration...\r\n");	
	
	ret=gsm_send_at("ATE0\r\n", "OK", 8);
	 if(ret != OK)
	 	printf("ATE0 ERROR.......%d\r\n",ret);
	// ²»¹ÜSIM¿¨ÊÇ·ñ²åÈë£¬Ä£¿éÉÏµçºó¶¼»áÊä³ö+EINDÌØÕ÷×Ö·û´®£¬
	// ÒÔ´Ë¼ì²âÄ£¿éÊÇ·ñ»ù±¾Õı³£(M660+Ä£¿éµÄ×¢²áÍøÂçÊ±¼äÒ»°ã¹Ì¶¨Îª10Ãë)¡£	
	/*
	if(gsm_find_pattern("+EIND: 1\r\n", TO_GSM_REGISTRATION) != OK)
	{	
		printf("+EIND: 1 not found.\r\n");

		gsm_exit();		
		
		return ER_GSM_INIT_REGISTRATION;		// ÍøÂç×¢²áÒì³£
	}	
	*/

#if 1
	// ²éÑ¯GSM¹Ì¼ş°æ±¾	
	if(gsm_get_swversion((char*)str, 64) == OK)
	{
		printf("GSM firmware version: %s.\r\n", str);
	}
	else
	{
		printf("failed to get GSM firmware version.\r\n");

//		gsm_exit();		
		
		return ER_GSM_INIT_SWVERSION;			// ²éÑ¯GSM¹Ì¼ş°æ±¾Òì³£
	}
#endif	

#if 1
	// ²éÑ¯IMEI´®(GSMÄ£¿é¸ÕÉÏµçºó²éÑ¯IMEI¾­³£Ê§°Ü£¬Ô­Òò²»Ïê)
	if(gsm_get_imei((char*)gsm_imei, STD_LEN_GSM_IMEI) != OK)
	{
		printf("failed to get GSM IMEI.\r\n");	

//		gsm_exit();

		return ER_GSM_INIT_IMEI1;
	}
	else
	{
		// ¼ì²éimei³¤¶ÈÊÇ·ñÎª15Î»
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
// SMS»·¾³³õÊ¼»¯
	// ½«GSMÄ£¿éµÄ¶ÌÏûÏ¢Ä£Ê½ÉèÖÃÎªÎÄ±¾Ä£Ê½
	if(gsm_sms_mode_txt()!= OK)
	{
		gsm_exit();

		printf("failed to set SMS mode to text.\r\n");
		
		return ER_GSM_INIT_SMS_MODE;
	}
#endif
	// ¶ÌÏûÏ¢Êä³öÄ£Ê½²ÉÓÃÄ¬ÈÏÉèÖÃ(ÒÔ±ãGSMÄ£¿é½ÓÊÕµ½¶ÌĞÅÊ±ÄÜÊä³ö+SMSFLAGÌáÊ¾ÏûÏ¢)£¬²ÉÓÃÆäËûÉèÖÃ·´¶ø²»ÄÜÕı³£²úÉúringÖĞ¶Ï
#if 0	
	if(gsm_send_at("AT+CNMI=1,1,0,0,0\r\n", "OK", 2) != OK)		// ½ûÖ¹½ÓÊÕµ½µÄ¶ÌĞÅ×Ô¶¯Êä³ö£¬ÒÔÊ¹ÉÏ²ãÓ¦ÓÃÔÚÊä³ö¶ÌĞÅÇ°µÍRINGÒı½Å
	{
		printf("failed to set CNMI.\r\n");

		gsm_exit();
		
		return ER_GSM_INIT_SMS_SETTING;
	}
	// printf("enable outputing received SMS automatically.\r\n");
#endif	
#if 0
	// É¾³ıµ±Ç°¶ÌĞÅ´æ´¢Æ÷ÖĞµÄËùÓĞ¶ÌĞÅ£¬ÒÔÃâ¶ÌĞÅ½ÓÊÕÒì³£
	if(gsm_send_at("AT+CMGD=\"DEL ALL\"\r\n", "OK", 5) != OK)
	{
		gsm_exit();

		printf("failed to delete all sms in current memory.\r\n");
		
		return ER_GSM_INIT_SMS_DELETE;
	}

// Call»·¾³³õÊ¼»¯
	// Ê¹ÄÜÀ´µçºÅÂë×Ô¶¯Êä³ö
	if(gsm_send_at("at+clip=1\r\n", "OK", 2) != OK)
	{
		gsm_exit();

		printf("failed to enable phone number of incoming call output automatically.\r\n");
		
		return ER_GSM_INIT_CLIP;
	}
#endif
// GPRS»·¾³³õÊ¼»¯
	// ÉèÖÃÊı¾İÊÕ·¢¶¼ÎªASCIIÄ£Ê½(¼´´«Í³µÄ¶ş½øÖÆÄ£Ê½)
	/*
	if(gsm_send_at("AT+DATAFORMAT=1,1\r\n", "OK", 2) != OK)
	{
		gsm_exit();

		printf("failed to set data mode to ASCII.\r\n");
		
		return ER_GSM_INIT_DATAFORMAT;
	}
	*/
	// ²éÑ¯RSSI
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

	// ³õÊ¼»¯gsm³É¹¦ºó½«is_gsm_readyÉèÖÃÎªTRUE
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

// °´Õı³£Á÷³Ì¹Ø¶ÏGSMÄ£¿é¡£
void gsm_exit(void)
{
	gsm_onoff();
				
	gsm_power_down();
}

/**********************************************************************************************************************
										 		GSMÓ²¼şIO²Ù×÷º¯Êı
***********************************************************************************************************************/

// ÖØÆôGSMÄ£¿é(À­µÍSYSRST½Å50msÒÔÉÏ)¡£
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
>> SIM¿¨²åÈë×´Ì¬ÏÂ£¬M660+ÉÏµçºó»á×Ô¶¯Êä³öÈçÏÂÏûÏ¢:

+EIND: 128

+EUSIM: 0

+STKPCI: 0,"D081B6810301250082028182850B80795E5DDE884C592957308F0A01808F7B677E95EE50198F0A038077ED4FE17FA453D18F0A0480670065B063A883508F0A06804E1A52A17CBE90098F10078065E07EBF97F34E504FF14E5090E88F0E08800031003300394E9280547F518F0809808D224FE1901A8F0E0A8079FB52A84F1860E04E13533A8F0E0B8079FB52A875355B50554652A18F120C806211768400530049004D84254E1A53858F0E058000530049004D53614FE1606F"

+EIND: 2		// ´ÓÊä³ö+STKPCIµ½Êä³ö+EIND: 2Ö®¼ä¿ÉÄÜ¼ä¸ô³¤´ï3Ãë

+EIND: 1		// ´ÓÊä³ö+EIND: 2µ½Êä³ö+EIND: 1Ö®¼ä¿ÉÄÜ¼ä¸ô³¤´ï5Ãë(Ê±¼ä³¤¶ÈÈ¡¾öÓÚSIM¿¨µÄÀàĞÍºÍÏàÓ¦µÄ³É¹¦×¢²áµ½ÍøÂçµÄÊ±¼ä)

>> SIM¿¨Î´²å×´Ì¬ÏÂ£¬M660+ÉÏµçºó»á×Ô¶¯Êä³öÈçÏÂÏûÏ¢:

+EIND: 128

+EIND: 2

+EIND: 1		// ³õÊ¼»¯Íê³Éºó×Ô¶¯Êä³öµÄ×îºóÒ»ÌõÏûÏ¢

*/
// ¸øGSMÄ£¿éÉÏµç/ÏÂµç(À­µÍ/À­¸ßPOWERONÒı½Å3sÒÔÉÏ)¡£
void gsm_onoff(void)
{	
	//MCU_GPIO_LOW(GPIO_GSM_ONOFF);
	
	//delay_100ms(30);	

	//MCU_GPIO_HIGH(GPIO_GSM_ONOFF);

	//printf("GSM On/OFF.\r\n");
}

// ´ò¿ª¸øGSMÄ£¿éµÄ¹©µç¡£
void gsm_power_up(void)
{
	// GPIO_ResetBits(GPIOB,GPIO_Pin_13);
	MCU_GPIO_HIGH(GPIO_GSM_PWR);
	
	delay_100ms(3);	

	sts_gsm_power = ON;

	printf("GSM powered up.\r\n");
}

// ¹Ø±Õ¸øGSMÄ£¿éµÄ¹©µç¡£
void gsm_power_down(void)
{
	// GPIO_SetBits(GPIOB,GPIO_Pin_13);
	MCU_GPIO_LOW(GPIO_GSM_PWR);
	
	delay_100ms(3);	

	sts_gsm_power = OFF;

	printf("GSM powered down.\r\n");
}

/*
¶ÔGSMÄ£¿éµÄĞİÃßºÍ»½ĞÑ²Ù×÷£¬²ÉÈ¡"Ä¬ÈÏĞİÃß¡¢Ö÷¶¯Ê¹ÓÃÇ°»½ĞÑ¡¢Ö÷¶¯Ê¹ÓÃºóĞİÃß¡¢±»¶¯½ÓÊÕ²»´¦Àí"µÄÔ­Ôò¡£
------------------------------------------------------------------------------------------------
¿ØÖÆÄ£¿é½øÈë´ı»úÄ£Ê½µÄ»ù±¾Á÷³Ì£º
1¡¢±£³ÖÄ£¿éµÄ DTR ÊäÈëÎª¸ßµçÆ½£¬Í¨¹ıAT Ö¸Áî½«Ä£¿éÉèÖÃÎªÔÊĞí½øÈëĞİÃßÄ£Ê½£¬²Î¿¼Ö¸
Áî£ºat+enpwrsave¡£
2¡¢½«Ä£¿éµÄ DTR ÊäÈëÖÃµÍ£¬Ó²¼ş¿ØÖÆÄ£¿é½øÈëµÍ¹¦ºÄ×´Ì¬¡£Í¨³£Ä£¿é»áÔÚ2 Ãë×óÓÒ½øÈë
´ı»ú¡£ÔÚ´ı»úÄ£Ê½ÏÂ£¬Ä£¿éµÄ´®¿ÚÊÇ¹Ø±ÕµÄ£¬Ã»ÓĞÏìÓ¦¡£ÔËĞĞµÆÒ²»áÍ£Ö¹ÉÁË¸¡£
Ä£¿éÖ»ÓĞÔÚ¿ÕÏĞÊ±²Å»á½øÈë´ı»úÄ£Ê½£¬Èç¹ûÓĞÊı¾İ½»»¥Î´½áÊø£¬²»»á½øÈë´ı»ú¡£
3¡¢Èç¹û±¾¶ËÓĞÊı¾İ»òÕßºô½ĞµÈÖ÷½ĞÒµÎñ£¬¿ÉÒÔ½« DTR ÖÃ¸ß£¬Ä£¿éÁ¢¼´ÍË³ö´ı»úÄ£Ê½£¬½ø
ÈëÕı³£Ä£Ê½£¬´®¿Ú´ò¿ªÏìÓ¦AT Ö¸Áî¡£ÔÚÖ÷½ĞÒµÎñ´¦ÀíÍê±Ïºó£¬Íâ²¿CPU ÔÙ½«DTR ÖÃ
µÍ£¬Ä£¿é½øÈë´ı»úÄ£Ê½¡£
4¡¢ÔÚ´ı»ú×´Ì¬ÏÂ£¬Èç¹ûÄ£¿éÓĞ±»½ĞÒµÎñ£¬±ÈÈçÀ´µç¡¢À´¶ÌĞÅ¡¢·şÎñÆ÷À´µÄÊı¾İ£¬Ä£¿é»áÁ¢
¿ÌÍË³ö´ı»úÄ£Ê½£¬²¢Í¨¹ı´®¿ÚÊä³öÀ´µçĞÅÏ¢£¬Íâ²¿CPU ÔÚ¼ì²âµ½´®¿ÚĞÅÏ¢ºó£¬½¨ÒéÏÈ
½«DTR ÖÃ¸ß£¬ÔÙ´¦ÀíÀ´µç¡¢Êı¾İµÈ¡£´ı´¦ÀíÍê±Ïºó£¬½«DTR ÖÃµÍ£¬Ê¹Ä£¿é½øÈë´ı»úÄ£
Ê½¡£Èç¹ûÀ´µçÊ±£¬DTR Ã»ÓĞÖÃ¸ß£¬ÇÒ´®¿ÚÃ»ÓĞĞÅÏ¢£¬ÔòÄ£¿é»áÔÚ2~30 Ãë×óÓÒ×Ô¶¯½øÈë
´ı»úÄ£Ê½¡£
*/

/*
at+enpwrsave=1
OK

at+enpwrsave=1
CME ERROR:<error>

at+enpwrsave?
+ENPWRSAVE:1
*/
// ½«Ä£¿éĞİÃß¡£
int gsm_sleep(void)
{	
	// À­¸ßDTRÒı½Å
//	MCU_GPIO_HIGH(GPIO_GSM_DTR);

	// ÔÊĞíÄ£¿é½øÈëĞİÃßÄ£Ê½
//	if(gsm_send_at("AT+ENPWRSAVE=1\r\n", "OK", 2) != OK)
//	{	
//		return NG;
//	}

	// À­µÍDTRÒı½Å
//	MCU_GPIO_LOW(GPIO_GSM_DTR);

	// Ä£¿éÖ»ÓĞÔÚ¿ÕÏĞÊ±²Å»á½øÈë´ı»úÄ£Ê½£¬Èç¹ûÓĞÊı¾İ½»»¥Î´½áÊø£¬²»»á½øÈë´ı»ú¡£

//	printf("gsm sleep.\r\n");

	return OK;
}

// Ç¿ĞĞ»½ĞÑGSMÄ£¿é(½öÔÚÄ£¿éÖ÷¶¯·¢ÆğÊı¾İ´«Êä»ò¶ÌĞÅ»òµç»°Ç°µ÷ÓÃ)¡£
void gsm_wakeup(void)
{		// ¼ì²éµ±Ç°DTRÊÇ·ñ±»À­µÍ£¬±»À­µÍÔòËµÃ÷Ö®Ç°Ä£¿é±»ÊÖ¶¯ÖÃÓÚĞİÃß×´Ì¬
//	if(MCU_GPIO_READ(GPIO_GSM_DTR)== 0)
//	{
		// À­¸ßDTRÒı½Å
//		MCU_GPIO_HIGH(GPIO_GSM_DTR);
		
//		delay_100ms(20);

//		printf("gsm woke up.\r\n");
//	}
}

// ÔÚM660+´®¿ÚÍ¨Ñ¶Òì³£(¶àÓÉGPRSÁ¬½ÓµÈÉæ¼°ÍøÂç×´¿öµÄºÄÊ±ÃüÁîËùÒıÆğ)Çé¿öÏÂ£¬
// ³¢ÊÔ½«Ä£¿é»Ö¸´µ½ÉÏµçºóµÄ³õÊ¼×´Ì¬(ÈôÏµÍ³Á¬Ğø¼ì²âµ½Ä£¿é³öÏÖÈı´ÎÒÔÉÏµÄÒì³££¬
// ÔòÈÏÎªÄ£¿é³öÏÖÖØ´ó¹ÊÕÏ£¬È»ºóÆô¶¯Ó²¼ş¹ÊÕÏ±¨¾¯»úÖÆ)¡£
int gsm_recover(void)
{
	printf("to recover GSM No.%d times...\r\n", cnt_gsm_recovered+1);
	
	if(gsm_init() == OK)
	{	
		printf("recover GSM module OK.\r\n");

		// »Ö¸´gsm³É¹¦ºó½«cnt_gsm_recoveredÖØÖÃÎª0
		cnt_gsm_recovered = 0;
			
		return OK;
	}
	// gsm³õÊ¼»¯Ê§°Ü¿ÉÄÜÊÇÓÉÓÚÍøÂçĞÅºÅ²»ºÃ¡¢sim¿¨Î´¼ì²âµ½¡¢sim¿¨Ç··ÑÒÔ¼°gsmÄ£¿é´®¿ÚÍ¨Ñ¶¹ÊÕÏÒıÆğµÄ
	else
	{
		printf("recover GSM module NG.\r\n");

		// cnt_gsm_recovered¼ÆÊıÆ÷µİÔö
		cnt_gsm_recovered++;

		// ¼ì²éÁ¬Ğø»Ö¸´gsmµÄ´ÎÊıÊÇ·ñ³¬ÏŞ
		if(cnt_gsm_recovered >= MAX_TIMES_GSM_RECOVER)
		{
			printf("failed to recovery gsm continuously %d times.\r\n", cnt_gsm_recovered);	
		}
		
		return NG;
	}
}

/**********************************************************************************************************************
											GSM´®¿Ú½ÓÊÕ»º³å²Ù×÷º¯Êı
***********************************************************************************************************************/

// ÔÚGSM UART½ÓÊÕ»º³åÖĞ¡¢ÔÚ¸ø¶¨µÄÊ±¼äÄÚ´Óµ±Ç°¶ÁÖ¸ÕëÎ»ÖÃ¿ªÊ¼ËÑË÷Ö¸¶¨µÄPattern¡£
ERR_NO gsm_find_pattern(char* ptn, unsigned int to)
{
	int 			i = 0;
    unsigned int 	len = 0;
	unsigned int	RxCnt3_wr_backup = RxCnt3_wr;
	
	to = systick+to*1000/SYSTICK_PERIOD;

	// ¼ì²éÊäÈë²ÎÊıµÄºÏ·¨ĞÔ
	len = strlen(ptn);
	
	if(len <= 0)
	{
		return ER_GSM_PATTERN_NULL;
	}

	while(systick < to)
	{  		
		// ²éÕÒÄ£Ê½×Ö·û´®µÄ¹ı³Ì¿ÉÄÜºÄÊ±½Ï³¤£¬Òò´Ë²éÕÒ¹ı³ÌÖĞÓ¦¼°Ê±Î¹¹·
		WATCHDOG_RELOAD();

		// ¼ì²é´®¿ÚÃüÁîÊäÈë
		check_buf_com();	
		// check_que_cmd(CHECK_MODE_CONTINUOUS);
		
        if((RxCnt3_wr - RxCnt3_rd) >= len)
        {   			
        	// Îª¼ò»¯Æğ¼û£¬ÕâÀï²ÉÓÃÁËÖğ×Ö½ÚÒÆ¶¯µÄ»¬¶¯´°¿Ú±È½Ï·¨£¬¶øÎ´²ÉÓÃKMPµÈ¿ìËÙËÑË÷Ëã·¨£¬
        	// Êµ¼ÊÉÏ¶ÔÓÚGSM½ÓÊÕ»º³åµÄ¼ì²âÒ»°ã¶¼ÊÇÓĞÄ¿µÄµÄ£¬¼´ÌØÕ÷×Ö·û´®³öÏÖµÄÎ»ÖÃ¾àÀëµ±Ç°
        	// ¶ÁÓÎ±êÒ»°ã¶¼²»Ô¶£¬Òò¶ø×Ö·û´®Æ¥ÅäµÄ¹ı³ÌºÄÊ±ÓĞÏŞ¡£
        	for(i = 0; i < len; i++)
            {
            	if(GSM_RX_RD(RxCnt3_rd+i) != ptn[i])
            	{
            		break;
            	}
            }
            
            if(i == len)
            {
            	RxCnt3_rd += i;		// ¶ÁÓÎ±êºóÒÆÏàÓ¦³¤¶È
            	
            	return OK;
            }
            else
            {
            	RxCnt3_rd++;		// ´ÓÏÂÒ»×Ö½Ú´¦ÖØĞÂËÑË÷Ö¸¶¨µÄPattern
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

// µÈ´ı½ÓÊÕ»º³åÄÚ½ÓÊÕµ½Ö¸¶¨³¤¶ÈµÄÎ´¶ÁÊı¾İ£¨´Ëº¯Êı¶àÓÃÓÚĞèÒªµÈ´ı½ÓÊÜ¸ü¶àÊı¾İµÄÊ±ºò£¬´Ó¶øÎªºóĞø´¦Àí»ıÀÛĞÂµÄÊı¾İ£©¡£
ERR_NO gsm_wait_output(unsigned int len, unsigned int to)
{
	unsigned int	RxCnt3_wr_backup = RxCnt3_wr;
		
	to  = systick + to*1000/SYSTICK_PERIOD;

    while(systick < to)
    {
		WATCHDOG_RELOAD();

		// µÈ´ıgsm´®¿ÚÊä³öÊ±¼ì²é´®¿Ú1ÓĞÎŞÃüÁîÊäÈë¡£
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

// ´Óµ±Ç°¶ÁÓÎ±êÎ»ÖÃ¿ªÊ¼ÌáÈ¡Êı×Ö×Ö·û´®²¢½«Æä×ª»¯ÎªÊıÖµÒÔ·µ»Ø(Ö§³Ö¸ºÊıÕûÊıµÄÌáÈ¡)¡£
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
	
 	// ÏÈ¶¨Î»µ½Êı×Ö´®µÄÆğÊ¼×Ö·û£¨'-'»ò'0~9'£© 
    while(systick < to)
    {    	
		WATCHDOG_RELOAD();
		
    	if(RxCnt3_rd < RxCnt3_wr)
    	{
	    	ch = GSM_RX_RD(RxCnt3_rd++);		
	    	
			if(ch == '-')
			{
				// Î´¼û²âµ½ÈÎºÎÊı×Ö×Ö·ûÇ°¼ì²âµ½'-'¿ÉÈÏÎªÊÇÊıÖµµÄÕı¸º·ûºÅ
				if(i == 0)
				{
					*val = -1;
				}
				// ·ñÔò¼ì²âµ½'-'¿ÉÈÏÎªÊÇÊı×Ö×Ö·ûÄ©Î²µÄºóÒ»×Ö·û
				else
				{
					str[i] = '\0';		// ÔÚÊı×Ö×Ö·ûÊı×éºóÃæÌí¼Ó×Ö·û´®½áÎ²·ûºÅ

					break;
				}
			}
			else if(ch >= '0' && ch <= '9')
			{
				// ¶ÁÓÎ±êÔİÊ±²»ÒÆ¶¯£¬Òò¸ÕºÃÖ¸ÏòÊı×Ö×Ö·û
				str[i++] = ch;

				if(i >= MAX_DECIMAL_DIGITS_FOR_INT)
				{
					str[i] = '\0';		// ÔÚÊı×Ö×Ö·ûÊı×éºóÃæÌí¼Ó×Ö·û´®½áÎ²·ûºÅ

					break;
				}
			}
			else
			{
				// ¼ì²éstrµÄÓÎ±êÊÇ·ñ´óÓÚ0£¬Èô´óÓÚ£¬ÔòËµÃ÷strÖĞÒÑ¾­Ìî³äÁËÊı×Ö×Ö·û¡¢
				// µ±Ç°¼ì²âµ½·ÇÊı×Ö×Ö·ûµÄ»°ËµÃ÷Êı×Ö×Ö·û´®ÒÑ¾­½áÊø¡£
				if(i > 0)
				{
					str[i] = '\0';		// ÔÚÊı×Ö×Ö·ûÊı×éºóÃæÌí¼Ó×Ö·û´®½áÎ²·ûºÅ

					break;
				}
				// else: ÂÔ¹ıµ±Ç°µÄ·ÇÊı×Ö×Ö·û£¬¼ì²âÏÂÒ»¸ö×Ö·û
	    	}
	    }
    }

	if(RxCnt3_wr == RxCnt3_wr_backup)
	{
		return ER_GSM_UART_RECV_NOTHING;
	}
	else
	{
		// ½«¼ì²âµ½µÄÊı×Ö×Ö·û»á´®×ª»¯ÎªÊıÖµ(×¢ÒâÊıÖµµÄÕı¸ººÅ)
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

// ´Óµ±Ç°¶ÁÓÎ±êÎ»ÖÃ¿ªÊ¼ÌáÈ¡Á¬ĞøµÄÊı×Ö×Ö·û£¨°üÀ¨¸¡µãÊıÖĞµÄ'.'£©¡£
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
	
	// ÏÈ¶¨Î»µ½Ê×¸öÊı×Ö×Ö·û
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

	// ¸üĞÂto
	to -= (systick-systick_backup);

	// ÌáÈ¡Á¬ĞøµÄÊı×Ö×Ö·û
    while(systick < to)
    {        
		WATCHDOG_RELOAD();
		
        if(RxCnt3_rd < RxCnt3_wr)   // GSM UART½ÓÊÕ»º³åµÄ¶ÁÖ¸Õë²»ÄÜ³¬¹ıĞ´Ö¸Õë£¬·ñÔòĞèÒªµÈ´ıĞ´Ö¸Õë¸üĞÂ
        {  
        	ch = GSM_RX_RD(RxCnt3_rd++); // RxBuf3[(RxCnt3_rd++) & (RxBuf3Size-1)];
			
            if(	(ch >= '0' && ch <= '9') || ch == '.')	// Ö§³Ö¸¡µãÊıµÄÌáÈ¡£¨²»Çø·ÖÓĞ¼¸¸ö'.'£©
            {  
                dig[i++] = ch; 	

				if(i >= len)
				{
					dig[i] = '\0';	

					// ½ÓÊÕÂúÄ¿±ê×Ö·û´®ÍË³ö
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
					// ½ÓÊÕÍêÊı×Ö×Ö·ûÍË³ö
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

// ´Óµ±Ç°¶ÁÓÎ±êÎ»ÖÃ¿ªÊ¼ÌáÈ¡ÒÔ'\r\n'½áÎ²µÄ×Ö·û´®¡£
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
        	// ½ÓÊÕµ½»»ĞĞ·ûÍË³ö     
            if(GSM_RX_RD(RxCnt3_rd+0) == '\r' && GSM_RX_RD(RxCnt3_rd+1) == '\n')
            {  
                str[i] = '\0';
            
                RxCnt3_rd += 2;		// Ç°ÒÆ¶ÁÓÎ±ê
            
                return OK;				
            }
            else
            {
            	str[i++] = GSM_RX_RD(RxCnt3_rd++);

				// Ä¿±ê×Ö·û´®½ÓÊÕÂúÍË³ö
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
Ìá½»ÓĞĞ§µÄ»ùÕ¾ĞÅÏ¢¸øGoogle·şÎñÆ÷ºó£¬´Ó·şÎñÆ÷·µ»ØµÄ¶¨Î»ĞÅÏ¢:
d5
{"location":{"latitude":32.117301,"longitude":114.116606,"address":{"country":"China","country_code":"CN","region":"Henan","city":"Xinyang"},"accuracy":1625.0},"access_token":"2:OkivQMlLSpRHuPTx:Beb-gnvVNJBB3-vt"}
0
*/

// ÌáÈ¡Ö¸¶¨·Ö¸ô·ûÄÚµÄ×Ö·û(·Ö¸ô·û¿ÉÄÜÊÇ"»ò|µÈ)¡£
ERR_NO gsm_fetch_spliters(char ptn, char* str, int len, unsigned int to)
{
	unsigned int 	i 			= 0;
	int				spliters 	= 0;

	unsigned int	RxCnt3_wr_backup = RxCnt3_wr;
	
	to  = systick+to*1000/SYSTICK_PERIOD;

	// ÌáÈ¡µÚÒ»¸öË«ÒıºÅÖ®ºó¡¢µÚ¶ş¸öË«ÒıºÅÖ®Ç°µÄ×Ö·û(×¢Òâ×ö³¬Ê±¼ì²âÒÔ¹æ±ÜÒì³£Çé¿ö)
	while(systick < to)
	{
		WATCHDOG_RELOAD();
		
		if(RxCnt3_rd < RxCnt3_wr)
		{			
			// ÀàËÆAT+IPSTATUS=0ÃüÁîµÄÏìÓ¦ĞÅÏ¢ÖĞ×´Ì¬×Ö·û´®Ã»ÓĞÒÔ¶ººÅ¶øÊÇÒÔ»»ĞĞ·û½áÎ²£¬Òò´ËÒ²ÒªÄÜÌáÈ¡³öÀ´
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
					// ½ÓÊÕÍê·Ö¸ô·ûÄÚµÄ×Ö·û´®ÍË³ö
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

					// Ä¿±ê×Ö·û´®½ÓÊÕÂúÍË³ö
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
										 		GSM´®¿ÚÍ¨Ñ¶¹¦ÄÜº¯Êı
***********************************************************************************************************************/

// ·¢ËÍÖ¸¶¨µÄATÃüÁî²¢¼ì²é·µ»ØĞÅÏ¢ÖĞÊÇ·ñ³öÏÖÖ¸¶¨µÄÌØÕ÷×Ö·û´®£¬´Ó¶øÅĞ¶ÏATÃüÁî·¢ËÍºóµÄ·´À¡½á¹û¡£
// ×¢: ÓÉÓÚÃ¿¸öATÃüÁîµÄÖ´ĞĞÊ±¼ä»ò³¤»ò¶Ì£¬Òò´Ë·¢ËÍATÃüÁîµÄ³¬Ê±Ê±¼äÒ²Ó¦ÏàÓ¦Éè¶¨¡£
ERR_NO gsm_send_at(char* at, char* ptn, int to)
{	
	RxCnt3_rd = RxCnt3_wr;		// ·¢ËÍATÃüÁîÇ°½«¶ÁÓÎ±êÖ±½ÓÒÆµ½Ğ´ÓÎ±êÎ»ÖÃ£¬´Ó¶øºöÂÔÖ®¼äµÄÎ´¶ÁÊı¾İ

	// Í¨¹ıGSM UART¿Ú·¢ËÍATÃüÁî
	usart3_send_int(at, strlen(at));	

	// ÔÚ¹æ¶¨µÄÊ±¼äÄÚ£¬´ÓGSM UART½ÓÊÕ»º³åµÄÖ¸¶¨Î»ÖÃ¿ªÊ¼ËÑË÷Ö¸¶¨µÄPattern
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
// ²éÑ¯µ±Ç°µÄGSMĞÅºÅÇ¿¶È(SIM¿¨Î´²åÈëÊ±Ò²ÄÜÕı³£²éÑ¯RSSI)¡£
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
358511020024166		// ·µ»ØÏûÏ¢ÖĞÎŞ+CGSN:ÏûÏ¢Ç°×º£¬Òò´Ë¸ÄÓÃ+CGSN\r\n×÷Îª¼ì²â·µ»ØÏûÏ¢µÄÌØÕ÷×Ö·û´®
OK
*/
// ²éÑ¯GSMÄ£¿éµÄIMEI¡£
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

// ²éÑ¯GSMÄ£¿éµÄÈí¼ş°æ±¾ºÅ¡£
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

/* Ê¹ÓÃAT+POSIÃüÁî²éÑ¯ÖÜ±ß»ùÕ¾µÄĞÅÏ¢(×î¶à7¸ö£¬SIM¿¨Î´²åÈëÊ±Ò²ÄÜÕı³£¼ì²â)
>> SIM¿¨²åÈëÊ±:

AT+POSI=1

+POSI: 1,460,01,517B,D2A3,38,42,0,460,01,517B,A872,30,38,0,460,01,517A,FFAC,0F,36,0,460,01,517A,6A7E,2F,34,0,460,01,517B,A871,19,31,0,460,01,517B,576D,36,29,0,460,01,517A,AAE8,2E,28,1

OK

1		// mode

// ÍÆ²â»ùÕ¾ÁĞ±íÖĞµÄµÚÒ»¸öÎªµ±Ç°×¢²á»ùÕ¾£¬´Ë»ùÕ¾ĞÅÏ¢¿É×÷Îª»ùÕ¾¶¨Î»µÄÊ×Ñ¡
,
460,	// mcc	
01,		// mnc
517B,	// lac
D2A3,	// ci
38,		// bsic
42,		// rxlev
0		// ended

>> SIM¿¨Î´²åÈëÊ±:

AT+POSI=1

+POSI: 1,460,01,2533,71E0,39,53,0,460,01,2533,741E,17,37,0,460,01,2533,7172,22,34,0,460,01,2533,735E,35,32,0,460,01,2533,7171,2E,30,0,460,01,2533,71F4,1B,27,0,460,01,2533,741B,00,25,1

OK

Èç¹û»¹Ã»ÓĞÕÒµ½ÈÎºÎĞ¡Çø£¬ÔòÖ±½Ó·µ»ØOK£¬Èç¹û»ùÕ¾ĞÅÏ¢Îª¶àÌõ·µ»ØÊı¾İÔÚMCCºÍENDED Ö®¼äÑ­»·¡£
*/

/* Ê¹ÓÃAT+CREGÃüÁî²éÑ¯µ±Ç°×¢²á»ùÕ¾µÄĞÅÏ¢(SIM¿¨Î´²åÈëÊ±²»ÄÜ¼ì²â):
>> SIM¿¨²åÈëÊ±:

AT+CREG=2

OK
AT+CREG?

+CREG: 2, 1, "517B", "A872", 0	// µÚÒ»´ÎÏÔÊ¾µÄ»ùÕ¾ËÆºõ²¢·Çµ±Ç°×¢²áµÄ»ùÕ¾

OK

+CREG: 1, "517B", "D2A3", 0		// µÚ¶ş´ÎÏÔÊ¾µÄÍ·Ò»¸ö»ùÕ¾²ÅÊÇµ±Ç°×¢²áµÄ»ùÕ¾?

+CREG: 1, "517B", "A872", 0

>> SIM¿¨Î´²åÈëÊ±:
AT+CREG=2

OK
AT+CREG?

+CREG: 2, 0

OK

>> ÉèÖÃ×¢²á»ùÕ¾ĞÅÏ¢×Ô¶¯Êä³öºó£¬Ä£¿é¿ÉÄÜÒòÎª»ùÕ¾ÇĞ»»¶ø×Ô¶¯Êä³ö×¢²á»ùÕ¾ÏûÏ¢:

AT+CREG=2			// Ê¹ÄÜ×¢²á»ùÕ¾ĞÅÏ¢×Ô¶¯Êä³ö

OK				
AT+CREG?

+CREG: 2, 1, "2796", "140C", 0

OK

+CREG: 1, "2796", "140D", 0		// ÒÔÏÂ¶¼ÊÇ»ùÕ¾ÇĞ»»Ê±×Ô¶¯Êä³öµÄ×¢²á»ùÕ¾ĞÅÏ¢

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
// ¼ì²âGSMÄ£¿éÖÜ±ß7¸öÁÚ½ü»ùÕ¾µÄĞÅÏ¢¡£
// ×¢: ¸Õ³õÊ¼»¯gsmÄ£¿éºó²»ÒËÂíÉÏ²éÑ¯gsm»ùÕ¾ĞÅÏ¢£¬¶à»á³ö´í¡£
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

	ret = gsm_send_at("AT+POSI=1\r\n", "+POSI: ", 6);  //sim900aÖĞÎŞ´ËÖ¸Áî
	if(ret < 0)
	{		
		return ret;
	}

	// ¼ÇÂ¼»ùÕ¾ĞÅÏ¢×Ö·û´®µÄÆğÊ¼Î»ÖÃ¡£
	RxCnt3_rd_backup = RxCnt3_rd;

	ret = gsm_find_pattern("\r\n", 2);
	if(ret < 0)
	{		
		return ret;
	}

	to = systick+2*1000/SYSTICK_PERIOD;

	// »Ö¸´»ùÕ¾ĞÅÏ¢×Ö·û´®µÄÆğÊ¼Î»ÖÃ¡£
	RxCnt3_rd = RxCnt3_rd_backup;

	// ÌáÈ¡¸÷¸ö»ùÕ¾µÄĞÅÏ¢(Ã¿¸ö»ùÕ¾°üº¬7¸öÓò)
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
						
						// ½ÓÊÕÍêµ±ÆÚ»ùÕ¾µÄµ±ÆÚÓòĞÅÏ¢
						break;
					}
					else if(ch == '\r')
					{
						de_gsm_cell[i][j][k] = '\0';

						// È«²¿»ùÕ¾µÄÈ«²¿ÓòĞÅÏ¢½ÓÊÕÍê³É
						return OK;
					}
					else
					{
						// ½ÓÊÕµ½ÂÒÂë
						return ER_GSM_UART_RECV_CHAOS;
					}
				}

				// ¼ì²é½ÓÊÕÈ«²¿»ùÕ¾ĞÅÏ¢ÊÇ·ñ³¬Ê±¡£
				if(systick > to)
				{
					// ½ÓÊÕ³¬Ê±ÍË³ö
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
// ÔÚGPRSÁ¬½Ó½¨Á¢ºó£¬²éÑ¯Ä£¿é×ÔÉíµÄIPµØÖ·¡£
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
// ½«ÖÆ¶¨µÄÓòÃû½âÎö³ÉIPµØÖ·(Ó¦ÔÚAPNÉèÖÃºóÖ´ĞĞ)¡£
ERR_NO gsm_get_dns(char* ip, int len_ip, char* dn)
{
	char		at[128];
	ERR_NO		ret;

	printf("to inquire dns...\r\n");
	
	sprintf(at, "AT+CDNSGIP=\"%s\"\r\n", dn);  //sim900a  AT+CDNSGIP

	// ÏÈ¼ì²érssiÖµÊÇ·ñÕı³£
	ret = gsm_check_rssi();
	if(ret < 0)
	{
		return ret;
	}

	// OKºÍ+DNS¼¸ºõÍ¬Ê±³öÏÖ
	ret = gsm_send_at(at, "OK", TO_GPRS_DO_DNS);
	if(ret < 0)	// M660+µÄAT+DNSÃüÁî³¬Ê±ÒÑ¾­ÉèÖÃ²¢Ä¬ÈÏÎª5Ãë£¬2012-04-19 23:23
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
>> Í¨»°¹ı³ÌÖĞ·¢ËÍ

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

>> ·ÇÍ¨»°¹ı³ÌÖĞ·¢ËÍ

AT+VTS=1

ERROR


>> Í¨»°¹ı³ÌÖĞ½ÓÊÕDTMF

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
// ·¢ËÍÖ¸¶¨µÄDTMF×Ö·û(Ö§³Ö0,1,2,3,4,5,6,7,8,9,A,B,C,D,#,*£¬ĞëÔÚÍ¨»°¹ı³ÌÖĞ·¢ËÍ)¡£
ERR_NO gsm_send_dtmf(char dtmf)
{
	char		at[16];

	sprintf(at, "AT+VTS=%c\r\n", dtmf);

	return gsm_send_at(at, "OK", 3);
}

/*
>> SIM¿¨²åÈëÊ±:

AT+CCID

+CCID: 89860109520204253890

OK

>> SIM¿¨Î´²åÈëÊ±:

AT+CCID

ERROR

*/
// Í¨¹ı²éÑ¯SIM IDºÅ¼ì²âSIM¿¨ÊÇ·ñ²åÈë¡£
ERR_NO gsm_check_sim(void)
{
	return gsm_send_at("AT+CPIN?\r\n", "CPIN", 2);
}

/*
>> ×¢²áºó

AT+CREG=2

OK
AT+CREG?

+CREG: 2, 1, "25F0", "0F8C", 0

OK

>> Î´×¢²á


*/
// ¼ì²éµ±Ç°µÄÍøÂç×¢²á×´¿ö¡£
ERR_NO gsm_check_reg(void)
{
	char	str[4+1];
	int		state;
	
	ERR_NO	ret;
	
	// ÔÊĞíÍøÂç×¢²áÖ÷¶¯Ìá¹©ËùÔÚµØÑ¶Ï¢£¨CELL ID¡¢LOCAL ID£©
	ret = gsm_send_at("AT+CREG=2\r\n", "OK", 3);
	if(ret < 0)
	{	
		return ret;
    }

	// ²éÑ¯ÍøÂç×¢²á×´¿ö
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
		return OK;	// ÒÑ×¢²á
	}
	else
	{
		return ER_GSM_NETWORK_UNREGISTERED;	// Î´×¢²á
	}
}

// ¼ì²érssiÊÇ·ñÕı³££¬Ò»°ãÔÚ´ò³öµç»°¡¢·¢¶ÌĞÅ¡¢·¢Êı¾İÒÔ¼°²éÑ¯dns(ĞèÒª·¢Êı¾İ)Ç°Ö´ĞĞÒÔ±ãÈ·ÈÏÄ£¿é¹¤×÷×´Ì¬Õı³£¡£
/*
------------------------------------
	signal 		rssi
------------------------------------
0 	<4»ò99 		<-107 dBm or unknown
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
	
	// ¼ì²âÈı´ÎrssiÖµ£¬Èç¹û´óÓÚ5Ôò·µ»Øok
	for(i = 0; i < MAX_TIEMS_GSM_GET_RSSI; i++)
	{
// rssi¼ì²âÎª99Ê±£¬Ã¿¸ô3Ãë¼ì²âÒ»´Î¡¢×î¶à¼ì²âÈı´Î£¬ÒÔ±ã¾¡¿ÉÄÜµÈµ½gsm×¢²áÍøÂç³É¹¦¡£
GET_GSM_RSSI:
		rssi = -1;

		if(gsm_get_rssi((int*)&rssi) == OK)
		{
			printf("#%d RSSI = %d\r\n", i+1, rssi);

			// rssi=99Ò»°ã³öÏÖÔÚÍøÂçÎ´×¢²áÇé¿öÏÂ
			if(rssi == 99)
			{
				cnt--;

				if(cnt >0)
				{
					delay_100ms(30);	// ¼ä¸ô3Ãë
					
					goto GET_GSM_RSSI;
				}
				else
				{
					return -1;			// ÉĞÎ´³É¹¦×¢²áÍøÂç
				}
			}
			else if(rssi <= 4)
			{
				return -2;				// ĞÅºÅÌ«Èõ
			}
			else if(rssi >= MIN_RSSI_FOR_COMMUNICATION)
			{
				return OK;
			}
		}
		// else: ²éÑ¯rssiÊ§°ÜÔò³¢ÊÔÖØ²é
	}

	if(times > 0)
	{
		gsm_recover();

		times--;

		goto CHECK_GSM_RSSI;
	}

	// »Ö¸´gsmºó²éÑ¯rssiÈÔ²»Õı³££¬·µ»ØNG
	printf("failed to check rssi after once trial of recovering gsm.\r\n");
	
	return -2;
}

/**********************************************************************************************************************
										 		GSMÓ¦ÓÃ½Ó¿Úº¯Êı
***********************************************************************************************************************/

// ²¦´òµç»°µ½Ö¸¶¨ºÅÂë¡£
/*
>> ºô½Ğ¶Ô·½(²âÊÔÖĞ·¢ÏÖ£¬ĞÅºÅµÍµ½4Ê±ÈÔ¿É½¨Á¢Í¨»°)

ATD13923887347	// ºô½Ğ¶Ô·½(Í¨»°¿ÉÄÜÔÚ³¤´ï10ÃëÄÚ²Å½¨Á¢)

OK

SPEECH ON

ALERTING

CONNECT

SPEECH OFF		// ¶Ô·½¹Ò¶Ïµç»°

RELEASE

NO CARRIER	

>> ¶Ô·½À´µç

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

	// ¸ñÊ½»¯gsmºÅÂë
	if(format_pn(pn) != OK)
	{
		return NG;
	}

	// ÏÈ¼ì²érssiÖµÊÇ·ñÕı³£
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

	// ÉèÖÃgsmÍ¨»°×´Ì¬±êÖ¾
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

	// ÖØÖÃgsmÍ¨»°×´Ì¬±êÖ¾
	is_gsm_calling = FALSE;
	
	return OK;
}

/********************************* SMS Ïà¹ØµÄ²Ù×÷º¯Êı *************************************/
ERR_NO gsm_sms_mode_txt(void)
{
	return gsm_send_at("AT+CMGF=1\r\n", "OK", 5);
}

ERR_NO gsm_sms_mode_pdu(void)
{
	return gsm_send_at("AT+CMGF=0\r\n", "OK", 5);
}

// ½«Ö¸¶¨µÄÄÚÈİÒÔPDU¸ñÊ½µÄSMS·¢ËÍ¸øÖ¸¶¨Ä¿±ê·½¡£
// ascii_utf8	- 	ÒÔASCII±àÂëµÄÓ¢ÎÄ´ı·¢ËÍ¶ÌĞÅÄÚÈİ»òÒÔUTF-8±àÂëµÄÖĞÎÄ´ı·¢ËÍ¶ÌĞÅÄÚÈİ¡£
ERR_NO sms_snd_pdu(T_SMS_LANGUAGE language, char* pns, char* ascii_utf8)
{
	ERR_NO			ret;	
	
	char			pdu[320];					// pduµÄ×î´ó³¤¶ÈÒ»°ãĞ¡ÓÚ320×Ö½Ú

	unsigned char	encoded[MAX_LEN_CNC_SMS*MAX_NUM_CNC_SMS];	// ¾­ÌØ¶¨±àÂëºóµÄ¶ÌĞÅÄÚÈİ(ÖĞÎÄ²ÉÓÃUCS2±àÂë¡¢Ó¢ÎÄ²ÉÓÃ7bit±àÂë)
	unsigned char*	pencoded = (unsigned char*)encoded;	
	
	char			at[32];

	int				len = 0;	
	
	int				rest = 0;	
	unsigned int 	tobesent = 0; 
	unsigned int	len_seg = 0;		// Ã¿¸öpdu°üÖĞ°üº¬µÄ´¿ÓÃ»§Êı¾İ³¤¶È(ÖĞÎÄºÍÓ¢ÎÄ²»Í¬)

	unsigned int	feed;

	unsigned char	udh[7];				// uder data header
	unsigned int	len_udh = 0;		// udh³¤¶È(ÖĞÎÄÊ¹ÓÃ6×Ö½Úudh¡¢Ó¢ÎÄÊ¹ÓÃ7×Ö½Úudh)
	int				segment_total = 0;	// ³¤¶ÌĞÅ²ğ·Ö·¢ËÍµÄ×ÜÌõÊı
	int				segment_count = 0;	// ³¤¶ÌĞÅ²î·Ö·¨ËÍµÄµ±Ç°ĞòºÅ(´Ó1¿ªÊ¼¼ÆÊı)	

	// ¼ÆËã´ı·¢ËÍ×Ö·û´®µÄÔ­Ê¼³¤¶È(Î´±àÂëÇ°)
	rest = strlen(ascii_utf8);

	printf("content to be sent in pdu:%s\r\n", ascii_utf8);

	// ÏÈ½«´ı·¢ËÍÄÚÈİ¸ù¾İÓïÑÔÀà±ğ±àÂëÎªÌØ¶¨µÄ¸ñÊ½
	if(language == SMS_LANGUAGE_CHINESE)
	{
		rest = pdu_encode16bit(encoded, ascii_utf8, rest);	

		// ¼ì²é´ı·¢ËÍµÄ¶ÌĞÅÄÚÈİÖĞÊÇ·ñ°üº¬·Çutf8±àÂëÒ²·Çascii±àÂëµÄ×Ö·û¡£
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

		// ¼ì²é´ı·¢ËÍ¶ÌĞÅÄÚÈİÖĞÊÇ·ñ°üº¬·ÇASCII×Ö·û¡£
		if(rest <= 0)
		{
			return ER_GSM_SMS_PDU_CHAOS;
		}

		len_udh = 7;
		len_seg = MAX_BYTE_SMS_PDU - len_udh;
	}

	// ¼ÆËã¶ÌĞÅÒª²ğ·ÖµÄ×ÜÌõÊı(Ã¿Ìõ²ğ·ÖºóµÄ¶ÌĞÅ×î´ó³¤¶ÈÎªMAX_BYTE_SMS_PDU¼õÈ¥udhµÄ³¤¶È6×Ö½Ú)
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
	else	// : ·¢ËÍ¶ÌĞÅÄÚÈİ³¤¶ÈĞ¡ÓÚ×î´ópdu³¤¶ÈÊ±£¬²»ÒÔ¼¶Áª¶ÌĞÅ·½Ê½·¢ËÍ
	{
		segment_total = 1;
	}

	// ÓÃÓÚ²úÉúÎ±Ëæ»úÊı
	feed = systick;

	// ²ğ·Ö·¢ËÍ³¤¶ÌĞÅ
	while(rest > 0)
	{			
		// ¼ì²éÊÇ·ñĞèÒª½«¶ÌĞÅ²ğ·Ö·¢ËÍ
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

			// ¹¹Ôìudh(ÖĞÎÄÊ¹ÓÃ6×Ö½Úudh£¬ÒÔ±ãÊ£ÓàµÄ134×Ö½Ú¿ÉÒÔÈİÄÉÕûÊı¸öºº×Ö£¬Ó¢ÎÄÊ¹ÓÃ7×Ö½Úudh£¬ÒÔ±ãudh±¾ÉíºÍudsÒ»Ñù¹¹³É7×Ö½Ú×é)
			if(len_udh == 6)
			{				
				udh[0] = 0x05;
				udh[1] = 0x00;
				udh[2] = 0x03;
				udh[3] = feed&0xFF;			// serial number			
				udh[4] = segment_total;		
				udh[5] = segment_count;				// ·ÖÌõ¶ÌĞÅ´Ó1¿ªÊ¼¼ÆÊı
			}
			else if(len_udh == 7)
			{			
				udh[0] = 0x06;
				udh[1] = 0x08;
				udh[2] = 0x04;
				udh[3] = feed&0xFF;			// serial number			
				udh[4] = (feed>>2)&0xFF;		// serial number	
				udh[5] = segment_total;		
				udh[6] = segment_count;				// ·ÖÌõ¶ÌĞÅ´Ó1¿ªÊ¼¼ÆÊı
			}			

			// ½«±àÂëºóµÄ¶ÌĞÅÄÚÈİ´ò°ü³Épdu×Ö·û´®
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

		// ¼ì²éSMSÄ£Ê½ÉèÖÃÃüÁîÊÇ·ñÖ´ĞĞ³É¹¦£¬Èô²»³É¹¦£¬Ôò¿ÉÄÜÒòÎªµ±Ç°´¦ÓÚSMS PDUÊäÈëÄ£Ê½£¬´ËÊ±Ğè·¢ËÍ0x1BÖÕÖ¹ÊäÈëÄ£Ê½
		ret = gsm_send_at("AT+CMGF=0\r\n", "OK", 2);		
		if(ret < 0)
		{
			usart3_send_int("\x1B", 1);	

			printf("error in \"AT+CMGF=0\".\r\n");
			
			return ret;
		}
		
		// ·¢ËÍPDUÄ£Ê½¶ÌĞÅµÄÃüÁîAT+CMGS=ºóÃæĞèÒª½ô¸ú´ı·¢ËÍPDU°üµÄ³¤¶È(²»°üÀ¨SCA²¿·ÖµÄ£¬¼´²»°üÀ¨gsm_sca_len + gsm_sca_fmt + gsm_sca_str)
		// ´Ë³¤¶ÈÊıÖµ±ØĞëÕıÈ·£¬·ñÔòSMS·¢ËÍ»áÊ§°Ü¡£
		sprintf(at, "AT+CMGS=%d\r", len); 	// ÕâÀïÓÃ\r¶ø·Ç\r\nÈ¥½áÊøATÃüÁî£¬¿ÉÄÜÊÇMTKĞ­ÒéÕ»µÄBug
		
		// ¼ì²éÊÇ·ñ½øÈëSMS PDUÊäÈëÄ£Ê½£¬ÈôÎ´½øÈë£¬Ôò¿ÉÄÜÒòÎªµ±Ç°Õı´¦ÓÚSMS PDUÊäÈëÄ£Ê½£¬´ËÊ±ĞèÒª·¢ËÍ0x1BÖÕÖ¹ÊäÈëÄ£Ê½
		ret = gsm_send_at(at, ">", 2);
		if(ret < 0)	// µÈ´ıSMS PDUÊäÈëÖ¸Ê¾·û'>'
		{
			usart3_send_int("\x1B", 1);	

			printf("error in waiting for '>'.\r\n");
			 
			return ret;
		}
		
		// ·¢ËÍ¶ÌĞÅ
		ret = gsm_send_at(pdu, "+CMGS:", TO_SMS_TX);
		if(ret < 0)	
		{
			usart3_send_int("\x1B", 1);	// cancel SMS inputing mode so to accept following AT command 

			printf("error in sending pdu.\r\n");
			
			return ret;
		}	
	}
	
	// ·¢ËÍÍêSMSºó£¬Ç¿ĞĞ½«SMSÊÕ·¢Ä£Ê½ÉèÖÃÎªTXTÄ£Ê½£¬±ãÓÚ½ÓÊÕSMSÃüÁî	
	ret = gsm_sms_mode_txt();
	if(ret < 0)
	{
		printf("error in setting sms to text mode.\r\n");

		return ret;
	}
	
	return OK;
}

// ½«¸ø¶¨µÄÄÚÈİÒÔTXT¸ñÊ½µÄSMS·¢ËÍ¸øÖ¸¶¨Ä¿±ê·½(Í¨¹ıÊÊµ±µÄÖĞÎÄ±àÂë£¬ÈçÔÚ´úÂë±à¼­Æ÷ÖĞÊäÈëÖĞÎÄ×Ö·û£¬¿É·¢ËÍÖĞÎÄSMS)¡£
ERR_NO sms_snd_txt(char* pns, char* ascii)
{
	ERR_NO	ret;
	char 	at[281];

	// ¼ì²éSMSÄ£Ê½ÉèÖÃÃüÁîÊÇ·ñÖ´ĞĞ³É¹¦£¬Èô²»³É¹¦£¬Ôò¿ÉÄÜÒòÎªµ±Ç°´¦ÓÚSMS PDUÊäÈëÄ£Ê½£¬´ËÊ±Ğè·¢ËÍ0x1BÖÕÖ¹ÊäÈëÄ£Ê½
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

	// ÊäÈëTXT¸ñÊ½µÄSMSÄÚÈİ£¬×¢ÒâÔÚÄÚÈİºóÃæÌí¼Ó¶ş½øÖÆ1A¼´»»ĞĞ·û\r\n£¬·ñÔòSMS·¢ËÍ²»»á³É¹¦
	sprintf((char*)at, "%s\x1A\r\n", ascii);
	
	// ·¢ËÍ¶ÌĞÅ
	ret = gsm_send_at(at, "+CMGS:", TO_SMS_TX);
	if(ret < 0)
	{
		usart3_send_int("\x1B", 1);		// cancel SMS inputing mode so to accept following AT command 

		printf("error in sending pdu.\r\n");

		return ret;
	}

	return OK;
}

// ·¢ËÍÖ¸¶¨µÄÏûÏ¢µ½Ö¸¶¨µÄºÅÂë£¬¼¯³ÉÁË¶ÌÏûÏ¢²ğ·Ö·¢ËÍºÍ·¢ËÍÊ§°Üºó×Ô¶¯ÖØ·¢¹¦ÄÜ¡£
// ×¢: ÊäÈëµÄ´ı·¢ËÍÏûÏ¢ÒªÃ´ÒÔASCII±àÂë£¬ÒªÃ´ÒÔUTF-8±àÂë£¬ÒÔ±ãÍ³Ò»×÷Îª×Ö·û´®´¦Àí¡£
// Ê¹ÓÃgsm_send_smsº¯Êıµ¥´Î¿É·¢ËÍµÄ×î´óºº×ÖÊıÁ¿ÎªMAX_CHAR_CNC_SMS_CN¶¨Òå¡¢µ¥´Î
// ¿É·¢ËÍµÄ×î´óÓ¢ÎÄÊıÁ¿²»ÊÜ´Ëºê¶¨ÒåÏŞÖÆ£¬µ«ÊÇÊÜ³ÌĞò¿É·ÖÅäµÄÕ»¿Õ¼äÏŞÖÆ(µ¥´Î·¢ËÍ269×Ö·ûÃ»ÎÊÌâ)¡£
ERR_NO gsm_send_sms(char* pn, char* ascii_utf8)
{	
	ERR_NO			ret;

	char				pns[MAX_LEN_PN+1];			// ±ê×¼»¯µÄgsmºÅÂë(°üº¬¹ú¼ÒÇøºÅÇ°×º)
	
	int				len	 = 0;	// ´ı·¢ËÍµÄ×Ö·ûÊıÁ¿

	T_SMS_LANGUAGE	language = SMS_LANGUAGE_ENGLISH;

	// Í¨»°×´Ì¬ÏÂ¿ÉÕı³£·¢ËÍ¶ÌĞÅ!!!

	// Èô·¢ËÍºÅÂëÎª¿Õ£¬Ôò½«¶ÌĞÅÖØ¶¨Ïòµ½´®¿ÚÊä³ö
	if(strlen(pn) <= 0)
	{	
		printf("pn is null and redirect to uart:%s", ascii_utf8);

		return NG;
	}

	// ¼ì²é´ı·¢ÄÚÈİÊÇ·ñÎª¿Õ
	len = strlen(ascii_utf8);
	
	if(len <= 0)
	{
		printf("reply is null.\r\n");
		
		return NG;
	}

	// ¸ñÊ½»¯gsmºÅÂë
	if(format_pn(pn) != OK)
	{
		return NG;
	}

	// ½«´¿gsmºÅÂëÌí¼Ó¹ú¼ÒÇøºÅÇ°×º(·¢ËÍpdu¶ÌĞÅÊ±Ä¬ÈÏÉè¶¨ºÅÂë¸ñÊ½ĞèĞ¯´ø¹ú¼ÒÇøºÅÇ°×º)¡£
	strcpy(pns, gsm_telecode);
	strcat(pns, pn);

	// ¼ì²é´ı·¢ËÍÄÚÈİÊÇ·ñÎªÖĞÎÄ
	if(is_all_ascii(ascii_utf8) == FALSE)
	{
		language = SMS_LANGUAGE_CHINESE;
	}

	// printf("sms length = %d and is written in %d.\r\n", len, language);

	// ¼ì²é´ı·¢ËÍµÄ¶ÌÏ¢×Ö·ûÊÇ·ñ³¬¹ıÔÊĞíµÄ×î´óÊıÁ¿(ÖĞÎÄ¶ÌĞÅÄÚÈİ²ÉÓÃUTF8±àÂëÊäÈë¡¢Ó¢ÎÄ¶ÌĞÅÄÚÈİ²ÉÓÃASCII±àÂëÊäÈë£¬Ç°ÕßÃ¿×Ö·ûÕ¼3×Ö½Ú¡¢ºóÕßÃ¿×Ö·ûÕ¼1×Ö½Ú)
	// ·¢ËÍµÄµ¥Ìõ¶ÌĞÅ×î´ó³¤¶ÈÎª4Ìõ¼¶Áª¶ÌĞÅ
	if(len > MAX_LEN_CNC_SMS*MAX_NUM_CNC_SMS)
	{
		printf("max. length of a Chinese or English SMS is %d .\r\n", MAX_LEN_CNC_SMS*MAX_NUM_CNC_SMS);

		return NG;
	}
		
	gsm_wakeup();

	// ¼ì²éRSSIÖµÊÇ·ñÎª-1£¬ÈôÊÇ£¬Ôò¶àÎªÄ£¿éGSM×Ô¶¯¶Ïµç»ò¸´Î»£¬´ËÊ±Ğè»Ö¸´GSMÄ£¿é
#if 1
	// ÏÈ¼ì²érssiÖµÊÇ·ñÕı³£
	ret = gsm_check_rssi();

	if(ret < 0)
	{
		gsm_sleep();

		return NG;
	}
#endif

	// ¹Ì¶¨²ÉÓÃpdu¸ñÊ½·¢ËÍ¶ÌĞÅ(Ó¢ÎÄÒÔ8bit±àÂë¡¢ÖĞÎÄÒÔucs2±àÂë)	
	ret = sms_snd_pdu(language, pns, ascii_utf8);

	gsm_sleep();
	
	// ·¢ËÍ³ö´íÇÒ´íÎó²»ÊÇ¶ÌÏ¢ÄÚÈİÂÒÂëÇé¿öÏÂ£¬»º´æµ±Ç°¶ÌĞÅ×¼±¸ÖØ·¢		
	if(ret == ER_GSM_SMS_PDU_CHAOS)	// ´ı·¢ËÍ¶ÌÏ¢ÄÚÈİÖĞ°üº¬²»Ö§³ÖµÄ×Ö·û±àÂë
	{
		// ¶ÔÓÚ°üº¬²»Ö§³ÖµÄ×Ö·û±àÂëµÄ¶ÌĞÅ£¬²»ÓèÖØ·¢
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
>> SIM¿¨²åÈë×´Ì¬ÏÂ£¬ÊäÈëTCPÁ¬½ÓÃüÁîºó£¬Èç¹ûÁ¬½Ó³É¹¦µÄ»°£¬¶àÔÚ5ÃëÄÚ£¬
   Èç¹ûÁ¬½ÓÊ§°ÜµÄ»°£¬¶àÔÚ18ÃëÄÚ:
   
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

>> SIM¿¨Î´²å×´Ì¬ÏÂ£¬ÊäÈëTCPÁ¬½ÓÃüÁîºó£¬»á¼´¿Ì·µ»ØSIM¿¨Î´²åÈëµÄ´íÎóÏûÏ¢:

AT+NETAPN="CMNET","",""

OK
AT+TCPSETUP=0,74.125.71.105,80

OK
+CME ERROR: NO SIM

+TCPSETUP:0,FAIL
*/
// ¸ù¾İÖ¸¶¨µÄGPRSÁ¬½ÓºÅ½¨Á¢ÏàÓ¦µÄGPRSÁ¬½Ó¡£
// 1£¬M660+³£³öÏÖ½×¶ÎĞÔµÄÕı?òÒì³£ÏÖÏó£¬¼´Ä³´Î¿ª»úºó£¬ÈôÊ×´ÎGPRSÁ¬½ÓÕı³£µÄ»°£¬
//	  ÔòËæºóµÄÁ¬½ÓÕı³£µÄ¸ÅÂÊºÜ´ó£¬·´Ö®ÒàÈ»¡£Òò´ËÄ³´Î¿ª»úºóÈô·¢ÏÖGPRSÁ¬½Ó²»Õı³££¬
//	  ½ÏºÃµÄ°ì·¨ÊÇ¹Ø±ÕÄ£¿é¹©µçÈ»ºóÉÏµç;
// 2£¬GPRSÁ¬½Ó¹ı³ÌÖĞÈôÓĞÀ´µçºôÈë£¬ÔòGPRSÁ¬½ÓÒ»°ã»áÊ§°Ü£¬
//    GPRSÒÑ¾­Á¬½ÓÊ±ÈôÓĞÀ´µçºôÈë£¬ÔòGPRSÁ¬½Ó²»»á¶Ï¿ª;
//    Í¨»°¹ı³ÌÖĞ½¨Á¢GPRSÁ¬½Ó£¬ÔòÒ»°ã»áÊ§°Ü;

//modified by double lin
const u8 *modetbl[2]={"TCP","UDP"};//Á¬½ÓÄ£Ê½

int sim900a_tcpudp_test(u8 mode,u8* ipaddr,u8* port,u8 id)
{ 
	char	at[64];

	if(mode)printf("SIM900A UDPÁ¬½Ó²âÊÔ\n");
	else printf("SIM900A TCPÁ¬½Ó²âÊÔ\n"); 

	//sprintf(at,"AT+CIPSTATUS=?\r\n");
	//if(gprs_soc_status(id) != OK)
	{
		sprintf(at,"AT+CIPSTART=\"%s\",\"%s\",\"%s\"\r\n",modetbl[mode],ipaddr,port);
		if(gsm_send_at(at, "OK", 5) != OK)return NG;		//·¢ÆğÁ¬½Ó
	}
	return OK;
}
int gprs_soc_setup_dns(void)
{
	char	at[64];
	int ret;
	sprintf(at, "AT+CIPSHUT\r\n");	
	ret=gsm_send_at(at, "SHUT OK", 1) ;	 
	if(ret!= OK)		// to = 10 ĞèÒª²âÊÔ
	{
		printf("failed to CIPSHUT.\r\n");
		
		return NG;
	}


	sprintf(at, "AT+CIPMUX=0\r\n");	
	ret=gsm_send_at(at, "OK", 1);
	if(ret!= OK)		// to = 10 ĞèÒª²âÊÔ
	{
		printf("failed to CIPMUX.\r\n");
		
		return NG;
	}


	sprintf(at, "AT+CIPRXGET=1\r\n");	
	ret=gsm_send_at(at, "OK", 1);
	if(ret!= OK)		// to = 10 ĞèÒª²âÊÔ
	{
		printf("failed to CIPRXGET.\r\n");
		
		return NG;
	}


	sprintf(at, "AT+CIPQRCLOSE=1\r\n");	
	ret=gsm_send_at(at, "OK", 1);
	if(ret!= OK)		// to = 10 ĞèÒª²âÊÔ
	{
		printf("failed to CIPQRCLOSE.\r\n");
		
		return NG;
	}


	sprintf(at, "AT+CIPMODE=0\r\n");	
	ret=gsm_send_at(at, "OK", 1);
	if(ret!= OK)		// to = 10 ĞèÒª²âÊÔ
	{
		printf("failed to set CIPMODE.\r\n");
		
		return NG;
	}


	sprintf(at, "AT+CIPSTART=\"TCP\",\"bin172133.oicp.net\",8080\r\n");	
	ret=gsm_send_at(at, "OK", 1);
	if(ret!= OK)		// to = 10 ĞèÒª²âÊÔ
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
//	const u8 *port="8086";	//¶Ë¿Ú¹Ì¶¨Îª8086,µ±ÄãµÄµçÄÔ8086¶Ë¿Ú±»ÆäËû³ÌĞòÕ¼ÓÃµÄÊ±ºò,ÇëĞŞ¸ÄÎªÆäËû¿ÕÏĞ¶Ë¿Ú
	u8 mode=0;				//0,TCPÁ¬½Ó;1,UDPÁ¬½Ó


	printf("to setup gprs connection to %s:%s...\r\n", (char*)tcp_conn[id].ip, (char*)tcp_conn[id].port);
	/// printf("to setup GPRS connection.\r\n");

	// Í¨»°×´Ì¬ÏÂ²»ÄÜ½¨Á¢gprsÁ¬½Ó
	if(is_gsm_calling == TRUE)
	{
		printf("gsm is in calling and can not send data!\r\n");
		
		return NG;
	}

	// ¼ì²é´ı½¨Á¢Á¬½ÓµÄGPRSÅäÖÃ²ÎÊıÖĞIPµØÖ·ÊÇ·ñÎª¿Õ£¬Îª¿ÕµÄ»°ÏÈÖ´ĞĞÓòÃû½âÎö
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
	//¹Ø±ÕÁ¬½Ó
	sprintf(at, "AT+CIPCLOSE=1\r\n");	
	gsm_send_at(at, "CLOSE OK", 1);
	
	//¹Ø±ÕÒÆ¶¯³¡¾°
	sprintf(at, "AT+CIPSHUT\r\n");	
	gsm_send_at(at, "SHUT OK", 1) ;
	
	//ÉèÖÃÒÆ¶¯Ì¨Àà±ğ
	sprintf(at, "AT+CGCLASS=\"B\"\r\n");	
	if(gsm_send_at(at, "OK", 10) != OK)		// to = 10 ĞèÒª²âÊÔ
	{
		printf("failed to set CGCLASS.\r\n");
		
		return NG;
	}
	//ÉèÖÃPDPÉÏÏÂÎÄ
	sprintf(at, "AT+CGDCONT=1,\"IP\",\"CMNET\"\r\n");
	if(gsm_send_at(at, "OK", 10) != OK)		// to = 10 ĞèÒª²âÊÔ
	{
		printf("failed to set CGDCONT.\r\n");
		
		return NG;
	}
	//ÉèÖÃ¸½×ÅGPRSÒµÎñ
	sprintf(at, "AT+CGATT=1\r\n");
	if(gsm_send_at(at, "OK", 5) != OK)		// to = 5 ĞèÒª²âÊÔ
	{
		printf("failed to set CGATT.\r\n");
		
		return NG;
	}
	//ÉèÖÃÎªGPRSÁ¬½ÓÄ£Ê½
	sprintf(at, "AT+CIPCSGP=1,\"CMNET\"\r\n");
	if(gsm_send_at(at, "OK", 5) != OK)		// to = 5 ĞèÒª²âÊÔ
	{
		printf("failed to set CIPCSGP.\r\n");
		
		return NG;
	}
	//ÉèÖÃ½ÓÊÕÊı¾İÏÔÊ¾IPÍ·(·½±ãÅĞ¶ÏÊı¾İÀ´Ô´)
	sprintf(at, "AT+CIPHEAD=1\r\n");
	if(gsm_send_at(at, "OK", 5) != OK)		// to = 5 ĞèÒª²âÊÔ
	{
		printf("failed to set CIPHEAD.\r\n");
		
		return NG;
	}

//	return sim900a_tcpudp_test(mode,(u8 *)tcp_conn[id].ip,(u8 *)tcp_conn[id].port,id);
	if(sim900a_tcpudp_test(mode,(u8 *)tcp_conn[id].ip,(u8 *)tcp_conn[id].port,id) !=OK) return NG;
	// ÉèÖÃAPN	"AT+NETAPN=%s,\"\",\"\"\r\n"
//	sprintf(at, "AT+NETAPN=\"%s\",\"\",\"\"\r\n", gsm_apn);	//SIM900A AT+CSTT
//	
//	if(gsm_send_at(at, "OK", TO_GPRS_SET_APN) != OK)		// to = 2
//	{
//		printf("failed to set APN.\r\n");
//		
//		return NG;
//	}

	// printf("APN set.\r\n");
		
//	sprintf(ptn, "+TCPSETUP:%d,OK", id);	// ¼ÓÇ¿ÅĞ¶Ï£¬ÁîËüÄÜÇø±ğOKÓëFALL
	
	// ½¨Á¢GPRSÁ¬½Ó
//	sprintf(at, "AT+TCPSETUP=%d,%s,%s\r\n", id, tcp_conn[id].ip, tcp_conn[id].port); //SIM900A  AT+CIPSTART

	
	while(errors < (MAX_TIMES_GPRS_SETUP_CONNECTION+2) )//µÈ´ı4 X 5 = 20s  modified by double lin
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
// ¸ù¾İÖ¸¶¨µÄGPRSÁ¬½ÓºÅ¹Ø±ÕÏàÓ¦µÄGPRSÁ¬½Ó¡£

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
// ¼ì²éÖ¸¶¨GPRSÁ¬½ÓµÄÁ¬½Ó×´Ì¬£¬Èô·µ»ØÖµÎªOK£¬ÔòÎªÁ¬½Ó×´Ì¬£¬·ñÔòÎªÎ´Á¬½Ó×´Ì¬¡£
// ×¢: GPRSÁ¬½Ó×Ô¶¯±£³Ö³¬Ê±Ê±¼äÎª30·ÖÖÓ£¬¼´GPRSÁ¬½Ó¿ÕÏĞ³¬¹ı30·ÖÖÓ£¬GSM»ùÕ¾»á½«Æä¶Ï¿ª¡£

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

    // ÏÈ¼ì²â·´À¡ÏûÏ¢µÄÏûÏ¢Í·
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

>						// Ä£¿é·¢ËÍ1234£¬·şÎñÆ÷ÊÕµ½1234
OK
+TCPSEND:0,4

+TCPRECV: 0,4,4321		// ·şÎñÆ÷·¢ËÍ4321£¬Ä£¿éÊÕµ½4321

+TCPRECV: 0,8,abcd1234	// ·şÎñÆ÷·¢ËÍabcd1234£¬Ä£¿éÊÕµ½abcd1234
*/
// ÔÚ·ÇÍ¸´«Ä£Ê½ÏÂ´ÓÖ¸¶¨µÄGPRSÁ¬½ÓÉÏ·¢ËÍ¸ø¶¨³¤¶ÈµÄÊı¾İ£¬
// ²¢·µ»ØÊµ¼Ê³É¹¦·¢ËÍµÄÊı¾İ³¤¶È(¼´´Ó×Ô¶¯·Ö¸î·ÖÅú·¢ËÍ»úÖÆ)¡£

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
	int			  	n_tobesent 	= 0;		// Ã¿´Î´ı·¢ËÍµÄ×Ö½ÚÊı
	int 			n_reported		= 0;		// Ã¿´Î·¢ËÍºóGSMÄ£¿é·´À¡µÄ·¢ËÍ³É¹¦×Ö½ÚÊı
	int				n_sentout 		= 0;		// ÀÛ¼Æ·¢ËÍ³É¹¦µÄ×Ö½ÚÊı

	int				repeat  		= MAX_TIMES_GPRS_SEND_PACKET;    // repat times to sent the identical packet

	char			at[32];						// to accomodate the command header																		

//	char			ptn[16];	

	unsigned int 	to;

	// °´ÕÕ´ı·¢ËÍ×Ö½ÚÊı¼°·¢ËÍÃ¿Ö¡Êı¾İµÄ³¬Ê±¼ÆËã×Ü·¢ËÍ³¬Ê±Ê±¼ä¡£
	to = ((size/MAX_LEN_GPRS_PACKET_SEND)+((size%MAX_LEN_GPRS_PACKET_SEND)?1:0))*(TO_GPRS_TX_FRAME+1);

	printf("to send %d bytes within %d seconds.\r\n", size, to);

	// Í¨»°×´Ì¬ÏÂ²»ÄÜ·¢ËÍgprsÊı¾İ
	if(is_gsm_calling == TRUE)
	{
		printf("gsm is in calling and can not send data!\r\n");
		
		return NG;
	}

	// ·¢ËÍGPRSÊı¾İ°üÖ®Ç°£¬ÂÔ¹ı´ËÇ°½ÓÊÕµ½µ«Î´¶ÁµÄËùÓĞÊı¾İ
	RxCnt3_rd = RxCnt3_wr;

	to = systick+to*1000/SYSTICK_PERIOD;

	// ÖğÖ¡·¢ËÍGPRSÊı¾İ
	while(size > 0 && systick < to)
	{     
	    n_reported = 0;    

        // Ã¿´Î·¢ËÍµÄÊı¾İ³¤¶È²»ÄÜ³¬¹ı¹ıAT+SOCSENDÃüÁîÔÊĞíµÄ×î´óÊı¾İ³¤¶È(=496×Ö½Ú)
		n_tobesent = size > MAX_LEN_GPRS_PACKET_SEND ? MAX_LEN_GPRS_PACKET_SEND : size;

		// ¹¹ÔìÏûÏ¢Í·	:¼Ó\r\n
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
			//sprintf(ptn, "SEND FAIL");	//¼ì²é·µ»ØÊı¾İÖĞÊÇ·ñÓĞSEND FAIL  Èç¹ûÓĞÔòËµÃ÷·¢ËÍÊ§°Ü			 
			//if(gsm_find_pattern(ptn, TO_GPRS_TX_FRAME) == OK)
			//{
			//	return n_sentout;
			//}

			return n_sentout;
		}
		

		// µÈ´ıÖÁÉÙ½ÓÊÕµ½Ò»¸ö×Ö½ÚµÄºóĞøÊı¾İ
		/*
       	if(!gsm_wait_output(1, 2))
        {
        	return n_sentout;
        }

		// ¼ì²â·µ»ØµÄÒÑ³É¹¦·¢ËÍµÄÊı¾İ×Ö½ÚÊı
		if(gsm_fetch_value((int*)&n_reported, 2) != OK)
		{
			return n_sentout;
		}
		*/
        // ±È½ÏÆÚÍû·¢ËÍµÄÊı¾İ³¤¶ÈºÍÊµ¼Ê·¢ËÍµÄÊı¾İ³¤¶ÈÊÇ·ñÒ»ÖÂ
        // ×¢: ½öÔÚ·´À¡µÄ·¢ËÍ×Ö½ÚÊı²»µÈÓÚÆÚÍû·¢ËÍµÄ×Ö½ÚÊıÊ±²ÅÖØ·¢£¬ÒòÎªÕâÖÖÇé¿ö¶à
        // ³öÏÖÔÚPRSÁ¬½ÓÔÚ´«Êä¹ı³ÌÖĞÍ»È»±äµÃ²»ÎÈ¶¨Ê±(ÈçGSMĞÅºÅÍ»È»ÖĞ¶Ï»ò¶Ô·½·şÎñÆ÷ÔİÊ±å´»úµÈ)£¬
        // ÔÚ´ËÖ®Ç°Èô³öÏÖÒì³££¬Ôò¶àÎªGSMÓ²¼ş/Èí¼ş³öÏÖÖØ´ó¹ÊÕÏ£¬Òò´ËÃ»±ØÒª¼´¿ÌÖØ·¢(»á½«Î´·¢ËÍ³É¹¦
        // µÄÊı¾İ¼ÓÈëµ½È«¾ÖµÄÖØ·¢Êı¾İ¶ÓÁĞÔÚÔñ»úÖØ·¢)¡£
        n_reported = n_tobesent;
		if(n_reported == n_tobesent)
		{
			size        -= n_reported;		// Ê£ÓàÊı¾İµÄ³¤¶Èµİ¼õ
	        ptr         += n_reported;		// ¶ÁÖ¸ÕëºóÒÆ
	        n_sentout   += n_reported;		// ÀÛ¼Æ·¢ËÍ×Ö½ÚÊıµİÔö
            
            repeat       = MAX_TIMES_GPRS_SEND_PACKET; 	// ÎªÏÂÒ»¸öÊı¾İÖ¡µÄ·¢ËÍÖØÖÃÖØ·¢´ÎÊı¼ÆÊıÆ÷ 
		}
		// ¸Õ¸Õ·¢ËÍµÄÊı¾İ³¤¶È²»µÈÓÚ´ı·¢ËÍÊı¾İ³¤¶ÈÊ±£¬³¢ÊÔÖØ·¢µ±Ç°Êı¾İÖ¡£¬ÖØ·¢´ÎÊıÓÉÏµÍ³Éè¶¨
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

// ½«Çı¶¯²ãµÄgprs_soc_tx()º¯Êı·â×°³É´øÖØ·¢´¦ÀíµÄÊÊÅä²ãº¯Êı£¬²¢¼¯³É×Ô¶¯ÖØ·¢
// »úÖÆ(ÊÇ·ñÖØ·¢È¡¾öÓÚÍâ²¿µ÷ÓÃº¯ÊıµÄÉèÖÃ)ºÍÁ´Â·×Ô¶¯¹ÜÀí»úÖÆ(Á´Â·ÓĞĞ§Ôò²»ÖØ½¨£¬
// ·ñÔòÖØ½¨Á´Â·)¡£

//modified by double lin
int gsm_send_data(int id, unsigned char* data, unsigned int size)
{
	volatile unsigned int systick1 = 0;
	volatile unsigned int systick2 = 0;

	int			ret;
	int			errors = 0;

#if 1
	// ÏÈ¼ì²érssiÖµÊÇ·ñÕı³£
	ret = gsm_check_rssi();
	if(ret < 0)
	{
		return -1;
	}
#endif

#if 1
	// ¼ì²éÖ¸¶¨µÄGPRSÁ¬½ÓÊÇ·ñÓĞĞ§£¬ÈôÎŞĞ§£¬Ôò½¨Á¢Ö®
	ret = gprs_soc_status(id);
#endif

	// printf("step out gprs_soc_status().\r\n");

	// ÈôÖ¸¶¨GPRSÁ¬½Óµ±Ç°´¦ÓÚ¶Ï¿ª×´Ì¬£¬Ôò³¢ÊÔÁ¬½Ó(Á¬½Óº¯ÊıÄÚ²¿»á×î¶àÁ¬ĞøÖØÊÔÈı´Î£¬Èı´Î¶¼Ê§°Ü²Å·µ»Ø³ö´í½á¹û)
	if(ret != OK)
	{
		printf("Ö¸¶¨GPRSÁ¬½Óµ±Ç°´¦ÓÚ¶Ï¿ª×´Ì¬£¬³¢ÊÔÁ¬½Ó\n");	
		systick1 = systick;
#if 0
		ret = gprs_soc_setup(id);
#endif
#if 1
		ret = gprs_soc_setup_dns();
#endif
		systick2 = systick;		

		// Èô½¨Á¢Á¬½ÓÊ§°Ü£¬Ôò½«´ı·¢ËÍÊı¾İĞ´ÈëFlashÖĞÒÔ´ıÖØ·¢
		if(ret != OK)
		{
			printf("failed to set up GPRS connection within %d ms.\r\n", (systick2-systick1)*SYSTICK_PERIOD);
			
			return -2;		// Á¬½ÓÊ§°Ü
		}	
		else
		{
			printf("it took %5d ms to set up GPRS connection.\r\n", (systick2-systick1)*SYSTICK_PERIOD);
		}
	}

	// ·¢ËÍÍêÕûÊı¾İ£¬·¢ËÍ³ö´í(ÍêÈ«·¢ËÍ³É¹¦²ÅËã³É¹¦)×î¶àÁ¬ĞøÖØÊÔÈı´Î¡£
	while(errors < MAX_TIMES_GPRS_SEND_DATA)
	{			
		systick1 = systick;
		ret = (int)gprs_soc_tx(id, data, size);
		systick2 = systick;
		
		if(ret != size)
		{		
			errors++;	

			// Á½´Î·¢ËÍÖ®¼ä¼ä¸ôÒ»¶ÎÊ±¼ä
			delay_100ms(5);
		}
		else
		{
			printf("it took %5d ms to send out %5d bytes data.\r\n", (systick2-systick1)*SYSTICK_PERIOD, size);

			// ·µ»Ø·¢ËÍµÄÊı¾İ³¤¶È(Êµ¼ÊÎª¿½±´)
			return size;
		}
	}

	return -3;			// ·¢ËÍÊ§°Ü
}

// ÔÚÖ¸¶¨µÄGPRSÁ¬½ÓÉÏ½ÓÊÕÊı¾İ¡£
// Ò»°ãÔÚ·¢ËÍÊı¾İºó²Åµ÷ÓÃ½ÓÊÕÊı¾İµÄº¯Êı¡£
// +TCPRECV: 0,8,abcd1234	// ·şÎñÆ÷·¢ËÍabcd1234£¬Ä£¿éÊÕµ½abcd1234

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
	// ÏÈµÈ´ı½ÓÊÕ10×Ö½ÚÊı¾İ(TCP½ÓÊÕÊı¾İ°üµÄ°üÍ·"+TCPRECV: "¾ÍÕ¼È¥ÁË10×Ö½Ú³¤¶È)
	if(gsm_wait_output(10, to) != OK)
	{
		printf("failed to receive 10 bytes within %d seconds.\r\n", to);

		return -1;
	}
#endif

	// µÈ´ı½ÓÊÕ·µ»ØÊı¾İ
	if(gsm_wait_output(7, to) != OK)
	{
		printf("faield to wait for 7 bytes within 10s.\n");

		return -1;
	}

	// µÈ´ı½ÓÊÕ²¢¶¨Î»µ½TCP½ÓÊÕÊı¾İ°üµÄ°üÍ·¡£
	if(gsm_find_pattern(pattern, 5) != OK)
	{
		printf("failed to find %s.\r\n", pattern);
	
		return -2;
	}

	// ÌáÈ¡TCPÊı¾İ°üµÄÊı¾İ³¤¶ÈĞÅÏ¢¡
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

	// ÌáÈ¡TCPÊı¾İ°üµÄÊı¾İÄÚÈİ¡£
	RxCnt3_rd++;//Ìø¹ı":"
	i = 0;
	
	while(i < size)
	{
		if(RxCnt3_rd < RxCnt3_wr)
		{
			data[i++] = GSM_RX_RD(RxCnt3_rd++);
		}
	}

	printf("\r\n");

	// ·µ»Ø½ÓÊÕµ½µÄÊı¾İµÄ³¤¶È
	return size;		
}

#endif

