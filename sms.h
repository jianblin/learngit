#ifndef __SMS_H
#define __SMS_H

/*
TP-DCS
------------------------------
Bit 3 	Bit2 	Alphabet:
------------------------------
0 		0 		Default alphabet
0 		1 		8 bit data
1 		0 		UCS2 (16bit)
1 		1 		Reserved
------------------------------
*/
#define SMS_PDU_ENCODING_7BIT					(0<<2)	// Default alphabet
#define SMS_PDU_ENCODING_8BIT					(1<<2)	// 8 bit data
#define SMS_PDU_ENCODING_UCS2					(2<<2)	// UCS2 (16bit)
#define SMS_PDU_ALPHABET_RESERVED				(3<<2)	// Reserved

#define SMS_MODE_TXT			1
#define SMS_MODE_PDU			0

typedef struct
{	
	// SCA
	unsigned char	sca_len;			// number of octets represent sca_fmt AND sca_str
	unsigned char	sca_fmt;		// normally fixed to 0x91 if SCA specified explicitly
	char			sca_str[20];			// encoded destination address(invert withi a pair and attach 'F' if neccessary)
									// Note: SCA phone number MUST be with 86 prefix if internal numbering format set
	// MTI and friends			   	 	// set to 0x11 normally for SMS-Submit
	unsigned char	tp_mti;			// Message Type indicator
	unsigned char	tp_rd;			// Reject Duplicates
	unsigned char	tp_vpf;			// Validity Period Format
	unsigned char	tp_srr;			// Status Report Request
	unsigned char	tp_udhi;			// User Data header Indicator: 0 - without UDH, 1 - with UDH
	unsigned char	tp_rp;			// Reply Path

	// MR
	unsigned char	tp_mr;			// Message reference field

	// DA
	unsigned char	tp_dal;			// number of decimal digits represent RAW destination address(exclude tp_dat and attached 'F')
	unsigned char	tp_dat;			// normally fixed to 0x91, which implies international numbering format used
	unsigned char	tp_das[20];		// encoded destination address(invert withi a pair and attach 'F' if neccessary)
									// Note: destination phone number MUST be with 86 prefix if internal numbering format set
	// PDI
	unsigned char	tp_pid;			// Protocol identifier, set to 0x00 normally

	// DCS
	unsigned char	tp_dcs;			// Data coding scheme: , set to 0x00 if 7-bit encoding used, to 0x08 if UCS2 encoding used 

	// VP
	unsigned char	tp_vp;			// Validity period, set to 0x00 normally which implies 5min

	// User Data
	unsigned char	tp_udl;			// bytes of RAW User Data(not encoded User Data)
	unsigned char	tp_uds[160];		// User Data string, can be up to 140 octets
									// Note: User Data is sent to MMA3328 module via UART in ASCII format, which implies a unsigned char
									// occupies 2 char space after conversion 
}T_SMS_SUBMIT;	// SMS format definition while sending	

typedef struct
{
	unsigned char	udh_cr;			// SM concatenation 8bit ref.
	unsigned char	udh_eb;			// Bytes Information Element
	unsigned char	udh_sr;			// SM reference number
	unsigned char	udh_nm;			// number of messages
	unsigned char	udh_sn;			// this SM sequence number
}T_SMS_DELIVER_UDH;					// User Data Header, used in concatenation SMS		

typedef struct
{
	unsigned char	sca_len;			// SCA号码长度(包括号码格式字节)
	unsigned char	sca_fmt;		// SCA号码格式
	char			sca_str[20];			// SCA号码

	unsigned char	tp_mti;			// Message Type indicator
	unsigned char	tp_mms;			// More Messages to Send
	unsigned char	tp_sri;			// Status Report Indication
	unsigned char	tp_udhi;			// User Data header Indicator: 0 - without UDH, 1 - with UDH
	unsigned char	tp_rp;			// Reply Path
	
	unsigned char	tp_oal;			// Originating Address length
	unsigned char	tp_oat;			// Originating Address type
//	unsigned char tp_contry;  	 		// 国家代码
	char			tp_oas[20];			// Originating Address string

	unsigned char	tp_pid;			// Protocol identifier

	unsigned char	tp_dcs;			// Data coding scheme

	char			tp_scts[16];	// The service center time stamp
	
	unsigned char	tp_udl;			// User data length
	unsigned char	tp_udhl;		// User Data Header length
	unsigned char	tp_udh[32];		// user Data Header
	unsigned char	tp_uds[281];	// User Data, can be up to 140 octets, but in ASCII encoding in MMA3328 output.

}T_SMS_DELIVER;	// SMS format definition while receiving
	
int pdu_construct(char* pdu, char* pn, char* sca, unsigned char dcs, unsigned char* encoded, unsigned int size, unsigned char udhi, unsigned char* udh, unsigned int len_udh);
int pdu_deconstruct(T_SMS_DELIVER * deliver, char* asc);

unsigned int pdu_atoh(unsigned char* hex, char* asc, unsigned int len_asc);

int pdu_encode7bit(unsigned char* octect, char* ascii, unsigned int len_ascii);
unsigned int pdu_encode8bit(char* dst, char* src, unsigned int len_src);
int pdu_encode16bit(unsigned char* encoded, char* utf8, unsigned int len_utf8);

int utf8_to_ucs2( unsigned char* ucs2, char* utf8, int len);

#endif

