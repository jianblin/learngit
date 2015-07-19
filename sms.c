#include "stdio.h"
#include "string.h"
#include "sms.h"
#include "dev_gsm.h"
#include "common.h"

/********************************************************************************************************************
*
*
*
*
*********************************************************************************************************************/
// Invert phone number in pair unit, 'F' attached to make length odd.
// e.g.: 86 13 80 07 57 51 1  --> 68 31 08 70 75 15 F1, F is attached.
unsigned int pdu_invert(char* dst, char* src, unsigned int len_src)
{
	unsigned int	i 			= 0;
	unsigned int 	len_dst		= 0;
	char 			c			= 0;	

	len_dst = len_src;

	for(i = 0 ; i<len_src; i += 2)
	{
		c = *src++;		
		*dst++ = *src++;	
		*dst++ = c;		
	}

	if(len_src & 1)
	{
		*(dst-2) = 'F';	
		len_dst++;		
	}

	*dst = '\0';

	return len_dst;
}

// Recover inverted phone number.
// e.g.: 68 31 08 70 75 15 F1 --> 86 13 80 07 57 51 1F, F should be deleted.
unsigned int  pdu_recover(char* dst, char* src, unsigned int len_src)
{	
	unsigned int	i  =0;
	unsigned int 	len_dst;	
	char 			c;		

	len_dst = len_src;

	for(i=0; i<len_src;i+=2)
	{
		c = *src++;	
		*dst++ = *src++;	
		*dst++ = c;		
	}

	if(*(dst-1) == 'F')
	{
		dst--;
		len_dst--;		
	}

	*dst = '\0';

	return len_dst;
}

// Convert string in ASCI format into values in 8-bit.
// Note:
// 1, two continuous '0' in src will produce a 0x00 value in dst, 
//    which will be conflict with the ender of string '\0', so dst 
//    can not be regarded as a normal string herein;
unsigned int pdu_atoh(unsigned char* hex, char* asc, unsigned int len_asc)
{
	unsigned int 		i;
	unsigned int		len_hex	= 0;
	unsigned char 		quad 	= 0;
	unsigned char 		octet 	= 0;
	
	for(i = 0; i < len_asc; i++)
	{	
		if(*asc >= 'a' && *asc <= 'f')
		{
			quad = *asc -'a'+10;
		}
		else if(*asc >= 'A' && *asc <= 'F')
		{
			quad = *asc -'A'+10;
		}
		else if(*asc >= '0' && *asc <= '9')
		{
			quad = *asc -'0';
		}

		asc++;

		octet |= quad;

		if(i&1)
		{			
			*hex++ = octet;
			len_hex++;

			octet = 0;
		}
		else
		{
			octet <<= 4;
		}
		
	}

	return len_hex;	
}

// Convert hexical value into ASCII stringin big endian.
unsigned int pdu_htoa(char*asc, unsigned char* hex, unsigned int len_hex)
{	
	unsigned int 	i ;	  
	char			tbl_htoa[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};	 

	// recoveried by WJC, 17:57, 2012-04-07 
	for(i = 0; i < len_hex; i++)
	{
		*asc++ = tbl_htoa[(*hex>>4)&0x0F];	
		*asc++ = tbl_htoa[*hex & 0x0F];

		hex++;
	}
	
	*asc = '\0';

	return (len_hex*2);	
	
/*
	 for(i = 0; i < len_hex*2; i++)
	 {
		*asc++ = *hex++;		
		len_asc ++;
	 }
	 *asc = '\0';
	 return len_hex;
*/
}


// Encod 8-bit ASCII character into 7-bit code and convert value of 7-bit code into string(double characters).
int pdu_encode7bit(unsigned char* octect, char* ascii, unsigned int len_ascii)
{
	unsigned int		i;
	unsigned int		len_octect = 0;
	unsigned int		mod;

	for(i = 0; i < len_ascii; i++)
	{
		// ����Ƿ��з�ASCII�ַ�
		if(ascii[i] > 0x7F)
		{
			return NG;
		}
		
		mod = i & 7;

		if(mod !=  7)
		{
			octect[len_octect++] = (ascii[i]>>mod) | ascii[i+1]<<(7-mod);
		}
	}

	return len_octect;
}

// Decode 7bit- encoded data into 8bit encoded data and conveert value into ASCII string.
unsigned int pdu_decode7bit(char* asc, unsigned char* hex, unsigned int len_hex)
{
	int 	len_asc = 0;
	int 	i;
	int 	mod = 0;

	for(i = 0; i < len_hex; i++)
	{
		mod = i%7;

		if(mod == 0)
		{		
			*asc = (unsigned char)(hex[i]<<(mod+1));
			*asc = (unsigned char)(*asc>>1);		}
		else
		{			
			*asc = (unsigned char)(hex[i]<<(mod+1));
			*asc = (unsigned char)(*asc>>1);
			*asc |= (hex[i-1]>>(8-mod));	
			
			// �����ǰ�ֽ����Ϊ7�ı���(��7ȡ��ֵΪ6)����Ҫ��ȡ���ֽڵ�ǰ7λ
			if(mod == 6)
			{
				asc++;
				len_asc++;

				*asc = (unsigned char)(hex[i]>>1);
			}
		}
		
		asc++;
		len_asc++;		
	}

	*asc = '\0';

	return len_asc;
}

#if 0
void test_codec_7bit(void)
{
	char	asc[256];
	unsigned char	bin[256] = "\x54\x74\x19\x34\x44\xA7\xDD\xE5\x79\x19\x44\x2E\xB3\xCB\xE7\x30\x3D\xFD\x76\x83\xE8\x6F\x10\x1D\x5D\x06\x25\xDD\xF4\xB2\xDC\x1D\xA6\xA7\xDF\xEE\x30\x1B";
	// unsigned char	bin[256] = "\x10\x1D\x5D\x06\xC9\xCB\xF3\x7A\x1B\x4E\x4F\xBF\xDD\xA0\xB7\x19\x34\x4D\xE3\x5B\xD0\xB0\x9C\x9E\x07\xD1\xC3\xEC\xF5\x1C\x14\x9E\x83\xCA\x61\x39\x3B\x0F\x0A\xCF\x41\xF0\xF7\x7C\x9E\x16\xB3\xCB\x2E\x10\x15\x5D\x06";
	
	int		len;


	// len = pdu_encode7bit(bin, asc1, strlen(asc1));
	
	len = pdu_decode7bit(asc, bin, 38);

	// printf("len = %d, asc = %s\r\n", len, asc);

	while(1);

}
#endif

unsigned int pdu_encode8bit(char* dst, char* src, unsigned int len_src)
{
	int 		i;

	// ����ʹ��strcpy����ִ���ַ���������!!!
	for(i = 0; i < len_src; i++)
	{
		dst[i] = src[i];
	}

	return len_src;
}

unsigned int pdu_decode8bit(char *dst, char* src, unsigned int len_src)
{
	int 			i;

	// ����ʹ��strcpy����ִ���ַ���������!!!
	for(i = 0; i < len_src; i++)
	{
		dst[i] = src[i];
	}

	return len_src;
}

// ��ָ����������Ϣ����Unicode Big Endian���룬����ϵͳ��������Ϣ
// Ĭ�ϲ���Unicode Big Endian���룬��˶�������Ϣ����������ת����
// ��ֻ��ԭ���������ɡ�
int pdu_encode16bit(unsigned char* encoded, char* utf8, unsigned int len_utf8)
{
	int len_encoded = 0;

	len_encoded = utf8_to_ucs2(encoded, utf8, len_utf8);

	return len_encoded;	
}

// ��GSMģ����յ���ͨ���ܿ��������Ϣת��ΪUnicode Big Endian���룬����
// GSMģ����յ���ͨ���ܿ��������Ϣ�������Unicode Big Endian����ģ����
// ����ת�������ֻ��ԭ���������ɡ�(������M660+ģ��߲��Է��֣�textģʽ��
// ģ����ڽ��յ���pduģʽ�Ķ���Ϣֱ���������Ϣ���ݵ�Unicode Big Endian����ֵ)
unsigned int pdu_decode16bit(unsigned char* decoded, unsigned char* ucs2, unsigned int len_ucs2)
{
	unsigned int i;	
	unsigned int len_decoded = 0;
	
	// swap characters within a pair
	for(i = 0; i < len_ucs2;)
	{
		*decoded++ = ucs2[i++];
		*decoded++ = ucs2[i++];

		len_decoded += 2;
	}

	return len_decoded;	
}

// Encode input arguments into a complete PDU stream in ASCCI format and 
// return the PDU length excluding SCA part(sca_len + sca_fmt + sca_str).
int pdu_construct(char* pdu, char* pn, char* sca, unsigned char dcs, unsigned char* encoded, unsigned int size, unsigned char udhi, unsigned char* udh, unsigned int len_udh)
{
	char			buf_sca[32];
	char			buf_da[32];
	unsigned int	len = 0;
	unsigned int 	len_encoded = 0;
	unsigned int 	len_fnl = 0;
	unsigned char	arg = 0;
	T_SMS_SUBMIT	smssubmit = {0x00, 0x00, "", 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x91,"", 0x00, 0x00, 0x00, 0x00, "" };

	// // printf("size = %d, encoded = %s\r\n", size, (char*)encoded);
		
	// handle SCA( SCA length counts both sca_str and sca_type, but in unsigned char format)
	while(*sca == '+')
	{
		sca++;							// ignore '+' in the front of phone number
	}
	
	len = strlen(sca);

	// // printf("sca = %s\r\n", sca);

	if(len == 0)
	{
		smssubmit.sca_len = len;

		len = sprintf(buf_sca, "%02x", smssubmit.sca_len);
	}
	else
	{
		len = pdu_invert((char*)smssubmit.sca_str, sca, len);

		smssubmit.sca_len = len/2 + 1;	// SCA length counts both sca_str in unsigned char and sca_fmt(1 byte) 

		smssubmit.sca_fmt = 0x91;		// international numbering with "86" prefix
		
		len = sprintf(buf_sca, "%02x%02x%s", smssubmit.sca_len, smssubmit.sca_fmt, smssubmit.sca_str);		
	}

	// // printf("buf_sca = %s\r\n", buf_sca);
	
	// ����udhiλ
	smssubmit.tp_udhi = udhi;

	// handle TP-MTI/VFP etc. as a complete argument
	arg =   (smssubmit.tp_mti&3) | ((smssubmit.tp_rd&1)<<2) | ((smssubmit.tp_vpf &3)<<3) | \
		   ((smssubmit.tp_srr &1)<<5) |((smssubmit.tp_udhi&1)<<6) | ((smssubmit.tp_rp &1)<<7);

	// // printf("arg = %02x\r\n", arg);

	// handle DA
	while(*pn == '+')
	{
		pn++;								// ignore '+' in the front of phone number
	}

	smssubmit.tp_dal = strlen(pn);			// decimal digit number of destination adress, excluding tp_dat and attached 'F'
	pdu_invert((char*)smssubmit.tp_das, pn, smssubmit.tp_dal);	

	smssubmit.tp_dat = 0x91;				// internal numbering

	len = sprintf(buf_da, "%02x%02x%s", smssubmit.tp_dal, smssubmit.tp_dat,smssubmit.tp_das);

	// // printf("buf_da = %s\r\n", buf_da);

	// ���ô��������ݲ��õ��ַ������ʽ
	smssubmit.tp_dcs = dcs;

	// ����udl
	if(dcs == SMS_PDU_ENCODING_UCS2)
	{
		smssubmit.tp_udl = size;				// ����ucs2���룬udl��¼����δ�������ݵĳ���
	}
	else if(dcs == SMS_PDU_ENCODING_7BIT)		// ����7bit���룬udl��¼����δ�������ݵĳ���
	{
		if(udhi == 1)
		{
			smssubmit.tp_udl = (size*8/7)+1;	// ���ڼ������ţ�udlΪԭʼ���ݳ���+1
		}
		else
		{
			smssubmit.tp_udl = (size*8/7);		// ���ڼ������ţ�udl�պ�Ϊԭʼ���ݳ���
		}
	}	
	else if(dcs == SMS_PDU_ENCODING_8BIT)
	{
		smssubmit.tp_udl = size;				// ����ucs2���룬udl��¼����δ�������ݵĳ���
	}

	// ���������Ĵ��������ݵ�uds
	if(udhi == 1)
	{
		memcpy((unsigned char*)smssubmit.tp_uds, udh, len_udh);
		len_encoded += len_udh;
		 
		memcpy(smssubmit.tp_uds+len_udh, encoded, size);
		len_encoded += size;

		smssubmit.tp_udl += len_udh;
	}
	else if(udhi == 0)
	{
		memcpy((unsigned char*)smssubmit.tp_uds, encoded, size);	
		len_encoded += size;
	}
	
	// package entire PDU, which should be in ASCII format(PDU dose not include SCA part, 0x1A and \r\b).
	// 1, 0x1A(unsigned char, 1 size) aattached to PDU string;
	// 2, \r\n(0x0D, 0x0A) attached further;
	len = sprintf(pdu, "%s%02x%02x%s%02x%02x%02x%02x",	buf_sca,	
														arg,
														smssubmit.tp_mr,
														buf_da,
														smssubmit.tp_pid,
														smssubmit.tp_dcs,
														smssubmit.tp_vp,
														smssubmit.tp_udl);															

	pdu += len;
	len_fnl += len;

	len = pdu_htoa(pdu, (unsigned char*)smssubmit.tp_uds, len_encoded);	

	pdu += len;
	len_fnl += len;	

	// ���PDU�ַ�����β����0x1A
	len = sprintf(pdu, "\x1A\r\n");						// �������ֽڲ������ڳ�����	
	
	len_fnl += len;		

	return ((len_fnl - 3)/2 - 1 - smssubmit.sca_len);	// ���س�SCA(����sca_len�ֽڵ�sca���ݺ�sca_len�ֽڱ���)�����pdu���ֽ���
}

// ��ʮ�������ַ�����ʽ��pdu�����벢���ؽ����������ݵ��ַ���(�ַ�����Ϊascii�����ucs2����)��
int pdu_deconstruct(T_SMS_DELIVER * deliver, char* asc)
{
	unsigned char	buf[320];
	unsigned char	arg;
	unsigned int 	len = 0;

	// ��ʮ�������ַ���ת��Ϊ�������ֽ���(���Ƚ�����)
	len = pdu_atoh((unsigned char*)buf, asc, strlen(asc));

	// ��ת����õ��Ķ������ֽ������asc����
	memcpy((unsigned char*)asc, buf, len);
	
	// ���SCA���볤��
	deliver->sca_len = *asc++;

	// ��SCA���볤�ȴ���0�����һ�����SCA�����ʽ����ȡSCA����
	if(deliver->sca_len > 0)
	{				
		// ��ȡSCA�����ʽ
		deliver->sca_fmt = *asc++;
	
		// ��ȡSCA����(sca���볤�ȼ�����������ʽ�ֽڣ���˴�sca���볤��Ϊsca���볤�ȼ�ȥ1)
		memcpy((unsigned char*)deliver->sca_str, asc, deliver->sca_len - 1);
		
		// ���������ֽ���ת��Ϊʮ�������ַ���
		len = pdu_htoa((char*)buf, (unsigned char*)deliver->sca_str, deliver->sca_len - 1);

		// ��ʮ�������ַ�����2�ַ�Ϊ��λ���ڲ�������
		pdu_recover((char*)deliver->sca_str, (char*)buf, len);

		// // printf("deliver->sca_str = %s", deliver->sca_str);

		asc += (deliver->sca_len - 1);		
	}

	// handle TP_MTI and friends	
	arg = *asc++;

	deliver->tp_mti 	= (arg&0x03);
	deliver->tp_mms 	= (arg&0x04)>>2;
	deliver->tp_sri 	= (arg&0x20)>>5;
	deliver->tp_udhi 	= (arg&0x40)>>6;
	deliver->tp_rp 		= (arg&0x80)>>7;

	// handle OA	
	deliver->tp_oal = *asc++;	
	deliver->tp_oat = *asc++;
	
	len = (deliver->tp_oal&1?deliver->tp_oal+1:deliver->tp_oal)/2;
	
	memcpy((unsigned char*)deliver->tp_oas, asc, len);

	asc += len;
	
	len = pdu_htoa((char*)buf, (unsigned char*)deliver->tp_oas, len);
	
	pdu_recover((char*)deliver->tp_oas, (char*)buf, len);

	// // printf("deliver->tp_oas = %s", deliver->tp_oas);
		
	// handle PID
	deliver->tp_pid = *asc++;
	
	// handle DCS	
	deliver->tp_dcs = *asc++;
	
	// handle SCTS
	memcpy((unsigned char*)deliver->tp_scts, asc, 7);

	asc += 7;	
	
	len = pdu_htoa((char*)buf, (unsigned char*)deliver->tp_scts, 7);
	
	pdu_recover((char*)deliver->tp_scts, (char*)buf, len);	
	// // printf("deliver->tp_scts = %s\r\n", deliver->tp_scts);

	// handle UD	
	deliver->tp_udl = *asc++;

	// ���user data�Ƿ����ͷ������
	if(deliver->tp_udhi == 1)
	{
		deliver->tp_udhl = *asc++;
		
		memcpy((unsigned char*)deliver->tp_udh, asc, deliver->tp_udhl);
		
		asc += deliver->tp_udhl;
		
		memcpy((unsigned char*)deliver->tp_uds, asc, deliver->tp_udl -1 - deliver->tp_udhl);

		deliver->tp_udl = deliver->tp_udl -1 - deliver->tp_udhl;
	}
	else
	{
		memcpy((unsigned char*)deliver->tp_uds, asc, deliver->tp_udl );

		/*
		// printf("deliver->tp_uds = ");
		
		for(i = 0; i < deliver->tp_udl; i++)
		{
			// printf("%02x", deliver->tp_uds[i]);
		}

		// printf("\r\n");
		*/
	}

	// Ӣ�Ŀɲ���7bit��8bit����ucs2����(��360�ֻ���ʿ�Ķ��Ź���)����������ֻ�ܲ���ucs2���롣
	// ���Ӣ�Ķ������ݿ�ʹ��strcpy���������������Ķ���ֻ��ʹ��memcpy����������
	if((deliver->tp_dcs&0x0C) == (SMS_PDU_ENCODING_7BIT))			// 7-bit encoding
	{
		len = pdu_decode7bit((char*)buf, (unsigned char*)deliver->tp_uds, deliver->tp_udl);
	
		// memcpy((char*)deliver->tp_uds, (char*)buf, len);	
		strcpy((char*)deliver->tp_uds, (char*)buf);	
		
		return len;		// ���ؽ����������ݵ��ַ���(����ascii���룬ÿ���ַ�ռ��1���ֽ�)
	}
	else if((deliver->tp_dcs&0x0C) == (SMS_PDU_ENCODING_8BIT))		// 8-bit encoding
	{
		len = pdu_decode8bit((char*)buf, (char*)deliver->tp_uds, deliver->tp_udl);

		// memcpy((char*)deliver->tp_uds, (char*)buf, len);
		strcpy((char*)deliver->tp_uds, (char*)buf);

		return len;		// ���ؽ����������ݵ��ַ���(����ascii���룬ÿ���ַ�ռ��1���ֽ�)
	}
	else if((deliver->tp_dcs&0x0C) == (SMS_PDU_ENCODING_UCS2))		// UCS2 encoding
	{
		len = pdu_decode16bit((unsigned char*)buf, (unsigned char*)deliver->tp_uds, deliver->tp_udl);

		// �������ݲ���ucs2����ʱ��Ӧ������Ϊ�������ֽ�������
		memcpy((unsigned char*)deliver->tp_uds, (unsigned char*)buf, len);

		return len/2;		// ���ؽ����������ݵ��ַ���(����ucs2���룬ÿ���ַ�ռ��2���ֽ�)
	}
	else							
	{	
		return -1;		// ���������ʽ�Ķ������ݲ���֧��				
	}
}

// ��UTF-8������ַ���(UTF8�������ASCII����)ת����UCS2(Unicode Big Endian)������ֽ�����
int utf8_to_ucs2( unsigned char* ucs2, char* utf8, int len)
{
	unsigned int i = 0;
	unsigned int j = 0;
	
	unsigned short doublebytes = 0x0000;

	while(i < len)
	{
		doublebytes = 0x0000;

		if(utf8[i] < 0x80)							// ASCII�ַ�
		{
			ucs2[j++] = 0x00;
			ucs2[j++] = utf8[i++];		
		}
		else if(utf8[i] >= 0xE0 && utf8[i] < 0xF0)	// �����ֽں��� 1110**** 10****** 10******  
		{
			doublebytes	 =  (utf8[i++])<<12;
			doublebytes |= ((utf8[i++]&0x3F)<<6);
			doublebytes |=  (utf8[i++]&0x3F);

			ucs2[j++] = (doublebytes>>8)&0xFF;
			ucs2[j++] = doublebytes&0xFF;			
		}
		else	// ���ڼȷ�ASCII�����ַ�UTF8������ַ������账���ֱ�ӷ��ش���
		{
			// printf("neither utf8 nor ascii encoded character found in stream!\r\n");

			return NG;
		}
	}
	
	return j;	// ����ת�����ucs2�ֽ���
}

