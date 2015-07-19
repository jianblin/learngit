#include "string.h"
#include "stdio.h"
#include "stdlib.h"

#include "common.h"

#include "mcu_usart.h"

#include "log_pos.h"

unsigned char	sys_language;		// ϵͳ������������

char				swversion[16+1];		// ϵͳ����汾��
char				hwversion[16+1];		// ϵͳӲ���汾��

// crc16����ʱ�Ĳ��ұ�
const unsigned short crc16tab[256]= 
{
	0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
	0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
	0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
	0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
	0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
	0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
	0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
	0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
	0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
	0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
	0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
	0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
	0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
	0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
	0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
	0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
	0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
	0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
	0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
	0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
	0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
	0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
	0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
	0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
	0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
	0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
	0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
	0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
	0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
	0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
	0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
	0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};

// ����ָ�����뻺�������ݵ�crc16У��͡�
unsigned short crc16_ccitt(unsigned char* buf, int size)
{
	register int counter;

	register unsigned short crc = 0;

	for( counter = 0; counter < size; counter++)
	{
		crc = (crc<<8) ^ crc16tab[((crc>>8) ^ *buf++)&0x00FF];
	}

	return crc;
}

// ��ʮ�����Ʊ�ʾ����ֵ�ַ���ת��Ϊ������ֵ��
int __atoih(char* asc_h)
{
	int len = strlen(asc_h);
	int	val = 0;
	int	i;

	for(i = 0; i < len; i++)
	{
		if(asc_h[i] >= '0' && asc_h[i] <= '9')
		{
			val += asc_h[i]-'0'+0;
		}
		else if(asc_h[i] >= 'a' && asc_h[i] <= 'f')
		{
			val += asc_h[i]-'a'+10;
		}
		else if(asc_h[i] >= 'A' && asc_h[i] <= 'F')
		{
			val += asc_h[i]-'A'+10;
		}
		else
		{
			return val;
		}

		// �������һ���ַ�ʱ����λ
		if(i < (len-1))
		{
			val <<=4; 
		}
	}

	return val;
}

// �����ܰ�����������Ϣ��ʮ�����ַ���ת��Ϊ��ֵ��
int __atoid(char* asc_d)
{
	char* 	ptr   = asc_d;
	int		sign;
	int 	val = 0;
	
	if(*ptr == '+')
	{
		sign = 1;

		ptr++;
	}
	else if(*ptr == '-')
	{
		sign = -1;

		ptr++;
	}
	else
	{
		sign = 1;
	}

	while(*ptr != '\0')
	{		
		if(*ptr >= '0' && *ptr <= '9')
		{
			val *= 10;

			val += *ptr-'0';	

			ptr++;
		}
		else
		{
			break;
		}
	}

	return sign*val;
}

// ��ʮ����(֧�ָ���)��ʮ������(��0x��0X��ͷ����֧�ָ���)��ʾ����ֵ�ַ����仯Ϊ��ֵ��
int	__atoi(char* asc)
{	
	// �Թ��ַ���ǰ��Ŀո�
	while(*asc == ' ')
	{
		asc++;
	}

	// ����Ƿ��0x��0Xǰ׺
	if((asc[0] == '0' && asc[1] == 'x') || (asc[0] == '0' && asc[1] == 'X'))
	{
		// ����0x��0Xǰ׺������ʮ������ת��
		return __atoih((char*)asc+2);
	}
	else
	{
		// ������0x��0Xǰ׺������ʮ����ת��
		return __atoid((char*)asc);
	}
}

// ���㸡�����ľ���ֵ��
float __absf(float v)
{
	v = v>0.000001?v:-1.0*v;

	return v;
}

// ����short�����ݵľ���ֵ��
unsigned short __abs16(short val)
{
	val = val>=0?val:-1*val;

	return val;
}

// ��Դ�ַ�������ȡָ���������ַ���ΪĿ���ַ������Զ�����ַ�����β���š�
void __strncpy(char* dst, char* src, int n)
{
	int i;

	for(i = 0; i < n; i++)
	{
		dst[i] = src[i];
	}

	dst[i++] = '\0';
}

// �Ƚ�ָ���������ַ���������ȫƥ���򷵻�0�����򷵻ز�ƥ�䴦���α�ֵ��
int __strcmp(char* dst, char* src)
{
	int i;
	
	int len_src;
	int	len_dst;

	len_src = strlen(src);
	len_dst = strlen(dst);

	if(len_dst != len_src)
	{
		return -1;				// dst�ַ�����src�ַ������Ȳ���ȣ�����-1
	}
	
	for(i = 0; i < len_src; i++)
	{		
		if((unsigned char)dst[i] != (unsigned char)src[i])
		{
			return i+1;			// ƥ��ʧ��ʱ�����ز�ƥ�䴦���ַ����
		}
	}

	// ��ȫƥ��ɹ��󣬷���0
	return 0;
}

// ��ָ��Դ�ַ�����˫����֮����ַ���ȡ���������浽ָ����Ŀ���ַ����С�
int fetch_quotes(char* dst, char* src)
{
	int i = 0;
	int j = 0;
	
	// ��λ����һ��˫���Ŵ�
	while(src[i++] != '"');

	for(;;)
	{
		if(src[i] != '"')
		{
			// ��ȡ��һ��˫���ź͵ڶ���˫����֮����ַ�
			dst[j++] = src[i++];
		}
		else
		{
			// �����ڶ���˫����ʱ�����ַ���ȡ
			dst[j++] = '\0';

			// ���صڶ���˫���ŵ���һ���ַ����α�ֵ
			return ++i;
		}
	}
}

// ��ָ��λ�ÿ�ʼ��ȡ������ʽ���ַ�����֧��С���㣩��
int fetch_digits(char* dst, char* src)
{
	int i  = 0;
	
	while(*src > '9' || *src < '0')
	{
		src++;

		i++;
	}

	while((*src >= '0' && *src <= '9') || *src == '.')
	{
		*dst++ = *src++;

		i++;
	}

	*dst++ = '\0';

	// ����Դ�ַ������α��ƶ��ľ���
	return i;
}

// ��ȡ�Զ��Ÿ����Ķ����¼(����SMS�ط���¼��SMS���������)��
int fetch_domain_in_sms_record(char* dst, unsigned char* src)
{
	int i = 0;
	int j = 0;

	for(i = 0; ; i++)
	{
		if(src[i] != ',')
		{
			dst[j++] = src[i];
		}
		else
		{
			dst[j++] = '\0';

			return j;
		}
	}
}

// ��dtmf�����ַ�������ȡ��*��������
int fetch_domain_in_dtmf_command(char* domain, char* dtmf)
{
	int i = 0;
	int	j = 0;
	
	int	len = strlen(dtmf);

	strcpy(domain, "");

	while(i < len && dtmf[i++] != '*');

	while(i < len)
	{
		if(dtmf[i] != '#')
		{
			domain[j++] = dtmf[i++];
		}
		else
		{
			domain[j++] = '\0';
			
			break;
		}
	}

	// ������ȡ�����򳤶�
	return j;
}

// ���ָ���ַ������Ƿ�Ϊȫ�����ַ���
BOOL is_all_digit(char* str)
{
	int	i;
	int 	len = strlen(str);

	for(i = 0; i < len; i++)
	{
		if(!(str[i] >= '0' && str[i] <= '9'))
		{
			return FALSE;
		}
	}

	return TRUE;
}

// ���ָ���ַ����Ƿ�Ϊȫascii�ַ���
BOOL is_all_ascii(char* str)
{
	int i;
	int len = strlen(str);
	
	for(i = 0; i < len; i++)
	{
		if(str[i] > 0x7F)
		{
			return FALSE;
		}		
	}

	return TRUE;
}

#ifdef USING_DEV_GSM
// ��ʽ��gsm����(ͳһΪ������������ǰ׺�Ĵ����ָ�ʽ)��
// ע: gsm������ϵͳ�����ı߽��Ͼ͸�ʽ���������յ����ź󡢽�������������������ǰ��ʽ�����ͷ����룬���Ͷ��ų�ȥǰ��ʽ��Ŀ����롣
int format_pn(char* pn)
{
	char*	ptr;
	int 	len = strlen(pn);
	int		n = -1;
	
	if(len <= 0)		// ��������
	{
		return OK;
	}	
	else if(len == 1)	// TCP����
	{ 
		n = atoi(pn);
		
		if(n >=0 && n < MAX_NUM_TCP_CONN)
		{			
			return OK;
		}
		else
		{
			return NG;
		}
	}
	else if(len < MAX_LEN_PN)
	{
		ptr = strstr(pn, gsm_telecode);

		if(ptr == pn)
		{
			// ȥ����������
			strcpy(pn, pn+strlen(gsm_telecode));				
		}

		return is_all_digit(pn);
	}
	else
	{
		return NG;		
	}
}
#endif

// ��ָ���Ķ������ֽ�����ӡ�������ҿ�ָ����ӡ��ǰ���ַ�����
void dump_bin(unsigned char* data, int size, char* remark)
{
	int i ;

	printf("%s", remark);

	for(i = 0; i < size; i++)
	{
		printf("%02x", data[i]);
	}

	printf("\r\n");
}

// ���ָ���ַ����Ƿ�Ϊ�����ֻ��š�
int is_float_digit(char* str)
{
	int i;
	int	len = strlen(str);

	for(i = 0; i < len; i++)
	{
		if(!((str[i] >= '0' && str[i] <= '9') || str[i] == '.'))
		{
			return NG;
		}
	}

	return OK;
}

// ͳ��ָ���ַ�����ָ���ַ��ĸ�����
int cal_char_num(char* str, char ch)
{
	int i;
	int len = strlen(str);
	int	num = 0;

	for(i = 0; i < len; i++)
	{
		if(str[i] == ch)
		{
			num++;
		}
	}

	return num;
}

// ������ת��Ϊ��Ȼʱ��(�졢ʱ���֡���)��
void second_to_dhms(unsigned int seconds, T_TIME_DHMS* dhms)
{
	// ��������
	dhms->second = seconds%60;
	seconds	     /= 60;			

	// ���������
	dhms->minute = seconds%60;	
	seconds	     /= 60;			

	// ����Сʱ��
	dhms->hour   = seconds%24;	
	seconds	  	 /= 24;			

	// ��������
	dhms->day    = seconds;
}

