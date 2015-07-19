/*�˶�������*/
#include "common.h"
#include "log_motion.h"
#include "log_queue.h"
#include "log_pos.h"
#include "dev_mma845x.h"
#include "mcu_i2c.h"
#include "log_power.h"
#include "mcu_exti.h"

#include "mcu_systick.h"
#include "mcu_gpio.h"

#include "stdio.h"
#include "stdlib.h"
#include "math.h"

#include "stm32f10x.h"


// �ն˵�ǰ���˶�״̬
volatile STATUS	sts_motion;	

int				swtimer_still_lasting;		// ����豸������ֹ�Ķ�ʱ��(������ֹ�¼��ﵽ�����򴥷�������ֹ�¼�)��
BOOL			is_still_lasting;	

// �෭������ȫ�ֱ���
unsigned int 		cnt_rollint;
unsigned int		swtimer_rollint;

unsigned char	sw_detect_still;	
unsigned char	sw_detect_motion; 
int 				swtimer_still_detection;

unsigned int		err_detect_motion;			// �˶�������ֵ����

BOOL			rollint_filtered;		// ϵͳ��ʼ���󷭹�����ж��Ƿ񱻹��˵�

/********************************************************************************************************************
*
*
*
*
*********************************************************************************************************************/
void motion_init(void)
{	
	cnt_rollint 		= 0;
	swtimer_rollint 	= 0;
	rollint_filtered	= FALSE;

	// ��ʼʱ���ն˵��˶�״̬δ֪�������������ֹ��ⶨʱ���Լ����ܵľ�ֹ״̬
	sw_detect_still = OFF;

	sw_detect_motion = OFF;	
	
	// ��ʼʱ���˶�״̬δ֪
	sts_motion = STS_MOTION_UNKNOWN;

	// �򿪾�ֹ���
	detect_still(ON);

	// ���˶����
	detect_motion(ON);
}

#if 0

// ��/�ر�data ready��⡣
void sw_detect_dataready(SWITCH sw)
{
	u8 n;
	
	// ���ò���֮ǰ���Ƚ���������Ϊ����ģʽ
	MMA845x_Standby();
	
	if(sw == OFF)
	{		
		// ��ֹ��ֹ�¼��ж�
		n = I2C_RegRead(REG_CTRL_REG4);
		I2C_RegWrite(CTRL_REG4, n & ~INT_EN_DRDY_MASK);	
	}
	else
	{
		// ������INT1�ϲ����ж�
		n = I2C_RegRead(CTRL_REG5);		
		I2C_RegWrite(CTRL_REG5, n | INT_CFG_DRDY_MASK);			

		// ʹ�ܾ�ֹ�¼��ж�
		n = I2C_RegRead(REG_CTRL_REG4);
		I2C_RegWrite(REG_CTRL_REG4, n | INT_EN_DRDY_MASK);	
	}

	// �������������Ϊ�ģʽ����������Ч
	MMA845x_Active ();	
}

// ��/�رվ�ֹ��⡣
void sw_detect_aslp(SWITCH sw, float ms, unsigned char ints)
{
	u8 n;

	unsigned char cnt = (unsigned char)(ms/320);

	// ���ò���֮ǰ���Ƚ���������Ϊ����ģʽ
	MMA845x_Standby();	

	if(sw == OFF)
	{
		// �ر�XYZ�����ϵľ�ֹ�¼����
		n = I2C_RegRead(REG_CTRL_REG2);
		I2C_RegWrite(REG_CTRL_REG2, n & ~0x04);  
		
		// ��ֹ��ֹ�¼��ж�
		n = I2C_RegRead(REG_CTRL_REG4);
		I2C_RegWrite(CTRL_REG4, n & ~INT_EN_ASLP_MASK);	
	}
	else
	{
		// ���þ�ֹ��ⴰ�ڳ���Ϊ3.2�룬320 ms/step
		I2C_RegWrite(REG_ASLP_COUNT, cnt);		

		// ʹ���𶯡�λ�ƿɻ���	
		n = I2C_RegRead(REG_CTRL_REG3);
		I2C_RegWrite(REG_CTRL_REG3, n | (ints << 3));		// active low, push-pull

		// ������INT1�ϲ����ж�
		n = I2C_RegRead(CTRL_REG5);		
		I2C_RegWrite(CTRL_REG5, n | INT_CFG_ASLP_MASK);			

		// �򿪾�ֹ�¼����
		n = I2C_RegRead(REG_CTRL_REG2);
		I2C_RegWrite(REG_CTRL_REG2, n | 0x04);  

		// ʹ�ܾ�ֹ�¼��ж�
		n = I2C_RegRead(REG_CTRL_REG4);
		I2C_RegWrite(REG_CTRL_REG4, n | INT_EN_ASLP_MASK);	

		// ���ݼĴ����е����ݹ��˹ر�
		n = I2C_RegRead(REG_XYZ_DATA_CFG);
		// I2C_RegWrite(REG_XYZ_DATA_CFG, n | 0x10);	// HPF on
	}

	// �������������Ϊ�ģʽ����������Ч
	MMA845x_Active ();	
}

// ��/�رշ���仯��⡣
/*
1�����ٶȴ������κ�ʱ�򶼻ᴦ���ض��ķ������һ���򿪷���仯��Ⲣʹ�ܶ�Ӧ�жϣ����������������
   �仯�жϣ����жϿ������жϼ��ٶȴ������ĳ�ʼ״̬;
2������仯�������X���Y��(���ڼ�����������)���нǳ���45��(����+-15�ȵ�ԣ��)����Ϊ��������˱仯;
*/
void sw_detect_lndprt(unsigned char sw, float ms)
{
	u8 n;

	unsigned char cnt = (unsigned char)(ms/10);

	// ���ò���֮ǰ���Ƚ���������Ϊ����ģʽ
	MMA845x_Standby();	

	if(sw == OFF)
	{
		// �رշ���仯���
		n = I2C_RegRead(REG_PL_CFG);		
		I2C_RegWrite(REG_PL_CFG, 0x00);

		// ��ֹ����仯�¼��ж�
		n = I2C_RegRead(REG_CTRL_REG4);		
		I2C_RegWrite(REG_CTRL_REG4, n & ~INT_EN_LNDPRT_MASK);
	}
	else
	{
		// ʹ����б��⡢ȥ��������ʹ�õݼ�ģʽ(����ڼ��������Ϲ��¼��ݼ�ȥ��������)
		I2C_RegWrite(REG_PL_CFG, 0x40); 

		// ����ȥ�����ڳ���
		I2C_RegWrite(REG_PL_COUNT, cnt); 
		
		// 0x13��0x14�Ĵ���ֻ����MMA8451Q���޸ģ�ȫϵ��оƬ���ɶ�

		// ʹ�ܷ���仯�ж�
		n = I2C_RegRead(REG_CTRL_REG4);
		I2C_RegWrite(REG_CTRL_REG4, n | INT_EN_LNDPRT_MASK);

		// ������INT1�ϲ����ж�
		n = I2C_RegRead(REG_CTRL_REG5);		
		I2C_RegWrite(REG_CTRL_REG5, n | INT_CFG_LNDPRT_MASK);

		// ���ݼĴ����е����ݹ��˹ر�
		n = I2C_RegRead(REG_XYZ_DATA_CFG);
		// I2C_RegWrite(REG_XYZ_DATA_CFG, n & ~0x10);	// HPF off
	}

	// �������������Ϊ�ģʽ����������Ч
	MMA845x_Active ();	
}

// ��/�ر������⡣
void sw_detect_pulse(unsigned char taps, SWITCH sw, float ms_lmt, float ms_lat, float ms_win)
{
	u8 n;

	unsigned char cnt_lmt = (unsigned char)(ms_lmt/10);
	unsigned char cnt_lat = (unsigned char)(ms_lat/5);
	unsigned char cnt_win = (unsigned char)(ms_win/5);

	// ���ò���֮ǰ���Ƚ���������Ϊ����ģʽ
	MMA845x_Standby();	

	if(sw == OFF)
	{
		// �رյ����¼����
		if(taps == 1)		// single tap
		{
			n = I2C_RegRead(REG_PULSE_CFG);
			I2C_RegWrite(REG_PULSE_CFG, n & ~0x15); 
		}
		else if(taps == 2)	// double tap
		{
			n = I2C_RegRead(REG_PULSE_CFG);
			I2C_RegWrite(REG_PULSE_CFG, n & ~0x2A); 
		}
		
		// ��ֹ�����¼��ж�
		n = I2C_RegRead(CTRL_REG4);
		I2C_RegWrite(CTRL_REG4, n & ~INT_EN_PULSE_MASK); 
	}
	else
	{	
		// ���������ϵĵ����¼������ٶ�����ֵ��0.0625 g/count @8g
		I2C_RegWrite(REG_PULSE_THSX, DEF_DETECT_SINGLETAP_THS);  		// 0.625 g/count
		I2C_RegWrite(REG_PULSE_THSY, DEF_DETECT_SINGLETAP_THS);  		// 0.625 g/count
		I2C_RegWrite(REG_PULSE_THSZ, DEF_DETECT_SINGLETAP_THS+16);  	// 0.625 g/count

		// ������ʱʹ�ø�ͨ�˲�������ʹ�õ�ͨ�˲���
		n = I2C_RegRead(REG_HP_FILTER_CUTOFF);	
		I2C_RegWrite(REG_HP_FILTER_CUTOFF, n & ~0x03); 		

		// ���õ��������������ʱ��Ϊ50ms��10 ms/count @100Hz, normal power scheme
		I2C_RegWrite(REG_PULSE_TMLT, cnt_lmt); 	

		// ѡ���ⵥ���¼���ʹ�ܵ����¼���־���浽PULSE_SRC�Ĵ�����ʹ��XYZ���ϵĵ����¼����
		if(taps == 1)		// single tap detection X, Y, Z
		{
			n = I2C_RegRead(REG_PULSE_CFG);
			I2C_RegWrite(REG_PULSE_CFG, n | 0x15 | 0x40);

			I2C_RegWrite(REG_PULSE_LTCY, cnt_lat); 
		}
		else if(taps == 2)	// double tap detection on X, Y, Z
		{
			n = I2C_RegRead(REG_PULSE_CFG);
			I2C_RegWrite(REG_PULSE_CFG, n | 0x2A | 0x40);

			I2C_RegWrite(REG_PULSE_LTCY, cnt_lat); 

			I2C_RegWrite(REG_PULSE_WIND, cnt_win); 
		}

		// ������INT1�ϲ����ж�
		n = I2C_RegRead(0x2E);		
		I2C_RegWrite(0x2E, n | INT_CFG_PULSE_MASK);

		// ʹ�ܵ����¼��ж�		
		n = I2C_RegRead(CTRL_REG4);
		I2C_RegWrite(CTRL_REG4, n | INT_EN_PULSE_MASK); 

		// ���ݼĴ����е����ݹ��˹ر�
		n = I2C_RegRead(REG_XYZ_DATA_CFG);
		// I2C_RegWrite(REG_XYZ_DATA_CFG, n & ~0x10);	// HPF off
	}

	MMA845x_Active();
}

// ��/�ر���������/�˶���⡣
void sw_detect_ffmt(unsigned char ffmt, unsigned char axis, float g, float ms)
{
	u8 n;

	// �����ٶ�������ֵת��Ϊ��Ӧ�Ĵ�����ֵ������ⴰ�ڳ�����ֵת��Ϊ��Ӧ�Ĵ�����ֵ
	unsigned char	ths = (unsigned char)(g/0.0625);
	unsigned char	cnt = (unsigned char)(ms/10);

	// // printf("ths = %d, cnt = %d for ffmt setting\r\n", ths, cnt);
	
	// ���ò���֮ǰ���Ƚ���������Ϊ����ģʽ
	MMA845x_Standby();	

	if(!(axis&0x07))
	{
		// �ر���������/�˶��¼����
		n = I2C_RegRead(REG_FF_MT_CFG);
		I2C_RegWrite(REG_FF_MT_CFG, n & ~(0x07<<3));
		
		// ��ֹ��������/�˶��¼��ж�
		n = I2C_RegRead(REG_CTRL_REG4);
		I2C_RegWrite(REG_CTRL_REG4, n & ~INT_EN_FF_MT_1_MASK);
	}
	else
	{
		// ѡ������������/�˶��¼�(�����˶��¼�)��ʹ����������/�˶��¼���־���浽FF_MT_SRC�Ĵ���
		I2C_RegWrite(REG_FF_MT_CFG, 0x00 | (axis<<3) | (ffmt<<6));		// ELE disabled�� OAE=0 for freefall detection

		// ������������/�˶���������ֵΪ0.375g��0.0625 g/count @8g
		// ע: PCBA��JLink�ʹ�������ʱ����������/�˶��������Ӧ��С��0.25g(����ֵ��С��4)��
		I2C_RegWrite(REG_FF_MT_THS, ths);	

		// ������������/�˶���ⴰ�ڳ���Ϊ40ms��10 ms/count @100 Hz
		// ע: ��������/�˶���ⴰ�ڳ���Ӧ�����ն���������/�˶��Ŀ��ܸ߶����ã�������״���£���������/�˶�120ms������߶�Ϊ7cm��
		I2C_RegWrite(REG_FF_MT_COUNT, cnt);			

		// ʹ����������/�˶��¼��ж�
		n = I2C_RegRead(REG_CTRL_REG4);
		I2C_RegWrite(REG_CTRL_REG4, n | INT_EN_FF_MT_1_MASK);		

		// ������INT2�ϲ����ж�
		n = I2C_RegRead(REG_CTRL_REG5);	
		
		I2C_RegWrite(REG_CTRL_REG5, n & ~INT_CFG_FF_MT_1_MASK);

		// ���ݼĴ����е����ݹ��˹ر�
		n = I2C_RegRead(REG_XYZ_DATA_CFG);
		// I2C_RegWrite(REG_XYZ_DATA_CFG, n & ~0x10);	// HPF off
	}

	// �������������Ϊ�ģʽ����������Ч
	MMA845x_Active ();	
}

// ��/�ر�˲̬��⡣
void sw_detect_trans(SWITCH hpf_sw, unsigned char axis, float g, float ms)
{
	u8 n;

	// �����ٶ�������ֵת��Ϊ��Ӧ�Ĵ�����ֵ������ⴰ�ڳ�����ֵת��Ϊ��Ӧ�Ĵ�����ֵ
	unsigned char	ths = (unsigned char)(g/0.0625);
	unsigned char	cnt = (unsigned char)(ms/10);

	// // printf("ths = %d, cnt = %d\r\n", ths, cnt);

	// // printf("ths = %d, cnt = %d for trans setting\r\n", ths, cnt);

	// ���ò���֮ǰ���Ƚ���������Ϊ����ģʽ
	MMA845x_Standby();	

	if(!(axis&0x07))
	{
		// �ر�XYZ�����ϵ��𶯼��
		n = I2C_RegRead(REG_TRANSIENT_CFG);
		I2C_RegWrite(REG_TRANSIENT_CFG, n & ~(0x07<<1));
		
		// ��ֹ���¼��ж�
		n = I2C_RegRead(CTRL_REG4);
		I2C_RegWrite(CTRL_REG4, n & ~INT_EN_TRANS_MASK);
	}
	else
	{
		// ʹ��˲̬�¼���־���浽TRANSIENT_SRC�Ĵ��������뵽˲̬���ͨ��������ʹ�ø�ͨ�˲����˲�
		if(hpf_sw == ON)
		{
			// ����˲̬�������ݹ��˴�(Transient event flags are NOT latched into the TRANSIENT_SRC register.)
			I2C_RegWrite(REG_TRANSIENT_CFG, 0x00 | (axis<<1) & ~0x01);		// ELE disabled.

			// ���ݼĴ����е����ݹ���Ҳ��
			n = I2C_RegRead(REG_XYZ_DATA_CFG);
			I2C_RegWrite(REG_XYZ_DATA_CFG, n | 0x10);	// HPF on
		}
		else if(hpf_sw == OFF)
		{	
			// ����˲̬�������ݹ��˴�(Transient event flags are NOT latched into the TRANSIENT_SRC register.)
			I2C_RegWrite(REG_TRANSIENT_CFG, 0x00 | (axis<<1) |  0x01);		// ELE disabled.
		}				

		// �����𶯼�������ֵΪ~0.4g��0.0625 g/count @100Hz
		// ע: ���𶯼������ֵ��Ϊ~0.2g������ٶȴ�����������������������(ͨ��˦��PCBA����)��
		I2C_RegWrite(REG_TRANSIENT_THS, ths);			

		// �����𶯼�ⴰ�ڳ���Ϊ100ms��10 ms/count @100Hz
		I2C_RegWrite(REG_TRANSIENT_COUNT, cnt);

		// ���ø�ͨ�˲����Ľ�ֹƵ��Ϊ8Hz, @400Hz fs
		// Ϊ��ȷ�������𶯺��ٴ�Ƕ���б��ֹ�ڷ�ʱ������������񶯼���жϣ�
		// ��ͨ�˲����Ľ�ֹƵ�ʲ��õ���8Hz��
		n = I2C_RegRead(REG_HP_FILTER_CUTOFF);
		I2C_RegWrite(REG_HP_FILTER_CUTOFF, (n & ~0x03) | 0x01);

		// ʹ�����¼��ж�
		n = I2C_RegRead(CTRL_REG4);
		I2C_RegWrite(CTRL_REG4, n | INT_EN_TRANS_MASK);

		// ������INT2�ϲ����ж�
		n = I2C_RegRead(CTRL_REG5);
		I2C_RegWrite(CTRL_REG5, n & ~INT_CFG_TRANS_MASK);
	}
	
	MMA845x_Active ();	// �������������Ϊ�ģʽ����������Ч
}

#endif

// ��/�رվ�ֹ�¼���⡣
void detect_still(SWITCH sw)
{
	if(sw == ON)
	{
		swtimer_still_detection = DEF_SWTIMER_STILL_DETECTION;

		sw_detect_still = ON;
	}
	else
	{
		sw_detect_still = OFF;
	}
}

// ��/�ر��˶��¼����(ʹ��FF+TRANS�бռ�⣬ÿ�δ򿪼бռ��ʱ�����Ե�ǰ���ٶ�ֵΪ��׼������Խ����)��
void detect_motion(SWITCH sw)
{
	if(sw == ON)
	{
		sw_detect_motion = ON;
	}
	else
	{
		sw_detect_motion = OFF;
	}		
}

#if 0

// ��/�ر����¼���⡣
void detect_shake(SWITCH sw)
{
	if(sw == ON)
	{
		sw_detect_trans(ON, DEF_DETECT_SHAKE_AXIS, DEF_DETECT_SHAKE_THS, DEF_DETECT_SHAKE_CNT);
	}
	else
	{
		sw_detect_trans(ON, 0x00, 0, 0);
	}
}

// ��/�رշ����¼���⡣
// ע: (�б�)�˶�����ʱ�����㾲ֹб��Ҳ�����������������жϣ���˷�����ⲻ�ܺ�(�б�)�˶����ͬʱ�򿪡�
// 
// �����ķ��������ֳ������:1�������෭�󼴿̸�λ;2�������෭90�Ⱥ�ֹͣ����;3�������෭90�Ⱥ��������;
// �ڵ�1�͵�3������»��ڶ�ʱ���ڲ������η���仯�жϣ������жϲ���ʱZ��ļ��ٶ���ֵ��Ȼ��ͬ(���1��Z����ٶȽӽ�-1g��
// ���3��Z����ٶȽӽ�+0.50g)����2�������Z����ٶȽӽ�0g��
void detect_roll(SWITCH sw)
{
	if(sw == ON)
	{
		// printf("turned on detection on roll.\r\n");
		
		sw_detect_lndprt(ON, DEF_DETECT_ROLL_CNT);

		// �򿪷���仯����жϺ��ڷ���仯����жٴ������лῪʼִ����Ӧ�ļ���㷨
	}
	else
	{
		// printf("turned off detection on roll.\r\n");
		
		sw_detect_lndprt(OFF, 0);
	}
}

// ��/�رյ����¼���⡣
void detect_singletap(SWITCH sw)
{
	if(sw == ON)
	{	
		sw_detect_pulse(1, ON, DEF_DETECT_SINGLETAP_CNT_LMT, 0, 0);
	}
	else
	{
		sw_detect_pulse(1, OFF, 0, 0, 0);
	}
}

// ��/�ر�˫���¼���⡣
// MMA8452Q�Ὣһ��˫������ʶ���1�ε���+1��˫��������ż������ʶ���1������+1��˫��+1������������
void detect_doubletap(SWITCH sw)
{
	if(sw == ON)
	{
		sw_detect_pulse(2, ON, DEF_DETECT_SINGLETAP_CNT_LMT, DEF_DETECT_DOUBLETAP_CNT_LAT, DEF_DETECT_DOUBLETAP_CNT_WIN);
	}
	else
	{
		sw_detect_pulse(2, OFF, 0, 0, 0);
	}
}

#endif

/********************************** ���ּ���¼�����ʱ���жϴ����� *****************************************/
// ��ֹ�¼���⵽��Ĵ�������
void event_handler_still(void)
{			
	// ����Ϊȷ����ֹ״̬
	sts_motion = STS_MOTION_STILL;

	// ��⵽��ֹ�¼������ó�����ֹ��ⶨʱ��(�����豸��δ���ڳ�����ֹʱ�ż�������ֹ�¼���
	// ���Ѿ����ڳ�����ֹ״̬����Ӧ���ټ�������ֹ�������ظ�)��
	swtimer_still_lasting = CYCLE_STILL_LASTING*1000/SYSTICK_PERIOD;

	// Ϩ��������ָʾ��⵽��ֹ�¼���
	// led_switch(GPIO_LED_R, OFF);

	// �ر��˶����ʱ�Ĵ�ӡ��䣬����������ȼ��ģʽ��Ƶ����⵽���˶��¼���ӡ���̫���ռ���ж�ʱ�䡣
#if 0
	printf("----------------\r\n");
	printf("still detected. \r\n");
	printf("----------------\r\n");
#endif

	// ���Ķ�λ����
	cycle_periodic_pos = DEF_CYCLE_PERIODIC_POS_WHEN_STILL;
}

// �˶��¼���⵽��Ĵ�������
void event_handler_motion(void)
{
	sts_motion = STS_MOTION_MOTIONING;

	// ��⵽�˶���رճ�����ֹ��ⶨʱ��
	swtimer_still_lasting = 0;
	is_still_lasting = FALSE;

	// ����������ָʾ��⵽�˶��¼���
	// led_switch(GPIO_LED_R, ON);

	// �ر��˶����ʱ�Ĵ�ӡ��䣬����������ȼ��ģʽ��Ƶ����⵽���˶��¼���ӡ���̫���ռ���ж�ʱ�䡣
#if 0
	printf("----------------\r\n");
	printf("motion detected.\r\n");
	printf("----------------\r\n");
#endif

	// ���Ķ�λ����
	cycle_periodic_pos = DEF_CYCLE_PERIDOIC_POS_WHEN_MOTIONING;
}

#if 0
// �����¼�����жϴ�������
void event_handler_roll(void)
{		
	// ÿ�������ķ���仯�������У�ÿ����һ�η���仯����жϣ���Ӧ����������
	cnt_rollint++;

	// // printf("\r\ncnt_rollint = %d\r\n", cnt_rollint);

	// �򿪷���仯���󣬴��ھ�ֹ״̬�Ĵ��������Զ�����һ������仯�жϣ����ж�Ӧ�Թ�
	if(rollint_filtered == FALSE)
	{		
		if(cnt_rollint == 1)
		{
			return;
		}
		else if(cnt_rollint == 2)
		{
			rollint_filtered = TRUE;

			cnt_rollint = 0;

			return;
		}
	}
	
	// ��鷽��仯������ⶨʱ���Ƿ�Ϊ��λ״̬���ǵĻ�������֮���Կ�ʼһ���µ���������仯�����̡�
	// �˶�ʱ���ĳ���Ӧ����ǡ��: ���̿���û�����͹������������ķ���仯�жϣ������ᵼ����ν��ʱ���˷ѡ�
	if(swtimer_rollint == 0)
	{
		swtimer_rollint = DEF_ROLLINT_INTERVAL/SYSTICK_PERIOD;
	}	

	// �����жϼ�������ʱ������󣬻��������ʱ������жϴ������ڸ���
	// �����жϸ����жϾ���ķ����¼����Ͳ�����Ӧ����
}

// �����¼�����жϴ�������
void event_handler_singletap(void)
{

}

// ˫���¼�����жϴ�������
void event_handler_doubletap(void)
{

}

#endif

// ����ָ�������̣���ָ����Ĵ�����MSB�ֽں�LSB�ֽ�����ת��Ϊ���ٶ���ֵ��
// ע: 2g��4g��8g�����¾��Ѳ��Դ˺�������������
float reg2val(unsigned char msb, unsigned char lsb)
{
	unsigned short 	reg;
	float			val;

	// ��MSB��LSBƴ�ӳ�16λ�޷�����
	reg = (unsigned short)((msb<<8)|lsb);

	// ����ƴ�Ӻ�Ĵ�����ֵ���λ�Ƿ�Ϊ1���жϼ��ٶ���ֵ�Ƿ�Ϊ�����ٸ������̼���Ĵ�����ֵ��Ӧ�ļ��ٶ���ֵ
	if(reg & 0x8000)
	{
		// �������Ʋ����ʾ�ļĴ�����ֵתΪ��ʮ��������
		reg = reg-(1<<4);	// ��1(ע��16λ�����е�ĩβ4λΪ��Чλ)
		reg = ~reg;			// ȡ��

		val = (float)DEF_GSENSOR_DETECT_RANGE*(float)(reg>>4)/(float)(1<<(12-1));		
		
		return -1*val;
	}
	else
	{
		// �������Ʋ����ʾ�ļĴ�����ֵתΪ��ʮ��������
		val = (float)DEF_GSENSOR_DETECT_RANGE*(float)(reg>>4)/(float)(1<<(12-1));		
		
		return  1*val;
	}
}

#if 1
// ��ȡ���ٶȴ�����X��ĵ�ǰ���ٶ�ֵ(��λ: g)��
float gsr_read_valx(void)
{
	float xvalue = 0.0;
	unsigned char	msb;
	unsigned char	lsb;

	// if(I2C_RegRead(REG_STATUS) & 0x01)
	{
		msb = I2C_RegRead(REG_OUT_X_MSB);
		lsb = I2C_RegRead(REG_OUT_X_LSB);
		
		xvalue = reg2val(msb, lsb);
	}

	return xvalue;
}

// ��ȡ���ٶȴ�����Y��ĵ�ǰ���ٶ�ֵ(��λ: g)��
float gsr_read_valy(void)
{
	float yvalue = 0.0;
	unsigned char	msb;
	unsigned char	lsb;

	// if(I2C_RegRead(REG_STATUS) & 0x02)
	{
		msb = I2C_RegRead(REG_OUT_Y_MSB);
		lsb = I2C_RegRead(REG_OUT_Y_LSB);
		
		yvalue = reg2val(msb, lsb);
	}

	return yvalue;
}

// ��ȡ���ٶȴ�����Z��ĵ�ǰ���ٶ�ֵ(��λ: g)��
float gsr_read_valz(void)
{
	float zvalue = 0.0;
	unsigned char	msb;
	unsigned char	lsb;

	// if(I2C_RegRead(REG_STATUS) & 0x04)
	{
		msb = I2C_RegRead(REG_OUT_Z_MSB);
		lsb = I2C_RegRead(REG_OUT_Z_LSB);
		
		zvalue = reg2val(msb, lsb);
	}
	
	return zvalue;
}
#endif

// ��ȡָ�����ϵļĴ�����ֵ(�����з��Ŷ�������ֵ)��
short gsr_read_reg(unsigned char axis)
{
	unsigned char	msb;
	unsigned char	lsb;

	unsigned short	reg;

	msb = I2C_RegRead(REG_OUT_X_MSB+axis*2);
	lsb   = I2C_RegRead(REG_OUT_X_MSB+axis*2+1);

	// ��MSB��LSBƴ�ӳ�16λ�޷�����
	reg = (unsigned short)((msb<<8)|lsb);

	// ����ƴ�Ӻ�Ĵ�����ֵ���λ�Ƿ�Ϊ1���жϼ��ٶ���ֵ�Ƿ�Ϊ�����ٸ������̼���Ĵ�����ֵ��Ӧ�ļ��ٶ���ֵ
	if(reg & 0x8000)
	{
		// �������Ʋ����ʾ�ļĴ�����ֵתΪ��ʮ��������
		reg = reg-(1<<4);	// ��1(ע��16λ�����е�ĩβ4λΪ��Чλ)
		reg = ~reg;			// ȡ��

		// val = (float)DEF_GSENSOR_DETECT_RANGE*(float)(reg>>4)/(float)(1<<(12-1));		
		
		return (short)(-1*(reg>>4));
	}
	else
	{
		// �������Ʋ����ʾ�ļĴ�����ֵתΪ��ʮ��������
		// val = (float)DEF_GSENSOR_DETECT_RANGE*(float)(reg>>4)/(float)(1<<(12-1));		
		
		return  (short)(reg>>4);
	}
}

