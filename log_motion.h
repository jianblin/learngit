#ifndef __MOTION
#define __MOTION

#include "common.h"
#include "stm32f10x.h"

#define DEF_GSENSOR_DETECT_RANGE	2	// Ĭ������Ϊ2g

// �ն��˶�״̬
#define STS_MOTION_UNKNOWN			0	// δ֪״̬
#define STS_MOTION_STILL				1	// ���Ծ�ֹ
#define STS_MOTION_MOTIONING		2	// ��
/*
#define STS_MOTION_MOTIONING			2	// ��
#define STS_MOTION_MOVING				2	// λ��
#define STS_MOTION_CRASH				3	// ��ײ
#define STS_MOTION_ROLLING				4	// ����
#define STS_MOTION_FREEFALL			5	// ��������
*/

// �����¼�
#define EVENT_ROLL_RECOVERY			3	// �෭�󼴿̸�λ
#define EVENT_ROLL_HALTED			4	// �෭��ֹͣ����
#define EVENT_ROLL_CONTINUE			5	// �෭���������

// MMA8452Q���ٶȴ������Ĵ�������
#define	REG_STATUS				0x00
#define	REG_OUT_X_MSB			0x01
#define	REG_OUT_X_LSB			0x02
#define	REG_OUT_Y_MSB			0x03
#define	REG_OUT_Y_LSB			0x04
#define	REG_OUT_Z_MSB			0x05
#define	REG_OUT_Z_LSB			0x06
#define	REG_SYSMOD				0x0B
#define	REG_INT_SOURCE			0x0C
#define	REG_WHO_AM_I			0x0D
#define	REG_XYZ_DATA_CFG		0x0E
#define	REG_HP_FILTER_CUTOFF		0x0F
#define	REG_PL_STATUS			0x10
#define	REG_PL_CFG				0x11
#define	REG_PL_COUNT			0x12
#define	REG_PL_BF_ZCOMP			0x13
#define	REG_P_L_THS_REG			0x14
#define	REG_FF_MT_CFG			0x15
#define	REG_FF_MT_SRC			0x16
#define	REG_FF_MT_THS			0x17
#define	REG_FF_MT_COUNT			0x18
#define	REG_TRANSIENT_CFG		0x1D
#define	REG_TRANSIENT_SRC		0x1E
#define	REG_TRANSIENT_THS		0x1F
#define	REG_TRANSIENT_COUNT	0x20
#define	REG_PULSE_CFG			0x21
#define	REG_PULSE_SRC			0x22
#define	REG_PULSE_THSX			0x23
#define	REG_PULSE_THSY			0x24
#define	REG_PULSE_THSZ			0x25
#define	REG_PULSE_TMLT			0x26
#define	REG_PULSE_LTCY			0x27
#define	REG_PULSE_WIND			0x28
#define	REG_ASLP_COUNT			0x29
#define	REG_CTRL_REG1			0x2A
#define	REG_CTRL_REG2			0x2B
#define	REG_CTRL_REG3			0x2C
#define	REG_CTRL_REG4			0x2D
#define	REG_CTRL_REG5			0x2E
#define	REG_OFF_X				0x2F
#define	REG_OFF_Y				0x30
#define	REG_OFF_Z				0x31

// ���ּ���Ĭ�����ò���
// ASLP
#define DEF_DETECT_STILL_CNT				(10*320)	// 320 ms/count
#define DEF_WAKEUP_STILL_INTS			0x48		// all disabled

// FF_MT(�˶�����ܸ����𶯼�⣬��֮��Ȼ������˶������������˶����⣬������˶������𶯡�λ�ơ���ײ�����������������)
#define FFMT_MOTION						1						
#define FFMT_FREEFALL					0

#define DEF_DETECT_MOTION_AXIS			0x07		// Y+X (ˮƽ����ʱ��Z����˶���ⲻ�ܴ򿪣���ΪZ���й̶����������ٶ�)
#define DEF_DETECT_MOTION_CNT			(50*10)		// 100ms, 10 ms/count(ʵ��400Hz�����ʶ��ӳ��˶���ⴰ��Ϊ100ms����������ȳ���200Hz������+50ms�˶���ⴰ������)
													// ��ⴰ�ڳ��ȶԼ�������ȵ�Ӱ��ܴ�(�ȼ������ֵ�ߵ͸�����)
													
// #define DEF_DETECT_MOTION_ERR			(1*0.0625)	// ����˶�ʱ���õļ���������¸���ԣ��
// ���˶����Ĳ������޴�64����Ϊ8��������˶����������ȣ��Ӷ������豸�����������������ʻʱ���Ϊ��ֹ(�������¶�λ���̴���)��
#define DEF_DETECT_MOTION_ERR			16			// ����˶�ʱ���õļ���������¸���ԣ��

#define DEF_DETECT_FREEFALL_AXIS			0x04		// Z(����ˮƽ���õĴ�������������Z�������������¼�)
#define DEF_DETECT_FREEFALL_THS			(8*0.0625)	//  4 * 0.0625 g/count = 0.25 g
#define DEF_DETECT_FREEFALL_CNT			(50*10)		// 10 ms/count (���������ⴰ�ڵĳ��Ⱥ���������ĸ߳�������أ��߳�Խ������ⴰ��Խ����
													// ��ⴰ�ڹ��̿�����ɳ��������������������������������������¼�)
// TRANSICENT
#define DEF_DETECT_CRASH_AXIS			0x07		// Z+Y+X axis
#define DEF_DETECT_CRASH_THS				(64*0.0625)	// 0.0625 g/count
#define DEF_DETECT_CRASH_CNT			(5*10)		// 5 count * 10 ms/count = 50 ms

// Shake detection
#define DEF_DETECT_SHAKE_AXIS			0x03		// Y+X axis(Z�����ھ�ֵ̬�ܴ󣬲�����˲̬���)
#define DEF_DETECT_SHAKE_THS				(2*0.0625)	// 0.0625 g/count
#define DEF_DETECT_SHAKE_CNT			(20*10)		// 5 count * 10 ms/count = 50 ms

// LNDPRT
// 
// ˮƽ���õĴ�������X-Z��Y-Zƽ����ת180���Ĺ����л���30�ȡ�120�ȡ�150��������ת�Ƕȴ���������仯�жϣ����
// �������������л�����ת��30�ȸ�������һ������仯�жϣ����������λ������ڻع鵽0��ʱ�����µķ���仯�жϣ�
// ��������������������ת�Ƕ���ת��120�Ⱥ�150�ȴ������µķ���仯�жϣ�����������������������Ҫ�ڶ�ʱ����
// �������2�η����жϣ�ͨ����ȡ����������ļ��ٶ���ֵ�жϵ�ǰ����ת�Ƕȡ�
#define DEF_DETECT_ROLL_CNT				(20*10)		// 10 ms/count 
#define DEF_ROLLINT_INTERVAL				3000		// 3000ms

// PULSE
#define DEF_DETECT_SINGLETAP_THS			(24*0.0625)	// 0.0625 g/count
#define DEF_DETECT_SINGLETAP_CNT_LMT		(6*10)		// 10 ms/count, 100Hz
#define DEF_DETECT_DOUBLETAP_CNT_LAT	(20*5)		//  5 ms/count, 100 Hz and Pulse_LPF_EN = 0
#define DEF_DETECT_DOUBLETAP_CNT_WIN	(60*5)		//  5 ms/count, 100 Hz and Pulse_LPF_EN = 0

#define CYCLE_STILL_LASTING				30			// ���ڶ�λģʽ�¼�������ֹ�����ʱ��(һ����⵽������ֹ���򲻻ᶨλҲ�����ϴ�λ����Ϣ)����λ: �롣

// �ն˵��˶�״̬
extern volatile STATUS	sts_motion;	

extern int			swtimer_still_lasting;
extern BOOL			is_still_lasting;	

extern unsigned int 	cnt_rollint;
extern unsigned int	swtimer_rollint;

extern unsigned char	sw_detect_still;
extern unsigned char	sw_detect_motion;
extern int 			swtimer_still_detection;

extern unsigned int	err_detect_motion;

#define DEF_SWTIMER_STILL_DETECTION		3000		// ��ֹ��ⴰ��Ĭ�ϳ���Ϊ3000ms

#define AXIS_X	0
#define AXIS_Y	1
#define AXIS_Z	2

// ��������
void	motion_init(void);

#if 0
void 	sw_detect_dataready(SWITCH sw);
void 	sw_detect_aslp(SWITCH sw, float ms, unsigned char ints);
void 	sw_detect_lndprt(unsigned char sw, float ms);
void 	sw_detect_pulse(unsigned char taps, SWITCH sw, float ms_lmt, float ms_lat, float ms_win);
void 	sw_detect_ffmt(unsigned char ffmt, unsigned char axis, float g, float ms);
void 	sw_detect_trans(SWITCH hpf_sw, unsigned char axis, float g, float ms);
#endif

void 	detect_still(SWITCH sw);
void 	detect_motion(SWITCH sw);

#if 0
void 	detect_roll(SWITCH sw);
void 	detect_singletap(SWITCH sw);
void 	detect_doubletap(SWITCH sw);
#endif


void 	event_handler_still(void);
void 	event_handler_motion(void);
#if 0
void 	event_handler_roll(void);
void 	event_handler_singletap(void);
void 	event_handler_doubletap(void);
#endif

float 	gsr_read_valx(void);
float 	gsr_read_valy(void);
float 	gsr_read_valz(void);

short 	gsr_read_reg(unsigned char axis);

#endif

