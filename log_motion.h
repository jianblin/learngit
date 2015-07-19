#ifndef __MOTION
#define __MOTION

#include "common.h"
#include "stm32f10x.h"

#define DEF_GSENSOR_DETECT_RANGE	2	// 默认量程为2g

// 终端运动状态
#define STS_MOTION_UNKNOWN			0	// 未知状态
#define STS_MOTION_STILL				1	// 绝对静止
#define STS_MOTION_MOTIONING		2	// 震动
/*
#define STS_MOTION_MOTIONING			2	// 震动
#define STS_MOTION_MOVING				2	// 位移
#define STS_MOTION_CRASH				3	// 碰撞
#define STS_MOTION_ROLLING				4	// 翻滚
#define STS_MOTION_FREEFALL			5	// 自由落体
*/

// 翻滚事件
#define EVENT_ROLL_RECOVERY			3	// 侧翻后即刻复位
#define EVENT_ROLL_HALTED			4	// 侧翻后停止翻滚
#define EVENT_ROLL_CONTINUE			5	// 侧翻后继续翻滚

// MMA8452Q加速度传感器寄存器定义
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

// 各种检测的默认设置参数
// ASLP
#define DEF_DETECT_STILL_CNT				(10*320)	// 320 ms/count
#define DEF_WAKEUP_STILL_INTS			0x48		// all disabled

// FF_MT(运动检测能覆盖震动检测，反之不然，因此运动检测是最宽泛的运动类检测，广义的运动包括震动、位移、碰撞、翻滚、自由落体等)
#define FFMT_MOTION						1						
#define FFMT_FREEFALL					0

#define DEF_DETECT_MOTION_AXIS			0x07		// Y+X (水平放置时，Z轴的运动检测不能打开，因为Z轴有固定的重力加速度)
#define DEF_DETECT_MOTION_CNT			(50*10)		// 100ms, 10 ms/count(实用400Hz采样率而延长运动检测窗口为100ms，检测灵敏度超过200Hz采样率+50ms运动检测窗口设置)
													// 检测窗口长度对检测灵敏度的影响很大(比检测门限值高低更明显)
													
// #define DEF_DETECT_MOTION_ERR			(1*0.0625)	// 检测运动时设置的检测门限上下浮动裕量
// 将运动检测的波动门限从64降低为8，以提高运动检测的灵敏度，从而避免设备在汽车或火车上匀速行驶时检测为静止(进而导致定位流程错误)。
#define DEF_DETECT_MOTION_ERR			16			// 检测运动时设置的检测门限上下浮动裕量

#define DEF_DETECT_FREEFALL_AXIS			0x04		// Z(对于水平放置的传感器，仅需在Z轴检测自由落体事件)
#define DEF_DETECT_FREEFALL_THS			(8*0.0625)	//  4 * 0.0625 g/count = 0.25 g
#define DEF_DETECT_FREEFALL_CNT			(50*10)		// 10 ms/count (自由落体检测窗口的长度和自由落体的高程密切相关，高程越长、检测窗口越长，
													// 检测窗口过短可能造成长搞成自由落体过程中连续监测出国此自由落体事件)
// TRANSICENT
#define DEF_DETECT_CRASH_AXIS			0x07		// Z+Y+X axis
#define DEF_DETECT_CRASH_THS				(64*0.0625)	// 0.0625 g/count
#define DEF_DETECT_CRASH_CNT			(5*10)		// 5 count * 10 ms/count = 50 ms

// Shake detection
#define DEF_DETECT_SHAKE_AXIS			0x03		// Y+X axis(Z轴由于静态值很大，不能做瞬态检测)
#define DEF_DETECT_SHAKE_THS				(2*0.0625)	// 0.0625 g/count
#define DEF_DETECT_SHAKE_CNT			(20*10)		// 5 count * 10 ms/count = 50 ms

// LNDPRT
// 
// 水平放置的传感器在X-Z或Y-Z平面旋转180读的过程中会在30度、120度、150度三个旋转角度处产生方向变化中断，因此
// 汽车翻滚过程中会在旋转到30度附近产生一个方向变化中断，如果翻滚后复位，则会在回归到0度时产生新的方向变化中断，
// 如果继续翻滚，则会在旋转角度旋转到120度和150度处产生新的方向变化中断，因此真正的汽车翻滚检测需要在短时间内
// 检测连续2次翻滚中断，通过读取翻滚后三轴的加速度数值判断当前的旋转角度。
#define DEF_DETECT_ROLL_CNT				(20*10)		// 10 ms/count 
#define DEF_ROLLINT_INTERVAL				3000		// 3000ms

// PULSE
#define DEF_DETECT_SINGLETAP_THS			(24*0.0625)	// 0.0625 g/count
#define DEF_DETECT_SINGLETAP_CNT_LMT		(6*10)		// 10 ms/count, 100Hz
#define DEF_DETECT_DOUBLETAP_CNT_LAT	(20*5)		//  5 ms/count, 100 Hz and Pulse_LPF_EN = 0
#define DEF_DETECT_DOUBLETAP_CNT_WIN	(60*5)		//  5 ms/count, 100 Hz and Pulse_LPF_EN = 0

#define CYCLE_STILL_LASTING				30			// 周期定位模式下检测持续静止的最大时间(一旦检测到持续静止，则不会定位也不会上传位置信息)，单位: 秒。

// 终端的运动状态
extern volatile STATUS	sts_motion;	

extern int			swtimer_still_lasting;
extern BOOL			is_still_lasting;	

extern unsigned int 	cnt_rollint;
extern unsigned int	swtimer_rollint;

extern unsigned char	sw_detect_still;
extern unsigned char	sw_detect_motion;
extern int 			swtimer_still_detection;

extern unsigned int	err_detect_motion;

#define DEF_SWTIMER_STILL_DETECTION		3000		// 静止检测窗口默认长度为3000ms

#define AXIS_X	0
#define AXIS_Y	1
#define AXIS_Z	2

// 函数声明
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

