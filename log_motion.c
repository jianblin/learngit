/*运动检测管理*/
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


// 终端当前的运动状态
volatile STATUS	sts_motion;	

int				swtimer_still_lasting;		// 检测设备持续静止的定时器(持续静止事件达到上限则触发持续静止事件)。
BOOL			is_still_lasting;	

// 侧翻检测相关全局变量
unsigned int 		cnt_rollint;
unsigned int		swtimer_rollint;

unsigned char	sw_detect_still;	
unsigned char	sw_detect_motion; 
int 				swtimer_still_detection;

unsigned int		err_detect_motion;			// 运动检测的数值门限

BOOL			rollint_filtered;		// 系统初始化后翻滚检测中断是否被过滤掉

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

	// 初始时，终端的运动状态未知，因此需启动静止检测定时器以检测可能的静止状态
	sw_detect_still = OFF;

	sw_detect_motion = OFF;	
	
	// 初始时，运动状态未知
	sts_motion = STS_MOTION_UNKNOWN;

	// 打开静止检测
	detect_still(ON);

	// 打开运动检测
	detect_motion(ON);
}

#if 0

// 打开/关闭data ready检测。
void sw_detect_dataready(SWITCH sw)
{
	u8 n;
	
	// 设置参数之前，先将传感器置为待机模式
	MMA845x_Standby();
	
	if(sw == OFF)
	{		
		// 禁止静止事件中断
		n = I2C_RegRead(REG_CTRL_REG4);
		I2C_RegWrite(CTRL_REG4, n & ~INT_EN_DRDY_MASK);	
	}
	else
	{
		// 设置在INT1上产生中断
		n = I2C_RegRead(CTRL_REG5);		
		I2C_RegWrite(CTRL_REG5, n | INT_CFG_DRDY_MASK);			

		// 使能静止事件中断
		n = I2C_RegRead(REG_CTRL_REG4);
		I2C_RegWrite(REG_CTRL_REG4, n | INT_EN_DRDY_MASK);	
	}

	// 设置完后重新置为活动模式，令设置生效
	MMA845x_Active ();	
}

// 打开/关闭静止检测。
void sw_detect_aslp(SWITCH sw, float ms, unsigned char ints)
{
	u8 n;

	unsigned char cnt = (unsigned char)(ms/320);

	// 设置参数之前，先将传感器置为待机模式
	MMA845x_Standby();	

	if(sw == OFF)
	{
		// 关闭XYZ三轴上的静止事件检测
		n = I2C_RegRead(REG_CTRL_REG2);
		I2C_RegWrite(REG_CTRL_REG2, n & ~0x04);  
		
		// 禁止静止事件中断
		n = I2C_RegRead(REG_CTRL_REG4);
		I2C_RegWrite(CTRL_REG4, n & ~INT_EN_ASLP_MASK);	
	}
	else
	{
		// 设置静止检测窗口长度为3.2秒，320 ms/step
		I2C_RegWrite(REG_ASLP_COUNT, cnt);		

		// 使能震动、位移可唤醒	
		n = I2C_RegRead(REG_CTRL_REG3);
		I2C_RegWrite(REG_CTRL_REG3, n | (ints << 3));		// active low, push-pull

		// 设置在INT1上产生中断
		n = I2C_RegRead(CTRL_REG5);		
		I2C_RegWrite(CTRL_REG5, n | INT_CFG_ASLP_MASK);			

		// 打开静止事件检测
		n = I2C_RegRead(REG_CTRL_REG2);
		I2C_RegWrite(REG_CTRL_REG2, n | 0x04);  

		// 使能静止事件中断
		n = I2C_RegRead(REG_CTRL_REG4);
		I2C_RegWrite(REG_CTRL_REG4, n | INT_EN_ASLP_MASK);	

		// 数据寄存器中的数据过滤关闭
		n = I2C_RegRead(REG_XYZ_DATA_CFG);
		// I2C_RegWrite(REG_XYZ_DATA_CFG, n | 0x10);	// HPF on
	}

	// 设置完后重新置为活动模式，令设置生效
	MMA845x_Active ();	
}

// 打开/关闭方向变化检测。
/*
1，加速度传感器任何时候都会处于特定的方向，因此一旦打开方向变化检测并使能对应中断，则会立即产生方向
   变化中断，此中断可用于判断加速度传感器的初始状态;
2，方向变化检测仅针对X轴和Y轴(便于检测横屏和竖屏)，夹角超过45度(留有+-15度的裕量)即认为方向产生了变化;
*/
void sw_detect_lndprt(unsigned char sw, float ms)
{
	u8 n;

	unsigned char cnt = (unsigned char)(ms/10);

	// 设置参数之前，先将传感器置为待机模式
	MMA845x_Standby();	

	if(sw == OFF)
	{
		// 关闭方向变化检测
		n = I2C_RegRead(REG_PL_CFG);		
		I2C_RegWrite(REG_PL_CFG, 0x00);

		// 禁止方向变化事件中断
		n = I2C_RegRead(REG_CTRL_REG4);		
		I2C_RegWrite(REG_CTRL_REG4, n & ~INT_EN_LNDPRT_MASK);
	}
	else
	{
		// 使能倾斜检测、去抖计数器使用递减模式(检测期间遇到不合规事件递减去抖计数器)
		I2C_RegWrite(REG_PL_CFG, 0x40); 

		// 设置去抖窗口长度
		I2C_RegWrite(REG_PL_COUNT, cnt); 
		
		// 0x13、0x14寄存器只有在MMA8451Q能修改，全系列芯片均可读

		// 使能方向变化中断
		n = I2C_RegRead(REG_CTRL_REG4);
		I2C_RegWrite(REG_CTRL_REG4, n | INT_EN_LNDPRT_MASK);

		// 设置在INT1上产生中断
		n = I2C_RegRead(REG_CTRL_REG5);		
		I2C_RegWrite(REG_CTRL_REG5, n | INT_CFG_LNDPRT_MASK);

		// 数据寄存器中的数据过滤关闭
		n = I2C_RegRead(REG_XYZ_DATA_CFG);
		// I2C_RegWrite(REG_XYZ_DATA_CFG, n & ~0x10);	// HPF off
	}

	// 设置完后重新置为活动模式，令设置生效
	MMA845x_Active ();	
}

// 打开/关闭脉冲检测。
void sw_detect_pulse(unsigned char taps, SWITCH sw, float ms_lmt, float ms_lat, float ms_win)
{
	u8 n;

	unsigned char cnt_lmt = (unsigned char)(ms_lmt/10);
	unsigned char cnt_lat = (unsigned char)(ms_lat/5);
	unsigned char cnt_win = (unsigned char)(ms_win/5);

	// 设置参数之前，先将传感器置为待机模式
	MMA845x_Standby();	

	if(sw == OFF)
	{
		// 关闭单击事件检测
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
		
		// 禁止单击事件中断
		n = I2C_RegRead(CTRL_REG4);
		I2C_RegWrite(CTRL_REG4, n & ~INT_EN_PULSE_MASK); 
	}
	else
	{	
		// 设置三轴上的单击事件检测加速度下限值，0.0625 g/count @8g
		I2C_RegWrite(REG_PULSE_THSX, DEF_DETECT_SINGLETAP_THS);  		// 0.625 g/count
		I2C_RegWrite(REG_PULSE_THSY, DEF_DETECT_SINGLETAP_THS);  		// 0.625 g/count
		I2C_RegWrite(REG_PULSE_THSZ, DEF_DETECT_SINGLETAP_THS+16);  	// 0.625 g/count

		// 脉冲检测时使用高通滤波器但不使用低通滤波器
		n = I2C_RegRead(REG_HP_FILTER_CUTOFF);	
		I2C_RegWrite(REG_HP_FILTER_CUTOFF, n & ~0x03); 		

		// 设置单击脉冲的最大持续时长为50ms，10 ms/count @100Hz, normal power scheme
		I2C_RegWrite(REG_PULSE_TMLT, cnt_lmt); 	

		// 选择检测单击事件、使能单击事件标志锁存到PULSE_SRC寄存器、使能XYZ轴上的单击事件检测
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

		// 设置在INT1上产生中断
		n = I2C_RegRead(0x2E);		
		I2C_RegWrite(0x2E, n | INT_CFG_PULSE_MASK);

		// 使能单击事件中断		
		n = I2C_RegRead(CTRL_REG4);
		I2C_RegWrite(CTRL_REG4, n | INT_EN_PULSE_MASK); 

		// 数据寄存器中的数据过滤关闭
		n = I2C_RegRead(REG_XYZ_DATA_CFG);
		// I2C_RegWrite(REG_XYZ_DATA_CFG, n & ~0x10);	// HPF off
	}

	MMA845x_Active();
}

// 打开/关闭自由落体/运动检测。
void sw_detect_ffmt(unsigned char ffmt, unsigned char axis, float g, float ms)
{
	u8 n;

	// 将加速度门限数值转化为相应寄存器数值、将检测窗口长度数值转化为相应寄存器数值
	unsigned char	ths = (unsigned char)(g/0.0625);
	unsigned char	cnt = (unsigned char)(ms/10);

	// // printf("ths = %d, cnt = %d for ffmt setting\r\n", ths, cnt);
	
	// 设置参数之前，先将传感器置为待机模式
	MMA845x_Standby();	

	if(!(axis&0x07))
	{
		// 关闭自由落体/运动事件检测
		n = I2C_RegRead(REG_FF_MT_CFG);
		I2C_RegWrite(REG_FF_MT_CFG, n & ~(0x07<<3));
		
		// 禁止自由落体/运动事件中断
		n = I2C_RegRead(REG_CTRL_REG4);
		I2C_RegWrite(REG_CTRL_REG4, n & ~INT_EN_FF_MT_1_MASK);
	}
	else
	{
		// 选择检测自由落体/运动事件(而非运动事件)、使能自由落体/运动事件标志锁存到FF_MT_SRC寄存器
		I2C_RegWrite(REG_FF_MT_CFG, 0x00 | (axis<<3) | (ffmt<<6));		// ELE disabled， OAE=0 for freefall detection

		// 设置自由落体/运动检测的上限值为0.375g，0.0625 g/count @8g
		// 注: PCBA有JLink和串口连线时的自由落体/运动检测上限应不小于0.25g(计数值不小于4)。
		I2C_RegWrite(REG_FF_MT_THS, ths);	

		// 设置自由落体/运动检测窗口长度为40ms，10 ms/count @100 Hz
		// 注: 自由落体/运动检测窗口长度应根据终端自由落体/运动的可能高度设置，如理想状况下，自由落体/运动120ms的下落高度为7cm。
		I2C_RegWrite(REG_FF_MT_COUNT, cnt);			

		// 使能自由落体/运动事件中断
		n = I2C_RegRead(REG_CTRL_REG4);
		I2C_RegWrite(REG_CTRL_REG4, n | INT_EN_FF_MT_1_MASK);		

		// 设置在INT2上产生中断
		n = I2C_RegRead(REG_CTRL_REG5);	
		
		I2C_RegWrite(REG_CTRL_REG5, n & ~INT_CFG_FF_MT_1_MASK);

		// 数据寄存器中的数据过滤关闭
		n = I2C_RegRead(REG_XYZ_DATA_CFG);
		// I2C_RegWrite(REG_XYZ_DATA_CFG, n & ~0x10);	// HPF off
	}

	// 设置完后重新置为活动模式，令设置生效
	MMA845x_Active ();	
}

// 打开/关闭瞬态检测。
void sw_detect_trans(SWITCH hpf_sw, unsigned char axis, float g, float ms)
{
	u8 n;

	// 将加速度门限数值转化为相应寄存器数值、将检测窗口长度数值转化为相应寄存器数值
	unsigned char	ths = (unsigned char)(g/0.0625);
	unsigned char	cnt = (unsigned char)(ms/10);

	// // printf("ths = %d, cnt = %d\r\n", ths, cnt);

	// // printf("ths = %d, cnt = %d for trans setting\r\n", ths, cnt);

	// 设置参数之前，先将传感器置为待机模式
	MMA845x_Standby();	

	if(!(axis&0x07))
	{
		// 关闭XYZ三轴上的震动检测
		n = I2C_RegRead(REG_TRANSIENT_CFG);
		I2C_RegWrite(REG_TRANSIENT_CFG, n & ~(0x07<<1));
		
		// 禁止震动事件中断
		n = I2C_RegRead(CTRL_REG4);
		I2C_RegWrite(CTRL_REG4, n & ~INT_EN_TRANS_MASK);
	}
	else
	{
		// 使能瞬态事件标志锁存到TRANSIENT_SRC寄存器、输入到瞬态检测通道的数据使用高通滤波器滤波
		if(hpf_sw == ON)
		{
			// 用于瞬态检测的数据过滤打开(Transient event flags are NOT latched into the TRANSIENT_SRC register.)
			I2C_RegWrite(REG_TRANSIENT_CFG, 0x00 | (axis<<1) & ~0x01);		// ELE disabled.

			// 数据寄存器中的数据过滤也打开
			n = I2C_RegRead(REG_XYZ_DATA_CFG);
			I2C_RegWrite(REG_XYZ_DATA_CFG, n | 0x10);	// HPF on
		}
		else if(hpf_sw == OFF)
		{	
			// 用于瞬态检测的数据过滤打开(Transient event flags are NOT latched into the TRANSIENT_SRC register.)
			I2C_RegWrite(REG_TRANSIENT_CFG, 0x00 | (axis<<1) |  0x01);		// ELE disabled.
		}				

		// 设置震动检测的下限值为~0.4g，0.0625 g/count @100Hz
		// 注: 若震动检测下限值设为~0.2g，则加速度传感器在软重启后会产生误检测(通过甩动PCBA测试)。
		I2C_RegWrite(REG_TRANSIENT_THS, ths);			

		// 设置震动检测窗口长度为100ms，10 ms/count @100Hz
		I2C_RegWrite(REG_TRANSIENT_COUNT, cnt);

		// 设置高通滤波器的截止频率为8Hz, @400Hz fs
		// 为了确保快速震动后再大角度倾斜静止摆放时不会持续产生振动检测中断，
		// 高通滤波器的截止频率不得低于8Hz。
		n = I2C_RegRead(REG_HP_FILTER_CUTOFF);
		I2C_RegWrite(REG_HP_FILTER_CUTOFF, (n & ~0x03) | 0x01);

		// 使能震动事件中断
		n = I2C_RegRead(CTRL_REG4);
		I2C_RegWrite(CTRL_REG4, n | INT_EN_TRANS_MASK);

		// 设置在INT2上产生中断
		n = I2C_RegRead(CTRL_REG5);
		I2C_RegWrite(CTRL_REG5, n & ~INT_CFG_TRANS_MASK);
	}
	
	MMA845x_Active ();	// 设置完后重新置为活动模式，令设置生效
}

#endif

// 打开/关闭静止事件检测。
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

// 打开/关闭运动事件检测(使用FF+TRANS夹闭检测，每次打开夹闭检测时都会以当前加速度值为基准做上下越界检测)。
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

// 打开/关闭震动事件检测。
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

// 打开/关闭翻滚事件检测。
// 注: (夹闭)运动检测打开时，即便静止斜放也会持续产生翻滚检测中断，因此翻滚检测不能和(夹闭)运动检测同时打开。
// 
// 汽车的翻滚有三种常见情况:1，车辆侧翻后即刻复位;2，车辆侧翻90度后停止翻滚;3，车辆侧翻90度后继续翻滚;
// 在第1和第3种情况下会在短时间内产生两次方向变化中断，不过中断产生时Z轴的加速度数值截然不同(情况1下Z轴加速度接近-1g、
// 情况3下Z轴加速度接近+0.50g)，第2种情况下Z轴加速度接近0g。
void detect_roll(SWITCH sw)
{
	if(sw == ON)
	{
		// printf("turned on detection on roll.\r\n");
		
		sw_detect_lndprt(ON, DEF_DETECT_ROLL_CNT);

		// 打开方向变化检测中断后，在方向变化检测中顿处理函数中会开始执行相应的检测算法
	}
	else
	{
		// printf("turned off detection on roll.\r\n");
		
		sw_detect_lndprt(OFF, 0);
	}
}

// 打开/关闭单击事件检测。
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

// 打开/关闭双击事件检测。
// MMA8452Q会将一次双击动作识别成1次单击+1次双击动作，偶尔还会识别成1个单击+1个双击+1个单击动作。
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

/********************************** 各种检测事件产生时的中断处理函数 *****************************************/
// 静止事件检测到后的处理函数。
void event_handler_still(void)
{			
	// 更新为确定静止状态
	sts_motion = STS_MOTION_STILL;

	// 检测到静止事件后设置持续静止检测定时器(仅在设备尚未处于持续静止时才检测持续静止事件，
	// 若已经处于持续静止状态，则不应继再检测持续静止，以免重复)。
	swtimer_still_lasting = CYCLE_STILL_LASTING*1000/SYSTICK_PERIOD;

	// 熄灭蓝灯以指示检测到静止事件。
	// led_switch(GPIO_LED_R, OFF);

	// 关闭运动检测时的打印语句，以免高灵敏度检测模式下频繁检测到的运动事件打印语句太多而占用中断时间。
#if 0
	printf("----------------\r\n");
	printf("still detected. \r\n");
	printf("----------------\r\n");
#endif

	// 更改定位周期
	cycle_periodic_pos = DEF_CYCLE_PERIODIC_POS_WHEN_STILL;
}

// 运动事件检测到后的处理函数。
void event_handler_motion(void)
{
	sts_motion = STS_MOTION_MOTIONING;

	// 检测到运动后关闭持续静止检测定时器
	swtimer_still_lasting = 0;
	is_still_lasting = FALSE;

	// 点亮蓝灯以指示检测到运动事件。
	// led_switch(GPIO_LED_R, ON);

	// 关闭运动检测时的打印语句，以免高灵敏度检测模式下频繁检测到的运动事件打印语句太多而占用中断时间。
#if 0
	printf("----------------\r\n");
	printf("motion detected.\r\n");
	printf("----------------\r\n");
#endif

	// 更改定位周期
	cycle_periodic_pos = DEF_CYCLE_PERIDOIC_POS_WHEN_MOTIONING;
}

#if 0
// 翻滚事件检测中断处理函数。
void event_handler_roll(void)
{		
	// 每个连续的方向变化检测过程中，每产生一次方向变化检测中断，相应计数器递增
	cnt_rollint++;

	// // printf("\r\ncnt_rollint = %d\r\n", cnt_rollint);

	// 打开方向变化检测后，处于静止状态的传感器会自动产生一个方向变化中断，此中断应略过
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
	
	// 检查方向变化连续检测定时器是否为复位状态，是的话则启动之，以开始一次新的连续方向变化检测过程。
	// 此定时器的长度应设置恰当: 过短可能没法检测客观上连续产生的方向变化中断，过长会导致无谓的时间浪费。
	if(swtimer_rollint == 0)
	{
		swtimer_rollint = DEF_ROLLINT_INTERVAL/SYSTICK_PERIOD;
	}	

	// 翻滚中断检测软件定时器溢出后，会在软件定时器溢出中断处理函数内根据
	// 翻滚中断个数判断具体的翻滚事件类型并作相应处理。
}

// 单击事件检测中断处理函数。
void event_handler_singletap(void)
{

}

// 双击事件检测中断处理函数。
void event_handler_doubletap(void)
{

}

#endif

// 根据指定的量程，将指定轴寄存器的MSB字节和LSB字节数据转化为加速度数值。
// 注: 2g、4g、8g量程下均已测试此函数工作正常。
float reg2val(unsigned char msb, unsigned char lsb)
{
	unsigned short 	reg;
	float			val;

	// 将MSB和LSB拼接成16位无符号数
	reg = (unsigned short)((msb<<8)|lsb);

	// 根据拼接后寄存器数值最高位是否为1，判断加速度数值是否为负，再根据量程计算寄存器数值对应的加速度数值
	if(reg & 0x8000)
	{
		// 将二进制补码表示的寄存器数值转为化十进制整数
		reg = reg-(1<<4);	// 减1(注意16位数据中的末尾4位为无效位)
		reg = ~reg;			// 取反

		val = (float)DEF_GSENSOR_DETECT_RANGE*(float)(reg>>4)/(float)(1<<(12-1));		
		
		return -1*val;
	}
	else
	{
		// 将二进制补码表示的寄存器数值转为化十进制整数
		val = (float)DEF_GSENSOR_DETECT_RANGE*(float)(reg>>4)/(float)(1<<(12-1));		
		
		return  1*val;
	}
}

#if 1
// 读取加速度传感器X轴的当前加速度值(单位: g)。
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

// 读取加速度传感器Y轴的当前加速度值(单位: g)。
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

// 读取加速度传感器Z轴的当前加速度值(单位: g)。
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

// 读取指定轴上的寄存器数值(返回有符号短整型数值)。
short gsr_read_reg(unsigned char axis)
{
	unsigned char	msb;
	unsigned char	lsb;

	unsigned short	reg;

	msb = I2C_RegRead(REG_OUT_X_MSB+axis*2);
	lsb   = I2C_RegRead(REG_OUT_X_MSB+axis*2+1);

	// 将MSB和LSB拼接成16位无符号数
	reg = (unsigned short)((msb<<8)|lsb);

	// 根据拼接后寄存器数值最高位是否为1，判断加速度数值是否为负，再根据量程计算寄存器数值对应的加速度数值
	if(reg & 0x8000)
	{
		// 将二进制补码表示的寄存器数值转为化十进制整数
		reg = reg-(1<<4);	// 减1(注意16位数据中的末尾4位为无效位)
		reg = ~reg;			// 取反

		// val = (float)DEF_GSENSOR_DETECT_RANGE*(float)(reg>>4)/(float)(1<<(12-1));		
		
		return (short)(-1*(reg>>4));
	}
	else
	{
		// 将二进制补码表示的寄存器数值转为化十进制整数
		// val = (float)DEF_GSENSOR_DETECT_RANGE*(float)(reg>>4)/(float)(1<<(12-1));		
		
		return  (short)(reg>>4);
	}
}

