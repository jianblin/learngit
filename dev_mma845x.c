#include "dev_mma845x.h"
#include "mcu_i2c.h"
#include "common.h"
#include "log_motion.h"
#include "log_time.h"
#include "stdio.h"

#define FULL_SCALE_8G         2
#define FULL_SCALE_4G         1
#define FULL_SCALE_2G         0


/*
1，相同的运动检测窗口情况下，400Hz采样率时对运动检测的灵敏度明显高于200Hz采样时对运动件的灵敏度，
但是Normal过采样模式下前者的工号为165mA、后者功耗为85mA，接近2倍的关系；
2，影响运动检测灵敏度的因素除了采样率外还有运动检测窗口的长度的，但是前者的影响更为重要；
*/

/*********************************************************
* Put MMA845xQ into Active Mode
**********************************************************/
void MMA845x_Active ()
{ 
	//注意，从休眠模式到活动模式，要等一会儿（毫秒级，应该为一个ODR周期）才能响应，如果此时候有动状态要检测，则被忽略
	//不过一般情况下不会一启动马上用
	I2C_RegWrite(REG_CTRL_REG1, 0x09 | 0x02 & ~0xC0);	// 休眠状态下采样率50Hz，唤醒状态下采样率400Hz, fast read
}

/*********************************************************
* Put MMA845xQ into Standby Mode
**********************************************************/
void MMA845x_Standby (void)
{
  	I2C_RegWrite(REG_CTRL_REG1, 0x08 | 0x02 & ~0xC0);	// 休眠状态下采样率50Hz，唤醒状态下采样率400Hz, fast read
}

// 重启加速度传感器。
void MMA845x_reset(void)
{
	MMA845x_Standby();

	// 重启加速度传感器
	I2C_RegWrite(REG_CTRL_REG2, 0x40);	

  	MMA845x_Active ();
}

/*********************************************************
* Initialize MMA845xQ
**********************************************************/
void MMA845x_Init (void)
{		
	// printf("\r\nto initialize G-Sensor...\r\n");
	
	// 使用加速度传感器之前应将其软重启，以便内部寄存器设置重置，从而避免残余设置引起的误检测或误数据
	MMA845x_reset();

	delay_100ms(5);

	// 设置加速度传感器运行的全局参数
  	MMA845x_Standby();

	// 设置唤醒状态下的功耗模式为normal(44uA)、休眠状态下的功耗模式为normal(24uA) 、禁止静止事件检测
	I2C_RegWrite(REG_CTRL_REG2, 0x00);	

	// 选择检测量程为2g
  	I2C_RegWrite(REG_XYZ_DATA_CFG, FULL_SCALE_2G);	
	// printf("measurement range = 2g.\r\n");	

	I2C_RegWrite(REG_CTRL_REG3, 0x00);		// active low, push-pull

	// 禁止所有检测中断
	I2C_RegWrite(REG_CTRL_REG4, 0x00);	
	
	MMA845x_Active ();
	
	// 延时500ms，等待传感器内部电路稳定
	delay_100ms(5);  	

	printf("who_am_i register = %x\r\n", I2C_RegRead(REG_WHO_AM_I));
}

#ifdef TEST_GSR_ACQUIRE_DATA

#define DATA_ACAUIRING_ACTIVE()		I2C_RegWrite(REG_CTRL_REG1, 0x03 | 0x18)	// 100Hz, 50Hz, active, fast read
#define DATA_ACAUIRING_STANDBY()		I2C_RegWrite(REG_CTRL_REG1, 0x02 | 0x18)	// 100Hz, 50Hz, active, fast read

float				gsensor_data[GSENSOR_DATA_LENGTH][3];
unsigned int		gsensor_data_wr = 0;
unsigned int		gsensor_data_rd = 0;

// 采集加速度传感器数值。
void MMA845x_Acquiye_Data(void)
{
	DATA_ACAUIRING_STANDBY();
		
	I2C_RegWrite(REG_CTRL_REG2, 0x00);		// auto-sleep is not enabled
	I2C_RegWrite(REG_CTRL_REG5, 0x01);		// data ready interrupt is routed to INT1 pin
	I2C_RegWrite(REG_CTRL_REG4, 0x01);		// data ready interrupt enabled

	// I2C_RegWrite(REG_XYZ_DATA_CFG, FULL_SCALE_2G | 0x10);
	I2C_RegWrite(REG_XYZ_DATA_CFG, FULL_SCALE_2G | 0x10);	// HPF on

	I2C_RegWrite(REG_HP_FILTER_CUTOFF, 0x00);				// 16Hz cutoff @400Hz fs
		
	DATA_ACAUIRING_ACTIVE();
	
	while(1)
	{
		if(gsensor_data_rd < gsensor_data_wr)
		{
			printf("%1.4f\t%1.4f\t%1.4f\r\n", gsensor_data[gsensor_data_rd&(GSENSOR_DATA_LENGTH-1)][0], gsensor_data[gsensor_data_rd&(GSENSOR_DATA_LENGTH-1)][1], gsensor_data[gsensor_data_rd&(GSENSOR_DATA_LENGTH-1)][2]);

			gsensor_data_rd++;
		}
	}
}

#endif

