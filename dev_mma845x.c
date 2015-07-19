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
1����ͬ���˶���ⴰ������£�400Hz������ʱ���˶��������������Ը���200Hz����ʱ���˶����������ȣ�
����Normal������ģʽ��ǰ�ߵĹ���Ϊ165mA�����߹���Ϊ85mA���ӽ�2���Ĺ�ϵ��
2��Ӱ���˶���������ȵ����س��˲������⻹���˶���ⴰ�ڵĳ��ȵģ�����ǰ�ߵ�Ӱ���Ϊ��Ҫ��
*/

/*********************************************************
* Put MMA845xQ into Active Mode
**********************************************************/
void MMA845x_Active ()
{ 
	//ע�⣬������ģʽ���ģʽ��Ҫ��һ��������뼶��Ӧ��Ϊһ��ODR���ڣ�������Ӧ�������ʱ���ж�״̬Ҫ��⣬�򱻺���
	//����һ������²���һ����������
	I2C_RegWrite(REG_CTRL_REG1, 0x09 | 0x02 & ~0xC0);	// ����״̬�²�����50Hz������״̬�²�����400Hz, fast read
}

/*********************************************************
* Put MMA845xQ into Standby Mode
**********************************************************/
void MMA845x_Standby (void)
{
  	I2C_RegWrite(REG_CTRL_REG1, 0x08 | 0x02 & ~0xC0);	// ����״̬�²�����50Hz������״̬�²�����400Hz, fast read
}

// �������ٶȴ�������
void MMA845x_reset(void)
{
	MMA845x_Standby();

	// �������ٶȴ�����
	I2C_RegWrite(REG_CTRL_REG2, 0x40);	

  	MMA845x_Active ();
}

/*********************************************************
* Initialize MMA845xQ
**********************************************************/
void MMA845x_Init (void)
{		
	// printf("\r\nto initialize G-Sensor...\r\n");
	
	// ʹ�ü��ٶȴ�����֮ǰӦ�������������Ա��ڲ��Ĵ����������ã��Ӷ����������������������������
	MMA845x_reset();

	delay_100ms(5);

	// ���ü��ٶȴ��������е�ȫ�ֲ���
  	MMA845x_Standby();

	// ���û���״̬�µĹ���ģʽΪnormal(44uA)������״̬�µĹ���ģʽΪnormal(24uA) ����ֹ��ֹ�¼����
	I2C_RegWrite(REG_CTRL_REG2, 0x00);	

	// ѡ��������Ϊ2g
  	I2C_RegWrite(REG_XYZ_DATA_CFG, FULL_SCALE_2G);	
	// printf("measurement range = 2g.\r\n");	

	I2C_RegWrite(REG_CTRL_REG3, 0x00);		// active low, push-pull

	// ��ֹ���м���ж�
	I2C_RegWrite(REG_CTRL_REG4, 0x00);	
	
	MMA845x_Active ();
	
	// ��ʱ500ms���ȴ��������ڲ���·�ȶ�
	delay_100ms(5);  	

	printf("who_am_i register = %x\r\n", I2C_RegRead(REG_WHO_AM_I));
}

#ifdef TEST_GSR_ACQUIRE_DATA

#define DATA_ACAUIRING_ACTIVE()		I2C_RegWrite(REG_CTRL_REG1, 0x03 | 0x18)	// 100Hz, 50Hz, active, fast read
#define DATA_ACAUIRING_STANDBY()		I2C_RegWrite(REG_CTRL_REG1, 0x02 | 0x18)	// 100Hz, 50Hz, active, fast read

float				gsensor_data[GSENSOR_DATA_LENGTH][3];
unsigned int		gsensor_data_wr = 0;
unsigned int		gsensor_data_rd = 0;

// �ɼ����ٶȴ�������ֵ��
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

