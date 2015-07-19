#ifndef __MCU_I2C__
#define __MCU_I2C__

#include "stm32f10x.h"


#define SCL_H()         (GPIOB->BSRR = GPIO_Pin_15)
#define SCL_L()         (GPIOB->BRR  = GPIO_Pin_15)
   
#define SDA_H()         (GPIOB->BSRR = GPIO_Pin_14)
#define SDA_L()         (GPIOB->BRR  = GPIO_Pin_14)

// ��ȡGPIO�ڵ�ƽֵӦʹ��STM32�⺯��������Ӧֱ�Ӷ�ȡ�Ĵ���(ĳЩIO�ڿ���ͨ��ֱ�Ӷ�ȡ�Ĵ�����ȷ��������Щ����)
#define SDA_read()      GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)	// (GPIOB->IDR  & GPIO_Pin_14)

//�豸��ַΪ��0x1C
#define IIC_ADDR_READ  	0x39  	//�����ָ��
#define IIC_ADDR_WRITE 	0x38    //����дָ��

void I2C_Configure(void);

void I2C_RegWrite(unsigned char byte_add,unsigned char wdata);	//�������ܣ�����ַд��һ�ֽ�����
unsigned char   I2C_RegRead(unsigned char byte_add);	//�������ܣ�����ַ����һ�ֽ�����

#endif 

