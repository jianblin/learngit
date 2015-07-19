#ifndef __MCU_I2C__
#define __MCU_I2C__

#include "stm32f10x.h"


#define SCL_H()         (GPIOB->BSRR = GPIO_Pin_15)
#define SCL_L()         (GPIOB->BRR  = GPIO_Pin_15)
   
#define SDA_H()         (GPIOB->BSRR = GPIO_Pin_14)
#define SDA_L()         (GPIOB->BRR  = GPIO_Pin_14)

// 读取GPIO口电平值应使用STM32库函数，而不应直接读取寄存器(某些IO口可以通过直接读取寄存器正确读出，有些不行)
#define SDA_read()      GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)	// (GPIOB->IDR  & GPIO_Pin_14)

//设备地址为：0x1C
#define IIC_ADDR_READ  	0x39  	//定义读指令
#define IIC_ADDR_WRITE 	0x38    //定义写指令

void I2C_Configure(void);

void I2C_RegWrite(unsigned char byte_add,unsigned char wdata);	//函数功能：按地址写入一字节数据
unsigned char   I2C_RegRead(unsigned char byte_add);	//函数功能：按地址读出一字节数据

#endif 

