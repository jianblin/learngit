#include "mcu_i2c.h"

/********************************************************************************************************************
*
*
*
*
*********************************************************************************************************************/
void I2C_Configure(void)
{
  	GPIO_InitTypeDef  GPIO_InitStructure;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	 
	//   								  G_SDA         G_SCL
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_OD;	// 开漏输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	// 配置完GPIO后，挂起IIC总线，令其处于空闲状态
	SDA_H();
	SCL_H();
}

void I2C_delay(void)
{	
//   u16 i=5; //这里可以优化速度，经超频与降频测试，保持为空函数时还可以写入
//   while(i) 
//   { 
//     i--; 
//   } 
}

//开始信号 SCL在高电平期间，SDA一个下降沿则表示启动信号，本函数也用来产生重复起始条件
//当前只有一个IIC设备，不会有其它设备占用的情况，所以不会有总线忙的情况出现，所以可以没有返回值
void I2C_Start(void)
{
	SDA_H();//释放SDA总线
	SCL_H();
	I2C_delay();
//	if(!SDA_read())return FALSE;	//SDA线为低电平则总线忙,退出
	SDA_L();
	I2C_delay();
//	if(SDA_read()) return FALSE;	//SDA线为高电平则总线出错,退出
// 	return TRUE;
}

void I2C_Stop(void)//停止 SCL在高电平期间，SDA一个上升沿则表示停止信号
{
	SDA_L();
	I2C_delay();
	SCL_H();
	I2C_delay();
	SDA_H();
	I2C_delay();
}

void I2C_Ack(void)//应答 SCL在高电平期间，SDA被从设备拉为低电平表示应答
{	
	unsigned char i = 0;
	
    SCL_H();  
    I2C_delay();  
    while((SDA_read()==1)&&(i<250))i++;  
    SCL_L();  
    I2C_delay(); 
}

/******************************************************************************
函数：I2C_PutAck()
功能：主机产生应答位（应答或非应答），用于通知从机：主机是否成功接收从机数据
参数：Ack = 0：主机应答
      Ack = 1：主机非应答
说明：主机在收到每一个从机字节后都要求从机产生应答，在收到最后一个字节时，应当产生非应答，通知从机不要再应答
******************************************************************************/
void I2C_PutAck(BOOL Ack)
{
	if(Ack==TRUE)
	{
		SDA_H();
	}
	else
	{
		SCL_L();
	}
  	I2C_delay();
	SCL_H();
	I2C_delay();
	SCL_L();
	I2C_delay();
}

void I2C_SendByte(unsigned char SendByte) //数据从高位到低位//
{
	unsigned char i=8;
	while(i--)
	{
		SCL_L();//拉低SCL，因为只有在时钟信号为低电平期间，数据线上的高低电平状态才允许变化;并在此时和上一个循环的SDA_H一起形成一个上升沿
		I2C_delay();
		if(SendByte&0x80)
		{
			SDA_H(); 
		}       
		else 
		{
			SDA_L();
		}    
		SendByte<<=1;
		I2C_delay();
		SCL_H();
		I2C_delay();	
	}
	SCL_L();//拉低SCL，为下次数据传输做好准备   
	I2C_delay();	  
	SDA_H();//释放SDA总线，接下来由从设备控制，比如从设备接收完数据后，在SCL为高时，拉低SDA作为应答信号   
	I2C_delay();
}

unsigned char I2C_ReceiveByte(void)  //数据从高位到低位
{ 
	unsigned char i,k = 0;  
	SCL_L();  
	I2C_delay(); 
	SDA_H();  
	I2C_delay();  
    for(i=0;i<8;i++)  
    {  
		SCL_H();//上升沿时，IIC设备将数据放在sda线上，并在高电平期间数据已经稳定，可以接收啦   
		I2C_delay();      
		k=(k<<1)|SDA_read();
		SCL_L();//拉低SCL，使发送端可以把数据放在SDA上   
		I2C_delay();      
    }  

    return k; 
}
void I2C_RegWrite(unsigned char byte_add,unsigned char wdata)//函数功能：按地址写入一字节数据
{
	I2C_Start();
	I2C_SendByte(IIC_ADDR_WRITE);
	I2C_Ack();
	I2C_SendByte(byte_add);
	I2C_Ack();
	I2C_SendByte(wdata);
	I2C_Ack();
	I2C_Stop();
}

unsigned char I2C_RegRead(unsigned char byte_add)	//函数功能：按地址读出一字节数据
{
   	volatile unsigned char x;
	
	I2C_Start();
	I2C_SendByte(IIC_ADDR_WRITE);
	I2C_Ack();
	I2C_SendByte(byte_add);
	I2C_Ack();
	I2C_Start();
	I2C_SendByte(IIC_ADDR_READ);
	I2C_Ack();
	x=I2C_ReceiveByte();
	
	I2C_PutAck(FALSE);//接收结束，发送非应答信号
	I2C_Stop();
	
	return x;
}
