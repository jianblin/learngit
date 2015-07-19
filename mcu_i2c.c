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
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_OD;	// ��©���
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	// ������GPIO�󣬹���IIC���ߣ����䴦�ڿ���״̬
	SDA_H();
	SCL_H();
}

void I2C_delay(void)
{	
//   u16 i=5; //��������Ż��ٶȣ�����Ƶ�뽵Ƶ���ԣ�����Ϊ�պ���ʱ������д��
//   while(i) 
//   { 
//     i--; 
//   } 
}

//��ʼ�ź� SCL�ڸߵ�ƽ�ڼ䣬SDAһ���½������ʾ�����źţ�������Ҳ���������ظ���ʼ����
//��ǰֻ��һ��IIC�豸�������������豸ռ�õ���������Բ���������æ��������֣����Կ���û�з���ֵ
void I2C_Start(void)
{
	SDA_H();//�ͷ�SDA����
	SCL_H();
	I2C_delay();
//	if(!SDA_read())return FALSE;	//SDA��Ϊ�͵�ƽ������æ,�˳�
	SDA_L();
	I2C_delay();
//	if(SDA_read()) return FALSE;	//SDA��Ϊ�ߵ�ƽ�����߳���,�˳�
// 	return TRUE;
}

void I2C_Stop(void)//ֹͣ SCL�ڸߵ�ƽ�ڼ䣬SDAһ�����������ʾֹͣ�ź�
{
	SDA_L();
	I2C_delay();
	SCL_H();
	I2C_delay();
	SDA_H();
	I2C_delay();
}

void I2C_Ack(void)//Ӧ�� SCL�ڸߵ�ƽ�ڼ䣬SDA�����豸��Ϊ�͵�ƽ��ʾӦ��
{	
	unsigned char i = 0;
	
    SCL_H();  
    I2C_delay();  
    while((SDA_read()==1)&&(i<250))i++;  
    SCL_L();  
    I2C_delay(); 
}

/******************************************************************************
������I2C_PutAck()
���ܣ���������Ӧ��λ��Ӧ����Ӧ�𣩣�����֪ͨ�ӻ��������Ƿ�ɹ����մӻ�����
������Ack = 0������Ӧ��
      Ack = 1��������Ӧ��
˵�����������յ�ÿһ���ӻ��ֽں�Ҫ��ӻ�����Ӧ�����յ����һ���ֽ�ʱ��Ӧ��������Ӧ��֪ͨ�ӻ���Ҫ��Ӧ��
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

void I2C_SendByte(unsigned char SendByte) //���ݴӸ�λ����λ//
{
	unsigned char i=8;
	while(i--)
	{
		SCL_L();//����SCL����Ϊֻ����ʱ���ź�Ϊ�͵�ƽ�ڼ䣬�������ϵĸߵ͵�ƽ״̬������仯;���ڴ�ʱ����һ��ѭ����SDA_Hһ���γ�һ��������
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
	SCL_L();//����SCL��Ϊ�´����ݴ�������׼��   
	I2C_delay();	  
	SDA_H();//�ͷ�SDA���ߣ��������ɴ��豸���ƣ�������豸���������ݺ���SCLΪ��ʱ������SDA��ΪӦ���ź�   
	I2C_delay();
}

unsigned char I2C_ReceiveByte(void)  //���ݴӸ�λ����λ
{ 
	unsigned char i,k = 0;  
	SCL_L();  
	I2C_delay(); 
	SDA_H();  
	I2C_delay();  
    for(i=0;i<8;i++)  
    {  
		SCL_H();//������ʱ��IIC�豸�����ݷ���sda���ϣ����ڸߵ�ƽ�ڼ������Ѿ��ȶ������Խ�����   
		I2C_delay();      
		k=(k<<1)|SDA_read();
		SCL_L();//����SCL��ʹ���Ͷ˿��԰����ݷ���SDA��   
		I2C_delay();      
    }  

    return k; 
}
void I2C_RegWrite(unsigned char byte_add,unsigned char wdata)//�������ܣ�����ַд��һ�ֽ�����
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

unsigned char I2C_RegRead(unsigned char byte_add)	//�������ܣ�����ַ����һ�ֽ�����
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
	
	I2C_PutAck(FALSE);//���ս��������ͷ�Ӧ���ź�
	I2C_Stop();
	
	return x;
}
