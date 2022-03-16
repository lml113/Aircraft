#include "mpuiic.h"
#include "gpio.h"


//定义I2C PB10->SCL;PB11->SDA   		   

//#define GPIOType GPIOB
//#define GPIO_SCL GPIO_PIN_10
//#define GPIO_SDA GPIO_PIN_11

//实现简单的微秒级延迟
#define CPU_FREQUENCY_MHZ 84
void delay_us(uint32_t us)
{
	uint32_t delay = us * CPU_FREQUENCY_MHZ/4;
	do 
	{
		__NOP();
	}while(delay--);
}

//MPU IIC 延时函数
void MPU_IIC_Delay(void)
{
	delay_us(2);
}

//初始化IIC
//默认 PB10->SCL;PB11->SDA
void MPU_IIC_Init(void)
{					     
  GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	__HAL_RCC_GPIOB_CLK_ENABLE();				//先使能外设IO PORTC时钟 
 
	GPIO_InitStruct.Pin = GPIO_SCL|GPIO_SDA;		// 端口配置
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;					//推挽输出
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;			//IO口速度:range 25 MHz to 100 MHz
  HAL_GPIO_Init(GPIOType, &GPIO_InitStruct);						//根据设定参数初始化GPIO 
	
	HAL_GPIO_WritePin(GPIOType, GPIO_SCL|GPIO_SDA, GPIO_PIN_SET);//PB10,PB11 输出高	
}

void MPU_SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	//__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_SDA;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOType, &GPIO_InitStruct);
}
void MPU_SDA_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	//__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_SDA;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOType, &GPIO_InitStruct);
}

//产生IIC起始信号
void MPU_IIC_Start(void)
{
	MPU_SDA_OUT();     //sda线输出
	MPU_IIC_SDA_SET;	  	  
	MPU_IIC_SCL_SET;
	MPU_IIC_Delay();
 	MPU_IIC_SDA_RESET;//START:when CLK is high,DATA change form high to low 
	MPU_IIC_Delay();
	MPU_IIC_SCL_RESET;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void MPU_IIC_Stop(void)
{
	MPU_SDA_OUT();//sda线输出
	MPU_IIC_SCL_RESET;
	MPU_IIC_SDA_RESET;//STOP:when CLK is high DATA change form low to high
 	MPU_IIC_Delay();
	MPU_IIC_SCL_SET;  
	MPU_IIC_SDA_SET;//发送I2C总线结束信号
	MPU_IIC_Delay();							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 MPU_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	MPU_SDA_IN();      //SDA设置为输入  
	MPU_IIC_SDA_SET;MPU_IIC_Delay();	   
	MPU_IIC_SCL_SET;MPU_IIC_Delay();	 
	while(MPU_READ_SDA_Read)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			MPU_IIC_Stop();
			return 1;
		}
	}
	MPU_IIC_SCL_RESET;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void MPU_IIC_Ack(void)
{
	MPU_IIC_SCL_RESET;
	MPU_SDA_OUT();
	MPU_IIC_SDA_RESET;
	MPU_IIC_Delay();
	MPU_IIC_SCL_SET;
	MPU_IIC_Delay();
	MPU_IIC_SCL_RESET;
}
//不产生ACK应答		    
void MPU_IIC_NAck(void)
{
	MPU_IIC_SCL_RESET;
	MPU_SDA_OUT();
	MPU_IIC_SDA_SET;
	MPU_IIC_Delay();
	MPU_IIC_SCL_SET;
	MPU_IIC_Delay();
	MPU_IIC_SCL_RESET;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void MPU_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	MPU_SDA_OUT(); 	    
    MPU_IIC_SCL_RESET;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
      if((txd&0x80)>>7)
			{MPU_IIC_SDA_SET;}
			else{MPU_IIC_SDA_RESET;}
      txd<<=1; 	  
			MPU_IIC_SCL_SET;
			MPU_IIC_Delay(); 
			MPU_IIC_SCL_RESET;	
			MPU_IIC_Delay();
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 MPU_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        MPU_IIC_SCL_RESET; 
        MPU_IIC_Delay();
		MPU_IIC_SCL_SET;
        receive<<=1;
        if(MPU_READ_SDA_Read)receive++;   
		MPU_IIC_Delay(); 
    }					 
    if (!ack)
        MPU_IIC_NAck();//发送nACK
    else
        MPU_IIC_Ack(); //发送ACK   
    return receive;
}


















