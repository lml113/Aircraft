#include "mpuiic.h"
#include "gpio.h"


//����I2C PB10->SCL;PB11->SDA   		   

//#define GPIOType GPIOB
//#define GPIO_SCL GPIO_PIN_10
//#define GPIO_SDA GPIO_PIN_11

//ʵ�ּ򵥵�΢�뼶�ӳ�
#define CPU_FREQUENCY_MHZ 84
void delay_us(uint32_t us)
{
	uint32_t delay = us * CPU_FREQUENCY_MHZ/4;
	do 
	{
		__NOP();
	}while(delay--);
}

//MPU IIC ��ʱ����
void MPU_IIC_Delay(void)
{
	delay_us(2);
}

//��ʼ��IIC
//Ĭ�� PB10->SCL;PB11->SDA
void MPU_IIC_Init(void)
{					     
  GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	__HAL_RCC_GPIOB_CLK_ENABLE();				//��ʹ������IO PORTCʱ�� 
 
	GPIO_InitStruct.Pin = GPIO_SCL|GPIO_SDA;		// �˿�����
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;					//�������
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;			//IO���ٶ�:range 25 MHz to 100 MHz
  HAL_GPIO_Init(GPIOType, &GPIO_InitStruct);						//�����趨������ʼ��GPIO 
	
	HAL_GPIO_WritePin(GPIOType, GPIO_SCL|GPIO_SDA, GPIO_PIN_SET);//PB10,PB11 �����	
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

//����IIC��ʼ�ź�
void MPU_IIC_Start(void)
{
	MPU_SDA_OUT();     //sda�����
	MPU_IIC_SDA_SET;	  	  
	MPU_IIC_SCL_SET;
	MPU_IIC_Delay();
 	MPU_IIC_SDA_RESET;//START:when CLK is high,DATA change form high to low 
	MPU_IIC_Delay();
	MPU_IIC_SCL_RESET;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void MPU_IIC_Stop(void)
{
	MPU_SDA_OUT();//sda�����
	MPU_IIC_SCL_RESET;
	MPU_IIC_SDA_RESET;//STOP:when CLK is high DATA change form low to high
 	MPU_IIC_Delay();
	MPU_IIC_SCL_SET;  
	MPU_IIC_SDA_SET;//����I2C���߽����ź�
	MPU_IIC_Delay();							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 MPU_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	MPU_SDA_IN();      //SDA����Ϊ����  
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
	MPU_IIC_SCL_RESET;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
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
//������ACKӦ��		    
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
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void MPU_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	MPU_SDA_OUT(); 	    
    MPU_IIC_SCL_RESET;//����ʱ�ӿ�ʼ���ݴ���
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
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 MPU_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_SDA_IN();//SDA����Ϊ����
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
        MPU_IIC_NAck();//����nACK
    else
        MPU_IIC_Ack(); //����ACK   
    return receive;
}


















