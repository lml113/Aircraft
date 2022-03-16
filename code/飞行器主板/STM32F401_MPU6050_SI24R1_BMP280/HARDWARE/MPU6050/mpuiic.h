#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "main.h"

//正点原子MPU6050通讯线驱动
//定义I2C PB10->SCL;PB11->SDA   		   

#define GPIOType GPIOB
#define GPIO_SCL GPIO_PIN_10
#define GPIO_SDA GPIO_PIN_3


//IO操作函数	 
#define MPU_IIC_SCL_SET    	HAL_GPIO_WritePin(GPIOType, GPIO_SCL, GPIO_PIN_SET); 		//SCL
#define MPU_IIC_SCL_RESET   HAL_GPIO_WritePin(GPIOType, GPIO_SCL, GPIO_PIN_RESET); 		//SCL
#define MPU_IIC_SDA_SET    	HAL_GPIO_WritePin(GPIOType, GPIO_SDA, GPIO_PIN_SET); 		//SDA
#define MPU_IIC_SDA_RESET  	HAL_GPIO_WritePin(GPIOType, GPIO_SDA, GPIO_PIN_RESET); 		//SDA	 
#define MPU_READ_SDA_Read  	HAL_GPIO_ReadPin(GPIOType, GPIO_SDA) 		//输入SDA 


#define u8 uint8_t
#define u16 uint16_t

//IIC所有操作函数
void MPU_IIC_Delay(void);				//MPU IIC延时函数
void MPU_IIC_Init(void);                //初始化IIC的IO口				 
void MPU_IIC_Start(void);				//发送IIC开始信号
void MPU_IIC_Stop(void);	  			//发送IIC停止信号
void MPU_IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 MPU_IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 MPU_IIC_Wait_Ack(void); 				//IIC等待ACK信号
void MPU_IIC_Ack(void);					//IIC发送ACK信号
void MPU_IIC_NAck(void);				//IIC不发送ACK信号

//IO方向设置
void MPU_SDA_IN(void);
void MPU_SDA_OUT(void);

void IMPU_IC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 MPU_IIC_Read_One_Byte(u8 daddr,u8 addr);	  
#endif
















