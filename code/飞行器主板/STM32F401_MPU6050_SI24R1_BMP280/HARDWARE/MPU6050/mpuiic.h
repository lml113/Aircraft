#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "main.h"

//����ԭ��MPU6050ͨѶ������
//����I2C PB10->SCL;PB11->SDA   		   

#define GPIOType GPIOB
#define GPIO_SCL GPIO_PIN_10
#define GPIO_SDA GPIO_PIN_3


//IO��������	 
#define MPU_IIC_SCL_SET    	HAL_GPIO_WritePin(GPIOType, GPIO_SCL, GPIO_PIN_SET); 		//SCL
#define MPU_IIC_SCL_RESET   HAL_GPIO_WritePin(GPIOType, GPIO_SCL, GPIO_PIN_RESET); 		//SCL
#define MPU_IIC_SDA_SET    	HAL_GPIO_WritePin(GPIOType, GPIO_SDA, GPIO_PIN_SET); 		//SDA
#define MPU_IIC_SDA_RESET  	HAL_GPIO_WritePin(GPIOType, GPIO_SDA, GPIO_PIN_RESET); 		//SDA	 
#define MPU_READ_SDA_Read  	HAL_GPIO_ReadPin(GPIOType, GPIO_SDA) 		//����SDA 


#define u8 uint8_t
#define u16 uint16_t

//IIC���в�������
void MPU_IIC_Delay(void);				//MPU IIC��ʱ����
void MPU_IIC_Init(void);                //��ʼ��IIC��IO��				 
void MPU_IIC_Start(void);				//����IIC��ʼ�ź�
void MPU_IIC_Stop(void);	  			//����IICֹͣ�ź�
void MPU_IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 MPU_IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 MPU_IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void MPU_IIC_Ack(void);					//IIC����ACK�ź�
void MPU_IIC_NAck(void);				//IIC������ACK�ź�

//IO��������
void MPU_SDA_IN(void);
void MPU_SDA_OUT(void);

void IMPU_IC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 MPU_IIC_Read_One_Byte(u8 daddr,u8 addr);	  
#endif
















