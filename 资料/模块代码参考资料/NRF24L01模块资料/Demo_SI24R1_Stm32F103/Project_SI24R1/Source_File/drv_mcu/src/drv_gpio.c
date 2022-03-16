/**
  ******************************************************************************
  * @author  
  * @version 
  * @date    
  * @brief   
  ******************************************************************************
  * @attention
  *
  * 
  * 
  * 
  ******************************************************************************
  */



#include "drv_gpio.h"






































///**
//  * @brief :输入引脚初始化
//  * @param :
//			Gpio_Port: GPIO端口
//			Gpio_Pins: GPIO引脚
//			Gpio_Input_Mode: GPIO输入模式
//  * @note  :无
//  * @retval:无
//  */ 
//void drv_gpio_input_init( GPIO_TypeDef* Gpio_Port, uint16_t Gpio_Pins, GpioInputConfigType Gpio_Input_Mode )
//{
//	uint8_t l_Pinpos = 0;
//	uint32_t l_Tempreg = 0, l_Pos = 0, l_CurrentPin = 0;
//	
//	//低7位
//	if( 0x00 != ( Gpio_Pins & 0x00FF ))		
//	{
//		l_Tempreg = Gpio_Port->CRL;				//读取寄存器当前值
//		
//		for( l_Pinpos = 0; l_Pinpos < 0x08; l_Pinpos++ )	//轮训扫描bit0~bit7
//		{
//			l_Pos = ((uint32_t) 0x01) << l_Pinpos ;
//			l_CurrentPin = Gpio_Pins & l_Pos;
//			
//			if( l_CurrentPin == l_Pos )		//查询到相应的位
//			{	
//				l_Pos = l_Pinpos << 2;		//每种模式4个bit配置即右移2位
//				l_Tempreg &= ( uint32_t )( ~( (uint32_t)0x0F << l_Pos ));	//清零相应的位
//				
//				switch( Gpio_Input_Mode )
//				{
//					case GPIO_IN_AIN:		
//							l_Tempreg |= (uint32_t)0x00 << l_Pos;break;		//CNF1 = 0, CNF0 = 0, MODE1 = 0, MODE0 = 0
//					case GPIO_IN_FLOAT:
//							l_Tempreg |= (uint32_t)0x02 << l_Pos;break;		//CNF1 = 0, CNF0 = 1, MODE1 = 0, MODE0 = 0
//					case GPIO_IN_UP:
//					case GPIO_IN_DOWN:
//							l_Tempreg |= (uint32_t)0x04 << l_Pos;break;		//CNF1 = 1, CNF0 = 0, MODE1 = 0, MODE0 = 0
//					default:
//						break;
//				}	
//			}
//		}
//		
//		Gpio_Port->CRL = l_Tempreg;			//写入寄存器
//	}
//	
//	//高8位
//	if( Gpio_Pins > 0x00FF )
//	{
//		l_Tempreg = Gpio_Port->CRH;				//读取寄存器当前值
//		
//		for( l_Pinpos = 0; l_Pinpos < 0x08; l_Pinpos++ )	//轮训扫描bit8~bit15
//		{
//			l_Pos = ((uint32_t) 0x01) << l_Pinpos ;
//			l_CurrentPin = Gpio_Pins & l_Pos;
//			
//			if( l_CurrentPin == l_Pos )		//查询到相应的位
//			{	
//				l_Pos = l_Pinpos << 2;		//每种模式4个bit配置即右移2位
//				l_Tempreg &= ( uint32_t )( ~( (uint32_t)0x0F << l_Pos ));	//清零相应的位
//				
//				switch( Gpio_Input_Mode )
//				{
//					case GPIO_IN_AIN:		
//							l_Tempreg |= (uint32_t)0x00 << l_Pos;break;		//CNF1 = 0, CNF0 = 0, MODE1 = 0, MODE0 = 0  模拟输入
//					case GPIO_IN_FLOAT:
//							l_Tempreg |= (uint32_t)0x02 << l_Pos;break;		//CNF1 = 0, CNF0 = 1, MODE1 = 0, MODE0 = 0	浮空输入
//					case GPIO_IN_UP:
//					case GPIO_IN_DOWN:
//							l_Tempreg |= (uint32_t)0x04 << l_Pos;break;		//CNF1 = 1, CNF0 = 0, MODE1 = 0, MODE0 = 0	上/下拉输入
//					default:
//						break;
//				}	
//			}
//		}
//		
//		Gpio_Port->CRH = l_Tempreg;		//写入寄存器
//	}
//	
//	if( Gpio_Input_Mode == GPIO_IN_UP )
//	{
//		Gpio_Port->BSRR |= (uint32_t)Gpio_Pins;    		//相应引脚初始状态为高
//	}
//	if( Gpio_Input_Mode == GPIO_IN_DOWN )
//	{
//		Gpio_Port->BSRR |= (uint32_t)Gpio_Pins << 16;	//相应引脚初始状态为低
//	}
//	
//}

///**
//  * @brief :通用输出引脚初始化
//  * @param :
//			Gpio_Port: GPIO端口
//			Gpio_Pins: GPIO引脚
//			Gpio_Output_Mode: GPIO输出模式
//			Gpio_Speed: 输出速度
//  * @note  :无
//  * @retval:无
//  */ 
//void drv_gpio_output_init( GPIO_TypeDef* Gpio_Port, uint16_t Gpio_Pins, GpioOutputConfigType Gpio_Output_Mode, GpioSpeedConfigType Gpio_Speed )
//{
//	uint8_t l_Pinpos = 0;
//	uint32_t l_Tempreg = 0, l_Pos = 0, l_CurrentPin = 0;
//	
//	//低7位
//	if( 0x00 != ( Gpio_Pins & 0x00FF ))		
//	{
//		l_Tempreg = Gpio_Port->CRL;			//读取寄存器当前值
//		
//		for( l_Pinpos = 0; l_Pinpos < 0x08; l_Pinpos++ )	//轮训扫描bit0~bit7
//		{
//			l_Pos = ((uint32_t) 0x01) << l_Pinpos ;
//			l_CurrentPin = Gpio_Pins & l_Pos;
//			
//			if( l_CurrentPin == l_Pos )		//查询到相应的位
//			{	
//				l_Pos = l_Pinpos << 2;		//每种模式4个bit配置即右移2位
//				l_Tempreg &= ( uint32_t )( ~( (uint32_t)0x0F << l_Pos ));	//清零相应的位
//				l_Tempreg |= ( uint32_t )( (uint32_t)((uint8_t)Gpio_Output_Mode | (uint8_t)Gpio_Speed ) << l_Pos );		//设置输出模式和速度
//			}
//		}
//		
//		Gpio_Port->CRL = l_Tempreg;		//写入寄存器
//	}
//	
//	//高8位
//	if( Gpio_Pins > 0x00FF )
//	{
//		l_Tempreg = Gpio_Port->CRH;
//		
//		for( l_Pinpos = 0; l_Pinpos < 0x08; l_Pinpos++ )	//轮训扫描bit8~bit15
//		{
//			l_Pos = ((uint32_t) 0x01) << l_Pinpos ;
//			l_CurrentPin = Gpio_Pins & l_Pos;
//			
//			if( l_CurrentPin == l_Pos )		//查询到相应的位
//			{	
//				l_Pos = l_Pinpos << 2;		//每种模式4个bit配置即右移2位
//				l_Tempreg &= ( uint32_t )( ~( (uint32_t)0x0F << l_Pos ));	//清零相应的位
//				l_Tempreg |= ( uint32_t )( (uint32_t)((uint8_t)Gpio_Output_Mode | (uint8_t)Gpio_Speed ) << l_Pos );		//设置输出模式和速度	
//			}
//		}
//		
//		Gpio_Port->CRH = l_Tempreg;			//写入寄存器
//	}

//	Gpio_Port->BSRR |= (uint32_t)Gpio_Pins << 16;	//相应引脚初始状态为低
//}

///**
//  * @brief :复用引脚初始化
//  * @param :
//			Gpio_Port: GPIO端口
//			Gpio_Pins: GPIO引脚
//			Gpio_Af: GPIO复用类型
//			Gpio_Speed: 输出速度
//  * @note  :无
//  * @retval:无
//  */
//void drv_gpio_af_init( GPIO_TypeDef* Gpio_Port, uint16_t Gpio_Pins, GpioAFConfigType Gpio_Af, GpioSpeedConfigType Gpio_Speed )
//{
//	uint8_t l_Pinpos = 0;
//	uint32_t l_Tempreg = 0, l_Pos = 0, l_CurrentPin = 0;
//	
//	//低7位
//	if( 0x00 != ( Gpio_Pins & 0x00FF ))		
//	{
//		l_Tempreg = Gpio_Port->CRL;			//读取寄存器当前值
//		
//		for( l_Pinpos = 0; l_Pinpos < 0x08; l_Pinpos++ )	//轮训扫描bit0~bit7
//		{
//			l_Pos = ((uint32_t) 0x01) << l_Pinpos ;
//			l_CurrentPin = Gpio_Pins & l_Pos;
//			
//			if( l_CurrentPin == l_Pos )		//查询到相应的位
//			{	
//				l_Pos = l_Pinpos << 2;		//每种模式4个bit配置即右移2位
//				l_Tempreg &= ( uint32_t )( ~( (uint32_t)0x0F << l_Pos ));	//清零相应的位
//				l_Tempreg |= ( uint32_t )( (uint32_t)((uint8_t)Gpio_Af | (uint8_t)Gpio_Speed ) << l_Pos );		//设置复用类型和速度
//			}
//		}
//		
//		Gpio_Port->CRL = l_Tempreg;		//写入寄存器
//	}
//	
//	//高8位
//	if( Gpio_Pins > 0x00FF )
//	{
//		l_Tempreg = Gpio_Port->CRH;
//		
//		for( l_Pinpos = 0; l_Pinpos < 0x08; l_Pinpos++ )	//轮训扫描bit8~bit15
//		{
//			l_Pos = ((uint32_t) 0x01) << l_Pinpos ;
//			l_CurrentPin = Gpio_Pins & l_Pos;
//			
//			if( l_CurrentPin == l_Pos )		//查询到相应的位
//			{	
//				l_Pos = l_Pinpos << 2;		//每种模式4个bit配置即右移2位
//				l_Tempreg &= ( uint32_t )( ~( (uint32_t)0x0F << l_Pos ));	//清零相应的位
//				l_Tempreg |= ( uint32_t )( (uint32_t)((uint8_t)Gpio_Af | (uint8_t)Gpio_Speed ) << l_Pos );		//设置复用类型和速度	
//			}
//		}
//		
//		Gpio_Port->CRH = l_Tempreg;			//写入寄存器
//	}

//	Gpio_Port->BSRR |= (uint32_t)Gpio_Pins << 16;	//相应引脚初始状态为低	
//}

///**
//* @brief :设置引脚为高
//  * @param :
//			Gpio_Port: GPIO端口
//			Gpio_Pins: GPIO引脚
//  * @note  :无
//  * @retval:无
//  */
//void drv_gpio_set_pins_high( GPIO_TypeDef* Gpio_Port, uint16_t Gpio_Pins )
//{
//	Gpio_Port->BSRR = Gpio_Pins;		//口线置高
//}

///**
//* @brief :设置引脚为低
//  * @param :
//			Gpio_Port: GPIO端口
//			Gpio_Pins: GPIO引脚
//  * @note  :无
//  * @retval:无
//  */
//void drv_gpio_set_pins_low( GPIO_TypeDef* Gpio_Port, uint16_t Gpio_Pins )
//{
//	Gpio_Port->BRR = Gpio_Pins;			//口线置低
//}

///**
//* @brief :读端口输入状态
//  * @param :
//			Gpio_Port: GPIO端口
//  * @note  :无
//  * @retval:无
//  */
//uint16_t drv_gpio_read_input_status( GPIO_TypeDef* Gpio_Port )
//{
//	return (uint16_t)(Gpio_Port->IDR);	//返回口线状态
//}

