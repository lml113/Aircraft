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


#include "drv_button.h"
#include "drv_delay.h"



/**
  * @brief :按键初始化
  * @param :无
  * @note  :无
  * @retval:无
  */ 
void drv_button_init( void )
{
	GPIO_InitTypeDef	GpioInitStructer;
	
	//使能口线时钟
	RCC_APB2PeriphClockCmd( BUTTON_GPIO_CLK, ENABLE );	//打开端口时钟
	
	GpioInitStructer.GPIO_Mode = GPIO_Mode_IPU;						
	GpioInitStructer.GPIO_Speed = GPIO_Speed_2MHz;		//输出速度，配置为输入时无影响
	GpioInitStructer.GPIO_Pin = BUTTON_GPIO_PIN;		
	GPIO_Init( BUTOTN_GPIO_PORT, &GpioInitStructer );	//初始化按键 IO引脚,上拉输入
}

/**
  * @brief :按键查询
  * @param :无
  * @note  :无
  * @retval:
  *			0:按键没有按下
  *			1:检测到按键动作
  */
uint8_t drv_button_check( void )
{
	if( BUTTON_GPIO_PIN != ( BUTOTN_GPIO_PORT->IDR & BUTTON_GPIO_PIN ))		//检测按键输入状态
	{
		drv_delay_ms( 50 );			//消抖
		if( BUTTON_GPIO_PIN != ( BUTOTN_GPIO_PORT->IDR & BUTTON_GPIO_PIN ))
		{
			return 1;				//按键按下，返回按键状态
		}
	}
	
	return 0;
}
