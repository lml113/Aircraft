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


#ifndef __DRV_BUTTON_H__
#define __DRV_BUTTON_H__


#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

//按键硬件定义 KEY0
#define BUTOTN_GPIO_PORT			GPIOE									
#define BUTTON_GPIO_CLK				RCC_APB2Periph_GPIOE
#define BUTTON_GPIO_PIN				GPIO_Pin_4


/** 按键状态定义 */
enum
{
	BUTOTN_UP = 0,		//按键未按下
	BUTOTN_PRESS_DOWN	//按键按下
};



void drv_button_init( void );
uint8_t drv_button_check( void );

#endif

