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


#ifndef __DRV_DELAY_H__
#define __DRV_DELAY_H__


#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"


/** 采用定时器做精确延时 */
//延时硬件定义
#define DELAY_TIME_BASE					TIM2
#define DELAY_TIME_BASE_CLK				RCC_APB1Periph_TIM2


void drv_delay_init( void );
void drv_delay_us( uint16_t Us );
void drv_delay_ms( uint8_t Ms );
void drv_delay_500Ms( uint8_t Ms_500 );
void drv_delay_free( uint32_t Delay_Time );

#endif

