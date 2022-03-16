/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "mpu6050.h"//MPU6050驱动库
#include "inv_mpu.h"//陀螺仪驱动库
#include "inv_mpu_dmp_motion_driver.h" //DMP姿态解读库
#include "SI24R1.h"
#include "bmp280.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
char tmp_buf[33];			//字符串数组
float pitch,roll,yaw; 		//欧拉角:俯仰角，偏航角，滚转角
short aacx,aacy,aacz;		//加速度传感器原始数据  angular acceleration
short gyrox,gyroy,gyroz;	//陀螺仪原始数据  gyroscope
short temp;					//温度

struct MPU6050				//MPU6050结构体
{
	u8 flag;				//采集成功标志位
	u8 speed;				//上报速度
}mpu6050;					//唯一结构体变量

//SI24R1有关变量
uint8_t Data[32]={0};

//BMP280相关变量
BMP280_HandleTypedef bmp280;

float pressure, temperature, humidity;

uint8_t bmp280_id = 0;

uint8_t ctr_reg = 0;
uint8_t status_reg = 0;
 
int32_t tem = 0;

double Pressure,Temperature;

PID_TYPE PID_ROL_Angle;
PID_TYPE PID_PIT_Angle;
PID_TYPE PID_YAW_Angle;

//
uint8_t Airplane_Enable = 0;
uint16_t motor_pwm[4]={0};
uint16_t motor_THROTTLE = 0;
float pitch_target=0,roll_target=0,yaw_target=0; 		//当前期望值：欧拉角:俯仰角，偏航角，滚转角

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	MPU_Init();	//初始化MPU6050
	while(mpu_dmp_init())                   	//初始化mpu_dmp库
 	{
		printf("MPU6050 initialization failed！\r\n");	//串口初始化失败上报
	}
	printf("MPU6050 initialization succeed！\r\n");		//串口初始化成功上报
	HAL_Delay(999);														//延时初界面显示
	mpu6050.flag = 0;                      		//采集成功标志位初始化
	mpu6050.speed = 0;												//上报速度初始化
	
	//初始化BMP280
	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c1;

	while (!bmp280_init(&bmp280, &bmp280.params)) {
		printf("BMP280 initialization failed!\n");
		HAL_Delay(2000);
	}
	//bool bme280p = bmp280.id == BME280_CHIP_ID;
	printf("BMP280 initialization succeed!\n");
	
	//初始化SI24R1
	while(NRF24L01_Check())
  {
	  printf("SI24R1 initialization failed！\r\n");
	  HAL_Delay(100);
  }
  printf("SI24R1 initialization succeed！\r\n");
  NRF24L01_RX_Mode();
	//开启PWM，控制电机
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	
	//初始化PID
	pidParameter_init();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//处理获取的MPU6050数据
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)//dmp处理得到数据，对返回值进行判断
		{ 
			temp=MPU_Get_Temperature();	                //得到温度值
			//MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			//MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
			mpu6050.speed++;                            //上报速度自加
			if(mpu6050.speed == 1)						//上报速度阈值设置
			{
				mpu6050.flag = 1;						//采集成功标志位设置为有效
				mpu6050.speed = 0;						//上报速度归零
			}	
		}
		else 											//采集不成功										
		{
			mpu6050.flag = 0;							//采集成功标志位设置为无效
		}
		
		if(mpu6050.flag == 1)						//采集成功时
		{ 
			PID_Postion_Cal(&PID_ROL_Angle,roll_target,roll);
			PID_Postion_Cal(&PID_PIT_Angle,pitch_target,pitch);
			PID_Postion_Cal(&PID_YAW_Angle,yaw_target,yaw);
			mpu6050.flag = 0;									//采集成功标志位设置为无效
		}
		else ;														//防卡死
		
		//获取BMP280数据
		while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
			printf("BMP280 initialization failed!\t");
			HAL_Delay(500);
		}

//		printf("Pressure: %.2f Pa, Temperature: %.2f C\t\n",pressure, temperature);
//		if (bme280p) {
//			printf(", Humidity: %.2f\n", humidity);
//		}

		//获取SI24R1数据
		NRF24L01_RxPacket(Data);
//		if(NRF24L01_RxPacket(Data)==0)
//		{
//			printf("SI24R1数据为：");
//			for(char i = 0; i<8 ; i++)
//			{
//				printf("%d\t",Data[i]);
//			}
//			printf("\n");
//		}
		
		//控制电机
		if(Data[0]==1&&Airplane_Enable==0)		//一键起飞
		{
			Airplane_Enable = 1;
			motor_THROTTLE = 360;
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,motor_THROTTLE);
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,motor_THROTTLE);
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,motor_THROTTLE);
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,motor_THROTTLE);
		}
		else if(Data[0]==2)
		{
			Airplane_Enable = 0;
			motor_THROTTLE = 0;
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,0);
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,0);
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,0);
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,0);
		}
		
		if(Airplane_Enable){
			motor_pwm[0]=motor_THROTTLE-PID_ROL_Angle.OutPut-PID_PIT_Angle.OutPut+PID_YAW_Angle.OutPut;
			motor_pwm[1]=motor_THROTTLE-PID_ROL_Angle.OutPut+PID_PIT_Angle.OutPut-PID_YAW_Angle.OutPut;
			motor_pwm[2]=motor_THROTTLE+PID_ROL_Angle.OutPut+PID_PIT_Angle.OutPut+PID_YAW_Angle.OutPut;
			motor_pwm[3]=motor_THROTTLE+PID_ROL_Angle.OutPut-PID_PIT_Angle.OutPut-PID_YAW_Angle.OutPut;

			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,motor_pwm[0]);
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,motor_pwm[1]);
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,motor_pwm[2]);
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,motor_pwm[3]);
			
		if(Data[4]>135)
			motor_THROTTLE = 360+5*(Data[4]-135);
		else if(Data[4]<100)
			motor_THROTTLE = 360-3*(100-Data[4]);
		else
			motor_THROTTLE = 360;
		if(Data[1]>135)
			roll_target = Data[1]-135;
		else if(Data[1]<100)
			roll_target = Data[1]-100;
		else
			roll_target=0;
		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
