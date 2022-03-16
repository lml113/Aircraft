#include "key.h"

uint8_t KEY_Scan(void)
{
	if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6)==0)
	{
		HAL_Delay(20);
		if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6)==0)
			return 1;
	}
	else if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_7)==0)
	{
		HAL_Delay(20);
		if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_7)==0)
			return 2;
	}
	else;
	return 0;
}

