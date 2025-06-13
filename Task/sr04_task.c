#include "sr04_task.h"

TaskHandle_t sr04_task_handler;
extern TIM_HandleTypeDef  htim3;
uint16_t count;
float distance;

void sr04_task(void const * argument)
{
	sr04_task_handler=xTaskGetHandle(pcTaskGetName(NULL));
	while(1)
	{
		//等待中断中的任务通知
		while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2)==GPIO_PIN_SET)
		{
			__HAL_TIM_SetCounter(&htim3,0);
			HAL_TIM_Base_Start(&htim3);
		}
		else
		{
			HAL_TIM_Base_Stop(&htim3);
			count=__HAL_TIM_GetCounter(&htim3);
			distance=count*0.017;
		}
	}
}