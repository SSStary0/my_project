#include "get_distance_task.h"

TaskHandle_t get_distance_task_handler;

void RCCdelay_us(uint32_t udelay) {
__IO uint32_t Delay = udelay * 72 / 8;
do {
__NOP();
} while (Delay--);
}

void get_distance_task()
{
	while(1)
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
		RCCdelay_us(12);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
		vTaskDelay(40);  // 每40ms触发一次测距
	}
}