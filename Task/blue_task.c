#include "blue_task.h"
#include "usart.h"

uint8_t Bluetooth_data;
uint8_t Fore,Back,Left,Right;
extern UART_HandleTypeDef huart3;

TaskHandle_t blue_task_handler;

void blue_task(void const * argument)
{
	//获取当前任务的任务句柄
  blue_task_handler= xTaskGetHandle(pcTaskGetName(NULL));
	
	HAL_UART_Receive_DMA(&huart3, bt_rx_buffer, BT_RX_BUFFER_SIZE);
	
	while(1)
	{
		//等待中断中的任务通知
		while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
		{
		}
		for (int i = 0; i < rx_len; ++i)
    {
			Bluetooth_data=bt_rx_buffer[i];
			switch(Bluetooth_data)
			{
				case 0x00:
					Fore=0,Back=0,Left=0,Right=0;
					break;
				case 0x01:
					Fore=1,Back=0,Left=0,Right=0;
					break;
				case 0x03:
					Fore=0,Back=1,Left=0,Right=0;
					break;
				case 0x05:
					Fore=0,Back=0,Left=0,Right=1;
					break;
				case 0x07:
					Fore=0,Back=0,Left=1,Right=0;
					break;
				default:
					Fore=0,Back=0,Left=0,Right=0;
			}
		}
		HAL_UART_Receive_DMA(&huart3, bt_rx_buffer, BT_RX_BUFFER_SIZE);
		rx_len=0;
	}
}