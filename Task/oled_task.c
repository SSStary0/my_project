#include "oled_task.h"
#include "oled.h"

TaskHandle_t oled_task_handler;

extern int Encoder_Left,Encoder_Right;
extern float roll;
uint8_t display_buf[20];
extern float distance;

void oled_task()
{
	while(1)
	{
		vTaskDelay(100);  // 每100ms刷新一次即可
		sprintf((char *)display_buf,"Encoder_L:%d   ",Encoder_Left);
		OLED_ShowString(0,4,display_buf,16);
		sprintf((char *)display_buf,"Encoder_R:%d   ",Encoder_Right);
		OLED_ShowString(0,6,display_buf,16);
		sprintf((char *)display_buf,"roll:%.1f   ",roll);
		OLED_ShowString(0,2,display_buf,16);
		sprintf((char *)display_buf,"distance:%.2f   ",distance);
		OLED_ShowString(0,2,display_buf,12);
	}
}