#ifndef _SR04_TASK_H
#define _SR04_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx_hal.h"

extern TaskHandle_t sr04_task_handler;

void sr04_task(void const * argument);

#endif
