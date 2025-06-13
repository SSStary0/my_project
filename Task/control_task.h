#ifndef _CONTROL_TASK_H
#define _CONTROL_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx_hal.h"

extern TaskHandle_t control_task_handler;

void control_task();

#endif
