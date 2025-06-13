#ifndef _BLUE_TASK_H
#define _BLUE_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

extern TaskHandle_t blue_task_handler;

void blue_task();

#endif
