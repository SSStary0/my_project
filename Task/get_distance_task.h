#ifndef _GET_DISTANCE_TASK_H
#define _GET_DISTANCE_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx_hal.h"

extern TaskHandle_t get_distance_task_handler;

void get_distance_task();

#endif