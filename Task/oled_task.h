#ifndef _OLED_TASK_H
#define _OLED_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

extern TaskHandle_t oled_task_handler;

void oled_task();

#endif
