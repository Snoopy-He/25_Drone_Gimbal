//
// Created by Snoopy on 2024/9/23.
//

#ifndef INC_2024_GIMBAL_PRINTF_TASK_H
#define INC_2024_GIMBAL_PRINTF_TASK_H


#ifdef __cplusplus
extern "C" {
#endif


#include "FreeRTOS.h"
#include "task.h"
#include "can_bsp.h"
#include "debug.h"
#include "INS_task.h"
#include "gimbal.h"

void Printf_Task(void const * argument);

#ifdef __cplusplus
}
#endif


#endif //INC_2024_GIMBAL_PRINTF_TASK_H
