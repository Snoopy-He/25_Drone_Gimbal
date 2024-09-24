//
// Created by 29358 on 2024/9/23.
//

#include "Printf_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "debug.h"
#include "cstdio"

void Printf_Task(void const * argument)
{
    /* USER CODE BEGIN Printf_Task */
    /* Infinite loop */
    portTickType CurrentTime;
    for(;;)
    {
        CurrentTime = xTaskGetTickCount();

        vTaskDelayUntil(&CurrentTime, 2 / portTICK_RATE_MS);
    }
    /* USER CODE END Printf_Task */
}
