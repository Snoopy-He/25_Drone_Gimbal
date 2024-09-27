//
// Created by 29358 on 2024/9/23.
//

#include "Gimbal_Motor_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can_bsp.h"
#include "debug.h"

void Gimbal_Motor_Task(void const * argument)
{
    /* USER CODE BEGIN Gimbal_Motor_Task */
    /* Infinite loop */
    portTickType CurrentTime;
    for(;;)
    {
        CurrentTime = xTaskGetTickCount();
        //LED_ON();

        Gimbal_loop();
        //Can_Send(0X200,1000,0,0,0);

        vTaskDelayUntil(&CurrentTime, 2 / portTICK_RATE_MS);
    }
    /* USER CODE END Gimbal_Motor_Task */
}
