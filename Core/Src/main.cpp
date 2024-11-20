//
// Created by Snoopy on 2024/9/21.
//
#include "main.h"
#include "remotec.h"
#include "can_bsp.h"
#include "gimbal.h"
#include "debug.h"
#include "BMI088driver.h"
#include "ist8310driver.h"


int main(void)
{
    main_Init();
    REMOTEC_Init();
    Can_Init();
    Algorithm_Init();
    HAL_Delay(1500);    //达妙电机的上电自检，等一下自检
    Motor_Init();
    Debug_Init();
    Middle_Angle_Set(8135.0f,140.0f);

    os_Init();

    while (1)
    {
    }
}