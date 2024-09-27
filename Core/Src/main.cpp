//
// Created by Snoopy on 2024/9/21.
//
#include "main.h"
#include "remotec.h"
#include "can_bsp.h"
#include "gimbal.h"
#include "debug.h"




int main(void)
{
    main_Init();
    REMOTEC_Init();
    Can_Init();
    Algorithm_Init();
    Debug_Init();
    os_Init();
    while (1)
    {
    }
}