//
// Created by Snoopy on 2024/9/21.
//
#include "main.h"
#include "remotec.h"
#include "can_bsp.h"




int main(void)
{
    main_Init();
    REMOTEC_Init();
    Can_Init();
    Can_Filter_Init();
    while (1)
    {
        Can_Send(0x200,500,500,0,0);
    }
}