//
// Created by 29358 on 2024/9/22.
//

#include "debug.h"

void Debug_Init(void)
{

}

void LED_ON(void)
{
    HAL_GPIO_WritePin(GPIOH,GPIO_PIN_10,GPIO_PIN_SET);
}
