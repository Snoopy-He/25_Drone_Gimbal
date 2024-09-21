//
// Created by Snoopy on 2024/9/21.
//

#ifndef INC_2024_GIMBAL_CAN_BSP_H
#define INC_2024_GIMBAL_CAN_BSP_H


#include "can.h"

void Can_Init(void);
void Can_Filter_Init(void);

#ifdef __cplusplus
class can_bsp {

};
#endif

#endif //INC_2024_GIMBAL_CAN_BSP_H
