//
// Created by 29358 on 2024/11/10.
//

#ifndef INC_2024_GIMBAL_IMU_BSP_DELAY_H
#define INC_2024_GIMBAL_IMU_BSP_DELAY_H

#include "stm32f4xx_hal.h"

extern void delay_init(void);
extern void delay_us(uint16_t nus);
extern void delay_ms(uint16_t nms);

class imu_bsp_delay {

};


#endif //INC_2024_GIMBAL_IMU_BSP_DELAY_H
