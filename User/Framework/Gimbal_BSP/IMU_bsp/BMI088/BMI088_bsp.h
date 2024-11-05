//
// Created by snoopy on 2024/11/5.
//

#ifndef INC_2024_GIMBAL_BMI088_BSP_H
#define INC_2024_GIMBAL_BMI088_BSP_H

#include "stm32f4xx_hal.h"
#include "BMI088_reg_bsp.h"

typedef struct{
    struct {
        float x;
        float y;
        float z;
    } Accel_raw_data;

    struct {
        float roll;
        float pitch;
        float yaw;
    } Gyro_raw_data;

    int16_t temperature;
}BMI088_Raw_Data;

void BMI088_Gyro_Enable(void);
void BMI088_Accel_Enable(void);
void BMI088_Enable(void);
BMI088_Raw_Data BMI088_Temp_Read(void);
BMI088_Raw_Data BMI088_Accel_Read(void);
BMI088_Raw_Data BMI088_Gyro_Read(void);
void BMI088_Accel_Read_Irqhandler(void);
void BMI088_Gyro_Read_Irqhandler(void);

class BMI088_bsp {

};


#endif //INC_2024_GIMBAL_BMI088_BSP_H
