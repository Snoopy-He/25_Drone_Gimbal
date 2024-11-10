//
// Created by 29358 on 2024/11/10.
//

#ifndef INC_2024_GIMBAL_BMI088_MIDDLEWARE_H
#define INC_2024_GIMBAL_BMI088_MIDDLEWARE_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "imu_bsp_delay.h"

#define BMI088_USE_SPI
#define CS1_ACCEL_GPIO_Port GPIOA
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_GYRO_GPIO_Port GPIOB
#define CS1_GYRO_Pin GPIO_PIN_0
//#define BMI088_USE_IIC

extern void BMI088_GPIO_init(void);
extern void BMI088_com_init(void);
extern void BMI088_delay_ms(uint16_t ms);
extern void BMI088_delay_us(uint16_t us);

#if defined(BMI088_USE_SPI)
extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern uint8_t BMI088_read_write_byte(uint8_t reg);

#elif defined(BMI088_USE_IIC)

#endif

class BMI088_Middleware {

};


#endif //INC_2024_GIMBAL_BMI088_MIDDLEWARE_H
