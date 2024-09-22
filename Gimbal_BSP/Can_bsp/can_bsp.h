//
// Created by Snoopy on 2024/9/21.
//

#ifndef INC_2024_GIMBAL_CAN_BSP_H
#define INC_2024_GIMBAL_CAN_BSP_H

#ifdef __cplusplus
extern "C" {
#endif


#include "can.h"
#include "usart.h"

#define YAW_MOTOR_ID       0x209
#define PITCH_MOTOR_ID     0x206
#define L_FRIC_ID          0x202
#define R_FRIC_ID          0x201
#define RAMMC_ID           0x203

void Can_Init(void);
void Can_Filter_Init(void);
void Can_Send(int16_t ID,int16_t Mess_1,int16_t Mess_2,int16_t Mess_3,int16_t Mess_4);
void Can_bsp_IRQHandler(void);

typedef struct
{
    int16_t Angle;         //电机旋转的角度，0-8191对应0-360
    int16_t Speed;         //电机旋转的速度，单位rpm
    int16_t Torque;        //电机旋转的转矩
    int16_t Temperature;   //电机的温度

}Rx_Data;

extern Rx_Data FricL_Data;
extern Rx_Data FricR_Data;
extern Rx_Data Rammc_Data;
extern Rx_Data YawMotor_Data;
extern Rx_Data PitchMotor_Data;

extern int16_t Can_Tx_Data[5];


#ifdef __cplusplus
class can_bsp {

};
#endif

#ifdef __cplusplus
}
#endif

#endif //INC_2024_GIMBAL_CAN_BSP_H
