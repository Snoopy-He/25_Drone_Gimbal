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
#include "pid.h"
#include "debug.h"

#define YAW_MOTOR_ID       0x209  //0x204+5
#define PITCH_MOTOR_ID     0x001
#define L_FRIC_ID          0x202
#define R_FRIC_ID          0x201
#define RAMMC_ID           0x203

void Can_Init(void);
void Can_bsp_Init(void);
void Can_Filter_Init(void);
void Can1_Send(int16_t ID,int16_t Mess_1,int16_t Mess_2,int16_t Mess_3,int16_t Mess_4);
void Can2_Send(int16_t ID,int16_t Mess_1,int16_t Mess_2,int16_t Mess_3,int16_t Mess_4);
void DM_Motor_Enable(int16_t ID);
void DM_Motor_Speed_Mode_Send(int16_t ID,float Message);
void Can2_bsp_IRQHandler(void);
void Can1_bsp_IRQHandler(void);

typedef struct
{
    int16_t Angle;         //电机旋转的角度，0-8191对应0-360
    int16_t Speed;         //电机旋转的速度，单位rpm
    int16_t Torque;        //电机旋转的转矩
    int16_t Temperature;   //电机的温度

}Rx_Data;

typedef struct
{
    float Angle;         //电机旋转的角度，0-8191对应0-360
    float Speed;         //电机旋转的速度，单位rpm
    float Torque;        //电机旋转的转矩
    int16_t Temperature;   //电机的温度

}Rx_DM_Data;

extern Rx_Data FricL_Data;
extern Rx_Data FricR_Data;
extern Rx_Data Rammc_Data;
extern Rx_Data YawMotor_Data;
extern Rx_DM_Data PitchMotor_Data;

extern int16_t Can_Tx_Data[5];


#ifdef __cplusplus
class can_bsp {

};
#endif

#ifdef __cplusplus
}
#endif

#endif //INC_2024_GIMBAL_CAN_BSP_H
