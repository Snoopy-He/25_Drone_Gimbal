//
// Created by Snoopy on 2024/9/21.
//

#ifndef INC_2024_GIMBAL_MOTOR_H
#define INC_2024_GIMBAL_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "remotec.h"
#include "can_bsp.h"
#include "pid.h"

#define YawAngMax   10
#define YawAngMin   -10
#define PitchAngMax   30
#define PitchAngMin   -30
#define TIMpiece    0.002   //控制周期，时间切片

void FricL_PID_Init(void);
void FricR_PID_Init(void);
void Rammc_PID_Init(void);
void Pitch_PID_Init(void);
void Yaw_PID_Init(void);
float GM6020_Angle_limit(float input_data,float Angle_Set,float Middle_Angle,float Angle_now);
float DM4310_Angle_limit(float input_data,float Angle_Set,float Middle_Angle,float Angle_now);
float DJI_Motor_Angle_Correction(float Angle,float Middle_Angle);
float DM_Motor_Angle_Correction(float Angle,float Middle_Angle);
void Pitch_Target_Set(void);
void Yaw_Target_Set(void);


typedef struct
{
    float Algo_Data = 0.0f;
    float Middle_Angle = 0.0f;
    float Target_before_Filter = 0.0f;
    float Target = 0.0f;
    float Corrected_Angle;
}gimbal_angle;



class MotorC {
private:

public:
    PIDc PID;

};

#ifdef __cplusplus
}
#endif

#endif //INC_2024_GIMBAL_MOTOR_H
