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

void FricL_PID_Init(void);
void FricR_PID_Init(void);
void Rammc_PID_Init(void);
void Pitch_PID_Init(void);
void Yaw_PID_Init(void);
float GM6020_Angle_limit(float input_data,float Angle_Set,float Middle_Angle,float Angle_now);
float DM4310_Angle_limit(float input_data,float Angle_Set,float Middle_Angle,float Angle_now);

class MotorC {
private:

public:
    PIDc PID;

};

#ifdef __cplusplus
}
#endif

#endif //INC_2024_GIMBAL_MOTOR_H
