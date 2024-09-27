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


class MotorC {
private:

public:
    PIDc PID;

};

#ifdef __cplusplus
}
#endif

#endif //INC_2024_GIMBAL_MOTOR_H
