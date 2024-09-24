//
// Created by Snoopy on 2024/9/24.
//

#include "gimbal.h"

MotorC FricL;
MotorC FricR;
MotorC Yaw;
MotorC Pitch;
MotorC Rammc;

extern RC_ctrl_t rc_ctrl;

void Shoot_Mode_Command(void)
{
    Shoot_mode shootmode ;
    shootmode = (Shoot_mode)rc_ctrl.rc.s[0];
    switch (shootmode)
    {
        case expression:
    }
}

void Shoot_Speed_Command(void)
{

}

void Shoot_Command(void)
{

}


void Gimbal_loop(void)
{

}