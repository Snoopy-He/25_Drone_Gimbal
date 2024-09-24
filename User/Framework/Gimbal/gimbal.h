//
// Created by Snoopy on 2024/9/24.
//

#ifndef INC_2024_GIMBAL_GIMBAL_H
#define INC_2024_GIMBAL_GIMBAL_H

#include "motor.h"
#include "remote_bsp.h"

//右侧开关决定射速，上为关->2，中为一挡射速->3，下为二挡射速->1
//左侧开关决定初速,上为关->1，中为一档初速->3，下为二档初速->2

typedef enum
{
    Shoot_OFF = 2,     //关闭
    Shoot_Mode_1 = 3,  //一档射速
    Shoot_Mode_2 = 1,  //二档射速
}Shoot_mode;

typedef enum
{
    Shoot_Speed_0 = 1,   //关闭
    Shoot_Speed_1 = 3,   //一档初速
    Shoot_Speed_2 = 2,   //二档初速
}Shoot_Speed;

extern RC_ctrl_t rc_ctrl;

void Gimbal_loop(void);
void Shoot_Mode_Command(void);
void Shoot_Speed_Command(void);
void Shoot_Command(void);

class GimbalC {

};


#endif //INC_2024_GIMBAL_GIMBAL_H
