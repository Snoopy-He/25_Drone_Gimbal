//
// Created by Snoopy on 2024/9/21.
//

#ifndef INC_2024_GIMBAL_PID_H
#define INC_2024_GIMBAL_PID_H

#include "stm32f4xx_hal.h"
#include "usart.h"
#include <stdlib.h>

#define PID_DEFAULT_PRECISION 0.0f         //控制精度，当目标速度与实际速度小于此值时，认为没有误差，使PID更稳定
#define PID_DEFAULT_ERRALL_MAX 3000         //控制ERR_ALL最大值，否则ERR_ALL最大值过大，会使PID反应慢，不稳定，积分限幅
#define PID_DEFAULT_OUTPUT_MAX 10192     //输出限幅
#define PID_DEFAULT_OUTPUT_STEP_MAX 3192 //输出微分限幅

typedef enum
{
    IncrementPID = 0,
    PositionPID = 1,
} PID_Type;

typedef struct
{
    float Kp1;
    float Ki1;
    float Kd1;

    float PID_Err_now;
    float PID_Err_last;
    float PID_Err_lastlast;
    float PID_Err_all;

    float PID_Out;
    float PID_lastout;
    float PID_Target;      //PID的目标值
    float PID_lastTarget;
    float PID_Input;       //PID输入
    float PID_LastInput;

    uint8_t PID_WorkType;  // PID工作在位置式还是增量式
    float PID_Precision; // PID最小精度
    float PID_ErrAllMax; // PID积分限幅
    float PID_OutMax;     // PID输出限幅
    float PID_OutStep;     // PID输出步幅限制

    float PID_Vari_Spd_Max;    //变速积分误差上限
    float PID_Vari_Spd_Min;    //变速积分误差下限
} PID_t;


#ifdef __cplusplus
class PIDc {

public:
    void PID_Init(PID_t *WhichPID);
    void PID_Update(PID_t *WhichPID,float NowInput,float Target_Spd);
    float PID_PositionPID(PID_t *WhichPID);
    float PID_IncrementPID(PID_t *WhichPID);
    float Double_Param_Pos_PID(PID_t *SpdParam,PID_t *PosParam,float Pos_Input,float Pos_Target,float Spd_Target);
    float Variable_Speed_PID(PID_t *WhichPID);
    float Double_Param_VSpd_PID(PID_t *SpdParam,PID_t *PosParam);
    PID_t SpdParam;
    PID_t PosParam;

};
#endif

#endif //INC_2024_GIMBAL_PID_H
