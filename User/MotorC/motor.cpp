//
// Created by Snoopy on 2024/9/21.
//

#include "motor.h"

PIDc FricL_PID;
PIDc FricR_PID;
PIDc Yaw_PID;
PIDc Pitch_PID;
PIDc Rammc_PID;

extern float Algo_Yaw_Data;
extern float Algo_Pitch_Data;
extern RC_ctrl_t rc_ctrl;

void FricL_PID_Init(void)
{
    FricL_PID.PID_Init(&FricL_PID.SpdParam);
    FricL_PID.PID_Init(&FricL_PID.PosParam);

    FricL_PID.SpdParam.Kp1 = 9.0f;
    FricL_PID.SpdParam.Ki1 = 0.02f;
    FricL_PID.SpdParam.Kd1 = 1.0f;

    FricL_PID.PosParam.Kp1 = 3.0f;
    FricL_PID.PosParam.Ki1 = 0.0f;
    FricL_PID.PosParam.Kd1 = 0.0f;


}

void FricR_PID_Init(void)
{
    FricR_PID.PID_Init(&FricR_PID.SpdParam);
    FricR_PID.PID_Init(&FricR_PID.PosParam);

    FricR_PID.SpdParam.Kp1 = 9.0f;
    FricR_PID.SpdParam.Ki1 = 0.02f;
    FricR_PID.SpdParam.Kd1 = 1.0f;

    FricR_PID.PosParam.Kp1 = 3.0f;
    FricR_PID.PosParam.Ki1 = 0.0f;
    FricR_PID.PosParam.Kd1 = 0.0f;

}

void Rammc_PID_Init(void)
{
    Rammc_PID.PID_Init(&Rammc_PID.SpdParam);
    Rammc_PID.PID_Init(&Rammc_PID.PosParam);

    Rammc_PID.SpdParam.Kp1 = 2.0f;
    Rammc_PID.SpdParam.Ki1 = 0.1f;
    Rammc_PID.SpdParam.Kd1 = 5.0f;

    Rammc_PID.PosParam.Kp1 = 2.0f;
    Rammc_PID.PosParam.Ki1 = 0.0f;
    Rammc_PID.PosParam.Kd1 = 0.0f;

}

void Pitch_PID_Init(void)
{
    Pitch_PID.PID_Init(&Pitch_PID.SpdParam);
    Pitch_PID.PID_Init(&Pitch_PID.PosParam);

    Pitch_PID.SpdParam.Kp1 = 0.1f;
    Pitch_PID.SpdParam.Ki1 = 0.01f;
    Pitch_PID.SpdParam.Kd1 = 0.1f;
    Pitch_PID.SpdParam.PID_ErrAllMax = 80000;

    Pitch_PID.PosParam.Kp1 = 1.0f;
    Pitch_PID.PosParam.Ki1 = 0.0f;
    Pitch_PID.PosParam.Kd1 = 0.0f;

}

void Yaw_PID_Init(void)
{
    Yaw_PID.PID_Init(&Yaw_PID.SpdParam);
    Yaw_PID.PID_Init(&Yaw_PID.PosParam);

    Yaw_PID.SpdParam.Kp1 = 90.0f;
    Yaw_PID.SpdParam.Ki1 = 2.5f;
    Yaw_PID.SpdParam.Kd1 = 70.0f;
    Yaw_PID.SpdParam.PID_ErrAllMax = 80000;
    Yaw_PID.SpdParam.PID_OutStep = 700;
    Yaw_PID.SpdParam.PID_Vari_Spd_Min = 50;
    Yaw_PID.SpdParam.PID_Vari_Spd_Max = 150;

    Yaw_PID.PosParam.Kp1 = 1.0f;
    Yaw_PID.PosParam.Ki1 = 0.0f;
    Yaw_PID.PosParam.Kd1 = 0.1f;

}

float GM6020_Angle_limit(float input_data,float Angle_Set,float Middle_Angle,float Angle_now)     //6020电机角度限位
{
    float result;
    float reserved_middle_Angle;
    float Angle_min = Middle_Angle - Angle_Set/360 * 8191;
    float Angle_max = Middle_Angle + Angle_Set/360 * 8191;
    if (Middle_Angle > 4095.5)
    {
        reserved_middle_Angle = Middle_Angle - 4095.5;    //reserved_middle_Angle 和 Middle_Yaw_Angle 处在同一直线上
    }
    else
    {
        reserved_middle_Angle = 4095.5 + Middle_Angle;
    }

    if (Angle_max > 8191)   //角度上限溢出
    {
        Angle_max -= 8191;
        if ((Angle_min < Angle_now && Angle_now < 8191) || (0 < Angle_now && Angle_now < (Angle_max - 8191)))  //在角度限位中
        {
            result = input_data;
        }
        else
        {
            if ((reserved_middle_Angle < Angle_now && Angle_now < Angle_min) && input_data < 0)    //角度小于下限，且在对称轴偏向Angle_min一侧，且角度值继续减小
            {
                result = 0;
            }
            else if (Angle_max < Angle_now && Angle_now < reserved_middle_Angle && input_data > 0) //角度大于下限，且在对称轴偏向Angle_max一侧，且角度值继续增大
            {
                result = 0;
            }
            else
            {
                result = input_data;
            }
        }
    }
    else if (Angle_min < 0)     //角度下限溢出
    {
        Angle_min += 8191;
        if ((0 < Angle_now && Angle_now < Angle_max)||((Angle_min + 8191) < Angle_now && Angle_now < 8191))
        {
            result = input_data;
        }
        else
        {
            if ((reserved_middle_Angle < Angle_now && Angle_now < Angle_min) && input_data < 0)    //角度小于下限，且在对称轴偏向Angle_min一侧，且角度值继续减小
            {
                result = 0;
            }
            else if (Angle_max < Angle_now && Angle_now < reserved_middle_Angle && input_data > 0) //角度大于下限，且在对称轴偏向Angle_max一侧，且角度值继续增大
            {
                result = 0;
            }
            else
            {
                result = input_data;
            }
        }
    }
    else
    {
        if (Angle_min < Angle_now && Angle_now < Angle_max)
        {
            result = input_data;
        }
        else
        {
            if (Middle_Angle > 4095.5)    //在圆弧较大一侧
            {
                if (((Angle_max < Angle_now && Angle_now < 8191) || (0 < Angle_now && Angle_now < reserved_middle_Angle)) && input_data > 0)
                {
                    result = 0;
                }
                else if (reserved_middle_Angle < Angle_now && Angle_now < Angle_min && input_data < 0)
                {
                    result = 0;
                }
                else
                {
                    result = input_data;
                }
            }
            else      //在圆弧较小一侧
            {
                if (((0 < Angle_now && Angle_now < Angle_min) ||(reserved_middle_Angle < Angle_now && Angle_now < 8191)) && input_data < 0)
                {
                    result = 0;
                }
                else if (Angle_max < Angle_now && Angle_now < reserved_middle_Angle && input_data > 0)
                {
                    result = 0;
                }
                else
                {
                    result = input_data;
                }
            }
        }
    }
    if (result == 0)    //清零积分项
    {
        Yaw_PID.SpdParam.PID_Err_all = 0;
        Yaw_PID.PosParam.PID_Err_all = 0;
    }
    return result;
}

float DM4310_Angle_limit(float input_data,float Angle_Set,float Middle_Angle,float Angle_now)     //6020电机角度限位
{
    float result;
    float reserved_middle_Angle;
    float Angle_min = Middle_Angle - Angle_Set;
    float Angle_max = Middle_Angle + Angle_Set;
    Angle_now += 180;
    if (Middle_Angle > 180)
    {
        reserved_middle_Angle = Middle_Angle - 180;    //reserved_middle_Angle 和 Middle_Yaw_Angle 处在同一直线上
    }
    else
    {
        reserved_middle_Angle = 180 + Middle_Angle;
    }

    if (Angle_max > 360)   //角度上限溢出
    {
        Angle_max -= 360;
        if ((Angle_min < Angle_now && Angle_now < 360) || (0 < Angle_now && Angle_now < (Angle_max - 360)))  //在角度限位中
        {
            result = input_data;
        }
        else
        {
            if ((reserved_middle_Angle < Angle_now && Angle_now < Angle_min) && input_data < 0)    //角度小于下限，且在对称轴偏向Angle_min一侧，且角度值继续减小
            {
                result = 0;
            }
            else if (Angle_max < Angle_now && Angle_now < reserved_middle_Angle && input_data > 0) //角度大于下限，且在对称轴偏向Angle_max一侧，且角度值继续增大
            {
                result = 0;
            }
            else
            {
                result = input_data;
            }
        }
    }
    else if (Angle_min < 0)     //角度下限溢出
    {
        Angle_min += 360;
        if ((0 < Angle_now && Angle_now < Angle_max)||((Angle_min + 360) < Angle_now && Angle_now < 360))
        {
            result = input_data;
        }
        else
        {
            if ((reserved_middle_Angle < Angle_now && Angle_now < Angle_min) && input_data < 0)    //角度小于下限，且在对称轴偏向Angle_min一侧，且角度值继续减小
            {
                result = 0;
            }
            else if (Angle_max < Angle_now && Angle_now < reserved_middle_Angle && input_data > 0) //角度大于下限，且在对称轴偏向Angle_max一侧，且角度值继续增大
            {
                result = 0;
            }
            else
            {
                result = input_data;
            }
        }
    }
    else
    {
        if (Angle_min < Angle_now && Angle_now < Angle_max)
        {
            result = input_data;
        }
        else
        {
            if (Middle_Angle > 180)    //在圆弧较大一侧
            {
                if (((Angle_max < Angle_now && Angle_now < 360) || (0 < Angle_now && Angle_now < reserved_middle_Angle)) && input_data > 0)
                {
                    result = 0;
                }
                else if (reserved_middle_Angle < Angle_now && Angle_now < Angle_min && input_data < 0)
                {
                    result = 0;
                }
                else
                {
                    result = input_data;
                }
            }
            else      //在圆弧较小一侧
            {
                if (((0 < Angle_now && Angle_now < Angle_min) ||(reserved_middle_Angle < Angle_now && Angle_now < 8191)) && input_data < 0)
                {
                    result = 0;
                }
                else if (Angle_max < Angle_now && Angle_now < reserved_middle_Angle && input_data > 0)
                {
                    result = 0;
                }
                else
                {
                    result = input_data;
                }
            }
        }
    }
    if (result == 0)    //清零积分项
    {
        Pitch_PID.SpdParam.PID_Err_all = 0;
        Pitch_PID.PosParam.PID_Err_all = 0;
    }
    return result;
}



