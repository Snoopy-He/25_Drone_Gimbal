//
// Created by Snoopy on 2024/9/21.
//

#include "motor.h"

PIDc FricL_PID;
PIDc FricR_PID;
PIDc Yaw_PID;
PIDc Pitch_PID;
PIDc Rammc_PID;

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
    Pitch_PID.PID_Init(&Rammc_PID.SpdParam);
    Pitch_PID.PID_Init(&Rammc_PID.PosParam);

}

void Yaw_PID_Init(void)
{
    Yaw_PID.PID_Init(&Yaw_PID.SpdParam);
    Yaw_PID.PID_Init(&Yaw_PID.PosParam);

    Yaw_PID.SpdParam.Kp1 = 1.56f;
    Yaw_PID.SpdParam.Ki1 = 0.0041f;
    Yaw_PID.SpdParam.Kd1 = 10.0f;
    Yaw_PID.SpdParam.PID_ErrAllMax = 100000;
    Yaw_PID.SpdParam.PID_OutStep = 700;
    Yaw_PID.SpdParam.PID_Vari_Spd_Min = 50;
    Yaw_PID.SpdParam.PID_Vari_Spd_Max = 150;

    Yaw_PID.PosParam.Kp1 = 1.45f;
    Yaw_PID.PosParam.Ki1 = 0.0f;
    Yaw_PID.PosParam.Kd1 = 0.0f;

}

