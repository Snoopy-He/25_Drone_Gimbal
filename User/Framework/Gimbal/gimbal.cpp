//
// Created by Snoopy on 2024/9/24.
//

#include "gimbal.h"

MotorC FricL;
MotorC FricR;
MotorC Yaw;
MotorC Pitch;
MotorC Rammc;

extern PIDc FricL_PID;
extern PIDc FricR_PID;
extern PIDc Yaw_PID;
extern PIDc Pitch_PID;
extern PIDc Rammc_PID;

extern Rx_Data FricL_Data;
extern Rx_Data FricR_Data;
extern Rx_Data Rammc_Data;
extern Rx_Data YawMotor_Data;
extern Rx_Data PitchMotor_Data;

int16_t can_send[4];


extern RC_ctrl_t rc_ctrl;

void Algorithm_Init(void)
{
    FricL_PID_Init();
    FricR_PID_Init();
    Rammc_PID_Init();

}


void Shoot_Mode_Command(void)
{
    Shoot_mode shootmode ;
    shootmode = (Shoot_mode)rc_ctrl.rc.s[0];

}

void Shoot_Speed_Command(void)
{

}

void Shoot_Command(void)
{

}

void Get_CtrlData(void)
{
    Shoot_Speed_Command();
    Shoot_Command();
}

void Algorithm_run(void)
{
    //PID_Debug_Set(&FricL_PID.SpdParam,&FricL_PID.PosParam);`
    //PID_Debug_Set(&FricR_PID.SpdParam,&FricR_PID.PosParam);
    //PID_Debug_Set(&Rammc_PID.SpdParam,&Rammc_PID.PosParam);
    FricL_PID.PID_Update(&FricL_PID.SpdParam,FricL_Data.Speed,(int16_t)(rc_ctrl.rc.ch[0] * 7));
    FricR_PID.PID_Update(&FricR_PID.SpdParam,FricR_Data.Speed,(int16_t)(-(rc_ctrl.rc.ch[0] * 7)));
    Rammc_PID.PID_Update(&Rammc_PID.SpdParam,Rammc_Data.Speed,(int16_t)(rc_ctrl.rc.ch[2] * 8));

    can_send[2] = Rammc_PID.Double_Param_PID(&Rammc_PID.SpdParam,&Rammc_PID.PosParam);
    can_send[1] = FricL_PID.Double_Param_PID(&FricL_PID.SpdParam,&FricL_PID.PosParam);
    can_send[0] = FricR_PID.Double_Param_PID(&FricR_PID.SpdParam,&FricR_PID.PosParam);
}

void Motor_Command_Send(void)
{
    Can_Send(SHOOT_ID,can_send[0],can_send[1],can_send[2],can_send[3]);
}

void Gimbal_loop(void)
{
    Get_CtrlData();
    Algorithm_run();
    Motor_Command_Send();

}