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
extern Rx_DM_Data PitchMotor_Data;

int16_t can2_send[4];


extern RC_ctrl_t rc_ctrl;

void Algorithm_Init(void)
{
    FricL_PID_Init();
    FricR_PID_Init();
    Rammc_PID_Init();
    Yaw_PID_Init();
}

void Motor_Init(void)
{
    DM_Motor_Enable(PITCH_ID);
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
    //PID_Debug_Set(&Yaw_PID.SpdParam,&Yaw_PID.PosParam);
    Yaw_PID.PID_Update(&Yaw_PID.SpdParam,((float)YawMotor_Data.Angle)-4095.5,-(int16_t)(rc_ctrl.rc.ch[2] * 4));
    FricL_PID.PID_Update(&FricL_PID.SpdParam,FricL_Data.Speed,(int16_t)(rc_ctrl.rc.ch[0] * 7));
    FricR_PID.PID_Update(&FricR_PID.SpdParam,FricR_Data.Speed,(int16_t)(-(rc_ctrl.rc.ch[0] * 7)));
    //Rammc_PID.PID_Update(&Rammc_PID.SpdParam,Rammc_Data.Speed,(int16_t)(rc_ctrl.rc.ch[2] * 8));

    can2_send[3] = (int16_t)Yaw_PID.Double_Param_Pos_PID(&Yaw_PID.SpdParam,&Yaw_PID.PosParam);
    can2_send[2] = (int16_t)Rammc_PID.Double_Param_Pos_PID(&Rammc_PID.SpdParam,&Rammc_PID.PosParam);
    can2_send[1] = (int16_t)FricL_PID.Double_Param_Pos_PID(&FricL_PID.SpdParam,&FricL_PID.PosParam);
    can2_send[0] = (int16_t)FricR_PID.Double_Param_Pos_PID(&FricR_PID.SpdParam,&FricR_PID.PosParam);
}

void Shoot_Command_Send(void)
{
    Can2_Send(SHOOT_ID,can2_send[0],can2_send[1],can2_send[2],can2_send[3]);
}

void Yaw_Command_Send(void)
{
    Can2_Send(GIMBAL_ID,can2_send[0],can2_send[1],can2_send[2],can2_send[3]);
}

void Pitch_Command_Send(void)
{
    //Can1_Send()
}

void Gimbal_loop(void)
{
    Get_CtrlData();
    Algorithm_run();
    //Shoot_Command_Send();
    DM_Motor_Speed_Mode_Send(PITCH_ID,3.1415926 * 2);
    Can2_Send(SHOOT_ID,1000,1000,0,0);
    //Shoot_Command_Send();
    //Yaw_Command_Send();

}