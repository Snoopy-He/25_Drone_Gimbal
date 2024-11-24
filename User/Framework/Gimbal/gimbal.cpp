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
extern gimbal_angle Pitch_Data;
extern gimbal_angle Yaw_Data;
extern RC_ctrl_t rc_ctrl;
extern IMU_data C_IMU_Data;



void Middle_Angle_Set(float Yaw,float pitch)
{
    Yaw_Data.Middle_Angle = Yaw;
    Pitch_Data.Middle_Angle = pitch;
    Pitch_Data.Middle_Angle = Pitch_Data.Middle_Angle + 180;
    //Yaw_PID.PID_Update(&Yaw_PID.SpdParam,YawMotor_Data.Angle,  Yaw);

}

float Yaw_Angle_limit(float input_data,float Angle_Set,float Angle_now)   //yaw轴限位
{
    float result;
    result = GM6020_Angle_limit(input_data,Angle_Set,Yaw_Data.Middle_Angle,Angle_now);
    return result;
}



float Pitch_Angle_limit(float input_data,float Angle_Set,float Angle_now)   //pitch轴限位
{
    float result;
    result = DM4310_Angle_limit(input_data,Angle_Set,Pitch_Data.Middle_Angle,Angle_now);
    return result;
}


void Algorithm_Init(void)
{
    FricL_PID_Init();
    FricR_PID_Init();
    Rammc_PID_Init();
    Yaw_PID_Init();
    Pitch_PID_Init();
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
    //Gimbal_Data.Corrected_Yaw_Angle = DJI_Motor_Angle_Correction(YawMotor_Data.Angle,Gimbal_Data.Middle_Yaw_Angle);
    //Gimbal_Data.Corrected_Pitch_Angle = DM_Motor_Angle_Correction(PitchMotor_Data.Angle,Gimbal_Data.Middle_Pitch_Angle);
}

void Gimbal_Target_Set(void)
{
    Pitch_Target_Set();
    Yaw_Target_Set();
}

void Algorithm_run(void)
{
    //PID_Debug_Set(&FricL_PID.SpdParam,&FricL_PID.PosParam);
    //PID_Debug_Set(&FricR_PID.SpdParam,&FricR_PID.PosParam);
    //PID_Debug_Set(&Rammc_PID.SpdParam,&Rammc_PID.PosParam);
    //PID_Debug_Set(&Yaw_PID.SpdParam,&Yaw_PID.PosParam);
    //PID_Debug_Set(&Pitch_PID.SpdParam,&Pitch_PID.PosParam);
    //Yaw_PID.PID_Update(&Yaw_PID.SpdParam,YawMotor_Data.Speed,  (float)rc_ctrl.rc.ch[2] / 100);
    //Pitch_PID.PID_Update(&Pitch_PID.SpdParam,C_IMU_Data.Speed.Roll,  (float)rc_ctrl.rc.ch[3] / 10);
    //FricL_PID.PID_Update(&FricL_PID.SpdParam,FricL_Data.Speed,(int16_t)(rc_ctrl.rc.ch[0] * 9));
    //FricR_PID.PID_Update(&FricR_PID.SpdParam,FricR_Data.Speed,(int16_t)(-(rc_ctrl.rc.ch[0] * 9)));
    //Rammc_PID.PID_Update(&Rammc_PID.SpdParam,Rammc_Data.Speed,(int16_t)(rc_ctrl.rc.ch[0] * 9));
    Yaw_Data.Algo_Data = Yaw_PID.Double_Param_Pos_PID(&Yaw_PID.SpdParam,&Yaw_PID.PosParam,C_IMU_Data.Angle.Yaw,Yaw_Data.Target,C_IMU_Data.Speed.Yaw * TIMpiece);
    Pitch_Data.Algo_Data = Pitch_PID.Double_Param_Pos_PID(&Pitch_PID.SpdParam,&Pitch_PID.PosParam,C_IMU_Data.Angle.Roll,Pitch_Data.Target,C_IMU_Data.Speed.Roll * TIMpiece);

    Yaw_Data.Algo_Data = Yaw_Angle_limit(Yaw_Data.Algo_Data,12,YawMotor_Data.Angle);
    Pitch_Data.Algo_Data = Pitch_Angle_limit(Pitch_Data.Algo_Data,30,PitchMotor_Data.Angle);

    can2_send[0] = (int16_t)Yaw_Data.Algo_Data;
    //can2_send[2] = (int16_t)Rammc_PID.Double_Param_Pos_PID(&Rammc_PID.SpdParam,&Rammc_PID.PosParam);
    //can2_send[1] = (int16_t)FricL_PID.Double_Param_Pos_PID(&FricL_PID.SpdParam,&FricL_PID.PosParam);
    //can2_send[0] = (int16_t)FricR_PID.Double_Param_Pos_PID(&FricR_PID.SpdParam,&FricR_PID.PosParam);
}

void Shoot_Command_Send(void)
{
    Can2_Send(SHOOT_ID,can2_send[0],can2_send[1],can2_send[2],can2_send[3]);
}

void Yaw_Command_Send(void)
{
    Can2_Send(YAW_ID,can2_send[0],can2_send[1],can2_send[2],can2_send[3]);
}

void Pitch_Command_Send(void)
{
    DM_Motor_Speed_Mode_Send(PITCH_ID,Pitch_Data.Algo_Data);
}

void Gimbal_loop(void)
{
    Get_CtrlData();
    C_IMU_Update();
    Gimbal_Target_Set();
    //Shoot_Command_Send();
    //DM_Motor_Speed_Mode_Send(PITCH_ID,0);
    //Can2_Send(SHOOT_ID,1000,1000,0,0);
    //Can1_Send(SHOOT_ID,1000,1000,0,0);
    Algorithm_run();
    Yaw_Command_Send();
    Pitch_Command_Send();
    //Can2_Send(0X2FE,1000,1000,1000,1000);
}