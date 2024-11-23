//
// Created by Snoopy on 2024/9/21.
//

#include "pid.h"

/*
	* @name   PID_SpeedParamInit
	* @brief  初始化PID速度环的参数
	* @param  WhichPID PID结构体指针
	* @retval None
*/
void PIDc::PID_Init(PID_t *WhichPID)
{ //初始化PID的默认参数
    WhichPID->Kp1 = 0.0f;
    WhichPID->Ki1 = 0.0f;
    WhichPID->Kd1 = 0.0f;
    WhichPID->PID_Err_now = 0.0f;
    WhichPID->PID_Err_last = 0.0f;
    WhichPID->PID_Err_lastlast = 0.0f;
    WhichPID->PID_Err_all = 0.0f;
    WhichPID->PID_Out = 0.0f;
    WhichPID->PID_lastout = 0.0f;
    WhichPID->PID_Target = 0.0f;
    WhichPID->PID_Input = 0.0f;
    WhichPID->PID_WorkType = 0.0f;

    WhichPID->PID_Precision = PID_DEFAULT_PRECISION;
    WhichPID->PID_ErrAllMax = PID_DEFAULT_ERRALL_MAX;
    WhichPID->PID_OutMax = PID_DEFAULT_OUTPUT_MAX;
    WhichPID->PID_OutStep = PID_DEFAULT_OUTPUT_STEP_MAX;

    WhichPID->PID_Vari_Spd_Min = 0.0f;
    WhichPID->PID_Vari_Spd_Max = 0.0f;
}

/*
	* @name   PID_Update
	* @brief  更新PID的数据，即更新PID的输入值
	* @param  WhichPID PID结构体指针,NowInput 当前PID输入值
  	* @retval None
*/
void PIDc::PID_Update(PID_t *WhichPID,float NowInput,float Target_Spd)
{
    WhichPID->PID_Input = NowInput;
    WhichPID->PID_Target = Target_Spd;
    WhichPID->PID_Err_lastlast = WhichPID->PID_Err_last;
    WhichPID->PID_Err_last = WhichPID->PID_Err_now;
    WhichPID->PID_Err_now = WhichPID->PID_Target - WhichPID->PID_Input;

    if (WhichPID->PID_Err_now < WhichPID->PID_Precision && WhichPID->PID_Err_now > -WhichPID->PID_Precision)
    {
        WhichPID->PID_Err_now = 0;
    }  //数值在PID最小精度之内

    WhichPID->PID_Err_all += WhichPID->PID_Err_now;

    if (WhichPID->PID_Err_all > WhichPID->PID_ErrAllMax)   //积分限幅
    {
        WhichPID->PID_Err_all = WhichPID->PID_ErrAllMax;
    }
    else if (WhichPID->PID_Err_all < -WhichPID->PID_ErrAllMax)
    {
        WhichPID->PID_Err_all = -WhichPID->PID_ErrAllMax;
    }

}

/*
	* @name   PID_PositionPID
	* @brief  位置式PID
	* @param  WhichPID PID结构体指针
    * @retval None
*/
float PIDc::PID_PositionPID(PID_t *WhichPID)
{

    WhichPID->PID_Out =
            WhichPID->Kp1 * WhichPID->PID_Err_now +
            WhichPID->Kd1 * (WhichPID->PID_Err_now - WhichPID->PID_Err_last);
    WhichPID->PID_Out += (WhichPID->PID_Err_all * WhichPID->Ki1);

    if (WhichPID->PID_Out >= WhichPID->PID_OutMax)   //PID输出限幅
        WhichPID->PID_Out = WhichPID->PID_OutMax;
    if (WhichPID->PID_Out <= -WhichPID->PID_OutMax)
        WhichPID->PID_Out = -WhichPID->PID_OutMax;

    if (WhichPID->PID_Out - WhichPID->PID_lastout > WhichPID->PID_OutStep)    //PID输出步长限制
    {
        WhichPID->PID_Out = WhichPID->PID_lastout + WhichPID->PID_OutStep;
    }
    if (WhichPID->PID_Out - WhichPID->PID_lastout < -WhichPID->PID_OutStep)
    {
        WhichPID->PID_Out = WhichPID->PID_lastout + -WhichPID->PID_OutStep;
    }

    WhichPID->PID_lastout = WhichPID->PID_Out;

    return WhichPID->PID_Out;
}


/*
	* @name   PID_IncrementPID
	* @brief  增量式PID
	* @param  WhichPID PID结构体指针
    * @retval None
*/
float PIDc::PID_IncrementPID(PID_t *WhichPID)
{
    WhichPID->PID_Out =
            WhichPID->Kp1 * (WhichPID->PID_Err_now - WhichPID->PID_Err_last) +
            WhichPID->Kd1 * (WhichPID->PID_Err_now - 2 * WhichPID->PID_Err_last + WhichPID->PID_Err_lastlast);
    WhichPID->PID_Out += (WhichPID->PID_Err_all * WhichPID->Ki1);
    if (WhichPID->PID_Out >= WhichPID->PID_OutMax)   //PID输出限幅
        WhichPID->PID_Out = WhichPID->PID_OutMax;
    if (WhichPID->PID_Out <= -WhichPID->PID_OutMax)
        WhichPID->PID_Out = -WhichPID->PID_OutMax;

    if (WhichPID->PID_Out - WhichPID->PID_lastout > WhichPID->PID_OutStep)    //PID输出步长限制
    {
        WhichPID->PID_Out = WhichPID->PID_lastout + WhichPID->PID_OutStep;
    }
    if (WhichPID->PID_Out - WhichPID->PID_lastout < -WhichPID->PID_OutStep)
    {
        WhichPID->PID_Out = WhichPID->PID_lastout + -WhichPID->PID_OutStep;
    }

    WhichPID->PID_lastout = WhichPID->PID_Out;

    return WhichPID->PID_Out;
}

/*
	* @name   Double_Param_PID
	* @brief  串级PID
	* @param  WhichPID PID结构体指针
    * @retval None
*/
float PIDc::Double_Param_Pos_PID(PID_t *SpdParam,PID_t *PosParam,float Pos_Target,float Spd_Target)
{
    SpdParam->PID_Target = Pos_Target;
    PID_Update(PosParam,SpdParam->PID_Target,SpdParam->PID_Out);
    PID_PositionPID(PosParam);
    PID_Update(SpdParam,SpdParam->PID_Target,SpdParam->PID_Out);  //Target如此赋值是为让位置环的Erroe_Now和input相等，Error_Now=Target - NowInput
    PID_PositionPID(SpdParam);

    return PosParam->PID_Out;
}

/*
	* @name   Variable_Speed_PID
	* @brief  变速积分PID
	* @param  WhichPID PID结构体指针
    * @retval None
*/
float PIDc::Variable_Speed_PID(PID_t *WhichPID)
{
    {
        float coefficient;  //变速积分PID中积分项加的系数
        if (abs(WhichPID->PID_Err_now) <= WhichPID->PID_Vari_Spd_Min)
        {
            coefficient = 1;
        }
        else if (WhichPID->PID_Vari_Spd_Min < abs(WhichPID->PID_Err_now) && abs(WhichPID->PID_Err_now) <= WhichPID->PID_Vari_Spd_Max + WhichPID->PID_Vari_Spd_Min)
        {
            coefficient = (WhichPID->PID_Vari_Spd_Max - abs(WhichPID->PID_Err_now) + WhichPID->PID_Vari_Spd_Min)/WhichPID->PID_Vari_Spd_Max;
        }
        else
        {
            coefficient = 0;
        }

        WhichPID->PID_Out =
                WhichPID->Kp1 * WhichPID->PID_Err_now +
                WhichPID->Kd1 * (WhichPID->PID_Err_now - WhichPID->PID_Err_last);
        WhichPID->PID_Out += (WhichPID->PID_Err_all + coefficient * WhichPID->PID_Err_now) * WhichPID->Ki1;

        if (WhichPID->PID_Out >= WhichPID->PID_OutMax)   //PID输出限幅
            WhichPID->PID_Out = WhichPID->PID_OutMax;
        if (WhichPID->PID_Out <= -WhichPID->PID_OutMax)
            WhichPID->PID_Out = -WhichPID->PID_OutMax;

        if (WhichPID->PID_Out - WhichPID->PID_lastout > WhichPID->PID_OutStep)    //PID输出步长限制
        {
            WhichPID->PID_Out = WhichPID->PID_lastout + WhichPID->PID_OutStep;
        }
        if (WhichPID->PID_Out - WhichPID->PID_lastout < -WhichPID->PID_OutStep)
        {
            WhichPID->PID_Out = WhichPID->PID_lastout + -WhichPID->PID_OutStep;
        }

        WhichPID->PID_lastout = WhichPID->PID_Out;

        return WhichPID->PID_Out;
    }
}

float PIDc::Double_Param_VSpd_PID(PID_t *SpdParam,PID_t *PosParam)
{
    Variable_Speed_PID(SpdParam);
    PID_Update(PosParam,SpdParam->PID_Out,2 * SpdParam->PID_Out);  //Target如此赋值是为让位置环的Erroe_Now和input相等，Error_Now=Target - NowInput
    PID_PositionPID(PosParam);

    return PosParam->PID_Out;
}