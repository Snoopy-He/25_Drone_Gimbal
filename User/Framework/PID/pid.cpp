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
    WhichPID->Kp1 = 20.0;
    WhichPID->Ki1 = 1;
    WhichPID->Kd1 = 0;
    WhichPID->PID_Err_now = 0.0;
    WhichPID->PID_Err_last = 0.0;
    WhichPID->PID_Err_lastlast = 0.0;
    WhichPID->PID_Err_all = 0.0;
    WhichPID->PID_Out = 0.0;
    WhichPID->PID_lastout = 0.0;
    WhichPID->PID_Target = 0.0;
    WhichPID->PID_Input = 0.0;
    WhichPID->PID_WorkType = 0;

    WhichPID->PID_Precision = PID_DEFAULT_PRECISION;
    WhichPID->PID_ErrAllMax = PID_DEFAULT_ERRALL_MAX;
    WhichPID->PID_OutMax = PID_DEFAULT_OUTPUT_MAX;
    WhichPID->PID_OutStep = PID_DEFAULT_OUTPUT_STEP_MAX;
}

/*
	* @name   PID_Update
	* @brief  更新PID的数据，即更新PID的输入值
	* @param  WhichPID PID结构体指针,NowInput 当前PID输入值
  	* @retval None
*/
void PIDc::PID_Update(PID_t *WhichPID,float NowInput)
{
    WhichPID->PID_Input = NowInput;
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
float PIDc::IncrementPID(PID_t *WhichPID)
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