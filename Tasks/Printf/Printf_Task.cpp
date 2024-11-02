//
// Created by 29358 on 2024/9/23.
//

#include "Printf_Task.h"
#include "remote_bsp.h"

extern Rx_Data FricL_Data;
extern Rx_Data FricR_Data;
extern Rx_DM_Data PitchMotor_Data;
extern int16_t can2_send[4];
extern PIDc FricL_PID;
extern PIDc FricR_PID;
extern PIDc Yaw_PID;
extern RC_ctrl_t rc_ctrl;
extern char RxData1[MESSAGE_LENGTH];
extern float PID_Data[6];

void Printf_Task(void const * argument)
{
    /* USER CODE BEGIN Printf_Task */
    /* Infinite loop */
    portTickType CurrentTime;
    for(;;)
    {
        CurrentTime = xTaskGetTickCount();
        //usart_printf("%c%c%c%c\r\n",RxData1[2],RxData1[3],RxData1[4],RxData1[5]);
        //usart_printf("%d,%d,%d,%d\r\n",rc_ctrl.rc.ch[0],rc_ctrl.rc.ch[1],rc_ctrl.rc.ch[2],rc_ctrl.rc.ch[3]);
        usart_printf("%d,%d\r\n",FricL_Data.Speed,FricR_Data.Speed);
        //usart_printf("%f,%f\r\n",FricL_PID.SpdParam.Kp1);
        //usart_printf("%f\r\n",PID_Data[0]);
        //usart_printf("%f\r\n",RxData1[2]);
        //usart_printf("%f\r\n",((float)RxData1[2]-48) + ((float)RxData1[4]-48)/10 + ((float)RxData1[5]-48)/100);
        //usart_printf("%f\r\n",(float)RxData1[2]-48);
        //usart_printf("%d\r\n",can_send[3]);
        //usart_printf("%f\r\n",Yaw_PID.SpdParam.Kp1);
        //usart_printf("%d,%d\r\n",-Rammc_Data.Speed,FricL_Data.Speed);
        //usart_printf("%f,%d\r\n",YawMotor_Data.Angle-4095.5,-rc_ctrl.rc.ch[2] * 4);
        //usart_printf("%f,%f\r\n",YawMotor_Data.Angle-4095.5,Yaw_PID.SpdParam.PID_Err_all);
        //usart_printf("%f,%f,%f\r\n",PitchMotor_Data.Speed,PitchMotor_Data.Angle,PitchMotor_Data.Torque);
        vTaskDelayUntil(&CurrentTime, 2 / portTICK_RATE_MS);
    }
    /* USER CODE END Printf_Task */
}
