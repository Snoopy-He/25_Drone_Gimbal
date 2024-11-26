//
// Created by 29358 on 2024/9/23.
//

#include "Printf_Task.h"
#include "remote_bsp.h"


extern Rx_Data FricL_Data;
extern Rx_Data FricR_Data;
extern Rx_Data YawMotor_Data;
extern Rx_DM_Data PitchMotor_Data;
extern int16_t can2_send[4];
extern PIDc FricL_PID;
extern PIDc FricR_PID;
extern PIDc Yaw_PID;
extern RC_ctrl_t rc_ctrl;
extern char RxData1[MESSAGE_LENGTH];
extern float PID_Data[6];
extern IMU_data C_IMU_Data;
extern IMU_Data_t BMI088;
extern gimbal_angle Pitch_Data;
extern gimbal_angle Yaw_Data;


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
        //usart_printf("%d,%d\r\n",FricL_Data.Speed,FricR_Data.Speed);
        //usart_printf("%f,%f\r\n",FricL_PID.SpdParam.Kp1);
        //usart_printf("%f\r\n",PID_Data[0]);
        //usart_printf("%f\r\n",RxData1[2]);
        //usart_printf("%f\r\n",((float)RxData1[2]-48) + ((float)RxData1[4]-48)/10 + ((float)RxData1[5]-48)/100);
        //usart_printf("%f\r\n",(float)RxData1[2]-48);
        //usart_printf("%d\r\n",can2_send[0]);
        //usart_printf("%f\r\n",Yaw_PID.SpdParam.Kp1);
        //usart_printf("%d,%d\r\n",Rammc_Data.Speed,rc_ctrl.rc.ch[0] * 8);
        //usart_printf("%d,%d,%f\r\n",YawMotor_Data.Angle,YawMotor_Data.Speed,(float)rc_ctrl.rc.ch[2]/100);
        //usart_printf("%f,%f\r\n",YawMotor_Data.Angle-4095.5,Yaw_PID.SpdParam.PID_Err_all);
        //usart_printf("%f,%f,%f\r\n",PitchMotor_Data.Speed,PitchMotor_Data.Angle,PitchMotor_Data.Torque);
        //usart_printf("%f\r\n",(float)rc_ctrl.rc.ch[2] / 10);
        //usart_printf("%f,%f\r\n",PitchMotor_Data.Angle + 180,Algo_Pitch_Data);
        //usart_printf("%f,%f\r\n",(float)PitchMotor_Data.Speed/3.1415926/2*60,(float)rc_ctrl.rc.ch[3]/100);
        //usart_printf("%f,%f\r\n",(float)rc_ctrl.rc.ch[3] / 20,(float)PitchMotor_Data.Speed*30/3.1415926);
        //usart_printf("%f\r\n",C_IMU_Data.Speed.Roll);
        usart_printf("%f,%f,%f\r\n",C_IMU_Data.Angle.Yaw,Yaw_Data.Target,Yaw_Data.Algo_Data);
        //usart_printf("%f,%f\r\n",C_IMU_Data.Angle.Roll,Pitch_Data.Target);
        //usart_printf("%.3f,%.3f\r\n",C_IMU_Data.Speed.Roll,(float)rc_ctrl.rc.ch[3] / 10);
        //usart_printf("%f,%f,%f,%f,%f,%f\r\n",BMI088.Gyro[0],BMI088.Gyro[1],BMI088.Gyro[2],BMI088.Accel[0],BMI088.Accel[1],BMI088.Accel[2]);
        vTaskDelayUntil(&CurrentTime, 2 / portTICK_RATE_MS);
    }
    /* USER CODE END Printf_Task */
}
