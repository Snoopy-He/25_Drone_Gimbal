//
// Created by Snoopy on 2024/9/21.
//

#include "can_bsp.h"

int16_t Can_Tx_Data[5];
Rx_Data FricL_Data;
Rx_Data FricR_Data;
Rx_Data Rammc_Data;
Rx_Data YawMotor_Data;
Rx_Data PitchMotor_Data;

void Can_Init(void)
{
    __HAL_RCC_CAN1_CLK_ENABLE();     //开启can1的时钟
    HAL_CAN_Start(&hcan1);     //开启can1
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);  //开启中断

//    __HAL_RCC_CAN2_CLK_ENABLE();     //开启can2的时钟
//    HAL_CAN_Start(&hcan2);     //开启can2
//    HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);  //开启中断
}



void Can_Filter_Init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterActivation = ENABLE;
//    can_filter_st.FilterBank =
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);

    can_filter_st.FilterBank = 14;
    can_filter_st.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
}

void Can_Send(int16_t ID,int16_t Mess_1,int16_t Mess_2,int16_t Mess_3,int16_t Mess_4)
{
    uint32_t Send_Mail_Box = 0;
    CAN_TxHeaderTypeDef Chassis_Tx_Message;
    uint8_t chassis_can_send_message[8];

    Chassis_Tx_Message.DLC = 0x08;
    Chassis_Tx_Message.IDE = CAN_ID_STD;
    Chassis_Tx_Message.StdId = ID;
    Chassis_Tx_Message.RTR = CAN_RTR_DATA;

    chassis_can_send_message[0] = Mess_1 >> 8;
    chassis_can_send_message[1] = Mess_1;
    chassis_can_send_message[2] = Mess_2 >> 8;
    chassis_can_send_message[3] = Mess_2;
    chassis_can_send_message[4] = Mess_3 >> 8;
    chassis_can_send_message[5] = Mess_3;
    chassis_can_send_message[6] = Mess_4 >> 8;
    chassis_can_send_message[7] = Mess_4;

    HAL_CAN_AddTxMessage(&hcan1,&Chassis_Tx_Message,chassis_can_send_message,&Send_Mail_Box);
    //HAL_CAN_AddTxMessage(&hcan2,&Chassis_Tx_Message,chassis_can_send_message,&Send_Mail_Box);

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_buf[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_buf);
    int16_t ID = rx_header.StdId;

    switch(ID)   //检测不同ID号，适应不同电机的数据
    {
        case L_FRIC_ID:
        {
            FricL_Data.Angle = rx_buf[0] << 8 | rx_buf[1];
            FricL_Data.Speed = rx_buf[2] << 8 | rx_buf[3];
            FricL_Data.Torque = rx_buf[4] << 8 | rx_buf[5];
            FricL_Data.Temperature = rx_buf[6];
            usart_printf("%X,%d,%d,%d\r\n", ID,FricL_Data.Angle,FricL_Data.Speed,FricL_Data.Torque,FricL_Data.Temperature);
            break;
        }

        case R_FRIC_ID:
        {
            FricR_Data.Angle = rx_buf[0] << 8 | rx_buf[1];
            FricR_Data.Speed = rx_buf[2] << 8 | rx_buf[3];
            FricR_Data.Torque = rx_buf[4] << 8 | rx_buf[5];
            FricR_Data.Temperature = rx_buf[6];
            usart_printf("%X,%d,%d,%d\r\n", ID,FricR_Data.Angle,FricR_Data.Speed,FricR_Data.Torque,FricR_Data.Temperature);
            break;
        }

        case RAMMC_ID:
        {
            Rammc_Data.Angle = rx_buf[0] << 8 | rx_buf[1];
            Rammc_Data.Speed = rx_buf[2] << 8 | rx_buf[3];
            Rammc_Data.Torque = rx_buf[4] << 8 | rx_buf[5];
            Rammc_Data.Temperature = 0;    //无温度值反馈
            usart_printf("%X,%d,%d,%d\r\n", ID,Rammc_Data.Angle,Rammc_Data.Speed,Rammc_Data.Torque,Rammc_Data.Temperature);
            break;
        }

        case PITCH_MOTOR_ID:
        {
            PitchMotor_Data.Angle = rx_buf[0] << 8 | rx_buf[1];
            PitchMotor_Data.Speed = rx_buf[2] << 8 | rx_buf[3];
            PitchMotor_Data.Torque = rx_buf[4] << 8 | rx_buf[5];
            PitchMotor_Data.Temperature = rx_buf[6];
            usart_printf("%X,%d,%d,%d\r\n", ID,PitchMotor_Data.Angle,PitchMotor_Data.Speed,PitchMotor_Data.Torque,PitchMotor_Data.Temperature);
            break;
        }

        case YAW_MOTOR_ID:
        {
            YawMotor_Data.Angle = rx_buf[0] << 8 | rx_buf[1];
            YawMotor_Data.Speed = rx_buf[2] << 8 | rx_buf[3];
            YawMotor_Data.Torque = rx_buf[4] << 8 | rx_buf[5];
            YawMotor_Data.Temperature = rx_buf[6];
            usart_printf("%X,%d,%d,%d\r\n", ID,YawMotor_Data.Angle,YawMotor_Data.Speed,YawMotor_Data.Torque,YawMotor_Data.Temperature);
            break;
        }

        default:
            usart_printf("Error\r\n");
            break;
    }
}