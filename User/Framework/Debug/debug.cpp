//
// Created by Snoopy on 2024/9/22.
//

#include "debug.h"

char RxBuffer1[MESSAGE_LENGTH] = {0};    //原始数据缓存
char RxData1[MESSAGE_LENGTH] = {0};     //数据转存
float PID_Data[6];

void Debug_Init(void)
{
    __HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart6, (uint8_t *)&RxBuffer1,RECEIVE_SIZE);

}

void LED_ON(void)
{
    HAL_GPIO_WritePin(GPIOH,GPIO_PIN_10,GPIO_PIN_SET);
}

void LED_OFF(void)
{
    HAL_GPIO_WritePin(GPIOH,GPIO_PIN_10,GPIO_PIN_RESET);
}

#define TX_BUF_SIZE 512
static uint8_t send_buf[TX_BUF_SIZE];
void usart_printf(const char *format, ...)
{
    va_list args;
    uint32_t length;

    va_start(args, format);
    length = vsnprintf((char *)send_buf, TX_BUF_SIZE, (const char *)format, args);
    va_end(args);
    HAL_UART_Transmit(&huart6, (uint8_t *)send_buf, length,0xffff);
}

void Data_Operation(void)
{
    float data;
    data = ((float)RxData1[2]-48) + ((float)RxData1[4]-48)/10 + ((float)RxData1[5]-48)/100;  //-48为ascii码
    switch ((int8_t)RxData1[0])   //包头首位
    {
        case '1' :
            PID_Data[0] = data;   //数据位
            break;
        case '2' :
            PID_Data[1] = data;   //数据位
            break;
        case '3' :
            PID_Data[2] = data;   //数据位
            break;
        case '4' :
            PID_Data[3] = data;   //数据位
            break;
        case '5' :
            PID_Data[4] = data;   //数据位
            break;
        case '6' :
            PID_Data[5] = data;   //数据位
            break;

    }
}

void Debug_IrqHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE) != RESET)       //串口空闲中断
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart6);
        HAL_UART_DMAStop(&huart6);
        memcpy(RxData1,RxBuffer1,10);
        Data_Operation();
        memset(RxBuffer1, 0, sizeof(RxBuffer1));
        HAL_UART_Receive_DMA(&huart6, (uint8_t *)&RxBuffer1, RECEIVE_SIZE);
    }
}

void PID_Debug_Set(PID_t *SpdParam,PID_t *PosParam)
{
    SpdParam->Kp1 = PID_Data[0];
    SpdParam->Ki1 = PID_Data[1];
    SpdParam->Kd1 = PID_Data[2];
    PosParam->Kp1 = PID_Data[3];
    PosParam->Ki1 = PID_Data[4];
    PosParam->Kd1 = PID_Data[5];
}
