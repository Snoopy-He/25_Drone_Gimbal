//
// Created by Snoopy on 2024/9/21.
//

#include "remote_bsp.h"
#include "cstdlib"
#include "cstring"

void REMOTEIO_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
//    usart_printf("hh\r\n");
}

uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
RC_ctrl_t rc_ctrl;

static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
void RC_DataHandle(RC_ctrl_t *rc_ctrl);

void Remote_bsp_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度q
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
//                usart_printf("%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X\r\n",
//                             (int16_t)sbus_rx_buf[1][0],
//                             (int16_t)sbus_rx_buf[1][1],
//                             (int16_t)sbus_rx_buf[1][2],
//                             (int16_t)sbus_rx_buf[1][3],
//                             (int16_t)sbus_rx_buf[1][4],
//                             (int16_t)sbus_rx_buf[1][5],
//                             (int16_t)sbus_rx_buf[1][6],
//                             (int16_t)sbus_rx_buf[1][7],
//                             (int16_t)sbus_rx_buf[1][8],
//                             (int16_t)sbus_rx_buf[1][9],
//                             (int16_t)sbus_rx_buf[1][10],
//                             (int16_t)sbus_rx_buf[1][11],
//                             (int16_t)sbus_rx_buf[1][12],
//                             (int16_t)sbus_rx_buf[1][13],
//                             (int16_t)sbus_rx_buf[1][14],
//                             (int16_t)sbus_rx_buf[1][15],
//                             (int16_t)sbus_rx_buf[1][16],
//                             (int16_t)sbus_rx_buf[1][17]);
            }
        }
    }
}

static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
    RC_DataHandle(rc_ctrl);
    //usart_printf("%d, %d, %d, %d, %d, %d, %d\r\n", rc_ctrl->rc.ch[0], rc_ctrl->rc.ch[1], rc_ctrl->rc.ch[2], rc_ctrl->rc.ch[3], rc_ctrl->rc.ch[4],rc_ctrl->rc.s[0], rc_ctrl->rc.s[1]);
}

void RC_DataHandle(RC_ctrl_t *rc_ctrl)  //抑制零漂
{
    if (abs(rc_ctrl->rc.ch[0]) < 5)rc_ctrl->rc.ch[0] = 0;
    if (abs(rc_ctrl->rc.ch[1]) < 5)rc_ctrl->rc.ch[1] = 0;
    if (abs(rc_ctrl->rc.ch[2]) < 5)rc_ctrl->rc.ch[2] = 0;
    if (abs(rc_ctrl->rc.ch[3]) < 5)rc_ctrl->rc.ch[3] = 0;
    if (abs(rc_ctrl->rc.ch[4]) < 5)rc_ctrl->rc.ch[4] = 0;
    if (abs(rc_ctrl->rc.ch[0]) > 670 || abs(rc_ctrl->rc.ch[3]) > 670)
    {
        memset(rc_ctrl, 0, sizeof(RC_ctrl_t)); //异常值
    }
}