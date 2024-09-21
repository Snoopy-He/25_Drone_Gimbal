//
// Created by Snoopy on 2024/9/21.
//

#include "remotec.h"

extern uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

void REMOTEC_Init(void)
{
    REMOTEIO_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

void REMOTEC_UartIrqHandler(void)
{
    Remote_bsp_IRQHandler();
}
