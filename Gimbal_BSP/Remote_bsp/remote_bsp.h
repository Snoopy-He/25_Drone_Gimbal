//
// Created by 29358 on 2024/9/21.
//

#ifndef INC_2024_GIMBAL_REMOTE_BSP_H
#define INC_2024_GIMBAL_REMOTE_BSP_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f4xx_hal.h"
#include "usart.h"

void REMOTEIO_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

#ifdef __cplusplus
class remote_bsp {

};
#endif

#ifdef __cplusplus
}
#endif

#endif //INC_2024_GIMBAL_REMOTE_BSP_H
