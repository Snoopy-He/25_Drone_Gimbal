//
// Created by Snoopy on 2024/9/22.
//

#ifndef INC_2024_GIMBAL_DEBUG_H
#define INC_2024_GIMBAL_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "pid.h"

#define RECEIVE_SIZE     255
#define MESSAGE_LENGTH   10

void Debug_Init(void);
void LED_ON(void);
void LED_OFF(void);
void usart_printf(const char *format, ...);
void Data_Operation(void);
void Debug_IrqHandler(void);
void PID_Debug_Set(PID_t *SpdParam,PID_t *PosParam);


#ifdef __cplusplus
class debug {

};
#endif

#ifdef __cplusplus
}
#endif

#endif //INC_2024_GIMBAL_DEBUG_H
