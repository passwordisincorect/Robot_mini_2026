#ifndef HWT901B_BRIDGE_H
#define HWT901B_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

HAL_StatusTypeDef HWT901B_Bridge_Init(UART_HandleTypeDef *sensorUart,
                                      UART_HandleTypeDef *megaUart);
void HWT901B_Bridge_Task(void);
void HWT901B_Bridge_RxCpltCallback(UART_HandleTypeDef *uart);
void HWT901B_Bridge_ErrorCallback(UART_HandleTypeDef *uart);

#ifdef __cplusplus
}
#endif

#endif
