#ifndef STM32_HWT_TEST_MAIN_H
#define STM32_HWT_TEST_MAIN_H

#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2 } HAL_StatusTypeDef;
typedef struct UART_HandleTypeDef {
  int id;
  uint8_t *rxData;
  uint16_t rxSize;
  HAL_StatusTypeDef receiveResult;
  HAL_StatusTypeDef transmitResult;
  unsigned receiveCalls;
  unsigned transmitCalls;
  uint32_t lastTimeout;
  uint8_t output[4096];
  uint16_t outputSize;
} UART_HandleTypeDef;

uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart,
                                      uint8_t *buffer, uint16_t size);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart,
                                    const uint8_t *buffer, uint16_t size,
                                    uint32_t timeout);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);
void __DMB(void);

#endif
