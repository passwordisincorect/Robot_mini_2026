# HWT901B angle bridge for Blue Pill

This is a HAL module for an existing generated STM32F103C8T6 CubeMX project; it is not a standalone firmware image. It forwards only checksum-valid WIT angle frames (`55 53`, 11 bytes) unchanged. The Mega performs yaw PID.

## UART setup

- HWT901B TX → Blue Pill PA3 / USART2 RX; sensor UART baud must match the sensor output setting. Set USART2 to 8N1. This example assumes 115200 baud.
- Blue Pill PA9 / USART1 TX → Mega D15 / RX3. Set USART1 to 115200 baud, 8N1.
- Connect the device grounds. Verify the actual module supply and signal levels from its own label/datasheet before wiring; HWT901B variants differ.

In the generated project, copy `hwt901b_bridge.c` to `Core/Src` and its header to `Core/Inc`. Keep the generated `main.h` include path and configure PA3/USART2 and PA9/USART1 in CubeMX. In `main.c`, add:

```c
#include "hwt901b_bridge.h"
```

After `MX_USART1_UART_Init()` and `MX_USART2_UART_Init()` in `main()`, start reception and call the bridge task regularly from the main loop:

```c
if (HWT901B_Bridge_Init(&huart2, &huart1) != HAL_OK) {
  /* Report the UART startup error through the application's diagnostics. */
}
while (1) {
  HWT901B_Bridge_Task();
  /* Other existing nonblocking application tasks. */
}
```

Call the bridge helpers from the existing HAL callbacks (merge them into any callbacks the project already defines):

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  HWT901B_Bridge_RxCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  HWT901B_Bridge_ErrorCallback(huart);
}
```

Enable the generated USART2 NVIC interrupt in CubeMX. In `stm32f1xx_it.c`, keep the generated IRQ handler calling `HAL_UART_IRQHandler(&huart2)`; do not define a second `USART2_IRQHandler`. The application must continue calling `HWT901B_Bridge_Task()` after an initial RX start failure so its bounded retry can rearm USART2.

The interrupt only stores bytes and rearms one-byte reception. Frame parsing and the bounded USART1 transmit run from the main loop. Invalid, non-angle, timed-out, and overflowed input is discarded; the Mega sees no refreshed yaw and its 200 ms freshness check expires.
