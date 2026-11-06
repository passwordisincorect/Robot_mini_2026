#include "hwt901b_bridge.h"
#include <string.h>

#define RX_CAPACITY 128U
#define FRAME_SIZE 11U
#define BYTE_TIMEOUT_MS 30U
#define TX_TIMEOUT_MS 3U
#define TASK_BUDGET 32U

static UART_HandleTypeDef *sensor;
static UART_HandleTypeDef *mega;
static uint8_t rxByte;
static volatile uint8_t rxBuffer[RX_CAPACITY];
static volatile uint32_t rxAt[RX_CAPACITY];
static volatile uint8_t head, tail, overflow, rxError, rxRestart;
static uint8_t frame[FRAME_SIZE], used;
static uint32_t byteAt;

static uint8_t angleType(uint8_t type) {
  return type == 0x53U;
}

static void resetParser(void) {
  used = 0;
  byteAt = HAL_GetTick();
}

static void push(uint8_t byte, uint32_t at) {
  uint8_t next = (uint8_t)((head + 1U) & (RX_CAPACITY - 1U));
  if (next == tail) {
    overflow = 1;
    return;
  }
  rxBuffer[head] = byte;
  rxAt[head] = at;
  __DMB();
  head = next;
}

static uint8_t pop(uint8_t *byte, uint32_t *at) {
  if (tail == head) return 0;
  *byte = rxBuffer[tail];
  *at = rxAt[tail];
  __DMB();
  tail = (uint8_t)((tail + 1U) & (RX_CAPACITY - 1U));
  return 1;
}

static void recover(void) {
  uint8_t start = 1;
  while (start < FRAME_SIZE && frame[start] != 0x55U) start++;
  used = (uint8_t)(FRAME_SIZE - start);
  if (used) memmove(frame, frame + start, used);
  if (used == 1U && frame[0] != 0x55U) used = 0;
}

static uint8_t ingest(uint8_t byte) {
  if (used == 0U) {
    if (byte == 0x55U) frame[used++] = byte;
    return 0;
  }
  if (used == 1U && (byte < 0x50U || byte > 0x5FU)) {
    if (byte == 0x55U) frame[0] = byte;
    else used = 0;
    return 0;
  }
  frame[used++] = byte;
  if (used < FRAME_SIZE) return 0;

  uint8_t sum = 0;
  for (uint8_t i = 0; i < FRAME_SIZE - 1U; i++) sum += frame[i];
  const uint8_t valid = (sum == frame[FRAME_SIZE - 1U]);
  const uint8_t isAngle = valid && angleType(frame[1]);
  if (!valid) recover();
  else used = 0;
  return isAngle;
}

HAL_StatusTypeDef HWT901B_Bridge_Init(UART_HandleTypeDef *sensorUart,
                                      UART_HandleTypeDef *megaUart) {
  if (!sensorUart || !megaUart) return HAL_ERROR;
  sensor = sensorUart;
  mega = megaUart;
  head = tail = overflow = rxError = rxRestart = used = 0;
  byteAt = HAL_GetTick();
  const HAL_StatusTypeDef status = HAL_UART_Receive_IT(sensor, &rxByte, 1U);
  if (status != HAL_OK) rxRestart = 1;
  return status;
}

void HWT901B_Bridge_RxCpltCallback(UART_HandleTypeDef *uart) {
  if (uart != sensor) return;
  push(rxByte, HAL_GetTick());
  if (HAL_UART_Receive_IT(sensor, &rxByte, 1U) != HAL_OK) {
    rxError = 1;
    rxRestart = 1;
  } else rxRestart = 0;
}

void HWT901B_Bridge_ErrorCallback(UART_HandleTypeDef *uart) {
  if (uart != sensor) return;
  rxError = 1;
  rxRestart = 1;
}

void HWT901B_Bridge_Task(void) {
  if (!sensor || !mega) return;
  uint32_t irqState = __get_PRIMASK();
  __disable_irq();
  const uint8_t reset = (uint8_t)(overflow || rxError);
  const uint8_t restart = rxRestart;
  overflow = rxError = rxRestart = 0;
  if (reset) tail = head;
  __set_PRIMASK(irqState);
  if (reset) resetParser();
  if (restart && HAL_UART_Receive_IT(sensor, &rxByte, 1U) != HAL_OK) {
    irqState = __get_PRIMASK();
    __disable_irq();
    rxRestart = 1;
    __set_PRIMASK(irqState);
  }

  if (used && (uint32_t)(HAL_GetTick() - byteAt) >= BYTE_TIMEOUT_MS) resetParser();
  uint8_t budget = TASK_BUDGET;
  uint8_t byte;
  uint32_t receivedAt;
  while (budget-- && pop(&byte, &receivedAt)) {
    if (used && (uint32_t)(receivedAt - byteAt) >= BYTE_TIMEOUT_MS) resetParser();
    byteAt = receivedAt;
    if ((uint32_t)(HAL_GetTick() - receivedAt) >= BYTE_TIMEOUT_MS) {
      resetParser();
      continue;
    }
    if (!ingest(byte)) continue;
    if (HAL_UART_Transmit(mega, frame, FRAME_SIZE, TX_TIMEOUT_MS) != HAL_OK) {
      resetParser();
    }
    return; // Send at most one yaw frame per main-loop pass.
  }
}
