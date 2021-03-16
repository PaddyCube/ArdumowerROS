#ifndef _UART_TASK_H_
#define _UART_TASK_H_

// UART Definitions
#define UART_TXD (CONFIG_UART_TXD)
#define UART_RXD (CONFIG_UART_RXD)
#define UART_RTS (UART_PIN_NO_CHANGE)
#define UART_CTS (UART_PIN_NO_CHANGE)

#define UART_PORT_NUM (CONFIG_UART_PORT_NUM)
#define UART_BAUD_RATE (CONFIG_UART_BAUD_RATE)
#define UART_TASK_STACK_SIZE (CONFIG_UART_TASK_STACK_SIZE)

#define MIN_PATTERN_INTERVAL (9)
#define MIN_POST_IDLE (0)
#define MIN_PRE_IDLE (0)
#define UART_EVENT_QUEUE_SIZE (100)
#define UART_PATTERN_QUEUE_SIZE (200)

#define BUF_SIZE (1024)

#include "driver/uart.h"
#include "freertos/queue.h"

extern QueueHandle_t uartQueue ;
extern QueueHandle_t inboundMessageQueue ;


void setup_uart();
void uart_task(void *arg);

#endif