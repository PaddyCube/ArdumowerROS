#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"

#include "uart_task.h"
#include "driver/uart.h"
#include "freertos/queue.h"

static const char *TAG = "Ardumower";
//QueueHandle_t uartQueue = NULL;
//QueueHandle_t inboundMessageQueue = NULL;

void uart_task(void *arg)
{
	uart_event_t event;
	size_t buffered_size;
	uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
	int len = 0;
	while (1)
	{
		if (xQueueReceive(uartQueue, (void *)&event, (portTickType)portMAX_DELAY))
		{
			switch (event.type)
			{
			case UART_DATA:
				//handle_uart_data();
				ESP_LOGI(TAG, "UART DATA Event");
				break;
			case UART_FIFO_OVF:
				ESP_LOGW(TAG, "HW FIFO Overflow");
				uart_flush_input(UART_PORT_NUM);
				xQueueReset(uartQueue);
				break;
			case UART_BUFFER_FULL:
				ESP_LOGW(TAG, "Ring Buffer Full");
				uart_flush_input(UART_PORT_NUM);
				xQueueReset(uartQueue);
				break;
			case UART_BREAK:
				ESP_LOGW(TAG, "Rx Break");
				break;
			case UART_PARITY_ERR:
				ESP_LOGE(TAG, "Parity Error");
				break;
			case UART_FRAME_ERR:
				ESP_LOGE(TAG, "Frame Error");
				break;
			case UART_PATTERN_DET:
			{
				//handle_uart_pattern();
				uart_get_buffered_data_len(UART_PORT_NUM, &buffered_size);
				int pos = 0;
				pos = uart_pattern_pop_pos(UART_PORT_NUM);
				//ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
				if (pos == -1)
				{
					// There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
					// record the position. We should set a larger queue size.
					// As an example, we directly flush the rx buffer here.
					uart_flush_input(UART_PORT_NUM);
				}
				else
				{
					len = uart_read_bytes(UART_PORT_NUM, data, pos, 100 / portTICK_PERIOD_MS);
					data[len-1] = '\0';
					//ESP_LOGI(TAG, "data: %s", data);
					xQueueSendFromISR(inboundMessageQueue, data+1, NULL);
					//xQueueSend(inboundMessageQueue, data, (TickType_t)0);
				}
			}
				break;
			case UART_DATA_BREAK:
				ESP_LOGW(TAG, "unknown uart event type: %d", event.type);
				break;
			case UART_EVENT_MAX:
				ESP_LOGW(TAG, "unknown uart event type: %d", event.type);
				break;
			default:
				ESP_LOGW(TAG, "unknown uart event type: %d", event.type);
				break;
			}
		}
	}

	vTaskDelete(NULL);
}

void setup_uart() //(void *arg)
{
	/* Configure parameters of an UART driver,
     * communication pins and install the driver */
	uart_config_t uart_config = {
		.baud_rate = UART_BAUD_RATE,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_APB};
	int intr_alloc_flags = 0;
// .rx_flow_ctrl_thresh = 122
#if CONFIG_UART_ISR_IN_IRAM
	intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

	ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE, 0, UART_EVENT_QUEUE_SIZE, &uartQueue, intr_alloc_flags));
	ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TXD, UART_RXD, UART_RTS, UART_CTS));

	ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(UART_PORT_NUM, '\n', 1, MIN_PATTERN_INTERVAL, MIN_POST_IDLE, MIN_PRE_IDLE));
	/* Set pattern queue size */
	ESP_ERROR_CHECK(uart_pattern_queue_reset(UART_PORT_NUM, UART_PATTERN_QUEUE_SIZE));
	/* Create semaphore */
	//process_sem = xSemaphoreCreateBinary();
	/* Create UART Event task */
	//	xTaskCreate(uart_event_task_entry, "uart_event", UART_EVENT_TASK_STACK_SIZE, NULL, UART_EVENT_TASK_PRIORITY, &uart_event_task_hdl);
}
