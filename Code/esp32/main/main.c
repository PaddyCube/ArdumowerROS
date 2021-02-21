#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_uros/options.h>
#include "uxr/client/config.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

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


static const char *TAG = "Ardumower";
QueueHandle_t uartQueue = NULL;
QueueHandle_t inboundMessageQueue = NULL;

// ROS Definitions
#define RCCHECK(fn)                                                                      \
	{                                                                                    \
		rcl_ret_t temp_rc = fn;                                                          \
		if ((temp_rc != RCL_RET_OK))                                                     \
		{                                                                                \
			printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
			vTaskDelete(NULL);                                                           \
		}                                                                                \
	}
#define RCSOFTCHECK(fn)                                                                    \
	{                                                                                      \
		rcl_ret_t temp_rc = fn;                                                            \
		if ((temp_rc != RCL_RET_OK))                                                       \
		{                                                                                  \
			printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
		}                                                                                  \
	}

rcl_publisher_t publisher_int32;
rcl_publisher_t publisher_string;
std_msgs__msg__Int32 msgInt;
std_msgs__msg__String msgStr;


void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL)
	{
		RCSOFTCHECK(rcl_publish(&publisher_int32, &msgInt, NULL));
		msgInt.data = msgInt.data + 5;
	}
}

void micro_ros_task(void *arg)
{

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));
	rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "esp32_int32_publisher", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher_int32,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"freertos_int32_publisher"));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher_string,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
		"ardumower_uart"));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	msgInt.data = 0;
	uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
		while (1)
	{
		if (inboundMessageQueue != NULL)
		{
			
			if (xQueueReceive(inboundMessageQueue, data, (TickType_t)(10)) == pdPASS)
			{
				//ESP_LOGI(TAG, "data avaialble");
				ESP_LOGI(TAG, "ros2: %s", data);
				msgStr.data.data = (char *)data;
				//msgStr.data.size = BUF_SIZE;
				 //msgStr.data.data = "Hello from micro-ROS ";
		         msgStr.data.size = strlen(msgStr.data.data);
				RCSOFTCHECK(rcl_publish(&publisher_string, &msgStr, NULL));
			}
		}

		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
		//usleep(10000);
	}
	// free resources
	RCCHECK(rcl_publisher_fini(&publisher_int32, &node));
	RCCHECK(rcl_publisher_fini(&publisher_string, &node));
	RCCHECK(rcl_node_fini(&node));

	vTaskDelete(NULL);
}

static void uart_task(void *arg)
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
				//handle_uart_pattern();
				uart_get_buffered_data_len(UART_PORT_NUM, &buffered_size);
				int pos = uart_pattern_pop_pos(UART_PORT_NUM);
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
					data[len] = '\0';
					//ESP_LOGI(TAG, "data: %s", data);
					xQueueSendFromISR(inboundMessageQueue, data, NULL);
					//xQueueSend(inboundMessageQueue, data, (TickType_t)0);
				}

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
		.source_clk = UART_SCLK_APB,
	};
	int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
	intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

	ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE, 0, UART_EVENT_QUEUE_SIZE, &uartQueue, intr_alloc_flags));
	ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TXD, UART_RXD, UART_RTS, UART_CTS));

	// Configure a temporary buffer for the incoming data
	//uint8_t *data = (uint8_t *)malloc(BUF_SIZE);

	// while (1)
	// {
	// 	// Read data from the UART
	// 	int len = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE, 20 / portTICK_RATE_MS);
	// 	// Write data back to the UART
	// 	if (len > 0)
	// 	{
	// 		data[len] = 0;
	// 		ESP_LOGI(TAG, "Data received: %i", len);
	// 		xQueueSend(uartQueue, data, (TickType_t)0);
	// 	}
	// }

	ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(UART_PORT_NUM, '\n', 1, MIN_PATTERN_INTERVAL, MIN_POST_IDLE, MIN_PRE_IDLE));
	/* Set pattern queue size */
	ESP_ERROR_CHECK(uart_pattern_queue_reset(UART_PORT_NUM, UART_PATTERN_QUEUE_SIZE));
	/* Create semaphore */
	//process_sem = xSemaphoreCreateBinary();
	/* Create UART Event task */
	//	xTaskCreate(uart_event_task_entry, "uart_event", UART_EVENT_TASK_STACK_SIZE, NULL, UART_EVENT_TASK_PRIORITY, &uart_event_task_hdl);
}

void app_main(void)
{
#ifdef UCLIENT_PROFILE_UDP
	// Start the networking if required
	ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif // UCLIENT_PROFILE_UDP

	// Create Queue to transform received UART messages
	inboundMessageQueue = xQueueCreate(20, BUF_SIZE);
	setup_uart();
	//pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
	xTaskCreate(micro_ros_task,
				"uros_task",
				CONFIG_MICRO_ROS_APP_STACK,
				NULL,
				CONFIG_MICRO_ROS_APP_TASK_PRIO,
				NULL);
	// UART monitor Task
	xTaskCreate(uart_task, "uart_task", UART_TASK_STACK_SIZE, NULL, 20, NULL);
}
