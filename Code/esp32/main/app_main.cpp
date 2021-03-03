#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"

extern "C"
{
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <rcl_interfaces/msg/log.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_uros/options.h>
#include "uxr/client/config.h"
}

#include "driver/gpio.h"
#include "sdkconfig.h"
#include "uart_task.h"

extern "C"
{
	void app_main(void);
}

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
rcl_publisher_t publisher_log;
std_msgs__msg__Int32 msgInt;
std_msgs__msg__String msgStr;
rcl_interfaces__msg__Log msgLog;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL)
	{
		RCSOFTCHECK(rcl_publish(&publisher_int32, &msgInt, NULL));
		msgInt.data = msgInt.data + 5;

		// msgLog.level = rcl_interfaces__msg__Log__INFO;
		// msgLog.name.data = (char *)"ESP32 ";
		// msgLog.name.size = strlen(msgLog.name.data);
		// msgLog.msg.data = (char *)"Hello World";
		// msgLog.msg.size = strlen(msgLog.msg.data);
		// RCSOFTCHECK(rcl_publish(&publisher_log, &msgLog, NULL));
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
	//RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	if ((strlen(CONFIG_MICRO_ROS_AGENT_IP) != 0) && (CONFIG_MICRO_ROS_AGENT_PORT != 0) && (strcmp(CONFIG_MICRO_ROS_AGENT_IP, "-") != 0))
	{
		while (rmw_uros_options_set_udp_address(
				   CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options) != RCL_RET_OK)
		{
			ESP_LOGW(TAG, "microROS %s %s agent not found", CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
			usleep(1000);
		}
	}
	else
	{
		while (rmw_uros_discover_agent(rmw_options) != RCL_RET_OK)
		{
			ESP_LOGW(TAG, "microROS agent not found");
			usleep(1000);
		}
	}
	ESP_LOGW(TAG, "microROS agent connected");

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

	// // create publisher
	 RCCHECK(rclc_publisher_init_default(
	 	&publisher_log,
	 	&node,
	 	ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
	 	"rosout"));
		 
		 ESP_LOGW(TAG, "Error: %s", rcutils_get_error_string().str);
		 rcl_reset_error();

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
	RCCHECK(rcl_publisher_fini(&publisher_log, &node));
	RCCHECK(rcl_node_fini(&node));

	vTaskDelete(NULL);
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
