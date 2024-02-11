// WORKING
// this program toggles the on-board red led when it receives a message
//    publish a message from terminal with $ros2 topic pub --once /subscriber std_msgs/msg/Int32 "{data: 4}"
// it then publishes the same number it received
// this program also has a timer that publishes the last number received at a regular interval

#include "stm32f4xx_hal.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <stdio.h>

UART_HandleTypeDef huart1;

rcl_subscription_t subscriber;
std_msgs__msg__Int32 rec;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rcl_publisher_t publisher_tim;
std_msgs__msg__Int32 msg_tim;

int last_rec = 0;


// STM configuration code - modify if needed for different stm functionality
void config()
{
	HAL_Init();
	//SystemClock_Config();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  //init USART1 RX: A10, TX: A9 (interrupt mode)
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
  msg_tim.data = last_rec;
	if (timer != NULL) {
		rcl_publish(&publisher_tim, &msg_tim, NULL);
	}
}

void subscription_callback(const void * msgin)
{
	// will toggle the red on-board pin when a message is received 
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
    const std_msgs__msg__Int32 * rec = (const std_msgs__msg__Int32 *)msgin;
	msg.data = rec->data;
  last_rec = rec->data;
	rcl_publish(&publisher, &msg, NULL);
}

void appMain(void * arg)
{
  config();
  rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	rclc_support_init(&support, 0, NULL, &allocator);

	// create node
	rcl_node_t node;
	rclc_node_init_default(&node, "velocity_sub", "", &support);

	// create subscriber
  
	rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"/subscriber");
  
	
  rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"/publisher");
  
  rclc_publisher_init_default(
		&publisher_tim,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"/publisher_tim");

  rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback);


	// create executor
	rclc_executor_t executor;
	rclc_executor_init(&executor, &support.context, 2, &allocator);
	rclc_executor_add_subscription(&executor, &subscriber, &rec, &subscription_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);

	while(1){
			rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
			//usleep(100000);
	}

	// free resources
	rclc_executor_fini(&executor);
  rcl_timer_fini(&timer);
  rcl_publisher_fini(&publisher, &node);
  rcl_publisher_fini(&publisher_tim, &node);
  rcl_subscription_fini(&subscriber, &node);
	rcl_node_fini(&node);
  rclc_support_fini(&support);
	
}
