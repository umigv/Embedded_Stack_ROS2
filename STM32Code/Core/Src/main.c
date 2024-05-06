/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"

#include <stdlib.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void StartDefaultTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx_buff[1];
char message[1];


unsigned long lastData = 0;
const float WHEEL_BASE = 0.62;			//verify!!
const float WHEEL_DIAMETER = 0.312928;
const long CONTROL_TIMEOUT = 1000;
const int LEFT_POLARITY = 1;
const int RIGHT_POLARITY = -1;
const float PI = 3.14159265359;
const float VEL_TO_RPS = 1.0 / (WHEEL_DIAMETER * PI) * 98.0/3.0;
const float RPS_LIMIT = 20;
const float  VEL_LIMIT = RPS_LIMIT / VEL_TO_RPS; // 1.2 mph (~0.57 m/s) limit


int volatile estop_mul = 1;
int long volatile right_encoder_tick = 0;
int long volatile left_encoder_tick = 0;


float right_prev_time = 0;
float right_curr_time;
float right_prev_dist = 0;
float right_curr_dist = 0;
float right_vel = 0;

float left_prev_time = 0;
float left_curr_time;
float left_prev_dist = 0;
float left_curr_dist = 0;
float left_vel = 0;

//constants for the robot
const float TICK_PER_REV = 720;
rcl_publisher_t enc_vel_publisher;
geometry_msgs__msg__Twist enc_vel_msg;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
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
}

/* USER CODE BEGIN 4 */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);


void update_right_dist_time_vel(){
    right_prev_dist = right_curr_dist;
    right_curr_dist = right_encoder_tick / TICK_PER_REV * (PI * WHEEL_DIAMETER);
    right_prev_time = right_curr_time;
    right_curr_time = HAL_GetTick();
    right_vel = (right_curr_dist - right_prev_dist) / (right_curr_time - right_prev_time) * 1000;
 }

void update_left_dist_time_vel(){
    left_prev_dist = left_curr_dist;
    left_curr_dist = left_encoder_tick / TICK_PER_REV * (PI * WHEEL_DIAMETER);
    left_prev_time = left_curr_time;
    left_curr_time = HAL_GetTick();
    left_vel = -1 * (left_curr_dist - left_prev_dist) / (left_curr_time - left_prev_time) * 1000;
 }


void MX_FREERTOS_Init(void) {
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
}
//			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,1);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  //maybe receive one character at a time like in arduino



  //message = rx_buff[0];
  HAL_UART_Receive_IT(&huart2, rx_buff, 1); //You need to toggle a breakpoint on this line!

  message[0] = (char)(50);//rx_buff[0]+20);//bytes_to_float(rx_buff);//uint8_t)(rx_buff[0]);
  //message--;
}


void subscription_callback(const void * msgin){
	float left_vel;
	float right_vel;
	const geometry_msgs__msg__Twist * rec = (const geometry_msgs__msg__Twist *)msgin;
	uint8_t * vel = "v 0 0\n";


	float linear = rec->linear.x;
	float angular = rec->angular.z;

	left_vel = estop_mul * LEFT_POLARITY * (linear - WHEEL_BASE * angular / 2.0);
	right_vel = estop_mul * RIGHT_POLARITY * (linear + WHEEL_BASE * angular / 2.0);

	/*
	int length = snprintf(NULL, 0, "%s%d", vel, rec->data)+1;
	char *newBuffer = malloc(length);
	snprintf(newBuffer, length, )
	*/
	//uint8_t * vel=//fprintf("v 0 %i\n",rec->data);//"v 0 15\n";
	//uint8_t * vel = "v 0 0\n";

	// Velocity limit should happen in ros message?
//	if (left_vel > VEL_LIMIT) {
//		left_vel = VEL_LIMIT;
//	}
//
//	if (right_vel > VEL_LIMIT) {
//		right_vel = VEL_LIMIT;
//	}

	float left_rpm = left_vel * VEL_TO_RPS;
	float right_rpm = right_vel * VEL_TO_RPS;

	char *msgOutLeft;
	char *msgOutRight;
	// change to float?

	asprintf(&msgOutRight, "v 0 %i\n", (int)right_rpm);
	if(HAL_UART_Transmit_IT(&huart2, msgOutRight, strlen(msgOutRight)) != HAL_OK){};

	asprintf(&msgOutLeft, "v 0 %i\n", (int)left_rpm);
	if(HAL_UART_Transmit_IT(&huart6, msgOutLeft, strlen(msgOutLeft)) != HAL_OK){};


//	asprintf(&msgOutRight, "v 0 %i\n", (int)right_vel);
//	if(HAL_UART_Transmit_IT(&huart2, msgOutRight, strlen(msgOutRight)) != HAL_OK){};
//
//	asprintf(&msgOutLeft, "v 0 %i\n", (int)left_vel);
//	if(HAL_UART_Transmit_IT(&huart6, msgOutLeft, strlen(msgOutLeft)) != HAL_OK){};

	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);

	free(msgOutRight);
	free(msgOutLeft);

	msgOutRight = NULL;
	msgOutLeft = NULL;






	//message = rec->data;
}


void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	// micro-ROS configuration
	  rmw_uros_set_custom_transport(
	    true,
	    (void *) &huart3,
	    cubemx_transport_open,
	    cubemx_transport_close,
	    cubemx_transport_write,
	    cubemx_transport_read);


	  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	  freeRTOS_allocator.allocate = microros_allocate;
	  freeRTOS_allocator.deallocate = microros_deallocate;
	  freeRTOS_allocator.reallocate = microros_reallocate;
	  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;


	  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
	      printf("Error on default allocators (line %d)\n", __LINE__);
	  }


	  // micro-ROS app

	  rcl_subscription_t subscriber;


	  rosidl_runtime_c__String s;

	  geometry_msgs__msg__Twist rec;
	  std_msgs__msg__String msg;


	  rclc_support_t support;
	  rcl_allocator_t allocator;
	  rcl_node_t node;


	  right_curr_time = HAL_GetTick();
	  left_curr_time = HAL_GetTick();

	  allocator = rcl_get_default_allocator();

	  // create publisher timer
//	  rcl_timer_t timer;
//	  const unsigned int spin_period = RCL_MS_TO_NS(20); //20 ms
//	  rcl_ret_t rc = rclc_timer_init_default(&timer, &support, spin_period, publish_callback);






	  //create init_options
	  rclc_support_init(&support, 0, NULL, &allocator);


	  // create node
	  rclc_node_init_default(&node, "cubemx_node", "", &support);


	  // create subscriber
	  rclc_subscription_init_default(
	  	    &subscriber,
	  	    &node,
	  	    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
	  	    "/cmd_vel");




	  //create publisher
	  rclc_publisher_init_default(
	    &enc_vel_publisher,
	    &node,
	    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
	    "/enc_vel");




	  rclc_executor_t executor;
	  rclc_executor_init(&executor, &support.context, 2, &allocator); //gpt says it should be 1 for the 3rd param
	  rclc_executor_add_subscription(&executor, &subscriber, &rec, &subscription_callback, ON_NEW_DATA);
//	  rclc_executor_add_timer(&executor, &timer);


	  s.data = message;
	  s.capacity = 1;
	  s.size = 1;
	  msg.data = s;

/*
	  osDelay(3000);
	  uint8_t calibrate[]="w axis0.requested_state 3\n";
	  HAL_UART_Transmit_IT(&huart2, calibrate, strlen(calibrate));
	  osDelay(60000);
	  HAL_UART_Transmit_IT(&huart6, calibrate, strlen(calibrate));
	  osDelay(60000);
	  uint8_t closed_loop[]="w axis0.requested_state 8\n";
	  HAL_UART_Transmit_IT(&huart2, closed_loop, strlen(closed_loop));
	  HAL_UART_Transmit_IT(&huart6, closed_loop, strlen(closed_loop));
	  osDelay(3000);
	  uint8_t vel0[]="v 0 5\n";
	  HAL_UART_Transmit_IT(&huart2, vel0, strlen(vel0));
	  HAL_UART_Transmit_IT(&huart6, vel0, strlen(vel0));
*/



	  // to read: r axis0.vel_estimate

	  float prev_pub_time = 0;
	  rclc_executor_spin(&executor);
	  for(;;)
	  {
		  update_right_dist_time_vel();
		  update_left_dist_time_vel();
		  if(right_encoder_tick == 0){
			  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin,1);
		  }
		  if(right_encoder_tick == -2){
			  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin,0);
		  }
		  update_right_dist_time_vel();
		  update_left_dist_time_vel();
		  if(HAL_GetTick() - prev_pub_time >= 20){
			  // Compute and publish the Twist message
			  enc_vel_msg.linear.x = (left_vel + right_vel) / 2.0;
			  enc_vel_msg.angular.z = (right_vel - left_vel) / WHEEL_BASE;
			  rcl_publish(&enc_vel_publisher, &enc_vel_msg, NULL);
			  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		  }

	  }
	  rclc_executor_fini(&executor);
	  rclc_publisher_fini(&enc_vel_publisher, &node);
	  rcl_subscrption_fini(&subscriber, &node);
	  rcl_node_fini(&node);
	  rclc_support_fini(&support);
	  rclc_publisher_fini(&enc_vel_publisher, &node);

//	  rcl_timer_fini(&timer);

  /* USER CODE END StartDefaultTask */
}


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  //ROS Publisher periodic callback
  if(htim == &htim6){
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);




  }

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {

  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
