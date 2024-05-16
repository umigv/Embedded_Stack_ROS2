/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PEPin PEPin PEPin PEPin */
  GPIO_InitStruct.Pin = enc_left_green_Pin|enc_left_white_Pin|enc_right_green_Pin|enc_right_white_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin */
  GPIO_InitStruct.Pin = phy_estop_Pin|remote_estop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */
extern int volatile estop_mul;
extern int volatile right_encoder_tick;
extern int volatile left_encoder_tick;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	//Estop
	if(GPIO_Pin == phy_estop_Pin || GPIO_Pin == remote_estop_Pin){
		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8)== 1 || HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)== 1){
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin,1);
			estop_mul = 0;
		}
		else{
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin,0);
			estop_mul = 1;
		}
	}

	//Encoders
	if(GPIO_Pin == enc_right_green_Pin){
		if((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == SET)&&(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14) == RESET)){
			right_encoder_tick ++;
		}
		else{
			right_encoder_tick --;
		}
		if((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == RESET)&&(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14) == SET)){
			right_encoder_tick ++;
		}
		else{
			right_encoder_tick --;
		}
	}

	if(GPIO_Pin == enc_right_white_Pin){
		if ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14) == RESET) && (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == SET)) {
			right_encoder_tick --;
		}
		else {
			right_encoder_tick ++;
		}


		if ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14) == SET) && (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == RESET)) {
			right_encoder_tick --;
		}
		else {
			right_encoder_tick ++;
		}
	}
	if(GPIO_Pin == enc_left_green_Pin){
		if ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12) == SET)&&(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10) == RESET)){
			left_encoder_tick ++;
		}
		else{
			left_encoder_tick --;
		}
		if ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12) == RESET)&&(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10) == SET)){
			left_encoder_tick ++;
		}
		else{
			left_encoder_tick --;
		}
	}
	if(GPIO_Pin == enc_left_white_Pin){
		if ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10) == RESET) && (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12) == SET)) {
			left_encoder_tick --;
		}
		else {
			left_encoder_tick ++;
		}


		if ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10) == SET) && (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12) == RESET)) {
			left_encoder_tick --;
		}
		else {
			left_encoder_tick ++;
		}
	}

}
/* USER CODE END 2 */
