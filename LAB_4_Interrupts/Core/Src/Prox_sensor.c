/*
 * Prox_sensor.c
 *
 *  Created on: Feb 22, 2024
 *      Author: Royce
 */

#include "Prox_sensor.h"
#include <stdbool.h>

/*
 *     Examples
 *
 *     Prox_sensor_X_statues_check(); return a boolean value 0 or 1 (aka false or true)
 *
 *
 *     Prox_sensor_Y_statues_check(); return a boolean value 0 or 1 (aka false or true)
 *
 */
// Example initialization function where you enable the GPIO clock

void Prox_sensor_Init(void) {
    __HAL_RCC_GPIOE_CLK_ENABLE();
    // Initialize GPIO pins if not already initialized
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1|GPIO_PIN_2; // Configure both pins simultaneously
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // Set pins as input
    GPIO_InitStruct.Pull = GPIO_NOPULL; // No pull-up or pull-down resistor
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

bool Prox_sensor_X_Axis_Armature_status_check(void) {
    // Corrected to return bool
    return (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_SET) ? true : false;
}

bool Prox_sensor_Y_Axis_Armature_status_check(void) {
    // Corrected to return bool
    return (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) == GPIO_PIN_SET) ? true : false;
}

bool Prox_sensor_elevator_status_check(void) {
    // Corrected to return bool
    return (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_SET) ? true : false;
}

