#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define BMI088_ACC_CS_PIN       GPIO_PIN_4
#define BMI088_ACC_CS_PORT      GPIOA
#define BMI088_GYRO_CS_PIN      GPIO_PIN_0
#define BMI088_GYRO_CS_PORT     GPIOB

#define BMI088_ACC_CS_LOW()     HAL_GPIO_WritePin(BMI088_ACC_CS_PORT, BMI088_ACC_CS_PIN, GPIO_PIN_RESET)
#define BMI088_ACC_CS_HIGH()    HAL_GPIO_WritePin(BMI088_ACC_CS_PORT, BMI088_ACC_CS_PIN, GPIO_PIN_SET)
#define BMI088_GYRO_CS_LOW()    HAL_GPIO_WritePin(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN, GPIO_PIN_RESET)
#define BMI088_GYRO_CS_HIGH()   HAL_GPIO_WritePin(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN, GPIO_PIN_SET)

void Error_Handler(void);

extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart6;

#ifdef __cplusplus
}
#endif

#endif