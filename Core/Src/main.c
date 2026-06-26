#include "main.h"
#include "bsp_bmi088.h"
#include "bsp_vofa.h"
#include "ahrs.h"
#include "gimbal.h"
#include <stdio.h>

SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart6;
TIM_HandleTypeDef htim6;

static float last_cmd_char;
static float last_cmd_val;
static uint8_t vofa_view = 0;   /* 0 gimbal view, 1 imu view */

static float gyro_lpf_roll  = 0.0f;
static float gyro_lpf_pitch = 0.0f;
static float gyro_lpf_yaw   = 0.0f;
#define GYRO_LPF_ALPHA  0.8f

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM6_Init(void);

int main(void)
{
    uint8_t bmi088_ret;
    uint32_t last_tick = 0;

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_USART6_UART_Init();
    MX_TIM6_Init();

    vofa_init();

    USART6->CR1 |= USART_CR1_RXNEIE;

    bmi088_ret = bmi088_init();

    if (bmi088_ret != 0) {
        while (1) {
            vofa_send_all(0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_12);
            HAL_Delay(5);
        }
    }

    {
        char msg[64];
        int len = snprintf(msg, sizeof(msg),
            "BMI088 OK! acc_id=0x%02X, gyro_id=0x%02X\r\n",
            bmi088_data.accel_chip_id, bmi088_data.gyro_chip_id);
        HAL_UART_Transmit(&huart6, (uint8_t *)msg, len, 100);
    }

    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_SET);

    ahrs_init();
    gimbal_init();

    gimbal_set_target(5, 0.0f);   /* yaw */
    gimbal_set_target(1, 0.0f);   /* pitch */

    while (1) {
        if (HAL_GetTick() - last_tick >= 5) {
            float roll, pitch, yaw;

            last_tick = HAL_GetTick();

            bmi088_read_all();

            gyro_lpf_roll  = GYRO_LPF_ALPHA * bmi088_data.gyro.x + (1.0f - GYRO_LPF_ALPHA) * gyro_lpf_roll;
            gyro_lpf_pitch = GYRO_LPF_ALPHA * bmi088_data.gyro.y + (1.0f - GYRO_LPF_ALPHA) * gyro_lpf_pitch;
            gyro_lpf_yaw   = GYRO_LPF_ALPHA * bmi088_data.gyro.z + (1.0f - GYRO_LPF_ALPHA) * gyro_lpf_yaw;

            ahrs_update(
                gyro_lpf_roll, gyro_lpf_pitch, gyro_lpf_yaw,
                bmi088_data.accel.x, bmi088_data.accel.y, bmi088_data.accel.z,
                0.005f
            );

            ahrs_get_euler(&roll, &pitch, &yaw);

            gimbal_update();

            if (vofa_rx_available()) {
                float rx_data[VOFA_RX_CHANNELS];
                vofa_rx_get(rx_data);

                char cmd = (char)rx_data[0];
                float val = rx_data[1];

                last_cmd_char = (float)cmd;
                last_cmd_val = val;

                if (cmd == 'a') {
                    gimbal_set_target(5, val);
                } else if (cmd == 'p') {
                    gimbal_pid[5].kp = val;
                    gimbal_pid[5].integral = 0.0f;
                } else if (cmd == 'i') {
                    gimbal_pid[5].ki = val;
                    gimbal_pid[5].integral = 0.0f;
                } else if (cmd == 'd') {
                    gimbal_pid[5].kd = val;
                    gimbal_pid[5].integral = 0.0f;
                } else if (cmd == 's') {
                    gimbal_speed_pid[5].kp = val;
                    gimbal_speed_pid[5].integral = 0.0f;
                } else if (cmd == 'x') {
                    gimbal_speed_pid[5].ki = val;
                    gimbal_speed_pid[5].integral = 0.0f;
                } else if (cmd == 'z') {
                    gimbal_speed_pid[5].kd = val;
                    gimbal_speed_pid[5].integral = 0.0f;
                } else if (cmd == 'A') {
                    gimbal_set_target(1, val);
                } else if (cmd == 'P') {
                    gimbal_pid[1].kp = val;
                    gimbal_pid[1].integral = 0.0f;
                } else if (cmd == 'I') {
                    gimbal_pid[1].ki = val;
                    gimbal_pid[1].integral = 0.0f;
                } else if (cmd == 'D') {
                    gimbal_pid[1].kd = val;
                    gimbal_pid[1].integral = 0.0f;
                } else if (cmd == 'S') {
                    gimbal_speed_pid[1].kp = val;
                    gimbal_speed_pid[1].integral = 0.0f;
                } else if (cmd == 'X') {
                    gimbal_speed_pid[1].ki = val;
                    gimbal_speed_pid[1].integral = 0.0f;
                } else if (cmd == 'Z') {
                    gimbal_speed_pid[1].kd = val;
                    gimbal_speed_pid[1].integral = 0.0f;
                } else if (cmd == 'f') {
                    gimbal_pitch_enabled = !gimbal_pitch_enabled;
                } else if (cmd == 'm') {
                    gimbal_set_ctrl_mode((uint8_t)val);
                } else if (cmd == 'v') {
                    vofa_view = (uint8_t)val;
                } else if (cmd == 'c') {
                    gimbal_imu_calibrate(pitch * 57.2957795131f, yaw * 57.2957795131f);
                    gimbal_set_target(1, 0.0f);
                    gimbal_set_target(5, 0.0f);
                }
            }

            if (vofa_view == 1) {
                vofa_send_imu(
                    bmi088_data.accel.x, bmi088_data.accel.y, bmi088_data.accel.z,
                    bmi088_data.gyro.x, bmi088_data.gyro.y, bmi088_data.gyro.z,
                    roll, pitch, yaw,
                    gimbal_imu_pitch, gimbal_imu_yaw,
                    gyro_lpf_pitch, gyro_lpf_yaw);
            } else {
                vofa_send_all(
                    gimbal_imu_pitch,
                    gimbal_target[1],
                    gimbal_actual_speed[1],
                    gimbal_speed_target[1],
                    gimbal_output[1],
                    gimbal_pid[1].kp, gimbal_pid[1].ki, gimbal_pid[1].kd,
                    gimbal_speed_pid[1].kp, gimbal_speed_pid[1].ki,
                    gimbal_imu_yaw,
                    gimbal_target[5],
                    gimbal_actual_speed[5],
                    gimbal_speed_target[5],
                    gimbal_output[5],
                    gimbal_pid[5].kp, gimbal_pid[5].ki, gimbal_pid[5].kd,
                    gimbal_speed_pid[5].kp, gimbal_speed_pid[5].ki
                );
            }

            HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_11);
        }
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 6;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_USART6_UART_Init(void)
{
    huart6.Instance = USART6;
    huart6.Init.BaudRate = 921600;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart6) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_TIM6_Init(void)
{
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 8399;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 0xFFFF;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    HAL_GPIO_WritePin(BMI088_ACC_CS_PORT, BMI088_ACC_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = BMI088_ACC_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BMI088_ACC_CS_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BMI088_GYRO_CS_PIN;
    HAL_GPIO_Init(BMI088_GYRO_CS_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hspi->Instance == SPI1) {
        __HAL_RCC_SPI1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (huart->Instance == USART6) {
        __HAL_RCC_USART6_CLK_ENABLE();
        __HAL_RCC_GPIOG_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_14;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
        HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
        HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART6_IRQn);
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_12);
        HAL_Delay(200);
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif