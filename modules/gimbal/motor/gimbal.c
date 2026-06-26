#include "gimbal.h"
#include "ahrs.h"
#include <string.h>
#include <math.h>

#define RAD2DEG  57.2957795131f

CAN_HandleTypeDef hcan1;
static CAN_TxHeaderTypeDef   tx_header_volt_1_4;
static CAN_TxHeaderTypeDef   tx_header_volt_5_7;
static CAN_TxHeaderTypeDef   tx_header_curr_1_4;
static CAN_TxHeaderTypeDef   tx_header_curr_5_7;
static uint32_t              tx_mailbox;

gm6020_motor_t gimbal_motor[8];
float          gimbal_target[8];
float          gimbal_output[8];
pid_t          gimbal_pid[8];
pid_t          gimbal_speed_pid[8];
float          gimbal_speed_target[8];
float          gimbal_actual_speed[8];
uint8_t        gimbal_ctrl_mode = 0;
uint8_t        gimbal_pitch_enabled = 1;

float          gimbal_imu_pitch = 0.0f;
float          gimbal_imu_yaw = 0.0f;

static uint8_t  can_rx_buf[8];
uint8_t  gimbal_can_rx_buf[8];
uint32_t gimbal_can_send_cnt;
uint32_t gimbal_can_fail_cnt;
uint32_t gimbal_can_recv_cnt;
uint32_t gimbal_can_recv_all;

float gimbal_imu_offset_pitch = 0.0f;
float gimbal_imu_offset_yaw = 0.0f;
uint8_t gimbal_imu_calibrated = 0;

float gimbal_ff_output[8];

static void gimbal_can_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 3;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = DISABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        gimbal_can_fail_cnt = 9999;
        return;
    }

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &filter);

    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

static void init_tx_header(CAN_TxHeaderTypeDef *hdr, uint32_t stdid)
{
    hdr->StdId = stdid;
    hdr->ExtId = 0;
    hdr->RTR = CAN_RTR_DATA;
    hdr->IDE = CAN_ID_STD;
    hdr->DLC = 8;
    hdr->TransmitGlobalTime = DISABLE;
}

void gimbal_init(void)
{
    memset(gimbal_motor, 0, sizeof(gimbal_motor));
    memset(gimbal_target, 0, sizeof(gimbal_target));
    memset(gimbal_output, 0, sizeof(gimbal_output));

    gimbal_can_send_cnt = 0;
    gimbal_can_fail_cnt = 0;
    gimbal_can_recv_cnt = 0;

    for (int i = 0; i < 8; i++) {
        pid_init(&gimbal_pid[i], 0.0f, 0.0f, 0.0f, 3000.0f, 500.0f);
        pid_init(&gimbal_speed_pid[i], 0.0f, 0.0f, 0.0f, 3000.0f, GM6020_MAX_VOLTAGE);
        gimbal_ff_output[i] = 0.0f;
    }

    pid_init(&gimbal_pid[1], 27.0f, 0.1f, 0.0f, 500.0f, 500.0f);
    gimbal_pid[1].wrap_angle = 1;
    pid_init(&gimbal_speed_pid[1], 10.8f, 1.0f, 0.0f, 30000.0f, GM6020_MAX_VOLTAGE);
    gimbal_speed_pid[1].d_on_measurement = 1;

    pid_init(&gimbal_pid[5], 15.0f, 0.0f, 0.0f, 300.0f, 1500.0f);
    gimbal_pid[5].wrap_angle = 1;
    pid_init(&gimbal_speed_pid[5], 16.0f, 0.1f, 0.0f, 8000.0f, GM6020_MAX_VOLTAGE);
    gimbal_speed_pid[5].d_on_measurement = 1;

    gimbal_can_init();

    init_tx_header(&tx_header_volt_1_4, GM6020_VOLT_CTRL_ID_1_4);
    init_tx_header(&tx_header_volt_5_7, GM6020_VOLT_CTRL_ID_5_7);
    init_tx_header(&tx_header_curr_1_4, GM6020_CURR_CTRL_ID_1_4);
    init_tx_header(&tx_header_curr_5_7, GM6020_CURR_CTRL_ID_5_7);
}

void gimbal_set_target(uint8_t id, float angle_deg)
{
    if (id < 1 || id > 7) return;
    gimbal_target[id] = angle_deg;
}

void gimbal_set_ctrl_mode(uint8_t mode)
{
    gimbal_ctrl_mode = mode;
}

void gimbal_imu_calibrate(float pitch, float yaw)
{
    gimbal_imu_offset_pitch = pitch;
    gimbal_imu_offset_yaw = yaw;
    gimbal_imu_calibrated = 1;
}

static HAL_StatusTypeDef can_send_safe(CAN_TxHeaderTypeDef *hdr, uint8_t *data)
{
    HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(&hcan1, hdr, data, &tx_mailbox);
    if (ret == HAL_OK) {
        gimbal_can_send_cnt++;
    } else {
        gimbal_can_fail_cnt++;
        if (HAL_CAN_GetError(&hcan1) & (HAL_CAN_ERROR_BOF | HAL_CAN_ERROR_EPV)) {
            HAL_CAN_ResetError(&hcan1);
            HAL_CAN_Start(&hcan1);
        }
    }
    return ret;
}

void gimbal_update(void)
{
    float dt = 0.005f;
    uint8_t data[8];
    float roll, pitch, yaw;

    ahrs_get_euler(&roll, &pitch, &yaw);
    gimbal_imu_pitch = pitch * RAD2DEG - gimbal_imu_offset_pitch;
    gimbal_imu_yaw = yaw * RAD2DEG - gimbal_imu_offset_yaw;

    if (gimbal_can_fail_cnt >= 9999) {
        return;
    }

    for (int i = 0; i < 8; i++) {
        gimbal_output[i] = 0.0f;
        gimbal_speed_target[i] = 0.0f;
    }

    if (gimbal_pitch_enabled && gimbal_motor[1].online) {
        gimbal_actual_speed[1] = gimbal_motor[1].speed_rpm * 6.0f;

        gimbal_speed_target[1] = pid_calc(&gimbal_pid[1],
            gimbal_target[1], gimbal_imu_pitch, dt);

        gimbal_output[1] = pid_calc(&gimbal_speed_pid[1],
            gimbal_speed_target[1], gimbal_actual_speed[1], dt);
    }

    if (gimbal_motor[5].online) {
        gimbal_actual_speed[5] = gimbal_motor[5].speed_rpm * 6.0f;

        gimbal_speed_target[5] = pid_calc(&gimbal_pid[5],
            gimbal_target[5], gimbal_imu_yaw, dt);

        gimbal_output[5] = pid_calc(&gimbal_speed_pid[5],
            gimbal_speed_target[5], gimbal_actual_speed[5], dt);
    }

    CAN_TxHeaderTypeDef *hdr_1_4 = gimbal_ctrl_mode
        ? &tx_header_curr_1_4 : &tx_header_volt_1_4;
    CAN_TxHeaderTypeDef *hdr_5_7 = gimbal_ctrl_mode
        ? &tx_header_curr_5_7 : &tx_header_volt_5_7;

    data[0] = ((int16_t)gimbal_output[1] >> 8) & 0xFF;
    data[1] = ((int16_t)gimbal_output[1] >> 0) & 0xFF;
    data[2] = 0; data[3] = 0;
    data[4] = 0; data[5] = 0;
    data[6] = 0; data[7] = 0;
    can_send_safe(hdr_1_4, data);

    data[0] = ((int16_t)gimbal_output[5] >> 8) & 0xFF;
    data[1] = ((int16_t)gimbal_output[5] >> 0) & 0xFF;
    data[2] = 0; data[3] = 0;
    data[4] = 0; data[5] = 0;
    data[6] = 0; data[7] = 0;
    can_send_safe(hdr_5_7, data);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, can_rx_buf);
    gimbal_can_recv_all++;
    gimbal_can_rx_buf[0] = can_rx_buf[0];
    gimbal_can_rx_buf[1] = can_rx_buf[1];
    gimbal_can_rx_buf[2] = can_rx_buf[2];
    gimbal_can_rx_buf[3] = can_rx_buf[3];

    if (rx_header.StdId >= GM6020_FEEDBACK_ID_BASE
        && rx_header.StdId <= GM6020_FEEDBACK_ID_BASE + 7) {
        uint8_t id = rx_header.StdId - GM6020_FEEDBACK_ID_BASE;
        if (id < 8) {
            gimbal_motor[id].ecd         = ((uint16_t)can_rx_buf[0] << 8) | can_rx_buf[1];
            gimbal_motor[id].speed_rpm   = ((int16_t)((uint16_t)can_rx_buf[2] << 8) | can_rx_buf[3]);
            gimbal_motor[id].current     = ((int16_t)((uint16_t)can_rx_buf[4] << 8) | can_rx_buf[5]);
            gimbal_motor[id].temperature = can_rx_buf[6];
            gimbal_motor[id].angle_deg   = gimbal_motor[id].ecd * 360.0f / (float)GM6020_ECD_MAX;
            gimbal_motor[id].online      = 1;
            gimbal_can_recv_cnt++;
        }
    }
}