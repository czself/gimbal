#include "bsp_vofa.h"
#include <string.h>

static UART_HandleTypeDef *vofa_uart;

uint32_t pack_vofa_justfloat(uint8_t* buffer, float* data, uint8_t channels)
{
    uint32_t offset = 0;
    for (uint8_t i = 0; i < channels; i++) {
        *((float*)(buffer + offset)) = data[i];
        offset += sizeof(float);
    }
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x00;
    buffer[offset++] = 0x80;
    buffer[offset++] = 0x7F;
    return offset;
}

void vofa_init(void)
{
    extern UART_HandleTypeDef huart6;
    vofa_uart = &huart6;
}

void vofa_send_all(float yaw, float yaw_target, float yaw_current,
    float yaw_angle_integral, float yaw_speed_integral,
    float yaw_angle_kp, float yaw_angle_ki, float yaw_angle_kd,
    float yaw_speed_kp, float yaw_speed_ki, float yaw_speed_kd)
{
    float data[11] = {yaw, yaw_target, yaw_current,
        yaw_angle_integral, yaw_speed_integral,
        yaw_angle_kp, yaw_angle_ki, yaw_angle_kd,
        yaw_speed_kp, yaw_speed_ki, yaw_speed_kd};
    uint8_t send_buf[48];
    uint32_t len = pack_vofa_justfloat(send_buf, data, 11);
    HAL_UART_Transmit(vofa_uart, send_buf, len, HAL_MAX_DELAY);
}

static uint8_t  rx_buf[32];
static uint8_t  rx_idx;
static uint8_t  rx_frame_ready;

uint8_t vofa_rx_idx_debug;
uint8_t vofa_rx_ready_debug;
uint32_t vofa_rx_err_debug;

void vofa_rx_byte(uint8_t byte)
{
    if (byte == '\n' || byte == '\r') {
        if (rx_idx > 0) {
            rx_buf[rx_idx] = '\0';
            rx_frame_ready = 1;
        }
        vofa_rx_idx_debug = rx_idx;
        vofa_rx_ready_debug = rx_frame_ready;
        return;
    }

    if (rx_idx < sizeof(rx_buf) - 1) {
        rx_buf[rx_idx++] = byte;
    }
    vofa_rx_idx_debug = rx_idx;
    vofa_rx_ready_debug = rx_frame_ready;
}

uint8_t vofa_rx_available(void)
{
    return rx_frame_ready;
}

void vofa_rx_get(float *data)
{
    char cmd = rx_buf[0];
    float val = 0.0f;

    if (rx_idx > 1) {
        val = 0.0f;
        uint8_t i = 1;
        uint8_t decimal = 0;
        float sign = 1.0f;
        if (rx_buf[i] == '-') { sign = -1.0f; i++; }
        for (; i < rx_idx; i++) {
            if (rx_buf[i] == '.') { decimal = 1; continue; }
            if (decimal) { decimal *= 10; }
            val = val * 10.0f + (rx_buf[i] - '0');
        }
        if (decimal) val /= decimal;
        val *= sign;
    }

    data[0] = (float)cmd;
    data[1] = val;
    data[2] = 0.0f;
    data[3] = 0.0f;

    rx_idx = 0;
    rx_frame_ready = 0;
}