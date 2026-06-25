#ifndef __BSP_BMI088_H
#define __BSP_BMI088_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct {
    float x;
    float y;
    float z;
} bmi088_axis_t;

typedef struct {
    bmi088_axis_t accel;
    bmi088_axis_t gyro;
    uint8_t accel_chip_id;
    uint8_t gyro_chip_id;
} bmi088_data_t;

extern bmi088_data_t bmi088_data;

#define BMI088_ACC_CHIP_ID_ADDR     0x00
#define BMI088_ACC_CHIP_ID_VALUE    0x1E
#define BMI088_ACC_ERR_REG_ADDR     0x02
#define BMI088_ACC_STATUS_ADDR      0x03
#define BMI088_ACC_X_LSB_ADDR       0x12
#define BMI088_ACC_X_MSB_ADDR       0x13
#define BMI088_ACC_Y_LSB_ADDR       0x14
#define BMI088_ACC_Y_MSB_ADDR       0x15
#define BMI088_ACC_Z_LSB_ADDR       0x16
#define BMI088_ACC_Z_MSB_ADDR       0x17
#define BMI088_ACC_SENSORTIME_0     0x18
#define BMI088_ACC_INT_STAT_1       0x1D
#define BMI088_ACC_TEMP_MSB_ADDR    0x22
#define BMI088_ACC_TEMP_LSB_ADDR    0x23
#define BMI088_ACC_CONF_ADDR        0x40
#define BMI088_ACC_RANGE_ADDR       0x41
#define BMI088_ACC_INT1_IO_CTRL     0x53
#define BMI088_ACC_INT2_IO_CTRL     0x54
#define BMI088_ACC_INT1_MAP         0x56
#define BMI088_ACC_INT2_MAP         0x57
#define BMI088_ACC_INT1_MAP_DATA    0x58
#define BMI088_ACC_INT2_MAP_DATA    0x59
#define BMI088_ACC_SELF_TEST        0x6D
#define BMI088_ACC_PWR_CONF_ADDR    0x7C
#define BMI088_ACC_PWR_CTRL_ADDR    0x7D
#define BMI088_ACC_SOFTRESET_ADDR   0x7E

#define BMI088_GYRO_CHIP_ID_ADDR    0x00
#define BMI088_GYRO_CHIP_ID_VALUE   0x0F
#define BMI088_GYRO_X_LSB_ADDR      0x02
#define BMI088_GYRO_X_MSB_ADDR      0x03
#define BMI088_GYRO_Y_LSB_ADDR      0x04
#define BMI088_GYRO_Y_MSB_ADDR      0x05
#define BMI088_GYRO_Z_LSB_ADDR      0x06
#define BMI088_GYRO_Z_MSB_ADDR      0x07
#define BMI088_GYRO_INT_STAT_1      0x0A
#define BMI088_GYRO_RANGE_ADDR      0x0F
#define BMI088_GYRO_BANDWIDTH_ADDR  0x10
#define BMI088_GYRO_INT1_MAP_ADDR   0x18
#define BMI088_GYRO_INT2_MAP_ADDR   0x19
#define BMI088_GYRO_INT_CTRL_ADDR   0x1A
#define BMI088_GYRO_INT3_INT4_IO_CONF  0x1B
#define BMI088_GYRO_INT3_INT4_IO_MAP   0x1C
#define BMI088_GYRO_SELF_TEST      0x3C
#define BMI088_GYRO_SOFTRESET_ADDR 0x14

#define BMI088_ACC_RANGE_3G         0x00
#define BMI088_ACC_RANGE_6G         0x01
#define BMI088_ACC_RANGE_12G        0x02
#define BMI088_ACC_RANGE_24G        0x03

#define BMI088_ACC_ODR_12_5         0x05
#define BMI088_ACC_ODR_25           0x06
#define BMI088_ACC_ODR_50           0x07
#define BMI088_ACC_ODR_100          0x08
#define BMI088_ACC_ODR_200          0x09
#define BMI088_ACC_ODR_400          0x0A
#define BMI088_ACC_ODR_800          0x0B
#define BMI088_ACC_ODR_1600         0x0C

#define BMI088_ACC_BWP_NORMAL       0x00
#define BMI088_ACC_BWP_OSR4         0x01
#define BMI088_ACC_BWP_OSR2         0x02

#define BMI088_GYRO_RANGE_2000      0x00
#define BMI088_GYRO_RANGE_1000      0x01
#define BMI088_GYRO_RANGE_500       0x02
#define BMI088_GYRO_RANGE_250       0x03
#define BMI088_GYRO_RANGE_125       0x04

#define BMI088_GYRO_BW_532_ODR_2000 0x00
#define BMI088_GYRO_BW_230_ODR_2000 0x01
#define BMI088_GYRO_BW_116_ODR_1000 0x02
#define BMI088_GYRO_BW_47_ODR_400   0x03
#define BMI088_GYRO_BW_23_ODR_200   0x04
#define BMI088_GYRO_BW_12_ODR_100   0x05
#define BMI088_GYRO_BW_64_ODR_3200  0x06
#define BMI088_GYRO_BW_32_ODR_1600  0x07

#define BMI088_ACC_3G_SEN       0.0008974358974f
#define BMI088_ACC_6G_SEN       0.0017948717948f
#define BMI088_ACC_12G_SEN      0.0035897435897f
#define BMI088_ACC_24G_SEN      0.0071794871794f

#define BMI088_GYRO_2000_SEN    0.0010652644f
#define BMI088_GYRO_1000_SEN    0.0005326322f
#define BMI088_GYRO_500_SEN     0.0002663161f
#define BMI088_GYRO_250_SEN     0.0001331581f
#define BMI088_GYRO_125_SEN     0.0000665790f

#define BMI088_SPI_READ_BIT     0x80

uint8_t bmi088_init(void);
void bmi088_accel_read(void);
void bmi088_gyro_read(void);
void bmi088_read_all(void);

#ifdef __cplusplus
}
#endif

#endif