#include "bsp_bmi088.h"

bmi088_data_t bmi088_data;

static SPI_HandleTypeDef *bmi088_spi;

static uint8_t spi_read_write_byte(uint8_t tx_data)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(bmi088_spi, &tx_data, &rx_data, 1, 100);
    return rx_data;
}

static void bmi088_accel_read_reg(uint8_t reg, uint8_t *buf, uint16_t len)
{
    BMI088_ACC_CS_LOW();
    spi_read_write_byte(reg | BMI088_SPI_READ_BIT);
    spi_read_write_byte(0x55);
    for (uint16_t i = 0; i < len; i++) {
        buf[i] = spi_read_write_byte(0x55);
    }
    BMI088_ACC_CS_HIGH();
}

static void bmi088_accel_write_reg(uint8_t reg, uint8_t data)
{
    BMI088_ACC_CS_LOW();
    spi_read_write_byte(reg);
    spi_read_write_byte(data);
    BMI088_ACC_CS_HIGH();
}

static void bmi088_gyro_read_reg(uint8_t reg, uint8_t *buf, uint16_t len)
{
    BMI088_GYRO_CS_LOW();
    spi_read_write_byte(reg | BMI088_SPI_READ_BIT);
    for (uint16_t i = 0; i < len; i++) {
        buf[i] = spi_read_write_byte(0x55);
    }
    BMI088_GYRO_CS_HIGH();
}

static void bmi088_gyro_write_reg(uint8_t reg, uint8_t data)
{
    BMI088_GYRO_CS_LOW();
    spi_read_write_byte(reg);
    spi_read_write_byte(data);
    BMI088_GYRO_CS_HIGH();
}

static uint8_t bmi088_accel_read_chip_id(void)
{
    uint8_t id = 0;
    bmi088_accel_read_reg(BMI088_ACC_CHIP_ID_ADDR, &id, 1);
    return id;
}

static uint8_t bmi088_gyro_read_chip_id(void)
{
    uint8_t id = 0;
    bmi088_gyro_read_reg(BMI088_GYRO_CHIP_ID_ADDR, &id, 1);
    return id;
}

static void bmi088_accel_soft_reset(void)
{
    bmi088_accel_write_reg(BMI088_ACC_SOFTRESET_ADDR, 0xB6);
    HAL_Delay(80);
}

static void bmi088_gyro_soft_reset(void)
{
    bmi088_gyro_write_reg(BMI088_GYRO_SOFTRESET_ADDR, 0xB6);
    HAL_Delay(30);
}

static void bmi088_accel_set_conf(uint8_t odr, uint8_t bwp)
{
    bmi088_accel_write_reg(BMI088_ACC_CONF_ADDR, (bwp << 4) | odr);
    HAL_Delay(5);
}

static void bmi088_accel_set_range(uint8_t range)
{
    bmi088_accel_write_reg(BMI088_ACC_RANGE_ADDR, range);
    HAL_Delay(5);
}

static void bmi088_accel_enable(void)
{
    bmi088_accel_write_reg(BMI088_ACC_PWR_CTRL_ADDR, 0x04);
    HAL_Delay(50);
    bmi088_accel_write_reg(BMI088_ACC_PWR_CONF_ADDR, 0x00);
    HAL_Delay(5);
}

static void bmi088_gyro_set_range(uint8_t range)
{
    bmi088_gyro_write_reg(BMI088_GYRO_RANGE_ADDR, range);
    HAL_Delay(5);
}

static void bmi088_gyro_set_bandwidth(uint8_t bw)
{
    bmi088_gyro_write_reg(BMI088_GYRO_BANDWIDTH_ADDR, bw);
    HAL_Delay(5);
}

uint8_t bmi088_init(void)
{
    extern SPI_HandleTypeDef hspi1;
    bmi088_spi = &hspi1;

    BMI088_ACC_CS_HIGH();
    BMI088_GYRO_CS_HIGH();
    HAL_Delay(100);

    bmi088_accel_read_chip_id();
    HAL_Delay(1);
    bmi088_data.accel_chip_id = bmi088_accel_read_chip_id();
    HAL_Delay(1);

    bmi088_accel_soft_reset();

    bmi088_accel_read_chip_id();
    HAL_Delay(1);
    bmi088_data.accel_chip_id = bmi088_accel_read_chip_id();
    HAL_Delay(1);

    if (bmi088_data.accel_chip_id != BMI088_ACC_CHIP_ID_VALUE) {
        return 1;
    }

    bmi088_gyro_read_chip_id();
    HAL_Delay(1);
    bmi088_data.gyro_chip_id = bmi088_gyro_read_chip_id();
    HAL_Delay(1);

    bmi088_gyro_soft_reset();

    bmi088_gyro_read_chip_id();
    HAL_Delay(1);
    bmi088_data.gyro_chip_id = bmi088_gyro_read_chip_id();
    HAL_Delay(1);

    if (bmi088_data.gyro_chip_id != BMI088_GYRO_CHIP_ID_VALUE) {
        return 2;
    }

    bmi088_accel_set_conf(BMI088_ACC_ODR_400, BMI088_ACC_BWP_NORMAL);
    bmi088_accel_set_range(BMI088_ACC_RANGE_6G);
    bmi088_accel_enable();

    bmi088_gyro_set_range(BMI088_GYRO_RANGE_2000);
    bmi088_gyro_set_bandwidth(BMI088_GYRO_BW_23_ODR_200);

    return 0;
}

void bmi088_accel_read(void)
{
    uint8_t buf[6];
    int16_t raw_x, raw_y, raw_z;

    bmi088_accel_read_reg(BMI088_ACC_X_LSB_ADDR, buf, 6);

    raw_x = (int16_t)((buf[1] << 8) | buf[0]);
    raw_y = (int16_t)((buf[3] << 8) | buf[2]);
    raw_z = (int16_t)((buf[5] << 8) | buf[4]);

    bmi088_data.accel.x = raw_x * BMI088_ACC_6G_SEN;
    bmi088_data.accel.y = raw_y * BMI088_ACC_6G_SEN;
    bmi088_data.accel.z = raw_z * BMI088_ACC_6G_SEN;
}

void bmi088_gyro_read(void)
{
    uint8_t buf[6];
    int16_t raw_x, raw_y, raw_z;

    bmi088_gyro_read_reg(BMI088_GYRO_X_LSB_ADDR, buf, 6);

    raw_x = (int16_t)((buf[1] << 8) | buf[0]);
    raw_y = (int16_t)((buf[3] << 8) | buf[2]);
    raw_z = (int16_t)((buf[5] << 8) | buf[4]);

    bmi088_data.gyro.x = raw_x * BMI088_GYRO_2000_SEN;
    bmi088_data.gyro.y = raw_y * BMI088_GYRO_2000_SEN;
    bmi088_data.gyro.z = raw_z * BMI088_GYRO_2000_SEN;
}

void bmi088_read_all(void)
{
    bmi088_accel_read();
    bmi088_gyro_read();
}