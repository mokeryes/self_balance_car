#include <stdbool.h>
#include <stdio.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"

#include "mpu6050_6Axis.h"

esp_err_t mpu6050_dmp_init(void) {
    esp_err_t ret = ESP_FAIL;

    /* Reset mpu6050 */
    mpu6050_enable();

    /* Disable sleep mode */
    mpu6050_mode_sleep(UNSET);

    /* Get MPU hardware revision */
    mpu6050_set_memory_bank(0x10, true, true);  // Select number of 0x10 memory bank
    mpu6050_set_memory_start_address(0x06);     // Set offset is 0x06
    mpu6050_read_memory();  // Read data from memory bank 0x10 offset 0x06
    mpu6050_set_memory_bank(0x00, false, false);  // Select numbder of 0x00 memory bank

    /* Check OTP bank valid */
    mpu6050_get_otp_bank_valid() == true ? printf("OTP valid.\n")
                                         : printf("OTP invalid.\n");

    /* Get X/Y/Z gyroscope offset */
    printf("%d\n", mpu6050_get_x_gyro_offset_tc());
    printf("%d\n", mpu6050_get_y_gyro_offset_tc());
    printf("%d\n", mpu6050_get_z_gyro_offset_tc());

    return ret;
}

double mpu6050_read_temp(void) {
    double temp = -404.0f;

    uint8_t buffer[2] = {0x00};

    esp_err_t ret = usei2c_read_reg_burst(MPU6050_TEMP_OUT, buffer, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(MPU6050_LOG_TAG, "Read temperature data error, error code: %s.",
                 esp_err_to_name(ret));
        return temp;
    }

    int16_t temp_raw = (buffer[0] << 8) | buffer[1];

    temp = (double)(temp_raw / 340.0) + 36.53;

    return temp;
}

esp_err_t mpu6050_read_gyro_raw(uint16_t *gyro_data_raw) {
    esp_err_t ret = ESP_FAIL;

    uint8_t buffer[6] = {0x00};

    ret = usei2c_read_reg_burst(MPU6050_GYRO_OUT, buffer, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(MPU6050_LOG_TAG, "Read gyroscope data error, error code: %s.",
                 esp_err_to_name(ret));
    }
    gyro_data_raw[0] = (buffer[0] << 8) | buffer[1];
    gyro_data_raw[1] = (buffer[2] << 8) | buffer[3];
    gyro_data_raw[2] = (buffer[4] << 8) | buffer[5];

    return ret;
}

esp_err_t mpu6050_read_accel_raw(uint16_t *accel_data_raw) {
    esp_err_t ret = ESP_FAIL;

    uint8_t buffer[6] = {0x00};

    ret = usei2c_read_reg_burst(MPU6050_ACCEL_OUT, buffer, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(MPU6050_LOG_TAG, "Read accelerometer data error, error code: %s.",
                 esp_err_to_name(ret));
    }
    accel_data_raw[0] = (buffer[0] << 8) | buffer[1];
    accel_data_raw[1] = (buffer[2] << 8) | buffer[3];
    accel_data_raw[2] = (buffer[4] << 8) | buffer[5];

    return ret;
}

esp_err_t mpu6050_enable(void) {
    esp_err_t ret = ESP_FAIL;

    ret = usei2c_write_reg(MPU6050_PWR_MGMT_1, 0x01);
    ret = usei2c_write_reg(MPU6050_SMPLRT_DIV, 0x07);
    ret = usei2c_write_reg(MPU6050_CONFIG, 0x06);
    ret = usei2c_write_reg(MPU6050_ACCEL_CONFIG, 0x01);
    ret = usei2c_write_reg(MPU6050_GYRO_CONFIG, 0x18);

    if (ret == ESP_OK) {
        ESP_LOGI(MPU6050_LOG_TAG, "MPU6050 enabled.");
    } else {
        ESP_LOGE(MPU6050_LOG_TAG, "MPU6050 enabling error, error code: %s.",
                 esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t mpu6050_init(mpu6050_t *mpu6050) {
    esp_err_t ret = ESP_FAIL;

    i2c_master_bus_handle_t bus_handle = NULL;
    i2c_master_dev_handle_t dev_handle = NULL;
    usei2c_config_t usei2c_cfg = {
        .bus_cfg = mpu6050->bus_cfg,
        .bus_handle = bus_handle,

        .dev_cfg = mpu6050->dev_cfg,
        .dev_handle = dev_handle,
    };
    ret = usei2c_init(&usei2c_cfg);
    bus_handle = usei2c_cfg.bus_handle;
    dev_handle = usei2c_cfg.dev_handle;

    uint8_t whoami = usei2c_read_reg(MPU6050_WHO_AM_I);
    if (whoami == MPU6050_ADDR) {
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP_LOGI(MPU6050_LOG_TAG, "MPU6050 is online.");
    } else {
        ret = ESP_FAIL;
        ESP_LOGW(MPU6050_LOG_TAG, "MPU6050 is not online, error code: %s.",
                 esp_err_to_name(ret));
    }

    return ret;
}
