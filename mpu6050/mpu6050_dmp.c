#include "mpu6050_dmp.h"
#include <stdio.h>
#include "freertos/projdefs.h"
#include "imu/inv_imu_driver.h"

static int read_reg(struct inv_imu_serif *serif, uint8_t reg, uint8_t *buf, uint32_t len) {
    esp_err_t ret = ESP_FAIL;

    ret = usei2c_read_reg_burst(reg, buf, (size_t)len);

    return ret;
}

static int write_reg(struct inv_imu_serif *serif, uint8_t reg, const uint8_t *buf,
                     uint32_t len) {
    esp_err_t ret = ESP_FAIL;

    ret = usei2c_write_reg_burst(reg, buf, (size_t)len);

    return ret;
}

static void sensor_event_cb(inv_imu_sensor_event_t *event) {}

esp_err_t mpu6050_dmp_init() {
    esp_err_t ret = ESP_FAIL;

    usei2c_config_t usei2c_cfg = {
        .bus_cfg.i2c_port = -1,
        .bus_cfg.sda_io_num = MPU6050_PIN_SDA,
        .bus_cfg.scl_io_num = MPU6050_PIN_SCL,
        .bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT,
        .bus_cfg.glitch_ignore_cnt = 7,

        .dev_cfg.dev_addr_length = I2C_ADDR_BIT_7,
        .dev_cfg.device_address = MPU6050_ADDR,
        .dev_cfg.scl_speed_hz = MPU6050_FREQ,
    };
    ret = usei2c_init(&usei2c_cfg);

    uint8_t whoami = usei2c_read_reg(MPU6050_WHO_AM_I);
    if (whoami == MPU6050_ADDR) {
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP_LOGI(MPU6050_LOG_TAG, "MPU6050 is online.");
    } else {
        ret = ESP_FAIL;
        ESP_LOGW(MPU6050_LOG_TAG, "MPU6050 is not online, error code: %s.",
                 esp_err_to_name(ret));
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    inv_imu_device_t imu_dev;
    inv_imu_serif_t imu_serif = {
        /* Initialize serial interface */
        .read_reg = read_reg, .write_reg = write_reg, .max_read = 32768,
        .max_write = 32768,   .serif_type = UI_I2C,   .context = 0,
    };

    /* Init device */
    ret = inv_imu_init(&imu_dev, &imu_serif, NULL);

    return ret;
}
