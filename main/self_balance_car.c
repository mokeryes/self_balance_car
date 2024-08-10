#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/projdefs.h"
#include "mpu6050.h"

void app_main(void) {
    mpu6050_t mpu6050 = {
        .bus_cfg.i2c_port = -1,
        .bus_cfg.sda_io_num = MPU6050_PIN_SDA,
        .bus_cfg.scl_io_num = MPU6050_PIN_SCL,
        .bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT,
        .bus_cfg.glitch_ignore_cnt = 7,

        .dev_cfg.dev_addr_length = I2C_ADDR_BIT_7,
        .dev_cfg.device_address = MPU6050_ADDR,
        .dev_cfg.scl_speed_hz = MPU6050_FREQ,
    };
    mpu6050_init(&mpu6050);
    mpu6050_enable();
    uint16_t accel_data_raw[3] = {0x00};
    while (true) {
        mpu6050_read_gyro_raw(accel_data_raw);
        printf("%u, %u, %u\n", accel_data_raw[0], accel_data_raw[1], accel_data_raw[2]);
        printf("%f\n", mpu6050_read_temp());
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
