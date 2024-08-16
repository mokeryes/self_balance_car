#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "esp_timer.h"

#include "mpu6050.h"

static Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

static Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

static uint32_t timer;

static const float accel_z_corrector = 14418.0;

float Kalman_getAngle(Kalman_t *Kalman, float newAngle, float newRate,
                      float dt) {
  float rate = newRate - Kalman->bias;
  Kalman->angle += dt * rate;

  Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] -
                           Kalman->P[1][0] + Kalman->Q_angle);
  Kalman->P[0][1] -= dt * Kalman->P[1][1];
  Kalman->P[1][0] -= dt * Kalman->P[1][1];
  Kalman->P[1][1] += Kalman->Q_bias * dt;

  float S = Kalman->P[0][0] + Kalman->R_measure;
  float K[2];
  K[0] = Kalman->P[0][0] / S;
  K[1] = Kalman->P[1][0] / S;

  float y = newAngle - Kalman->angle;
  Kalman->angle += K[0] * y;
  Kalman->bias += K[1] * y;

  float P00_temp = Kalman->P[0][0];
  float P01_temp = Kalman->P[0][1];

  Kalman->P[0][0] -= K[0] * P00_temp;
  Kalman->P[0][1] -= K[0] * P01_temp;
  Kalman->P[1][0] -= K[1] * P00_temp;
  Kalman->P[1][1] -= K[1] * P01_temp;

  return Kalman->angle;
};

void mpu6050_kalman(mpu6050_t *mpu6050) {
  // Kalman angle solve
  float dt = (float)((float)esp_timer_get_time() / 1000.0);
  timer = esp_timer_get_time();
  float roll;
  float roll_sqrt = sqrt(mpu6050->accel_x_raw * mpu6050->accel_x_raw +
                         mpu6050->accel_z_raw * mpu6050->accel_z_raw);
  if (roll_sqrt != 0.0) {
    roll = atan(mpu6050->accel_y_raw / roll_sqrt) * RAD_TO_DEG;
  } else {
    roll = 0.0;
  }
  float pitch = atan2(-mpu6050->accel_x_raw, mpu6050->accel_z_raw) * RAD_TO_DEG;
  if ((pitch < -90 && mpu6050->kalman_angle_y > 90) ||
      (pitch > 90 && mpu6050->kalman_angle_y < -90)) {
    KalmanY.angle = pitch;
    mpu6050->kalman_angle_y = pitch;
  } else {
    mpu6050->kalman_angle_y = Kalman_getAngle(&KalmanY, pitch, mpu6050->gy, dt);
  }
  if (fabs(mpu6050->kalman_angle_y) > 90)
    mpu6050->gx = -mpu6050->gx;
  mpu6050->kalman_angle_x = Kalman_getAngle(&KalmanX, roll, mpu6050->gy, dt);
}

esp_err_t mpu6050_read_temp(mpu6050_t *mpu6050) {
  float temp = -404.0f;

  uint8_t buffer[2] = {0x00};

  esp_err_t ret = usei2c_read_reg_burst(MPU6050_TEMP_OUT, buffer, 2);
  if (ret != ESP_OK) {
    ESP_LOGE(MPU6050_LOG_TAG, "Read temperature data error, error code: %s.",
             esp_err_to_name(ret));
    return ret;
  }

  int16_t temp_raw = (buffer[0] << 8) | buffer[1];

  temp = (float)(temp_raw / (float)340.0) + (float)36.53;

  mpu6050->temperature = temp;

  return ret;
}

esp_err_t mpu6050_read_gyro_raw(mpu6050_t *mpu6050) {
  esp_err_t ret = ESP_FAIL;

  uint8_t buffer[6] = {0x00};

  ret = usei2c_read_reg_burst(MPU6050_GYRO_OUT, buffer, 6);
  if (ret != ESP_OK) {
    ESP_LOGE(MPU6050_LOG_TAG, "Read gyroscope data error, error code: %s.",
             esp_err_to_name(ret));
  }
  mpu6050->gyro_x_raw = (int16_t)(buffer[0] << 8) | buffer[1];
  mpu6050->gyro_y_raw = (int16_t)(buffer[2] << 8) | buffer[3];
  mpu6050->gyro_z_raw = (int16_t)(buffer[4] << 8) | buffer[5];

  mpu6050->gx = mpu6050->accel_x_raw / 131.0;
  mpu6050->gy = mpu6050->accel_y_raw / 131.0;
  mpu6050->gz = mpu6050->accel_z_raw / 131.0;

  return ret;
}

esp_err_t mpu6050_read_accel_raw(mpu6050_t *mpu6050) {
  esp_err_t ret = ESP_FAIL;

  uint8_t buffer[6] = {0x00};

  ret = usei2c_read_reg_burst(MPU6050_ACCEL_OUT, buffer, 6);
  if (ret != ESP_OK) {
    ESP_LOGE(MPU6050_LOG_TAG, "Read accelerometer data error, error code: %s.",
             esp_err_to_name(ret));
  }
  mpu6050->accel_x_raw = (int16_t)(buffer[0] << 8) | buffer[1];
  mpu6050->accel_y_raw = (int16_t)(buffer[2] << 8) | buffer[3];
  mpu6050->accel_z_raw = (int16_t)(buffer[4] << 8) | buffer[5];

  mpu6050->ax = mpu6050->accel_x_raw / 16384.0;
  mpu6050->ay = mpu6050->accel_y_raw / 16384.0;
  mpu6050->az = mpu6050->accel_z_raw / accel_z_corrector;

  return ret;
}

esp_err_t mpu6050_enable(void) {
  esp_err_t ret = ESP_FAIL;

  ret = usei2c_write_reg(MPU6050_PWR_MGMT_1, 0x01);
  ret = usei2c_write_reg(MPU6050_SMPLRT_DIV, 0x07);
  ret = usei2c_write_reg(MPU6050_CONFIG, 0x06);
  ret = usei2c_write_reg(MPU6050_ACCEL_CONFIG, 0x00);
  ret = usei2c_write_reg(MPU6050_GYRO_CONFIG, 0x00);

  if (ret == ESP_OK) {
    ESP_LOGI(MPU6050_LOG_TAG, "MPU6050 enabled.");
  } else {
    ESP_LOGE(MPU6050_LOG_TAG, "MPU6050 enabling error, error code: %s.",
             esp_err_to_name(ret));
  }

  return ret;
}

esp_err_t mpu6050_init() {
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

  return ret;
}
