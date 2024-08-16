#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "mpu6050.h"

void app_main() {
  mpu6050_init();
  mpu6050_enable();

  mpu6050_t mpu6050;

  while (true) {
    mpu6050_read_accel_raw(&mpu6050);
    mpu6050_read_gyro_raw(&mpu6050);

    mpu6050_kalman(&mpu6050);

    // printf("ax: %f\tay: %f\taz: %f\n", mpu6050.ax, mpu6050.ay, mpu6050.az);
    printf("X: %f\tY: %f\n", mpu6050.kalman_angle_x, mpu6050.kalman_angle_y);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
