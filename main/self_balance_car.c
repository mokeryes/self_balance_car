#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"

#include "mpu6050.h"

void app_main(void) {
    mpu6050_init();
    mpu6050_enable();
}
