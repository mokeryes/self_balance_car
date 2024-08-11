#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "mpu6050_6Axis.h"
#include "esp_log.h"
#include "mpu6050.h"

void mpu6050_reset(void) { mpu6050_enable(); }

bool mpu6050_mode_sleep(int mode) {
    bool state = false;

    if (mode == SET) {
        state = usei2c_write_bit(MPU6050_PWR_MGMT_1, 6, 1) == ESP_OK ? true : false;
    } else if (mode == GET) {
        state = usei2c_read_bit(MPU6050_PWR_MGMT_1, 6) == 1 ? true : false;
    } else if (mode == UNSET) {
        usei2c_write_bit(MPU6050_PWR_MGMT_1, 6, 0);
        state = usei2c_read_bit(MPU6050_PWR_MGMT_1, 6) == 1 ? true : false;
    }

    return state;
}

void mpu6050_set_memory_bank(uint8_t bank, bool prefetch_enabled, bool user_bank) {
    bank &= 0x1F;
    if (user_bank) {
        bank |= 0x20;
    }
    if (prefetch_enabled) {
        bank |= 0x40;
    }
    usei2c_write_reg(MPU6050_BANK_SEL, bank);
}

void mpu6050_set_memory_start_address(uint8_t address) {
    usei2c_write_reg(MPU6050_MEM_START_ADDR, address);
}

uint8_t mpu6050_read_memory() { return usei2c_read_reg(MPU6050_MEM); }

bool mpu6050_get_otp_bank_valid() {
    return (usei2c_read_bit(MPU6050_XG_OFFS_TC, 0) ? true : false);
}

int8_t mpu6050_get_x_gyro_offset_tc() { return (int8_t)usei2c_read_bits(MPU6050_XG_OFFS_TC, 6, 6); }

int8_t mpu6050_get_y_gyro_offset_tc(void) {
    return (int8_t)usei2c_read_bits(MPU6050_YG_OFFS_TC, 6, 6);
}

int8_t mpu6050_get_z_gyro_offset_tc(void) {
    return (int8_t)usei2c_read_bits(MPU6050_ZG_OFFS_TC, 6, 6);
}

static bool mpu6050_write_mem_block_verify(uint8_t bank, uint8_t address, uint8_t *prog_buffer,
                                           uint8_t chunk_size) {
    return true;
}

bool mpu6050_write_memory_block(uint8_t *data, uint8_t data_size, uint8_t bank,
                                uint8_t offset_address) {
    return true;
}
