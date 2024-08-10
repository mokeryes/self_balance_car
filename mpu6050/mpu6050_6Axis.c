#include "mpu6050_6Axis.h"

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

bool get_otp_bank_valid() {
    return (usei2c_read_bit(MPU6050_XG_OFFS_TC, 0) ? true : false);
}
