#include <string.h>

#include "mpu6050_6Axis.h"
#include "mpu6050_dmp_data.h"

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

int8_t mpu6050_get_x_gyro_offset_tc() {
    return (int8_t)usei2c_read_bits(MPU6050_XG_OFFS_TC, 6, 6);
}

int8_t mpu6050_get_y_gyro_offset_tc(void) {
    return (int8_t)usei2c_read_bits(MPU6050_YG_OFFS_TC, 6, 6);
}

int8_t mpu6050_get_z_gyro_offset_tc(void) {
    return (int8_t)usei2c_read_bits(MPU6050_ZG_OFFS_TC, 6, 6);
}

bool mpu6050_write_prog_mem_block(const uint8_t *data, uint16_t data_size, uint8_t bank,
                                  uint8_t address, bool verify, bool use_prog_mem) {
    mpu6050_set_memory_bank(bank, false, false);
    mpu6050_set_memory_start_address(address);

    uint8_t chunkSize;
    uint8_t *verifyBuffer = 0;
    uint8_t *progBuffer = 0;
    uint16_t i;
    uint8_t j;

    if (verify) verifyBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    if (use_prog_mem) progBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);

    for (i = 0; i < data_size;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > data_size) chunkSize = data_size - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;

        if (use_prog_mem) {
            // write the chunk of data as specified
            for (j = 0; j < chunkSize; j++) progBuffer[j] = pgm_read_byte(data + i + j);
        } else {
            // write the chunk of data as specified
            progBuffer = (uint8_t *)data + i;
        }

        usei2c_write_reg_burst(MPU6050_MEM, progBuffer, chunkSize);

        // verify data if needed
        if (verify && verifyBuffer) {
            printf("VERIFY\n");
            mpu6050_set_memory_bank(bank, false, false);
            mpu6050_set_memory_start_address(address);
            usei2c_read_reg_burst(MPU6050_MEM, verifyBuffer, chunkSize);
            if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0) {
                printf("Block write verification error, bank \n");
                /*Serial.print("Block write verification error, bank ");*/
                printf("bank %d", bank);
                printf(", address ");
                printf("%d", address);
                printf("!\nExpected:");
                for (j = 0; j < chunkSize; j++) {
                    printf("%#04x", progBuffer[j]);
                }
                printf("\nReceived:");
                for (uint8_t j = 0; j < chunkSize; j++) {
                    printf("%#04x", verifyBuffer[i + j]);
                }
                printf("\n");
                // free(verifyBuffer);
                // if (useProgMem) free(progBuffer);
                // return false; // uh oh.
            }
        }

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < data_size) {
            if (address == 0) bank++;
            mpu6050_set_memory_bank(bank, false, false);
            mpu6050_set_memory_start_address(address);
        }
    }

    if (verify) free(verifyBuffer);
    if (use_prog_mem) free(progBuffer);

    return true;
}
