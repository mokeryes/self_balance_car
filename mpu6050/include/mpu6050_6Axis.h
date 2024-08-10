#include "esp_err.h"

#include "mpu6050.h"

#define SET 0
#define GET 1
#define UNSET -1

/*
 * @brief Reset MPU6050 for initialize the DMP.
 */
void mpu6050_reset(void);

/*
 * @brief Set, get or unset the PWR_MGMT_1 register's sleep mode.
 *
 * @note This function will get actual mode whatever the parameter is.
 *
 * @param[in] mode: SET, GET, UNSET
 *
 * @return
 *      - Sleep mode
 */
bool mpu6050_mode_sleep(int mode);

void mpu6050_set_memory_bank(uint8_t bank, bool prefetch_enabled,
                             bool user_bank);

void mpu6050_set_memory_start_address(uint8_t address);

uint8_t mpu6050_read_memory(void);

bool get_otp_bank_valid(void);
