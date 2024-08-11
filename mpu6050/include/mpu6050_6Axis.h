#include <stdbool.h>

#include "esp_err.h"

#define SET 0
#define GET 1
#define UNSET -1

#define pgm_read_byte(addr) (*(const unsigned char *)(addr))

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

/*
 * @brief Read or write register in the memory bank.
 *
 * @note What is memory bank?
 *       - In MPU6050, the available register space is only 128 bytes, but the
 *         number of registers exceeds this space. To manage this, the MPU6050
 *         uses memory banks to extend the addressable register space, allow in
 *         to access to a larger number of registers.
 *       How to read/write registers in a memory bank?
 *       - Each memory bank is a 256-byte memory space that includes register
 *         addresses, and their corresponding data. To read or write a register
 *         within a memory bank, you must fisrt select the correct memory bank
 *         and then access the register at its offset within that bank.
 *       How to select the memory bank?
 *       - To read from or write to different memory banks, you need to
 *         configure the BANK_SEL register.
 *       - The MPU6050 supports 32 memory banks, numbered from 0 to 31.
 *       - After selecting the correct memory bank, use the offset address to
 *         access the desired register within the bank.
 *       - Note: After selecting a memory bank, always use the offset address
 *         within that bank rather than an absolute address.
 *
 * @param[in] bank: Memory bank number to select.
 * @param[in] prefetch_enabled: Whether to enable prefetching.
 * @param[in] user_bank: Whether to select the user memory bank.
 */
void mpu6050_set_memory_bank(uint8_t bank, bool prefetch_enabled, bool user_bank);

/*
 * @brief Set memory offset in order to use memory bank for exceeds registers.
 *
 * @note Once you set offset, you have to use the offset address while read/write
 *       registers.
 *
 * @param[in] address: Offset of memory.
 */
void mpu6050_set_memory_start_address(uint8_t address);

/*
 * @brief Read 1 byte of data from MPU6050_MEM.
 *
 * @return
 *      - return uint8_t data.
 */
uint8_t mpu6050_read_memory(void);

/*
 * @brief Get whether the otp bank is valid.
 *
 * @return
 *      - return true if otp bank is valid.
 *        return false if otp bank is invalid.
 */
bool mpu6050_get_otp_bank_valid(void);

/*
 * @brief Return offset of x axis gyroscope.
 */
int8_t mpu6050_get_x_gyro_offset_tc(void);

/*
 * @brief Return offset of y axis gyroscope.
 */
int8_t mpu6050_get_y_gyro_offset_tc(void);

/*
 * @brief Return offset of z axis gyroscope.
 */
int8_t mpu6050_get_z_gyro_offset_tc(void);

/* @brief Write a data block to mpu6050.
 *
 * @param[in] data: Data buffer write into mpu6050.
 * @param[in] data_size: Size of data buffer.
 * @param[in] bank: Which memory bank will write into.
 * @param[in] offset_address: Offset address.
 *
 * @return
 *      - Return true if write and verify success.
 */
bool mpu6050_write_memory_block(uint8_t *data, uint8_t data_size, uint8_t bank,
                                uint8_t offset_address);
