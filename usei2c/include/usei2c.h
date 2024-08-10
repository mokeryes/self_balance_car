#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"

#define USEI2C_LOG_TAG "USEI2C"

typedef struct {
    i2c_master_bus_config_t bus_cfg;
    i2c_master_bus_handle_t bus_handle;

    i2c_device_config_t dev_cfg;
    i2c_master_dev_handle_t dev_handle;

} usei2c_config_t;

/*
 * @brief Read multiple data from I2C slave device register.
 *
 * @param[in] reg_addr: Address of the register in the slave device.
 * @param[out] data: Buffer to store the data read from the register.
 * @param[in] data_len: Length of data read from register.
 *
 * @return
 *      - ESP_OK: I2C master transmit-receive success
 *      - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid.
 *      - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms)
 *                         because the bus is busy or hardware crash.
 * */
esp_err_t usei2c_read_reg_burst(uint8_t reg_addr, uint8_t *data, size_t data_len);

/*
 * @brief Write multiple data into I2C slave device register.
 *
 * @note This function will overwrite all bits in the register before
 *
 * @param[in] reg_addr: Address of register.
 * @param[in] data: Data buffer to write into the register.
 * @param[in] data_len: Length of data you want to write into the register.
 *
 * @return
 *      - ESP_OK: I2C master transmit success
 *      - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid.
 *      - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms)
 *                         because the bus is busy or hardware crash.
 * */
esp_err_t usei2c_write_reg_burst(uint8_t reg_addr, const uint8_t *data, size_t data_len);

/*
 * @brief Read one byte of data from I2C slave device.
 *
 * @param[in] reg_addr: Address of register.
 *
 * @return
 *      - buffer: Return the value read from I2C device.
 */
uint8_t usei2c_read_reg(uint8_t reg_addr);

/*
 * @brief Write one byte of data into the device register.
 *
 * @note This function will overwrite all the bits on the register.
 *
 * @param[in] reg_addr: Address of register.
 * @param[in] data: One byte data to write into the register.
 *
 * @return
 *      - ESP_OK: I2C master transmit success
 *      - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid.
 *      - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms)
 *                         because the bus is busy or hardware crash.
 * */
esp_err_t usei2c_write_reg(uint8_t reg_addr, uint8_t data);

/*
 * @brief Read specified bits from register.
 *
 * @note The bits_start parameter is start from 0 to 7, range: [0, 7].
 *
 * @param[in] reg_addr: Address of register.
 * @param[in] bits_start: Read bits from bits_start.
 * @param[in] bits_len: Length of bits to read.
 *
 * @return
 *      - bits_data: Specified bits data from register.
 */
uint8_t usei2c_read_bits(uint8_t reg_addr, uint8_t bits_start, uint8_t bits_len);

/*
 * @brief Write data into register with specified bits.
 *
 * @note This function will save bits except which bits will write.
 *       The bits_start parameter is start from 0 to 7, range: [0, 7].
 *
 *
 * @param[in] reg_addr: Address of register.
 * @param[in] bits_data: The bits data ready to write into the register.
 * @param[in] bits_start: Write bits data from this position.
 * @param[in] bits_len: How many bits will write into the register.
 *
 * @return
 *      - ESP_OK: I2C master transmit success
 *      - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid.
 *      - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms)
 *                         because the bus is busy or hardware crash.
 * */
esp_err_t usei2c_write_bits(uint8_t reg_addr, uint8_t bits_data, uint8_t bits_start,
                            uint8_t bits_len);

/*
 * @brief Read one bit to specified register.
 *
 * @param[in] reg_addr: Address of register.
 * @param[in] bit_num: Bit refer to read.
 *            The bit_num parameter is start from 0 to 7, range: [0, 7].
 *
 * @return
 *      - data: The value of the specified bit (0 or 1).
 */
uint8_t usei2c_read_bit(uint8_t reg_addr, uint8_t bit_num);

/*
 * @brief Write one bit to specified register.
 *
 * @param[in] reg_addr: Address of register.
 * @param[in] bit_num: Position of the bit to write (0-indexed).
 *            The bit_num parameter is start from 0 to 7, range: [0, 7].
 * @param[in] bit_data: The value of the bit to write (0 or 1).
 *
 * @return
 *      - ESP_OK: I2C master transmit success
 *      - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid.
 *      - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms)
 *                         because the bus is busy or hardware crash.
 */
esp_err_t usei2c_write_bit(uint8_t reg_addr, uint8_t bit_num, uint8_t bit_data);

/*
 * @brief Initialize I2C with usei2c_cfg_t configuration.
 *
 * @param[in] usei2c_cfg: configurations for usei2c initialize.
 *
 * @return
 *      - ESP_OK: I2C master bus initialized successfully.
 *      - ESP_ERR_INVALID_ARG: I2C bus initialization failed because of invalid argument.
 *      - ESP_ERR_NO_MEM: Create I2C bus failed because of out of memory.
 *      - ESP_ERR_NOT_FOUND: No more free bus.
 */
esp_err_t usei2c_init(usei2c_config_t *usei2c_cfg);
