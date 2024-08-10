#include "esp_err.h"

#include "usei2c.h"

#define MPU6050_LOG_TAG "MPU6050"

/* I2C */
#define MPU6050_PIN_SCL 21
#define MPU6050_PIN_SDA 22
#define MPU6050_PIN_CS 23
#define MPU6050_FREQ 400000

/* MPU6050 BASIC REGS */
#define MPU6050_ADDR 0x68
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_PWR_MGMT_2 0x6C
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_GYRO_CONFIG 0x1B

#define MPU6050_ACCEL_OUT 0x3B
#define MPU6050_GYRO_OUT 0x43
#define MPU6050_TEMP_OUT 0x41

/* DMP REGS */
#define MPU6050_BANK_SEL 0x6D
#define MPU6050_MEM_START_ADDR 0x6E
#define MPU6050_MEM 0x6F
#define MPU6050_XG_OFFS_TC 0x00

typedef struct {
    i2c_master_bus_config_t bus_cfg;
    i2c_device_config_t dev_cfg;
} mpu6050_t;

esp_err_t mpu6050_dmp_init(void);

/*
 * @brief Read mpu6050 temperature.
 *
 * @return
 *      - temp: Double type value of temperature if read success.
 *              Return -404.0 if read failed.
 */
double mpu6050_read_temp(void);

/*
 * @brief Read raw gyroscope data from mpu6050.
 *
 * @param[out] gyro_data_raw: Buffer to store the data read from mpu6050.
 *
 * @return
 *      - ESP_OK: I2C master transmit-receive success
 *      - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid.
 *      - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms)
 *                         because the bus is busy or hardware crash.
 */
esp_err_t mpu6050_read_gyro_raw(uint16_t *gyro_data_raw);

/*
 * @brief Read raw accelerometer data from mpu6050.
 *
 * @param[out] accel_raw_data: Buffer to store the data read from mpu6050.
 *
 * @return
 *      - ESP_OK: I2C master transmit-receive success
 *      - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid.
 *      - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms)
 *                         because the bus is busy or hardware crash.
 */
esp_err_t mpu6050_read_accel_raw(uint16_t *accel_data_raw);

/*
 * @brief Enable mpu6050 with register command.
 *
 * @note You can edit the command value according your need, also you can add
 * some others register value to enable the mpu6050.
 *
 * @return
 *      - ESP_OK: I2C master transmit success
 *      - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid.
 *      - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms)
 *                         because the bus is busy or hardware crash.
 * */
esp_err_t mpu6050_enable(void);

/*
 * @brief Initialize the mpu6050 connected with esp32 chip.
 *
 * @note In the initialize function, you have to set bus_handle and dev_handle
 * as NULL before running usei2c_init() function. after usei2c_init() function
 * running, you have to set: bus_handle = usei2c_cfg.bus_handle dev_handle =
 * usei2c_cfg.dev_handle
 *
 * @param[in] mpu6050: The I2C configurations to initialize the mpu6050.
 *
 * @return
 *      - ESP_OK: Create I2C master device successfully.
 *      - ESP_ERR_INVALID_ARG: I2C bus initialization failed because of invalid
 * argument.
 *      - ESP_ERR_NO_MEM: Create I2C bus failed because of out of memory.
 */
esp_err_t mpu6050_init(mpu6050_t *mpu6050);
