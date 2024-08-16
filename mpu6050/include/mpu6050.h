#include "esp_err.h"

#include "usei2c.h"
#include "imu/inv_imu.h"

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

#define RAD_TO_DEG 57.29577951308232087679815481410

typedef struct {
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    float ax;
    float ay;
    float az;

    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    float gx;
    float gy;
    float gz;

    float temperature;

    float kalman_angle_x;
    float kalman_angle_y;
} mpu6050_t;

typedef struct {
    float Q_angle;
    float Q_bias;
    float R_measure;
    float angle;
    float bias;
    float P[2][2];
} Kalman_t;

/* @brief For kalman calculation.
 *
 * @param[in] Kalman: data.
 * @param[in] newAngle: New angle data for update.
 * @param[in] newRate: New rate data for update.
 * @param[in] dt: Dt time.
 *
 * @return:
 *      - return kalman angle value calculated.
 */
float Kalman_getAngle(Kalman_t *Kalman, float newAngle, float newRate, float dt);

/*
 * @brief Solve kalman calculation.
 *
 * @param[in] The mpu6050 structure contains accelerometer data and gyroscope
 *            data.
 */
void mpu6050_kalman(mpu6050_t *mpu6050);

/*
 * @brief Read mpu6050 temperature.
 *
 * @return
 *      - temp: float type value of temperature if read success.
 *              Return -404.0 if read failed.
 */
esp_err_t mpu6050_read_temp(mpu6050_t *mpu6050);

/*
 * @brief Read raw gyroscope data from mpu6050.
 *
 * @param[out] mpu6050: Struct to store the data read from mpu.
 *
 * @return
 *      - ESP_OK: I2C master transmit-receive success
 *      - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid.
 *      - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms)
 *                         because the bus is busy or hardware crash.
 */
esp_err_t mpu6050_read_gyro_raw(mpu6050_t *mpu6050);

/*
 * @brief Read raw accelerometer data from mpu6050.
 *
 * @param[out] mpu6050: Struct to store the data read from mpu.
 *
 * @return
 *      - ESP_OK: I2C master transmit-receive success
 *      - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid.
 *      - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms)
 *                         because the bus is busy or hardware crash.
 */
esp_err_t mpu6050_read_accel_raw(mpu6050_t *mpu6050);

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
esp_err_t mpu6050_init(void);
