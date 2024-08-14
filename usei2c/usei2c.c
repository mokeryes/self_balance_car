#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"

#include "usei2c.h"

static usei2c_config_t *usei2c;
static i2c_master_dev_handle_t dev_handle;
static i2c_master_bus_handle_t bus_handle;

esp_err_t usei2c_read_reg_burst(uint8_t reg_addr, uint8_t *data, size_t data_len) {
    esp_err_t ret = ESP_FAIL;

    if (data == NULL || data_len < 1) {
        ESP_LOGE(USEI2C_LOG_TAG,
                 "Error in read burst, data or data_len is invalid, error code: %s.",
                 esp_err_to_name(ret));
        return ret;
    }

    uint8_t *buffer = (uint8_t *)malloc((size_t)(sizeof(uint8_t) * 1));
    memcpy(buffer, &reg_addr, 1);

    ret = i2c_master_transmit_receive(dev_handle, buffer, 1, data, data_len, 1000);
    free(buffer);
    if (ret != ESP_OK) {
        ESP_LOGE(USEI2C_LOG_TAG, "Read data from register error, error code: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    return ret;
}

esp_err_t usei2c_write_reg_burst(uint8_t reg_addr, const uint8_t *data, size_t data_len) {
    esp_err_t ret = ESP_FAIL;

    if (data == NULL || data_len < 1) {
        ESP_LOGE(USEI2C_LOG_TAG,
                 "Error in write burst, data or data_len is invalid, error code: %s.",
                 esp_err_to_name(ret));
        return ret;
    }

    data_len++;
    uint8_t *buffer = (uint8_t *)malloc((size_t)(sizeof(uint8_t) * data_len));

    /* copy register address and data into buffer */
    memcpy(buffer, &reg_addr, 1);
    memcpy(buffer + 1, data, 1);

    ret = i2c_master_transmit(dev_handle, buffer, data_len, 1000);
    free(buffer);
    if (ret != ESP_OK) {
        ESP_LOGE(USEI2C_LOG_TAG, "Write data into device error, error code: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    return ret;
}

uint8_t usei2c_read_reg(uint8_t reg_addr) {
    uint8_t buffer = 0x00;
    usei2c_read_reg_burst(reg_addr, &buffer, 1);
    return buffer;
}

esp_err_t usei2c_write_reg(uint8_t reg_addr, uint8_t data) {
    return usei2c_write_reg_burst(reg_addr, &data, 1);
}

uint8_t usei2c_read_bits(uint8_t reg_addr, uint8_t bits_start, uint8_t bits_len) {
    if (bits_start > 8 || bits_len < 1 || bits_len > 9) {
        ESP_LOGE(USEI2C_LOG_TAG, "Parameter bits_start or bits_len out of range (0-7).");
        return 0xFF;
    }

    uint8_t bits_data = 0x00;
    uint8_t mask = 0x00;
    uint8_t data = usei2c_read_reg(reg_addr);

    mask = ((1 << bits_len) - 1);  // 取掩码
    data >>= bits_start;           // 要读取的数据与掩码有效位对齐
    bits_data = mask & data;       // 获取位

    return bits_data;
}

esp_err_t usei2c_write_bits(uint8_t reg_addr, uint8_t bits_data, uint8_t bits_start,
                            uint8_t bits_len) {
    esp_err_t ret = ESP_FAIL;

    if (bits_start > 8 || bits_len < 1 || bits_len > 9) {
        ESP_LOGE(USEI2C_LOG_TAG, "Parameter bits_start or bits_len out of range (1-8).");
        return ret;
    }

    uint8_t mask = 0x00;
    uint8_t data = usei2c_read_reg(reg_addr);

    mask = ~(((1 << bits_len) - 1) << bits_start);  // 取掩码
    data &= mask;                                   // 清零写入位
    bits_data <<= bits_start;                       // 写入数据移动到写入位
    bits_data |= data;                              // 合并数据
    printf("bits_data: 0x%x\n", bits_data);

    ret = usei2c_write_reg(reg_addr, bits_data);
    if (ret != ESP_OK) {
        ESP_LOGE(USEI2C_LOG_TAG, "Write bits error, error code: %s", esp_err_to_name(ret));
        return ret;
    }

    return ret;
}

uint8_t usei2c_read_bit(uint8_t reg_addr, uint8_t bit_num) {
    if (bit_num > 7) {
        ESP_LOGE(USEI2C_LOG_TAG, "Parameter bit_num out of range (0-7).");
        return 0xFF;
    }

    uint8_t data = usei2c_read_reg(reg_addr);
    uint8_t mask = 0x00;

    mask = (1 << bit_num);  // 获取掩码
    data &= mask;           // 利用掩码来获取读取位
    data >>= bit_num;       // 将读取到的位移动到LSB

    return data;
}

esp_err_t usei2c_write_bit(uint8_t reg_addr, uint8_t bit_num, uint8_t bit_data) {
    esp_err_t ret = ESP_FAIL;
    if (bit_num > 7) {
        ESP_LOGE(USEI2C_LOG_TAG, "Parameter bit_num out of range (0-7).");
        return ret;
    }

    uint8_t mask = 0x00;
    uint8_t data = usei2c_read_reg(reg_addr);

    mask = (1 << bit_num);          // 获取掩码
    mask = ~mask;                   // 掩码取反，用来清除写入位
    data &= mask;                   // 清除写入位
    data |= (bit_data << bit_num);  // 合并数据

    ret = usei2c_write_reg(reg_addr, data);
    if (ret != ESP_OK) {
        ESP_LOGE(USEI2C_LOG_TAG, "Write bit error, error code: %s", esp_err_to_name(ret));
        return ret;
    }

    return ret;
}

esp_err_t usei2c_init(usei2c_config_t *usei2c_cfg) {
    esp_err_t ret = ESP_FAIL;

    usei2c = usei2c_cfg;

    ret = i2c_new_master_bus(&usei2c->bus_cfg, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(USEI2C_LOG_TAG, "Error occured in new master bus, error code: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    ret = i2c_master_bus_add_device(bus_handle, &usei2c->dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(USEI2C_LOG_TAG, "Error occured in add device into master bus, error code: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    return ret;
}
