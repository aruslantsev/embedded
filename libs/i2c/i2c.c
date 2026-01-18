/**
 * @file i2c.c
 * @brief Universal I2C master-mode driver for STM32 and ESP32 microcontrollers.
 * @author Andrei Ruslantsev
 * @copyright (c) 2026 Andrei Ruslantsev. All rights reserved.
 * 
 * This software is provided "as is", without warranty of any kind, express or
 * implied, including but not limited to the warranties of merchantability,
 * fitness for a particular purpose and noninfringement.
 */

#include "i2c.h"


#if defined(STM32)

I2CStatus i2c_write(
    I2C_HandleTypeDef *handle, 
    const uint16_t dev_address, 
    const uint16_t mem_address, 
    const uint8_t *buffer, 
    const uint16_t num_bytes,
    const uint32_t timeout
) {
    const HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(
        handle, 
        dev_address << 1, 
        mem_address, 
        I2C_MEMADD_SIZE_8BIT, 
        (uint8_t *) buffer, 
        num_bytes, 
        timeout
    );
    if (ret != HAL_OK) {
        return I2C_ERR;
    }
    return I2C_OK;
}


I2CStatus i2c_read(
    I2C_HandleTypeDef *handle, 
    const uint16_t dev_address, 
    const uint16_t mem_address, 
    uint8_t *buffer, 
    const uint16_t num_bytes,
    const uint32_t timeout
) {
    const HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(
        handle, 
        dev_address << 1, 
        mem_address, 
        I2C_MEMADD_SIZE_8BIT,
        (uint8_t *)buffer, 
        num_bytes, 
        timeout
    );
    if (ret != HAL_OK) {
        return I2C_ERR;
    }
    return I2C_OK;
}

#elif defined (ESP32)

I2CStatus i2c_read(
    i2c_master_bus_handle_t *handle, 
    i2c_master_dev_handle_t dev_handle, 
    uint8_t mem_address, 
    uint8_t *buffer, 
    const uint16_t num_bytes,
    const uint32_t timeout
) {
    const esp_err_t ret = i2c_master_transmit_receive(
        dev_handle, 
        &mem_address, 
        1, buffer, 
        num_bytes, 
        timeout
    );
    if (ret != ESP_OK) {
        return I2C_ERR;
    }
    return I2C_OK;
}


I2CStatus i2c_write(
    i2c_master_bus_handle_t *handle, 
    i2c_master_dev_handle_t dev_handle, 
    uint16_t mem_address, 
    uint8_t *buffer, 
    const uint16_t num_bytes,
    const uint32_t timeout
) {
    if (num_bytes >= sizeof(buffer)) return I2C_ERR;
    uint8_t write_buf[WRITE_BUFFER_SIZE];
    write_buf[0] = mem_address;
    for (size_t ptr = 0; ptr < num_bytes; ptr++) {
        write_buf[ptr + 1] = buffer[ptr];
    }
    const esp_err_t ret = i2c_master_transmit(
        dev_handle, 
        write_buf, 
        num_bytes + 1, 
        timeout
    );
    if (ret != ESP_OK) {
        return I2C_ERR;
    }
    return I2C_OK;
}

#else 
#error No MCU specified
#endif
