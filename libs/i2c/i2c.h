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

#ifndef I2C_H
#define I2C_H

#include <stdint.h>

#if defined(STM32WB)
#include "stm32wbxx_hal.h"
#define STM32
#elif defined(STM32F0)
#include "stm32f0xx_hal.h"
#define STM32
#elif defined(STM32F1)
#include "stm32f1xx_hal.h"
#define STM32
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#define STM32
#elif defined(STM32L0)
#include "stm32l0xx_hal.h"
#define STM32
#elif defined(STM32L1)
#include "stm32l1xx_hal.h"
#define STM32
#elif defined(STM32L4)
#include "stm32l4xx_hal.h"
#define STM32
#elif defined(STM32L5)
#include "stm32l5xx_hal.h"
#define STM32
#elif defined(STM32F3)
#include "stm32f3xx_hal.h"
#define STM32
#elif defined(STM32H5)
#include "stm32h5xx_hal.h"
#define STM32
#elif defined(STM32H7)
#include "stm32h7xx_hal.h"
#define STM32
#elif defined(STM32F7)
#include "stm32f7xx_hal.h"
#define STM32
#elif defined(STM32G0)
#include "stm32g0xx_hal.h"
#define STM32
#elif defined(STM32G4)
#include "stm32g4xx_hal.h"
#define STM32
#elif defined(STM32C0)
#include "stm32c0xx_hal.h"
#define STM32
#elif defined(STM32U5)
#include "stm32u5xx_hal.h"
#define STM32
#elif defined(ESP32)
#include "driver/i2c_master.h"
#include "esp_err.h"
#else
#error "I2C library was tested only on STM32F0, STM32F1, STM32F3, STM32F4, \
STM32F7, STM32L0, STM32L1, STM32L4, STM32H7, STM32G0, STM32G4, STM32WB, STM32C0, \
STM32U5 and ESP32 MCU families. Please modify i2c.h if you know what you are doing. \
Also please send a pull request if it turns out the library works on other MCU's as well!"
#endif

typedef enum { I2C_OK = 0, I2C_ERR = 1 } I2CStatus;

#if defined(STM32)

 /**
 * @brief Writes data from an I2C device register.
 *
 * @param[in]  handle       I2C bus handle.
 * @param[in]  dev_address  I2C slave device address.
 * @param[in]  mem_address  Internal register address to start reading from.
 * @param[out] buffer       Pointer to the buffer containing data to be sent.
 * @param[in]  num_bytes    Number of bytes to read.
 * @param[in]  timeout      Timeout.
 *
 * @retval 0                Success
 * @retval 1                Failure
 */
I2CStatus i2c_write(
    I2C_HandleTypeDef *handle, 
    const uint16_t dev_address, 
    const uint16_t mem_address, 
    const uint8_t *buffer, 
    const uint16_t num_bytes,
    const uint32_t timeout
);


/**
 * @brief Reads data from an I2C device register.
 *
 * @param[in]  handle       I2C bus handle.
 * @param[in]  dev_address  I2C slave device address.
 * @param[in]  mem_address  Internal register address to start reading from.
 * @param[in]  buffer       Pointer to the memory buffer to store the received data.
 * @param[in]  num_bytes    Number of bytes to read.
 * @param[in]  timeout      Timeout.
 *
 * @retval 0                Success
 * @retval 1                Failure
 */
I2CStatus i2c_read(
    I2C_HandleTypeDef *handle, 
    const uint16_t dev_address, 
    const uint16_t mem_address, 
    uint8_t *buffer, 
    const uint16_t num_bytes,
    const uint32_t timeout
);

#elif defined(ESP32)

#define WRITE_BUFFER_SIZE 255

/**
 * @brief Writes data from an I2C device register.
 *
 * @param[in]  handle       I2C bus handle.
 * @param[in]  dev_handle   I2C slave device handle.
 * @param[in]  mem_address  Internal register address to start reading from.
 * @param[out] buffer       Pointer to the buffer containing data to be sent.
 * @param[in]  num_bytes    Number of bytes to read.
 * @param[in]  timeout      Timeout.
 *
 * @retval 0                Success
 * @retval 1                Failure
 */
I2CStatus i2c_write(
    i2c_master_bus_handle_t *handle, 
    i2c_master_dev_handle_t dev_handle, 
    uint16_t mem_address, 
    uint8_t *buffer, 
    const uint16_t num_bytes,
    const uint32_t timeout
);


/**
 * @brief Reads data from an I2C device register.
 *
 * @param[in]  handle       I2C bus handle.
 * @param[in]  dev_handle   I2C slave device handle.
 * @param[in]  mem_address  Internal register address to start reading from.
 * @param[in]  buffer       Pointer to the memory buffer to store the received data.
 * @param[in]  num_bytes    Number of bytes to read.
 * @param[in]  timeout      Timeout.
 *
 * @retval 0                Success
 * @retval 1                Failure
 */
I2CStatus i2c_read(
    i2c_master_bus_handle_t *handle, 
    i2c_master_dev_handle_t dev_handle, 
    uint8_t mem_address, 
    uint8_t *buffer, 
    const uint16_t num_bytes,
    const uint32_t timeout
);

#endif

#endif
