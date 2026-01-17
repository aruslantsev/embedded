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
 * @brief Writes data to specified device at specified bus
 *
 * @param I2C_HandleTypeDef *handle: pointer i2c bus handle
 * @param uint16_t dev_address: device address
 * @param uint16_t mem_address: memory address (register)
 * @param uint8_t *buffer: buffer with data
 * @param uint16_t num_bytes: amount of data to be written
 * @param uint32_t timeout: timeout in ms

 * @return I2CStatus
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
 * @brief Reads data from specified device at specified bus
 *
 * @param I2C_HandleTypeDef *handle: pointer i2c bus handle
 * @param uint16_t dev_address: device address
 * @param uint16_t mem_address: memory address (register)
 * @param uint8_t *buffer: buffer where data will be stored
 * @param uint16_t num_bytes: amount of data to be read
 * @param uint32_t timeout: timeout in ms

 * @return I2CStatus
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
 * @brief Writes data to specified device at specified bus
 *
 * @param i2c_master_bus_handle_t *handle: pointer i2c bus handle
 * @param i2c_master_dev_handle_t dev_handle: device handle
 * @param uint16_t mem_address: memory address (register)
 * @param uint8_t *buffer: buffer with data
 * @param uint16_t num_bytes: amount of data to be written
 * @param uint32_t timeout: timeout in ms

 * @return I2CStatus
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
 * @brief Reads data from specified device at specified bus
 *
 * @param i2c_master_bus_handle_t *handle: pointer i2c bus handle
 * @param i2c_master_dev_handle_t dev_handle: device handle
 * @param uint16_t mem_address: memory address (register)
 * @param uint8_t *buffer: buffer where data will be stored
 * @param uint16_t num_bytes: amount of data to be read
 * @param uint32_t timeout: timeout in ms

 * @return I2CStatus
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
