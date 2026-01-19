/**
 * @file bmp180.h
 * @brief Driver implementation for the Bosch BMP180 Barometric Temperature and Pressure Sensor.
 * @author Andrei Ruslantsev
 * @copyright (c) 2026 Andrei Ruslantsev. All rights reserved.
 * 
 * This software is provided "as is", without warranty of any kind, express or
 * implied, including but not limited to the warranties of merchantability,
 * fitness for a particular purpose and noninfringement.
 */

#ifndef BMP180_H
#define BMP180_H

#include "i2c.h"

/*
 * BMP180 temperature and pressure sensor.
 * Usually uses address 0x77.
 * 1. Define macro for your MCU
 * 2. Include i2c library from this repository
 * 3. Allocate BMP180 structure
 * 4. Call bmp180_init to initialize BMP180 structure. This checks if sensor is 
 *    present, resets it and fetches calibration data.
 * 5. Call bmp180_set_oversampling to change oversampling setting.
 * 6. Call bmp180_get_temperature. This fills fields ut and temperature in BMP180 structure.
 * 7. Call bmp180_get_pressure. Uses results of bmp180_get_temperature.
 *    This fills fields up and pressure in BMP180 structure.
 */


/* Possible device statuses */
typedef enum {
    BMP180_OK           = 0x00U,
    BMP180_NORESPONSE   = 0x01U,
    BMP180_NOTFOUND     = 0x02U,
    BMP180_RESET_ERR    = 0x03U,
    BMP180_WRITE_ERR    = 0x04U,
    BMP180_READ_ERR     = 0x05U,
    BMP180_VALUE_ERR    = 0x06U,
    BMP180_TIMEOUT_ERR  = 0x07U
} BMP180_STATUS;


/* Possible values of oversampling setting */
typedef enum {
    BMP180_OSS_SINGLE   = 0b00,
    BMP180_OSS_DOUBLE   = 0b01,
    BMP180_OSS_QUAD     = 0b10,
    BMP180_OSS_OCT      = 0b11
} BMP180_OSS;


typedef struct {
#if defined(STM32)
    I2C_HandleTypeDef   *i2c_bus;
    uint16_t            addr; // 0x77
#elif defined(ESP32)
    i2c_master_bus_handle_t *i2c_bus;
    i2c_master_dev_handle_t addr;
#else
#error No MCU defined
#endif
    /* Calibration data */
    int16_t             ac1;
    int16_t             ac2;
    int16_t             ac3;
    uint16_t            ac4;
    uint16_t            ac5;
    uint16_t            ac6;
    int16_t             b1;
    int16_t             b2;
    int16_t             mb;
    int16_t             mc;
    int16_t             md;
    /* Oversampling setting: 0b00 <= oss <= 0b11 */
    uint8_t             oss;
    /* Uncompensated temperature and pressure */
    int32_t             ut;
    int32_t             up;
    /* Compensated temperature and pressure */
    int32_t             temperature; // 0.1 deg. Celsius
    int32_t             pressure; // Pa
} BMP180;


/**
 * @brief  Initializes the BMP180 sensor and its internal state structure.
 *
 * This function performs the complete sensor startup sequence:
 * 1. Links the I2C peripheral and device address to the handle.
 * 2. Verifies the Chip ID (0x55) to ensure communication is established.
 * 3. Triggers a soft reset to clear internal registers.
 * 4. Reads factory-programmed calibration coefficients from the sensor's EEPROM.
 *
 * @param[out] bmp180   Pointer to the BMP180 handle structure to be initialized.
 * @param[in]  i2c_bus  Pointer to the hardware-specific I2C bus handle.
 * @param[in]  addr     The 7-bit I2C device address (typically 0x77).
 *
 * @return BMP180_STATUS  Status code indicating success or specific error
 */
#if defined(STM32)
BMP180_STATUS bmp180_init(BMP180 *bmp180, I2C_HandleTypeDef *i2c_bus, const uint16_t addr);
#elif defined(ESP32)
BMP180_STATUS bmp180_init(BMP180 *bmp180, i2c_master_bus_handle_t *i2c_bus, const uint8_t addr);
#else
#error
#endif


 /**
 * @brief  Configures the oversampling setting (OSS) for pressure measurements.
 *
 * This function updates the internal handle with the desired oversampling ratio.
 * Higher oversampling values increase measurement precision and reduce noise, 
 * but require more time and power to complete a conversion.
 * @note   This function only updates the internal configuration within the 
 * @ref BMP180 structure. The setting is not sent to the sensor immediately; 
 * it is applied later during the actual pressure measurement sequence.
 *
 * @param[in,out] bmp180  Pointer to the BMP180 handle structure.
 * @param[in]     oss     The oversampling setting to be applied.
 * Supported values:
 * - @ref BMP180_OSS_SINGLE: Ultra low power (1 internal sample)
 * - @ref BMP180_OSS_DOUBLE: Standard (2 internal samples)
 * - @ref BMP180_OSS_QUAD:   High resolution (4 internal samples)
 * - @ref BMP180_OSS_OCT:    Ultra high resolution (8 internal samples)
 *
 * @return BMP180_STATUS
 */
BMP180_STATUS bmp180_set_oversampling(BMP180 *bmp180, const BMP180_OSS oss);


/**
 * @brief  Performs a complete temperature measurement cycle.
 *
 * This function triggers the sensor to start a temperature conversion, waits for 
 * the hardware to complete the measurement, reads the raw data from the output 
 * registers, and finally calculates the real-world temperature.
 *
 * @note The resulting temperature is stored in the @p bmp180 structure with 
 * a resolution of 0.1 deg. Celsius.
 *
 * @param[in,out] bmp180  Pointer to the BMP180 handle structure. 
 * The 'ut' (raw) and 'temperature' (compensated) 
 * fields will be updated.
 *
 * @return BMP180_STATUS
 */
BMP180_STATUS bmp180_get_temperature(BMP180 *bmp180);


/**
 * @brief  Performs a complete pressure measurement cycle.
 *
 * This function triggers the sensor to start a pressure conversion using the 
 * current oversampling setting (OSS), waits for the conversion to finish, 
 * reads the raw data (UP), and calculates the compensated pressure in Pascals.
 *
 * @note   Before calling this function, ensure that @ref bmp180_get_temperature 
 * has been called, as the pressure compensation algorithm requires 
 * the latest temperature compensation coefficients.
 *
 * @param[in,out] bmp180  Pointer to the BMP180 handle structure. 
 * The 'up' (raw) and 'pressure' (compensated) 
 * fields will be updated.
 *
 * @return BMP180_STATUS
 */
BMP180_STATUS bmp180_get_pressure(BMP180 *bmp180);


/**
 * @brief  Performs a complete measurement cycle (Temperature and Pressure).
 *
 * This is a convenience function that sequentially calls @ref bmp180_get_temperature 
 * and @ref bmp180_get_pressure. It ensures that the temperature compensation 
 * coefficients are updated before the pressure calculation.
 *
 * @param[in,out] bmp180  Pointer to the BMP180 handle structure. 
 * All measurement fields (ut, up, temperature, pressure) will be updated
 *
 * @return BMP180_STATUS
 */
BMP180_STATUS bmp180_get_all(BMP180 *bmp180);


#define BMP180_AC1_MSB_REG      0xAA
#define BMP180_AC1_LSB_REG      0xAB
#define BMP180_AC2_MSB_REG      0xAC
#define BMP180_AC2_LSB_REG      0xAD
#define BMP180_AC3_MSB_REG      0xAE
#define BMP180_AC3_LSB_REG      0xAF
#define BMP180_AC4_MSB_REG      0xB0
#define BMP180_AC4_LSB_REG      0xB1
#define BMP180_AC5_MSB_REG      0xB2
#define BMP180_AC5_LSB_REG      0xB3
#define BMP180_AC6_MSB_REG      0xB4
#define BMP180_AC6_LSB_REG      0xB5
#define BMP180_B1_MSB_REG       0xB6
#define BMP180_B1_LSB_REG       0xB7
#define BMP180_B2_MSB_REG       0xB8
#define BMP180_B2_LSB_REG       0xB9
#define BMP180_MB_MSB_REG       0xBA
#define BMP180_MB_LSB_REG       0xBB
#define BMP180_MC_MSB_REG       0xBC
#define BMP180_MC_LSB_REG       0xBD
#define BMP180_MD_MSB_REG       0xBE
#define BMP180_MD_LSB_REG       0xBF

#define BMP180_CTRL_MEAS_REG    0xF4

#define BMP180_OUT_MSB_REG      0xF6
#define BMP180_OUT_LSB_REG      0xF7
#define BMP180_OUT_XLSB_REG     0xF8

#define BMP180_ID_REG           0xD0
#define BMP180_SOFT_RST_REG     0xE0

#define BMP180_CHIP_ID          0x55

#define BMP180_RST_VAL          0xB6

#define BMP180_SCO_BIT          5
#define BMP180_OSS_LS_BIT       6 /* Bits 6 and 7,  zero indexed */

#define BMP180_TEMP_MEAS_VAL    0x2E
#define BMP180_PRES_MEAS_VAL    0x34

#define BMP180_MAX_RETRIES      128

#define BMP180_I2C_TIMEOUT_MS   128
#define BMP180_RESET_TIMEOUT_MS 50
#define BMP180_MEAS_TIMEOUT_MS  5

#endif
