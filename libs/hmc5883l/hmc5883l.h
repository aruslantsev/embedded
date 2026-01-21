/**
 * @file hmc5883l.h
 * @brief Driver implementation for the HMC5883L 3-Axis Digital Compass.
 * @author Andrei Ruslantsev
 * @copyright (c) 2026 Andrei Ruslantsev. All rights reserved.
 * 
 * This software is provided "as is", without warranty of any kind, express or
 * implied, including but not limited to the warranties of merchantability,
 * fitness for a particular purpose and noninfringement.
 */

#ifndef HMC5883L_H
#define HMC5883L_H

#include "i2c.h"

/*
 * HMC5883L compass.
 * Usually uses address 0x1E.
 * 1. Allocate HMC5883L structure
 * 2. Call hmc5883l_init(HMC5883L *hmc5883l, I2C_HandleTypeDef *hi2c, uint16_t addr)
 *    to initialize HMC5883L structure. This checks if compass is present.
 * 3. Set gain, rate, mode, etc...
 * 4. Call hmc5883l_single_measurement(HMC5883L *hmc5883l)
 *    or hmc5883l_continuous_measurement(HMC5883L *hmc5883l)
 * 5. Read fields x, y, z of hmc5883l
 */

typedef enum {
    HMC5883L_OK             = 0x00U,
    HMC5883L_NORESPONSE     = 0x01U,
    HMC5883L_NOTFOUND       = 0x02U,
    HMC5883L_WRITE_ERR      = 0x03U,
    HMC5883L_READ_ERR       = 0x04U,
    HMC5883L_VALUE_ERR      = 0x05U,
    HMC5883L_TIMEOUT_ERR    = 0x06U,
    HMC5883L_OVERFLOW       = 0x07U
} HMC5883L_STATUS;


typedef struct {
#if defined(STM32)
    I2C_HandleTypeDef *i2c_bus;
    uint16_t addr; // 0x1E
#elif defined(ESP32)
    i2c_master_bus_handle_t *i2c_bus;
    i2c_master_dev_handle_t addr;
#else
#error No MCU defined
#endif
    int16_t x;
    int16_t y;
    int16_t z;
} HMC5883L;


/**
  * @brief Number of samples averaged per measurement. Default value: HMC5883L_SAMPLING_SINGLE, 1 sample per measurement
  */
typedef enum {
    HMC5883L_SAMPLING_SINGLE = 0U,
    HMC5883L_SAMPLING_DOUBLE = 1U,
    HMC5883L_SAMPLING_QUAD   = 2U,
    HMC5883L_SAMPLING_OCT    = 3U,
} HMC5883L_SAMPLING;


/**
  * @brief Output rate in 0.01 Hz (0075_HZ -> 0.75 Hz). Default value: HMC5883L_OUTPUT_RATE_1500_HZ, 15 Hz
  * @note Value 7U is reserved
  */
typedef enum {
    HMC5883L_OUTPUT_RATE_0075_HZ = 0U,
    HMC5883L_OUTPUT_RATE_0150_HZ = 1U,
    HMC5883L_OUTPUT_RATE_0300_HZ = 2U,
    HMC5883L_OUTPUT_RATE_0750_HZ = 3U,
    HMC5883L_OUTPUT_RATE_1500_HZ = 4U,
    HMC5883L_OUTPUT_RATE_3000_HZ = 5U,
    HMC5883L_OUTPUT_RATE_7500_HZ = 6U
} HMC5883L_OUTPUT_RATE;


/**
  * @brief Measurement mode. Default value: Normal mode, HMC5883L_MEASUREMENT_MODE_NORMAL
  * @note Value 3U is reserved
  */
typedef enum {
    HMC5883L_MEASUREMENT_MODE_NORMAL = 0U, /* Normal configuration */
    HMC5883L_MEASUREMENT_MODE_POS_BIAS = 1U, /* Positive bias */
    HMC5883L_MEASUREMENT_MODE_NEG_BIAS = 2U, /* Negative bias */
} HMC5883L_MEASUREMENT_MODE;


/**
  * @brief Gain. Default value 1090 LSb/Ga, HMC5883L_GAIN_1090
  * @note Value 3U is reserved
  * @note Output range: 0xF800-0x07FF (-2048-2047)
  */

/*                              Recommended sensor  Gain        Digital resolution
                                field range, Ga     LSb/Gauss   mG/LSb              */
typedef enum {
    HMC5883L_GAIN_1370 = 0U, /*  +/-0.88            1370        0.73                */
    HMC5883L_GAIN_1090 = 1U, /*  +/-1.3             1090        0.92                */
    HMC5883L_GAIN_820  = 2U, /*  +/-1.9             820         1.22                */
    HMC5883L_GAIN_660  = 3U, /*  +/-2.5             660         1.52                */
    HMC5883L_GAIN_440  = 4U, /*  +/-4.0             440         2.27                */
    HMC5883L_GAIN_390  = 5U, /*  +/-4.7             390         2.56                */
    HMC5883L_GAIN_330  = 6U, /*  +/-5.6             330         3.03                */
    HMC5883L_GAIN_230  = 7U, /*  +/-8.1             230         4.35                */
} HMC5883L_GAIN;


/**
  * @brief Operating mode. Default value: Single, HMC5883L_OPERATING_MODE_SINGLE
  */
typedef enum {
    HMC5883L_OPERATING_MODE_CONTINUOUS = 0U, /* Continuous measurement mode */
    HMC5883L_OPERATING_MODE_SINGLE = 1U, /* Single measurement mode */
    HMC5883L_OPERATING_MODE_IDLE  = 2U, /* Idle */
    HMC5883L_OPERATING_MODE_IDLE1 = 2U, /* Idle */
} HMC5883L_OPERATING_MODE;


/**
 * @brief Initializes the HMC5883L sensor.
 * @param hmc5883l Pointer to driver handle.
 * @param i2c_bus Pointer to I2C bus handle (I2C_HandleTypeDef or i2c_master_bus_handle_t).
 * @param addr 7-bit device address (usually 0x1E).
 * @return HMC5883L_STATUS result.
 */
#if defined(STM32)
HMC5883L_STATUS hmc5883l_init(HMC5883L *hmc5883l, I2C_HandleTypeDef *i2c_bus, uint16_t addr);
#elif defined(ESP32)
HMC5883L_STATUS hmc5883l_init(HMC5883L *hmc5883l, i2c_master_bus_handle_t *i2c_bus, const uint8_t addr);
#else
#error No MCU defined
#endif


/**
 * @brief Gets averaged number of samples per measurement
 * @param[in]  hmc5883l Pointer to driver handle.
 * @param[out] sampling Pointer to sampling setting
 * @return HMC5883L_STATUS
 */
HMC5883L_STATUS hmc5883l_get_number_of_samples(HMC5883L *hmc5883l, HMC5883L_SAMPLING *sampling);


/**
 * @brief Sets averaged number of samples per measurement
 * @param[in] hmc5883l Pointer to driver handle.
 * @param[in] sampling Sampling setting
 * @return HMC5883L_STATUS
 */
HMC5883L_STATUS hmc5883l_set_number_of_samples(HMC5883L *hmc5883l, HMC5883L_SAMPLING sampling);


/**
 * @brief Gets output rate
 * @param[in]  hmc5883l Pointer to driver handle.
 * @param[out] rate Pointer to rate setting
 * @return HMC5883L_STATUS
 */
HMC5883L_STATUS hmc5883l_get_output_rate(HMC5883L *hmc5883l, HMC5883L_OUTPUT_RATE *rate);


/**
 * @brief Sets output rate
 * @param[in] hmc5883l Pointer to driver handle.
 * @param[in] rate Rate setting
 * @return HMC5883L_STATUS
 */
HMC5883L_STATUS hmc5883l_set_output_rate(HMC5883L *hmc5883l, HMC5883L_OUTPUT_RATE rate);


/**
 * @brief Gets measurement configuration
 * @param[in]  hmc5883l Pointer to driver handle.
 * @param[out] configuration Pointer to configuration setting
 * @return HMC5883L_STATUS
 */
HMC5883L_STATUS hmc5883l_get_measurement_mode(HMC5883L *hmc5883l, HMC5883L_MEASUREMENT_MODE *configuration);


/**
 * @brief Sets measurement configuration
 * @param[in] hmc5883l Pointer to driver handle.
 * @param[in] configuration Configuration setting
 * @return HMC5883L_STATUS
 */
HMC5883L_STATUS hmc5883l_set_measurement_mode(HMC5883L *hmc5883l, HMC5883L_MEASUREMENT_MODE configuration);


/**
 * @brief Gets gain
 * @param[in]  hmc5883l Pointer to driver handle.
 * @param[out] gain Pointer to gain setting
 * @return HMC5883L_STATUS
 */
HMC5883L_STATUS hmc5883l_get_gain(HMC5883L *hmc5883l, HMC5883L_GAIN *gain);


/**
 * @brief Sets gain
 * @param[in] hmc5883l Pointer to driver handle.
 * @param[in] gain Gain setting
 * @return HMC5883L_STATUS
 */
HMC5883L_STATUS hmc5883l_set_gain(HMC5883L *hmc5883l, HMC5883L_GAIN gain);


/**
 * @brief Gets operating mode
 * @param[in]  hmc5883l Pointer to driver handle.
 * @param[out] mode Pointer to operating mode setting
 * @return HMC5883L_STATUS
 */
HMC5883L_STATUS hmc5883l_get_operating_mode(HMC5883L *hmc5883l, HMC5883L_OPERATING_MODE *mode);


/**
 * @brief Sets operating mode
 * @param[in] hmc5883l Pointer to driver handle.
 * @param[in] mode Operating mode setting
 * @return HMC5883L_STATUS
 */
HMC5883L_STATUS hmc5883l_set_operating_mode(HMC5883L *hmc5883l, HMC5883L_OPERATING_MODE mode);


/**
 * @brief Sets operating mode to single measurement and performs measurement
 * @note hmc5883l changes mode to idle automatically
 * @param[in,out] hmc5883l Pointer to driver handle.
 * @return HMC5883L_STATUS
 */
HMC5883L_STATUS hmc5883l_single_measurement(HMC5883L *hmc5883l);


/**
 * @brief Waits for readiness and performs measurement
 * @note Make sure to change mode to single or continuous measurement
 * @param[in,out] hmc5883l Pointer to driver handle.
 * @return HMC5883L_STATUS
 */
HMC5883L_STATUS hmc5883l_measurement(HMC5883L *hmc5883l);


#define HMC5883L_CRA    0x00
#define HMC5883L_CRB    0x01
#define HMC5883L_MR     0x02
#define HMC5883L_X_MSB  0x03
#define HMC5883L_X_LSB  0x04
#define HMC5883L_Z_MSB  0x05
#define HMC5883L_Z_LSB  0x06
#define HMC5883L_Y_MSB  0x07
#define HMC5883L_Y_LSB  0x08
#define HMC5883L_SR     0x09
#define HMC5883L_IDA    0x0A
#define HMC5883L_IDB    0x0B
#define HMC5883L_IDC    0x0C

#define HMC5883L_TIMEOUT_MS     100

#define HMC5883L_CRA_SAMPLES_MASK     0b01100000
#define HMC5883L_CRA_SAMPLES_POS      5

#define HMC5883L_CRA_RATE_MASK        0b00011100
#define HMC5883L_CRA_RATE_POS         2

#define HMC5883L_CRA_MEAS_CONF_MASK   0b00000011
#define HMC5883L_CRA_MEAS_CONF_POS    0

#define HMC5883L_CRB_GAIN_MASK        0b11100000
#define HMC5883L_CRB_GAIN_POS         5

#define HMC5883L_MR_MODE_MASK         0b00000011
#define HMC5883L_MR_MODE_POS          0

#define HMC5883L_SR_LOCK_MASK         0b00000010
#define HMC5883L_SR_READY_MASK        0b00000001

#endif
