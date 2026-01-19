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


#if defined(STM32WB)
#include "stm32wbxx_hal.h"
#elif defined(STM32F0)
#include "stm32f0xx_hal.h"
#elif defined(STM32F1)
#include "stm32f1xx_hal.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#elif defined(STM32L0)
#include "stm32l0xx_hal.h"
#elif defined(STM32L1)
#include "stm32l1xx_hal.h"
#elif defined(STM32L4)
#include "stm32l4xx_hal.h"
#elif defined(STM32L5)
#include "stm32l5xx_hal.h"
#elif defined(STM32F3)
#include "stm32f3xx_hal.h"
#elif defined(STM32H5)
#include "stm32h5xx_hal.h"
#elif defined(STM32H7)
#include "stm32h7xx_hal.h"
#elif defined(STM32F7)
#include "stm32f7xx_hal.h"
#elif defined(STM32G0)
#include "stm32g0xx_hal.h"
#elif defined(STM32G4)
#include "stm32g4xx_hal.h"
#elif defined(STM32C0)
#include "stm32c0xx_hal.h"
#elif defined(STM32U5)
#include "stm32u5xx_hal.h"
#else
#error "HMC5883L library was tested only on STM32F0, STM32F1, STM32F3, STM32F4, \
STM32F7, STM32L0, STM32L1, STM32L4, STM32H7, STM32G0, STM32G4, STM32WB, STM32C0, \
STM32U5 MCU families. Please modify hmc5883l.h if you know what you are doing. \
Also please send a pull request if it turns out the library works on other MCU's as well!"
#endif


typedef enum {
    HMC5883L_OK             = 0x00U,
    HMC5883L_NORESPONSE     = 0x01U,
    HMC5883L_NOTFOUND       = 0x02U,
    HMC5883L_WRITE_ERR      = 0x03U,
    HMC5883L_READ_ERR       = 0x04U,
    HMC5883L_VALUE_ERR      = 0x05U,
    HMC5883L_TIMEOUT_ERR    = 0x06U
} HMC5883L_STATUS;


typedef struct {
    I2C_HandleTypeDef *i2c_bus;
    uint16_t addr; // 0x1E
    int16_t x;
    int16_t y;
    int16_t z;
} HMC5883L;


/*
 * Sets MSB of MR to 0
 */
HMC5883L_STATUS hmc5883l_reset_mr7(HMC5883L *); 


/*
 * Copies MSB of MR to mr7_is_set
 */
HMC5883L_STATUS hmc5883l_get_mr7(HMC5883L *, uint8_t *);


/*
 * Initializes structure with bus handler and address, checks if compass is present.
 * Resets some bits of registers
 */
HMC5883L_STATUS hmc5883l_init(HMC5883L *hmc5883l, I2C_HandleTypeDef *i2c_bus, uint16_t addr);


/*
 * Get averaged number of samples per measurement
 * 0U = 1, 1U, 2U = 4, 3U = 8
 */
HMC5883L_STATUS hmc5883l_get_number_of_samples(HMC5883L *hmc5883l, uint8_t *number_of_samples);


/*
 * Set averaged number of samples per measurement
 * 0U = 1, 1U, 2U = 4, 3U = 8 (default)
 */
HMC5883L_STATUS hmc5883l_set_number_of_samples(HMC5883L *hmc5883l, uint8_t number_of_samples);


/*
 * Gets data output rate
 * 0U = 0.75Hz, 1U = 1.5Hz, 2U = 3Hz, 3U = 7.5Hz, 4U = 15Hz (default),
 * 5U = 30Hz, 6U = 75Hz, 7U NOT USED
 */
HMC5883L_STATUS hmc5883l_get_output_rate(HMC5883L *hmc5883l, uint8_t *rate);


/*
 * Sets data output rate
 * 0U = 0.75Hz, 1U = 1.5Hz, 2U = 3Hz, 3U = 7.5Hz, 4U = 15Hz (default),
 * 5U = 30Hz, 6U = 75Hz, 7U NOT USED
 */
HMC5883L_STATUS hmc5883l_set_output_rate(HMC5883L *hmc5883l, uint8_t rate);


/*
 * Gets measurement configuration
 * 0U = Normal (default), 1U = positive bias, 2U = negative bias, 3U NOT USED
 */
HMC5883L_STATUS hmc5883l_get_measurement_configuration(HMC5883L *hmc5883l, uint8_t *configuration);


/*
 * Sets measurement configuration
 * 0U = Normal (default), 1U = positive bias, 2U = negative bias, 3U NOT USED
 */
HMC5883L_STATUS hmc5883l_set_measurement_configuration(HMC5883L *hmc5883l, uint8_t configuration);


/*
 * Sets gain
 * Value        Recommended sensor  Gain        Digital resolution  Output range
                field range         LSb/Gauss   mG/LSb
 * 0U           +-0.88Ga            1370        0.73                0xF800-0x07FF (-2048-2047)
 * 1U (default) +-1.3               1090        0.92                0xF800-0x07FF (-2048-2047)
 * 2U           +-1.9               820         1.22                0xF800-0x07FF (-2048-2047)
 * 3U           +-2.5               660         1.52                0xF800-0x07FF (-2048-2047)
 * 4U           +-4.0               440         2.27                0xF800-0x07FF (-2048-2047)
 * 5U           +-4.7               390         2.56                0xF800-0x07FF (-2048-2047)
 * 6U           +-5.6               330         3.03                0xF800-0x07FF (-2048-2047)
 * 7U           +-8.1               230         4.35                0xF800-0x07FF (-2048-2047)
 */
HMC5883L_STATUS hmc5883l_get_gain(HMC5883L *hmc5883l, uint8_t *gain);


/*
 * Gets gain
 * Value        Recommended sensor  Gain        Digital resolution  Output range
                field range         LSb/Gauss   mG/LSb
 * 0U           +-0.88Ga            1370        0.73                0xF800-0x07FF (-2048-2047)
 * 1U (default) +-1.3               1090        0.92                0xF800-0x07FF (-2048-2047)
 * 2U           +-1.9               820         1.22                0xF800-0x07FF (-2048-2047)
 * 3U           +-2.5               660         1.52                0xF800-0x07FF (-2048-2047)
 * 4U           +-4.0               440         2.27                0xF800-0x07FF (-2048-2047)
 * 5U           +-4.7               390         2.56                0xF800-0x07FF (-2048-2047)
 * 6U           +-5.6               330         3.03                0xF800-0x07FF (-2048-2047)
 * 7U           +-8.1               230         4.35                0xF800-0x07FF (-2048-2047)
 */
HMC5883L_STATUS hmc5883l_set_gain(HMC5883L *hmc5883l, uint8_t gain);


/*
 * Gets operating mode
 * 0U = Continuous measurement, 1U = Single measurement (default), 2U, 3U = Idle
 */
HMC5883L_STATUS hmc5883l_get_operating_mode(HMC5883L *hmc5883l, uint8_t *mode);


/*
 * Sets operating mode
 * 0U = Continuous measurement, 1U = Single measurement (default), 2U, 3U = Idle
 */
HMC5883L_STATUS hmc5883l_set_operating_mode(HMC5883L *hmc5883l, uint8_t mode);


/*
 * Reads data from registers, places data in x, y, z fields
 */
HMC5883L_STATUS hmc5883l_read(HMC5883L* hmc5883l);


/*
 * Checks if ready bit set in status register
 */
HMC5883L_STATUS hmc5883l_is_ready(HMC5883L *hmc5883l, uint8_t *is_ready);


/*
 * Checks if locked bit set in status register
 */
HMC5883L_STATUS hmc5883l_is_locked(HMC5883L *hmc5883l, uint8_t *is_locked);


/*
 * Waits for ready bit in status register
 */
HMC5883L_STATUS hmc5883l_wait_ready(HMC5883L *hmc5883l, uint8_t timeout_ms);


/*
 * Performs single measurement
 */
HMC5883L_STATUS hmc5883l_single_measurement(HMC5883L *hmc5883l);

/*
 * Performs continuous measurement
 * Make sure to set mode to continuous measurement
 */
HMC5883L_STATUS hmc5883l_continuous_measurement(HMC5883L *hmc5883l);

#endif
