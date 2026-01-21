/**
 * @file hmc5883l.c
 * @brief Driver implementation for the HMC5883L 3-Axis Digital Compass.
 * @author Andrei Ruslantsev
 * @copyright (c) 2026 Andrei Ruslantsev. All rights reserved.
 * 
 * This software is provided "as is", without warranty of any kind, express or
 * implied, including but not limited to the warranties of merchantability,
 * fitness for a particular purpose and noninfringement.
 */

#include "hmc5883l.h"
#include "i2c.h"



#if defined(STM32)
/**
 * @brief  Provides a delay in milliseconds.
 *
 * This function pauses the program execution for a specific amount of time.
 *
 * @param[in] timeout  Delay duration in milliseconds (ms).
 *
 * @return None
 */
static void delay_ms(uint32_t timeout) {
    HAL_Delay(timeout);
}
#elif defined(ESP32)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief  Provides a delay in milliseconds.
 *
 * This function pauses the program execution for a specific amount of time.
 *
 * @param[in] timeout  Delay duration in milliseconds (ms).
 *
 * @return None
 */
static void delay_ms(uint32_t timeout) {
    const TickType_t xDelay = timeout / portTICK_PERIOD_MS;
    vTaskDelay(xDelay);
}
#else
#error No MCU defined
#endif


/**
  * @brief Copies Configuration register A (CRA) to uint8_t
  * @param HMC5883L *: pointer to HMC5883L structure
  * @param uint8_t *: pointer to uint8_t, where value of CRA will be saved
 */
static HMC5883L_STATUS hmc5883l_get_cra(HMC5883L *hmc5883l, uint8_t *cra) {
    const I2CStatus i2c_ret = i2c_read(
        hmc5883l->i2c_bus, 
        hmc5883l->addr,
        HMC5883L_CRA,
        cra, 
        1, 
        HMC5883L_TIMEOUT_MS
    );
    return (i2c_ret == I2C_OK) ? HMC5883L_OK : HMC5883L_READ_ERR;
}


/**
  * @brief Copies specified uint8_t value to Configuration register A (CRA)
  * @param HMC5883L *: pointer to HMC5883L structure
  * @param uint8_t: integer with desired value of CRA
 */
static HMC5883L_STATUS hmc5883l_set_cra(HMC5883L *hmc5883l, uint8_t *cra) {
    const I2CStatus i2c_ret = i2c_write(
        hmc5883l->i2c_bus,
        hmc5883l->addr,
        HMC5883L_CRA,
        cra,
        1,
        HMC5883L_TIMEOUT_MS
    );
    return (i2c_ret == I2C_OK) ? HMC5883L_OK : HMC5883L_WRITE_ERR;
}


/**
  * @brief Copies Configuration register B (CRB) to uint8_t
  * @param HMC5883L *: pointer to HMC5883L structure
  * @param uint8_t *: pointer to uint8_t, where value of CRB will be saved
 */
static HMC5883L_STATUS hmc5883l_get_crb(HMC5883L *hmc5883l, uint8_t *crb) {
    const I2CStatus i2c_ret = i2c_read(
        hmc5883l->i2c_bus,
        hmc5883l->addr,
        HMC5883L_CRB,
        crb,
        1,
        HMC5883L_TIMEOUT_MS
    );
    return (i2c_ret == I2C_OK) ? HMC5883L_OK : HMC5883L_READ_ERR;
}


/**
  * @brief Copies specified uint8_t value to Configuration register B (CRB)
  * @param HMC5883L *: pointer to HMC5883L structure
  * @param uint8_t: integer with desired value of CRB
 */
static HMC5883L_STATUS hmc5883l_set_crb(HMC5883L *hmc5883l, uint8_t *crb) {
    const I2CStatus i2c_ret = i2c_write(
        hmc5883l->i2c_bus,
        hmc5883l->addr,
        HMC5883L_CRB,
        crb,
        1,
        HMC5883L_TIMEOUT_MS
    );
    return (i2c_ret == I2C_OK) ? HMC5883L_OK : HMC5883L_WRITE_ERR;
}


/**
  * @brief Copies Mode register (MR) to uint8_t
  * @param HMC5883L *: pointer to HMC5883L structure
  * @param uint8_t *: pointer to uint8_t, where value of MR will be saved
 */
static HMC5883L_STATUS hmc5883l_get_mr(HMC5883L *hmc5883l, uint8_t *mr) {
    const I2CStatus i2c_ret = i2c_read(
        hmc5883l->i2c_bus,
        hmc5883l->addr,
        HMC5883L_MR,
        mr,
        1,
        HMC5883L_TIMEOUT_MS
    );
    return (i2c_ret == I2C_OK) ? HMC5883L_OK : HMC5883L_READ_ERR;
}


/**
  * @brief Copies specified uint8_t value to Mode register (MR)
  * @param HMC5883L *: pointer to HMC5883L structure
  * @param uint8_t: integer with desired value of MR
 */
static HMC5883L_STATUS hmc5883l_set_mr(HMC5883L *hmc5883l, uint8_t *mr) {
    const I2CStatus i2c_ret = i2c_write(
        hmc5883l->i2c_bus,
        hmc5883l->addr,
        HMC5883L_MR,
        mr,
        1,
        HMC5883L_TIMEOUT_MS
    );
    return (i2c_ret == I2C_OK) ? HMC5883L_OK : HMC5883L_WRITE_ERR;
}


/**
  * @brief Copies Status register (SR) to uint8_t
  * @param HMC5883L *: pointer to HMC5883L structure
  * @param uint8_t *: pointer to uint8_t, where value of SR will be saved
 */
static HMC5883L_STATUS hmc5883l_get_sr(HMC5883L *hmc5883l, uint8_t *sr) {
    const I2CStatus i2c_ret = i2c_read(
        hmc5883l->i2c_bus,
        hmc5883l->addr,
        HMC5883L_SR,
        sr,
        1,
        HMC5883L_TIMEOUT_MS
    );
    return (i2c_ret == I2C_OK) ? HMC5883L_OK : HMC5883L_READ_ERR;
}


#if defined(STM32)
HMC5883L_STATUS hmc5883l_init(HMC5883L *hmc5883l, I2C_HandleTypeDef *i2c_bus, uint16_t addr) {
#elif defined(ESP32)
HMC5883L_STATUS hmc5883l_init(HMC5883L *hmc5883l, i2c_master_bus_handle_t *i2c_bus, const uint8_t addr) {
#else
#error No MCU defined
#endif
    hmc5883l->i2c_bus = i2c_bus;
#if defined(STM32)
    hmc5883l->addr = addr;
#elif defined(ESP32)
    i2c_master_dev_handle_t dev_handle;
    i2c_device_config_t dev_config = {
        .dev_addr_length = 0, /*I2C_ADDR_BIT_LEN_7 */
        .device_address = addr,
        .scl_speed_hz = 400000, /* CONFIG_I2C_MASTER_FREQUENCY */
    };
    const esp_err_t ret = i2c_master_bus_add_device(*i2c_bus, &dev_config, &dev_handle);
    if (ret != ESP_OK) return HMC5883L_READ_ERR;
    hmc5883l->addr = dev_handle;
#else
#error No MCU defined
#endif
    
    uint8_t buf[3];

    /* Check identification registers */
    if (
        i2c_read(
            hmc5883l->i2c_bus,
            hmc5883l->addr,
            HMC5883L_IDA,
            buf,
            3,
            HMC5883L_TIMEOUT_MS
        ) != I2C_OK
    ) return HMC5883L_NORESPONSE;
    if (buf[0] != 'H' || buf[1] != '4' || buf[2] != '3') return HMC5883L_NOTFOUND;

    /* Reset bits in registers, see datasheet */
    HMC5883L_STATUS cret;
    uint8_t reg;
    cret = hmc5883l_get_cra(hmc5883l, &reg);
    if (cret != HMC5883L_OK) return cret;
    reg = reg & ~(1 << 7); /* Clean cra[7] */
    cret = hmc5883l_set_cra(hmc5883l, &reg);
    if (cret != HMC5883L_OK) return cret;

    cret = hmc5883l_get_crb(hmc5883l, &reg);
    if (cret != HMC5883L_OK) return cret;
    reg = reg & 0b11100000; /* Clean crb[0] - crb[4] */
    cret = hmc5883l_set_crb(hmc5883l, &reg);
    if (cret != HMC5883L_OK) return cret;

    cret = hmc5883l_get_mr(hmc5883l, &reg);
    if (cret != HMC5883L_OK) return cret;
    reg = reg & 0b00000011; /* Clean mr[2] - mr[7] */
    cret = hmc5883l_set_mr(hmc5883l, &reg);
    if (cret != HMC5883L_OK) return cret;

    return HMC5883L_OK;
}


HMC5883L_STATUS hmc5883l_get_number_of_samples(HMC5883L *hmc5883l, HMC5883L_SAMPLING *sampling) {
    uint8_t cra;
    HMC5883L_STATUS ret;
    ret = hmc5883l_get_cra(hmc5883l, &cra);
    if (ret != HMC5883L_OK) return ret;
    *sampling = (HMC5883L_SAMPLING) ((cra & HMC5883L_CRA_SAMPLES_MASK) >> HMC5883L_CRA_SAMPLES_POS);
    return HMC5883L_OK;
}


HMC5883L_STATUS hmc5883l_set_number_of_samples(HMC5883L *hmc5883l, HMC5883L_SAMPLING sampling) {
    uint8_t cra;
    HMC5883L_STATUS ret;
    if (sampling < HMC5883L_SAMPLING_SINGLE || sampling > HMC5883L_SAMPLING_OCT) return HMC5883L_VALUE_ERR;
    ret = hmc5883l_get_cra(hmc5883l, &cra);
    if (ret != HMC5883L_OK) return ret;
    cra &= ~HMC5883L_CRA_SAMPLES_MASK;
    cra |= (sampling << HMC5883L_CRA_SAMPLES_POS) & HMC5883L_CRA_SAMPLES_MASK;
    return hmc5883l_set_cra(hmc5883l, &cra);
}


HMC5883L_STATUS hmc5883l_get_output_rate(HMC5883L *hmc5883l, HMC5883L_OUTPUT_RATE *rate) {
    uint8_t cra;
    HMC5883L_STATUS ret;
    ret = hmc5883l_get_cra(hmc5883l, &cra);
    if (ret != HMC5883L_OK) return ret;
    *rate = (HMC5883L_OUTPUT_RATE) ((cra & HMC5883L_CRA_RATE_MASK) >> HMC5883L_CRA_RATE_POS);
    return HMC5883L_OK;
}


HMC5883L_STATUS hmc5883l_set_output_rate(HMC5883L *hmc5883l, HMC5883L_OUTPUT_RATE rate) {
    uint8_t cra;
    HMC5883L_STATUS ret;
    if (rate < HMC5883L_OUTPUT_RATE_0075_HZ || rate > HMC5883L_OUTPUT_RATE_7500_HZ) return HMC5883L_VALUE_ERR;
    ret = hmc5883l_get_cra(hmc5883l, &cra);
    if (ret != HMC5883L_OK) return ret;
    cra &= ~HMC5883L_CRA_RATE_MASK;
    cra |= (rate << HMC5883L_CRA_RATE_POS) & HMC5883L_CRA_RATE_MASK;
    return hmc5883l_set_cra(hmc5883l, &cra);
}


HMC5883L_STATUS hmc5883l_get_measurement_mode(HMC5883L *hmc5883l, HMC5883L_MEASUREMENT_MODE *mode) {
    uint8_t cra;
    HMC5883L_STATUS ret;
    ret = hmc5883l_get_cra(hmc5883l, &cra);
    if (ret != HMC5883L_OK) return ret;
    *mode = (HMC5883L_MEASUREMENT_MODE) ((cra & HMC5883L_CRA_MEAS_CONF_MASK) >> HMC5883L_CRA_MEAS_CONF_POS);
    return HMC5883L_OK;
}


HMC5883L_STATUS hmc5883l_set_measurement_mode(HMC5883L *hmc5883l, HMC5883L_MEASUREMENT_MODE mode) {
    uint8_t cra;
    HMC5883L_STATUS ret;
    if (mode < HMC5883L_MEASUREMENT_MODE_NORMAL || mode > HMC5883L_MEASUREMENT_MODE_NEG_BIAS) return HMC5883L_VALUE_ERR;
    ret = hmc5883l_get_cra(hmc5883l, &cra);
    if (ret != HMC5883L_OK) return ret;
    cra &= ~HMC5883L_CRA_MEAS_CONF_MASK;
    cra |= (mode << HMC5883L_CRA_MEAS_CONF_POS) & HMC5883L_CRA_MEAS_CONF_MASK;
    return hmc5883l_set_cra(hmc5883l, &cra);
}


HMC5883L_STATUS hmc5883l_get_gain(HMC5883L *hmc5883l, HMC5883L_GAIN *gain) {
    uint8_t crb;
    HMC5883L_STATUS ret;
    ret = hmc5883l_get_crb(hmc5883l, &crb);
    if (ret != HMC5883L_OK) return ret;
    *gain = (HMC5883L_GAIN) ((crb & HMC5883L_CRB_GAIN_MASK) >> HMC5883L_CRB_GAIN_POS);
    return HMC5883L_OK;
}


HMC5883L_STATUS hmc5883l_set_gain(HMC5883L *hmc5883l, HMC5883L_GAIN gain) {
    uint8_t crb;
    HMC5883L_STATUS ret;
    if (gain < HMC5883L_GAIN_1370 || gain > HMC5883L_GAIN_230) return HMC5883L_VALUE_ERR;
    ret = hmc5883l_get_crb(hmc5883l, &crb);
    if (ret != HMC5883L_OK) return ret;
    crb &= ~HMC5883L_CRB_GAIN_MASK;
    crb |= (gain << HMC5883L_CRB_GAIN_POS) & HMC5883L_CRB_GAIN_MASK;
    return hmc5883l_set_crb(hmc5883l, &crb);
}


HMC5883L_STATUS hmc5883l_get_operating_mode(HMC5883L *hmc5883l, HMC5883L_OPERATING_MODE *mode) {
    uint8_t mr;
    HMC5883L_STATUS ret;
    ret = hmc5883l_get_mr(hmc5883l, &mr);
    if (ret != HMC5883L_OK) return ret;
    *mode = (HMC5883L_OPERATING_MODE) ((mr & HMC5883L_MR_MODE_MASK) >> HMC5883L_MR_MODE_POS);
    return HMC5883L_OK;
}


HMC5883L_STATUS hmc5883l_set_operating_mode(HMC5883L *hmc5883l, HMC5883L_OPERATING_MODE mode) {
    uint8_t mr;
    HMC5883L_STATUS ret;
    if (mode < HMC5883L_OPERATING_MODE_CONTINUOUS || mode > HMC5883L_OPERATING_MODE_IDLE1) return HMC5883L_VALUE_ERR;
    ret = hmc5883l_get_mr(hmc5883l, &mr);
    if (ret != HMC5883L_OK) return ret;
    mr &= ~HMC5883L_MR_MODE_MASK;
    mr |= (mode << HMC5883L_MR_MODE_POS) & HMC5883L_MR_MODE_MASK;
    return hmc5883l_set_mr(hmc5883l, &mr);
}


/**
 * @brief Reads the magnetic field data from the sensor. Fills fields x, y, z in hmc5883l structure
 * @param[in,out] hmc5883l Pointer to driver handle.
 * @return HMC5883L_STATUS (HMC5883L_OK on success, HMC5883L_OVERFLOW if saturation occurs).
 */
static HMC5883L_STATUS hmc5883l_read(HMC5883L* hmc5883l) {
    uint8_t buf[6];

    if (
        i2c_read(
            hmc5883l->i2c_bus,
            hmc5883l->addr,
            HMC5883L_X_MSB,
            buf,
            6,
            HMC5883L_TIMEOUT_MS
        ) != I2C_OK
    ) return HMC5883L_READ_ERR;
    hmc5883l->x = (int16_t) ((buf[0] << 8) | buf[1]);
    hmc5883l->z = (int16_t) ((buf[2] << 8) | buf[3]);
    hmc5883l->y = (int16_t) ((buf[4] << 8) | buf[5]);

    if (hmc5883l->x == -4096 || hmc5883l->y == -4096 || hmc5883l->z == -4096) {
        return HMC5883L_OVERFLOW;
    }

    return HMC5883L_OK;
}


/**
 * @brief Checks of ready bit set
 * @param[in]  hmc5883l Pointer to driver handle.
 * @param[out] is_ready Pointer to ready status
 * @return HMC5883L_STATUS
 */
static HMC5883L_STATUS hmc5883l_is_ready(HMC5883L *hmc5883l, uint8_t *is_ready) {
    uint8_t sr;
    HMC5883L_STATUS ret;
    ret = hmc5883l_get_sr(hmc5883l, &sr);
    if (ret != HMC5883L_OK) return ret;
    *is_ready = (sr & HMC5883L_SR_READY_MASK) != 0 ? 1: 0;
    return HMC5883L_OK;
}


/**
 * @brief Checks if locked bit set
 * @param[in]  hmc5883l Pointer to driver handle.
 * @param[out] is_locked Pointer to locked status
 * @return HMC5883L_STATUS
 */
static HMC5883L_STATUS hmc5883l_is_locked(HMC5883L *hmc5883l, uint8_t *is_locked) {
    uint8_t sr;
    HMC5883L_STATUS ret;
    ret = hmc5883l_get_sr(hmc5883l, &sr);
    if (ret != HMC5883L_OK) return ret;
    *is_locked = (sr & HMC5883L_SR_LOCK_MASK) != 0 ? 1: 0;
    return HMC5883L_OK;
}


/**
 * @brief Waits for ready bin set
 * @param[in] hmc5883l Pointer to driver handle.
 * @param[in] timeout Maximal timeout
 * @return HMC5883L_STATUS
 */
static HMC5883L_STATUS hmc5883l_wait_ready(HMC5883L *hmc5883l, uint8_t timeout_ms) {
    HMC5883L_STATUS ret;
    uint8_t is_ready = 0;
    const uint8_t dt = 1;
    uint8_t curr_time = 0;
    while (!is_ready) {
        ret = hmc5883l_is_ready(hmc5883l, &is_ready);
        if (ret != HMC5883L_OK) return ret;
        delay_ms(dt);
        curr_time += dt;
        if (curr_time >= timeout_ms) return HMC5883L_TIMEOUT_ERR;
    }
    return HMC5883L_OK;
}


HMC5883L_STATUS hmc5883l_single_measurement(HMC5883L *hmc5883l) {
    HMC5883L_STATUS ret;
    ret = hmc5883l_set_operating_mode(hmc5883l, HMC5883L_OPERATING_MODE_SINGLE);
    if (ret != HMC5883L_OK) return ret;
    ret = hmc5883l_wait_ready(hmc5883l, HMC5883L_TIMEOUT_MS);
    if (ret != HMC5883L_OK) return ret;
    ret = hmc5883l_read(hmc5883l);
    if (ret != HMC5883L_OK) return ret;
    return HMC5883L_OK;
}


HMC5883L_STATUS hmc5883l_measurement(HMC5883L *hmc5883l) {
    HMC5883L_STATUS ret;
    ret = hmc5883l_wait_ready(hmc5883l, HMC5883L_TIMEOUT_MS);
    if (ret != HMC5883L_OK) return ret;
    return hmc5883l_read(hmc5883l);
}
