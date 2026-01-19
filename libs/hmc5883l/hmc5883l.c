#include "hmc5883l.h"


/**
  * @brief Copies Configuration register A (CRA) to uint8_t
  * @param HMC5883L *: pointer to HMC5883L structure
  * @param uint8_t *: pointer to uint8_t, where value of CRA will be saved
 */
static HMC5883L_STATUS hmc5883l_get_cra(HMC5883L *hmc5883l, uint8_t *cra) {
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Mem_Read(
        hmc5883l->i2c_bus,
        (hmc5883l->addr) << 1,
        0x00,
        I2C_MEMADD_SIZE_8BIT,
        (uint8_t *) cra,
        1,
        100
    );
    if (ret != HAL_OK)
        return HMC5883L_READ_ERR;
    return HMC5883L_OK;
}


/**
  * @brief Copies specified uint8_t value to Configuration register A (CRA)
  * @param HMC5883L *: pointer to HMC5883L structure
  * @param uint8_t: integer with desired value of CRA
 */
static HMC5883L_STATUS hmc5883l_set_cra(HMC5883L *hmc5883l, uint8_t cra) {
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Mem_Write(
        hmc5883l->i2c_bus,
        (hmc5883l->addr) << 1,
        0x00,
        I2C_MEMADD_SIZE_8BIT,
        (uint8_t *) &cra,
        1,
        100
    );
    if (ret != HAL_OK)
        return HMC5883L_WRITE_ERR;
    return HMC5883L_OK;
}


/**
  * @brief Copies Configuration register B (CRB) to uint8_t
  * @param HMC5883L *: pointer to HMC5883L structure
  * @param uint8_t *: pointer to uint8_t, where value of CRB will be saved
 */
static HMC5883L_STATUS hmc5883l_get_crb(HMC5883L *hmc5883l, uint8_t *crb) {
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Mem_Read(
        hmc5883l->i2c_bus,
        (hmc5883l->addr) << 1,
        0x01,
        I2C_MEMADD_SIZE_8BIT,
        (uint8_t *) crb,
        1,
        100
    );
    if (ret != HAL_OK)
        return HMC5883L_READ_ERR;
    return HMC5883L_OK;
}


/**
  * @brief Copies specified uint8_t value to Configuration register B (CRB)
  * @param HMC5883L *: pointer to HMC5883L structure
  * @param uint8_t: integer with desired value of CRB
 */
static HMC5883L_STATUS hmc5883l_set_crb(HMC5883L *hmc5883l, uint8_t crb) {
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Mem_Write(
        hmc5883l->i2c_bus,
        (hmc5883l->addr) << 1,
        0x01,
        I2C_MEMADD_SIZE_8BIT,
        (uint8_t *) &crb,
        1,
        100
    );
    if (ret != HAL_OK)
        return HMC5883L_WRITE_ERR;
    return HMC5883L_OK;
}


/**
  * @brief Copies Mode register (MR) to uint8_t
  * @param HMC5883L *: pointer to HMC5883L structure
  * @param uint8_t *: pointer to uint8_t, where value of MR will be saved
 */
static HMC5883L_STATUS hmc5883l_get_mr(HMC5883L *hmc5883l, uint8_t *mr) {
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Mem_Read(
        hmc5883l->i2c_bus,
        (hmc5883l->addr) << 1,
        0x02,
        I2C_MEMADD_SIZE_8BIT,
        (uint8_t *) mr,
        1,
        100
    );
    if (ret != HAL_OK)
        return HMC5883L_READ_ERR;
    return HMC5883L_OK;
}


/**
  * @brief Copies specified uint8_t value to Mode register (MR)
  * @param HMC5883L *: pointer to HMC5883L structure
  * @param uint8_t: integer with desired value of MR
 */
static HMC5883L_STATUS hmc5883l_set_mr(HMC5883L *hmc5883l, uint8_t mr) {
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Mem_Write(
        hmc5883l->i2c_bus,
        (hmc5883l->addr) << 1,
        0x02,
        I2C_MEMADD_SIZE_8BIT,
        (uint8_t *) &mr,
        1,
        100
    );
    if (ret != HAL_OK)
        return HMC5883L_WRITE_ERR;
    return HMC5883L_OK;
}


/**
  * @brief Copies Status register (SR) to uint8_t
  * @param HMC5883L *: pointer to HMC5883L structure
  * @param uint8_t *: pointer to uint8_t, where value of SR will be saved
 */
static HMC5883L_STATUS hmc5883l_get_sr(HMC5883L *hmc5883l, uint8_t *sr) {
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Mem_Read(
        hmc5883l->i2c_bus,
        (hmc5883l->addr) << 1,
        0x09,
        I2C_MEMADD_SIZE_8BIT,
        (uint8_t *) sr,
        1,
        100
    );
    if (ret != HAL_OK)
        return HMC5883L_READ_ERR;
    return HMC5883L_OK;
}


HMC5883L_STATUS hmc5883l_reset_mr7(HMC5883L *hmc5883l) {
    HMC5883L_STATUS ret;
    uint8_t mr;
    ret = hmc5883l_get_mr(hmc5883l, &mr);
    if (ret != HMC5883L_OK)
        return ret;
    mr = mr & 0b01111111;
    ret = hmc5883l_set_mr(hmc5883l, mr);
    if (ret != HMC5883L_OK)
        return ret;
    return HMC5883L_OK;
}


HMC5883L_STATUS hmc5883l_get_mr7(HMC5883L *hmc5883l, uint8_t *mr7_is_set) {
    HMC5883L_STATUS ret;
    uint8_t mr;
    ret = hmc5883l_get_mr(hmc5883l, &mr);
    if (ret != HMC5883L_OK)
        return ret;
    *mr7_is_set = (mr & 0b10000000) != 0? 1: 0;
    return HMC5883L_OK;
}


HMC5883L_STATUS hmc5883l_init(HMC5883L *hmc5883l, I2C_HandleTypeDef *i2c_bus, uint16_t addr) {
    hmc5883l->i2c_bus = i2c_bus;
    hmc5883l->addr = addr;
    HAL_StatusTypeDef ret;
    uint8_t buf[3];

    /* Check identification registers */
    ret = HAL_I2C_Mem_Read(
        hmc5883l->i2c_bus,
        (hmc5883l->addr) << 1,
        0x0A,
        I2C_MEMADD_SIZE_8BIT,
        (uint8_t *) buf,
        3,
        100
    );
    if (ret != HAL_OK)
        return HMC5883L_NORESPONSE;
    if (buf[0] != 'H' || buf[1] != '4' || buf[2] != '3')
        return HMC5883L_NOTFOUND;

    /* Reset bits in registers, see datasheet */
    HMC5883L_STATUS cret;
    uint8_t reg;
    cret = hmc5883l_get_cra(hmc5883l, &reg);
    if (cret != HMC5883L_OK)
        return cret;
    reg = reg & 0b01111111;
    cret = hmc5883l_set_cra(hmc5883l, reg);
    if (cret != HMC5883L_OK)
        return cret;

    cret = hmc5883l_get_crb(hmc5883l, &reg);
    if (cret != HMC5883L_OK)
        return cret;
    reg = reg & 0b11100000;
    cret = hmc5883l_set_crb(hmc5883l, reg);
    if (cret != HMC5883L_OK)
        return cret;

    cret = hmc5883l_get_mr(hmc5883l, &reg);
    if (cret != HMC5883L_OK)
        return cret;
    reg = reg & 0b10000011;
    cret = hmc5883l_set_mr(hmc5883l, reg);
    if (cret != HMC5883L_OK)
        return cret;

    cret = hmc5883l_reset_mr7(hmc5883l);
    if (cret != HMC5883L_OK)
        return cret;

    return HMC5883L_OK;
}


HMC5883L_STATUS hmc5883l_get_number_of_samples(HMC5883L *hmc5883l, uint8_t *number_of_samples) {
    uint8_t cra;
    HMC5883L_STATUS ret;
    ret = hmc5883l_get_cra(hmc5883l, &cra);
    if (ret != HMC5883L_OK)
        return ret;
    *number_of_samples = (cra & 0b01100000) >> 5;
    return HMC5883L_OK;
}


HMC5883L_STATUS hmc5883l_set_number_of_samples(HMC5883L *hmc5883l, uint8_t number_of_samples) {
    uint8_t cra;
    HMC5883L_STATUS ret;
    if (number_of_samples < 0 || number_of_samples > 3)
        return HMC5883L_VALUE_ERR;
    ret = hmc5883l_get_cra(hmc5883l, &cra);
    if (ret != HMC5883L_OK)
        return ret;
    cra = (cra & 0b10011111) | (number_of_samples << 5);
    ret = hmc5883l_set_cra(hmc5883l, cra);
    if (ret != HMC5883L_OK)
        return ret;
    return HMC5883L_OK;
}


HMC5883L_STATUS hmc5883l_get_output_rate(HMC5883L *hmc5883l, uint8_t *rate) {
    uint8_t cra;
    HMC5883L_STATUS ret;
    ret = hmc5883l_get_cra(hmc5883l, &cra);
    if (ret != HMC5883L_OK)
        return ret;
    *rate = (cra & 0b00011100) >> 2;
    return HMC5883L_OK;
}


HMC5883L_STATUS hmc5883l_set_output_rate(HMC5883L *hmc5883l, uint8_t rate) {
    uint8_t cra;
    HMC5883L_STATUS ret;
    if (rate < 0 || rate > 6)  // 7 is not used
        return HMC5883L_VALUE_ERR;
    ret = hmc5883l_get_cra(hmc5883l, &cra);
    if (ret != HMC5883L_OK)
        return ret;
    cra = (cra & 0b11100011) | (rate << 2);
    ret = hmc5883l_set_cra(hmc5883l, cra);
    if (ret != HMC5883L_OK)
        return ret;
    return HMC5883L_OK;
}


HMC5883L_STATUS hmc5883l_get_measurement_configuration(HMC5883L *hmc5883l, uint8_t *configuration) {
    uint8_t cra;
    HMC5883L_STATUS ret;
    ret = hmc5883l_get_cra(hmc5883l, &cra);
    if (ret != HMC5883L_OK)
        return ret;
    *configuration = (cra & 0b00000011);
    return HMC5883L_OK;
}


HMC5883L_STATUS hmc5883l_set_measurement_configuration(HMC5883L *hmc5883l, uint8_t configuration) {
    uint8_t cra;
    HMC5883L_STATUS ret;
    if (configuration < 0 || configuration > 2)  // 3 is not used
        return HMC5883L_VALUE_ERR;
    ret = hmc5883l_get_cra(hmc5883l, &cra);
    if (ret != HMC5883L_OK)
        return ret;
    cra = (cra & 0b11111100) | configuration;
    ret = hmc5883l_set_cra(hmc5883l, cra);
    if (ret != HMC5883L_OK)
        return ret;
    return HMC5883L_OK;
}


HMC5883L_STATUS hmc5883l_get_gain(HMC5883L *hmc5883l, uint8_t *gain) {
    uint8_t crb;
        HMC5883L_STATUS ret;
        ret = hmc5883l_get_crb(hmc5883l, &crb);
        if (ret != HMC5883L_OK)
            return ret;
        *gain = (crb & 0b11100000) >> 5;
    return HMC5883L_OK;
}


HMC5883L_STATUS hmc5883l_set_gain(HMC5883L *hmc5883l, uint8_t gain) {
    uint8_t crb;
        HMC5883L_STATUS ret;
        if (gain < 0 || gain > 7)
            return HMC5883L_VALUE_ERR;
        ret = hmc5883l_get_crb(hmc5883l, &crb);
        if (ret != HMC5883L_OK)
            return ret;
        crb = (crb & 0b00011111) | (gain << 5);
        ret = hmc5883l_set_crb(hmc5883l, crb);
        if (ret != HMC5883L_OK)
            return ret;
    return HMC5883L_OK;
}


HMC5883L_STATUS hmc5883l_get_operating_mode(HMC5883L *hmc5883l, uint8_t *mode) {
    uint8_t mr;
        HMC5883L_STATUS ret;
        ret = hmc5883l_get_mr(hmc5883l, &mr);
        if (ret != HMC5883L_OK)
            return ret;
        *mode = (mr & 0b00000011);
    return HMC5883L_OK;
}


HMC5883L_STATUS hmc5883l_set_operating_mode(HMC5883L *hmc5883l, uint8_t mode) {
    uint8_t mr;
        HMC5883L_STATUS ret;
        if (mode < 0 || mode > 3)
            return HMC5883L_VALUE_ERR;
        ret = hmc5883l_get_mr(hmc5883l, &mr);
        if (ret != HMC5883L_OK)
            return ret;
        mr = (mr & 0b11111100) | mode;
        ret = hmc5883l_set_mr(hmc5883l, mr);
        if (ret != HMC5883L_OK)
            return ret;
    return HMC5883L_OK;
}


HMC5883L_STATUS hmc5883l_read(HMC5883L* hmc5883l) {
    uint8_t buf[6];
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Mem_Read(
        hmc5883l->i2c_bus,
        (hmc5883l->addr) << 1,
        0x03,
        I2C_MEMADD_SIZE_8BIT,
        (uint8_t *) buf,
        6,
        100
    );
    if (ret != HAL_OK)
        return HMC5883L_READ_ERR;
    hmc5883l->x = (int16_t) ((buf[0] << 8) | buf[1]);
    hmc5883l->z = (int16_t) ((buf[2] << 8) | buf[3]);
    hmc5883l->y = (int16_t) ((buf[4] << 8) | buf[5]);

    return HMC5883L_OK;
}


HMC5883L_STATUS hmc5883l_is_ready(HMC5883L *hmc5883l, uint8_t *is_ready) {
    uint8_t sr;
    HMC5883L_STATUS ret;
    ret = hmc5883l_get_sr(hmc5883l, &sr);
    if (ret != HMC5883L_OK)
        return ret;
    *is_ready = (sr & 0b00000001) != 0? 1: 0;
    return HMC5883L_OK;
}


HMC5883L_STATUS hmc5883l_is_locked(HMC5883L *hmc5883l, uint8_t *is_locked) {
    uint8_t sr;
    HMC5883L_STATUS ret;
    ret = hmc5883l_get_sr(hmc5883l, &sr);
    if (ret != HMC5883L_OK)
        return ret;
    *is_locked = (sr & 0b00000010) != 0? 1: 0;
    return HMC5883L_OK;
}



HMC5883L_STATUS hmc5883l_wait_ready(HMC5883L *hmc5883l, uint8_t timeout_ms) {
    HMC5883L_STATUS ret;
    uint8_t is_ready = 0, dt = 1;
    uint8_t curr_time = 0;
    do {
        ret = hmc5883l_is_ready(hmc5883l, &is_ready);
        if (ret != HMC5883L_OK)
            return ret;
        HAL_Delay(dt);
        curr_time += dt;
        if (curr_time >= timeout_ms)
            return HMC5883L_TIMEOUT_ERR;
    } while (!is_ready);
    return HMC5883L_OK;
}


HMC5883L_STATUS hmc5883l_single_measurement(HMC5883L *hmc5883l) {
    HMC5883L_STATUS ret;
    ret = hmc5883l_set_operating_mode(hmc5883l, 1U);
    if (ret != HMC5883L_OK)
        return ret;
    ret = hmc5883l_wait_ready(hmc5883l, 100);
    if (ret != HMC5883L_OK)
        return ret;
    ret = hmc5883l_read(hmc5883l);
    if (ret != HMC5883L_OK)
        return ret;
    return HMC5883L_OK;
}


HMC5883L_STATUS hmc5883l_continuous_measurement(HMC5883L *hmc5883l) {
    HMC5883L_STATUS ret;
    ret = hmc5883l_wait_ready(hmc5883l, 100);
    if (ret != HMC5883L_OK)
        return ret;
    ret = hmc5883l_read(hmc5883l);
    if (ret != HMC5883L_OK)
        return ret;
    return HMC5883L_OK;
}
