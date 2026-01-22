/* Simple calibration example. Do not use it in real life */

#include "stdint.h"
#include "hmc5883l.h"
#include "math.h"
#include <stdint.h>


#define min(a, b) (a) < (b) ? (a) : (b)
#define max(a, b) (a) > (b) ? (a) : (b)
#define NUM_MEASUREMENTS_MIN  256

HMC5883L_STATUS hmc5883l_calibrate(HMC5883L *hmc5883l, uint16_t timeout, int16_t *x_avg, int16_t *y_avg, int16_t *z_avg) {
    /* Set initial calibration data values */
    *x_avg = 0;
    *y_avg = 0;
    *z_avg = 0;

    HMC5883L_STATUS status;
    int16_t x_min, x_max, y_min, y_max, z_min, z_max;
    int16_t num_measurements = 0;

    /* Save initial operating mode and set continuous mode */
    HMC5883L_OPERATING_MODE mode;
    status = hmc5883l_get_operating_mode(hmc5883l, &mode);
    if (status != HMC5883L_OK) return status;
    status = hmc5883l_set_operating_mode(hmc5883l, HMC5883L_OPERATING_MODE_CONTINUOUS);
    if (status != HMC5883L_OK) return status;

    /* Set initial minimum and maximum values */
    status = hmc5883l_measurement(hmc5883l);
    if (status != HMC5883L_OK) return status;
    
    x_min = x_max = hmc5883l->x;
    y_min = y_max = hmc5883l->y;
    z_min = z_max = hmc5883l->z;

    uint32_t calibration_start = HAL_GetTick();

    /* Make measurements and save minimal and maximal values */
    /* Important: rotate device during calibration */
    while ((HAL_GetTick() - calibration_start) < (timeout * 1000)) {
        status = hmc5883l_measurement(hmc5883l);
        if (status == HMC5883L_OK) {
            x_min = min(x_min, hmc5883l->x);
            y_min = min(y_min, hmc5883l->y);
            z_min = min(z_min, hmc5883l->z);
            x_max = max(x_max, hmc5883l->x);
            y_max = max(y_max, hmc5883l->y);
            z_max = max(z_max, hmc5883l->z);
            num_measurements++;
        } else if (status != HMC5883L_OVERFLOW) {
            return status;
        }
        HAL_Delay(5);
    }

    /* Successful calibration: rewrite calibration data */
    if (num_measurements > NUM_MEASUREMENTS_MIN) {
        *x_avg = (x_min + x_max) / 2;
        *y_avg = (y_min + y_max) / 2;
        *z_avg = (z_min + z_max) / 2;
    }
    /* Set initial operating mode */
    return hmc5883l_set_operating_mode(hmc5883l, mode);
}


int main(void) {
    HMC5883L hmc5883l;
    hmc5883l_init(&hmc5883l, &hi2c1, 0x1E);
    hmc5883l_set_operating_mode(&hmc5883l, HMC5883L_OPERATING_MODE_CONTINUOUS);
    hmc5883l_set_measurement_mode(&hmc5883l, HMC5883L_MEASUREMENT_MODE_NORMAL);
    hmc5883l_set_output_rate(&hmc5883l, HMC5883L_OUTPUT_RATE_1500_HZ);
    hmc5883l_set_gain(&hmc5883l, HMC5883L_GAIN_1090);
  
    int16_t x_avg, y_avg, z_avg, curr_x, curr_y, curr_z;
    float heading;
    int16_t headingDegrees;

    hmc5883l_calibrate(&hmc5883l, 10, &x_avg, &y_avg, &z_avg);
    while (1) {
        hmc5883l_measurement(&hmc5883l);
        curr_x = hmc5883l.x - x_avg;
        curr_y = hmc5883l.y - y_avg;
        curr_z = hmc5883l.z - z_avg;
      
        heading = atan2((float) curr_y, (float) curr_x);
        headingDegrees = heading * 180.0 / M_PI;
        while (headingDegrees < 0) {
            headingDegrees += 360.0;
        }
        /* Do something */
    }
}
