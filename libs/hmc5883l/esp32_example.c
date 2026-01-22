#define ESP32
#include "hmc5883l.h"

void app_main(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_new_master_bus(&bus_config, &bus_handle);
    HMC5883L hmc5883l;
    if (hmc5883l_init(&hmc5883l, &bus_handle, 0x1E) != HMC5883L_OK) {
        puts("Error");
    }
    if (hmc5883l_set_number_of_samples(&hmc5883l, HMC5883L_SAMPLING_OCT) != HMC5883L_OK)  {
        puts("Error");
    }
    if (hmc5883l_set_operating_mode(&hmc5883l, HMC5883L_OPERATING_MODE_CONTINUOUS) != HMC5883L_OK)  {
        puts("Error");
    }
    if (hmc5883l_set_measurement_mode(&hmc5883l, HMC5883L_MEASUREMENT_MODE_NORMAL) != HMC5883L_OK)  {
        puts("Error");
    }
    if (hmc5883l_set_output_rate(&hmc5883l, HMC5883L_OUTPUT_RATE_1500_HZ) != HMC5883L_OK)  {
        puts("Error");
    }
    if (hmc5883l_set_gain(&hmc5883l, HMC5883L_GAIN_1090) != HMC5883L_OK)  {
        puts("Error");
    }

    while (1) {
        /* Perform measurements */
        if (hmc5883l_measurement(&hmc5883l) != HMC5883L_OK) {
            puts("Error");
        }
        printf(
            "X: %d, Y: %d, Z: %d\n", hmc5883l.x, hmc5883l.y, hmc5883l.z
        );
        /* Do something else here */
    }

}
