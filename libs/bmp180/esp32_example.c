#define ESP32
#include "bmp180.h"

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
    BMP180 bmp180;
    if (bmp180_init(&bmp180, &bus_handle, 0x77) != BMP180_OK) {
        puts("Error");
    }
    if (bmp180_set_oversampling(&bmp180, 0) != BMP180_OK) {
        puts("Error");
    }

    while (1) {
        /* Perform measurements */
        if (bmp180_get_all(&bmp180) != BMP180_OK) {
            puts("Error");
        }
        printf(
            "Temperature: %lf deg Celsius. Pressure: %ld Pa\n", 
            bmp180.temperature / 10.0f, 
            bmp180.pressure
        );
        /* Do something else here */
    }

}
