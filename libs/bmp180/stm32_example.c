#define STM32H5
#include "bmp180.h"

int main(void)
{
    /* Initialize BMP180 and set oversampling */
    BMP180 bmp180;
    if (bmp180_init(&bmp180, &hi2c1, 0x77) != BMP180_OK) {
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
