# BMP180 Sensor Library

A C library for the **Honeywell HMC5883L** 3-axis digital compass. This driver provides high-level functions for data acquisition.

It is designed to be used in conjunction with the I2C Abstraction Layer, ensuring seamless portability between **STM32** and **ESP32** platforms.

## Features
- **Platform Agnostic**: Works on STM32 (all families) and ESP32 via a unified I2C HAL.
- **Configurable**: Gain, data output rate, measurement configuration, operation mode can be adjusted.
- **Error Diagnostics**: Returns detailed status codes for initialization and communication.

---

## Installation

1. Add `hmc5883l.h` and `hmc5883l.c` to your project's source directory.
2. Ensure the `i2c.h` abstraction layer is present and configured for your specific MCU family.
3. Include the header in your application:
   ```c
   #include "hmc5883l.h"
   ```

---

## API Reference

### Initialization

```c
HMC5883L_STATUS hmc5883l_init(HMC5883L *hmc5883l, I2C_BUS_TYPE *i2c_bus, uint16_t addr)
```

1. Verifies Chip ID (0x55).

2. Resets internal registers.

### Configuration

```c
HMC5883L_STATUS hmc5883l_set_number_of_samples(HMC5883L *hmc5883l, HMC5883L_SAMPLING sampling)
HMC5883L_STATUS hmc5883l_set_operating_mode(HMC5883L *hmc5883l, HMC5883L_OPERATING_MODE mode);
HMC5883L_STATUS hmc5883l_set_measurement_mode(HMC5883L *hmc5883l, HMC5883L_MEASUREMENT_MODE mode);
HMC5883L_STATUS hmc5883l_set_output_rate(HMC5883L *hmc5883l, HMC5883L_OUTPUT_RATE rate);
HMC5883L_STATUS hmc5883l_set_gain(HMC5883L *hmc5883l, HMC5883L_GAIN gain);
BMP180_STATUS bmp180_set_oversampling(BMP180 *bmp180, BMP180_OSS oss)
```

### Measurement Functions

- `hmc5883l_single_measurement(...)`: Sets mode to single and performs measurement.

- `hmc5883l_measurement(...)`: Performs measurement.

--

## TODO

- Add support for DRDY pin
