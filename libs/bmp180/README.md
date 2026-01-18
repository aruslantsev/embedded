# BMP180 Sensor Library

A professional C library for the **BMP180** barometric pressure and temperature sensor. This driver provides high-level functions for data acquisition and implements the manufacturer's complex compensation algorithms for high-precision results.

It is designed to be used in conjunction with the I2C Abstraction Layer, ensuring seamless portability between **STM32** and **ESP32** platforms.



## Features
- **Platform Agnostic**: Works on STM32 (all families) and ESP32 via a unified I2C HAL.
- **Full Compensation**: Precise integer-based arithmetic for temperature and pressure calculation as per the Bosch datasheet.
- **Adjustable Resolution**: Full support for all Oversampling Settings (OSS).
- **Error Diagnostics**: Returns detailed status codes for initialization and communication.

---

## Technical Specifications

| Parameter | Value |
| :--- | :--- |
| **Pressure Range** | 300 to 1100 hPa |
| **Temperature Range** | -40 to +85 °C |
| **I2C Address** | 0x77 (7-bit) |
| **Supply Voltage** | 1.8V to 3.6V |

---

## Installation

1. Add `bmp180.h` and `bmp180.c` to your project's source directory.
2. Ensure the `i2c.h` abstraction layer is present and configured for your specific MCU family.
3. Include the header in your application:
   ```c
   #include "bmp180.h"
   ```

---

## API Reference

### Initialization

```
BMP180_STATUS bmp180_init(BMP180 *bmp180, I2C_BUS_TYPE *i2c_bus, uint16_t addr)
```

1. Performs soft reset.

2. Verifies Chip ID (0x55).

3. Reads and stores calibration coefficients from sensor EEPROM.

### Configuration

```
BMP180_STATUS bmp180_set_oversampling(BMP180 *bmp180, BMP180_OSS oss)
```

1. Updates the oversampling setting in the handle.

Note: This setting is applied to the hardware only during the next pressure measurement.

### Measurement Functions

- `bmp180_get_temperature(...)`: Updates raw ut and compensated temperature.

- `bmp180_get_pressure(...)`: Updates raw up and compensated pressure. Requires valid temperature data.

- `bmp180_update_all(...)`: Recommended function for most use cases; ensures correct sequence.

## Mathematical Compensation

The BMP180 provides "uncompensated" raw values. This library implements the following workflow internally:

- Read raw temperature -> Calculate true temperature.

- Use true temperature variables (B5​, etc.) + raw pressure -> Calculate true pressure.

The implementation uses fixed-point arithmetic to maintain high performance on microcontrollers without an FPU.
