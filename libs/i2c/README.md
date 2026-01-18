# I2C Abstraction Layer

A lightweight, cross-platform I2C hardware abstraction layer (HAL) designed to provide a unified interface for **STM32** and **ESP32** microcontrollers. This library allows higher-level sensor drivers to remain platform-independent.

## Features
- **Unified API**: Identical function signatures for different MCUs.
- **Platform Auto-switching**: Uses preprocessor definitions to toggle between STM32 HAL and ESP-IDF.
- **Register-oriented**: Built specifically for reading/writing memory-mapped registers (standard for most I2C sensors).
- **Address Handling**: Manages the difference between 7-bit and 8-bit addressing across platforms.

---

## Supported Architectures

The library automatically configures itself based on your compiler defines:

| Vendor | Supported Families |
| :--- | :--- |
| **STMicroelectronics** | C0, F0, F1, F3, F4, F7, G0, G4, H5, H7, L0, L1, L4, L5, WB, U5 |
| **Espressif** | ESP32, ESP32-S3, ESP32-C3 (via ESP-IDF I2C Master Driver) |

---

## Setup & Configuration

### 1. Define Your Target
Ensure your build system (CMake, Makefile, or IDE) has the family define active. For example, if you are using an STM32F407, the symbol `STM32F4` must be defined.

### 2. Include the Library
Simply include `i2c.h` in your project. It will handle the inclusion of the specific HAL headers.

```c
#include "i2c.h"
```

---

## API Reference

### Data Types
`I2CStatus`: Returns `I2C_OK` on success or `I2C_ERR` on failure.

### Functions

#### 1. `i2c_write`
Writes data to a specific device register.
- **STM32**: Handles the `dev_address << 1` shift required by HAL.
- **ESP32**: Implements a write buffer to combine `mem_address` and data into a single transmission.

#### 2. `i2c_read`
Reads data from a specific device register.
- **STM32**: Performs a memory read operation.
- **ESP32**: Executes a "transmit register address" followed by a "receive data" sequence (Repeated Start).

---

## Technical Details

### STM32 Implementation
On STM32, the library assumes a **7-bit** address is provided. It automatically converts it to the 8-bit format expected by `HAL_I2C_Mem_...` functions.


### ESP32 Implementation
For write operations, the library uses a local buffer of size `WRITE_BUFFER_SIZE`. 
> **Note:** Ensure `num_bytes + 1` does not exceed your defined buffer size (default is usually set in `i2c.h`).

---

## Usage Example

```c
#include "i2c.h"

// Example: Reading 2 bytes from register 0x00
uint8_t rx_data[2];
I2CStatus status = i2c_read(i2c_handle, DEV_ADDR, 0x00, rx_data, 2, 100);

if (status == I2C_OK) {
    // Data processed successfully
}
```

## Error Handling

The functions return a generic I2CStatus. If you receive I2C_ERR:

- Check pull-up resistors on SDA/SCL lines.
- Verify the dev_address is correct.
- Ensure the correct platform define (STM32 or ESP32) is active in your build configuration.
