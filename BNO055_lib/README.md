### BNO055 IMU Library
## Overview
This library provides an interface for interacting with the BNO055 Intelligent Multi-Sensor (IMU) via I2C communication. It is designed for use with microcontrollers and supports a wide range of features offered by the BNO055, including reading accelerometer, magnetometer, gyroscope, Euler angles, quaternion, linear acceleration, and gravity vector data.

## Features
1. Support for reading various sensor data:
    - Acceleration (X, Y, Z)
    - Magnetometer (X, Y, Z)
    - Gyroscope (X, Y, Z)
    - Euler angles (Heading, Roll, Pitch)
    - Quaternion (W, X, Y, Z)
    - Linear acceleration (X, Y, Z)
    - Gravity vector (X, Y, Z)
1. Configuration options for power modes, operation modes, axis remap, and unit selection
1. Functions for calibration status checks
1. Error handling
## Requirements
* A microcontroller with I2C support
## Installation
Download the library files BNO055.c and BNO055.h.
Include these files in your project directory.
Include BNO055.h in your source code.
## Usage
* Initialize the IMU with desired configurations using bno055_init().
* Read sensor data using the respective functions (e.g., bno055_read_acc_xyz(), bno055_read_mag_xyz(), etc.).
* Use the calibration status functions to check the calibration status of different sensors.
# Example

c
```
#include "BNO055.h"

int main() {
    // Initialize BNO055 with default settings
    bno055_init(&default_bno055_config, &default_bno055_verification);

    // Read accelerometer data
    bno055_acc_t acc_data;
    bno055_read_acc_xyz(&acc_data);
    printf("Accelerometer data: X = %f, Y = %f, Z = %f\n", acc_data.x, acc_data.y, acc_data.z);

    // Add additional sensor readings and logic here
}
```

[Ines Garcia]
[https://github.com/Ineso1]
# Repo Link
[https://github.com/Ineso1/SEM_Embedded_Telemetry]

