# STM32 IMU-Based Servo Control & BLE System

This project implements a real-time orientation tracking and servo control system using the **B-L4S5I-IOT01A** (STM32L4+ Discovery kit). It leverages onboard sensors to calculate pitch and yaw, which are then used to drive two servos via PWM. The system also supports BLE data reporting.

## 🚀 Features

- **IMU Orientation Tracking**: Reads data from onboard accelerometer (LSM6DSL) and magnetometer (LIS3MDL).
- **Signal Processing**: Includes calibration offsets and a moving average filter for smoothed sensor data.
- **Servo Control**: Real-time mapping of Pitch and Yaw angles to PWM signals (50Hz, 1ms-2ms pulse).
- **BLE Integration**: Implements a BLE server using the BlueNRG-MS stack to share motion data.
- **Serial Diagnostics**: Detailed UART debug output at 115200 baud.

## 🛠 Hardware Setup

### Components
1. **STM32 Board**: B-L4S5I-IOT01A
2. **Servos**: 2x Standard PWM Servos (e.g., SG90)
3. **Power**: USB or external 5V for servos.

### Pinout
| Component | Pin | Timer/Channel | Arduino Header |
| :--- | :--- | :--- | :--- |
| **Pitch Servo** | PA15 | TIM2_CH1 | **D9** |
| **Yaw Servo** | PA2 | TIM2_CH3 | **D10** |
| **Debug UART** | PB6/PB7 | USART1 | ST-LINK VCP |

## 📂 Project Structure

- `Core/Src/main.c`: Main application loop, pitch/yaw calculation, and PWM updates.
- `BlueNRG_MS/App/imu.c`: IMU driver abstraction, calibration, and filtering logic.
- `BlueNRG_MS/App/app_bluenrg_ms.c`: BLE stack initialization and application processes.
- `Drivers/`: STM32L4xx HAL and Board Support Package (BSP) drivers.

## ⚙️ Configuration

### IMU Calibration
At startup, the system collects 100 samples to compute gyroscope and accelerometer offsets. Ensure the board is stationary during the first few seconds after reset.

### PWM Parameters
- **Frequency**: 50Hz (20ms period)
- **Pulse Width**: 1000µs to 2000µs
- **Mapping**: 
  - Pitch: -90° to +90° → 1000µs to 2000µs
  - Yaw: -180° to +180° → 1000µs to 2000µs

## ⏩ Getting Started

1. Open the project in **STM32CubeIDE**.
2. Connect your **B-L4S5I-IOT01A** board via USB.
3. Build the project (Ctrl+B).
4. Run/Debug (F11) to flash the firmware.
5. Open a Serial Terminal (115200, 8N1) to view diagnostic output.

---
*Created by [jocer](https://github.com/jocer)*
