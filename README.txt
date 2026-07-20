# ESP32 Thrust Vector Control (TVC) Flight Computer

An ESP32-powered flight control system for TVC rocket stabilization and telemetry logging.

## 🚀 Features
- **Sensor Fusion:** MPU6050 IMU for tilt tracking and BMP280 barometric altimeter for apogee/landing detection.
- **Active Guidance:** Real-time PID control loop operating at ~50Hz driving a TVC gimbal servo.
- **State Machine:** 4-stage flight profile (`PAD_IDLE`, `POWERED_FLIGHT`, `COAST_TO_APOGEE`, `RECOVERY_LANDED`).
- **Data Logging:** Stores flight telemetry into internal Flash memory (`LittleFS`) as a CSV.

## 📊 Telemetry Analysis
Flight telemetry is recorded at 50Hz and exported via Serial command (`dump`). Below is the data visualization from a successful benchtop flight test showing TVC correction, state transitions, and acceleration profile:

![Flight Telemetry](tvc_telemetry_plots.png)

## 🛠️ Hardware Requirements
- ESP32 Development Board
- MPU6050 Accelerometer / Gyroscope (I2C)
- BMP280 Barometric Pressure Sensor (I2C)
- Servo Motor (Gimbal Actuator on GPIO 13/18)