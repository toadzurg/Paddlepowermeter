# Paddle Power Meter

Modular ESP32 canoe paddle power meter with dynamic lever arm, BLE integration, and Kalman filtering.

## Structure

```
PaddlePowerMeter_Full/
└── src/
    ├── PaddlePowerMeter.ino
    ├── Scale.{h,cpp}
    ├── IMU.{h,cpp}
    ├── Kalman.{h,cpp}
    ├── PowerModel.{h,cpp}
    ├── Battery.{h,cpp}
    └── BLEManager.{h,cpp}
.gitignore
README.md
```

## Features

- NAU7802 load cell (force)
- Dual BMI270 IMUs for dynamic lever arm
- Madgwick AHRS for orientation
- Kalman filter for lever-arm smoothing
- True energy integration per stroke → Watts
- BLE Cycling Power & Battery services
