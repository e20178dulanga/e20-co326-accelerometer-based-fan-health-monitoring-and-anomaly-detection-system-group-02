# ML Model — Vibration Sensor Inference Pipeline

This directory contains the **machine learning inference pipeline** for the Accelerometer-Based Fan Health Monitoring system. The ML model classifies vibration patterns from a 6-axis IMU (MPU9250) to detect fan anomalies in real-time on an ESP32 microcontroller.

## Architecture

```
ml-model/
├── src/
│   ├── edge-impulse-sdk/        # Edge Impulse inference SDK (TFLite Micro)
│   ├── model-parameters/        # Model metadata and configuration
│   ├── tflite-model/            # Trained TFLite model (quantized INT8)
│   └── vibration_sensor_inferencing.h  # Main inference header
├── examples/                    # Board-specific inference examples
│   └── esp32/                   # ESP32 fusion, camera, microphone examples
├── sketch_apr25a/               # Main inference sketch for the project
│   └── sketch_apr25a.ino        # Full integration: MPU9250 + OLED + ML
├── sensor_tests/                # Sensor validation firmware
│   ├── platformio.ini           # PlatformIO config for ESP32-S3
│   └── src/main.cpp             # ADXL345 accelerometer test
├── library.properties           # Arduino library metadata
└── README.md                    # This file
```

## ML Model Details

| Property | Value |
|---|---|
| **Platform** | Edge Impulse Studio |
| **Project ID** | 974007 |
| **Model Type** | TFLite (quantized INT8) |
| **Inference Engine** | TFLite Micro (compiled) |
| **Input** | 6-axis sensor fusion (ax, ay, az, gx, gy, gz) |
| **Sampling Rate** | 100 Hz |
| **Window Size** | 200 samples (2 seconds) |
| **DSP Block** | Spectral Analysis (FFT, length=128) |
| **Output Classes** | 2 (healthy vs anomaly) |
| **Quantization** | INT8 (optimized for ESP32) |

## How It Works

1. **Data Collection**: MPU9250 reads accelerometer (±4g) and gyroscope (±500°/s) data at 100Hz
2. **Calibration**: Sensor offsets are computed from 500 static samples at startup
3. **Feature Extraction**: 6-axis data fills a sliding window buffer (200 samples × 6 axes = 1200 features)
4. **DSP Processing**: Edge Impulse SDK applies spectral analysis (FFT) to extract frequency-domain features
5. **Classification**: Quantized neural network classifies vibration pattern
6. **Output**: Health status and confidence displayed on OLED + Serial monitor

## Hardware Requirements

- ESP32-S3 DevKitC-1
- MPU9250 9-axis IMU (I2C: SDA=GPIO8, SCL=GPIO9)
- SSD1306 OLED Display (128×32, I2C address 0x3C)
- Push button (GPIO2) for fan ON/OFF control
- Motor relay output (GPIO4)

## Building

### Arduino IDE
1. Copy the entire `ml-model/` folder to your Arduino libraries directory
2. Open `sketch_apr25a/sketch_apr25a.ino`
3. Select ESP32-S3 board and upload

### PlatformIO (Sensor Tests)
```bash
cd sensor_tests/
pio run --target upload
pio device monitor
```

## Author

**Tharaka Dilshan** — ML model training, Edge Impulse integration, and inference pipeline development.
