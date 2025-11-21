# Crueller Cruller - An ESP32 Meltybrain Combat Robot

Single-wheel combat robot controller with directional translation control while spinning.

## What is Meltybrain?

Meltybrain is a control technique for single-wheel robots that allows directional translation while the robot is spinning. By modulating the motor throttle in sync with the robot's rotation phase, the robot can translate in any direction while maintaining its spin.

## Hardware

- **ESP32** microcontroller
- **Google Stadia controller** (Bluetooth Classic via Bluepad32)
- **Two NeoPixels** on pin 23 (status indicators)
- **AM32 ESC** via DShot600 on pin 19 (motor control)
- **LIS331HH accelerometer** on I2C (SDA=21, SCL=22) for rotation tracking

## How It Works

### Rotation Detection

The accelerometer measures horizontal acceleration as the robot spins. Using the formula `a = ω²r`, the controller calculates RPM from the measured acceleration and the calibrated radius of the accelerometer from the center of rotation.

### Translation Control

When the robot spins above 600 RPM, meltybrain mode activates. The motor throttle is modulated using a parabolic sine wave pattern synchronized to the robot's rotation:

- The modulation creates a smooth throttle variation over each half-rotation
- The phase of the modulation determines the translation direction
- By applying more throttle when the robot is oriented in the target direction, net force is created in that direction

### POV LED

A Persistence of Vision (POV) LED blinks at a fixed position (10% of the rotation) to create a visual indicator of the robot's heading. The LED position shifts to show the translation direction, like a headlight on a tank drive robot.

## Control

### Basic Operation

- **Right trigger**: Throttle control (0-100%)
- **Left stick**: Translation direction and strength
- **D-pad**: Cardinal direction translation (full strength)

### Config Mode

Hold **X button** for 2 seconds to enter/exit config mode:
- Throttle limited to 50% for safer tuning
- **Right stick X**: Adjust accelerometer radius (left = decrease, right = increase)
- **Right stick Y**: Adjust modulation strength (up = increase, down = decrease)
- Settings saved to EEPROM when exiting config mode

### Calibration

- **Y button** (hold 1 second when stopped): Recalibrate accelerometer offsets
- Radius and modulation strength are saved to EEPROM and persist across reboots

## LED Status Indicators

- **Red (solid)**: Controller disconnected
- **Red (blinking)**: Startup sequence (3 seconds)
- **Blue (solid)**: Connected and ready, or spinning up
- **Green (POV)**: Meltybrain mode active, blinks at heading position
- **Purple (POV)**: Config mode active
- **Yellow (flash)**: Settings saved confirmation

## Features

- **DShot600 protocol** for fast, precise ESC control
- **Real-time radius adjustment** for fine-tuning POV LED stability
- **Adaptive RPM smoothing** for handling rapid speed changes during battle
- **EEPROM persistence** for calibration values
- **Resilient to RPM changes** - adapts quickly to speed variations

## Setup

1. Install required libraries:
   - Bluepad32: https://github.com/ricardoquesada/bluepad32-arduino
   - Adafruit LIS331HH
   - Adafruit NeoPixel
   - Adafruit Sensor

2. Upload `crueller_cruller/crueller_cruller.ino` to your ESP32

3. Pair your Stadia controller:
   - Put controller in pairing mode (hold Stadia button for 3 seconds)
   - Controller will connect automatically

4. Calibrate radius:
   - Enter config mode (hold X for 2 seconds)
   - Spin up the robot
   - Adjust right stick X until the POV LED stabilizes at a single point
   - Exit config mode to save

## Technical Details

### DShot Protocol

The controller uses DShot600 (600 kbit/s) for ESC communication. DShot provides faster response and more precise control than PWM. The ESC is armed by sending 0 commands for 2 seconds at startup.

### RPM Calculation

RPM is calculated from horizontal acceleration using:
```
RPM = 60 × √(a / r) / (2π)
```
Where `a` is acceleration in m/s² and `r` is the accelerometer radius in meters.

### Throttle Modulation

The modulation uses a parabolic sine wave approximation:
- Creates a smooth throttle variation from 0 to maximum and back to 0 over half a rotation
- Applied in phase with the target translation direction
- Maximum modulation is 50% of base throttle

### Rotation Tracking

The controller tracks rotation using time-based phase tracking:
- Continuously monitors rotation interval
- Adapts to RPM changes during battle
- Resets phase tracking on large RPM changes (>20%) to prevent drift

## License

This project is provided as-is for educational and hobby use.
