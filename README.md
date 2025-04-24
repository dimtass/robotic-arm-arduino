# Robotic Arm Control System

This project implements a control system for a 6-servo robotic arm using an Arduino UNO. The system supports both direct and incremental control modes using two analog joysticks.

## Hardware Requirements

- Arduino UNO
- 6x Servo Motors
- 2x Analog Joysticks
- Robotic Arm Model: [MakerWorld Robotic Arm](https://makerworld.com/en/models/528885-robotic-arm#profileId-445995)

## Features

### Control Modes

1. **Mode 1 (Direct Control)**
   - Direct mapping of joystick position to servo angle
   - Servos return to center when joysticks are released
   - Configurable thresholds to prevent jitter

2. **Mode 2 (Incremental Control)**
   - Servos maintain their position when joysticks are released
   - Incremental movement based on joystick position
   - Configurable movement speed
   - Deadzone around center position
   - Threshold-based movement detection

### Additional Features

- Button toggle control for one servo (0° ↔ 180°)
- Configurable inversion for each joystick axis
- Moving average filter for ADC readings
- Serial command interface for configuration
- Automatic servo reset to 90° when changing modes

## Serial Commands

The system accepts the following commands via serial interface:

- `MODE=<1|2>` - Set operation mode (1=Direct, 2=Incremental)
- `SPEED=<value>` - Set rotation speed for mode 2 (1-10)
- `VX1=<value>` - Set threshold for Joystick 1 X-axis
- `VY1=<value>` - Set threshold for Joystick 1 Y-axis
- `VX2=<value>` - Set threshold for Joystick 2 X-axis
- `VY2=<value>` - Set threshold for Joystick 2 Y-axis
- `VX1I=<0|1>` - Set invert for Joystick 1 X-axis
- `VY1I=<0|1>` - Set invert for Joystick 1 Y-axis
- `VX2I=<0|1>` - Set invert for Joystick 2 X-axis
- `VY2I=<0|1>` - Set invert for Joystick 2 Y-axis
- `PWM=<ch>,<val>` - Direct servo control
- `HELP` - Show help message

## Pin Configuration

### Servos
- Servo 0: Pin 2
- Servo 1: Pin 3
- Servo 2: Pin 4
- Servo 3: Pin 5
- Servo 4: Pin 6
- Servo 5: Pin 7

### Joystick 1
- VRx: A0
- VRy: A1
- SW: Pin 8

### Joystick 2
- VRx: A2
- VRy: A3
- SW: Pin 9

## Default Settings

- Operation Mode: 1 (Direct Control)
- Rotation Speed: 1
- Thresholds: 20 (Mode 1), 50 (Mode 2)
- Deadzone: 20
- Default Inversion: Y1 inverted (true), others false

## Building and Uploading

1. Install PlatformIO
2. Clone this repository
3. Open the project in PlatformIO
4. Build and upload to your Arduino UNO

## Dependencies

- Arduino Servo library

## License

This project is open source and available under the MIT License.
