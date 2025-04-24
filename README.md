# Robotic Arm Control System

This project implements a control system for a robotic arm using Arduino, servos, and joysticks. The system provides multiple control modes and configuration options through serial commands.

The robotic arm that was used is a [3D print from Trax](https://makerworld.com/en/models/528885-robotic-arm#profileId-445995) that I've found in makerworld and printed on my Bambu Lab P1S.

## Features

- Multiple control modes:
  - Mode 1: Direct control with thresholds
  - Mode 2: Incremental control with deadzone
  - Mode 3: Manual control (joysticks disabled, only PWM commands)
- Visual mode indication through LED patterns
- Configurable thresholds and inversion for each joystick axis
- Adjustable rotation speed for incremental control
- Direct servo control through PWM commands
- Button-controlled mode switching
- Serial command interface for configuration

## Hardware Requirements

- Arduino board
- 6 servos
- 2 analog joysticks
- Connecting wires
- Power supply for servos

## Pin Configuration

### Servos
- Servo 0: Pin 2
- Servo 1: Pin 3
- Servo 2: Pin 4
- Servo 3: Pin 5
- Servo 4: Pin 6
- Servo 5: Pin 7

### Joysticks
- Joystick 1:
  - VRx: A0
  - VRy: A1
  - SW: 8
- Joystick 2:
  - VRx: A2
  - VRy: A3
  - SW: 9

### LED
- Built-in LED: Used for mode indication

## Control Modes

### Mode 1: Direct Control
- Direct mapping of joystick position to servo angle
- Configurable thresholds to prevent small movements
- Inversion options for each axis

### Mode 2: Incremental Control
- Servos move in small increments based on joystick position
- Deadzone around center position
- Adjustable rotation speed
- Configurable thresholds for movement

### Mode 3: Manual Control
- Joysticks are disabled
- Servos can only be controlled through PWM commands
- Useful for precise positioning or testing

## LED Patterns

The built-in LED indicates the current mode through specific patterns (each digit represents 200ms):
- Mode 1: `10000000` (One flash)
- Mode 2: `10100000` (Two flashes)
- Mode 3: `10101000` (Three flashes)

## Serial Commands

All commands are case-sensitive and must be sent with a newline character.

### Mode Control
- `MODE=<1|2|3>` - Set operation mode
  - 1: Direct control
  - 2: Incremental control
  - 3: Manual control (PWM only)

### Speed Control
- `SPEED=<value>` - Set rotation speed for mode 2 (1-10)

### Threshold Configuration
- `VX1=<value>` - Set threshold for Joystick 1 X-axis
- `VY1=<value>` - Set threshold for Joystick 1 Y-axis
- `VX2=<value>` - Set threshold for Joystick 2 X-axis
- `VY2=<value>` - Set threshold for Joystick 2 Y-axis

### Inversion Configuration
- `VX1I=<0|1>` - Set invert for Joystick 1 X-axis
- `VY1I=<0|1>` - Set invert for Joystick 1 Y-axis
- `VX2I=<0|1>` - Set invert for Joystick 2 X-axis
- `VY2I=<0|1>` - Set invert for Joystick 2 Y-axis

### Direct Servo Control
- `PWM=<ch>,<val>` - Direct servo control
  - `<ch>`: Servo number (0-5)
  - `<val>`: Angle value (0-180)

### Help
- `HELP` - Show available commands

## Button Controls

- **JOYSTICK1_SW**: Toggles Servo 5 between 0° and 180°
- **JOYSTICK2_SW**: Hold for >250ms to cycle through modes 1, 2, and 3

## Default Settings

- Operation Mode: 1 (Direct control)
- Rotation Speed: 1
- Thresholds: 20 for all axes
- Inversion: Y1 inverted, others normal
- Servo Initial Position: 90° (center)

## Installation

1. Connect the hardware according to the pin configuration
2. Upload the code to your Arduino
3. Open the Serial Monitor at 9600 baud
4. Send `HELP` to see available commands

## Usage

1. Power on the system
2. Observe the LED pattern to confirm current mode
3. Use joysticks to control servos (in modes 1 and 2)
4. Use serial commands to configure the system
5. Press JOYSTICK2_SW to switch between modes
6. Press JOYSTICK1_SW to toggle Servo 5

## Notes

- All servos reset to 90° when changing modes
- Mode changes can be done through serial commands or the JOYSTICK2_SW button
- The system includes debouncing for button inputs
- ADC readings are averaged over 10 samples to reduce noise
