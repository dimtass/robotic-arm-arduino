#include <Arduino.h>
#include <Servo.h>

// Define the number of servos
#define NUM_SERVOS 6
#define ADC_BUFFER_SIZE 10  // Number of samples to average
#define BUTTON_DEBOUNCE_TIME 250  // Debounce time in milliseconds
#define ADC_CENTER 512  // Center value for ADC readings
#define ADC_DEADZONE 20  // Deadzone around center value
#define LED_PATTERN_INTERVAL 200  // LED pattern interval in milliseconds

// Create an array of Servo objects
Servo servos[NUM_SERVOS];

// Define the pins for each servo
const int servoPins[NUM_SERVOS] = {2, 3, 4, 5, 6, 7};

// Define LED pin
const int LED_PIN = LED_BUILTIN;

// Define mode patterns
const bool mode1Pattern[8] = {1, 0, 0, 0, 0, 0, 0, 0};
const bool mode2Pattern[8] = {1, 0, 1, 0, 0, 0, 0, 0};
const bool mode3Pattern[8] = {1, 0, 1, 0, 1, 0, 0, 0};

// LED pattern variables
unsigned long lastPatternTime = 0;
int patternIndex = 0;

// Define joystick pins
// Joystick 1
const int JOYSTICK1_VRX = A0;  // VRx of first joystick
const int JOYSTICK1_VRY = A1;  // VRy of first joystick
const int JOYSTICK1_SW = 8;    // SW of first joystick

// Joystick 2
const int JOYSTICK2_VRX = A2;  // VRx of second joystick
const int JOYSTICK2_VRY = A3;  // VRy of second joystick
const int JOYSTICK2_SW = 9;    // SW of second joystick

// Operation mode and speed
int operationMode = 1;  // 1 = Direct control, 2 = Incremental control
int rotationSpeed = 1;  // Degrees per update for mode 2

// Threshold values for joystick movement
int vx1Threshold = 20;  // Default threshold for Joystick 1 X-axis
int vy1Threshold = 20;  // Default threshold for Joystick 1 Y-axis
int vx2Threshold = 20;  // Default threshold for Joystick 2 X-axis
int vy2Threshold = 20;  // Default threshold for Joystick 2 Y-axis

// Mode 2 thresholds
int vx1Mode2Threshold = 50; // Threshold for Joystick 1 X-axis in mode 2
int vy1Mode2Threshold = 50;// Threshold for Joystick 1 Y-axis in mode 2
int vx2Mode2Threshold = 50;// Threshold for Joystick 2 X-axis in mode 2
int vy2Mode2Threshold = 50;// Threshold for Joystick 2 Y-axis in mode 2

// Invert flags for each axis
bool vx1Invert = false;  // Invert flag for Joystick 1 X-axis
bool vy1Invert = true;  // Invert flag for Joystick 1 Y-axis
bool vx2Invert = false;  // Invert flag for Joystick 2 X-axis
bool vy2Invert = false;  // Invert flag for Joystick 2 Y-axis

// Current servo positions for mode 2
int currentServoPositions[NUM_SERVOS] = {90, 90, 90, 90, 90, 90};

// Last ADC values for mode 2
int lastMode2Joy1X = ADC_CENTER;
int lastMode2Joy1Y = ADC_CENTER;
int lastMode2Joy2X = ADC_CENTER;
int lastMode2Joy2Y = ADC_CENTER;

// Button state variables
bool buttonState = false;
bool lastButtonState = false;
unsigned long lastDebounceTime = 0;
bool servo5State = false;  // false = 180 degrees, true = 0 degrees

// Mode toggle button variables
bool modeButtonState = false;
bool lastModeButtonState = false;
unsigned long lastModeDebounceTime = 0;
bool modeButtonPressed = false;
unsigned long modeButtonPressTime = 0;

// ADC buffers for each axis
int joy1XBuffer[ADC_BUFFER_SIZE] = {0};
int joy1YBuffer[ADC_BUFFER_SIZE] = {0};
int joy2XBuffer[ADC_BUFFER_SIZE] = {0};
int joy2YBuffer[ADC_BUFFER_SIZE] = {0};
int bufferIndex = 0;

// Last known values for comparison
int lastJoy1X = ADC_CENTER;
int lastJoy1Y = ADC_CENTER;
int lastJoy2X = ADC_CENTER;
int lastJoy2Y = ADC_CENTER;

// Buffer for serial input
String inputString = "";
bool stringComplete = false;

// Function to calculate average of buffer
int calculateAverage(int* buffer) {
  long sum = 0;
  for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
    sum += buffer[i];
  }
  return sum / ADC_BUFFER_SIZE;
}

// Function to invert ADC value if needed
int applyInversion(int value, bool invert) {
  if (invert) {
    return 1023 - value;
  }
  return value;
}

// Function to print help information
void printHelp() {
  Serial.println("Available commands:");
  Serial.println("MODE=<1|2|3>  - Set operation mode (1=Direct, 2=Incremental, 3=LED pattern)");
  Serial.println("SPEED=<value>  - Set rotation speed for mode 2 (1-10)");
  Serial.println("VX1=<value>    - Set threshold for Joystick 1 X-axis");
  Serial.println("VY1=<value>    - Set threshold for Joystick 1 Y-axis");
  Serial.println("VX2=<value>    - Set threshold for Joystick 2 X-axis");
  Serial.println("VY2=<value>    - Set threshold for Joystick 2 Y-axis");
  Serial.println("VX1I=<0|1>     - Set invert for Joystick 1 X-axis");
  Serial.println("VY1I=<0|1>     - Set invert for Joystick 1 Y-axis");
  Serial.println("VX2I=<0|1>     - Set invert for Joystick 2 X-axis");
  Serial.println("VY2I=<0|1>     - Set invert for Joystick 2 Y-axis");
  Serial.println("PWM=<ch>,<val> - Direct servo control");
  Serial.println("HELP           - Show this help message");
}

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Servo Control System Ready");
  
  // Set LED pin as output
  pinMode(LED_PIN, OUTPUT);
  
  // Attach servos to their respective pins
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(90); // Initialize to center position
    currentServoPositions[i] = 90;
  }

  // Setup joystick buttons
  pinMode(JOYSTICK1_SW, INPUT_PULLUP);
  pinMode(JOYSTICK2_SW, INPUT_PULLUP);

  // Initialize ADC buffers with current readings
  int initialValue = analogRead(JOYSTICK1_VRX);
  for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
    joy1XBuffer[i] = initialValue;
    joy1YBuffer[i] = initialValue;
    joy2XBuffer[i] = initialValue;
    joy2YBuffer[i] = initialValue;
  }

  // Initialize servo 5 to 180 degrees (button not pressed)
  servos[5].write(180);
}

void processCommand(String command) {
  // Check for HELP command
  if (command == "HELP") {
    printHelp();
    return;
  }

  // Check for MODE command
  if (command.startsWith("MODE=")) {
    int newMode = command.substring(5).toInt();
    if (newMode >= 1 && newMode <= 3) {
      operationMode = newMode;
      // Reset all servos to 90 degrees
      for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].write(90);
        currentServoPositions[i] = 90;
      }
      Serial.print("Operation mode set to: ");
      Serial.println(operationMode);
      Serial.println("All servos reset to 90 degrees");
    } else {
      Serial.println("Invalid mode. Use 1, 2, or 3.");
    }
    return;
  }

  // Check for SPEED command
  if (command.startsWith("SPEED=")) {
    int newSpeed = command.substring(6).toInt();
    if (newSpeed >= 1 && newSpeed <= 10) {
      rotationSpeed = newSpeed;
      Serial.print("Rotation speed set to: ");
      Serial.println(rotationSpeed);
    } else {
      Serial.println("Invalid speed. Use 1-10.");
    }
    return;
  }

  // Check if command starts with "VX1I="
  if (command.startsWith("VX1I=")) {
    vx1Invert = command.substring(5).toInt() != 0;
    Serial.print("Joystick 1 X-axis invert set to: ");
    Serial.println(vx1Invert ? "true" : "false");
    return;
  }
  
  // Check if command starts with "VY1I="
  if (command.startsWith("VY1I=")) {
    vy1Invert = command.substring(5).toInt() != 0;
    Serial.print("Joystick 1 Y-axis invert set to: ");
    Serial.println(vy1Invert ? "true" : "false");
    return;
  }
  
  // Check if command starts with "VX2I="
  if (command.startsWith("VX2I=")) {
    vx2Invert = command.substring(5).toInt() != 0;
    Serial.print("Joystick 2 X-axis invert set to: ");
    Serial.println(vx2Invert ? "true" : "false");
    return;
  }
  
  // Check if command starts with "VY2I="
  if (command.startsWith("VY2I=")) {
    vy2Invert = command.substring(5).toInt() != 0;
    Serial.print("Joystick 2 Y-axis invert set to: ");
    Serial.println(vy2Invert ? "true" : "false");
    return;
  }

  // Check if command starts with "VX1="
  if (command.startsWith("VX1=")) {
    vx1Threshold = command.substring(4).toInt();
    vx1Mode2Threshold = vx1Threshold;  // Use same threshold for mode 2
    Serial.print("Joystick 1 X-axis threshold set to: ");
    Serial.println(vx1Threshold);
    return;
  }
  
  // Check if command starts with "VY1="
  if (command.startsWith("VY1=")) {
    vy1Threshold = command.substring(4).toInt();
    vy1Mode2Threshold = vy1Threshold;  // Use same threshold for mode 2
    Serial.print("Joystick 1 Y-axis threshold set to: ");
    Serial.println(vy1Threshold);
    return;
  }
  
  // Check if command starts with "VX2="
  if (command.startsWith("VX2=")) {
    vx2Threshold = command.substring(4).toInt();
    vx2Mode2Threshold = vx2Threshold;  // Use same threshold for mode 2
    Serial.print("Joystick 2 X-axis threshold set to: ");
    Serial.println(vx2Threshold);
    return;
  }
  
  // Check if command starts with "VY2="
  if (command.startsWith("VY2=")) {
    vy2Threshold = command.substring(4).toInt();
    vy2Mode2Threshold = vy2Threshold;  // Use same threshold for mode 2
    Serial.print("Joystick 2 Y-axis threshold set to: ");
    Serial.println(vy2Threshold);
    return;
  }

  // Check if command starts with "PWM="
  if (command.startsWith("PWM=")) {
    // Remove "PWM=" prefix
    command = command.substring(4);
    
    // Find the comma separator
    int commaIndex = command.indexOf(',');
    if (commaIndex != -1) {
      // Extract servo number and value
      int servoNum = command.substring(0, commaIndex).toInt();
      int servoValue = command.substring(commaIndex + 1).toInt();
      
      // Validate servo number and value
      if (servoNum >= 0 && servoNum < NUM_SERVOS && servoValue >= 0 && servoValue <= 180) {
        servos[servoNum].write(servoValue);
        currentServoPositions[servoNum] = servoValue;
        Serial.print("Servo ");
        Serial.print(servoNum);
        Serial.print(" set to ");
        Serial.println(servoValue);
      } else {
        Serial.println("Invalid servo number or value");
      }
    } else {
      Serial.println("Invalid command format");
    }
  } else {
    Serial.println("Unknown command");
  }
}

// Helper function to check if value has changed beyond threshold
bool hasChanged(int current, int last, int threshold) {
  return abs(current - last) >= threshold;
}

// Function to update servo position in mode 2
void updateServoPosition(int& currentPos, int adcValue, int& lastValue, int threshold, bool invert) {
  // Apply inversion if needed
  if (invert) {
    adcValue = 1023 - adcValue;
  }

  // Check if we're in the deadzone
  if (abs(adcValue - ADC_CENTER) <= ADC_DEADZONE) {
    return;  // No movement in deadzone
  }

  // Check if we've crossed the center with enough change
  if ((lastValue <= ADC_CENTER && adcValue > ADC_CENTER + threshold) ||
      (lastValue >= ADC_CENTER && adcValue < ADC_CENTER - threshold)) {
    lastValue = adcValue;
    return;  // Just update last value, no movement
  }

  // Update position based on which side of center we're on
  if (adcValue > ADC_CENTER + threshold) {
    currentPos = min(180, currentPos + rotationSpeed);
  } else if (adcValue < ADC_CENTER - threshold) {
    currentPos = max(0, currentPos - rotationSpeed);
  }

  lastValue = adcValue;
}

void loop() {
  // Update LED pattern
  if (millis() - lastPatternTime >= LED_PATTERN_INTERVAL) {
    lastPatternTime = millis();
    
    // Get the current pattern based on mode
    const bool* currentPattern;
    switch (operationMode) {
      case 1:
        currentPattern = mode1Pattern;
        break;
      case 2:
        currentPattern = mode2Pattern;
        break;
      case 3:
        currentPattern = mode3Pattern;
        break;
      default:
        currentPattern = mode1Pattern;
    }
    
    // Set LED state based on current pattern
    digitalWrite(LED_PIN, currentPattern[patternIndex]);
    
    // Update pattern index
    patternIndex = (patternIndex + 1) % 8;
  }

  // Read joystick values and update buffers
  joy1XBuffer[bufferIndex] = analogRead(JOYSTICK1_VRX);
  joy1YBuffer[bufferIndex] = analogRead(JOYSTICK1_VRY);
  joy2XBuffer[bufferIndex] = analogRead(JOYSTICK2_VRX);
  joy2YBuffer[bufferIndex] = analogRead(JOYSTICK2_VRY);
  
  // Calculate averages
  int avgJoy1X = calculateAverage(joy1XBuffer);
  int avgJoy1Y = calculateAverage(joy1YBuffer);
  int avgJoy2X = calculateAverage(joy2XBuffer);
  int avgJoy2Y = calculateAverage(joy2YBuffer);

  // Handle mode toggle button
  bool modeButtonReading = !digitalRead(JOYSTICK2_SW);  // Invert because we're using INPUT_PULLUP

  // Check if the mode button state has changed
  if (modeButtonReading != lastModeButtonState) {
    lastModeDebounceTime = millis();
  }

  // If the mode button state has been stable for the debounce period
  if ((millis() - lastModeDebounceTime) > BUTTON_DEBOUNCE_TIME) {
    // If the button state has changed
    if (modeButtonReading != modeButtonState) {
      modeButtonState = modeButtonReading;
      
      if (modeButtonState) {
        // Button pressed - start timing
        modeButtonPressTime = millis();
        modeButtonPressed = true;
      } else {
        // Button released - check if we should toggle mode
        if (modeButtonPressed && (millis() - modeButtonPressTime) > 250) {
          // Toggle between modes 1, 2, and 3
          operationMode = (operationMode % 3) + 1;
          Serial.print("Operation mode toggled to: ");
          Serial.println(operationMode);
          
          // Reset all servos to 90 degrees when mode changes
          for (int i = 0; i < NUM_SERVOS; i++) {
            servos[i].write(90);
            currentServoPositions[i] = 90;
          }
        }
        modeButtonPressed = false;
      }
    }
  }

  // Update the last mode button state
  lastModeButtonState = modeButtonReading;

  if (operationMode == 1) {
    // Mode 1: Direct control with thresholds
    if (hasChanged(avgJoy1X, lastJoy1X, vx1Threshold)) {
      // Apply inversion before mapping
      int mappedValue = applyInversion(avgJoy1X, vx1Invert);
      int servo0Angle = map(mappedValue, 0, 1023, 0, 180);
      servos[0].write(servo0Angle);
      currentServoPositions[0] = servo0Angle;
      lastJoy1X = avgJoy1X;
    }

    if (hasChanged(avgJoy1Y, lastJoy1Y, vy1Threshold)) {
      // Apply inversion before mapping
      int mappedValue = applyInversion(avgJoy1Y, vy1Invert);
      int servo1Angle = map(mappedValue, 0, 1023, 0, 180);
      servos[1].write(servo1Angle);
      currentServoPositions[1] = servo1Angle;
      lastJoy1Y = avgJoy1Y;
    }

    if (hasChanged(avgJoy2X, lastJoy2X, vx2Threshold)) {
      // Apply inversion before mapping
      int mappedValue = applyInversion(avgJoy2X, vx2Invert);
      int servo2Angle = map(mappedValue, 0, 1023, 0, 180);
      servos[2].write(servo2Angle);
      currentServoPositions[2] = servo2Angle;
      lastJoy2X = avgJoy2X;
    }

    if (hasChanged(avgJoy2Y, lastJoy2Y, vy2Threshold)) {
      // Apply inversion before mapping
      int mappedValue = applyInversion(avgJoy2Y, vy2Invert);
      int servo3Angle = map(mappedValue, 0, 1023, 0, 180);
      servos[3].write(servo3Angle);
      currentServoPositions[3] = servo3Angle;
      lastJoy2Y = avgJoy2Y;
    }
  } else if (operationMode == 2) {
    // Mode 2: Incremental control with deadzone and thresholds
    updateServoPosition(currentServoPositions[0], avgJoy1X, lastMode2Joy1X, vx1Mode2Threshold, vx1Invert);
    updateServoPosition(currentServoPositions[1], avgJoy1Y, lastMode2Joy1Y, vy1Mode2Threshold, vy1Invert);
    updateServoPosition(currentServoPositions[2], avgJoy2X, lastMode2Joy2X, vx2Mode2Threshold, vx2Invert);
    updateServoPosition(currentServoPositions[3], avgJoy2Y, lastMode2Joy2Y, vy2Mode2Threshold, vy2Invert);

    // Update servos with new positions
    servos[0].write(currentServoPositions[0]);
    servos[1].write(currentServoPositions[1]);
    servos[2].write(currentServoPositions[2]);
    servos[3].write(currentServoPositions[3]);
  }
  // Mode 3: Joysticks are disabled, only PWM commands work

  // Update buffer index
  bufferIndex = (bufferIndex + 1) % ADC_BUFFER_SIZE;

  // Read the button state for JOYSTICK1_SW
  bool reading = !digitalRead(JOYSTICK1_SW);  // Invert because we're using INPUT_PULLUP

  // Check if the button state has changed
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  // If the button state has been stable for the debounce period
  if ((millis() - lastDebounceTime) > BUTTON_DEBOUNCE_TIME) {
    // If the button state has changed
    if (reading != buttonState) {
      buttonState = reading;
      
      // Only toggle on button press (not release)
      if (buttonState) {
        // Toggle servo 5 position
        servo5State = !servo5State;
        if (servo5State) {
          servos[5].write(0);    // Toggle to 0 degrees
        } else {
          servos[5].write(180);  // Toggle to 180 degrees
        }
      }
    }
  }

  // Update the last button state
  lastButtonState = reading;

  // Process serial input when available
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }

  // Small delay to prevent overwhelming the servos
  delay(20);
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}