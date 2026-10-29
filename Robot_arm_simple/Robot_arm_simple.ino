#include <Servo.h>

// Pin definitions for potentiometers
const int POT1_PIN = A0;
const int POT2_PIN = A1;
const int POT3_PIN = A2;
const int POT4_PIN = A3;

// Pin definitions for servo motors
const int SERVO1_PIN = 3;
const int SERVO2_PIN = 5;
const int SERVO3_PIN = 6;
const int SERVO4_PIN = 9;

// Pin definition for reset button
const int RESET_BUTTON_PIN = 2;

// Individual Servo objects
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

void setup() {
  // Configure reset button pin with internal pull-up resistor
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

  // Attach servo objects to their assigned PWM pins
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
}

void loop() {
  // Check if reset button is pressed (active LOW)
  if (digitalRead(RESET_BUTTON_PIN) == LOW) {
    // Force all servos to 90 degrees default position
    servo1.write(90);
    servo2.write(90);
    servo3.write(90);
    servo4.write(90);
  } else {
    // Read potentiometer 1 and map value to degrees (0-180)
    int pot1Value = analogRead(POT1_PIN);
    int angle1 = map(pot1Value, 0, 1023, 0, 180);
    servo1.write(angle1);

    // Read potentiometer 2
    int pot2Value = analogRead(POT2_PIN);
    int angle2 = map(pot2Value, 0, 1023, 0, 180);
    servo2.write(angle2);

    // Read potentiometer 3
    int pot3Value = analogRead(POT3_PIN);
    int angle3 = map(pot3Value, 0, 1023, 0, 180);
    servo3.write(angle3);

    // Read potentiometer 4
    int pot4Value = analogRead(POT4_PIN);
    int angle4 = map(pot4Value, 0, 1023, 0, 180);
    servo4.write(angle4);
  }

  // Small delay to allow servos time to reach target positions
  delay(15);
}