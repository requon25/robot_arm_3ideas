#include <Servo.h>
#include <EEPROM.h>

// ---------- Potentiometers ----------
const int POT1_PIN = A0;
const int POT2_PIN = A1;
const int POT3_PIN = A2;
const int POT4_PIN = A3;

// ---------- Servos ----------
const int SERVO1_PIN = 3;
const int SERVO2_PIN = 5;
const int SERVO3_PIN = 6;
const int SERVO4_PIN = 9;

// ---------- Buttons ----------
const int HOME_BUTTON_PIN     = 2; // Forces 90 degrees while it is held down
const int SAVE_BUTTON_PIN     = 4; // Adds the current position to the saved sequence in EEPROM
const int RESET_EE_BUTTON_PIN = 7; // Erases the data saved in EEPROM (the whole sequence)

// ---------- HC-SR04 ultrasonic sensor ----------
const int TRIG_PIN = 8;
const int ECHO_PIN = 12;

// ---------- EEPROM addresses ----------
const int EE_FLAG_ADDR    = 0;     // "Valid data" flag
const byte EE_VALID_FLAG  = 0xAA;
const int EE_COUNT_ADDR   = 1;     // Number of positions currently saved in the sequence
const int EE_DATA_ADDR    = 2;     // Start of the position data (4 bytes per position, one per servo)

// ---------- Sequence settings ----------
const int MAX_POSITIONS = 20;      // Maximum number of positions in the sequence (uses 2 + 20*4 = 82 bytes of EEPROM)

Servo servo1, servo2, servo3, servo4;

int savedSequence[MAX_POSITIONS][4];     // Sequence of "taught" positions (EEPROM), stored in the order they were saved
int savedCount       = 0;                // How many positions are currently stored in the sequence
int currentAngles[4] = {90, 90, 90, 90}; // Last position applied to the servos

bool autoModeActive = false;
int autoStepIndex = 0;                   // Index of the sequence position currently being played
unsigned long autoStepStartTime = 0;     // Time when the current step started
const unsigned long AUTO_HOLD_TIME = 2000; // ms that each automatic position is held before moving to the next one

bool objectWasNear = false;
const float TRIGGER_DISTANCE_CM = 10.0; // distance (cm) that triggers the automation

unsigned long lastSensorCheck = 0;
const unsigned long SENSOR_INTERVAL = 100; // ms between HC-SR04 readings

bool saveButtonLastState  = HIGH;
bool resetButtonLastState = HIGH;
unsigned long lastSaveDebounce  = 0;
unsigned long lastResetDebounce = 0;
const unsigned long DEBOUNCE_DELAY = 250; // ms

void setup() {
  pinMode(HOME_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SAVE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RESET_EE_BUTTON_PIN, INPUT_PULLUP);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  pinMode(LED_BUILTIN, OUTPUT);

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);

  Serial.begin(9600);

  loadPositionsFromEEPROM();

  // On power-up, goes to the last saved position (or to 90 degrees if nothing is saved)
  if (savedCount > 0) {
    applyAngles(savedSequence[savedCount - 1]);
  } else {
    applyAngles(currentAngles);
  }
}

void loop() {
  // 1) HOME button: forces 90 degrees while it is held down
  if (digitalRead(HOME_BUTTON_PIN) == LOW) {
    autoModeActive = false; // Cancels any automatic sequence that is running
    int homeAngles[4] = {90, 90, 90, 90};
    applyAngles(homeAngles);
    delay(15);
    return;
  }

  // 2) SAVE button (falling edge + debounce)
  bool saveState = digitalRead(SAVE_BUTTON_PIN);
  if (saveState == LOW && saveButtonLastState == HIGH &&
      millis() - lastSaveDebounce > DEBOUNCE_DELAY) {
    lastSaveDebounce = millis();
    // Positions can only be added in manual mode (not while the sequence is playing)
    if (!autoModeActive) {
      if (savePositionToEEPROM(currentAngles)) {
        blinkFeedback(2);  // Position added to the sequence
      } else {
        blinkFeedback(6);  // Sequence is full, position was NOT saved
      }
    }
  }
  saveButtonLastState = saveState;

  // 3) EEPROM RESET button (falling edge + debounce)
  bool resetState = digitalRead(RESET_EE_BUTTON_PIN);
  if (resetState == LOW && resetButtonLastState == HIGH &&
      millis() - lastResetDebounce > DEBOUNCE_DELAY) {
    lastResetDebounce = millis();
    resetEEPROM();
    blinkFeedback(4);
  }
  resetButtonLastState = resetState;

  // 4) HC-SR04 sensor: checks the distance every SENSOR_INTERVAL ms
  if (millis() - lastSensorCheck > SENSOR_INTERVAL) {
    lastSensorCheck = millis();
    float distance = readDistanceCM();
    bool isNear = (distance > 0 && distance < TRIGGER_DISTANCE_CM);

    // Only triggers when the object has "just" appeared (edge),
    // so the sequence is not restarted all the time if the object stays close.
    // It also needs at least one saved position to play.
    if (isNear && !objectWasNear && !autoModeActive && savedCount > 0) {
      autoModeActive = true;
      autoStepIndex = 0;                 // Start from the first saved position (FIFO order)
      autoStepStartTime = millis();
      applyAngles(savedSequence[autoStepIndex]);
    }
    objectWasNear = isNear;
  }

  // 5) Automatic mode active: hold each position, play them in order, then go back to manual
  if (autoModeActive) {
    if (millis() - autoStepStartTime > AUTO_HOLD_TIME) {
      autoStepIndex++;                   // Move on to the next saved position
      if (autoStepIndex >= savedCount) {
        autoModeActive = false;          // Last position finished: sequence done, back to manual
      } else {
        autoStepStartTime = millis();
        applyAngles(savedSequence[autoStepIndex]);
      }
    }
  } else {
    // 6) Manual mode: the potentiometers control the servos
    int angles[4];
    angles[0] = map(analogRead(POT1_PIN), 0, 1023, 0, 180);
    angles[1] = map(analogRead(POT2_PIN), 0, 1023, 0, 180);
    angles[2] = map(analogRead(POT3_PIN), 0, 1023, 0, 180);
    angles[3] = map(analogRead(POT4_PIN), 0, 1023, 0, 180);
    applyAngles(angles);
  }

  delay(15);
}

// ==================== Helper functions ====================

void applyAngles(int angles[4]) {
  servo1.write(angles[0]);
  servo2.write(angles[1]);
  servo3.write(angles[2]);
  servo4.write(angles[3]);
  currentAngles[0] = angles[0];
  currentAngles[1] = angles[1];
  currentAngles[2] = angles[2];
  currentAngles[3] = angles[3];
}

float readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30 ms timeout (~5 m max)
  if (duration == 0) return -1; // no echo / out of range

  return duration * 0.0343 / 2.0; // cm
}

// Appends a position at the end of the sequence (EEPROM + RAM).
// Returns true if it was saved, false if the sequence is full.
bool savePositionToEEPROM(int angles[4]) {
  if (savedCount >= MAX_POSITIONS) {
    Serial.println("Sequence is full. Position NOT saved.");
    return false;
  }

  int baseAddr = EE_DATA_ADDR + savedCount * 4; // Each position takes 4 bytes
  for (int i = 0; i < 4; i++) {
    EEPROM.update(baseAddr + i, angles[i]);
    savedSequence[savedCount][i] = angles[i];
  }
  savedCount++;

  EEPROM.update(EE_FLAG_ADDR, EE_VALID_FLAG);
  EEPROM.update(EE_COUNT_ADDR, savedCount);

  Serial.print("Position ");
  Serial.print(savedCount);
  Serial.print(" of ");
  Serial.print(MAX_POSITIONS);
  Serial.println(" saved in EEPROM.");
  return true;
}

void loadPositionsFromEEPROM() {
  savedCount = 0;
  if (EEPROM.read(EE_FLAG_ADDR) == EE_VALID_FLAG) {
    int count = EEPROM.read(EE_COUNT_ADDR);
    if (count > MAX_POSITIONS) count = MAX_POSITIONS; // Safety check in case the stored data is corrupted

    for (int p = 0; p < count; p++) {
      int baseAddr = EE_DATA_ADDR + p * 4;
      for (int i = 0; i < 4; i++) {
        savedSequence[p][i] = EEPROM.read(baseAddr + i);
      }
    }
    savedCount = count;
  }

  Serial.print("Positions loaded from EEPROM: ");
  Serial.println(savedCount);
}

void resetEEPROM() {
  EEPROM.update(EE_FLAG_ADDR, 0xFF);
  EEPROM.update(EE_COUNT_ADDR, 0);
  savedCount = 0;
  autoModeActive = false; // Stop the sequence if it was playing, since it no longer exists
  Serial.println("EEPROM reset. Saved sequence erased.");
}

void blinkFeedback(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(80);
    digitalWrite(LED_BUILTIN, LOW);
    delay(80);
  }
}
