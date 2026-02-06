
#include <SoftwareSerial.h>

// ========== CONFIGURATION ==========
#define BAUDRATE 9600
#define SAMPLE_RATE 500
#define INPUT_PIN A0
#define BUFFER_SIZE 128

// Bluetooth pins
SoftwareSerial BTSerial(10, 11); // RX, TX

// Motor pins
const int IN1 = 6;
const int IN2 = 5;
const int IN3 = 9;
const int IN4 = 3;

// Ultrasonic pins
const int TRIG_PIN = 7;
const int ECHO_PIN = 8;
const int OBSTACLE_DISTANCE = 10; // cm

// EMG settings
const int EMG_THRESHOLD = 140;
const unsigned long EMG_DEBOUNCE_DELAY = 300;

// ========== LATENCY VARIABLES ==========
unsigned long commandReceivedTime = 0;
unsigned long motorActionTime = 0;

// ========== GLOBAL VARIABLES ==========
enum ControlMode {
  MODE_SELECTION,
  VOICE_CONTROL,
  EMG_CONTROL
};

ControlMode currentMode = MODE_SELECTION;

int circular_buffer[BUFFER_SIZE];
int data_index = 0;
long sum = 0;

unsigned long lastEMGActionTime = 0;
bool isMoving = false;
bool obstacleLock = false;

// ========== SETUP ==========
void setup() {
  Serial.begin(BAUDRATE);
  BTSerial.begin(BAUDRATE);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Robot_Stop();
  BTSerial.setTimeout(200);
  delay(100);

  for (int i = 0; i < BUFFER_SIZE; i++) {
    circular_buffer[i] = 0;
  }

  Serial.println("\n=== SYSTEM STARTED ===");
  promptModeSelection();
}

// ========== MAIN LOOP ==========
void loop() {
  safetyCheck();   // Always check obstacle

  if (obstacleLock) return; // Override all controls

  switch (currentMode) {
    case MODE_SELECTION:
      handleModeSelection();
      break;

    case VOICE_CONTROL:
      handleVoiceControl();
      break;

    case EMG_CONTROL:
      handleEMGControl();
      break;
  }
}

// ========== OBSTACLE DETECTION ==========
bool isObstacleDetected() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 20000);

  if (duration == 0) return false;

  int distance = duration * 0.034 / 2;

  if (distance <= OBSTACLE_DISTANCE) {
    return true;
  }
  return false;
}

void safetyCheck() {
  if (isObstacleDetected()) {
    if (!obstacleLock) {
      Robot_Stop();
      obstacleLock = true;
      isMoving = false;
      Serial.println("SAFETY STOP: Obstacle detected");
    }
  } else {
    obstacleLock = false;
  }
}

// ========== MODE SELECTION ==========
void promptModeSelection() {
  BTSerial.println("========================================");
  BTSerial.println("SMART WHEELCHAIR CONTROL SYSTEM");
  BTSerial.println("========================================");
  BTSerial.println("Please select control mode:");
  BTSerial.println("1. Send 'voice' or 'v' for Voice Control");
  BTSerial.println("2. Send 'emg' or 'e' for EMG Control");
  BTSerial.println("3. Send 'menu' or 'm' to return to this menu");
  BTSerial.println("========================================");
  
  Serial.println("Mode selection menu sent");
}

void handleModeSelection() {
  if (BTSerial.available() > 0 || Serial.available() > 0) {
    String command;

    if (BTSerial.available() > 0)
      command = BTSerial.readStringUntil('\n');
    else
      command = Serial.readStringUntil('\n');

    command.trim();
    command.toLowerCase();

    if (command == "voice" || command == "v") {
      currentMode = VOICE_CONTROL;
      Robot_Stop();
      Serial.println("VOICE MODE");
    } 
    else if (command == "emg" || command == "e") {
      currentMode = EMG_CONTROL;
      Robot_Stop();
      isMoving = false;
      lastEMGActionTime = 0;

      data_index = 0;
      sum = 0;
      for (int i = 0; i < BUFFER_SIZE; i++)
        circular_buffer[i] = 0;

      resetEMGFilter();
      Serial.println("EMG MODE");
    }
  }
}

// ========== VOICE CONTROL ==========
void handleVoiceControl() {
  if (BTSerial.available() > 0 || Serial.available() > 0) {
    String raw;

    if (BTSerial.available() > 0)
      raw = BTSerial.readStringUntil('\n');
    else
      raw = Serial.readStringUntil('\n');

    raw.trim();
    raw.toLowerCase();

    commandReceivedTime = micros();

    if (raw == "menu" || raw == "m") {
      currentMode = MODE_SELECTION;
      Robot_Stop();
      promptModeSelection();
      return;
    }

    if (raw == "left" || raw == "l") {
      Robot_Left();
      printLatency("LEFT");
      delay(500);
      Robot_Stop();
    }
    else if (raw == "right" || raw == "r") {
      Robot_Right();
      printLatency("RIGHT");
      delay(500);
      Robot_Stop();
    }
    else if (raw == "forward" || raw == "f") {
      Robot_Forward();
      isMoving = true;
      printLatency("FORWARD");
    }
    else if (raw == "back" || raw == "b") {
      Robot_Reverse();
      isMoving = true;
      printLatency("BACK");
    }
    else if (raw == "stop" || raw == "s") {
      Robot_Stop();
      isMoving = false;
      printLatency("STOP");
    }
  }
}

// ========== EMG CONTROL ==========
void handleEMGControl() {
  static unsigned long past = 0;
  unsigned long present = micros();
  unsigned long interval = present - past;
  past = present;

  static long timer = 0;
  timer -= interval;

  if (timer < 0) {
    timer += 1000000 / SAMPLE_RATE;

    int sensor_value = analogRead(INPUT_PIN);
    float signal = EMGFilter(sensor_value);
    int envelope = getEnvelope(abs(signal));

    if ((millis() - lastEMGActionTime) > EMG_DEBOUNCE_DELAY) {
      if (envelope > EMG_THRESHOLD) {

        commandReceivedTime = micros();

        if (isMoving) {
          Robot_Stop();
          isMoving = false;
        } else {
          Robot_Forward();
          isMoving = true;
        }

        motorActionTime = micros();
        Serial.print("EMG Latency (us): ");
        Serial.println(motorActionTime - commandReceivedTime);

        lastEMGActionTime = millis();
      }
    }
  }
}

// ========== LATENCY PRINT ==========
void printLatency(String action) {
  motorActionTime = micros();
  Serial.print("Action: ");
  Serial.print(action);
  Serial.print(" | Latency (us): ");
  Serial.println(motorActionTime - commandReceivedTime);
}

// ========== MOTOR FUNCTIONS ==========
void Robot_Reverse() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void Robot_Forward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void Robot_Left() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void Robot_Right() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void Robot_Stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ========== EMG PROCESSING ==========
void resetEMGFilter() {
  for (int i = 0; i < 10; i++) {
    EMGFilter(0);
  }
}

int getEnvelope(int abs_emg) {
  sum -= circular_buffer[data_index];
  sum += abs_emg;
  circular_buffer[data_index] = abs_emg;
  data_index = (data_index + 1) % BUFFER_SIZE;
  return (sum / BUFFER_SIZE) * 2;
}

float EMGFilter(float input) {
  float output = input;
  {
    static float z1, z2;
    float x = output - 0.05159732*z1 - 0.36347401*z2;
    output = 0.01856301*x + 0.03712602*z1 + 0.01856301*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2;
    float x = output - -0.53945795*z1 - 0.39764934*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2;
    float x = output - 0.47319594*z1 - 0.70744137*z2;
    output = 1.00000000*x + 2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2;
    float x = output - -1.00211112*z1 - 0.74520226*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}
