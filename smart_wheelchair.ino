// Smart Wheelchair with Voice and EMG Control
// Optimized with proper debouncing, filter reset, and minimal serial output
// Hardware: Arduino Uno/Nano + HC-05 Bluetooth + Muscle BioAmp Candy EMG Sensor

#include <SoftwareSerial.h>

// ========== CONFIGURATION ==========
#define BAUDRATE 9600
#define SAMPLE_RATE 500
#define INPUT_PIN A0
#define BUFFER_SIZE 128

// Bluetooth pins
// HC-05 TX -> Arduino Pin 10 (RX)
// HC-05 RX -> Arduino Pin 11 (TX) - USE VOLTAGE DIVIDER (1K + 2K resistor)
SoftwareSerial BTSerial(10, 11); // RX, TX

// Motor pins
const int IN1 = 6;
const int IN2 = 5;
const int IN3 = 9;
const int IN4 = 3;

// EMG threshold - adjust based on your muscle signal strength
// Monitor Serial output to calibrate this value
const int EMG_THRESHOLD = 140;

// Debouncing delay between EMG actions (milliseconds)
// Prevents multiple triggers from single muscle flex
const unsigned long EMG_DEBOUNCE_DELAY = 300;

// ========== GLOBAL VARIABLES ==========

// Control mode enumeration
enum ControlMode {
  MODE_SELECTION,
  VOICE_CONTROL,
  EMG_CONTROL
};

ControlMode currentMode = MODE_SELECTION;

// EMG signal processing variables
int circular_buffer[BUFFER_SIZE];
int data_index = 0;
long sum = 0;

// EMG control state variables
unsigned long lastEMGActionTime = 0;
bool isMoving = false;

// ========== SETUP ==========
void setup() {
  Serial.begin(BAUDRATE);      // For debugging via USB
  BTSerial.begin(BAUDRATE);    // For Bluetooth communication
  
  // Setup motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  Robot_Stop();
  BTSerial.setTimeout(200);
  
  // Small delay for Bluetooth to initialize
  delay(100);
  
  // Initialize EMG buffer
  for (int i = 0; i < BUFFER_SIZE; i++) {
    circular_buffer[i] = 0;
  }
  
  // Prompt for mode selection
  promptModeSelection();
}

// ========== MAIN LOOP ==========
void loop() {
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
  if (BTSerial.available() > 0) {
    String command = BTSerial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    if (command == "voice" || command == "v") {
      currentMode = VOICE_CONTROL;
      BTSerial.println("\n>>> VOICE CONTROL ACTIVATED <<<");
      BTSerial.println("Commands: left/right/forward/back/stop");
      BTSerial.println("Send 'menu' to change mode\n");
      Serial.println("Voice mode active");
      Robot_Stop();
    } 
    else if (command == "emg" || command == "e") {
      currentMode = EMG_CONTROL;
      BTSerial.println("\n>>> EMG CONTROL ACTIVATED <<<");
      BTSerial.print("Threshold: ");
      BTSerial.println(EMG_THRESHOLD);
      BTSerial.println("Muscle flex = STOP/START");
      BTSerial.println("Send 'menu' to change mode\n");
      Serial.println("EMG mode active");
      Robot_Stop();
      isMoving = false;
      lastEMGActionTime = 0;
      
      // Reset EMG buffers
      data_index = 0;
      sum = 0;
      for (int i = 0; i < BUFFER_SIZE; i++) {
        circular_buffer[i] = 0;
      }
      
      // Reset EMG filter states to prevent artifacts
      resetEMGFilter();
    }
    else if (command.length() > 0) {
      BTSerial.print("Unknown: ");
      BTSerial.println(command);
      promptModeSelection();
    }
  }
}

// ========== VOICE CONTROL MODE ==========
void handleVoiceControl() {
  // Check for commands from Bluetooth
  if (BTSerial.available() > 0) {
    String raw = BTSerial.readStringUntil('\n');
    raw.trim();
    if (raw.length() == 0) return;
    
    raw.toLowerCase();
    
    // Check if user wants to return to menu
    if (raw == "menu" || raw == "m") {
      currentMode = MODE_SELECTION;
      Robot_Stop();
      promptModeSelection();
      return;
    }
    
    // Process voice commands
    if (raw == "left" || raw == "l") {
      Robot_Left();
      BTSerial.println("LEFT");
    } else if (raw == "right" || raw == "r") {
      Robot_Right();
      BTSerial.println("RIGHT");
    } else if (raw == "forward" || raw == "f" || raw == "for") {
      Robot_Forward();
      BTSerial.println("FORWARD");
    } else if (raw == "back" || raw == "reverse" || raw == "b") {
      Robot_Reverse();
      BTSerial.println("BACK");
    } else if (raw == "stop" || raw == "s") {
      Robot_Stop();
      BTSerial.println("STOP");
    } else {
      BTSerial.print("Unknown: ");
      BTSerial.println(raw);
    }
  }
}

// ========== EMG CONTROL MODE ==========
void handleEMGControl() {
  // Check for mode change command from Bluetooth (non-blocking)
  if (BTSerial.available() > 0) {
    String command = BTSerial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    if (command == "menu" || command == "m") {
      currentMode = MODE_SELECTION;
      Robot_Stop();
      isMoving = false;
      promptModeSelection();
      return;
    }
  }
  
  // EMG sampling timing (non-blocking)
  static unsigned long past = 0;
  unsigned long present = micros();
  unsigned long interval = present - past;
  past = present;
  
  static long timer = 0;
  timer -= interval;
  
  // Sample EMG signal at defined rate
  if (timer < 0) {
    timer += 1000000 / SAMPLE_RATE;
    
    // Read and process EMG signal
    int sensor_value = analogRead(INPUT_PIN);
    float signal = EMGFilter(sensor_value);
    int envelope = getEnvelope(abs(signal));
    
    // Control logic with debouncing
    // Only process action if enough time has passed since last action
    if ((millis() - lastEMGActionTime) > EMG_DEBOUNCE_DELAY) {
      
      if (envelope > EMG_THRESHOLD) {
        // Muscle flexed - toggle state
        if (isMoving) {
          // Was moving -> STOP
          Robot_Stop();
          isMoving = false;
          BTSerial.print("EMG: ");
          BTSerial.print(envelope);
          BTSerial.println(" -> STOP");
          Serial.println("EMG STOP");
        } else {
          // Was stopped -> START
          Robot_Forward();
          isMoving = true;
          BTSerial.print("EMG: ");
          BTSerial.print(envelope);
          BTSerial.println(" -> START");
          Serial.println("EMG START");
        }
        
        // Update last action time for debouncing
        lastEMGActionTime = millis();
      }
    }
    
    // NO continuous serial printing here - it interferes with signal acquisition
    // Only print during state changes (see above)
  }
}

// ========== MOTOR CONTROL FUNCTIONS ==========
void Robot_Forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void Robot_Reverse() {
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

// ========== EMG SIGNAL PROCESSING ==========

// Reset EMG filter state variables
// Prevents artifacts when re-entering EMG mode
void resetEMGFilter() {
  // Call EMGFilter with 0 input multiple times to clear the filter state
  // This flushes out old values from the static variables
  for (int i = 0; i < 10; i++) {
    EMGFilter(0);
  }
}

// Envelope detection algorithm
int getEnvelope(int abs_emg) {
  sum -= circular_buffer[data_index];
  sum += abs_emg;
  circular_buffer[data_index] = abs_emg;
  data_index = (data_index + 1) % BUFFER_SIZE;
  return (sum / BUFFER_SIZE) * 2;
}

// Band-Pass Butterworth IIR digital filter
// Sampling rate: 500.0 Hz, frequency: [74.5, 149.5] Hz
// Filter is order 4, implemented as second-order sections (biquads)
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
float EMGFilter(float input) {
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - 0.05159732*z1 - 0.36347401*z2;
    output = 0.01856301*x + 0.03712602*z1 + 0.01856301*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -0.53945795*z1 - 0.39764934*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - 0.47319594*z1 - 0.70744137*z2;
    output = 1.00000000*x + 2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.00211112*z1 - 0.74520226*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}
