#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <FS.h>
#include <SPIFFS.h>
#include "GCodeParser.h"

// === A4988 Driver Pin Definitions ===
#define STEP_1 14
#define DIR_1 27
#define STEP_2 26
#define DIR_2 25
#define ESTOP_PIN 34

// === Presets ===
#define PRESET_1_PIN 32
#define PRESET_2_PIN 33
#define PRESET_3_PIN 35

#define FILE_PRESET_1 "/preset1.gcode"
#define FILE_PRESET_2 "/preset2.gcode"
#define FILE_PRESET_3 "/preset3.gcode"

// === Physical Specifications ===

// Primary arm
#define PRIMARY_LENGTH 10
#define RATIO_1 3.0
// Secondary arm
#define SECONDARY_LENGTH 10      
#define RATIO_2 1.0 

// === Kinematics Macros ===
#define ELBOW_TOGGLE
#define ELBOW_UP 1


// === Constants ===
#define LINE_SEGMENTS 50           // Number of segments for line interpolation
#define MAX_DECELERATION_TIME 1000 // 1 second
#define BRAKING_COEFFICIENT 3.0    // Increase if too sluw deceleration, decrease if too abrupt


GCodeParser GCode;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool emergency_triggered = false;

struct RobotState {
  float theta1_deg = 0;
  float theta2_deg = 0;
  bool absolute_mode = 0;
  bool units_in_inches = 0;
} Arm;

struct ProgramState {
  RobotState Arm;
  char current_file_path[64] = "";
  int last_file_offset = 0;
} execution_state;

void IRAM_ATTR onEmergencyStop() {
  portENTER_CRITICAL_ISR(&timerMux);
  emergency_triggered = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void saveState() { 
  EEPROM.put(0, execution_state); 
  EEPROM.commit();
  Serial.println("[SAVE] State saved to EEPROM");
  }
void loadState() { 
  
  EEPROM.get(0, execution_state);
   
  Serial.println("[LOAD] State loaded from EEPROM");
  Serial.print("Theta1: ");
  Serial.println(execution_state.Arm.theta1_deg);
  Serial.print("Theta2: ");
  Serial.println(execution_state.Arm.theta2_deg);
  Serial.print("File Path: ");
  Serial.println(execution_state.current_file_path);
  Serial.print("Last Offset: ");
  Serial.println(execution_state.last_file_offset);
}

void emergencyStop() {
  Serial.println("[EMG] Motors disabled");
  saveState();
}

void stepperPulse(int stepPin, int dirPin, bool direction) {
  digitalWrite(dirPin, direction);
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(100);
}

void stepMotorTo(float theta1_target, float theta2_target) {
  
  int steps1 = (theta1_target - execution_state.Arm.theta1_deg) * RATIO_1;
  int steps2 = (theta2_target - execution_state.Arm.theta1_deg) * RATIO_2;

  int dir1 = steps1 > 0;
  int dir2 = steps2 > 0;
  steps1 = abs(steps1);
  steps2 = abs(steps2);
  

  int max_steps = max(steps1, steps2);
  unsigned long start_time = 0;
  const unsigned long decel_time = MAX_DECELERATION_TIME; 
  // Easing coefficient
  const float EXP_K = BRAKING_COEFFICIENT;                  

 for (int i = 0; i < max_steps; ++i) {
    
    if (emergency_triggered) {
      if (start_time == 0) start_time = millis();  // Mark when decel starts
      float elapsed = millis() - start_time;
      float progress = min(1.0f, elapsed / (float)decel_time);
      float scale = 1.0f - exp(-EXP_K * progress);
      int dynamic_delay = 5 + (int)(50 * scale);  // 5ms base + scaled slowdown

      if (i < steps1) stepperPulse(STEP_1, DIR_1, dir1);
      if (i < steps2) stepperPulse(STEP_2, DIR_2, dir2);
      delay(dynamic_delay);

      if (elapsed >= decel_time) break;  // Fully stopped
    } else {
      if (i < steps1) stepperPulse(STEP_1, DIR_1, dir1);
      if (i < steps2) stepperPulse(STEP_2, DIR_2, dir2);
      delay(5);  // Normal stepping delay
    }
  }
  execution_state.Arm.theta1_deg = theta1_target;
  execution_state.Arm.theta2_deg = theta2_target;

}

void origin() {
  Serial.println("Homing to origin...");
  execution_state.Arm.theta1_deg = 0;
  execution_state.Arm.theta2_deg = 0;
}



void Motion(float x, float y, float feedrate) {
  
    if (!execution_state.Arm.units_in_inches) {
        feedrate /= 25.4; // Convert mm to inches
    } 
        
    float l1 = PRIMARY_LENGTH, l2 = SECONDARY_LENGTH;
    float D = (x*x + y*y - l1*l1 - l2*l2) / (2 * l1 * l2); //cos(theta2)
    
    //Range Validation
    if (D > 1 || D < -1) {
    Serial.println("Out of range");
    return;
    }
    
    #ifdef ELBOW_TOGGLE
    float curr_theta = execution_state.Arm.theta2_deg;
    float target1 = curr_theta + acos(D);
    float target2 = curr_theta - acos(D);
    
    // toggle between solutions based on distance from target 
    bool elbow_up = fabs(target1 * 180.0 / PI) < fabs(target2 * 180.0 / PI);
    float theta2 = elbow_up ? -acos(D) : acos(D);
    float offset = atan2(l2 * sin(theta2), l1 + l2 * cos(theta2));
    float theta1 = elbow_up ? atan2(y, x) + offset : atan2(y, x) - offset;
    
    #else
    float theta1;
    float theta2 = ELBOW_UP? -acos(D) : acos(D); //Use Macro to default to a single solution
    //float theta2 = atan2(sqrt(1 - D*D), D); //Alternative Solution Computation
    float offset =  atan2(l2 * sin(theta2), l1 + l2 * cos(theta2));
    
    theta1 = theta2 >= 0? atan2(y, x) - offset : atan2(y, x) + offset;
    
    #endif
    
    stepMotorTo(theta1 * 180.0 / PI, theta2 * 180.0 / PI); //Convert to degrees
    }
    
void LineInterpolation(float x_target, float y_target, float feedrate) {
    float l1 = PRIMARY_LENGTH, l2 = SECONDARY_LENGTH;
    
    if (!execution_state.Arm.units_in_inches) {
        x_target /= 25.4;
        y_target /= 25.4;
    }
    
    // Recompute current X, Y from current joint angles
    float theta1_rad = execution_state.Arm.theta1_deg * PI / 180.0;
    float theta2_rad = execution_state.Arm.theta2_deg * PI / 180.0;
    
    float x0 = l1 * cos(theta1_rad) + l2 * cos(theta1_rad + theta2_rad);
    float y0 = l1 * sin(theta1_rad) + l2 * sin(theta1_rad + theta2_rad);
    
    // Now interpolate in Cartesian space
    float dx = x_target - x0;
    float dy = y_target - y0;
    
    const int num_segments = LINE_SEGMENTS; // Adjustable: more for smoother
    for (int i = 1; i <= num_segments; ++i) {
    float t = (float)i / num_segments;
    float xt = x0 + t * dx;
    float yt = y0 + t * dy;
    
    Motion(xt, yt, feedrate);
  }
}

void processGCode() {
  if (GCode.HasWord('G')) {
    int code = GCode.GetWordValue('G');
    switch (code) {
      case 20: execution_state.Arm.units_in_inches = true; Serial.println("Units: inches"); break;
      case 21: execution_state.Arm.units_in_inches = false; Serial.println("Units: mm"); break;

      case 0:
      case 1:
        LineInterpolation(GCode.GetWordValue('X'), GCode.GetWordValue('Y'), GCode.GetWordValue('F'));
        break;
      case 90: execution_state.Arm.absolute_mode = true; break;
      case 91: execution_state.Arm.absolute_mode = false; break;
      default: Serial.print("Unknown G-code: G"); Serial.println(code);
    }
  }

  if (GCode.HasWord('M')) {
    int m = GCode.GetWordValue('M');
    if (m == 2) emergencyStop();
    else if (m == 6) Serial.println("Tool change requested.");
  }
}

void executeFile(const char* path, bool resume = false) {
  File file = SPIFFS.open(path);
  if (!file) {
    Serial.print("Failed to open file: ");
    Serial.println(path);
    return;
  }

  // Track file path
  strncpy(execution_state.current_file_path, path, sizeof(execution_state.current_file_path));
  execution_state.last_file_offset = 0;
  saveState();

  if (resume && execution_state.last_file_offset > 0) {
    file.seek(execution_state.last_file_offset, SeekSet);
    Serial.print("Resuming at last stored offset: ");
    Serial.println(execution_state.last_file_offset);
  }

  while (file.available()) {
    if (emergency_triggered) {
      execution_state.last_file_offset = file.position();
      saveState();
      emergencyStop();
      file.close();
      return;
    }

    char c = file.read();
    if (GCode.AddCharToLine(c)) {
      GCode.ParseLine();
      processGCode();
    }
  }

  file.close();
  Serial.println("Finished drawing from file.");
  execution_state.last_file_offset = 0;
  execution_state.current_file_path[0] = '\0';
  saveState();
}

void setup() {
  Serial.begin(115200);

  pinMode(STEP_1, OUTPUT);
  pinMode(DIR_1, OUTPUT);
  pinMode(STEP_2, OUTPUT);
  pinMode(DIR_2, OUTPUT);

  pinMode(ESTOP_PIN, INPUT_PULLUP);
  pinMode(PRESET_1_PIN, INPUT_PULLUP);
  pinMode(PRESET_2_PIN, INPUT_PULLUP);
  pinMode(PRESET_3_PIN, INPUT_PULLUP);

  EEPROM.begin(sizeof(ProgramState));

  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), onEmergencyStop, FALLING);

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
  }

  GCode = GCodeParser();
  loadState();
  
  // Validate State
  if (isnan(execution_state.Arm.theta1_deg) || isnan(execution_state.Arm.theta2_deg)) {
    origin(); 
    saveState(); // Only save if origin() used
  }
}

void loop() {
  if (digitalRead(PRESET_1_PIN) == LOW) executeFile(FILE_PRESET_1);
  if (digitalRead(PRESET_2_PIN) == LOW) executeFile(FILE_PRESET_2);
  if (digitalRead(PRESET_3_PIN) == LOW) executeFile(FILE_PRESET_3);

  if (emergency_triggered) {
    emergencyStop();
    Serial.println("Type 'r' to resume or 'o' to return to origin.");
    while (true) {
      if (Serial.available()) {
        char c = Serial.read();
        if (c == 'r') break;
        if (c == 'o') {
          origin();
          break;
        }
      }
    }
    emergency_triggered = false;
  }

  while (Serial.available()) {
    char c = Serial.read();
    if (GCode.AddCharToLine(c)) {
      GCode.ParseLine();
      processGCode();
    }
  }
}