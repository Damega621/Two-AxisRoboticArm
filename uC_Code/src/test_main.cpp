#include <Arduino.h>
#include "GCodeParser.h"

// Simulated robot state
struct RobotState {
  float x_pos;
  float y_pos;
  bool absolute_mode;
};

GCodeParser GCode;
RobotState state;

bool emergency_triggered = false;
bool start = true;

// === Function Declarations ===
void handleMotion(float x, float y, float f);
void resetState();
void processGCodeLine(const char* gcodeLine);
void logAction(const char* action, float x, float y, float f, const char* mode);

// === Setup ===
void setup() {
  Serial.begin(115200);

    while (!Serial) {
    ; // wait for Serial Monitor to open
  }
  delay(500);
  resetState();
 
}

// === Loop: User input simulation ===
void loop() {
  if(start){
    Serial.println("=== G-CODE & E-STOP VALIDATION ===");
    Serial.println("Timestamp(ms),Action,X,Y,Feedrate,Mode");

    processGCodeLine("G90");               // Absolute mode
    processGCodeLine("G1 X10 Y10 F100");   // Move
    processGCodeLine("G1 X20 Y5 F100");    // Move

    Serial.println("Simulating E-Stop (type 'e' to trigger)...");
    Serial.flush();  // <--- Add this
    start = false;
}

  if (Serial.available()) {
    char c = Serial.read();

    if (c == 'e') {
      Serial.println(">>> [E-STOP] triggered!");
      emergency_triggered = true;
      logAction("E-STOP Triggered", 0, 0, 0, "");
      Serial.println("Type 'r' to resume or 'o' to return to origin.");
    }

    if (emergency_triggered) {
      if (c == 'r') {
        Serial.println("[Resuming] Continuing to X30 Y0...");
        processGCodeLine("G1 X30 Y0 F100");
        emergency_triggered = false;
      } else if (c == 'o') {
        Serial.println("[Returning to Origin]...");
        processGCodeLine("G1 X0 Y0 F100");
        resetState();
        logAction("Return to Origin", 0, 0, 100, "ABS");
        emergency_triggered = false;
      }
    }
  }
}

// === Reset robot state ===
void resetState() {
  state.x_pos = 0;
  state.y_pos = 0;
  state.absolute_mode = true;
  Serial.println("[System] State reset. Position = (0,0), Mode = ABS");
}

// === Simulated motion execution ===
void handleMotion(float x, float y, float f) {
  if (!state.absolute_mode) {
    x += state.x_pos;
    y += state.y_pos;
  }

  Serial.print("→ Moving to X=");
  Serial.print(x);
  Serial.print(", Y=");
  Serial.print(y);
  Serial.print(" @ Feedrate=");
  Serial.println(f);

  logAction("G1 Command", x, y, f, state.absolute_mode ? "ABS" : "REL");

  delay(300);  // Simulated time delay
  state.x_pos = x;
  state.y_pos = y;
}

// === G-code parser driver ===
void processGCodeLine(const char* gcodeLine) {
  GCode.AddCharToLine('\n');  // Clear parser's previous state
  for (size_t i = 0; i < strlen(gcodeLine); ++i) {
    GCode.AddCharToLine(gcodeLine[i]);
  }

  GCode.ParseLine();

  if (GCode.HasWord('G')) {
    int g = (int)GCode.GetWordValue('G');
    switch (g) {
      case 0:
      case 1:
        handleMotion(GCode.GetWordValue('X'), GCode.GetWordValue('Y'), GCode.GetWordValue('F'));
        break;
      case 90:
        state.absolute_mode = true;
        Serial.println("[G90] Absolute positioning mode set.");
        logAction("Mode Set", 0, 0, 0, "ABS");
        break;
      case 91:
        state.absolute_mode = false;
        Serial.println("[G91] Relative positioning mode set.");
        logAction("Mode Set", 0, 0, 0, "REL");
        break;
      default:
        Serial.print("[Unknown G-code] G");
        Serial.println(g);
        break;
    }
  }

  if (GCode.HasWord('M')) {
    int m = (int)GCode.GetWordValue('M');
    Serial.print("[M-code detected] M");
    Serial.println(m);
    logAction("M-code Detected", 0, 0, 0, "");
  }
}

// === CSV Logging Function ===
void logAction(const char* action, float x, float y, float f, const char* mode) {
  Serial.print(millis());
  Serial.print(",");
  Serial.print(action);
  Serial.print(",");
  Serial.print(x, 2);
  Serial.print(",");
  Serial.print(y, 2);
  Serial.print(",");
  Serial.print(f, 2);
  Serial.print(",");
  Serial.println(mode);
}