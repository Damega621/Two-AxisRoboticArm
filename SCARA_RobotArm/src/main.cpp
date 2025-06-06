#include <Arduino.h>
#include <Wire.h>
#include <GCodeParser.h>
#include <AccelStepper.h>
#include <SPIFFS.h>
#include <FS.h>

const char* ssid = "RobotArm22";
const char* password = "Team22RoboticArm";

// === A4988 Driver Pin Definitions ===
#define STEP_1 14
#define DIR_1 27
#define STEP_2 26
#define DIR_2 25

// === Motor Control Pins ===
#define ENABLE_1 12  // ENABLE pin for Motor 1
#define ENABLE_2 13  // ENABLE pin for Motor 2
#define POWER_DOWN_PIN 15  // GPIOo pin for shutdown button

// === Emergency Stop Pin ===
#define ESTOP_PIN 34

#define FILE_PRESET_1 "/preset1.gcode"
#define FILE_PRESET_2 "/preset2.gcode"
#define FILE_PRESET_3 "/preset3.gcode"

// === Physical Specifications ===

// Primary arm
#define PRIMARY_LENGTH 8
#define RATIO_1 3.0
// Secondary arm
#define SECONDARY_LENGTH 4      
#define RATIO_2 1.0 

// === Kinematics Macros ===
#define ELBOW_TOGGLE
#define ELBOW_UP 1


// === Constants ===
#define LINE_SEGMENTS 5           // Number of segments for line interpolation
#define MAX_DECELERATION_TIME 1000 // 1 second
#define BRAKING_COEFFICIENT 3.0    // Increase if too sluw deceleration, decrease if too abrupt


GCodeParser GCode;

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}