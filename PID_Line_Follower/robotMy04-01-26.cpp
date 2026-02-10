#include <Arduino.h>
#include "TRSensors.h"
#include <math.h>
#include <Wire.h>

// ======= PROTOCOL ======= 
static const uint8_t STX = 0x02;
static const uint8_t ETX = 0x03;

uint8_t xorChecksum(const uint8_t* data, size_t len) {
  uint8_t cs = 0;
  for (size_t i = 0; i < len; ++i) cs ^= data[i];
  return cs;
}

void sendFrame(const String &payload) {
  Serial.write(STX);
  Serial.write((const uint8_t*)payload.c_str(), payload.length());
  Serial.write(ETX);
  Serial.write(xorChecksum((const uint8_t*)payload.c_str(), payload.length()));
}

// =======================================================
// ======================= DEFINES =======================
// =======================================================
String processCommand(const String& cmd);

unsigned long myTime = 0;
int t = 20;                  // loop interval (ms)

int baseSpeedLeft  = 80;
int baseSpeedRight = 80;   // slightly lower to compensate drift


float distance = 0.0f;

float kp = 0.25f; //0
float ki = 0.0f;
float kd = 0.0f;

float integral = 0.0f;
float previousError = 0.0f;
float distance_point = 26.0f;
int servo_zero = 85;         // baseline servo angle
float lastOutput = 0.0f;
bool command_move_forward = false;   // <===== NEW FLAG
bool command_line_follow = true;   // <===== NEW
int proportional = 0;               // <===== NEW GLOBAL



#define PIN_LEFT_MOTOR_SPEED 5
#define PIN_LEFT_MOTOR_FORWARD A1            
#define PIN_LEFT_MOTOR_REVERSE A0
#define PIN_LEFT_ENCODER 2
   
#define PIN_RIGHT_MOTOR_SPEED 6
#define PIN_RIGHT_MOTOR_FORWARD A2            
#define PIN_RIGHT_MOTOR_REVERSE A3
#define PIN_RIGHT_ENCODER 3

#define NUM_SENSORS 5
TRSensors trs =   TRSensors();
unsigned int sensorValues[NUM_SENSORS];
unsigned int last_proportional = 0;


int left_encoder_count=0;
int right_encoder_count=0;

void left_encoder(){
  left_encoder_count++;  
}

void right_encoder(){
  right_encoder_count++;  
}



// =======================================================
// ===================== SETUP ===========================
// =======================================================
void setup(){
  Serial.begin(115200);
  
  for (int i = 0; i < 400; i++){    // make the calibration take about 10 seconds
    trs.calibrate();                             // reads all sensors 10 times
  }
    

  pinMode(PIN_LEFT_MOTOR_SPEED, OUTPUT);
  analogWrite(PIN_LEFT_MOTOR_SPEED, 0);
  pinMode(PIN_LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(PIN_LEFT_MOTOR_REVERSE, OUTPUT);

  pinMode(PIN_RIGHT_MOTOR_SPEED, OUTPUT);
  analogWrite(PIN_RIGHT_MOTOR_SPEED, 0);
  pinMode(PIN_RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(PIN_RIGHT_MOTOR_REVERSE, OUTPUT);
  

  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENCODER), left_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENCODER), right_encoder, RISING);


  myTime = millis();


}



// =======================================================
// ===================== UNIFIED LOOP ====================
// =======================================================


// add these at global scope:
unsigned long myTimePID   = 0;
unsigned long myTimeLine  = 0;

void loop() {
  unsigned long now = millis();

  // ================================================
  // 1) PID / periodic logic every t ms
  // ================================================
  if (now - myTimePID >= (unsigned long)t) {
    myTimePID = now;

    if (command_line_follow) {      // <===== NEW
      // PID calculations
      int error = proportional;
      integral += error;
      int derivative = error - previousError;
      previousError = error;

      int output = (int)(kp * error + ki * integral + kd * derivative);
      output = constrain(output, -100, 100);

      lastOutput = (float)output / 100.0f;

      int leftSpeed  = baseSpeedLeft  + output;
      int rightSpeed = baseSpeedRight - output;


      leftSpeed = constrain(leftSpeed, 0, 255);
      rightSpeed = constrain(rightSpeed, 0, 255);

      // Left motor
      digitalWrite(PIN_LEFT_MOTOR_FORWARD, HIGH);
      digitalWrite(PIN_LEFT_MOTOR_REVERSE, LOW);
      analogWrite(PIN_LEFT_MOTOR_SPEED, leftSpeed);

      // Right motor
      digitalWrite(PIN_RIGHT_MOTOR_FORWARD, HIGH);
      digitalWrite(PIN_RIGHT_MOTOR_REVERSE, LOW);
      analogWrite(PIN_RIGHT_MOTOR_SPEED, rightSpeed);
    }



  }

  // ================================================
  // 2) PROTOCOL HANDLING 
  // ================================================
  static enum { WAIT_STX, READ_PAYLOAD, GOT_ETX, WAIT_CK } state = WAIT_STX;
  static char buf[128];
  static size_t idx = 0;

  while (Serial.available()) {
    uint8_t b = (uint8_t)Serial.read();

    switch (state) {
      case WAIT_STX:
        if (b == STX) { idx = 0; state = READ_PAYLOAD; }
        break;

      case READ_PAYLOAD:
        if (b == ETX) state = GOT_ETX;
        else if (idx + 1 < sizeof(buf)) buf[idx++] = (char)b;
        break;

      case GOT_ETX: {
        uint8_t cs_recv = b;
        uint8_t cs_calc = xorChecksum((uint8_t*)buf, idx);
        buf[idx] = '\0';

        if (cs_recv == cs_calc) {
          String reply = processCommand(String(buf));
          sendFrame(reply);
        } else {
          sendFrame("ERR_CSUM");
        }

        idx = 0;
        state = WAIT_STX;
      } break;

      default:
        state = WAIT_STX;
        break;
    }
  }

  // ================================================
  // 3) LINE FOLLOWING / MOVEMENT LOGIC
  // ================================================
  if (now - myTimeLine >= 100) {
    myTimeLine = now;

    unsigned int position = trs.readLine(sensorValues);
    proportional = (int)position - 2000;

    left_encoder_count = 0;
    right_encoder_count = 0;

    // If "M" command active → override everything
    if (command_move_forward) {
    // Move straight
    digitalWrite(PIN_LEFT_MOTOR_FORWARD, HIGH);
    digitalWrite(PIN_LEFT_MOTOR_REVERSE, LOW);
    analogWrite(PIN_LEFT_MOTOR_SPEED, 50);

    digitalWrite(PIN_RIGHT_MOTOR_FORWARD, HIGH);
    digitalWrite(PIN_RIGHT_MOTOR_REVERSE, LOW);
    analogWrite(PIN_RIGHT_MOTOR_SPEED, 50);
    return;
}

if (!command_line_follow) {
    // Stop when not in PID mode
    analogWrite(PIN_LEFT_MOTOR_SPEED, 0);
    analogWrite(PIN_RIGHT_MOTOR_SPEED, 0);
    digitalWrite(PIN_LEFT_MOTOR_FORWARD, LOW);
    digitalWrite(PIN_LEFT_MOTOR_REVERSE, LOW);
    digitalWrite(PIN_RIGHT_MOTOR_FORWARD, LOW);
    digitalWrite(PIN_RIGHT_MOTOR_REVERSE, LOW);
}


    
  
    }
  }




/*
    // PID calculations
    int error = proportional;
    integral += error;
    int derivative = error - previousError;
    previousError = error;

    int output = (int)(kp * error + ki * integral + kd * derivative);
    lastOutput = (float)output / 100.0f;  // store for reporting

    // Motor speed adjustments
    int leftSpeed = 150 + output;
    int rightSpeed = 150 - output;

    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    // Set left motor
    if (leftSpeed >= 0) {
      digitalWrite(PIN_LEFT_MOTOR_FORWARD, HIGH);
      digitalWrite(PIN_LEFT_MOTOR_REVERSE, LOW);
    } else {
      digitalWrite(PIN_LEFT_MOTOR_FORWARD, LOW);
      digitalWrite(PIN_LEFT_MOTOR_REVERSE, HIGH);
      leftSpeed = -leftSpeed;
    }
    analogWrite(PIN_LEFT_MOTOR_SPEED, leftSpeed);

    // Set right motor
    if (rightSpeed >= 0) {
      digitalWrite(PIN_RIGHT_MOTOR_FORWARD, HIGH);
      digitalWrite(PIN_RIGHT_MOTOR_REVERSE, LOW);
    } else {
      digitalWrite(PIN_RIGHT_MOTOR_FORWARD, LOW);
      digitalWrite(PIN_RIGHT_MOTOR_REVERSE, HIGH);
      rightSpeed = -rightSpeed;
    }
    analogWrite(PIN_RIGHT_MOTOR_SPEED, rightSpeed);
*/




// === Command handler ===
// Supported:
//  - "PING"                 -> "PONG"
//  - "PID?"                 -> "PID kp ki kd"
//  - "PID <kp> <ki> <kd>"   -> "OK" or "ERR_ARG"
//  - "DIST?"                -> "DIST <value_cm>"
//  - "SP <value_cm>"        -> "OK" or "ERR_ARG"
//  - "STEP"                 -> "STEP distance output"
//  - "MAE"                  -> "MAE:START"
String processCommand(const String& cmdIn) {
  String cmd = cmdIn;
  cmd.trim();

  if (cmd.equalsIgnoreCase("PING")) {
    return "PONG";
  }

  if (cmd.equalsIgnoreCase("PID?")) {
    return String("PID ") + String(kp, 4) + " " + String(ki, 4) + " " + String(kd, 4);
  }

  if (cmd.startsWith("PID ")) {
    // Expect: PID <kp> <ki> <kd>
    int s1 = cmd.indexOf(' ');
    if (s1 < 0) return "ERR_ARG";
    String rest = cmd.substring(s1 + 1);
    rest.trim();

    // Split by spaces
    int s2 = rest.indexOf(' ');
    int s3 = rest.lastIndexOf(' ');
    if (s2 <= 0 || s3 <= s2) return "ERR_ARG";

    String kpStr = rest.substring(0, s2);
    String kiStr = rest.substring(s2 + 1, s3);
    String kdStr = rest.substring(s3 + 1);

    float nkp = kpStr.toFloat();
    float nki = kiStr.toFloat();
    float nkd = kdStr.toFloat();

    // Basic validation: allow zero/negatives if desired; just set them
    kp = nkp; ki = nki; kd = nkd;
    return "OK";
  }

  if (cmd.startsWith("M")) {
    command_move_forward = true;
    return "MOVING";
  }

   if (cmd.startsWith("P")) {          
    command_line_follow = true;
    command_move_forward = false;   
    return "PID_ON";
}

if (cmd.startsWith("S")) {          
    command_move_forward = false;
    command_line_follow = false;    
    return "STOPPED";
}

if (cmd.startsWith("C")){
  for (int i = 0; i < 100; i++){    // make the calibration take about 10 seconds
    trs.calibrate();                             // reads all sensors 10 times
  }
  return "CALIBRATED";
}


  if (cmd.startsWith("SP ")) {
    String v = cmd.substring(3);
    v.trim();
    if (v.length() == 0) return "ERR_ARG";
    distance_point = v.toFloat();
    return "OK";
  }

  






  return "ERR_CMD";
}









