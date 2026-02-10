
#include <Arduino.h>
#include <Servo.h>
#include <math.h>

// === PROTOCOL (preserved) === [1]
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

// Forward declare
String processCommand(const String& cmd);

// === PID + distance (from pilka.cpp) === [2]
Servo myservo;
unsigned long myTime = 0;
int t = 100;                  // loop interval (ms)

float distance = 0.0f;
float kp = 1.5f;
float ki = 0.05f;
float kd = 1.1f;
float integral = 0.0f;
float previousError = 0.0f;
float distance_point = 25.0f;
int servo_zero = 130;         // baseline servo angle
float lastOutput = 0.0f;


// === MAE state ===
float mae_sum_of_errors = 0.0f;
int mae_sample_count = 0;
float calculated_mae = 0.0f;

unsigned long phase1_start_time = 0;
unsigned long phase2_start_time = 0;
bool phase1_active = false;
bool phase2_active = false;
bool mae_active = false;

const unsigned long PHASE1_DURATION = 10000UL; // 10 s
const unsigned long PHASE2_DURATION = 3000UL; // 3 s
// Control gating: when false, PID is stopped and servo held at baseline
bool control_enabled = true;


float get_dist(int n){
  long sum = 0;
  for (int i = 0; i < n; i++) {
    sum += analogRead(A0);
  }
  float adc = float(sum) / float(n);
  float distance_cm = 17569.7f * pow(adc, -1.2062f);  // model from pilka.cpp [2]
  return distance_cm;
}

void PIDStep(){
  float proportional = distance - distance_point;
  integral += proportional * 0.1f;                  // dt ≈ 0.1s for t=100ms
  float derivative = (proportional - previousError) / 0.1f;
  lastOutput = kp * proportional + ki * integral + kd * derivative;
  previousError = proportional;

  int cmd = servo_zero + (int)lastOutput;
  if (cmd < 0) cmd = 0;
  if (cmd > 180) cmd = 180;
  myservo.write(cmd);
}

// === SETUP ===
void setup() {
  Serial.begin(115200);
  myservo.attach(9);
  pinMode(A0, INPUT);
  myTime = millis();
}


void MAEhandle(){

  if (mae_active) {
unsigned long now = millis();

// Phase 1: stabilization (10 s)
if (phase1_active) {
if (now - phase1_start_time >= PHASE1_DURATION) {
phase1_active = false;
phase2_active = true;
phase2_start_time = now;
}
}
// Phase 2: MAE sampling (3 s)
else if (phase2_active) {
// Error in cm: distance vs distance_point
float current_error = distance - distance_point;
float absolute_error = fabsf(current_error);
mae_sum_of_errors += absolute_error;
mae_sample_count++;

if (now - phase2_start_time >= PHASE2_DURATION) {
phase2_active = false;
mae_active = false;

if (mae_sample_count > 0) {
calculated_mae = mae_sum_of_errors / (float)mae_sample_count;
} else {
calculated_mae = 0.0f;
}

// Report result in a framed payload (two decimals)
sendFrame(String("MAE ") + String(calculated_mae, 2)); // protocol-safe <source_id data="2" title="robotMy.cpp" />

// Stop all control as requested: hold servo at baseline
control_enabled = false;
}
}
  }}








// === LOOP ===
void loop() {
  // Periodic PID execution without any raw prints (protocol-safe)
  if (millis() - myTime >= (unsigned long)t) {
    distance = get_dist(100);
    myTime = millis();
    MAEhandle();
    // PID action or hold servo (avoid double-running PID)
  if (control_enabled) {
    PIDStep();
    } else {
   myservo.write(servo_zero);
    }
  }

  

  // Protocol state machine (preserved) [1]
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
        if (b == ETX) {
          state = GOT_ETX;
        } else {
          if (idx + 1 < sizeof(buf)) buf[idx++] = (char)b;  // keep room for '\0'
        }
        break;

      case GOT_ETX: {
          // b is checksum
          uint8_t cs_recv = b;
          uint8_t cs_calc = xorChecksum((uint8_t*)buf, idx);
          buf[idx] = '\0';
          if (cs_recv == cs_calc) {
            String reply = processCommand(String(buf));
            sendFrame(reply);
          } else {
            sendFrame("ERR_CSUM");
          }
          // reset
          idx = 0;
          state = WAIT_STX;
        }
        break;

      default:
        state = WAIT_STX;
        break;
    }
  }
}

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

  if (cmd.equalsIgnoreCase("DIST?")) {
    float d = get_dist(100);
    return String("DIST ") + String(d, 2);
  }

  if (cmd.startsWith("SP ")) {
    String v = cmd.substring(3);
    v.trim();
    if (v.length() == 0) return "ERR_ARG";
    distance_point = v.toFloat();
    return "OK";
  }

  if (cmd.equalsIgnoreCase("STEP")) {
    distance = get_dist(100);
    PIDStep();
    return String("STEP ") + String(distance, 2) + " " + String(lastOutput, 3);
  }

  if (cmd.equalsIgnoreCase("MAE")) {
    // Reset and start Phase 1
    mae_sum_of_errors = 0.0f;
    mae_sample_count = 0;
    calculated_mae = 0.0f;

    phase1_active = true;
    phase2_active = false;
    mae_active = true;
    control_enabled = true; // keep PID running during the test
    phase1_start_time = millis();

    return "MAE:START";
  }

  // Query last computed result
  if (cmd.equalsIgnoreCase("MAE?")) {
    return String("MAE:") + String(calculated_mae, 2);
  }




  return "ERR_CMD";
}


