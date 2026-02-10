#include <Arduino.h>
#define TRIGER_PIN 11
#define ECHO_PIN 12
#define ANALOG_READ_IR_LEFT A4
#define DIGITAL_READ_IR_LEFT 7
#define ANALOG_READ_IR_RIGHT A5
#define DIGITAL_READ_IR_RIGHT 8
#define PIN_LEFT_MOTOR_SPEED 5
#define PIN_LEFT_MOTOR_FORWARD A0            
#define PIN_LEFT_MOTOR_REVERSE A1
#define PIN_LEFT_ENCODER 2
   
#define PIN_RIGHT_MOTOR_SPEED 6
#define PIN_RIGHT_MOTOR_FORWARD A2            
#define PIN_RIGHT_MOTOR_REVERSE A3
#define PIN_RIGHT_ENCODER 3

static const uint8_t STX = 0x02;
static const uint8_t ETX = 0x03;


void setSonar(){
  pinMode(TRIGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, OUTPUT);

}

void setMotor(){
  pinMode(ANALOG_READ_IR_LEFT, INPUT);
  pinMode(DIGITAL_READ_IR_LEFT, INPUT);
  pinMode(ANALOG_READ_IR_RIGHT, INPUT);
  pinMode(DIGITAL_READ_IR_RIGHT, INPUT);
}


// === PROTOCOL ===

String processCommand(const String &cmd) {
  if (cmd == "PING") return "PONG";
  if (cmd.startsWith("M")) return "OK_MOVE_" + cmd.substring(1); // M+100
  if (cmd.startsWith("R")) return "OK_ROT_"  + cmd.substring(1); // R-90
  if (cmd.startsWith("V")) return "OK_VERT_" + cmd.substring(1); // V+20
  if (cmd == ("B")) return "OK_BSONAR_" + cmd.substring(1); // B
  if (cmd == ("I")) return "OK_IR_" + cmd.substring(1); // I
  if (cmd == ("S")) return "OK_STOP_" + cmd.substring(1); // S
  if (cmd == "STATUS")     return "OK_STATUS"; // STATUS
  return "ERR_UNKNOWN";
}

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


// === PROTOCOL ===



// ==== SETUP ====
void setup() {
  Serial.begin(115200);
  Serial.println("Arduino ready for communication");
}
// ==== SETUP ====




void loop() {
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

      case GOT_ETX:
        {
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
