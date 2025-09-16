#include <OptaBlue.h>
#include <OptaExpansion.h>

// -------------------- Expansion device and channels --------------------
const int EXP_DEVICE = 0;

// Actuator 1 (grouser)
const int EXP_ACTUATOR1_POS_FEEDBACK = A0;   // ADC input for position feedback
const int EXP_DAC_ACTUATOR1_SET_SPEED = 7;   // DAC channel for speed setpoint

// Actuator 2 (rubber)
const int EXP_ACTUATOR2_POS_FEEDBACK = A1;   // ADC input for position feedback
const int EXP_DAC_ACTUATOR2_SET_SPEED = 4;   // DAC channel for speed setpoint

// -------------------- Relays --------------------
const int RELAY_FWD_ACTUATOR1 = D0;  // Grouser forward
const int RELAY_BWD_ACTUATOR1 = D1;  // Grouser backward
const int RELAY_FWD_ACTUATOR2 = D2;  // Rubber forward
const int RELAY_BWD_ACTUATOR2 = D3;  // Rubber backward

// -------------------- LEDs (optional) --------------------
const int LED_FWD = LED_D0;
const int LED_BWD = LED_D1;

// -------------------- Command buffer --------------------
constexpr int CMD_BUF_LEN = 96;
char   cmdBuf[CMD_BUF_LEN];
size_t cmdLen = 0;

// -------------------- Helpers --------------------
void emit_sensor_line() {
  unsigned long ts_ms = millis();
  uint16_t grouser_pos = analogRead(EXP_ACTUATOR1_POS_FEEDBACK);
  uint16_t rubber_pos  = analogRead(EXP_ACTUATOR2_POS_FEEDBACK);

  Serial.print("SENS:ts_ms=");   Serial.print(ts_ms);
  Serial.print(",grouser_pos="); Serial.print(grouser_pos);
  Serial.print(",rubber_pos=");  Serial.println(rubber_pos);
}

void handle_cmd(const char* cmd) {
  // ---------------- Direction commands ----------------
  if (memcmp(cmd, "DIR:FWD:grouser", 15) == 0) {
    digitalWrite(RELAY_FWD_ACTUATOR1, HIGH);
    digitalWrite(RELAY_BWD_ACTUATOR1, LOW);
    digitalWrite(LED_FWD, HIGH);
    digitalWrite(LED_BWD, LOW);
    Serial.println("grouser:DIR:FWD");

  } else if (memcmp(cmd, "DIR:FWD:rubber", 14) == 0) {
    digitalWrite(RELAY_FWD_ACTUATOR2, HIGH);
    digitalWrite(RELAY_BWD_ACTUATOR2, LOW);
    digitalWrite(LED_FWD, HIGH);
    digitalWrite(LED_BWD, LOW);
    Serial.println("rubber:DIR:FWD");

  } else if (memcmp(cmd, "DIR:BWD:grouser", 15) == 0) {
    digitalWrite(RELAY_FWD_ACTUATOR1, LOW);
    digitalWrite(RELAY_BWD_ACTUATOR1, HIGH);
    digitalWrite(LED_FWD, LOW);
    digitalWrite(LED_BWD, HIGH);
    Serial.println("grouser:DIR:BWD");

  } else if (memcmp(cmd, "DIR:BWD:rubber", 14) == 0) {
    digitalWrite(RELAY_FWD_ACTUATOR2, LOW);
    digitalWrite(RELAY_BWD_ACTUATOR2, HIGH);
    digitalWrite(LED_FWD, LOW);
    digitalWrite(LED_BWD, HIGH);
    Serial.println("rubber:DIR:BWD");

  } else if (memcmp(cmd, "ALL:DIR:OFF", 11) == 0) {
    digitalWrite(RELAY_FWD_ACTUATOR1, LOW);
    digitalWrite(RELAY_BWD_ACTUATOR1, LOW);
    digitalWrite(RELAY_FWD_ACTUATOR2, LOW);
    digitalWrite(RELAY_BWD_ACTUATOR2, LOW);
    digitalWrite(LED_FWD, LOW);
    digitalWrite(LED_BWD, LOW);
    Serial.println("ALL:DIR:OFF");

  // ---------------- Speed setpoints ----------------
  } else if (memcmp(cmd, "grouser:SPEED:", 14) == 0) {
    int val = atoi(cmd + 14);
    val = constrain(val, 0, (1 << 13) - 1);
    Opta::Expansion &baseExp = OptaController.getExpansion(EXP_DEVICE);
    AnalogExpansion &exp = static_cast<AnalogExpansion&>(baseExp);
    exp.setDac(EXP_DAC_ACTUATOR1_SET_SPEED, uint16_t(val));
    Serial.print("grouser:SPEED:"); Serial.println(val);

  } else if (memcmp(cmd, "rubber:SPEED:", 13) == 0) {
    int val = atoi(cmd + 13);
    val = constrain(val, 0, (1 << 13) - 1);
    Opta::Expansion &baseExp = OptaController.getExpansion(EXP_DEVICE);
    AnalogExpansion &exp = static_cast<AnalogExpansion&>(baseExp);
    exp.setDac(EXP_DAC_ACTUATOR2_SET_SPEED, uint16_t(val));
    Serial.print("rubber:SPEED:");  Serial.println(val);

  // ---------------- Individual position reads (legacy) ----------------
  } else if (memcmp(cmd, "grouser:READ_POS", 16) == 0) {
    uint16_t bits = analogRead(EXP_ACTUATOR1_POS_FEEDBACK);
    Serial.print("grouser:POS:"); Serial.println(bits);

  } else if (memcmp(cmd, "rubber:READ_POS", 15) == 0) {
    uint16_t bits = analogRead(EXP_ACTUATOR2_POS_FEEDBACK);
    Serial.print("rubber:POS:");  Serial.println(bits);

  // ---------------- Full snapshot (positions only) ----------------
  } else if (memcmp(cmd, "SENSOR:READ_ALL", 15) == 0) {
    emit_sensor_line();

  } else {
    Serial.print("ERR:Unknown cmd:"); Serial.println(cmd);
  }
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(13);
  // Serial.setTimeout(10); // not needed now (we don't use readBytesUntil)
  while (!Serial) {}

  OptaController.begin();
  delay(200);

  pinMode(RELAY_FWD_ACTUATOR1, OUTPUT);
  pinMode(RELAY_BWD_ACTUATOR1, OUTPUT);
  pinMode(RELAY_FWD_ACTUATOR2, OUTPUT);
  pinMode(RELAY_BWD_ACTUATOR2, OUTPUT);
  pinMode(LED_FWD, OUTPUT);
  pinMode(LED_BWD, OUTPUT);

  digitalWrite(RELAY_FWD_ACTUATOR1, LOW);
  digitalWrite(RELAY_BWD_ACTUATOR1, LOW);
  digitalWrite(RELAY_FWD_ACTUATOR2, LOW);
  digitalWrite(RELAY_BWD_ACTUATOR2, LOW);
  digitalWrite(LED_FWD, LOW);
  digitalWrite(LED_BWD, LOW);

  // DAC config
  AnalogExpansion::beginChannelAsDac(
    OptaController, EXP_DEVICE, EXP_DAC_ACTUATOR1_SET_SPEED,
    OA_VOLTAGE_DAC, true, false, OA_SLEW_RATE_0
  );
  AnalogExpansion::beginChannelAsDac(
    OptaController, EXP_DEVICE, EXP_DAC_ACTUATOR2_SET_SPEED,
    OA_VOLTAGE_DAC, true, false, OA_SLEW_RATE_0
  );
}

void loop() {
  // Non-blocking line assembler
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      cmdBuf[cmdLen] = '\0';
      if (cmdLen > 0) handle_cmd(cmdBuf);
      cmdLen = 0;
    } else if (c != '\r') { // ignore CR
      if (cmdLen < CMD_BUF_LEN - 1) {
        cmdBuf[cmdLen++] = c;
      } else {
        cmdLen = 0; // overflow -> reset
      }
    }
  }

  // nothing else; loop remains responsive
}
