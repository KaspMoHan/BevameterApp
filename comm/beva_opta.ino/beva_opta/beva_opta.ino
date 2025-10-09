#include <OptaBlue.h>
#include <OptaExpansion.h>
#include <DigitalExpansion.h>

// -------------------- Expansion device and channels --------------------
const int EXP_DEVICE       = 0;   // Analog expansion (DAC/ADC)
const int EXP_RELAY_DEVICE = 1;   // Relay/Digital expansion (AFX00005)

// Actuator 1 (grouser)
const int EXP_ACTUATOR1_POS_FEEDBACK = A0;
const int EXP_DAC_ACTUATOR1_SET_SPEED = 7;

// Actuator 2 (rubber)
const int EXP_ACTUATOR2_POS_FEEDBACK = A1;
const int EXP_DAC_ACTUATOR2_SET_SPEED = 4;
const int EXP_TORQUE2_FEEDBACK = A2;

// -------------------- Base-unit relays (existing) --------------------
const int RELAY_FWD_ACTUATOR1 = D0;  // Grouser forward
const int RELAY_BWD_ACTUATOR1 = D1;  // Grouser backward
const int RELAY_FWD_ACTUATOR2 = D2;  // Rubber  forward
const int RELAY_BWD_ACTUATOR2 = D3;  // Rubber  backward

// -------------------- LEDs (optional) --------------------
const int LED_FWD = LED_D0;
const int LED_BWD = LED_D1;

// -------------------- Relay expansion channels (winch) --------------------
const uint8_t RELAY_WINCH_UP_CH   = 0;   // close to drive winch UP
const uint8_t RELAY_WINCH_DOWN_CH = 1;   // close to drive winch DOWN

// -------------------- Command buffer --------------------
constexpr int CMD_BUF_LEN = 96;
char   cmdBuf[CMD_BUF_LEN];
size_t cmdLen = 0;

// -------------------- Helpers --------------------
void emit_sensor_line() {
  unsigned long ts_ms = millis();
  uint16_t grouser_pos   = analogRead(EXP_ACTUATOR1_POS_FEEDBACK);
  uint16_t rubber_pos    = analogRead(EXP_ACTUATOR2_POS_FEEDBACK);
  uint16_t rubber_torque = analogRead(EXP_TORQUE2_FEEDBACK);

  Serial.print("SENS:ts_ms=");      Serial.print(ts_ms);
  Serial.print(",grouser_pos=");    Serial.print(grouser_pos);
  Serial.print(",rubber_pos=");     Serial.print(rubber_pos);
  Serial.print(",rubber_torque=");  Serial.println(rubber_torque);
}

// Set both winch relays in one shot (avoids momentary overlap)
static inline void winch_set(bool up_on, bool down_on) {
  Opta::Expansion& baseExp = OptaController.getExpansion(EXP_RELAY_DEVICE);
  auto& dexp = static_cast<Opta::DigitalExpansion&>(baseExp);
  // buffer both desired states
  dexp.digitalWrite(RELAY_WINCH_UP_CH,   up_on   ? HIGH : LOW);
  dexp.digitalWrite(RELAY_WINCH_DOWN_CH, down_on ? HIGH : LOW);
  // push to hardware
  dexp.updateDigitalOutputs();
}

void handle_cmd(const char* cmd) {
  // ---------------- Direction commands (base-unit relays) ----------------
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

  // ---------------- Speed setpoints (analog expansion) ----------------
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

  // ---------------- Winch commands (relay expansion) ----------------
  } else if (memcmp(cmd, "WINCH:UP", 8) == 0) {
    winch_set(true, false);
    Serial.println("WINCH:UP");

  } else if (memcmp(cmd, "WINCH:DOWN", 10) == 0) {
    winch_set(false, true);
    Serial.println("WINCH:DOWN");

  } else if (memcmp(cmd, "WINCH:STOP", 10) == 0) {
    winch_set(false, false);
    Serial.println("WINCH:STOP");

  } else {
    Serial.print("ERR:Unknown cmd:"); Serial.println(cmd);
  }
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(16);
  while (!Serial) {}

  OptaController.begin();
  delay(200);

  // Base-unit relays & LEDs
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

  // Analog expansion DAC channels
  AnalogExpansion::beginChannelAsDac(
    OptaController, EXP_DEVICE, EXP_DAC_ACTUATOR1_SET_SPEED,
    OA_VOLTAGE_DAC, true, false, OA_SLEW_RATE_0
  );
  AnalogExpansion::beginChannelAsDac(
    OptaController, EXP_DEVICE, EXP_DAC_ACTUATOR2_SET_SPEED,
    OA_VOLTAGE_DAC, true, false, OA_SLEW_RATE_0
  );

  // Relay expansion: default OPEN both winch relays
  {
    Opta::Expansion& baseExp = OptaController.getExpansion(EXP_RELAY_DEVICE);
    auto& dexp = static_cast<Opta::DigitalExpansion&>(baseExp);
    dexp.digitalWrite(RELAY_WINCH_UP_CH,   LOW);
    dexp.digitalWrite(RELAY_WINCH_DOWN_CH, LOW);
    dexp.updateDigitalOutputs();
  }
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
}
