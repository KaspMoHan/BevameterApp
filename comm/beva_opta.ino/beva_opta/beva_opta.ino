#include <OptaBlue.h>
#include <OptaExpansion.h>
#include <DigitalExpansion.h>

// -------------------- Expansion device and channels --------------------
const int EXP_DEVICE       = 0;   // Analog expansion (DAC/ADC)
const int EXP_RELAY_DEVICE = 1;   // Relay/Digital expansion (AFX00005)

// Actuator 1 (grouser)
const int EXP_ACTUATOR1_POS_FEEDBACK   = A0;
const int EXP_DAC_ACTUATOR1_SET_SPEED  = 7;   // shared with rubber

// Actuator 2 (rubber)
const int EXP_ACTUATOR2_POS_FEEDBACK   = A1;
const int EXP_DAC_ACTUATOR2_SET_SPEED  = 7;   // shared with grouser
const int EXP_TORQUE2_FEEDBACK         = A2;
const int EXP_TORQUE1_FEEDBACK         = A3;

// Actuator 3 (pressure)
const int EXP_PRESSURE_POS_FEEDBACK    = A4;  // position feedback
const int EXP_PRESSURE_FORCE_FEEDBACK  = A5;  // force feedback
const int EXP_DAC_PRESSURE_SET_SPEED   = 4;   // dedicated DAC channel

// -------------------- Base-unit relays (existing) --------------------
const int RELAY_FWD_ACTUATOR1 = D0;  // Grouser forward
const int RELAY_BWD_ACTUATOR1 = D1;  // Grouser backward
const int RELAY_FWD_ACTUATOR2 = D2;  // Rubber  forward
const int RELAY_BWD_ACTUATOR2 = D3;  // Rubber  backward

// -------------------- LEDs (optional) --------------------
const int LED_FWD = LED_D0;
const int LED_BWD = LED_D1;

// -------------------- Relay expansion channels (winches + pressure DIR) --------------------
const uint8_t RUBBER_RELAY_WINCH_UP_CH     = 0;   // winch UP
const uint8_t RUBBER_RELAY_WINCH_DOWN_CH   = 1;   // winch DOWN
const uint8_t GROUSER_RELAY_WINCH_UP_CH    = 2;   // winch UP
const uint8_t GROUSER_RELAY_WINCH_DOWN_CH  = 3;   // winch DOWN

// NEW: pressure actuator direction relays (DigitalExpansion)
const uint8_t PRESSURE_RELAY_FWD_CH        = 5;   // pressure FWD
const uint8_t PRESSURE_RELAY_BWD_CH        = 6;   // pressure BWD

// -------------------- Command buffer --------------------
constexpr int CMD_BUF_LEN = 96;
char   cmdBuf[CMD_BUF_LEN];
size_t cmdLen = 0;

// -------------------- Helpers --------------------
void emit_sensor_line() {
  unsigned long ts_ms = millis();

  uint16_t grouser_pos     = analogRead(EXP_ACTUATOR1_POS_FEEDBACK);
  uint16_t rubber_pos      = analogRead(EXP_ACTUATOR2_POS_FEEDBACK);
  uint16_t rubber_torque   = analogRead(EXP_TORQUE2_FEEDBACK);
  uint16_t grouser_torque  = analogRead(EXP_TORQUE1_FEEDBACK);

  // Pressure actuator signals
  uint16_t pressure_pos    = analogRead(EXP_PRESSURE_POS_FEEDBACK);
  uint16_t pressure_force  = analogRead(EXP_PRESSURE_FORCE_FEEDBACK);

  Serial.print("SENS:ts_ms=");      Serial.print(ts_ms);
  Serial.print(",grouser_pos=");    Serial.print(grouser_pos);
  Serial.print(",rubber_pos=");     Serial.print(rubber_pos);
  Serial.print(",rubber_torque=");  Serial.print(rubber_torque);
  Serial.print(",grouser_torque="); Serial.print(grouser_torque);
  Serial.print(",pressure_pos=");   Serial.print(pressure_pos);
  Serial.print(",pressure_force="); Serial.println(pressure_force);
}

// Atomic winch setters (avoid overlap)
static inline void rubber_winch_set(bool up_on, bool down_on) {
  Opta::Expansion& baseExp = OptaController.getExpansion(EXP_RELAY_DEVICE);
  auto& dexp = static_cast<Opta::DigitalExpansion&>(baseExp);
  dexp.digitalWrite(RUBBER_RELAY_WINCH_UP_CH,   up_on   ? HIGH : LOW);
  dexp.digitalWrite(RUBBER_RELAY_WINCH_DOWN_CH, down_on ? HIGH : LOW);
  dexp.updateDigitalOutputs();
}

static inline void grouser_winch_set(bool up_on, bool down_on) {
  Opta::Expansion& baseExp = OptaController.getExpansion(EXP_RELAY_DEVICE);
  auto& dexp = static_cast<Opta::DigitalExpansion&>(baseExp);
  dexp.digitalWrite(GROUSER_RELAY_WINCH_UP_CH,   up_on   ? HIGH : LOW);
  dexp.digitalWrite(GROUSER_RELAY_WINCH_DOWN_CH, down_on ? HIGH : LOW);
  dexp.updateDigitalOutputs();
}

// NEW: atomic pressure DIR setter
static inline void pressure_set_dir(bool fwd_on, bool bwd_on) {
  Opta::Expansion& baseExp = OptaController.getExpansion(EXP_RELAY_DEVICE);
  auto& dexp = static_cast<Opta::DigitalExpansion&>(baseExp);
  dexp.digitalWrite(PRESSURE_RELAY_FWD_CH, fwd_on ? HIGH : LOW);
  dexp.digitalWrite(PRESSURE_RELAY_BWD_CH, bwd_on ? HIGH : LOW);
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

  // NEW: pressure direction (DigitalExpansion ch 5/6)
  } else if (memcmp(cmd, "DIR:FWD:pressure", 16) == 0) {
    pressure_set_dir(true, false);
    Serial.println("pressure:DIR:FWD");

  } else if (memcmp(cmd, "DIR:BWD:pressure", 16) == 0) {
    pressure_set_dir(false, true);
    Serial.println("pressure:DIR:BWD");

  } else if (memcmp(cmd, "ALL:DIR:OFF", 11) == 0) {
    digitalWrite(RELAY_FWD_ACTUATOR1, LOW);
    digitalWrite(RELAY_BWD_ACTUATOR1, LOW);
    digitalWrite(RELAY_FWD_ACTUATOR2, LOW);
    digitalWrite(RELAY_BWD_ACTUATOR2, LOW);
    // Also drop pressure relays
    pressure_set_dir(false, false);
    digitalWrite(LED_FWD, LOW);
    digitalWrite(LED_BWD, LOW);
    Serial.println("ALL:DIR:OFF");

  // ---------------- Speed setpoints (analog expansion) ----------------
  } else if (memcmp(cmd, "grouser:SPEED:", 14) == 0) {
    int val = atoi(cmd + 14);
    val = constrain(val, 0, (1 << 13) - 1);
    Opta::Expansion &baseExp = OptaController.getExpansion(EXP_DEVICE);
    AnalogExpansion &exp = static_cast<AnalogExpansion&>(baseExp);
    exp.setDac(EXP_DAC_ACTUATOR1_SET_SPEED, uint16_t(val)); // shared 7
    Serial.print("grouser:SPEED:"); Serial.println(val);

  } else if (memcmp(cmd, "rubber:SPEED:", 13) == 0) {
    int val = atoi(cmd + 13);
    val = constrain(val, 0, (1 << 13) - 1);
    Opta::Expansion &baseExp = OptaController.getExpansion(EXP_DEVICE);
    AnalogExpansion &exp = static_cast<AnalogExpansion&>(baseExp);
    exp.setDac(EXP_DAC_ACTUATOR2_SET_SPEED, uint16_t(val)); // shared 7
    Serial.print("rubber:SPEED:");  Serial.println(val);

  } else if (memcmp(cmd, "pressure:SPEED:", 15) == 0) {
    int val = atoi(cmd + 15);
    val = constrain(val, 0, (1 << 13) - 1);
    Opta::Expansion &baseExp = OptaController.getExpansion(EXP_DEVICE);
    AnalogExpansion &exp = static_cast<AnalogExpansion&>(baseExp);
    exp.setDac(EXP_DAC_PRESSURE_SET_SPEED, uint16_t(val));  // DAC 4
    Serial.print("pressure:SPEED:"); Serial.println(val);

  // ---------------- Individual reads ----------------
  } else if (memcmp(cmd, "grouser:READ_POS", 16) == 0) {
    uint16_t bits = analogRead(EXP_ACTUATOR1_POS_FEEDBACK);
    Serial.print("grouser:POS:"); Serial.println(bits);

  } else if (memcmp(cmd, "rubber:READ_POS", 15) == 0) {
    uint16_t bits = analogRead(EXP_ACTUATOR2_POS_FEEDBACK);
    Serial.print("rubber:POS:");  Serial.println(bits);

  } else if (memcmp(cmd, "pressure:READ_POS", 17) == 0) {
    uint16_t bits = analogRead(EXP_PRESSURE_POS_FEEDBACK);
    Serial.print("pressure:POS:"); Serial.println(bits);

  } else if (memcmp(cmd, "pressure:READ_FORCE", 19) == 0) {
    uint16_t bits = analogRead(EXP_PRESSURE_FORCE_FEEDBACK);
    Serial.print("pressure:FORCE:"); Serial.println(bits);

  // ---------------- Full snapshot ----------------
  } else if (memcmp(cmd, "SENSOR:READ_ALL", 15) == 0) {
    emit_sensor_line();

  // ---------------- Winch commands (relay expansion) ----------------
  } else if (memcmp(cmd, "RUBBER:WINCH:UP", 15) == 0) {
    rubber_winch_set(true, false);
    Serial.println("RUBBER:WINCH:UP");

  } else if (memcmp(cmd, "RUBBER:WINCH:DOWN", 17) == 0) {
    rubber_winch_set(false, true);
    Serial.println("RUBBER:WINCH:DOWN");

  } else if (memcmp(cmd, "RUBBER:WINCH:STOP", 17) == 0) {
    rubber_winch_set(false, false);
    Serial.println("RUBBER:WINCH:STOP");

  } else if (memcmp(cmd, "GROUSER:WINCH:UP", 16) == 0) {
    grouser_winch_set(true, false);
    Serial.println("GROUSER:WINCH:UP");

  } else if (memcmp(cmd, "GROUSER:WINCH:DOWN", 18) == 0) {
    grouser_winch_set(false, true);
    Serial.println("GROUSER:WINCH:DOWN");

  } else if (memcmp(cmd, "GROUSER:WINCH:STOP", 18) == 0) {
    grouser_winch_set(false, false);
    Serial.println("GROUSER:WINCH:STOP");

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
    OptaController, EXP_DEVICE, EXP_DAC_ACTUATOR1_SET_SPEED,   // shared 7
    OA_VOLTAGE_DAC, true, false, OA_SLEW_RATE_0
  );
  AnalogExpansion::beginChannelAsDac(
    OptaController, EXP_DEVICE, EXP_DAC_ACTUATOR2_SET_SPEED,   // shared 7
    OA_VOLTAGE_DAC, true, false, OA_SLEW_RATE_0
  );
  AnalogExpansion::beginChannelAsDac(                           // pressure on 4
    OptaController, EXP_DEVICE, EXP_DAC_PRESSURE_SET_SPEED,
    OA_VOLTAGE_DAC, true, false, OA_SLEW_RATE_0
  );

  // Relay expansion: default OPEN all
  {
    Opta::Expansion& baseExp = OptaController.getExpansion(EXP_RELAY_DEVICE);
    auto& dexp = static_cast<Opta::DigitalExpansion&>(baseExp);
    // winches
    dexp.digitalWrite(RUBBER_RELAY_WINCH_UP_CH,   LOW);
    dexp.digitalWrite(RUBBER_RELAY_WINCH_DOWN_CH, LOW);
    dexp.digitalWrite(GROUSER_RELAY_WINCH_UP_CH,  LOW);
    dexp.digitalWrite(GROUSER_RELAY_WINCH_DOWN_CH,LOW);
    // pressure DIR (new)
    dexp.digitalWrite(PRESSURE_RELAY_FWD_CH, LOW);
    dexp.digitalWrite(PRESSURE_RELAY_BWD_CH, LOW);
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
