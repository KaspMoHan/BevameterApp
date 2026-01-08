#include <OptaBlue.h>
#include <OptaExpansion.h>
#include <DigitalExpansion.h>
#include <math.h>

// -------------------- Expansion device and channels --------------------
const int EXP_DEVICE       = 0;   // Analog expansion (DAC/ADC)
const int EXP_RELAY_DEVICE = 1;   // Relay/Digital expansion (AFX00005)

// ==================== PHYSICAL WIRING (unchanged) ====================
// Physical Actuator "A" (was grouser actuator hardware)
const int PHYS_A_POS_FEEDBACK   = A0;

// Physical Actuator "B" (was rubber actuator hardware)
const int PHYS_B_POS_FEEDBACK   = A1;

// Only wired torque SGA (physically still on actuator "B" wiring)
const int PHYS_B_TORQUE_FEEDBACK = A2;

// Load cell SGA (unchanged)
const int EXP_PRESSURE_FORCE_FEEDBACK  = A3;  // load cell force

// Grouser torque placeholder (unwired)
const int EXP_TORQUE1_FEEDBACK         = A5;  // placeholder (unwired)

// Pressure actuator (unchanged)
const int EXP_PRESSURE_POS_FEEDBACK    = A4;  // position feedback
const int EXP_DAC_PRESSURE_SET_SPEED   = 4;   // dedicated DAC channel

// DAC speed channels (unchanged; both grouser/rubber share DAC 7)
const int EXP_DAC_SHARED_SET_SPEED     = 7;

// -------------------- Base-unit relays (PHYSICAL) --------------------
// Physical Actuator "A" direction relays (was grouser)
const int PHYS_A_RELAY_FWD = D0;
const int PHYS_A_RELAY_BWD = D1;

// Physical Actuator "B" direction relays (was rubber)
const int PHYS_B_RELAY_FWD = D2;
const int PHYS_B_RELAY_BWD = D3;

// -------------------- LEDs (optional) --------------------
const int LED_FWD = LED_D0;
const int LED_BWD = LED_D1;

// -------------------- Relay expansion channels (winches + pressure DIR) --------------------
// Physical winch channels (as wired)
const uint8_t PHYS_B_WINCH_UP_CH     = 0;
const uint8_t PHYS_B_WINCH_DOWN_CH   = 1;
const uint8_t PHYS_A_WINCH_UP_CH     = 2;
const uint8_t PHYS_A_WINCH_DOWN_CH   = 3;

// pressure DIR relays (DigitalExpansion)
const uint8_t PRESSURE_RELAY_FWD_CH  = 5;
const uint8_t PRESSURE_RELAY_BWD_CH  = 6;

// ==================== LOGICAL MAPPING (SWAPPED) ====================
// You mounted attachments swapped:
// - Logical "grouser" is now on physical actuator B
// - Logical "rubber"  is now on physical actuator A
const int EXP_ACTUATOR_GROUSER_POS_FEEDBACK = PHYS_B_POS_FEEDBACK; // A1
const int EXP_ACTUATOR_RUBBER_POS_FEEDBACK  = PHYS_A_POS_FEEDBACK; // A0

const int RELAY_FWD_GROUSER = PHYS_B_RELAY_FWD; // D2
const int RELAY_BWD_GROUSER = PHYS_B_RELAY_BWD; // D3
const int RELAY_FWD_RUBBER  = PHYS_A_RELAY_FWD; // D0
const int RELAY_BWD_RUBBER  = PHYS_A_RELAY_BWD; // D1

// Winches swapped logically as well
const uint8_t GROUSER_RELAY_WINCH_UP_CH    = PHYS_B_WINCH_UP_CH;   // 0
const uint8_t GROUSER_RELAY_WINCH_DOWN_CH  = PHYS_B_WINCH_DOWN_CH; // 1
const uint8_t RUBBER_RELAY_WINCH_UP_CH     = PHYS_A_WINCH_UP_CH;   // 2
const uint8_t RUBBER_RELAY_WINCH_DOWN_CH   = PHYS_A_WINCH_DOWN_CH; // 3

// Torque feedback: the only wired torque is physically on actuator B,
// which is now logical "grouser". So publish it as grouser_torque.
const int EXP_GROUSER_TORQUE_FEEDBACK = PHYS_B_TORQUE_FEEDBACK; // A2
// rubber torque currently not available
// ========================================================

// -------------------- Command buffer --------------------
constexpr int CMD_BUF_LEN = 96;
char   cmdBuf[CMD_BUF_LEN];
size_t cmdLen = 0;

// -------------------- Safety failsafe (load cell) --------------------
// Trip at +/- 8 kN, auto-reset at +/- 7.5 kN (hysteresis)
const float SAFE_TRIP_LIMIT_N  = 8000.0f;
const float SAFE_RESET_LIMIT_N = 7500.0f;

// Calibration: V = 0.0039*kg + 4.9556  =>  N = ((V-b)/m)*g
const float CAL_M_V_PER_KG = 0.0039f;
const float CAL_B_V        = 4.9556f;
const float G_N_PER_KG     = 9.80665f;

// ADC scaling (16-bit, 0..10 V)
const float ADC_VREF_V     = 10.0f;
const float ADC_BITS_MAX_F = 65535.0f;

// Debounce: require N consecutive over-limit samples to trip
const uint8_t SAFE_DEBOUNCE_SAMPLES = 3;

volatile bool    g_safety_tripped = false;
volatile uint8_t g_over_ct = 0;

// Forward decl
static inline void pressure_set_dir(bool fwd_on, bool bwd_on);

// -------------------- Helpers --------------------
static inline float adc_bits_to_volts_u16(uint16_t bits) {
  return ((float)bits / ADC_BITS_MAX_F) * ADC_VREF_V;
}

static inline float volts_to_force_newton(float v) {
  return ((v - CAL_B_V) / CAL_M_V_PER_KG) * G_N_PER_KG;
}

static inline void stop_all_outputs() {
  // Stop ALL DAC outputs (set speed to 0)
  {
    Opta::Expansion &baseExp = OptaController.getExpansion(EXP_DEVICE);
    AnalogExpansion &exp = static_cast<AnalogExpansion&>(baseExp);
    exp.setDac(EXP_DAC_SHARED_SET_SPEED, 0);      // shared 7
    exp.setDac(EXP_DAC_PRESSURE_SET_SPEED, 0);    // 4
  }

  // Open ALL base-unit direction relays (both physical actuators)
  digitalWrite(PHYS_A_RELAY_FWD, LOW);
  digitalWrite(PHYS_A_RELAY_BWD, LOW);
  digitalWrite(PHYS_B_RELAY_FWD, LOW);
  digitalWrite(PHYS_B_RELAY_BWD, LOW);

  // Open pressure direction relays
  pressure_set_dir(false, false);

  // LEDs off
  digitalWrite(LED_FWD, LOW);
  digitalWrite(LED_BWD, LOW);
}

// Auto-reset safety check with hysteresis
static inline void safety_check_force() {
  uint16_t bits = analogRead(EXP_PRESSURE_FORCE_FEEDBACK);
  float v  = adc_bits_to_volts_u16(bits);
  float fN = volts_to_force_newton(v);
  float af = fabsf(fN);

  if (!g_safety_tripped) {
    if (af >= SAFE_TRIP_LIMIT_N) {
      if (++g_over_ct >= SAFE_DEBOUNCE_SAMPLES) {
        g_safety_tripped = true;
        g_over_ct = 0;
        stop_all_outputs();

        char msg[96];
        snprintf(msg, sizeof(msg), "SAFE:TRIP:F=%.0fN bits=%u V=%.3f", (double)fN, bits, (double)v);
        Serial.println(msg);
      }
    } else {
      g_over_ct = 0;
    }
  } else {
    stop_all_outputs();
    if (af <= SAFE_RESET_LIMIT_N) {
      g_safety_tripped = false;
      g_over_ct = 0;
      Serial.println("SAFE:RESET:AUTO");
    }
  }
}

void emit_sensor_line() {
  unsigned long ts_ms = millis();

  // LOGICAL readings (swapped)
  uint16_t grouser_pos     = analogRead(EXP_ACTUATOR_GROUSER_POS_FEEDBACK);
  uint16_t rubber_pos      = analogRead(EXP_ACTUATOR_RUBBER_POS_FEEDBACK);

  // Only wired torque is now considered grouser_torque
  uint16_t grouser_torque  = analogRead(EXP_GROUSER_TORQUE_FEEDBACK);
  uint16_t rubber_torque   = 0; // not wired right now

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

// Atomic pressure DIR setter
static inline void pressure_set_dir(bool fwd_on, bool bwd_on) {
  Opta::Expansion& baseExp = OptaController.getExpansion(EXP_RELAY_DEVICE);
  auto& dexp = static_cast<Opta::DigitalExpansion&>(baseExp);
  dexp.digitalWrite(PRESSURE_RELAY_FWD_CH, fwd_on ? HIGH : LOW);
  dexp.digitalWrite(PRESSURE_RELAY_BWD_CH, bwd_on ? HIGH : LOW);
  dexp.updateDigitalOutputs();
}

void handle_cmd(const char* cmd) {
  // Status is always allowed
  if (memcmp(cmd, "SAFE:STATUS", 11) == 0) {
    Serial.print("SAFE:STATUS:");
    Serial.println(g_safety_tripped ? "TRIPPED" : "OK");
    return;
  }

  // If tripped: allow sensor reads, reject motion/winch commands
  if (g_safety_tripped) {
    if (memcmp(cmd, "SENSOR:READ_ALL", 15) == 0) { emit_sensor_line(); return; }

    if (memcmp(cmd, "pressure:READ_FORCE", 19) == 0) {
      uint16_t bits = analogRead(EXP_PRESSURE_FORCE_FEEDBACK);
      Serial.print("pressure:FORCE:"); Serial.println(bits);
      return;
    }
    if (memcmp(cmd, "pressure:READ_POS", 17) == 0) {
      uint16_t bits = analogRead(EXP_PRESSURE_POS_FEEDBACK);
      Serial.print("pressure:POS:"); Serial.println(bits);
      return;
    }
    if (memcmp(cmd, "grouser:READ_POS", 16) == 0) {
      uint16_t bits = analogRead(EXP_ACTUATOR_GROUSER_POS_FEEDBACK);
      Serial.print("grouser:POS:"); Serial.println(bits);
      return;
    }
    if (memcmp(cmd, "rubber:READ_POS", 15) == 0) {
      uint16_t bits = analogRead(EXP_ACTUATOR_RUBBER_POS_FEEDBACK);
      Serial.print("rubber:POS:");  Serial.println(bits);
      return;
    }

    Serial.println("SAFE:TRIPPED:IGNORED_CMD");
    return;
  }

  // ---------------- Direction commands (LOGICAL) ----------------
  if (memcmp(cmd, "DIR:FWD:grouser", 15) == 0) {
    digitalWrite(RELAY_FWD_GROUSER, HIGH);
    digitalWrite(RELAY_BWD_GROUSER, LOW);
    digitalWrite(LED_FWD, HIGH);
    digitalWrite(LED_BWD, LOW);
    Serial.println("grouser:DIR:FWD");

  } else if (memcmp(cmd, "DIR:FWD:rubber", 14) == 0) {
    digitalWrite(RELAY_FWD_RUBBER, HIGH);
    digitalWrite(RELAY_BWD_RUBBER, LOW);
    digitalWrite(LED_FWD, HIGH);
    digitalWrite(LED_BWD, LOW);
    Serial.println("rubber:DIR:FWD");

  } else if (memcmp(cmd, "DIR:BWD:grouser", 15) == 0) {
    digitalWrite(RELAY_FWD_GROUSER, LOW);
    digitalWrite(RELAY_BWD_GROUSER, HIGH);
    digitalWrite(LED_FWD, LOW);
    digitalWrite(LED_BWD, HIGH);
    Serial.println("grouser:DIR:BWD");

  } else if (memcmp(cmd, "DIR:BWD:rubber", 14) == 0) {
    digitalWrite(RELAY_FWD_RUBBER, LOW);
    digitalWrite(RELAY_BWD_RUBBER, HIGH);
    digitalWrite(LED_FWD, LOW);
    digitalWrite(LED_BWD, HIGH);
    Serial.println("rubber:DIR:BWD");

  } else if (memcmp(cmd, "DIR:FWD:pressure", 16) == 0) {
    pressure_set_dir(true, false);
    Serial.println("pressure:DIR:FWD");

  } else if (memcmp(cmd, "DIR:BWD:pressure", 16) == 0) {
    pressure_set_dir(false, true);
    Serial.println("pressure:DIR:BWD");

  } else if (memcmp(cmd, "ALL:DIR:OFF", 11) == 0) {
    stop_all_outputs();
    Serial.println("ALL:DIR:OFF");

  // ---------------- Speed setpoints (analog expansion) ----------------
  } else if (memcmp(cmd, "grouser:SPEED:", 14) == 0) {
    int val = atoi(cmd + 14);
    val = constrain(val, 0, (1 << 13) - 1);
    Opta::Expansion &baseExp = OptaController.getExpansion(EXP_DEVICE);
    AnalogExpansion &exp = static_cast<AnalogExpansion&>(baseExp);
    exp.setDac(EXP_DAC_SHARED_SET_SPEED, uint16_t(val)); // shared 7
    Serial.print("grouser:SPEED:"); Serial.println(val);

  } else if (memcmp(cmd, "rubber:SPEED:", 13) == 0) {
    int val = atoi(cmd + 13);
    val = constrain(val, 0, (1 << 13) - 1);
    Opta::Expansion &baseExp = OptaController.getExpansion(EXP_DEVICE);
    AnalogExpansion &exp = static_cast<AnalogExpansion&>(baseExp);
    exp.setDac(EXP_DAC_SHARED_SET_SPEED, uint16_t(val)); // shared 7
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
    uint16_t bits = analogRead(EXP_ACTUATOR_GROUSER_POS_FEEDBACK);
    Serial.print("grouser:POS:"); Serial.println(bits);

  } else if (memcmp(cmd, "rubber:READ_POS", 15) == 0) {
    uint16_t bits = analogRead(EXP_ACTUATOR_RUBBER_POS_FEEDBACK);
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
  pinMode(PHYS_A_RELAY_FWD, OUTPUT);
  pinMode(PHYS_A_RELAY_BWD, OUTPUT);
  pinMode(PHYS_B_RELAY_FWD, OUTPUT);
  pinMode(PHYS_B_RELAY_BWD, OUTPUT);
  pinMode(LED_FWD, OUTPUT);
  pinMode(LED_BWD, OUTPUT);

  stop_all_outputs();

  // Analog expansion DAC channels
  AnalogExpansion::beginChannelAsDac(
    OptaController, EXP_DEVICE, EXP_DAC_SHARED_SET_SPEED,   // shared 7
    OA_VOLTAGE_DAC, true, false, OA_SLEW_RATE_0
  );
  AnalogExpansion::beginChannelAsDac(
    OptaController, EXP_DEVICE, EXP_DAC_SHARED_SET_SPEED,   // shared 7 (same channel, OK)
    OA_VOLTAGE_DAC, true, false, OA_SLEW_RATE_0
  );
  AnalogExpansion::beginChannelAsDac(
    OptaController, EXP_DEVICE, EXP_DAC_PRESSURE_SET_SPEED, // pressure on 4
    OA_VOLTAGE_DAC, true, false, OA_SLEW_RATE_0
  );

  // Relay expansion: default OPEN all
  {
    Opta::Expansion& baseExp = OptaController.getExpansion(EXP_RELAY_DEVICE);
    auto& dexp = static_cast<Opta::DigitalExpansion&>(baseExp);

    // winches (logical mapping already swapped via channel constants)
    dexp.digitalWrite(RUBBER_RELAY_WINCH_UP_CH,   LOW);
    dexp.digitalWrite(RUBBER_RELAY_WINCH_DOWN_CH, LOW);
    dexp.digitalWrite(GROUSER_RELAY_WINCH_UP_CH,  LOW);
    dexp.digitalWrite(GROUSER_RELAY_WINCH_DOWN_CH,LOW);

    // pressure DIR
    dexp.digitalWrite(PRESSURE_RELAY_FWD_CH, LOW);
    dexp.digitalWrite(PRESSURE_RELAY_BWD_CH, LOW);

    dexp.updateDigitalOutputs();
  }
}

void loop() {
  // Continuous safety check
  safety_check_force();

  // Non-blocking line assembler
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      cmdBuf[cmdLen] = '\0';
      if (cmdLen > 0) handle_cmd(cmdBuf);
      cmdLen = 0;
    } else if (c != '\r') {
      if (cmdLen < CMD_BUF_LEN - 1) {
        cmdBuf[cmdLen++] = c;
      } else {
        cmdLen = 0;
      }
    }
  }
}
