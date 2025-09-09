#include <OptaBlue.h>
#include <OptaExpansion.h>

// Expansion device and channels
const int EXP_DEVICE   = 0;                  // Expansion device index

const int EXP_ACTUATOR1_POS_FEEDBACK = A0;   // ADC terminal for positional feedback, shear-actuator 1
const int EXP_DAC_ACTUATOR1_SET_SPEED   = 7; // DAC channel for writing speed, shear-actuator 1


// TO DO: Shear-actuator 2
// const int EXP_ACTUATOR1_POS_FEEDBACK = A0;   // ADC terminal for positional feedback, shear-actuator 2
// const int EXP_DAC_ACTUATOR1_SET_SPEED   = 7; // DAC channel for writing speed, shear-actuator 2


// Relay control pins
const int RELAY_FWD_ACTUATOR1 = D0;       // Relay switching shear-actuator 1 to forward-mode
const int RELAY_BWD_ACTUATOR1 = D1;       // Relay switching shear-actuator 1 to reverse-mode

// TO DO: Shear-actuator 2
// const int RELAY_FWD_ACTUATOR2 = D0;       // Relay switching shear-actuator 2 to forward-mode
// const int RELAY_BWD_ACTUATOR2 = D1;       // Relay switching shear-actuator 2 to reverse-mode

// TO DO: Configure the relays, channels, etc. needed for the force-depth actuator


// User LEDs (optional)
const int LED_FWD      = LED_D0;
const int LED_BWD      = LED_D1;

// Command buffer
constexpr int CMD_BUF_LEN = 32;
char cmdBuf[CMD_BUF_LEN];

void setup() {
  // 1) Serial @115200, small RX timeout
  Serial.begin(115200);
  analogReadResolution(13);
  while (!Serial) {}

  // 2) Core Opta init
  OptaController.begin();
  delay(200);

  // 3) Relay & LED pins
  pinMode(RELAY_FWD_ACTUATOR1, OUTPUT);
  pinMode(RELAY_BWD_ACTUATOR1, OUTPUT);
  pinMode(LED_FWD,   OUTPUT);
  pinMode(LED_BWD,   OUTPUT);
  digitalWrite(RELAY_FWD_ACTUATOR1, LOW);
  digitalWrite(RELAY_BWD_ACTUATOR1, LOW);
  digitalWrite(LED_FWD,   LOW);
  digitalWrite(LED_BWD,   LOW);

  // 4) Configure expansion DAC
  AnalogExpansion::beginChannelAsDac(
    OptaController,
    EXP_DEVICE,
    EXP_DAC_ACTUATOR1_SET_SPEED,
    OA_VOLTAGE_DAC,
    true,    // enable
    false,   // unbuffered
    OA_SLEW_RATE_0
  );
  // TO DO: Configure other channels as DAC's, as above:
  //   AnalogExpansion::beginChannelAsDac(
  //  OptaController,
  //  EXP_DEVICE,
  //  EXP_DAC_ACTUATOR2_SET_SPEED,
  //  OA_VOLTAGE_DAC,
  //  true,    // enable
  //  false,   // unbuffered
  //  OA_SLEW_RATE_0
  // );
}

void loop() {
  // only read when there’s data
  if (Serial.available() == 0) return;

  // read up to newline
  int len = Serial.readBytesUntil('\n', cmdBuf, CMD_BUF_LEN - 1);
  if (len <= 0) return;
  cmdBuf[len] = '\0';

  // -- DIR:FWD:grouser --
  if (memcmp(cmdBuf, "DIR:FWD:grouser", 15) == 0) {
    digitalWrite(RELAY_FWD_ACTUATOR1, HIGH);
    digitalWrite(RELAY_BWD_ACTUATOR1, LOW);
    digitalWrite(LED_FWD,   HIGH);
    digitalWrite(LED_BWD,   LOW);
    Serial.println("grouser:DIR:FWD");
    Serial.flush();

  // -- DIR:BWD:grouser --
  } else if (memcmp(cmdBuf, "DIR:BWD:grouser", 15) == 0) {
    digitalWrite(RELAY_FWD_ACTUATOR1, LOW);
    digitalWrite(RELAY_BWD_ACTUATOR1, HIGH);
    digitalWrite(LED_FWD,   LOW);
    digitalWrite(LED_BWD,   HIGH);
    Serial.println("grouser:DIR:BWD");
    Serial.flush();

  // TODO: ADD LOGIC TO SET DIRECTION OF ACTUATOR2 (rubber) AND
  // ACTUATOR3 (pressure/sinkage)

  // -- DIR:OFF --
  } else if (memcmp(cmdBuf, "ALL:DIR:OFF", 11) == 0) {
    digitalWrite(RELAY_FWD_ACTUATOR1, LOW);
    digitalWrite(RELAY_BWD_ACTUATOR1, LOW);
    //digitalWrite(RELAY_FWD_ACTUATOR2, LOW);
    //digitalWrite(RELAY_BWD_ACTUATOR2, LOW);
    digitalWrite(LED_FWD,   LOW);
    digitalWrite(LED_BWD,   LOW);
    Serial.println("ALL:DIR:OFF");
    Serial.flush();

  // -- SPEED:<bits> --
  } else if (memcmp(cmdBuf, "grouser:SPEED:", 14) == 0) {
    int val = atoi(cmdBuf + 14);
    val = constrain(val, 0, (1 << 13) - 1);
    // use the same expansion instance you configured
    // retrieve the analog expansion instance
    // retrieve the base expansion instance and cast to AnalogExpansion
    Opta::Expansion &baseExp = OptaController.getExpansion(EXP_DEVICE);
    AnalogExpansion &exp = static_cast<AnalogExpansion&>(baseExp);
    exp.setDac(EXP_DAC_ACTUATOR1_SET_SPEED, uint16_t(val));
    Serial.print("grouser:SPEED:");
    Serial.println(val);  // or the actual value written to DAC
    Serial.flush();
  
  // TODO: ADD rubber:SPEED logic
    // -- READ_POS --
  } else if (memcmp(cmdBuf, "grouser:READ_POS", 16) == 0) {
    uint16_t bits = analogRead(EXP_ACTUATOR1_POS_FEEDBACK);
    // include the relay name here:
    Serial.print("grouser:POS:");
    Serial.println(bits);
    Serial.flush();
  }

  // TODO: ADD rubber READ_POS statement
}
