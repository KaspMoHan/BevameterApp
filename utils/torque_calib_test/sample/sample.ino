// Sample 1000 ADC readings from I3 (A3) on Arduino Opta when 's' is sent over Serial.
// ADC range: 0–4095 (12-bit), input range: 0–10 V.

const int I3_PIN = A2;     // I3 is A3 on Opta
const int NUM_SAMPLES = 1000;
int samples[NUM_SAMPLES];
unsigned long timestamps[NUM_SAMPLES];

void setup() {
  Serial.begin(115200);
  analogReadResolution(16);     // 0..4095
  delay(300);
  Serial.println("Ready. Type 's' and press Enter to sample 1000 points over 10 seconds.");
}

void loop() {
  // Wait for user to type 's' in Serial Monitor
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's' || c == 'S') {
      Serial.println("Starting sampling...");
      takeSamples();
      printSamples();
      Serial.println("Done!");
    }
  }
}

void takeSamples() {
  unsigned long interval = 10000UL / NUM_SAMPLES; // ~10 seconds total, in ms per sample
  for (int i = 0; i < NUM_SAMPLES; i++) {
    samples[i] = analogRead(I3_PIN);
    timestamps[i] = millis();
    delay(interval);
  }
}

void printSamples() {
  Serial.println("Index,Time_ms,Raw,Voltage_V");
  for (int i = 0; i < NUM_SAMPLES; i++) {
    float voltage = samples[i] * (10.0f / 4095.0f);
    Serial.print(i);
    Serial.print(",");
    Serial.print(timestamps[i]);
    Serial.print(",");
    Serial.print(samples[i]);
    Serial.print(",");
    Serial.println(voltage, 5);
  }
}
