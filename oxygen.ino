// ============================================================
// ESP32 DEVKIT V1
// POTENTIOMETER CONTROLLED RELAY RATE
// ============================================================
//
// POTENTIOMETER:
// VCC  -> 3.3V
// GND  -> GND
// SIG  -> GPIO 34
//
// RELAY:
// IN   -> GPIO 25
// VCC  -> 5V
// GND  -> GND
//
// NOTE:
// GPIO 34 is input-only, which is perfect for the potentiometer.
// ESP32 ADC resolution = 12-bit (0-4095)
//
// ============================================================

const int POT_PIN   = 34;
const int RELAY_PIN = 25;

// ------------------------------------------------------------
// Relay switching interval
// ------------------------------------------------------------
// Minimum interval = fastest switching
// Maximum interval = slowest switching
//
// 500 ms = 0.5 second
// 5000 ms = 5 seconds
// ------------------------------------------------------------

const unsigned long MIN_INTERVAL = 500;
const unsigned long MAX_INTERVAL = 5000;

bool relayState = LOW;

unsigned long previousMillis = 0;
unsigned long lastPrint = 0;

void setup() {

  Serial.begin(115200);

  // ESP32 ADC setup
  analogReadResolution(12);   // 0-4095

  pinMode(POT_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);

  // Relay initially OFF
  digitalWrite(RELAY_PIN, LOW);

  Serial.println();
  Serial.println("==========================================");
  Serial.println(" ESP32 POTENTIOMETER RELAY RATE CONTROL");
  Serial.println("==========================================");
  Serial.println();

  Serial.println("Potentiometer : GPIO 34");
  Serial.println("Relay         : GPIO 25");
  Serial.println("ADC Range     : 0 - 4095");
  Serial.println();
}

void loop() {

  // ==========================================================
  // READ POTENTIOMETER
  // ==========================================================

  int potValue = analogRead(POT_PIN);

  // ==========================================================
  // CONVERT POT VALUE TO RELAY INTERVAL
  // ==========================================================
  //
  // Pot at minimum  -> 5 seconds
  // Pot at maximum  -> 0.5 seconds
  //
  // ==========================================================

  unsigned long interval = map(
    potValue,
    0,
    4095,
    MAX_INTERVAL,
    MIN_INTERVAL
  );

  // ==========================================================
  // RELAY CONTROL
  // ==========================================================

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {

    previousMillis = currentMillis;

    // Toggle relay
    relayState = !relayState;

    digitalWrite(RELAY_PIN, relayState);
  }

  // ==========================================================
  // SERIAL MONITOR
  // ==========================================================

  if (currentMillis - lastPrint >= 500) {

    lastPrint = currentMillis;

    Serial.print("Pot: ");
    Serial.print(potValue);

    Serial.print(" / 4095");

    Serial.print(" | Interval: ");
    Serial.print(interval);
    Serial.print(" ms");

    Serial.print(" | Relay: ");

    if (relayState) {
      Serial.println("ON");
    }
    else {
      Serial.println("OFF");
    }
  }
}
