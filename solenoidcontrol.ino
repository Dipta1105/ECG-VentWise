// ============================================================
// POTENTIOMETER CONTROLLED RELAY RATE
// Arduino UNO
//
// POTENTIOMETER:
// VCC -> 5V
// GND -> GND
// SIG -> A0
//
// RELAY:
// IN -> D8
// VCC -> 5V
// GND -> GND
// ============================================================

const int POT_PIN   = A0;
const int RELAY_PIN = 8;

// Switching interval range
// Minimum = fastest
// Maximum = slowest
const unsigned long MIN_INTERVAL = 100;    // 100 ms
const unsigned long MAX_INTERVAL = 5000;   // 5 seconds

bool relayState = LOW;

unsigned long previousMillis = 0;

void setup() {
  Serial.begin(115200);

  pinMode(RELAY_PIN, OUTPUT);

  // Relay initially OFF
  digitalWrite(RELAY_PIN, LOW);

  Serial.println("======================================");
  Serial.println(" POTENTIOMETER RELAY RATE CONTROLLER");
  Serial.println("======================================");
}

void loop() {

  // ----------------------------------------------------------
  // Read potentiometer
  // ----------------------------------------------------------
  int potValue = analogRead(POT_PIN);

  // Convert 0-1023 to switching interval
  unsigned long interval = map(
    potValue,
    0,
    1023,
    MAX_INTERVAL,
    MIN_INTERVAL
  );

  // ----------------------------------------------------------
  // Non-blocking relay switching
  // ----------------------------------------------------------
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {

    previousMillis = currentMillis;

    relayState = !relayState;

    digitalWrite(RELAY_PIN, relayState);
  }

  // ----------------------------------------------------------
  // Serial monitor
  // ----------------------------------------------------------
  static unsigned long lastPrint = 0;

  if (currentMillis - lastPrint >= 500) {

    lastPrint = currentMillis;

    Serial.print("Potentiometer: ");
    Serial.print(potValue);

    Serial.print(" | Interval: ");
    Serial.print(interval);
    Serial.print(" ms");

    Serial.print(" | Relay: ");
    Serial.println(relayState ? "ON" : "OFF");
  }
}
