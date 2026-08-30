#include <Arduino.h>

// =====================================================
// RELAY PIN
// =====================================================

#define RELAY_PIN 25

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  delay(1000);

  pinMode(RELAY_PIN, OUTPUT);

  // Relay initially OFF
  digitalWrite(RELAY_PIN, LOW);

  Serial.println();
  Serial.println("==============================");
  Serial.println(" ESP32 DEVKIT V1 RELAY TEST");
  Serial.println("==============================");
  Serial.println("Type ON  -> Relay ON for 2 sec");
  Serial.println("Type OFF -> Relay ON for 2 sec");
  Serial.println("==============================");
}

// =====================================================
// RELAY PULSE
// =====================================================

void relayPulse() {

  Serial.println("Relay ON");

  digitalWrite(RELAY_PIN, HIGH);

  delay(2000);

  digitalWrite(RELAY_PIN, LOW);

  Serial.println("Relay OFF");
  Serial.println("Ready for next command...");
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  if (Serial.available()) {

    String command = Serial.readStringUntil('\n');

    command.trim();

    command.toUpperCase();

    if (command == "ON") {

      Serial.println("Command received: ON");

      relayPulse();

    }

    else if (command == "OFF") {

      Serial.println("Command received: OFF");

      relayPulse();

    }

    else {

      Serial.print("Unknown command: ");
      Serial.println(command);

      Serial.println("Use ON or OFF");
    }
  }
}
