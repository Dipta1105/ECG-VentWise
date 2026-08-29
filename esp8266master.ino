#include <Arduino.h>
#include <ESP8266WiFi.h>

extern "C" {
  #include <espnow.h>
}

// =====================================================
// ESP32 DOIT DEVKIT V1 MASTER MAC ADDRESS
// MASTER / OBC
// =====================================================
uint8_t esp32MAC[] = {
  0x00, 0x70, 0x07, 0xE2, 0x22, 0xE0
};

// =====================================================
// RECEIVE CALLBACK
// =====================================================
void onDataReceive(uint8_t *mac, uint8_t *data, uint8_t len) {

  Serial.print("\n[ESP-NOW] RECEIVED: ");

  for (uint8_t i = 0; i < len; i++) {
    Serial.print((char)data[i]);
  }

  Serial.println();

  Serial.print("From MAC: ");

  for (int i = 0; i < 6; i++) {

    if (mac[i] < 16)
      Serial.print("0");

    Serial.print(mac[i], HEX);

    if (i < 5)
      Serial.print(":");
  }

  Serial.println();
}

// =====================================================
// SETUP
// =====================================================
void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("==============================");
  Serial.println(" HYDROS ESP8266 RECEIVER");
  Serial.println("==============================");

  // ===================================================
  // WiFi Station
  // ===================================================

  WiFi.mode(WIFI_STA);

  Serial.print("ESP8266 MAC: ");
  Serial.println(WiFi.macAddress());

  // ===================================================
  // ESP-NOW INITIALIZATION
  // ===================================================

  if (esp_now_init() != 0) {

    Serial.println("ESP-NOW INIT FAILED");

    return;
  }

  Serial.println("ESP-NOW INIT OK");

  // ===================================================
  // SET RECEIVER ROLE
  // ===================================================

  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

  // ===================================================
  // REGISTER RECEIVE CALLBACK
  // ===================================================

  esp_now_register_recv_cb(onDataReceive);

  // ===================================================
  // ADD ESP32 DOIT DEVKIT V1 MASTER/OBC AS PEER
  // MAC:
  // 00:70:07:E2:22:E0
  // ===================================================

  int result = esp_now_add_peer(
    esp32MAC,
    ESP_NOW_ROLE_COMBO,
    1,
    NULL,
    0
  );

  if (result == 0) {

    Serial.println("ESP32 DOIT MASTER PEER ADDED");

  } else {

    Serial.print("FAILED TO ADD ESP32 MASTER PEER: ");
    Serial.println(result);
  }

  Serial.println("==============================");
  Serial.println("RECEIVER READY");
  Serial.println("==============================");
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  delay(10);
}
