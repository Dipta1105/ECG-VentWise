```cpp
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// =====================================================
// HYDROS MASTER / OBC
// BOARD: ESP32 DOIT DEVKIT V1
// =====================================================
//
// MASTER MAC:
// 8C:94:DF:6D:86:F4
//
// ENV MAC:
// 00:70:07:E2:22:E0
// =====================================================

uint8_t MASTER_MAC[] = {
  0x8C, 0x94, 0xDF, 0x6D, 0x86, 0xF4
};

uint8_t ENV_MAC[] = {
  0x00, 0x70, 0x07, 0xE2, 0x22, 0xE0
};

// =====================================================
// RECEIVE CALLBACK
// =====================================================

void onDataRecv(
  const esp_now_recv_info_t *info,
  const uint8_t *data,
  int len
) {
  if (data == nullptr || len <= 0) return;

  Serial.println();
  Serial.println("================================");
  Serial.println("      ESP-NOW DATA RECEIVED");
  Serial.println("================================");

  Serial.print("FROM MAC: ");

  for (int i = 0; i < 6; i++) {
    if (info->src_addr[i] < 16) Serial.print("0");
    Serial.print(info->src_addr[i], HEX);
    if (i < 5) Serial.print(":");
  }

  Serial.println();

  Serial.print("MESSAGE: ");

  for (int i = 0; i < len; i++) {
    Serial.print((char)data[i]);
  }

  Serial.println();

  Serial.print("LENGTH: ");
  Serial.println(len);

  Serial.println("================================");
}

// =====================================================
// SEND CALLBACK
// =====================================================

void onDataSent(
  const wifi_tx_info_t *info,
  esp_now_send_status_t status
) {
  Serial.print("[ESP-NOW] SEND STATUS: ");

  if (status == ESP_NOW_SEND_SUCCESS)
    Serial.println("SUCCESS");
  else
    Serial.println("FAILED");
}

// =====================================================
// ADD ENV PEER
// =====================================================

bool addEnvPeer() {

  if (esp_now_is_peer_exist(ENV_MAC)) {
    Serial.println("ENV PEER ALREADY EXISTS");
    return true;
  }

  esp_now_peer_info_t peerInfo = {};

  memcpy(peerInfo.peer_addr, ENV_MAC, 6);

  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  esp_err_t result = esp_now_add_peer(&peerInfo);

  if (result == ESP_OK) {
    Serial.println("ENV PEER ADDED");
    return true;
  }

  Serial.print("FAILED TO ADD ENV PEER: ");
  Serial.println(result);

  return false;
}

// =====================================================
// SEND TO ENV
// =====================================================

void sendToENV(const char *message) {

  if (message == nullptr) return;

  esp_err_t result = esp_now_send(
    ENV_MAC,
    (const uint8_t *)message,
    strlen(message) + 1
  );

  if (result == ESP_OK) {
    Serial.print("[ESP-NOW] SENT: ");
    Serial.println(message);
  } else {
    Serial.print("[ESP-NOW] SEND ERROR: ");
    Serial.println(result);
  }
}

// =====================================================
// COMMAND PROCESSING
// =====================================================

void processCommand(char *cmd) {

  if (cmd == nullptr) return;

  for (int i = 0; cmd[i]; i++) {

    if (cmd[i] == '\r' || cmd[i] == '\n')
      cmd[i] = '\0';

    if (cmd[i] >= 'a' && cmd[i] <= 'z')
      cmd[i] -= 32;
  }

  if (strlen(cmd) == 0) return;

  Serial.print("[COMMAND] ");
  Serial.println(cmd);

  if (strcmp(cmd, "PING") == 0) {

    sendToENV("PING");

  } else if (strcmp(cmd, "STATUS") == 0) {

    sendToENV("STATUS");

  } else if (strcmp(cmd, "DATA") == 0) {

    sendToENV("DATA");

  } else if (strcmp(cmd, "SCAN") == 0) {

    sendToENV("SCAN");

  } else if (strcmp(cmd, "HELP") == 0) {

    Serial.println();
    Serial.println("PING");
    Serial.println("STATUS");
    Serial.println("DATA");
    Serial.println("SCAN");
    Serial.println("HELP");

  } else {

    Serial.println("Unknown command.");
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("==============================================");
  Serial.println("       HYDROS ESP32 MASTER / OBC");
  Serial.println("       ESP32 DOIT DEVKIT V1");
  Serial.println("==============================================");

  WiFi.mode(WIFI_STA);

  delay(100);

  Serial.print("ACTUAL ESP32 MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.println("CONFIGURED MASTER MAC: 8C:94:DF:6D:86:F4");
  Serial.println("CONFIGURED ENV MAC   : 00:70:07:E2:22:E0");

  // ===================================================
  // ESP-NOW
  // ===================================================

  Serial.println();
  Serial.println("Initializing ESP-NOW...");

  if (esp_now_init() != ESP_OK) {

    Serial.println("ESP-NOW INIT FAILED");
    return;
  }

  Serial.println("ESP-NOW INIT OK");

  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  // ===================================================
  // ADD ENV PEER
  // ===================================================

  if (!addEnvPeer()) {

    Serial.println("ENV PEER ADD FAILED");

  } else {

    Serial.println("ENV PEER READY");
  }

  Serial.println();
  Serial.println("==============================================");
  Serial.println("          MASTER READY");
  Serial.println("==============================================");

  delay(500);

  sendToENV("MASTER:READY");
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  if (Serial.available()) {

    static char serialBuffer[64];
    static uint8_t index = 0;

    char c = Serial.read();

    if (c == '\n' || c == '\r') {

      if (index > 0) {

        serialBuffer[index] = '\0';

        processCommand(serialBuffer);

        index = 0;
      }

    } else {

      if (index < sizeof(serialBuffer) - 1) {
        serialBuffer[index++] = c;
      }
    }
  }

  delay(5);
}
```
