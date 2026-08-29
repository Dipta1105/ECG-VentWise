```cpp
#include <Arduino.h>
#include <Wire.h>
#include <DHT.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LIS3DH.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <esp_now.h>

// =====================================================
// HYDROS ENVIRONMENT SENSOR NODE
// BOARD: ESP32 DOIT DEVKIT V1
// =====================================================
//
// ENV MAC:
// 00:70:07:E2:22:E0
//
// MASTER MAC:
// 8C:94:DF:6D:86:F4
// =====================================================

// =====================================================
// PIN CONFIGURATION
// =====================================================

#define DHT_PIN       4
#define DHT_TYPE      DHT22

#define GPS_RX        16
#define GPS_TX        17

#define I2C_SDA       32
#define I2C_SCL       33

#define INDICATOR1    25
#define INDICATOR2    26

#define GPS_BAUD      9600
#define SERIAL_BAUD   115200

// =====================================================
// MASTER PEER MAC
// =====================================================

uint8_t MASTER_MAC[] = {
  0x8C, 0x94, 0xDF, 0x6D, 0x86, 0xF4
};

// =====================================================
// OBJECTS
// =====================================================

DHT dht(DHT_PIN, DHT_TYPE);

Adafruit_BMP280 bmp(&Wire);
Adafruit_LIS3DH lis = Adafruit_LIS3DH(&Wire);

TinyGPSPlus gps;
HardwareSerial GPS(2);

// =====================================================
// SENSOR DATA
// =====================================================

float temperature = 0;
float humidity = 0;

float pressure = 0;
float altitude = 0;

float accX = 0;
float accY = 0;
float accZ = 0;

double latitude = 0;
double longitude = 0;

uint32_t satellites = 0;

// =====================================================
// STATUS
// =====================================================

bool dhtOK = false;
bool bmpOK = false;
bool lisOK = false;
bool gpsOK = false;

bool espNowOK = false;

// =====================================================
// TIMERS
// =====================================================

unsigned long lastSensorRead = 0;
unsigned long lastSend = 0;
unsigned long lastLED = 0;

const unsigned long SENSOR_INTERVAL = 2000;
const unsigned long SEND_INTERVAL = 2000;

// =====================================================
// RECEIVE BUFFER
// =====================================================

volatile bool commandReady = false;

char commandBuffer[64];

// =====================================================
// SEND CALLBACK
// =====================================================

void onDataSent(
  const wifi_tx_info_t *info,
  esp_now_send_status_t status
) {
  // Keep callback lightweight.
}

// =====================================================
// RECEIVE CALLBACK
// =====================================================

void onDataRecv(
  const esp_now_recv_info_t *info,
  const uint8_t *data,
  int len
) {

  if (data == nullptr || len <= 0)
    return;

  if (len >= sizeof(commandBuffer))
    len = sizeof(commandBuffer) - 1;

  memcpy(commandBuffer, data, len);

  commandBuffer[len] = '\0';

  commandReady = true;
}

// =====================================================
// SEND MESSAGE
// =====================================================

void sendMessage(const char *msg) {

  if (!espNowOK || msg == nullptr)
    return;

  esp_now_send(
    MASTER_MAC,
    (const uint8_t *)msg,
    strlen(msg) + 1
  );
}

// =====================================================
// I2C SCANNER
// =====================================================

void scanI2C() {

  Serial.println();
  Serial.println("I2C SCAN");

  int count = 0;

  for (uint8_t address = 1; address < 127; address++) {

    Wire.beginTransmission(address);

    if (Wire.endTransmission() == 0) {

      Serial.print("Found: 0x");

      if (address < 16)
        Serial.print("0");

      Serial.println(address, HEX);

      count++;
    }
  }

  if (count == 0)
    Serial.println("No I2C devices found");
  else {
    Serial.print("I2C devices: ");
    Serial.println(count);
  }
}

// =====================================================
// SENSOR INITIALIZATION
// =====================================================

void initSensors() {

  Serial.println();
  Serial.println("=== SENSOR INITIALIZATION ===");

  // DHT22
  dht.begin();

  delay(500);

  Serial.println("DHT22 initialized");

  // BMP280
  if (bmp.begin(0x76)) {

    bmpOK = true;

    Serial.println("BMP280 OK @ 0x76");

  } else if (bmp.begin(0x77)) {

    bmpOK = true;

    Serial.println("BMP280 OK @ 0x77");

  } else {

    bmpOK = false;

    Serial.println("BMP280 NOT FOUND");
  }

  // LIS3DH
  if (lis.begin(0x19)) {

    lisOK = true;

    lis.setRange(LIS3DH_RANGE_4_G);

    Serial.println("LIS3DH OK @ 0x19");

  } else if (lis.begin(0x18)) {

    lisOK = true;

    lis.setRange(LIS3DH_RANGE_4_G);

    Serial.println("LIS3DH OK @ 0x18");

  } else {

    lisOK = false;

    Serial.println("LIS3DH NOT FOUND");
  }

  Serial.println("=============================");
}

// =====================================================
// READ DHT
// =====================================================

void readDHT() {

  float t = dht.readTemperature();
  float h = dht.readHumidity();

  if (!isnan(t) && !isnan(h)) {

    temperature = t;
    humidity = h;

    dhtOK = true;

  } else {

    dhtOK = false;
  }
}

// =====================================================
// READ BMP280
// =====================================================

void readBMP() {

  if (!bmpOK)
    return;

  float p = bmp.readPressure();

  if (isnan(p) || p <= 0) {

    bmpOK = false;

    return;
  }

  pressure = p / 100.0;

  altitude = bmp.readAltitude(1013.25);
}

// =====================================================
// READ LIS3DH
// =====================================================

void readLIS() {

  if (!lisOK)
    return;

  lis.read();

  accX = lis.x_g * 9.81;
  accY = lis.y_g * 9.81;
  accZ = lis.z_g * 9.81;
}

// =====================================================
// GPS
// =====================================================

void processGPS() {

  while (GPS.available()) {
    gps.encode(GPS.read());
  }

  if (gps.location.isValid()) {

    gpsOK = true;

    latitude = gps.location.lat();
    longitude = gps.location.lng();

  } else {

    gpsOK = false;
  }

  satellites = gps.satellites.value();
}

// =====================================================
// READ ALL SENSORS
// =====================================================

void readSensors() {

  readDHT();
  readBMP();
  readLIS();
  processGPS();
}

// =====================================================
// PRINT DATA
// =====================================================

void printData() {

  Serial.println();
  Serial.println("========== ENV DATA ==========");

  Serial.print("Temperature : ");

  if (dhtOK)
    Serial.print(temperature, 2);
  else
    Serial.print("ERROR");

  Serial.println(" C");

  Serial.print("Humidity    : ");

  if (dhtOK)
    Serial.print(humidity, 2);
  else
    Serial.print("ERROR");

  Serial.println(" %");

  Serial.print("Pressure    : ");

  if (bmpOK)
    Serial.print(pressure, 2);
  else
    Serial.print("ERROR");

  Serial.println(" hPa");

  Serial.print("Altitude    : ");

  if (bmpOK)
    Serial.print(altitude, 2);
  else
    Serial.print("ERROR");

  Serial.println(" m");

  Serial.print("Accel X     : ");
  Serial.print(accX, 3);
  Serial.println(" m/s2");

  Serial.print("Accel Y     : ");
  Serial.print(accY, 3);
  Serial.println(" m/s2");

  Serial.print("Accel Z     : ");
  Serial.print(accZ, 3);
  Serial.println(" m/s2");

  Serial.print("GPS         : ");

  if (gpsOK) {

    Serial.print(latitude, 6);
    Serial.print(", ");
    Serial.println(longitude, 6);

  } else {

    Serial.println("NO FIX");
  }

  Serial.print("Satellites  : ");
  Serial.println(satellites);

  Serial.println("==============================");
}

// =====================================================
// SEND TELEMETRY
// =====================================================

void sendTelemetry() {

  if (!espNowOK)
    return;

  char message[220];

  snprintf(
    message,
    sizeof(message),

    "DATA,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.6f,%.6f,%lu,%d,%d,%d,%d",

    temperature,
    humidity,
    pressure,
    altitude,

    accX,
    accY,
    accZ,

    latitude,
    longitude,

    (unsigned long)satellites,

    dhtOK ? 1 : 0,
    bmpOK ? 1 : 0,
    lisOK ? 1 : 0,
    gpsOK ? 1 : 0
  );

  sendMessage(message);

  Serial.println("[ESP-NOW] Telemetry sent");
}

// =====================================================
// SEND STATUS
// =====================================================

void sendStatus() {

  char message[100];

  snprintf(
    message,
    sizeof(message),

    "STATUS,%d,%d,%d,%d,%lu",

    dhtOK ? 1 : 0,
    bmpOK ? 1 : 0,
    lisOK ? 1 : 0,
    gpsOK ? 1 : 0,

    (unsigned long)satellites
  );

  sendMessage(message);
}

// =====================================================
// PRINT STATUS
// =====================================================

void printStatus() {

  Serial.println();
  Serial.println("========== ENV STATUS ==========");

  Serial.print("BOARD      : ");
  Serial.println("ESP32 DOIT DEVKIT V1");

  Serial.print("ACTUAL MAC : ");
  Serial.println(WiFi.macAddress());

  Serial.print("MASTER MAC : ");
  Serial.println("8C:94:DF:6D:86:F4");

  Serial.print("DHT22      : ");
  Serial.println(dhtOK ? "OK" : "FAIL");

  Serial.print("BMP280     : ");
  Serial.println(bmpOK ? "OK" : "FAIL");

  Serial.print("LIS3DH     : ");
  Serial.println(lisOK ? "OK" : "FAIL");

  Serial.print("GNSS       : ");
  Serial.println(gpsOK ? "FIX" : "NO FIX");

  Serial.print("Satellites : ");
  Serial.println(satellites);

  Serial.print("ESP-NOW    : ");
  Serial.println(espNowOK ? "OK" : "FAIL");

  Serial.println("===============================");
}

// =====================================================
// PROCESS COMMAND
// =====================================================

void processCommand(char *cmd) {

  if (cmd == nullptr)
    return;

  for (int i = 0; cmd[i]; i++) {

    if (cmd[i] == '\r' || cmd[i] == '\n')
      cmd[i] = '\0';

    if (cmd[i] >= 'a' && cmd[i] <= 'z')
      cmd[i] -= 32;
  }

  Serial.print("[COMMAND] ");
  Serial.println(cmd);

  if (strcmp(cmd, "PING") == 0) {

    Serial.println("PONG");
    sendMessage("PONG");

  } else if (strcmp(cmd, "MASTER:READY") == 0) {

    Serial.println("[MASTER] MASTER IS READY");
    sendMessage("SLAVE:READY");

  } else if (strcmp(cmd, "STATUS") == 0) {

    printStatus();
    sendStatus();

  } else if (strcmp(cmd, "DATA") == 0) {

    readSensors();
    printData();
    sendTelemetry();

  } else if (strcmp(cmd, "SCAN") == 0) {

    scanI2C();

  } else if (strcmp(cmd, "HELP") == 0) {

    Serial.println();
    Serial.println("PING");
    Serial.println("STATUS");
    Serial.println("DATA");
    Serial.println("SCAN");
    Serial.println("HELP");

  } else {

    Serial.println("Unknown command");
  }
}

// =====================================================
// PROCESS ESP-NOW COMMAND
// =====================================================

void processReceivedCommand() {

  if (!commandReady)
    return;

  char localCommand[64];

  noInterrupts();

  memcpy(
    localCommand,
    commandBuffer,
    sizeof(localCommand)
  );

  commandReady = false;

  interrupts();

  localCommand[sizeof(localCommand) - 1] = '\0';

  processCommand(localCommand);
}

// =====================================================
// INITIALIZE ESP-NOW
// =====================================================

bool initESPNow() {

  Serial.println();
  Serial.println("=== ESP-NOW INITIALIZATION ===");

  WiFi.mode(WIFI_STA);

  delay(100);

  Serial.print("ENV MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.println("MASTER PEER MAC: 8C:94:DF:6D:86:F4");

  if (esp_now_init() != ESP_OK) {

    Serial.println("ESP-NOW INIT FAILED");

    return false;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peer = {};

  memcpy(
    peer.peer_addr,
    MASTER_MAC,
    6
  );

  peer.channel = 0;
  peer.encrypt = false;

  if (!esp_now_is_peer_exist(MASTER_MAC)) {

    if (esp_now_add_peer(&peer) != ESP_OK) {

      Serial.println("MASTER PEER ADD FAILED");

      return false;
    }
  }

  Serial.println("MASTER PEER ADDED");
  Serial.println("ESP-NOW READY");

  return true;
}

// =====================================================
// LED STATUS
// =====================================================

void updateLEDs() {

  if (dhtOK && bmpOK && lisOK) {

    digitalWrite(
      INDICATOR1,
      HIGH
    );

  } else {

    digitalWrite(
      INDICATOR1,
      millis() % 1000 < 500
    );
  }

  if (millis() - lastLED >= 500) {

    lastLED = millis();

    digitalWrite(
      INDICATOR2,
      !digitalRead(INDICATOR2)
    );
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(SERIAL_BAUD);

  delay(1500);

  Serial.println();
  Serial.println();

  Serial.println("================================");
  Serial.println("   HYDROS ENV ESP32");
  Serial.println("   DOIT ESP32 DEVKIT V1");
  Serial.println("================================");

  pinMode(INDICATOR1, OUTPUT);
  pinMode(INDICATOR2, OUTPUT);

  digitalWrite(INDICATOR1, LOW);
  digitalWrite(INDICATOR2, LOW);

  // ===================================================
  // I2C
  // ===================================================

  Serial.println("Starting I2C...");

  Wire.begin(
    I2C_SDA,
    I2C_SCL
  );

  Wire.setClock(100000);

  delay(100);

  Serial.print("SDA: GPIO ");
  Serial.println(I2C_SDA);

  Serial.print("SCL: GPIO ");
  Serial.println(I2C_SCL);

  scanI2C();

  // ===================================================
  // SENSORS
  // ===================================================

  initSensors();

  // ===================================================
  // GNSS
  // ===================================================

  Serial.println("Starting GNSS...");

  GPS.begin(
    GPS_BAUD,
    SERIAL_8N1,
    GPS_RX,
    GPS_TX
  );

  // ===================================================
  // ESP-NOW
  // ===================================================

  espNowOK = initESPNow();

  // ===================================================
  // COMPLETE
  // ===================================================

  Serial.println();
  Serial.println("================================");
  Serial.println("     ENV INITIALIZATION DONE");
  Serial.println("================================");

  printStatus();

  if (espNowOK) {

    delay(100);

    sendMessage("SLAVE:BOOT_COMPLETE");

    delay(100);

    sendMessage("SLAVE:READY");
  }
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  processGPS();

  processReceivedCommand();

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

  unsigned long now = millis();

  if (now - lastSensorRead >= SENSOR_INTERVAL) {

    lastSensorRead = now;

    readSensors();

    printData();
  }

  if (now - lastSend >= SEND_INTERVAL) {

    lastSend = now;

    sendTelemetry();
  }

  updateLEDs();

  delay(5);
}
```
