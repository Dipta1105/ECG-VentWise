#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "MAX30102.h"
#include "Pulse.h"

// =====================================================
// HYDROS MASTER / HEALTH SENSOR NODE
// BOARD: ESP32 DOIT DEVKIT V1
// =====================================================
//
// MASTER ESP32 DEVKIT V1 MAC:
// 8C:94:DF:6D:86:F4
//
// ESP-NOW MAC:
// { 0x8C, 0x94, 0xDF, 0x6D, 0x86, 0xF4 }
//
// =====================================================


// =====================================================
// MASTER MAC ADDRESS
// =====================================================
// This is the physical MAC address of the ESP32.
// It is NOT assigned manually; the ESP32 hardware already
// uses this MAC address.
//
// Provided MAC:
// 8C:94:DF:6D:86:F4
//
uint8_t MASTER_MAC[] = {
  0x8C, 0x94, 0xDF, 0x6D, 0x86, 0xF4
};


// =====================================================
// MAX30102
// =====================================================
MAX30102 sensor;
Pulse pulseIR;
Pulse pulseRed;
MAFilter bpm;


// =====================================================
// LIS3DH
// =====================================================
Adafruit_LIS3DH lis = Adafruit_LIS3DH();


// =====================================================
// TEMPERATURE DS18B20
// =====================================================
#define TEMP_PIN 16

OneWire oneWire(TEMP_PIN);
DallasTemperature tempSensor(&oneWire);

// Temperature is now stored in Fahrenheit
float temperatureF = 0.0;


// =====================================================
// SPO2 TABLE
// =====================================================
const uint8_t spo2_table[184] = {
  95,95,95,96,96,96,97,97,97,97,97,98,98,98,98,98,98,98,98,99,
  99,99,99,99,99,99,99,100,100,100,100,100,100,100,100,100,100,100,
  100,100,100,100,100,100,100,100,99,99,99,99,99,98,98,98,98,98,
  97,97,97,97,96,96,96,96,95,95,95,94,94,94,93,93,93,92,92,92,
  91,91,90,90,89,89,89,88,88,87,87,86,86,85,85,84,84,83,82,82,
  81,81,80,80,79,78,78,77,76,76,75,74,74,73,72,72,71,70,69,69,
  68,67,66,66,65,64,63,62,62,61,60,59,58,57,56,56,55,54,53,52,
  51,50,49,48,47,46,45,44,43,42,41,40,39,38,37,36,35,34,33,31,
  30,29,28,27,26,25,23,22,21,20,19,17,16,15,14,12,11,10,9,7,6,5,3,2,1
};


// =====================================================
// ESP-NOW DATA PACKET
// =====================================================
typedef struct __attribute__((packed))
{
  int16_t heartRate;
  int16_t spo2;
  uint8_t fingerDetected;
  uint8_t fallDetected;
  int16_t accel;
  uint8_t panicPressed;

  // Temperature in Fahrenheit
  float temperatureF;

} SensorData;


SensorData dataToSend;


// =====================================================
// PIN DEFINITIONS - ESP32 DEVKIT V1
// =====================================================
#define PIN_PANIC_BUTTON  0
#define PIN_LED_COMM      14
#define PIN_LED_SENSOR    12
#define PIN_LED_ALERT     13
#define PIN_BUZZER        15


// =====================================================
// I2C PINS - ESP32 DOIT DEVKIT V1
// =====================================================
#define I2C_SDA 21
#define I2C_SCL 22


// =====================================================
// TIMING CONSTANTS
// =====================================================
const unsigned long ALERT_BUZZER_TIME   = 10000;
const unsigned long COMM_LED_PULSE_TIME = 200;
const unsigned long COMM_TIMEOUT        = 3000;
const unsigned long PANIC_HOLD_TIME     = 5000;
const unsigned long FALL_HOLD_TIME      = 5000;
const unsigned long DEBOUNCE_TIME       = 50;


// =====================================================
// STATE VARIABLES
// =====================================================

// --- Panic button ---
bool panicActive       = false;
unsigned long panicStartTime   = 0;
bool lastButtonState   = HIGH;
unsigned long lastDebounceTime = 0;


// --- Buzzer ---
bool buzzerActive      = false;
unsigned long buzzerStartTime = 0;


// --- Communication LED ---
bool commLedOn         = false;
unsigned long commLedOnTime   = 0;
unsigned long lastSuccessSend = 0;


// --- Fall ---
bool fallDetected      = false;
bool freeFallDetected  = false;
unsigned long freeFallTime = 0;
unsigned long fallTime     = 0;


// --- Alert LED ---
unsigned long lastAlertBlink = 0;
bool alertLedState = false;


// =====================================================
// HEALTH VARIABLES
// =====================================================
int16_t beatAvg = 0;
int16_t SPO2    = 0;
int     SPO2f   = 0;

long lastBeat = 0;
unsigned long lastPrint = 0;

bool fingerPresent = false;


// =====================================================
// RECEIVER MAC ADDRESS
// =====================================================
// OLED / Receiver ESP MAC
//
// KEEP THIS AS THE MAC ADDRESS OF THE RECEIVER.
//
// Current receiver MAC from original code:
// 50:02:91:51:EB:88
//
uint8_t receiverMAC[] = {
  0x50, 0x02, 0x91, 0x51, 0xEB, 0x88
};


// =====================================================
// ESP-NOW SEND CALLBACK
// =====================================================
#if ESP_ARDUINO_VERSION_MAJOR >= 3

void OnDataSent(const wifi_tx_info_t *info,
                esp_now_send_status_t status)
{
  if (status == ESP_NOW_SEND_SUCCESS)
  {
    digitalWrite(PIN_LED_COMM, HIGH);

    commLedOn       = true;
    commLedOnTime   = millis();
    lastSuccessSend = millis();

    Serial.println("ESP-NOW Send: OK");
  }
  else
  {
    digitalWrite(PIN_LED_COMM, LOW);

    commLedOn = false;

    Serial.println("ESP-NOW Send: FAILED");
  }
}

#else

void OnDataSent(const uint8_t *mac_addr,
                esp_now_send_status_t status)
{
  if (status == ESP_NOW_SEND_SUCCESS)
  {
    digitalWrite(PIN_LED_COMM, HIGH);

    commLedOn       = true;
    commLedOnTime   = millis();
    lastSuccessSend = millis();

    Serial.println("ESP-NOW Send: OK");
  }
  else
  {
    digitalWrite(PIN_LED_COMM, LOW);

    commLedOn = false;

    Serial.println("ESP-NOW Send: FAILED");
  }
}

#endif


// =====================================================
// STARTUP BEEP SEQUENCE
// =====================================================
void startupBeeps()
{
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(PIN_BUZZER, HIGH);
    delay(150);

    digitalWrite(PIN_BUZZER, LOW);
    delay(150);
  }
}


// =====================================================
// START EMERGENCY BUZZER
// =====================================================
void startAlertBuzzer()
{
  if (!buzzerActive)
  {
    digitalWrite(PIN_BUZZER, HIGH);

    buzzerActive    = true;
    buzzerStartTime = millis();
  }
}


// =====================================================
// SETUP
// =====================================================
void setup()
{
  Serial.begin(115200);
  delay(1000);

  // ===================================================
  // GPIO
  // ===================================================
  pinMode(PIN_PANIC_BUTTON, INPUT_PULLUP);

  pinMode(PIN_LED_COMM,   OUTPUT);
  pinMode(PIN_LED_SENSOR, OUTPUT);
  pinMode(PIN_LED_ALERT,  OUTPUT);
  pinMode(PIN_BUZZER,     OUTPUT);


  // ===================================================
  // ALL OUTPUTS OFF
  // ===================================================
  digitalWrite(PIN_LED_COMM,   LOW);
  digitalWrite(PIN_LED_SENSOR, LOW);
  digitalWrite(PIN_LED_ALERT,  LOW);
  digitalWrite(PIN_BUZZER,     LOW);


  // ===================================================
  // I2C
  // ESP32 DEVKIT V1
  // SDA = GPIO21
  // SCL = GPIO22
  // ===================================================
  Wire.begin(I2C_SDA, I2C_SCL);


  Serial.println();
  Serial.println("==============================================");
  Serial.println(" HYDROS MASTER - ESP32 DOIT DEVKIT V1");
  Serial.println("==============================================");

  Serial.print("MASTER MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.println("Expected MAC:");
  Serial.println("8C:94:DF:6D:86:F4");

  Serial.println();


  // =================================================
  // MAX30102
  // =================================================
  if (!sensor.begin())
  {
    Serial.println("ERROR: MAX30102 not found");

    while (1)
    {
      digitalWrite(
        PIN_LED_SENSOR,
        !digitalRead(PIN_LED_SENSOR)
      );

      delay(200);
    }
  }

  sensor.setup();

  Serial.println("MAX30102 OK");


  // =================================================
  // LIS3DH
  // =================================================
  if (!lis.begin(0x19))
  {
    Serial.println("ERROR: LIS3DH not found");

    while (1)
    {
      digitalWrite(
        PIN_LED_SENSOR,
        !digitalRead(PIN_LED_SENSOR)
      );

      delay(200);
    }
  }

  lis.setRange(LIS3DH_RANGE_4_G);

  Serial.println("LIS3DH OK");


  // =================================================
  // DS18B20
  // =================================================
  tempSensor.begin();

  if (tempSensor.getDeviceCount() == 0)
  {
    Serial.println("WARNING: DS18B20 not found on GPIO16");

    temperatureF = -127.0;
  }
  else
  {
    Serial.println("DS18B20 OK");

    Serial.print("Temperature sensors found: ");
    Serial.println(tempSensor.getDeviceCount());
  }


  // Both original sensors OK
  digitalWrite(PIN_LED_SENSOR, HIGH);


  // =================================================
  // ESP-NOW INITIALIZATION
  // =================================================
  WiFi.mode(WIFI_STA);

  delay(100);

  Serial.println();
  Serial.print("ESP32 STA MAC: ");
  Serial.println(WiFi.macAddress());


  // Verify MAC
  if (WiFi.macAddress() == "8C:94:DF:6D:86:F4")
  {
    Serial.println("MASTER MAC VERIFIED");
  }
  else
  {
    Serial.println("WARNING: MAC DOES NOT MATCH EXPECTED MAC");
  }


  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ERROR: ESP-NOW init failed");

    while (1)
    {
      digitalWrite(PIN_LED_COMM, !digitalRead(PIN_LED_COMM));
      delay(200);
    }
  }


  // Register send callback
  esp_now_register_send_cb(OnDataSent);


  // =================================================
  // ADD RECEIVER PEER
  // =================================================
  esp_now_peer_info_t peerInfo = {};

  memcpy(
    peerInfo.peer_addr,
    receiverMAC,
    6
  );

  peerInfo.channel = 0;
  peerInfo.encrypt = false;


  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("ERROR: Failed to add receiver peer");

    while (1)
    {
      digitalWrite(PIN_LED_COMM, !digitalRead(PIN_LED_COMM));
      delay(300);
    }
  }


  Serial.println("ESP-NOW OK");

  Serial.println();
  Serial.println("==============================================");
  Serial.println(" SYSTEM READY");
  Serial.println("==============================================");
  Serial.println();


  // =================================================
  // STARTUP BEEP
  // =================================================
  startupBeeps();
}


// =====================================================
// LOOP
// =====================================================
void loop()
{
  unsigned long now = millis();


  // =================================================
  // PULSE OX
  // =================================================
  readPulseOx();


  // =================================================
  // FALL DETECTION
  // =================================================
  detectFall();


  // =================================================
  // TEMPERATURE READING
  // =================================================
  tempSensor.requestTemperatures();

  float tempReadingC =
    tempSensor.getTempCByIndex(0);


  if (tempReadingC != DEVICE_DISCONNECTED_C)
  {
    // ===============================================
    // CELSIUS -> FAHRENHEIT
    //
    // °F = (°C × 9/5) + 32
    // ===============================================
    temperatureF =
      (tempReadingC * 9.0 / 5.0) + 32.0;
  }
  else
  {
    temperatureF = -127.0;
  }


  // =================================================
  // PANIC BUTTON
  // =================================================
  bool rawButton = digitalRead(PIN_PANIC_BUTTON);


  if (rawButton != lastButtonState)
  {
    lastDebounceTime = now;
  }


  if ((now - lastDebounceTime) > DEBOUNCE_TIME)
  {
    if (rawButton == LOW && !panicActive)
    {
      panicActive    = true;
      panicStartTime = now;

      Serial.println("[PANIC] Button pressed");
    }
  }


  lastButtonState = rawButton;


  // =================================================
  // PANIC AUTO-RESET
  // =================================================
  if (
    panicActive &&
    (now - panicStartTime > PANIC_HOLD_TIME)
  )
  {
    panicActive = false;

    Serial.println("[PANIC] Cleared");
  }


  // =================================================
  // COMM LED AUTO-OFF
  // =================================================
  if (
    commLedOn &&
    (now - commLedOnTime >= COMM_LED_PULSE_TIME)
  )
  {
    digitalWrite(PIN_LED_COMM, LOW);

    commLedOn = false;
  }


  // =================================================
  // COMMUNICATION LINK LOST
  // =================================================
  if (
    lastSuccessSend > 0 &&
    (now - lastSuccessSend > COMM_TIMEOUT)
  )
  {
    digitalWrite(PIN_LED_COMM, LOW);

    commLedOn = false;
  }


  // =================================================
  // ALERT LED + BUZZER
  // =================================================
  bool emergencyActive =
    fallDetected || panicActive;


  if (emergencyActive)
  {
    startAlertBuzzer();


    if (now - lastAlertBlink >= 250)
    {
      alertLedState = !alertLedState;

      digitalWrite(
        PIN_LED_ALERT,
        alertLedState ? HIGH : LOW
      );

      lastAlertBlink = now;
    }
  }
  else
  {
    digitalWrite(PIN_LED_ALERT, LOW);

    alertLedState = false;
  }


  // =================================================
  // BUZZER AUTO-OFF
  // =================================================
  if (
    buzzerActive &&
    (now - buzzerStartTime >= ALERT_BUZZER_TIME)
  )
  {
    digitalWrite(PIN_BUZZER, LOW);

    buzzerActive = false;
  }


  // =================================================
  // 1-SECOND TRANSMIT CYCLE
  // =================================================
  if (now - lastPrint >= 1000)
  {
    lastPrint = now;


    // =================================================
    // BUILD ESP-NOW PACKET
    // =================================================
    dataToSend.heartRate =
      beatAvg;

    dataToSend.spo2 =
      SPO2;

    dataToSend.fingerDetected =
      fingerPresent ? 1 : 0;

    dataToSend.fallDetected =
      fallDetected ? 1 : 0;

    dataToSend.panicPressed =
      panicActive ? 1 : 0;

    // Acceleration is set inside detectFall()

    dataToSend.temperatureF =
      temperatureF;


    // =================================================
    // SEND ESP-NOW PACKET
    // =================================================
    esp_err_t result = esp_now_send(
      receiverMAC,
      (uint8_t *)&dataToSend,
      sizeof(dataToSend)
    );


    if (result != ESP_OK)
    {
      Serial.print("ESP-NOW ERROR: ");
      Serial.println(result);
    }


    // =================================================
    // SERIAL MONITOR
    // =================================================
    Serial.println();
    Serial.println("========== HEALTH DATA ==========");


    if (fingerPresent)
    {
      Serial.print("Heart Rate : ");
      Serial.print(beatAvg);
      Serial.println(" BPM");


      Serial.print("SpO2       : ");
      Serial.print(SPO2);
      Serial.println(" %");
    }
    else
    {
      Serial.println("Finger     : Not Detected");
    }


    // =================================================
    // TEMPERATURE - FAHRENHEIT
    // =================================================
    Serial.print("Temperature: ");


    if (temperatureF == -127.0)
    {
      Serial.println("Sensor Not Found");
    }
    else
    {
      Serial.print(temperatureF, 2);
      Serial.println(" °F");
    }


    // =================================================
    // ACCELERATION
    // =================================================
    Serial.print("Accel      : ");
    Serial.print(dataToSend.accel / 10.0);
    Serial.println(" m/s2");


    // =================================================
    // FALL STATUS
    // =================================================
    Serial.print("Fall Status: ");

    Serial.println(
      fallDetected ? "DETECTED" : "NORMAL"
    );


    // =================================================
    // PANIC STATUS
    // =================================================
    Serial.print("Panic      : ");

    Serial.println(
      panicActive ? "ACTIVE" : "OFF"
    );


    Serial.println("=================================");
  }
}


// =====================================================
// readPulseOx
// =====================================================
void readPulseOx()
{
  sensor.check();


  if (!sensor.available())
    return;


  uint32_t irValue =
    sensor.getIR();

  uint32_t redValue =
    sensor.getRed();


  sensor.nextSample();


  // Finger detection
  if (irValue < 5000)
  {
    fingerPresent = false;
    return;
  }


  fingerPresent = true;


  int16_t IR_signal =
    pulseIR.dc_filter(irValue);

  int16_t Red_signal =
    pulseRed.dc_filter(redValue);


  bool beatIR =
    pulseIR.isBeat(
      pulseIR.ma_filter(IR_signal)
    );


  bool beatRed =
    pulseRed.isBeat(
      pulseRed.ma_filter(Red_signal)
    );


  if (beatIR)
  {
    long now = millis();


    if (lastBeat > 0)
    {
      long btpm =
        60000 / (now - lastBeat);


      if (btpm > 40 && btpm < 200)
      {
        beatAvg =
          bpm.filter(btpm);
      }
    }


    lastBeat = now;


    long numerator =
      (pulseRed.avgAC() *
       pulseIR.avgDC()) / 256;


    long denominator =
      (pulseRed.avgDC() *
       pulseIR.avgAC()) / 256;


    int RX100 =
      (denominator > 0)
      ? (numerator * 100) / denominator
      : 999;


    SPO2f =
      (10400 - RX100 * 17 + 50) / 100;


    if (RX100 >= 0 && RX100 < 184)
    {
      SPO2 =
        spo2_table[RX100];
    }
  }
}


// =====================================================
// detectFall
// =====================================================
void detectFall()
{
  sensors_event_t event;


  lis.getEvent(&event);


  float ax =
    event.acceleration.x;

  float ay =
    event.acceleration.y;

  float az =
    event.acceleration.z;


  float totalAccel =
    sqrt(
      ax * ax +
      ay * ay +
      az * az
    );


  // Store acceleration ×10
  dataToSend.accel =
    (int16_t)(totalAccel * 10.0);


  // =================================================
  // FREE-FALL
  // =================================================
  if (totalAccel < 4.0)
  {
    freeFallDetected = true;

    freeFallTime = millis();
  }


  // =================================================
  // IMPACT AFTER FREE-FALL
  // =================================================
  if (
    freeFallDetected &&
    totalAccel > 20.0 &&
    millis() - freeFallTime < 1000
  )
  {
    fallDetected = true;

    freeFallDetected = false;

    fallTime = millis();

    Serial.println("[FALL] Fall detected!");
  }


  // =================================================
  // AUTO-RESET FALL
  // =================================================
  if (
    fallDetected &&
    (millis() - fallTime > FALL_HOLD_TIME)
  )
  {
    fallDetected = false;

    fallTime = 0;

    Serial.println("[FALL] Cleared");
  }
}
