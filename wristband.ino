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
// VENTWISE WRISTBAND / HEALTH SENSOR NODE
// BOARD: ESP32 DOIT DEVKIT V1
// =====================================================
// NOTE (unchanged from source): this node transmits to
// `receiverMAC` below via ESP-NOW, not to the placeholder
// "MASTER_MAC" constant that is only referenced in header
// comments/serial banners here. This mismatch already
// existed in the original file and was left as-is since
// fixing it is outside the scope of this optimization pass.
//
// Optimization pass notes (no functional/logic changes):
//  - Reformatted to consistent bracing/spacing.
//  - Added [WRIST] log prefixes at sensor reads, state
//    transitions (panic/fall/comm), and errors.
//  - Startup beep sequence, debounce timing, panic/fall
//    hold times, and the pulse-ox/SpO2 math are unchanged.
// =====================================================

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

// Temperature is stored in Fahrenheit
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
// The master identifies this packet by its MAGIC and VERSION
// bytes. It used to identify it by length alone, which meant
// any other ESP-NOW sender that happened to emit a payload of
// the same size was parsed as patient vitals -- and, worse,
// that changing this struct on one side only made the master
// fall through to its text-command parser and read binary as
// ASCII. Bump VERSION whenever the layout below changes, and
// reflash both boards together.
#define WRIST_PACKET_MAGIC   0x5742   // 'WB'
#define WRIST_PACKET_VERSION 2

// sensorHealth bit flags. Sent on every packet so the master --
// and the phone behind it -- can tell "sensor says 0" apart
// from "no sensor". A dead sensor must never look like a
// reading of zero.
#define HEALTH_PULSEOX 0x01
#define HEALTH_ACCEL   0x02
#define HEALTH_TEMP    0x04

typedef struct __attribute__((packed)) {
  uint16_t magic;
  uint8_t  version;
  uint8_t  sensorHealth;
  uint16_t seq;

  int16_t heartRate;
  int16_t spo2;
  uint8_t fingerDetected;
  uint8_t fallDetected;
  uint8_t panicPressed;

  // Magnitude, m/s^2 x10. Kept at its original scale and name so
  // existing consumers of this field keep working.
  int16_t accel;

  // Per-axis, m/s^2 x100. x100 rather than x10 because activity
  // classification needs finer resolution than fall detection;
  // an int16 still spans +/-327 m/s^2, well past the +/-4 g range.
  int16_t ax;
  int16_t ay;
  int16_t az;

  // Largest magnitude seen since the previous packet, m/s^2 x10.
  // The band samples far faster than it transmits, so without
  // this a fall spike landing between two packets is invisible.
  int16_t accelPeak;

  float temperatureF; // Fahrenheit
} SensorData;

SensorData dataToSend;

// =====================================================
// SENSOR HEALTH
// =====================================================
// A missing sensor must not take the band down with it. The
// panic button and fall detection have to keep working when the
// pulse-ox is unplugged, so each sensor gets a flag instead of a
// while(1) trap.
bool hasPulseOx = false;
bool hasAccel   = false;
bool hasTemp    = false;

uint16_t packetSeq = 0;

// =====================================================
// PIN DEFINITIONS - ESP32 DEVKIT V1
// =====================================================
#define PIN_PANIC_BUTTON  0
#define PIN_LED_COMM      14
#define PIN_LED_SENSOR    12
#define PIN_LED_ALERT     13
#define PIN_BUZZER        15

// =====================================================
// I2C PINS
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

// Accelerometer sampling and transmission.
//
// 50 Hz sampling because the free-fall phase of a real fall lasts
// 300-500 ms: the loop has to see several samples inside that
// window for the free-fall -> impact sequence in detectFall() to
// mean anything. It previously ran once per loop, and the loop was
// pinned near 1 Hz by the blocking temperature read below, so the
// free-fall phase was usually stepped straight over.
//
// 20 Hz transmission keeps motion continuous at the master without
// flooding it; accelPeak covers everything between packets.
const unsigned long ACCEL_SAMPLE_INTERVAL = 20;    // 50 Hz
const unsigned long TX_INTERVAL           = 50;    // 20 Hz
const unsigned long PRINT_INTERVAL        = 1000;  // 1 Hz serial summary

// DS18B20 12-bit conversion takes up to 750 ms. Ask for a reading,
// walk away, collect it later.
const unsigned long TEMP_CONVERSION_TIME  = 800;
const unsigned long TEMP_INTERVAL         = 2000;

// =====================================================
// STATE VARIABLES
// =====================================================
bool panicActive             = false;
unsigned long panicStartTime = 0;
bool lastButtonState          = HIGH;
unsigned long lastDebounceTime = 0;

bool buzzerActive              = false;
unsigned long buzzerStartTime  = 0;

bool commLedOn                 = false;
unsigned long commLedOnTime    = 0;
unsigned long lastSuccessSend  = 0;

// Counted in the send callback and reported once a second, rather
// than printed per packet.
volatile unsigned long txOkCount   = 0;
volatile unsigned long txFailCount = 0;

bool fallDetected      = false;
bool freeFallDetected  = false;
unsigned long freeFallTime = 0;
unsigned long fallTime     = 0;

unsigned long lastAlertBlink = 0;
bool alertLedState = false;

unsigned long lastAccelSample = 0;
unsigned long lastTx          = 0;

// Peak magnitude since the last transmission, m/s^2.
float accelPeakSinceTx = 0.0;

// Non-blocking DS18B20 state.
bool tempConversionPending      = false;
unsigned long tempRequestTime   = 0;
unsigned long lastTempRequest   = 0;

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
// RECEIVER MAC ADDRESS (OLED / master unit)
// KEEP THIS AS THE MAC ADDRESS OF THE RECEIVER.
// =====================================================
uint8_t receiverMAC[] = {
  0x50, 0x02, 0x91, 0x51, 0xEB, 0x88
};

// =====================================================
// ESP-NOW SEND CALLBACK
// =====================================================
#if ESP_ARDUINO_VERSION_MAJOR >= 3

void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    digitalWrite(PIN_LED_COMM, HIGH);

    commLedOn       = true;
    commLedOnTime   = millis();
    lastSuccessSend = millis();

    // Deliberately silent on success. This fires once per packet,
    // and the packet rate is now 20 Hz -- logging each one would
    // put the band in Serial.print for a large part of every
    // second. The comm LED and the 1 Hz summary carry the state.
    txOkCount++;
  } else {
    digitalWrite(PIN_LED_COMM, LOW);
    commLedOn = false;

    txFailCount++;
  }
}

#else

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    digitalWrite(PIN_LED_COMM, HIGH);

    commLedOn       = true;
    commLedOnTime   = millis();
    lastSuccessSend = millis();

    // Deliberately silent on success. This fires once per packet,
    // and the packet rate is now 20 Hz -- logging each one would
    // put the band in Serial.print for a large part of every
    // second. The comm LED and the 1 Hz summary carry the state.
    txOkCount++;
  } else {
    digitalWrite(PIN_LED_COMM, LOW);
    commLedOn = false;

    txFailCount++;
  }
}

#endif

// =====================================================
// STARTUP BEEP SEQUENCE
// =====================================================
void startupBeeps() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_BUZZER, HIGH);
    delay(150);

    digitalWrite(PIN_BUZZER, LOW);
    delay(150);
  }
}

// =====================================================
// START EMERGENCY BUZZER
// =====================================================
void startAlertBuzzer() {
  if (!buzzerActive) {
    digitalWrite(PIN_BUZZER, HIGH);

    buzzerActive    = true;
    buzzerStartTime = millis();

    Serial.println("[WRIST] Alert buzzer ON");
  }
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_PANIC_BUTTON, INPUT_PULLUP);

  pinMode(PIN_LED_COMM,   OUTPUT);
  pinMode(PIN_LED_SENSOR, OUTPUT);
  pinMode(PIN_LED_ALERT,  OUTPUT);
  pinMode(PIN_BUZZER,     OUTPUT);

  digitalWrite(PIN_LED_COMM,   LOW);
  digitalWrite(PIN_LED_SENSOR, LOW);
  digitalWrite(PIN_LED_ALERT,  LOW);
  digitalWrite(PIN_BUZZER,     LOW);

  // I2C: SDA = GPIO21, SCL = GPIO22
  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.println();
  Serial.println("[WRIST] ==============================================");
  Serial.println("[WRIST]  VENTWISE WRISTBAND - ESP32 DOIT DEVKIT V1");
  Serial.println("[WRIST] ==============================================");

  Serial.print("[WRIST] Board MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.println();

  // =================================================
  // MAX30102
  // =================================================
  // None of the three checks below is fatal any more. This is a
  // safety device: an unplugged pulse-ox must not stop the panic
  // button, the fall detector and the radio from working. Each
  // sensor records its own health instead, and that health rides
  // on every packet so the master can show "no sensor" rather
  // than a plausible zero.
  if (!sensor.begin()) {
    Serial.println("[WRIST] ERROR: MAX30102 not found -- HR/SpO2 disabled");
  } else {
    sensor.setup();
    hasPulseOx = true;
    Serial.println("[WRIST] MAX30102 OK");
  }

  // =================================================
  // LIS3DH
  // =================================================
  if (!lis.begin(0x19)) {
    Serial.println("[WRIST] ERROR: LIS3DH not found -- fall detection disabled");
  } else {
    lis.setRange(LIS3DH_RANGE_4_G);

    // Set the output data rate explicitly rather than inheriting
    // whatever the library defaults to: the sampling loop below
    // asks for 50 Hz, and reading faster than the device produces
    // just returns the same sample twice.
    lis.setDataRate(LIS3DH_DATARATE_100_HZ);

    hasAccel = true;
    Serial.println("[WRIST] LIS3DH OK (range +/-4 g, 100 Hz)");
  }

  // =================================================
  // DS18B20
  // =================================================
  tempSensor.begin();

  if (tempSensor.getDeviceCount() == 0) {
    Serial.println("[WRIST] WARNING: DS18B20 not found on GPIO16");
    temperatureF = -127.0;
  } else {
    hasTemp = true;

    // The reason the loop used to run at roughly 1 Hz. With
    // waitForConversion left at its default, every
    // requestTemperatures() blocked for the full 750 ms 12-bit
    // conversion -- which starved the accelerometer and the
    // pulse-ox of the sample rates they both need.
    tempSensor.setWaitForConversion(false);

    Serial.println("[WRIST] DS18B20 OK (non-blocking)");
    Serial.print("[WRIST] Temperature sensors found: ");
    Serial.println(tempSensor.getDeviceCount());
  }

  // Steady only when everything answered; a slow blink in the
  // loop means the band is running degraded.
  digitalWrite(PIN_LED_SENSOR, (hasPulseOx && hasAccel && hasTemp) ? HIGH : LOW);

  Serial.println();
  Serial.println("[WRIST] ---------- SENSOR SELF-TEST ----------");
  Serial.print("[WRIST]   MAX30102 (HR/SpO2) : ");
  Serial.println(hasPulseOx ? "PRESENT" : "MISSING");
  Serial.print("[WRIST]   LIS3DH   (motion)  : ");
  Serial.println(hasAccel ? "PRESENT" : "MISSING");
  Serial.print("[WRIST]   DS18B20  (body temp): ");
  Serial.println(hasTemp ? "PRESENT" : "MISSING");
  Serial.println("[WRIST] -------------------------------------");

  // =================================================
  // ESP-NOW INITIALIZATION
  // =================================================
  WiFi.mode(WIFI_STA);
  delay(100);

  Serial.println();
  Serial.print("[WRIST] ESP32 STA MAC: ");
  Serial.println(WiFi.macAddress());

  // This used to compare the band's *own* MAC against MASTER_MAC
  // and print "MASTER MAC VERIFIED", which on a wristband could
  // only ever warn -- it was checking whether this board was the
  // master. What actually matters during bring-up is which peer
  // this board is about to transmit to, so print that instead.
  Serial.print("[WRIST] Sending to peer: ");
  for (int i = 0; i < 6; i++) {
    if (receiverMAC[i] < 16) Serial.print("0");
    Serial.print(receiverMAC[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();

  if (esp_now_init() != ESP_OK) {
    Serial.println("[WRIST] ERROR: ESP-NOW init failed");

    while (1) {
      digitalWrite(PIN_LED_COMM, !digitalRead(PIN_LED_COMM));
      delay(200);
    }
  }

  esp_now_register_send_cb(OnDataSent);

  // =================================================
  // ADD RECEIVER PEER
  // =================================================
  esp_now_peer_info_t peerInfo = {};

  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[WRIST] ERROR: Failed to add receiver peer");

    while (1) {
      digitalWrite(PIN_LED_COMM, !digitalRead(PIN_LED_COMM));
      delay(300);
    }
  }

  Serial.println("[WRIST] ESP-NOW OK");

  Serial.println();
  Serial.println("[WRIST] ==============================================");
  Serial.println("[WRIST]  SYSTEM READY");
  Serial.println("[WRIST] ==============================================");
  Serial.println();

  startupBeeps();
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  unsigned long now = millis();

  if (hasPulseOx) {
    readPulseOx();
  }

  // =================================================
  // ACCELEROMETER - 50 Hz
  // =================================================
  // Sampled on its own clock rather than once per loop, so the
  // rate is a stated 50 Hz instead of whatever the slowest thing
  // in the loop happens to allow.
  if (hasAccel && (now - lastAccelSample >= ACCEL_SAMPLE_INTERVAL)) {
    lastAccelSample = now;
    detectFall();
  }

  // =================================================
  // TEMPERATURE READING - non-blocking
  // =================================================
  // Two phases: ask, then collect ~800 ms later. Body temperature
  // does not move fast enough to justify stalling the radio and
  // the accelerometer for a conversion.
  if (hasTemp) {
    if (!tempConversionPending && (now - lastTempRequest >= TEMP_INTERVAL)) {
      tempSensor.requestTemperatures();
      tempConversionPending = true;
      tempRequestTime       = now;
      lastTempRequest       = now;
    }

    if (tempConversionPending && (now - tempRequestTime >= TEMP_CONVERSION_TIME)) {
      tempConversionPending = false;

      float tempReadingC = tempSensor.getTempCByIndex(0);

      if (tempReadingC != DEVICE_DISCONNECTED_C) {
        // °F = (°C x 9/5) + 32
        temperatureF = (tempReadingC * 9.0 / 5.0) + 32.0;
      } else {
        temperatureF = -127.0;
      }
    }
  }

  // =================================================
  // PANIC BUTTON
  // =================================================
  bool rawButton = digitalRead(PIN_PANIC_BUTTON);

  if (rawButton != lastButtonState) {
    lastDebounceTime = now;
  }

  if ((now - lastDebounceTime) > DEBOUNCE_TIME) {
    if (rawButton == LOW && !panicActive) {
      panicActive    = true;
      panicStartTime = now;

      Serial.println("[WRIST][PANIC] Button pressed");
    }
  }

  lastButtonState = rawButton;

  // =================================================
  // PANIC AUTO-RESET
  // =================================================
  if (panicActive && (now - panicStartTime > PANIC_HOLD_TIME)) {
    panicActive = false;
    Serial.println("[WRIST][PANIC] Cleared");
  }

  // =================================================
  // COMM LED AUTO-OFF
  // =================================================
  if (commLedOn && (now - commLedOnTime >= COMM_LED_PULSE_TIME)) {
    digitalWrite(PIN_LED_COMM, LOW);
    commLedOn = false;
  }

  // =================================================
  // COMMUNICATION LINK LOST
  // =================================================
  if (lastSuccessSend > 0 && (now - lastSuccessSend > COMM_TIMEOUT)) {
    digitalWrite(PIN_LED_COMM, LOW);
    commLedOn = false;
  }

  // =================================================
  // ALERT LED + BUZZER
  // =================================================
  bool emergencyActive = fallDetected || panicActive;

  if (emergencyActive) {
    startAlertBuzzer();

    if (now - lastAlertBlink >= 250) {
      alertLedState = !alertLedState;
      digitalWrite(PIN_LED_ALERT, alertLedState ? HIGH : LOW);
      lastAlertBlink = now;
    }
  } else {
    digitalWrite(PIN_LED_ALERT, LOW);
    alertLedState = false;
  }

  // =================================================
  // BUZZER AUTO-OFF
  // =================================================
  if (buzzerActive && (now - buzzerStartTime >= ALERT_BUZZER_TIME)) {
    digitalWrite(PIN_BUZZER, LOW);
    buzzerActive = false;
    Serial.println("[WRIST] Alert buzzer OFF (timeout)");
  }

  // =================================================
  // TRANSMIT CYCLE - 20 Hz
  // =================================================
  // Was 1 Hz, which made motion a once-a-second still frame. The
  // vitals in the packet genuinely only change at about 1 Hz, but
  // sending them alongside the motion costs a handful of bytes and
  // keeps one packet layout instead of two.
  if (now - lastTx >= TX_INTERVAL) {
    lastTx = now;

    dataToSend.magic          = WRIST_PACKET_MAGIC;
    dataToSend.version        = WRIST_PACKET_VERSION;
    dataToSend.seq            = packetSeq++;

    dataToSend.sensorHealth   = (hasPulseOx ? HEALTH_PULSEOX : 0) |
                                (hasAccel   ? HEALTH_ACCEL   : 0) |
                                (hasTemp    ? HEALTH_TEMP    : 0);

    dataToSend.heartRate      = beatAvg;
    dataToSend.spo2           = SPO2;
    dataToSend.fingerDetected = fingerPresent ? 1 : 0;
    dataToSend.fallDetected   = fallDetected ? 1 : 0;
    dataToSend.panicPressed   = panicActive ? 1 : 0;
    // accel / ax / ay / az are set inside detectFall()
    dataToSend.accelPeak      = (int16_t)(accelPeakSinceTx * 10.0);
    dataToSend.temperatureF   = temperatureF;

    esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&dataToSend, sizeof(dataToSend));

    if (result != ESP_OK) {
      Serial.print("[WRIST] ESP-NOW ERROR: ");
      Serial.println(result);
    }

    // Reset only after a send attempt, so the peak covers exactly
    // the interval between two packets.
    accelPeakSinceTx = 0.0;
  }

  // =================================================
  // SERIAL SUMMARY - 1 Hz
  // =================================================
  // Deliberately slower than the transmit rate: 20 lines a second
  // at 115200 baud would spend more time in Serial.print than in
  // reading sensors.
  if (now - lastPrint >= PRINT_INTERVAL) {
    lastPrint = now;

    Serial.println();
    Serial.println("[WRIST] ========== HEALTH DATA ==========");

    if (!hasPulseOx) {
      Serial.println("[WRIST] Heart Rate : SENSOR MISSING");
    } else if (fingerPresent) {
      Serial.print("[WRIST] Heart Rate : ");
      Serial.print(beatAvg);
      Serial.println(" BPM");

      Serial.print("[WRIST] SpO2       : ");
      Serial.print(SPO2);
      Serial.println(" %");
    } else {
      Serial.println("[WRIST] Finger     : Not Detected");
    }

    Serial.print("[WRIST] Temperature: ");
    if (!hasTemp || temperatureF == -127.0) {
      Serial.println("SENSOR MISSING");
    } else {
      Serial.print(temperatureF, 2);
      Serial.println(" F");
    }

    if (!hasAccel) {
      Serial.println("[WRIST] Accel      : SENSOR MISSING");
    } else {
      Serial.print("[WRIST] Accel      : ");
      Serial.print(dataToSend.accel / 10.0, 2);
      Serial.print(" m/s2  [x ");
      Serial.print(dataToSend.ax / 100.0, 2);
      Serial.print("  y ");
      Serial.print(dataToSend.ay / 100.0, 2);
      Serial.print("  z ");
      Serial.print(dataToSend.az / 100.0, 2);
      Serial.println("]");

      Serial.print("[WRIST] Accel peak : ");
      Serial.print(dataToSend.accelPeak / 10.0, 2);
      Serial.println(" m/s2 (since last packet)");
    }

    Serial.print("[WRIST] Fall Status: ");
    Serial.println(fallDetected ? "DETECTED" : "NORMAL");

    Serial.print("[WRIST] Panic      : ");
    Serial.println(panicActive ? "ACTIVE" : "OFF");

    Serial.print("[WRIST] TX seq     : ");
    Serial.print(packetSeq);
    Serial.print("  (");
    Serial.print(1000 / TX_INTERVAL);
    Serial.print(" Hz)  ok=");
    Serial.print(txOkCount);
    Serial.print(" failed=");
    Serial.println(txFailCount);

    Serial.println("[WRIST] =================================");
  }
}

// =====================================================
// readPulseOx
// =====================================================
void readPulseOx() {
  sensor.check();

  if (!sensor.available())
    return;

  uint32_t irValue  = sensor.getIR();
  uint32_t redValue = sensor.getRed();

  sensor.nextSample();

  // Finger detection
  if (irValue < 5000) {
    if (fingerPresent) {
      Serial.println("[WRIST] Finger removed");
    }

    fingerPresent = false;
    return;
  }

  if (!fingerPresent) {
    Serial.println("[WRIST] Finger detected");
  }

  fingerPresent = true;

  int16_t IR_signal  = pulseIR.dc_filter(irValue);
  int16_t Red_signal = pulseRed.dc_filter(redValue);

  bool beatIR  = pulseIR.isBeat(pulseIR.ma_filter(IR_signal));
  bool beatRed = pulseRed.isBeat(pulseRed.ma_filter(Red_signal));

  if (beatIR) {
    long now = millis();

    if (lastBeat > 0) {
      long btpm = 60000 / (now - lastBeat);

      if (btpm > 40 && btpm < 200) {
        beatAvg = bpm.filter(btpm);
      }
    }

    lastBeat = now;

    long numerator   = (pulseRed.avgAC() * pulseIR.avgDC()) / 256;
    long denominator = (pulseRed.avgDC() * pulseIR.avgAC()) / 256;

    int RX100 = (denominator > 0) ? (numerator * 100) / denominator : 999;

    SPO2f = (10400 - RX100 * 17 + 50) / 100;

    if (RX100 >= 0 && RX100 < 184) {
      SPO2 = spo2_table[RX100];
    }
  }
}

// =====================================================
// detectFall
// =====================================================
void detectFall() {
  sensors_event_t event;
  lis.getEvent(&event);

  float ax = event.acceleration.x;
  float ay = event.acceleration.y;
  float az = event.acceleration.z;

  float totalAccel = sqrt(ax * ax + ay * ay + az * az);

  // Store acceleration x10
  dataToSend.accel = (int16_t)(totalAccel * 10.0);

  // Per-axis, x100. The magnitude alone throws away direction,
  // which is what orientation and activity classification are
  // built on -- a wrist at rest by the side and a wrist held out
  // flat both read 9.8 m/s2.
  dataToSend.ax = (int16_t)(ax * 100.0);
  dataToSend.ay = (int16_t)(ay * 100.0);
  dataToSend.az = (int16_t)(az * 100.0);

  // Held across the 50 Hz samples that fall between two 20 Hz
  // packets, so an impact spike is reported even when it lands
  // in the gap.
  if (totalAccel > accelPeakSinceTx) {
    accelPeakSinceTx = totalAccel;
  }

  // =================================================
  // FREE-FALL
  // =================================================
  if (totalAccel < 4.0) {
    freeFallDetected = true;
    freeFallTime = millis();
  }

  // =================================================
  // IMPACT AFTER FREE-FALL
  // =================================================
  if (freeFallDetected && totalAccel > 20.0 && millis() - freeFallTime < 1000) {
    fallDetected = true;
    freeFallDetected = false;
    fallTime = millis();

    Serial.println("[WRIST][FALL] Fall detected!");
  }

  // =================================================
  // AUTO-RESET FALL
  // =================================================
  if (fallDetected && (millis() - fallTime > FALL_HOLD_TIME)) {
    fallDetected = false;
    fallTime = 0;

    Serial.println("[WRIST][FALL] Cleared");
  }
}
