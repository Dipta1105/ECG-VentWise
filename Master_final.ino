#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Hoisted above every function so the auto-generated prototypes
// that reference it are valid. See the note further down.
struct EnvField {
  const char *prefix;   // e.g. "TEMP="
  uint8_t prefixLen;
  const char *label;    // print label
  const char *suffix;   // unit / suffix appended after value, "" if none
  bool isBoolFlag;      // true => print OK/FAIL (DHT/BMP/LIS) or FIX/NO FIX (GPS) via atoi
  bool isFixFlag;       // true => use FIX/NO FIX wording instead of OK/FAIL
};

// =====================================================
// VENTWISE V3 - MASTER / OBC
// BOARD: ESP32 DOIT DEVKIT V1
// =====================================================
// Optimized pass: identical logic/behavior to the original,
// but with:
//   - compact formatting (huge line-count reduction, easier
//     to scroll/extend)
//   - direct digitalWrite(pin, bool) instead of if/else HIGH/LOW
//   - table-driven ENV telemetry/status parsing (was duplicated
//     twice almost verbatim -> now one shared function)
//   - Serial.printf() instead of long chains of print/println
//     (ESP32 supports printf; fewer calls, smaller flash use)
//   - constexpr for pins/constants instead of #define where it
//     doesn't change semantics (type-safe, same behavior)
//   - small helper functions to remove copy-pasted UI code
//
// NOTE: On ESP32 (unlike AVR/Uno), string literals already live
// in flash and are NOT copied into RAM at boot, so the F() macro
// is unnecessary here -- intentionally not used.
//
// FINAL PIN MAP (unchanged)
// D2  -> GSM LED           D18 -> OXYGEN RELAY
// D4  -> ENV UNIT LED      D19 -> ECG / HEARTBEAT LED
// D5  -> SOS LED           D21 -> OLED SDA
// D12 -> OXYGEN LED        D22 -> OLED SCL
// D13 -> BP RELAY          D25 -> SELECT BUTTON
// D15 -> WRISTBAND LED     D26 -> BACK BUTTON
//                          D27 -> AD8232 LO+
//                          D32 -> UP BUTTON
//                          D33 -> DOWN BUTTON
//                          D34 -> AD8232 ECG OUT
//                          D35 -> O2 POTENTIOMETER
// =====================================================


// =====================================================
// OLED DISPLAY
// =====================================================
constexpr int SCREEN_WIDTH  = 128;
constexpr int SCREEN_HEIGHT = 64;
constexpr int OLED_RESET    = -1;
constexpr uint8_t OLED_ADDR = 0x3C;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// =====================================================
// MAC ADDRESSES
// =====================================================
uint8_t MASTER_MAC[]   = {0x8C, 0x94, 0xDF, 0x6D, 0x86, 0xF4};
uint8_t ENV_MAC[]      = {0x00, 0x70, 0x07, 0xE2, 0x22, 0xE0};
uint8_t ESP8266_MAC[]  = {0x40, 0x91, 0x51, 0x58, 0xD3, 0x33};


// =====================================================
// NAVIGATION BUTTONS
// =====================================================
constexpr uint8_t PIN_BTN_UP     = 32;
constexpr uint8_t PIN_BTN_DOWN   = 33;
constexpr uint8_t PIN_BTN_SELECT = 25;
constexpr uint8_t PIN_BTN_BACK   = 26;
constexpr unsigned long BTN_DEBOUNCE_MS = 40;

struct Button {
  uint8_t pin;
  bool lastReading;
  bool stableState;
  unsigned long lastChangeTime;
};

Button btnUp     = {PIN_BTN_UP,     HIGH, HIGH, 0};
Button btnDown   = {PIN_BTN_DOWN,   HIGH, HIGH, 0};
Button btnSelect = {PIN_BTN_SELECT, HIGH, HIGH, 0};
Button btnBack   = {PIN_BTN_BACK,   HIGH, HIGH, 0};

bool buttonPressed(Button &b) {
  bool reading = digitalRead(b.pin);
  if (reading != b.lastReading) b.lastChangeTime = millis();

  bool firedPress = false;
  if ((millis() - b.lastChangeTime) > BTN_DEBOUNCE_MS && reading != b.stableState) {
    b.stableState = reading;
    if (b.stableState == LOW) firedPress = true;
  }
  b.lastReading = reading;
  return firedPress;
}


// =====================================================
// INDICATOR LED PINS
// =====================================================
constexpr uint8_t LED_WRISTBAND = 15;
constexpr uint8_t LED_GSM       = 2;
constexpr uint8_t LED_ENV       = 4;
constexpr uint8_t LED_SOS       = 5;
constexpr uint8_t LED_OXYGEN    = 12;
constexpr uint8_t LED_ECG       = 19;

constexpr uint8_t INDICATOR_LED_COUNT = 6;
const uint8_t indicatorLEDs[INDICATOR_LED_COUNT] = {
  LED_WRISTBAND, LED_GSM, LED_ENV, LED_SOS, LED_OXYGEN, LED_ECG
};


// =====================================================
// RELAY PINS
// =====================================================
constexpr uint8_t BP_RELAY_PIN = 13;
constexpr uint8_t O2_RELAY_PIN = 18;


// =====================================================
// SYSTEM CONNECTION STATES
// =====================================================
bool wristbandConnected = false;
bool gsmConnected       = false;
bool envConnected       = false;
bool ecgReady           = true;  // local hardware ready after init
bool oxygenReady        = true;

constexpr unsigned long WRISTBAND_TIMEOUT = 5000;
constexpr unsigned long ENV_TIMEOUT       = 5000;
unsigned long lastWristbandRxTime = 0;
unsigned long lastEnvRxTime       = 0;


// =====================================================
// INDICATOR STARTUP STATE MACHINE
// =====================================================
enum IndicatorStartupState { INDICATOR_HYPERBLINK, INDICATOR_ALL_OFF, INDICATOR_ALL_ON, INDICATOR_NORMAL };
IndicatorStartupState indicatorStartupState = INDICATOR_HYPERBLINK;

unsigned long indicatorStateStart = 0;
unsigned long indicatorBlinkTimer = 0;
bool indicatorBlinkState = false;

constexpr unsigned long HYPERBLINK_INTERVAL = 120;
constexpr unsigned long ALL_OFF_DURATION    = 500;
constexpr unsigned long ALL_ON_DURATION     = 5000;


// =====================================================
// PURPOSE LED TIMERS
// =====================================================
unsigned long sosLedTimer = 0;
bool sosLedState = false;

unsigned long oxygenLedTimer = 0;
bool oxygenLedState = false;

unsigned long ecgHeartbeatLedTimer = 0;
bool ecgHeartbeatLedState = false;
constexpr unsigned long ECG_LED_PULSE_MS = 100;


// =====================================================
// LED HELPERS
// =====================================================
inline void setAllIndicatorLEDs(bool state) {
  for (uint8_t i = 0; i < INDICATOR_LED_COUNT; i++) digitalWrite(indicatorLEDs[i], state);
}
inline void setIndicatorLED(uint8_t pin, bool state) { digitalWrite(pin, state); }


// =====================================================
// INDICATOR CONNECTION CHECK
// =====================================================
bool allSystemsStable() {
  if (wristbandConnected && millis() - lastWristbandRxTime > WRISTBAND_TIMEOUT) {
    wristbandConnected = false;
    Serial.println("[LED] WRISTBAND CONNECTION LOST");
  }
  if (envConnected && millis() - lastEnvRxTime > ENV_TIMEOUT) {
    envConnected = false;
    Serial.println("[LED] ENV UNIT CONNECTION LOST");
  }
  return wristbandConnected && gsmConnected && envConnected && ecgReady && oxygenReady;
}


// =====================================================
// START INDICATOR SEQUENCE
// =====================================================
void startIndicatorStartup() {
  indicatorStartupState = INDICATOR_HYPERBLINK;
  indicatorStateStart = millis();
  indicatorBlinkTimer = millis();
  indicatorBlinkState = false;
  setAllIndicatorLEDs(false);

  Serial.println();
  Serial.println("[LED] STARTUP INDICATOR SEQUENCE");
  Serial.println("[LED] Waiting for all systems...");
}


// =====================================================
// WRISTBAND ESP-NOW PACKET (declared early so LED logic can use it)
// =====================================================
// Must match wristband.ino byte for byte. Bump WRIST_PACKET_VERSION
// on both sides together whenever this changes.
#define WRIST_PACKET_MAGIC   0x5742   // 'WB'
#define WRIST_PACKET_VERSION 2

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

  int16_t accel;       // magnitude, m/s^2 x10
  int16_t ax;          // per-axis, m/s^2 x100
  int16_t ay;
  int16_t az;
  int16_t accelPeak;   // max magnitude since previous packet, m/s^2 x10

  float temperatureF;
} WristbandPacket;

WristbandPacket latestWristband = {
  WRIST_PACKET_MAGIC, WRIST_PACKET_VERSION, 0, 0,
  0, 0, 0, 0, 0,
  0, 0, 0, 0, 0,
  -127.0
};
unsigned long wristbandRxCount = 0;
unsigned long lastWristbandLogTime = 0;
unsigned long lastWristbandRejectLog = 0;
bool wristbandLinkEverSeen = false;
bool oxygenSupplyActive = false; // forward declared for use in LED logic below


// =====================================================
// NORMAL PURPOSE INDICATORS
// =====================================================
void updatePurposeIndicators() {
  unsigned long now = millis();

  setIndicatorLED(LED_WRISTBAND, wristbandConnected);
  setIndicatorLED(LED_GSM, gsmConnected);
  setIndicatorLED(LED_ENV, envConnected);

  // SOS LED: 1 Hz blink while panic active, else off
  bool sosActive = wristbandConnected && latestWristband.panicPressed;
  if (sosActive) {
    if (now - sosLedTimer >= 1000) {
      sosLedTimer = now;
      sosLedState = !sosLedState;
      setIndicatorLED(LED_SOS, sosLedState);
    }
  } else {
    sosLedState = false;
    setIndicatorLED(LED_SOS, LOW);
  }

  // Oxygen LED: 0.5 Hz blink while supply active, else off
  if (oxygenSupplyActive) {
    if (now - oxygenLedTimer >= 2000) {
      oxygenLedTimer = now;
      oxygenLedState = !oxygenLedState;
      setIndicatorLED(LED_OXYGEN, oxygenLedState);
    }
  } else {
    oxygenLedState = false;
    setIndicatorLED(LED_OXYGEN, LOW);
  }

  // ECG heartbeat LED: one-shot pulse, turned off after ECG_LED_PULSE_MS
  if (ecgHeartbeatLedState && now - ecgHeartbeatLedTimer >= ECG_LED_PULSE_MS) {
    ecgHeartbeatLedState = false;
    setIndicatorLED(LED_ECG, LOW);
  }
}


// =====================================================
// INDICATOR STARTUP STATE MACHINE
// =====================================================
void updateIndicatorStartup() {
  unsigned long now = millis();

  switch (indicatorStartupState) {

    case INDICATOR_HYPERBLINK:
      if (now - indicatorBlinkTimer >= HYPERBLINK_INTERVAL) {
        indicatorBlinkTimer = now;
        indicatorBlinkState = !indicatorBlinkState;
        setAllIndicatorLEDs(indicatorBlinkState);
      }
      if (allSystemsStable()) {
        indicatorStartupState = INDICATOR_ALL_OFF;
        indicatorStateStart = now;
        setAllIndicatorLEDs(false);
        Serial.println("[LED] ALL SYSTEMS STABLE");
        Serial.println("[LED] ALL LEDS OFF - 500 ms");
      }
      break;

    case INDICATOR_ALL_OFF:
      setAllIndicatorLEDs(false);
      if (now - indicatorStateStart >= ALL_OFF_DURATION) {
        indicatorStartupState = INDICATOR_ALL_ON;
        indicatorStateStart = now;
        setAllIndicatorLEDs(true);
        Serial.println("[LED] ALL LEDS ON - 5 SECONDS");
      }
      break;

    case INDICATOR_ALL_ON:
      setAllIndicatorLEDs(true);
      if (now - indicatorStateStart >= ALL_ON_DURATION) {
        indicatorStartupState = INDICATOR_NORMAL;
        setAllIndicatorLEDs(false);
        sosLedTimer = oxygenLedTimer = ecgHeartbeatLedTimer = now;
        sosLedState = oxygenLedState = ecgHeartbeatLedState = false;
        Serial.println("[LED] NORMAL INDICATOR MODE");
      }
      break;

    case INDICATOR_NORMAL:
      updatePurposeIndicators();
      break;
  }
}


// =====================================================
// NAV MENU STATE
// =====================================================
enum ScreenState {
  SCREEN_MENU, SCREEN_WRISTBAND, SCREEN_ECG, SCREEN_BP,
  SCREEN_TEMP_CONFIRM, SCREEN_TEMP, SCREEN_OXYGEN_CONFIRM, SCREEN_OXYGEN
};
ScreenState currentScreen = SCREEN_MENU;

const char *menuItems[] = {"WRIST BAND", "ECG", "BP", "TEMP", "Oxygen"};
constexpr int MENU_ITEM_COUNT = 5;

int menuIndex = 0;
int oxygenConfirmIndex = 0;
bool screenNeedsRedraw = true;


// =====================================================
// LOGO GRAPHIC
// =====================================================
const uint8_t logoGraphic[] PROGMEM = {
  0x00,0x00,0x00,0x00,0x00,0x07,0xFC,0x00,0x1F,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x3F,0xFF,0x04,0x7F,0xFF,0xC0,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x78,0x03,0xCF,0xFF,0xFF,0xF0,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x01,0xCF,0xFC,0xFB,0xFF,0xFF,0xFC,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x03,0x9F,0x0F,0x78,0x7F,0xFF,0xFE,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x03,0x78,0x03,0xF1,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x06,0xE0,0x00,0xF1,0xFF,0x3F,0xFF,0x80,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x0D,0xC0,0x00,0x35,0xFF,0x3F,0xFF,0xC0,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x0D,0x80,0x00,0x35,0xFF,0x1F,0xFF,0xE0,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x1B,0x00,0x00,0x3D,0xFF,0x1F,0xFF,0xF0,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x1B,0x00,0x00,0x2D,0xFE,0x1F,0xFF,0xF0,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x36,0x00,0x00,0x2E,0xFE,0x1F,0xFF,0xF8,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x36,0x00,0x00,0x6E,0xFE,0x5F,0xFF,0xF8,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x36,0x00,0x00,0x6E,0xFE,0x4F,0xFF,0xFC,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x36,0x00,0x00,0x6E,0xFE,0x4F,0xFF,0xFC,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x3C,0x00,0x00,0x6E,0xFE,0x4F,0xFF,0xFC,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x3C,0x00,0x00,0x7A,0xFE,0xE7,0xFF,0xFE,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x3C,0x00,0xF8,0x5A,0xFC,0xE7,0xFF,0xFE,0x18,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x3C,0x01,0x8C,0x5A,0xFC,0xE6,0x7F,0xFE,0x7C,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x7F,0xC7,0x64,0xDA,0xFC,0xF4,0x7F,0xFE,0xC2,0x00,0x00,0x00,
  0x00,0x00,0x01,0xFF,0xFF,0xFE,0xF6,0xD3,0xFC,0xF0,0x3F,0xFF,0xBB,0x00,0x00,0x00,
  0x00,0x00,0x03,0x00,0x00,0x01,0x96,0xD3,0x7D,0xF0,0x00,0x00,0x2D,0x00,0x00,0x00,
  0x00,0x00,0x01,0xFF,0xFB,0xFF,0x1A,0xD3,0x79,0xF9,0xC2,0x01,0x39,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x1B,0xFC,0x1A,0xD3,0x79,0xF9,0xFF,0xFF,0x9B,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x0D,0x80,0x1B,0xF1,0x79,0xFB,0xFF,0xFE,0xC6,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x0D,0x80,0x0B,0xB1,0x39,0xFF,0xFF,0xFE,0x7C,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x06,0xC0,0x0B,0xB1,0xB9,0xFF,0xFF,0xFC,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x06,0xC0,0x0D,0xB1,0xB3,0xFF,0xFF,0xFC,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x03,0x60,0x0D,0xB1,0xB3,0xFF,0xFF,0xFC,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x03,0x30,0x0D,0xA0,0x93,0xFF,0xFF,0xFC,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x01,0xB8,0x05,0xA0,0xD3,0xFF,0xFF,0xF8,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0xD8,0x05,0xA0,0x47,0xFF,0xFF,0xF8,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x6C,0x05,0xA0,0x47,0xFF,0xFF,0xF0,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x76,0x04,0x60,0x47,0xFF,0xFF,0xF0,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x33,0x06,0x60,0x47,0xFF,0xFF,0xE0,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x19,0x86,0x60,0x67,0xFF,0xFE,0xC0,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x0C,0xC6,0x60,0x67,0xFF,0xFF,0x80,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x06,0x66,0x60,0x7F,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x03,0x3A,0x40,0x7F,0xFF,0xFE,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x01,0xDF,0xC0,0x7F,0xFF,0xF8,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0xEF,0x00,0x7F,0xFF,0xF0,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x73,0x01,0xFF,0xFF,0x80,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x19,0xC7,0x67,0xFC,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x0E,0xFC,0xE0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x39,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xC7,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};


// =====================================================
// PARTICLE SYSTEM
// =====================================================
constexpr int PARTICLE_COUNT = 28;
struct Particle { float x, y, vx; uint8_t life, maxLife; };
Particle particles[PARTICLE_COUNT];

void resetParticles() {
  for (int i = 0; i < PARTICLE_COUNT; i++) {
    particles[i].x = random(-30, 10);
    particles[i].y = random(2, 47);
    particles[i].vx = random(8, 22) / 10.0f;
    particles[i].life = random(15, 50);
    particles[i].maxLife = particles[i].life;
  }
}

void updateParticles(float sweepX) {
  for (int i = 0; i < PARTICLE_COUNT; i++) {
    particles[i].x += particles[i].vx;
    if (particles[i].x > sweepX + 12 || particles[i].life == 0) {
      particles[i].x = sweepX - random(0, 25);
      particles[i].y = random(2, 47);
      particles[i].vx = random(8, 22) / 10.0f;
      particles[i].life = random(18, 50);
      particles[i].maxLife = particles[i].life;
    }
    if (particles[i].life > 0) particles[i].life--;
  }
}

void drawGraphic(uint8_t sweepX) {
  for (int y = 0; y < 48; y++) {
    for (int x = 0; x < sweepX; x++) {
      uint8_t b = pgm_read_byte(&logoGraphic[y * 16 + (x >> 3)]);
      if (b & (0x80 >> (x & 7))) display.drawPixel(x, y, SSD1306_WHITE);
    }
  }
}

void drawParticles(float sweepX) {
  for (int i = 0; i < PARTICLE_COUNT; i++) {
    if (particles[i].life == 0) continue;
    int x = (int)particles[i].x;
    int y = (int)particles[i].y;

    if (x >= 0 && x < 128 && y >= 0 && y < 48) display.drawPixel(x, y, SSD1306_WHITE);

    if (particles[i].life > particles[i].maxLife / 2) {
      if (x - 1 >= 0) display.drawPixel(x - 1, y, SSD1306_WHITE);
      if (random(0, 3) == 0 && x - 2 >= 0) display.drawPixel(x - 2, y, SSD1306_WHITE);
    }
  }

  if (sweepX >= 0 && sweepX < 128) {
    for (int y = 4; y < 45; y += 3) {
      if (random(0, 4) != 0) display.drawPixel((int)sweepX, y, SSD1306_WHITE);
    }
  }
}

void drawCircuitStormText(int charsToShow, bool cursorVisible) {
  const char *text = "CIRCUITSTORM";
  constexpr int len = 12;
  constexpr int textWidth = len * 6;
  constexpr int startX = (128 - textWidth) / 2;
  constexpr int textY = 54;

  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(startX, textY);

  for (int i = 0; i < charsToShow && i < len; i++) display.write(text[i]);

  if (cursorVisible && charsToShow < len) {
    display.fillRect(startX + charsToShow * 6, textY + 6, 4, 1, SSD1306_WHITE);
  }
}

void playCircuitStormIntro() {
  resetParticles();
  constexpr unsigned long graphicTime = 1900;
  const char *text = "CIRCUITSTORM";
  constexpr int textLen = 12;
  int typingDelays[] = {75, 155, 55, 210, 90, 135, 60, 185, 80, 240, 70, 170};

  unsigned long startTime = millis();
  int typedChars = 0;
  unsigned long nextTypeTime = startTime + typingDelays[0];

  while (true) {
    unsigned long now = millis();
    unsigned long elapsed = now - startTime;

    if (typedChars < textLen && now >= nextTypeTime) {
      typedChars++;
      if (typedChars < textLen) nextTypeTime = now + typingDelays[typedChars];
    }

    float t = (elapsed >= graphicTime) ? 1.0f : (float)elapsed / graphicTime;
    float eased = 1.0f - pow(1.0f - t, 3.0f);
    float sweepX = eased * 128.0f;

    display.clearDisplay();
    drawGraphic((uint8_t)sweepX);
    updateParticles(sweepX);
    drawParticles(sweepX);

    bool cursorVisible = ((now / 180) % 2) == 0;
    drawCircuitStormText(typedChars, cursorVisible);
    display.display();

    bool graphicFinished = elapsed >= graphicTime;
    bool textFinished = typedChars >= textLen;
    if (graphicFinished && textFinished) break;

    delay(25);
  }

  display.clearDisplay();
  drawGraphic(128);
  drawCircuitStormText(textLen, false);
  display.display();
  delay(3000);
}


// =====================================================
// VENTWISE STARTUP SCREEN
// =====================================================
void showVentwiseFade() {
  display.clearDisplay();
  display.display();

  for (int step = 0; step <= 20; step++) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);

    for (int thickness = 0; thickness < step / 4; thickness++) {
      display.setCursor(10 + thickness, 18 + thickness);
      display.println("VENTWISE");
      display.setCursor(30 + thickness, 36 + thickness);
      display.println("V3");
    }

    display.setCursor(10, 18);
    display.println("VENTWISE");
    display.setCursor(30, 36);
    display.println("V3");

    display.display();
    delay(50);
  }

  delay(3000);
}


// =====================================================
// CIRCUITSTORM TEXT SCREEN
// =====================================================
void showCircuitStormByText() {
  const char *text = "CIRCUITSTORM";
  int textLen = strlen(text);

  display.clearDisplay();
  display.display();

  for (int fadeStep = 0; fadeStep <= 10; fadeStep++) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(52, 14);

    for (int thick = 0; thick < fadeStep / 3; thick++) {
      display.setCursor(52 + thick, 14 + thick);
      display.println("by");
    }

    display.setCursor(52, 14);
    display.println("by");
    display.display();
    delay(60);
  }

  delay(300);

  for (int i = 0; i < textLen; i++) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(52, 14);
    display.println("by");

    display.setCursor(20, 36);
    for (int j = 0; j <= i; j++) display.write(text[j]);

    if (i > 0) {
      for (int blink = 0; blink < 4; blink++) {
        display.display();
        delay(35);
        display.setCursor(20 + (i * 6), 36);
        display.write(text[i]);
        display.display();
        delay(35);
      }
    }

    display.display();
    delay(80);
  }

  delay(3000);
}


// =====================================================
// ECG MODULE
// =====================================================
constexpr uint8_t ECG_PIN = 34;
constexpr uint8_t ECG_LO_PLUS = 27;
constexpr uint8_t ECG_LO_MINUS = 14;

constexpr int ECG_SAMPLE_RATE = 250;
constexpr unsigned long ECG_SAMPLE_INTERVAL = 1000000UL / ECG_SAMPLE_RATE;
unsigned long ecgLastSampleTime = 0;

constexpr int ECG_DISPLAY_RATE = 100;
constexpr int ECG_DISPLAY_DIVIDER = ECG_SAMPLE_RATE / ECG_DISPLAY_RATE;
int ecgDisplayCounter = 0;

constexpr unsigned long ECG_CALIBRATION_TIME = 5000;
bool ecgCalibrated = false;
unsigned long ecgCalibrationStart = 0;
float ecgDcBaseline = 0;
float ecgCalibrationSum = 0;
unsigned long ecgCalibrationSamples = 0;

float ecgBaseline = 2048.0;
float ecgFilteredSignal = 0;

constexpr int ECG_CENTER = 32;
constexpr int ECG_MAX_AMPLITUDE = 28;
float ecgDisplayScale = 300.0;

int ecgWaveform[SCREEN_WIDTH];
int ecgWritePosition = 0;

float ecgBeatThreshold = 0;
bool ecgBeatAboveThreshold = false;
unsigned long ecgLastBeatTime = 0;
constexpr unsigned long ECG_MIN_BEAT_INTERVAL = 300;
constexpr float ECG_BEAT_THRESHOLD_MULTIPLIER = 0.55f;


float processECG(float raw) {
  ecgBaseline += 0.0015f * (raw - ecgBaseline);
  float ac = raw - ecgBaseline;
  ecgFilteredSignal += 0.45f * (ac - ecgFilteredSignal);
  return ecgFilteredSignal;
}

void triggerECGHeartbeatLED() {
  if (indicatorStartupState != INDICATOR_NORMAL) return;
  ecgHeartbeatLedState = true;
  ecgHeartbeatLedTimer = millis();
  setIndicatorLED(LED_ECG, HIGH);
}

void ecgResetForNewSession() {
  ecgCalibrated = false;
  ecgCalibrationStart = millis();
  ecgCalibrationSum = 0;
  ecgCalibrationSamples = 0;
  ecgFilteredSignal = 0;
  ecgDisplayScale = 300.0;
  ecgWritePosition = 0;
  ecgDisplayCounter = 0;
  ecgLastSampleTime = micros();

  ecgBeatThreshold = 0;
  ecgBeatAboveThreshold = false;
  ecgLastBeatTime = 0;

  for (int i = 0; i < SCREEN_WIDTH; i++) ecgWaveform[i] = ECG_CENTER;

  Serial.println("[ECG] Session start -- calibrating 5s");
}

void ecgCalibrate(int raw, bool leadOff) {
  ecgCalibrationSum += raw;
  ecgCalibrationSamples++;

  if (ecgCalibrationSamples == 1) {
    ecgBaseline = raw;
    ecgFilteredSignal = 0;
  }

  processECG((float)raw);

  unsigned long elapsed = millis() - ecgCalibrationStart;
  static unsigned long lastCalDisplay = 0;

  if (millis() - lastCalDisplay >= 250) {
    lastCalDisplay = millis();
    int seconds = elapsed / 1000;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("ECG CALIBRATION");
    display.setCursor(0, 14);
    display.println(leadOff ? "LEAD OFF!" : "Stay still...");
    display.setCursor(0, 28);
    display.printf("%d / 5 sec\n", seconds);
    display.setCursor(0, 54);
    display.println("BACK: menu");
    display.display();
  }

  if (elapsed >= ECG_CALIBRATION_TIME) {
    ecgDcBaseline = ecgCalibrationSum / ecgCalibrationSamples;
    ecgBaseline = ecgDcBaseline;
    ecgFilteredSignal = 0;
    ecgCalibrated = true;
    ecgReady = true;

    ecgBeatThreshold = 0;
    ecgBeatAboveThreshold = false;
    ecgLastBeatTime = 0;

    for (int i = 0; i < SCREEN_WIDTH; i++) ecgWaveform[i] = ECG_CENTER;
    ecgWritePosition = 0;
    ecgDisplayScale = 300;

    Serial.printf("[ECG] Calibration complete, baseline=%.2f\n", ecgDcBaseline);
  }
}

void ecgDrawWaveform() {
  display.clearDisplay();

  for (int x = 0; x < SCREEN_WIDTH; x += 8) display.drawPixel(x, ECG_CENTER, SSD1306_WHITE);

  for (int x = 1; x < SCREEN_WIDTH; x++) {
    int index1 = (ecgWritePosition + x - 1) % SCREEN_WIDTH;
    int index2 = (ecgWritePosition + x) % SCREEN_WIDTH;
    display.drawLine(x - 1, ecgWaveform[index1], x, ecgWaveform[index2], SSD1306_WHITE);
  }

  display.display();
}

void ecgDrawLeadOff() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.println("ECG");
  display.setCursor(0, 26);
  display.println("LEAD OFF");
  display.setCursor(0, 54);
  display.println("BACK: menu");
  display.display();
}

void ecgProcessSample() {
  int raw = analogRead(ECG_PIN);
  bool leadOff = digitalRead(ECG_LO_PLUS) || digitalRead(ECG_LO_MINUS);

  if (!ecgCalibrated) {
    ecgCalibrate(raw, leadOff);
    return;
  }

  float ecg = processECG((float)raw);

  if (leadOff) Serial.println("[ECG] LEAD OFF");

  ecgDisplayCounter++;
  if (ecgDisplayCounter < ECG_DISPLAY_DIVIDER) return;
  ecgDisplayCounter = 0;

  if (leadOff) {
    ecgDrawLeadOff();
    return;
  }

  float magnitude = fabs(ecg);

  // Adaptive display scale
  if (magnitude > ecgDisplayScale) ecgDisplayScale = magnitude * 1.15f;
  else ecgDisplayScale *= 0.9995f;
  ecgDisplayScale = constrain(ecgDisplayScale, 100.0f, 1000.0f);

  // Heartbeat detection
  float dynamicThreshold = max(80.0f, ecgDisplayScale * ECG_BEAT_THRESHOLD_MULTIPLIER);
  ecgBeatThreshold = 0.90f * ecgBeatThreshold + 0.10f * dynamicThreshold;

  if (magnitude > ecgBeatThreshold && !ecgBeatAboveThreshold &&
      millis() - ecgLastBeatTime >= ECG_MIN_BEAT_INTERVAL) {
    ecgBeatAboveThreshold = true;
    ecgLastBeatTime = millis();
    triggerECGHeartbeatLED();
    Serial.println("[ECG] HEARTBEAT");
  }

  if (magnitude < ecgBeatThreshold * 0.65f) ecgBeatAboveThreshold = false;

  // Waveform
  float normalized = constrain(ecg / ecgDisplayScale, -1.0f, 1.0f);
  int y = constrain((int)(ECG_CENTER - (normalized * ECG_MAX_AMPLITUDE)), 1, SCREEN_HEIGHT - 2);

  ecgWaveform[ecgWritePosition] = y;
  ecgWritePosition = (ecgWritePosition + 1) % SCREEN_WIDTH;

  ecgDrawWaveform();
}

void ecgUpdate() {
  unsigned long now = micros();
  if ((unsigned long)(now - ecgLastSampleTime) >= ECG_SAMPLE_INTERVAL) {
    ecgLastSampleTime += ECG_SAMPLE_INTERVAL;
    ecgProcessSample();
  }
}


// =====================================================
// BP MODULE
// =====================================================
constexpr unsigned long BP_PULSE_DURATION_MS = 2000;
bool bpRelayActive = false;
unsigned long bpPulseStartTime = 0;

void bpStart() {
  digitalWrite(BP_RELAY_PIN, HIGH);
  bpRelayActive = true;
  bpPulseStartTime = millis();
  Serial.println("[BP] Relay ON -- measurement cycle started");
}

void bpStopImmediate() {
  digitalWrite(BP_RELAY_PIN, LOW);
  if (bpRelayActive) Serial.println("[BP] Relay OFF -- BACK pressed");
  bpRelayActive = false;
}

void bpUpdate() {
  if (!bpRelayActive) return;
  if (millis() - bpPulseStartTime >= BP_PULSE_DURATION_MS) {
    digitalWrite(BP_RELAY_PIN, LOW);
    bpRelayActive = false;
    Serial.println("[BP] Relay OFF -- measurement complete");
  }
}


// =====================================================
// OXYGEN MODULE
// =====================================================
constexpr uint8_t O2_POT_PIN = 35;
constexpr unsigned long O2_MIN_INTERVAL = 500;
constexpr unsigned long O2_MAX_INTERVAL = 5000;

bool o2RelayState = LOW;
unsigned long o2PreviousMillis = 0;
unsigned long o2LastPrint = 0;
unsigned long o2CurrentInterval = O2_MAX_INTERVAL;
int o2LastPotValue = 0;

void oxygenTurnOn() {
  oxygenSupplyActive = true;
  oxygenReady = true;
  o2PreviousMillis = millis();
  Serial.println("[O2] Oxygen supply ON");
}

void oxygenTurnOff() {
  oxygenSupplyActive = false;
  o2RelayState = LOW;
  digitalWrite(O2_RELAY_PIN, LOW);
  Serial.println("[O2] Oxygen supply OFF");
}

void oxygenUpdate() {
  if (!oxygenSupplyActive) return;

  int potValue = analogRead(O2_POT_PIN);
  o2LastPotValue = potValue;

  unsigned long interval = map(potValue, 0, 4095, O2_MAX_INTERVAL, O2_MIN_INTERVAL);
  o2CurrentInterval = interval;

  unsigned long currentMillis = millis();
  if (currentMillis - o2PreviousMillis >= interval) {
    o2PreviousMillis = currentMillis;
    o2RelayState = !o2RelayState;
    digitalWrite(O2_RELAY_PIN, o2RelayState);
  }

  if (currentMillis - o2LastPrint >= 500) {
    o2LastPrint = currentMillis;
    Serial.printf("[O2] Pot: %d / 4095 | Interval: %lu ms | Relay: %s\n",
                  potValue, interval, o2RelayState ? "ON" : "OFF");
  }
}


// =====================================================
// OLED HEADER
// =====================================================
void drawHeader(const char *title) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(title);
  display.drawLine(0, 9, SCREEN_WIDTH - 1, 9, SSD1306_WHITE);
}


// =====================================================
// MAIN MENU
// =====================================================
void drawMenuScreen() {
  display.clearDisplay();
  drawHeader("VENTWISE - MAIN MENU");

  for (int i = 0; i < MENU_ITEM_COUNT; i++) {
    display.setCursor(0, 13 + (i * 10));
    display.printf("%s%d. %s\n", (i == menuIndex) ? "> " : "  ", i + 1, menuItems[i]);
  }

  display.display();
}


// =====================================================
// WRISTBAND SCREEN
// =====================================================
void drawWristbandScreen() {
  display.clearDisplay();
  drawHeader("WRIST BAND");

  if (!wristbandLinkEverSeen) {
    display.setCursor(0, 16);
    display.println("Waiting for link...");
  } else {
    display.setCursor(0, 16);
    display.print("HR   : ");
    if (latestWristband.fingerDetected) display.printf("%d BPM\n", latestWristband.heartRate);
    else display.println("--");

    display.setCursor(0, 30);
    display.print("SpO2 : ");
    if (latestWristband.fingerDetected) display.printf("%d %%\n", latestWristband.spo2);
    else display.println("--");

    display.setCursor(0, 44);
    if (!latestWristband.fingerDetected) display.println("Finger not detected");
    else if (millis() - lastWristbandRxTime > 5000) display.println("Link stale");
    else display.println("Link OK");
  }

  display.setCursor(0, 56);
  display.println("BACK: menu");
  display.display();
}


// =====================================================
// BP SCREEN
// =====================================================
void drawBPScreen() {
  display.clearDisplay();
  drawHeader("BP MEASUREMENT");
  display.setCursor(0, 24);
  display.println("Check Other Display");
  display.setCursor(0, 34);
  display.println("for BP");
  display.setCursor(0, 56);
  display.println("BACK: stop + menu");
  display.display();
}


// =====================================================
// TEMPERATURE CONFIRM
// =====================================================
void drawTempConfirmScreen() {
  display.clearDisplay();
  drawHeader("TEMPERATURE");
  display.setCursor(0, 20);
  display.println("Ensure probe is");
  display.setCursor(0, 30);
  display.println("touching skin.");
  display.setCursor(0, 48);
  display.println("SELECT: continue");
  display.setCursor(0, 56);
  display.println("BACK: menu");
  display.display();
}


// =====================================================
// TEMPERATURE
// =====================================================
void drawTempScreen() {
  display.clearDisplay();
  drawHeader("TEMPERATURE");
  display.setCursor(0, 24);

  if (latestWristband.temperatureF == -127.0 && !wristbandLinkEverSeen) {
    display.println("Waiting for link...");
  } else if (latestWristband.temperatureF == -127.0) {
    display.println("Probe not found");
  } else {
    display.setTextSize(2);
    display.setCursor(10, 22);
    display.print(latestWristband.temperatureF, 1);
    display.println(" F");
    display.setTextSize(1);
  }

  display.setCursor(0, 56);
  display.println("BACK: menu");
  display.display();
}


// =====================================================
// OXYGEN CONFIRM
// =====================================================
void drawOxygenConfirmScreen() {
  display.clearDisplay();
  drawHeader("OXYGEN SUPPLY");
  display.setCursor(0, 18);
  display.println("Turn off oxygen");
  display.setCursor(0, 28);
  display.println("supply?");

  display.setCursor(10, 42);
  display.printf("%sYES\n", oxygenConfirmIndex == 0 ? "> " : "  ");
  display.setCursor(10, 52);
  display.printf("%sNO\n", oxygenConfirmIndex == 1 ? "> " : "  ");

  display.display();
}


// =====================================================
// OXYGEN SCREEN
// =====================================================
void drawOxygenScreen() {
  display.clearDisplay();
  drawHeader("OXYGEN - RUNNING");

  display.setCursor(0, 16);
  display.printf("Rate int: %lu ms\n", o2CurrentInterval);

  display.setCursor(0, 30);
  display.print("SpO2    : ");
  if (wristbandLinkEverSeen && latestWristband.fingerDetected) display.printf("%d %%\n", latestWristband.spo2);
  else display.println("--");

  display.setCursor(0, 44);
  display.printf("Relay: %s\n", o2RelayState ? "ON" : "OFF");

  display.setCursor(0, 56);
  display.println("BACK: menu (stays on)");
  display.display();
}


// =====================================================
// ENTER MENU
// =====================================================
void enterMenu() {
  currentScreen = SCREEN_MENU;
  screenNeedsRedraw = true;
  Serial.println("[NAV] -> MAIN MENU");
}


// =====================================================
// MENU SELECT
// =====================================================
void selectMenuItem() {
  switch (menuIndex) {

    case 0: // WRISTBAND
      currentScreen = SCREEN_WRISTBAND;
      Serial.println("[NAV] -> WRIST BAND");
      break;

    case 1: // ECG
      ecgResetForNewSession();
      currentScreen = SCREEN_ECG;
      Serial.println("[NAV] -> ECG");
      break;

    case 2: // BP
      bpStart();
      currentScreen = SCREEN_BP;
      Serial.println("[NAV] -> BP");
      break;

    case 3: // TEMPERATURE
      currentScreen = SCREEN_TEMP_CONFIRM;
      Serial.println("[NAV] -> TEMP");
      break;

    case 4: // OXYGEN
      if (oxygenSupplyActive) {
        oxygenConfirmIndex = 1;
        currentScreen = SCREEN_OXYGEN_CONFIRM;
        Serial.println("[NAV] -> OXYGEN CONFIRM OFF");
      } else {
        oxygenTurnOn();
        currentScreen = SCREEN_OXYGEN;
        Serial.println("[NAV] -> OXYGEN ON");
      }
      break;
  }

  screenNeedsRedraw = true;
}


// =====================================================
// NAVIGATION
// =====================================================
void handleNavigation() {
  bool up     = buttonPressed(btnUp);
  bool down   = buttonPressed(btnDown);
  bool select = buttonPressed(btnSelect);
  bool back   = buttonPressed(btnBack);

  switch (currentScreen) {

    case SCREEN_MENU:
      if (up) {
        menuIndex = (menuIndex - 1 + MENU_ITEM_COUNT) % MENU_ITEM_COUNT;
        screenNeedsRedraw = true;
      } else if (down) {
        menuIndex = (menuIndex + 1) % MENU_ITEM_COUNT;
        screenNeedsRedraw = true;
      } else if (select) {
        selectMenuItem();
      }
      break;

    case SCREEN_WRISTBAND:
      if (back) enterMenu();
      break;

    case SCREEN_ECG:
      if (back) {
        Serial.println("[ECG] Session stopped");
        enterMenu();
      }
      break;

    case SCREEN_BP:
      if (back) {
        bpStopImmediate();
        enterMenu();
      }
      break;

    case SCREEN_TEMP_CONFIRM:
      if (select) {
        currentScreen = SCREEN_TEMP;
        screenNeedsRedraw = true;
        Serial.println("[NAV] TEMP probe confirmed");
      } else if (back) {
        enterMenu();
      }
      break;

    case SCREEN_TEMP:
      if (back) enterMenu();
      break;

    case SCREEN_OXYGEN_CONFIRM:
      if (up || down) {
        oxygenConfirmIndex = 1 - oxygenConfirmIndex;
        screenNeedsRedraw = true;
      } else if (select) {
        if (oxygenConfirmIndex == 0) {
          oxygenTurnOff();
          enterMenu();
        } else {
          currentScreen = SCREEN_OXYGEN;
          screenNeedsRedraw = true;
          Serial.println("[NAV] Oxygen remains ON");
        }
      } else if (back) {
        enterMenu();
      }
      break;

    case SCREEN_OXYGEN:
      if (back) enterMenu();
      break;
  }
}


// =====================================================
// ACTIVE SCREEN LOGIC
// =====================================================
void updateActiveScreenLogic() {
  switch (currentScreen) {

    case SCREEN_ECG:
      ecgUpdate();
      return;

    case SCREEN_BP: {
      bool wasActive = bpRelayActive;
      bpUpdate();
      if (wasActive && !bpRelayActive) {
        enterMenu();
        return;
      }
      break;
    }

    default:
      break;
  }

  static unsigned long lastPeriodicRedraw = 0;
  bool isLiveScreen = (currentScreen == SCREEN_WRISTBAND ||
                        currentScreen == SCREEN_TEMP ||
                        currentScreen == SCREEN_OXYGEN);

  if (screenNeedsRedraw || (isLiveScreen && millis() - lastPeriodicRedraw > 300)) {
    lastPeriodicRedraw = millis();
    screenNeedsRedraw = false;

    switch (currentScreen) {
      case SCREEN_MENU:             drawMenuScreen();           break;
      case SCREEN_WRISTBAND:        drawWristbandScreen();      break;
      case SCREEN_BP:                drawBPScreen();             break;
      case SCREEN_TEMP_CONFIRM:     drawTempConfirmScreen();    break;
      case SCREEN_TEMP:             drawTempScreen();           break;
      case SCREEN_OXYGEN_CONFIRM:   drawOxygenConfirmScreen();  break;
      case SCREEN_OXYGEN:           drawOxygenScreen();         break;
      default: break;
    }
  }
}


// =====================================================
// ESP-NOW: SHARED ENV TELEMETRY / STATUS FIELD TABLE
// =====================================================
// TYPE=DATA and TYPE=STATUS used to have two near-identical
// blocks of strncmp/print chains. Both are now driven by one
// table + one parser, selectable per message type.
// Defined via #include-level placement at the top of the file
// instead -- see the EnvField block near the includes. This file
// did not build before that move: arduino-cli generates forward
// declarations for printEnvField() and parseEnvTokens() and
// inserts them above this point, so EnvField was referenced
// before it existed.

// Fields valid in TYPE=DATA telemetry packets
const EnvField envDataFields[] = {
  {"TEMP=",  5, "TEMPERATURE",  " C",     false, false},
  {"HUM=",   4, "HUMIDITY",     " %",     false, false},
  {"PRESS=", 6, "PRESSURE",     " hPa",   false, false},
  {"ALT=",   4, "ALTITUDE",     " m",     false, false},
  {"ACCX=",  5, "ACCEL X",      " m/s2",  false, false},
  {"ACCY=",  5, "ACCEL Y",      " m/s2",  false, false},
  {"ACCZ=",  5, "ACCEL Z",      " m/s2",  false, false},
  {"LAT=",   4, "LATITUDE",     "",       false, false},
  {"LON=",   4, "LONGITUDE",    "",       false, false},
  {"SAT=",   4, "SATELLITES",   "",       false, false},
  {"DHT=",   4, "DHT22",        "",       true,  false},
  {"BMP=",   4, "BMP280",       "",       true,  false},
  {"LIS=",   4, "LIS3DH",       "",       true,  false},
  {"GPS=",   4, "GNSS",         "",       true,  true},
};
constexpr int ENV_DATA_FIELD_COUNT = sizeof(envDataFields) / sizeof(envDataFields[0]);

// Fields valid in TYPE=STATUS packets (subset)
const EnvField envStatusFields[] = {
  {"DHT=", 4, "DHT22",      "", true,  false},
  {"BMP=", 4, "BMP280",     "", true,  false},
  {"LIS=", 4, "LIS3DH",     "", true,  false},
  {"GPS=", 4, "GNSS",       "", true,  true},
  {"SAT=", 4, "SATELLITES", "", false, false},
};
constexpr int ENV_STATUS_FIELD_COUNT = sizeof(envStatusFields) / sizeof(envStatusFields[0]);

void printEnvField(const EnvField &f, const char *value) {
  if (f.isBoolFlag) {
    const char *onWord  = f.isFixFlag ? "FIX" : "OK";
    const char *offWord = f.isFixFlag ? "NO FIX" : "FAIL";
    Serial.printf("%-12s: %s\n", f.label, atoi(value) ? onWord : offWord);
  } else {
    Serial.printf("%-12s: %s%s\n", f.label, value, f.suffix);
  }
}

// Parses ';'-separated "KEY=VAL" tokens from `message` against the given
// field table and prints each match. `message` is tokenized in place
// (same as original strtok-based approach).
void parseEnvTokens(char *message, const EnvField *fields, int fieldCount) {
  char *token = strtok(message, ";");
  while (token != nullptr) {
    if (strncmp(token, "TYPE=", 5) == 0) {
      Serial.printf("%-12s: %s\n", "TYPE", token + 5);
    } else {
      for (int i = 0; i < fieldCount; i++) {
        if (strncmp(token, fields[i].prefix, fields[i].prefixLen) == 0) {
          printEnvField(fields[i], token + fields[i].prefixLen);
          break;
        }
      }
    }
    token = strtok(nullptr, ";");
  }
}


// =====================================================
// ESP-NOW RECEIVE CALLBACK
// =====================================================
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (data == nullptr || len <= 0) return;

  // CHECK SOURCE
  bool fromENV = memcmp(info->src_addr, ENV_MAC, 6) == 0;
  if (fromENV) {
    envConnected = true;
    lastEnvRxTime = millis();
    Serial.println("[ENV] Connection established");
  }

  // WRISTBAND BINARY PACKET
  // Identified by MAGIC and VERSION, not by length alone.
  if (len == sizeof(WristbandPacket)) {
    WristbandPacket incoming;
    memcpy(&incoming, data, sizeof(WristbandPacket));

    unsigned long nowMs = millis();

    if (incoming.magic != WRIST_PACKET_MAGIC ||
        incoming.version != WRIST_PACKET_VERSION) {
      if (nowMs - lastWristbandRejectLog > 2000) {
        lastWristbandRejectLog = nowMs;
        Serial.printf("[WRIST] REJECTED: magic=0x%04X version=%u "
                      "(expected 0x%04X / %u). Reflash both boards.\n",
                      incoming.magic, incoming.version,
                      WRIST_PACKET_MAGIC, WRIST_PACKET_VERSION);
      }
      return;
    }

    latestWristband = incoming;
    lastWristbandRxTime = nowMs;
    wristbandLinkEverSeen = true;
    wristbandConnected = true;
    wristbandRxCount++;

    if (nowMs - lastWristbandLogTime >= 1000) {
      lastWristbandLogTime = nowMs;
      Serial.printf("[WRIST] seq=%u n=%lu HR=%d SpO2=%d Temp=%.1fF "
                    "Finger=%d Fall=%d Panic=%d accel=%.2f "
                    "[x %.2f y %.2f z %.2f] peak=%.2f\n",
                    latestWristband.seq, wristbandRxCount,
                    latestWristband.heartRate, latestWristband.spo2,
                    latestWristband.temperatureF, latestWristband.fingerDetected,
                    latestWristband.fallDetected, latestWristband.panicPressed,
                    latestWristband.accel / 10.0,
                    latestWristband.ax / 100.0, latestWristband.ay / 100.0,
                    latestWristband.az / 100.0, latestWristband.accelPeak / 10.0);
    }
    return;
  }

  // TEXT PACKET
  Serial.println();
  Serial.println("================================================");
  Serial.println("              ESP-NOW DATA RECEIVED");
  Serial.println("================================================");

  Serial.print("FROM MAC : ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X%s", info->src_addr[i], (i < 5) ? ":" : "");
  }
  Serial.println();

  char message[250];
  int copyLen = min(len, (int)sizeof(message) - 1);
  memcpy(message, data, copyLen);
  message[copyLen] = '\0';

  Serial.println();
  Serial.println("RAW PACKET:");
  Serial.println(message);

  if (strncmp(message, "TYPE=DATA", 9) == 0) {
    Serial.println();
    Serial.println("--------------- ENV TELEMETRY ---------------");
    parseEnvTokens(message, envDataFields, ENV_DATA_FIELD_COUNT);
    Serial.println("----------------------------------------------");

  } else if (strncmp(message, "TYPE=STATUS", 11) == 0) {
    Serial.println();
    Serial.println("--------------- ENV STATUS ----------------");
    parseEnvTokens(message, envStatusFields, ENV_STATUS_FIELD_COUNT);
    Serial.println("--------------------------------------------");

  } else {
    Serial.print("MESSAGE     : ");
    Serial.println(message);
  }

  Serial.println("================================================");
}


// =====================================================
// ESP-NOW SEND CALLBACK
// =====================================================
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.printf("[ESP-NOW] SEND STATUS: %s\n", status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAILED");
}


// =====================================================
// ESP-NOW PEER MANAGEMENT
// =====================================================
bool addPeer(const uint8_t *mac, const char *label) {
  if (esp_now_is_peer_exist(mac)) {
    Serial.printf("%s PEER ALREADY EXISTS\n", label);
    return true;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  esp_err_t result = esp_now_add_peer(&peerInfo);
  if (result == ESP_OK) {
    Serial.printf("%s PEER ADDED\n", label);
    return true;
  }

  Serial.printf("FAILED TO ADD %s PEER: %d\n", label, result);
  return false;
}

inline bool addEnvPeer()     { return addPeer(ENV_MAC, "ENV"); }
inline bool addESP8266Peer() { return addPeer(ESP8266_MAC, "ESP8266"); }


// =====================================================
// ESP-NOW SEND HELPERS
// =====================================================
void sendMessage(const uint8_t *mac, const char *label, const char *message) {
  if (message == nullptr) return;

  esp_err_t result = esp_now_send(mac, (const uint8_t *)message, strlen(message) + 1);
  if (result == ESP_OK) {
    Serial.printf("[ESP-NOW] SENT%s: %s\n", label, message);
  } else {
    Serial.printf("[ESP-NOW] SEND%s ERROR: %d\n", label, result);
  }
}

inline void sendToENV(const char *message)     { sendMessage(ENV_MAC, "", message); }
inline void sendToESP8266(const char *message) { sendMessage(ESP8266_MAC, " TO ESP8266", message); }


// =====================================================
// SERIAL COMMAND PROCESSOR
// =====================================================
void processCommand(char *cmd) {
  if (cmd == nullptr) return;

  for (int i = 0; cmd[i]; i++) {
    if (cmd[i] == '\r' || cmd[i] == '\n') cmd[i] = '\0';
    if (cmd[i] >= 'a' && cmd[i] <= 'z') cmd[i] -= 32;
  }

  if (strlen(cmd) == 0) return;

  Serial.printf("[COMMAND] %s\n", cmd);

  if (strcmp(cmd, "PING") == 0) {
    sendToENV("PING");
    sendToESP8266("PING");

  } else if (strcmp(cmd, "STATUS") == 0) {
    sendToENV("STATUS");

  } else if (strcmp(cmd, "DATA") == 0) {
    sendToENV("DATA");

  } else if (strcmp(cmd, "SCAN") == 0) {
    sendToENV("SCAN");

  } else if (strcmp(cmd, "GSM_CONNECTED") == 0) {
    gsmConnected = true;
    Serial.println("[GSM] Connection established");

  } else if (strcmp(cmd, "GSM_DISCONNECTED") == 0) {
    gsmConnected = false;
    Serial.println("[GSM] Connection lost");

  } else if (strcmp(cmd, "HELP") == 0) {
    Serial.println();
    Serial.println("AVAILABLE COMMANDS");
    Serial.println("------------------");
    Serial.println("PING");
    Serial.println("STATUS");
    Serial.println("DATA");
    Serial.println("SCAN");
    Serial.println("GSM_CONNECTED");
    Serial.println("GSM_DISCONNECTED");
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

  // INDICATOR LED GPIO
  for (uint8_t i = 0; i < INDICATOR_LED_COUNT; i++) pinMode(indicatorLEDs[i], OUTPUT);
  setAllIndicatorLEDs(LOW);
  Serial.println("[LED] Indicator LEDs initialized");

  // START INDICATOR SEQUENCE
  startIndicatorStartup();

  // ECG GPIO
  pinMode(ECG_LO_PLUS, INPUT);
  pinMode(ECG_LO_MINUS, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(ECG_PIN, ADC_11db);

  // BP RELAY
  pinMode(BP_RELAY_PIN, OUTPUT);
  digitalWrite(BP_RELAY_PIN, LOW);

  // OXYGEN
  pinMode(O2_POT_PIN, INPUT);
  pinMode(O2_RELAY_PIN, OUTPUT);
  digitalWrite(O2_RELAY_PIN, LOW);

  // BUTTONS
  pinMode(PIN_BTN_UP, INPUT_PULLUP);
  pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
  pinMode(PIN_BTN_SELECT, INPUT_PULLUP);
  pinMode(PIN_BTN_BACK, INPUT_PULLUP);

  Serial.println("[NAV] Buttons initialized");
  Serial.println("[BP] BP relay initialized");
  Serial.println("[O2] Oxygen relay OFF at boot");

  // OLED
  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 OLED NOT FOUND!");
    while (true) delay(1000);
  }

  display.clearDisplay();
  display.display();
  randomSeed(esp_random());

  // OLED STARTUP ANIMATIONS
  showVentwiseFade();
  showCircuitStormByText();
  playCircuitStormIntro();

  // SYSTEM INFORMATION
  Serial.println();
  Serial.println();
  Serial.println("==============================================");
  Serial.println("        VENTWISE V3 ESP32 MASTER / OBC");
  Serial.println("        ESP32 DOIT DEVKIT V1");
  Serial.println("==============================================");

  // WIFI
  WiFi.mode(WIFI_STA);
  delay(100);

  Serial.printf("ACTUAL ESP32 MAC: %s\n", WiFi.macAddress().c_str());
  Serial.println("CONFIGURED MASTER MAC: 8C:94:DF:6D:86:F4");
  Serial.println("CONFIGURED ENV MAC   : 00:70:07:E2:22:E0");
  Serial.println("CONFIGURED ESP8266 MAC: 40:91:51:58:D3:33");

  // ESP-NOW
  Serial.println();
  Serial.println("Initializing ESP-NOW...");

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW INIT FAILED");
  } else {
    Serial.println("ESP-NOW INIT OK");

    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);

    if (!addEnvPeer())     Serial.println("ENV PEER ADD FAILED");     else Serial.println("ENV PEER READY");
    if (!addESP8266Peer()) Serial.println("ESP8266 PEER ADD FAILED"); else Serial.println("ESP8266 PEER READY");

    delay(500);

    sendToENV("TYPE=COMMAND;CMD=MASTER_READY");
    sendToESP8266("TYPE=COMMAND;CMD=MASTER_READY");
  }

  // MASTER READY
  Serial.println();
  Serial.println("==============================================");
  Serial.println("             MASTER READY");
  Serial.println("==============================================");

  // INITIAL OLED MENU
  currentScreen = SCREEN_MENU;
  screenNeedsRedraw = true;
  drawMenuScreen();
  screenNeedsRedraw = false;
}


// =====================================================
// LOOP
// =====================================================
void loop() {

  // SERIAL COMMAND PROCESSING
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
    } else if (index < sizeof(serialBuffer) - 1) {
      serialBuffer[index++] = c;
    }
  }

  oxygenUpdate();
  updateIndicatorStartup();
  handleNavigation();
  updateActiveScreenLogic();

  if (currentScreen != SCREEN_ECG) delay(5);
}
