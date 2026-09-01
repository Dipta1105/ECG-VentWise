#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WebSocketsServer.h>

// =====================================================
// VENTWISE V3 - MASTER / OBC (CONSOLIDATED FIRMWARE)
// BOARD: ESP32 DOIT DEVKIT V1
// =====================================================
//
// This file merges the following standalone sketches into
// one master firmware, driven by a 4-button OLED nav menu:
//   - ecg.ino            (AD8232 ECG acquisition + waveform)
//   - BP_relaylogic.ino  (BP measurement relay trigger)
//   - oxygen.ino         (potentiometer-controlled O2 relay)
//
// All sampling rates, filter coefficients, thresholds, and
// relay timings from those sketches are preserved exactly.
// The BP relay pulse was converted from a blocking delay()
// sequence to a non-blocking millis() state machine so the
// nav menu / buttons stay responsive during a BP cycle and
// BACK can interrupt it -- the ON duration (2000 ms) itself
// is unchanged.
//
// =====================================================
// PIN MAP -- CONFLICTS RESOLVED FOR THIS BOARD
// =====================================================
// The source sketches were each written assuming they run
// alone on their own ESP32, so several of them reused the
// same GPIO numbers for different jobs. Now that ECG, BP,
// and Oxygen all run on ONE physical master board alongside
// the OLED + 4 nav buttons, those clashes had to be resolved:
//
//   GPIO21 -> OLED SDA                    (unchanged)
//   GPIO22 -> OLED SCL                    (unchanged)
//   GPIO34 -> AD8232 OUT (ECG analog in)  (unchanged, input-only pin)
//   GPIO27 -> AD8232 LO+ (leads-off)      (was GPIO32 in ecg.ino -- 32 is now the UP button)
//   GPIO14 -> AD8232 LO- (leads-off)      (was GPIO33 in ecg.ino -- 33 is now the DOWN button)
//   GPIO18 -> Oxygen solenoid relay       (was GPIO25 in oxygen.ino -- 25 is now the SELECT button)
//   GPIO35 -> Potentiometer (O2 rate)     (was GPIO34 in oxygen.ino -- 34 is now the ECG signal pin)
//   GPIO19 -> BP relay                    (was GPIO25 in BP_relaylogic.ino -- 25 is now SELECT;
//                                           19 was free on this board and is output-capable)
//   GPIO32 -> UP button
//   GPIO33 -> DOWN button
//   GPIO25 -> SELECT button
//   GPIO26 -> BACK button
//
// Only wiring/pin assignment changed for ECG/BP/Oxygen -- the
// actual sampling code, filter math, thresholds, and relay
// on/off timing are untouched from the source files.
// =====================================================

// =====================================================
// OLED DISPLAY CONFIGURATION
// =====================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// =====================================================
// MAC ADDRESSES (unchanged from masterwithoutnavbutton.ino)
// =====================================================
uint8_t MASTER_MAC[] = {
  0x8C, 0x94, 0xDF, 0x6D, 0x86, 0xF4
};

uint8_t ENV_MAC[] = {
  0x00, 0x70, 0x07, 0xE2, 0x22, 0xE0
};

uint8_t ESP8266_MAC[] = {
  0x40, 0x91, 0x51, 0x58, 0xD3, 0x33
};

// =====================================================
// NAV BUTTON PINS
// =====================================================
#define PIN_BTN_UP     32
#define PIN_BTN_DOWN   33
#define PIN_BTN_SELECT 25
#define PIN_BTN_BACK   26

#define BTN_DEBOUNCE_MS 40

struct Button {
  uint8_t pin;
  bool lastReading;
  bool stableState;
  unsigned long lastChangeTime;
};

Button btnUp     = { PIN_BTN_UP,     HIGH, HIGH, 0 };
Button btnDown   = { PIN_BTN_DOWN,   HIGH, HIGH, 0 };
Button btnSelect = { PIN_BTN_SELECT, HIGH, HIGH, 0 };
Button btnBack   = { PIN_BTN_BACK,   HIGH, HIGH, 0 };

// Returns true exactly once on the frame a button transitions HIGH -> LOW (press)
bool buttonPressed(Button &b) {
  bool reading = digitalRead(b.pin);

  if (reading != b.lastReading) {
    b.lastChangeTime = millis();
  }

  bool firedPress = false;

  if ((millis() - b.lastChangeTime) > BTN_DEBOUNCE_MS) {
    if (reading != b.stableState) {
      b.stableState = reading;

      if (b.stableState == LOW) {
        firedPress = true;
      }
    }
  }

  b.lastReading = reading;

  return firedPress;
}

// =====================================================
// NAV MENU STATE
// =====================================================
enum ScreenState {
  SCREEN_MENU,
  SCREEN_WRISTBAND,
  SCREEN_ECG,
  SCREEN_BP,
  SCREEN_TEMP_CONFIRM,
  SCREEN_TEMP,
  SCREEN_OXYGEN_CONFIRM,
  SCREEN_OXYGEN
};

ScreenState currentScreen = SCREEN_MENU;

const char *menuItems[] = {
  "WRIST BAND",
  "ECG",
  "BP",
  "TEMP",
  "Oxygen"
};

const int MENU_ITEM_COUNT = 5;
int menuIndex = 0;

// Confirmation sub-cursor for the Oxygen off prompt (0 = YES, 1 = NO)
int oxygenConfirmIndex = 0;

bool screenNeedsRedraw = true;

// =====================================================
// WRISTBAND LINK (ESP-NOW)
// =====================================================
// The wristband unit (wristband.ino) transmits this exact
// packed struct over ESP-NOW. Layout must match byte-for-byte.
// Must match wristband.ino byte for byte. Bump WRIST_PACKET_VERSION
// on both sides together whenever this changes -- the version byte
// is what turns a half-flashed pair into a clear log line instead
// of silently misread vitals.
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

unsigned long lastWristbandRxTime = 0;
bool wristbandLinkEverSeen = false;

// The band transmits at 20 Hz. Printing every packet would put the
// master in Serial.print for a large part of every second, so the
// log is throttled and the packet counter carries the rate.
unsigned long wristbandRxCount     = 0;
unsigned long lastWristbandLogTime = 0;
const unsigned long WRISTBAND_LOG_INTERVAL = 1000;

// =====================================================
// ENVIRONMENTAL READINGS - RETAINED
// =====================================================
// The ENV unit's telemetry was parsed straight to Serial and
// dropped -- nothing kept it, so there was nothing to put in a
// status frame. NAN means "never received" and serialises as
// JSON null, which the protocol requires: an absent sensor must
// not reach the phone as a plausible zero.
float envTemp     = NAN;   // deg C
float envHumidity = NAN;   // % RH
float envPressure = NAN;   // hPa
float envAltitude = NAN;   // m
float envLat      = NAN;   // WGS-84
float envLon      = NAN;
int   envSats     = 0;
bool  envGpsFix   = false;
unsigned long lastEnvDataTime = 0;

// Retains one "KEY=VALUE" token while the existing chain below
// prints it. One call at the top of the loop rather than an edit
// to each of ten branches.
void captureEnvToken(const char *tok) {
  if      (!strncmp(tok, "TEMP=",  5)) envTemp     = atof(tok + 5);
  else if (!strncmp(tok, "HUM=",   4)) envHumidity = atof(tok + 4);
  else if (!strncmp(tok, "PRESS=", 6)) envPressure = atof(tok + 6);
  else if (!strncmp(tok, "ALT=",   4)) envAltitude = atof(tok + 4);
  else if (!strncmp(tok, "LAT=",   4)) envLat      = atof(tok + 4);
  else if (!strncmp(tok, "LON=",   4)) envLon      = atof(tok + 4);
  else if (!strncmp(tok, "SAT=",   4)) envSats     = atoi(tok + 4);
  else if (!strncmp(tok, "GPS=",   4)) envGpsFix   = atoi(tok + 4) != 0;
  else return;

  lastEnvDataTime = millis();
}

// Rate-limited so a persistent version mismatch cannot flood the log.
unsigned long lastWristbandRejectLog = 0;

// =====================================================
// LOGO GRAPHIC - 128 x 48 PIXELS (unchanged)
// =====================================================

const uint8_t logoGraphic[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFC, 0x00, 0x1F, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0x04, 0x7F, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x03, 0xCF, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x01, 0xCF, 0xFC, 0xFB, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0x9F, 0x0F, 0x78, 0x7F, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0x78, 0x03, 0xF1, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x06, 0xE0, 0x00, 0xF1, 0xFF, 0x3F, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0D, 0xC0, 0x00, 0x35, 0xFF, 0x3F, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0D, 0x80, 0x00, 0x35, 0xFF, 0x1F, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x1B, 0x00, 0x00, 0x3D, 0xFF, 0x1F, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x1B, 0x00, 0x00, 0x2D, 0xFE, 0x1F, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x36, 0x00, 0x00, 0x2E, 0xFE, 0x1F, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x36, 0x00, 0x00, 0x6E, 0xFE, 0x5F, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x36, 0x00, 0x00, 0x6E, 0xFE, 0x4F, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x36, 0x00, 0x00, 0x6E, 0xFE, 0x4F, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x6E, 0xFE, 0x4F, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x7A, 0xFE, 0xE7, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x3C, 0x00, 0xF8, 0x5A, 0xFC, 0xE7, 0xFF, 0xFE, 0x18, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x3C, 0x01, 0x8C, 0x5A, 0xFC, 0xE6, 0x7F, 0xFE, 0x7C, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x7F, 0xC7, 0x64, 0xDA, 0xFC, 0xF4, 0x7F, 0xFE, 0xC2, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFE, 0xF6, 0xD3, 0xFC, 0xF0, 0x3F, 0xFF, 0xBB, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x03, 0x00, 0x00, 0x01, 0x96, 0xD3, 0x7D, 0xF0, 0x00, 0x00, 0x2D, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x01, 0xFF, 0xFB, 0xFF, 0x1A, 0xD3, 0x79, 0xF9, 0xC2, 0x01, 0x39, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x1B, 0xFC, 0x1A, 0xD3, 0x79, 0xF9, 0xFF, 0xFF, 0x9B, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0D, 0x80, 0x1B, 0xF1, 0x79, 0xFB, 0xFF, 0xFE, 0xC6, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0D, 0x80, 0x0B, 0xB1, 0x39, 0xFF, 0xFF, 0xFE, 0x7C, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x06, 0xC0, 0x0B, 0xB1, 0xB9, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x06, 0xC0, 0x0D, 0xB1, 0xB3, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0x60, 0x0D, 0xB1, 0xB3, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0x30, 0x0D, 0xA0, 0x93, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x01, 0xB8, 0x05, 0xA0, 0xD3, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xD8, 0x05, 0xA0, 0x47, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x6C, 0x05, 0xA0, 0x47, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x76, 0x04, 0x60, 0x47, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0x06, 0x60, 0x47, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x86, 0x60, 0x67, 0xFF, 0xFE, 0xC0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0xC6, 0x60, 0x67, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x66, 0x60, 0x7F, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3A, 0x40, 0x7F, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xDF, 0xC0, 0x7F, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEF, 0x00, 0x7F, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73, 0x01, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0xC7, 0x67, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0xFC, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x39, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xC7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// =====================================================
// PARTICLE SYSTEM (unchanged)
// =====================================================

struct Particle {
  float x;
  float y;
  float vx;
  uint8_t life;
  uint8_t maxLife;
};

Particle particles[28];

void resetParticles() {
  for (int i = 0; i < 28; i++) {
    particles[i].x = random(-30, 10);
    particles[i].y = random(2, 47);
    particles[i].vx = random(8, 22) / 10.0;
    particles[i].life = random(15, 50);
    particles[i].maxLife = particles[i].life;
  }
}

void updateParticles(float sweepX) {
  for (int i = 0; i < 28; i++) {
    particles[i].x += particles[i].vx;

    if (particles[i].x > sweepX + 12 || particles[i].life == 0) {
      particles[i].x = sweepX - random(0, 25);
      particles[i].y = random(2, 47);
      particles[i].vx = random(8, 22) / 10.0;
      particles[i].life = random(18, 50);
      particles[i].maxLife = particles[i].life;
    }

    if (particles[i].life > 0)
      particles[i].life--;
  }
}

void drawGraphic(uint8_t sweepX) {
  for (int y = 0; y < 48; y++) {
    for (int x = 0; x < sweepX; x++) {
      uint8_t b = pgm_read_byte(&logoGraphic[y * 16 + (x >> 3)]);
      if (b & (0x80 >> (x & 7))) {
        display.drawPixel(x, y, SSD1306_WHITE);
      }
    }
  }
}

void drawParticles(float sweepX) {
  for (int i = 0; i < 28; i++) {
    if (particles[i].life == 0)
      continue;

    int x = (int)particles[i].x;
    int y = (int)particles[i].y;

    if (x >= 0 && x < 128 && y >= 0 && y < 48) {
      display.drawPixel(x, y, SSD1306_WHITE);
    }

    if (particles[i].life > particles[i].maxLife / 2) {
      if (x - 1 >= 0) {
        display.drawPixel(x - 1, y, SSD1306_WHITE);
      }
      if (random(0, 3) == 0 && x - 2 >= 0) {
        display.drawPixel(x - 2, y, SSD1306_WHITE);
      }
    }
  }

  if (sweepX >= 0 && sweepX < 128) {
    for (int y = 4; y < 45; y += 3) {
      if (random(0, 4) != 0) {
        display.drawPixel(sweepX, y, SSD1306_WHITE);
      }
    }
  }
}

void drawCircuitStormText(int charsToShow, bool cursorVisible) {
  const char *text = "CIRCUITSTORM";
  const int len = 12;

  const int textWidth = len * 6;
  const int startX = (128 - textWidth) / 2;
  const int textY = 54;

  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(startX, textY);

  for (int i = 0; i < charsToShow && i < len; i++) {
    display.write(text[i]);
  }

  if (cursorVisible && charsToShow < len) {
    display.fillRect(startX + charsToShow * 6, textY + 6, 4, 1, SSD1306_WHITE);
  }
}

void playCircuitStormIntro() {
  resetParticles();

  const unsigned long graphicTime = 1900;

  const char *text = "CIRCUITSTORM";
  const int textLen = 12;

  int typingDelays[] = {
    75, 155, 55, 210, 90, 135, 60, 185, 80, 240, 70, 170
  };

  unsigned long startTime = millis();
  int typedChars = 0;
  unsigned long nextTypeTime = startTime + typingDelays[0];

  while (true) {
    unsigned long now = millis();
    unsigned long elapsed = now - startTime;

    if (typedChars < textLen && now >= nextTypeTime) {
      typedChars++;
      if (typedChars < textLen) {
        nextTypeTime = now + typingDelays[typedChars];
      }
    }

    float t;
    if (elapsed >= graphicTime) {
      t = 1.0;
    } else {
      t = (float)elapsed / graphicTime;
    }

    float eased = 1.0 - pow(1.0 - t, 3.0);
    float sweepX = eased * 128.0;

    display.clearDisplay();

    drawGraphic((uint8_t)sweepX);

    updateParticles(sweepX);
    drawParticles(sweepX);

    bool cursorVisible = ((now / 180) % 2) == 0;
    drawCircuitStormText(typedChars, cursorVisible);

    display.display();

    bool graphicFinished = elapsed >= graphicTime;
    bool textFinished = typedChars >= textLen;

    if (graphicFinished && textFinished) {
      break;
    }

    delay(25);
  }

  display.clearDisplay();

  drawGraphic(128);

  drawCircuitStormText(textLen, false);

  display.display();

  delay(3000);
}

void showVentwiseFade() {
  display.clearDisplay();
  display.display();

  for (int step = 0; step <= 20; step++) {
    display.clearDisplay();

    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 18);

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

void showCircuitStormByText() {
  const char* text = "CIRCUITSTORM";
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

    display.setTextSize(1);
    display.setCursor(20, 36);

    for (int j = 0; j <= i; j++) {
      display.write(text[j]);
    }

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
// ECG MODULE (ported from ecg.ino -- sampling/filter/
// lead-off logic unchanged; only the OLED draw geometry
// is adapted to the master's 128x64 panel instead of the
// standalone unit's 128x32 panel, and calibration is
// re-run each time the ECG screen is entered)
// =====================================================

#define ECG_PIN     34
#define ECG_LO_PLUS 27
#define ECG_LO_MINUS 14

#define ECG_SAMPLE_RATE 250
const unsigned long ECG_SAMPLE_INTERVAL = 1000000UL / ECG_SAMPLE_RATE;
unsigned long ecgLastSampleTime = 0;

#define ECG_DISPLAY_RATE 100
const int ECG_DISPLAY_DIVIDER = ECG_SAMPLE_RATE / ECG_DISPLAY_RATE;
int ecgDisplayCounter = 0;

#define ECG_CALIBRATION_TIME 5000
bool ecgCalibrated = false;
unsigned long ecgCalibrationStart = 0;
float ecgDcBaseline = 0;
float ecgCalibrationSum = 0;
unsigned long ecgCalibrationSamples = 0;

float ecgBaseline = 2048.0;
float ecgFilteredSignal = 0;

// Adapted for the 128x64 master screen (source used 128x32,
// center 16 / amplitude 14). Filter math below is untouched.
#define ECG_CENTER 32
#define ECG_MAX_AMPLITUDE 28

float ecgDisplayScale = 300.0;

int ecgWaveform[SCREEN_WIDTH];
int ecgWritePosition = 0;

float processECG(float raw) {
  // Slow baseline tracker -- removes AD8232 DC offset (unchanged: 0.0015 coefficient)
  ecgBaseline = ecgBaseline + 0.0015f * (raw - ecgBaseline);

  float ac = raw - ecgBaseline;

  // Gentle low-pass smoothing (unchanged: 0.45 coefficient)
  ecgFilteredSignal = ecgFilteredSignal + 0.45f * (ac - ecgFilteredSignal);

  return ecgFilteredSignal;
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

  for (int i = 0; i < SCREEN_WIDTH; i++) {
    ecgWaveform[i] = ECG_CENTER;
  }

  Serial.println("[ECG] Session start -- calibrating (5s), keep electrodes on and stay still");
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
    if (leadOff) {
      display.println("LEAD OFF!");
    } else {
      display.println("Stay still...");
    }

    display.setCursor(0, 28);
    display.print(seconds);
    display.println(" / 5 sec");

    display.setCursor(0, 54);
    display.println("BACK: menu");

    display.display();
  }

  if (elapsed >= ECG_CALIBRATION_TIME) {
    ecgDcBaseline = ecgCalibrationSum / ecgCalibrationSamples;
    ecgBaseline = ecgDcBaseline;
    ecgFilteredSignal = 0;
    ecgCalibrated = true;

    for (int i = 0; i < SCREEN_WIDTH; i++) {
      ecgWaveform[i] = ECG_CENTER;
    }

    ecgWritePosition = 0;
    ecgDisplayScale = 300;

    Serial.print("[ECG] Calibration complete, baseline=");
    Serial.println(ecgDcBaseline);
  }
}

void ecgDrawWaveform() {
  display.clearDisplay();

  for (int x = 0; x < SCREEN_WIDTH; x += 8) {
    display.drawPixel(x, ECG_CENTER, SSD1306_WHITE);
  }

  for (int x = 1; x < SCREEN_WIDTH; x++) {
    int index1 = (ecgWritePosition + x - 1) % SCREEN_WIDTH;
    int index2 = (ecgWritePosition + x) % SCREEN_WIDTH;

    int y1 = ecgWaveform[index1];
    int y2 = ecgWaveform[index2];

    display.drawLine(x - 1, y1, x, y2, SSD1306_WHITE);
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

// Raw samples for the phone, filled at the full 250 Hz and
// flushed as one `ecg` frame every 62 samples (4 Hz). Tapped
// before every early return below: the phone needs the samples
// the OLED does not draw, so display decimation must not also
// decimate the recording.
constexpr int ECG_TX_BATCH = ECG_SAMPLE_RATE / 4;   // 62
uint16_t ecgTxBuffer[ECG_TX_BATCH];
int ecgTxCount = 0;
bool ecgTxReady = false;
bool ecgTxLeadOff = false;
unsigned long ecgTxFirstSampleMs = 0;
uint32_t ecgTxSeq = 0;

void ecgProcessSample() {
  int raw = analogRead(ECG_PIN);

  bool leadOff = digitalRead(ECG_LO_PLUS) || digitalRead(ECG_LO_MINUS);

  // ---- phone tap ----
  if (!ecgTxReady) {
    if (ecgTxCount == 0) {
      ecgTxFirstSampleMs = millis();
      ecgTxLeadOff = false;
    }
    ecgTxBuffer[ecgTxCount++] = (uint16_t)raw;
    if (leadOff) ecgTxLeadOff = true;
    if (ecgTxCount >= ECG_TX_BATCH) ecgTxReady = true;
  }

  if (!ecgCalibrated) {
    ecgCalibrate(raw, leadOff);
    return;
  }

  float ecg = processECG((float)raw);

  if (leadOff) {
    Serial.println("[ECG] lead-off detected, signal=0");
  }

  ecgDisplayCounter++;

  if (ecgDisplayCounter < ECG_DISPLAY_DIVIDER) {
    return;
  }

  ecgDisplayCounter = 0;

  if (leadOff) {
    ecgDrawLeadOff();
    return;
  }

  float magnitude = fabs(ecg);

  if (magnitude > ecgDisplayScale) {
    ecgDisplayScale = magnitude * 1.15f;
  } else {
    ecgDisplayScale *= 0.9995f;
  }

  if (ecgDisplayScale < 100) {
    ecgDisplayScale = 100;
  }

  if (ecgDisplayScale > 1000) {
    ecgDisplayScale = 1000;
  }

  float normalized = ecg / ecgDisplayScale;
  normalized = constrain(normalized, -1.0f, 1.0f);

  int y = ECG_CENTER - (normalized * ECG_MAX_AMPLITUDE);
  y = constrain(y, 1, SCREEN_HEIGHT - 2);

  ecgWaveform[ecgWritePosition] = y;
  ecgWritePosition++;

  if (ecgWritePosition >= SCREEN_WIDTH) {
    ecgWritePosition = 0;
  }

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
// BP MODULE (ported from BP_relaylogic.ino -- relay ON
// duration of 2000 ms is unchanged. Converted from a
// blocking delay(2000) pulse to a non-blocking millis()
// state machine so BACK can stop the cycle immediately
// and the nav menu stays responsive during the pulse.)
// =====================================================

#define BP_RELAY_PIN 19
#define BP_PULSE_DURATION_MS 2000

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

  if (bpRelayActive) {
    Serial.println("[BP] Relay OFF -- unit powered off (BACK pressed)");
  }

  bpRelayActive = false;
}

void bpUpdate() {
  if (!bpRelayActive) {
    return;
  }

  if (millis() - bpPulseStartTime >= BP_PULSE_DURATION_MS) {
    digitalWrite(BP_RELAY_PIN, LOW);
    bpRelayActive = false;

    Serial.println("[BP] Relay OFF -- measurement cycle complete, ready for next command");
  }
}

// =====================================================
// OXYGEN MODULE (ported from oxygen.ino -- potentiometer
// mapping and MIN/MAX relay-toggle intervals unchanged.
// Runs continuously in the background once activated, per
// the "supply persists across menu navigation" spec, so it
// is updated every loop() iteration regardless of screen.)
// =====================================================

#define O2_POT_PIN   35
#define O2_RELAY_PIN 18

const unsigned long O2_MIN_INTERVAL = 500;
const unsigned long O2_MAX_INTERVAL = 5000;

bool oxygenSupplyActive = false;   // persistent flag: relay/toggle logic runs in background
bool o2RelayState = LOW;
unsigned long o2PreviousMillis = 0;
unsigned long o2LastPrint = 0;
unsigned long o2CurrentInterval = O2_MAX_INTERVAL;
int o2LastPotValue = 0;

void oxygenTurnOn() {
  oxygenSupplyActive = true;
  o2PreviousMillis = millis();
  Serial.println("[O2] Oxygen supply ON");
}

void oxygenTurnOff() {
  oxygenSupplyActive = false;
  o2RelayState = LOW;
  digitalWrite(O2_RELAY_PIN, LOW);
  Serial.println("[O2] Oxygen supply OFF (relay de-energized)");
}

void oxygenUpdate() {
  if (!oxygenSupplyActive) {
    return;
  }

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

    Serial.print("[O2] Pot: ");
    Serial.print(potValue);
    Serial.print(" / 4095 | Interval: ");
    Serial.print(interval);
    Serial.print(" ms | Relay: ");
    Serial.println(o2RelayState ? "ON" : "OFF");
  }
}

// =====================================================
// NAV MENU / SCREEN DRAWING
// =====================================================

void drawHeader(const char *title) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(title);
  display.drawLine(0, 9, SCREEN_WIDTH - 1, 9, SSD1306_WHITE);
}

void drawMenuScreen() {
  display.clearDisplay();
  drawHeader("VENTWISE - MAIN MENU");

  for (int i = 0; i < MENU_ITEM_COUNT; i++) {
    int y = 13 + (i * 10);

    display.setCursor(0, y);

    if (i == menuIndex) {
      display.print("> ");
    } else {
      display.print("  ");
    }

    display.print(i + 1);
    display.print(". ");
    display.println(menuItems[i]);
  }

  display.display();
}

void drawWristbandScreen() {
  display.clearDisplay();
  drawHeader("WRIST BAND");

  display.setCursor(0, 16);

  if (!wristbandLinkEverSeen) {
    display.println("Waiting for link...");
  } else {
    display.setTextSize(1);
    display.setCursor(0, 16);
    display.print("HR   : ");

    if (latestWristband.fingerDetected) {
      display.print(latestWristband.heartRate);
      display.println(" BPM");
    } else {
      display.println("--");
    }

    display.setCursor(0, 30);
    display.print("SpO2 : ");

    if (latestWristband.fingerDetected) {
      display.print(latestWristband.spo2);
      display.println(" %");
    } else {
      display.println("--");
    }

    display.setCursor(0, 44);

    if (!latestWristband.fingerDetected) {
      display.println("Finger not detected");
    } else if (millis() - lastWristbandRxTime > 5000) {
      display.println("Link stale");
    } else {
      display.println("Link OK");
    }
  }

  display.setCursor(0, 56);
  display.println("BACK: menu");

  display.display();
}

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

void drawOxygenConfirmScreen() {
  display.clearDisplay();
  drawHeader("OXYGEN SUPPLY");

  display.setCursor(0, 18);
  display.println("Turn off oxygen");
  display.setCursor(0, 28);
  display.println("supply?");

  display.setCursor(10, 42);
  display.print(oxygenConfirmIndex == 0 ? "> " : "  ");
  display.println("YES");

  display.setCursor(10, 52);
  display.print(oxygenConfirmIndex == 1 ? "> " : "  ");
  display.println("NO");

  display.display();
}

void drawOxygenScreen() {
  display.clearDisplay();
  drawHeader("OXYGEN - RUNNING");

  display.setCursor(0, 16);
  display.print("Rate int: ");
  display.print(o2CurrentInterval);
  display.println(" ms");

  display.setCursor(0, 30);
  display.print("SpO2    : ");

  if (wristbandLinkEverSeen && latestWristband.fingerDetected) {
    display.print(latestWristband.spo2);
    display.println(" %");
  } else {
    display.println("--");
  }

  display.setCursor(0, 44);
  display.print("Relay: ");
  display.println(o2RelayState ? "ON" : "OFF");

  display.setCursor(0, 56);
  display.println("BACK: menu (stays on)");

  display.display();
}

// =====================================================
// NAV MENU / SCREEN LOGIC
// =====================================================

void enterMenu() {
  currentScreen = SCREEN_MENU;
  screenNeedsRedraw = true;
  Serial.println("[NAV] -> MAIN MENU");
}

void selectMenuItem() {
  switch (menuIndex) {

    case 0: // WRIST BAND
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

    case 3: // TEMP
      currentScreen = SCREEN_TEMP_CONFIRM;
      Serial.println("[NAV] -> TEMP (confirm probe)");
      break;

    case 4: // Oxygen
      if (oxygenSupplyActive) {
        oxygenConfirmIndex = 1; // default to NO so a stray press can't shut it off
        currentScreen = SCREEN_OXYGEN_CONFIRM;
        Serial.println("[NAV] -> OXYGEN (already ON, confirm off?)");
      } else {
        oxygenTurnOn();
        currentScreen = SCREEN_OXYGEN;
        Serial.println("[NAV] -> OXYGEN (turning on)");
      }
      break;
  }

  screenNeedsRedraw = true;
}

void handleNavigation() {
  bool up = buttonPressed(btnUp);
  bool down = buttonPressed(btnDown);
  bool select = buttonPressed(btnSelect);
  bool back = buttonPressed(btnBack);

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
      if (back) {
        enterMenu();
      }
      break;

    case SCREEN_ECG:
      if (back) {
        Serial.println("[ECG] Session stopped (BACK pressed)");
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
        Serial.println("[NAV] TEMP probe confirmed, showing live reading");
      } else if (back) {
        enterMenu();
      }
      break;

    case SCREEN_TEMP:
      if (back) {
        enterMenu();
      }
      break;

    case SCREEN_OXYGEN_CONFIRM:
      if (up || down) {
        oxygenConfirmIndex = 1 - oxygenConfirmIndex;
        screenNeedsRedraw = true;
      } else if (select) {
        if (oxygenConfirmIndex == 0) {
          // YES -> turn off, return to main menu
          oxygenTurnOff();
          enterMenu();
        } else {
          // NO -> dismiss prompt, go straight to the live oxygen screen (stays ON)
          currentScreen = SCREEN_OXYGEN;
          screenNeedsRedraw = true;
          Serial.println("[NAV] Oxygen off-request dismissed, supply remains ON");
        }
      } else if (back) {
        // Treat BACK on the confirm prompt as dismiss-and-return, supply stays ON
        enterMenu();
      }
      break;

    case SCREEN_OXYGEN:
      if (back) {
        // Per spec: BACK leaves oxygen relay ON, only returns to menu
        enterMenu();
      }
      break;
  }
}

void updateActiveScreenLogic() {
  switch (currentScreen) {

    case SCREEN_ECG:
      ecgUpdate();
      // ecgUpdate() redraws the OLED itself on every new sample,
      // so no periodic redraw is needed here.
      return;

    case SCREEN_BP: {
      bool wasActive = bpRelayActive;
      bpUpdate();

      // If the BP cycle finishes on its own (not via BACK), return to menu.
      if (wasActive && !bpRelayActive) {
        enterMenu();
        return;
      }
      break;
    }

    default:
      break;
  }

  // Periodic redraw for screens showing live values
  static unsigned long lastPeriodicRedraw = 0;
  bool isLiveScreen = (currentScreen == SCREEN_WRISTBAND ||
                       currentScreen == SCREEN_TEMP ||
                       currentScreen == SCREEN_OXYGEN);

  if (screenNeedsRedraw || (isLiveScreen && millis() - lastPeriodicRedraw > 300)) {
    lastPeriodicRedraw = millis();
    screenNeedsRedraw = false;

    switch (currentScreen) {
      case SCREEN_MENU:            drawMenuScreen(); break;
      case SCREEN_WRISTBAND:       drawWristbandScreen(); break;
      case SCREEN_BP:              drawBPScreen(); break;
      case SCREEN_TEMP_CONFIRM:    drawTempConfirmScreen(); break;
      case SCREEN_TEMP:            drawTempScreen(); break;
      case SCREEN_OXYGEN_CONFIRM:  drawOxygenConfirmScreen(); break;
      case SCREEN_OXYGEN:          drawOxygenScreen(); break;
      default: break;
    }
  }
}

// =====================================================
// ESP-NOW RECEIVE CALLBACK
// =====================================================

void onDataRecv(
  const esp_now_recv_info_t *info,
  const uint8_t *data,
  int len
) {

  if (data == nullptr || len <= 0)
    return;

  // ---------------------------------------------------
  // WRISTBAND BINARY PACKET
  // ---------------------------------------------------
  // wristband.ino sends a fixed-size packed struct, not a
  // null-terminated command string -- detect it by exact
  // size before falling through to the text-based parser
  // below (which is unchanged from masterwithoutnavbutton.ino).
  // Identified by MAGIC and VERSION, not by length. Length alone
  // meant any ESP-NOW sender that happened to emit a payload of
  // this size was accepted as patient vitals, and a struct change
  // on one board only fell through to the text parser below and
  // read raw binary as an ASCII command.
  if (len == sizeof(WristbandPacket)) {
    WristbandPacket incoming;
    memcpy(&incoming, data, sizeof(WristbandPacket));

    if (incoming.magic != WRIST_PACKET_MAGIC ||
        incoming.version != WRIST_PACKET_VERSION) {

      unsigned long nowMs = millis();
      if (nowMs - lastWristbandRejectLog > 2000) {
        lastWristbandRejectLog = nowMs;

        Serial.print("[WRIST] REJECTED packet: magic=0x");
        Serial.print(incoming.magic, HEX);
        Serial.print(" version=");
        Serial.print(incoming.version);
        Serial.print(" (expected magic=0x");
        Serial.print(WRIST_PACKET_MAGIC, HEX);
        Serial.print(" version=");
        Serial.print(WRIST_PACKET_VERSION);
        Serial.println("). Reflash both boards.");
      }
      return;
    }

    latestWristband = incoming;
    lastWristbandRxTime = millis();
    wristbandLinkEverSeen = true;
    wristbandRxCount++;

    unsigned long nowMs = millis();
    if (nowMs - lastWristbandLogTime >= WRISTBAND_LOG_INTERVAL) {
      lastWristbandLogTime = nowMs;

      Serial.print("[WRIST] RX seq=");
      Serial.print(latestWristband.seq);
      Serial.print(" n=");
      Serial.print(wristbandRxCount);
      Serial.print(" HR=");
      Serial.print(latestWristband.heartRate);
      Serial.print(" SpO2=");
      Serial.print(latestWristband.spo2);
      Serial.print(" Temp=");
      Serial.print(latestWristband.temperatureF);
      Serial.print("F Finger=");
      Serial.print(latestWristband.fingerDetected);
      Serial.print(" Fall=");
      Serial.print(latestWristband.fallDetected);
      Serial.print(" Panic=");
      Serial.print(latestWristband.panicPressed);

      Serial.print(" | accel=");
      Serial.print(latestWristband.accel / 10.0, 2);
      Serial.print(" [x ");
      Serial.print(latestWristband.ax / 100.0, 2);
      Serial.print(" y ");
      Serial.print(latestWristband.ay / 100.0, 2);
      Serial.print(" z ");
      Serial.print(latestWristband.az / 100.0, 2);
      Serial.print("] peak=");
      Serial.print(latestWristband.accelPeak / 10.0, 2);

      Serial.print(" | health:");
      Serial.print((latestWristband.sensorHealth & HEALTH_PULSEOX) ? " pulseox" : " NO-PULSEOX");
      Serial.print((latestWristband.sensorHealth & HEALTH_ACCEL)   ? " accel"   : " NO-ACCEL");
      Serial.println((latestWristband.sensorHealth & HEALTH_TEMP)  ? " temp"    : " NO-TEMP");
    }

    return;
  }

  Serial.println();
  Serial.println("================================================");
  Serial.println("              ESP-NOW DATA RECEIVED");
  Serial.println("================================================");

  Serial.print("FROM MAC : ");

  for (int i = 0; i < 6; i++) {

    if (info->src_addr[i] < 16)
      Serial.print("0");

    Serial.print(info->src_addr[i], HEX);

    if (i < 5)
      Serial.print(":");
  }

  Serial.println();

  char message[250];

  int copyLen = len;

  if (copyLen >= sizeof(message))
    copyLen = sizeof(message) - 1;

  memcpy(message, data, copyLen);

  message[copyLen] = '\0';

  Serial.println();
  Serial.println("RAW PACKET:");
  Serial.println(message);

  if (strncmp(message, "TYPE=DATA", 9) == 0) {

    Serial.println();
    Serial.println("--------------- ENV TELEMETRY ---------------");

    char *token = strtok(message, ";");

    while (token != nullptr) {

      captureEnvToken(token);

      if (strncmp(token, "TYPE=", 5) == 0) {

        Serial.print("TYPE        : ");
        Serial.println(token + 5);

      } else if (strncmp(token, "TEMP=", 5) == 0) {

        Serial.print("TEMPERATURE : ");
        Serial.print(token + 5);
        Serial.println(" C");

      } else if (strncmp(token, "HUM=", 4) == 0) {

        Serial.print("HUMIDITY    : ");
        Serial.print(token + 4);
        Serial.println(" %");

      } else if (strncmp(token, "PRESS=", 6) == 0) {

        Serial.print("PRESSURE    : ");
        Serial.print(token + 6);
        Serial.println(" hPa");

      } else if (strncmp(token, "ALT=", 4) == 0) {

        Serial.print("ALTITUDE    : ");
        Serial.print(token + 4);
        Serial.println(" m");

      } else if (strncmp(token, "ACCX=", 5) == 0) {

        Serial.print("ACCEL X     : ");
        Serial.print(token + 5);
        Serial.println(" m/s2");

      } else if (strncmp(token, "ACCY=", 5) == 0) {

        Serial.print("ACCEL Y     : ");
        Serial.print(token + 5);
        Serial.println(" m/s2");

      } else if (strncmp(token, "ACCZ=", 5) == 0) {

        Serial.print("ACCEL Z     : ");
        Serial.print(token + 5);
        Serial.println(" m/s2");

      } else if (strncmp(token, "LAT=", 4) == 0) {

        Serial.print("LATITUDE    : ");
        Serial.println(token + 4);

      } else if (strncmp(token, "LON=", 4) == 0) {

        Serial.print("LONGITUDE   : ");
        Serial.println(token + 4);

      } else if (strncmp(token, "SAT=", 4) == 0) {

        Serial.print("SATELLITES  : ");
        Serial.println(token + 4);

      } else if (strncmp(token, "DHT=", 4) == 0) {

        Serial.print("DHT22       : ");
        Serial.println(atoi(token + 4) ? "OK" : "FAIL");

      } else if (strncmp(token, "BMP=", 4) == 0) {

        Serial.print("BMP280      : ");
        Serial.println(atoi(token + 4) ? "OK" : "FAIL");

      } else if (strncmp(token, "LIS=", 4) == 0) {

        Serial.print("LIS3DH      : ");
        Serial.println(atoi(token + 4) ? "OK" : "FAIL");

      } else if (strncmp(token, "GPS=", 4) == 0) {

        Serial.print("GNSS        : ");
        Serial.println(atoi(token + 4) ? "FIX" : "NO FIX");
      }

      token = strtok(nullptr, ";");
    }

    Serial.println("----------------------------------------------");

  } else if (strncmp(message, "TYPE=STATUS", 11) == 0) {

    Serial.println();
    Serial.println("--------------- ENV STATUS ----------------");

    char *token = strtok(message, ";");

    while (token != nullptr) {

      captureEnvToken(token);

      if (strncmp(token, "TYPE=", 5) == 0) {

        Serial.print("TYPE        : ");
        Serial.println(token + 5);

      } else if (strncmp(token, "DHT=", 4) == 0) {

        Serial.print("DHT22       : ");
        Serial.println(atoi(token + 4) ? "OK" : "FAIL");

      } else if (strncmp(token, "BMP=", 4) == 0) {

        Serial.print("BMP280      : ");
        Serial.println(atoi(token + 4) ? "OK" : "FAIL");

      } else if (strncmp(token, "LIS=", 4) == 0) {

        Serial.print("LIS3DH      : ");
        Serial.println(atoi(token + 4) ? "OK" : "FAIL");

      } else if (strncmp(token, "GPS=", 4) == 0) {

        Serial.print("GNSS        : ");
        Serial.println(atoi(token + 4) ? "FIX" : "NO FIX");

      } else if (strncmp(token, "SAT=", 4) == 0) {

        Serial.print("SATELLITES  : ");
        Serial.println(token + 4);
      }

      token = strtok(nullptr, ";");
    }

    Serial.println("--------------------------------------------");

  } else {

    Serial.print("MESSAGE     : ");
    Serial.println(message);
  }

  Serial.println("================================================");
}

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

bool addEnvPeer() {

  if (esp_now_is_peer_exist(ENV_MAC)) {

    Serial.println("ENV PEER ALREADY EXISTS");

    return true;
  }

  esp_now_peer_info_t peerInfo = {};

  memcpy(peerInfo.peer_addr, ENV_MAC, 6);

  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  esp_err_t result =
    esp_now_add_peer(&peerInfo);

  if (result == ESP_OK) {

    Serial.println("ENV PEER ADDED");

    return true;
  }

  Serial.print("FAILED TO ADD ENV PEER: ");
  Serial.println(result);

  return false;
}

bool addESP8266Peer() {

  if (esp_now_is_peer_exist(ESP8266_MAC)) {

    Serial.println("ESP8266 PEER ALREADY EXISTS");

    return true;
  }

  esp_now_peer_info_t peerInfo = {};

  memcpy(peerInfo.peer_addr, ESP8266_MAC, 6);

  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  esp_err_t result =
    esp_now_add_peer(&peerInfo);

  if (result == ESP_OK) {

    Serial.println("ESP8266 PEER ADDED");

    return true;
  }

  Serial.print("FAILED TO ADD ESP8266 PEER: ");
  Serial.println(result);

  return false;
}

void sendToENV(const char *message) {

  if (message == nullptr)
    return;

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

void sendToESP8266(const char *message) {

  if (message == nullptr)
    return;

  esp_err_t result = esp_now_send(
    ESP8266_MAC,
    (const uint8_t *)message,
    strlen(message) + 1
  );

  if (result == ESP_OK) {

    Serial.print("[ESP-NOW] SENT TO ESP8266: ");
    Serial.println(message);

  } else {

    Serial.print("[ESP-NOW] SEND TO ESP8266 ERROR: ");
    Serial.println(result);
  }
}

void processCommand(char *cmd) {

  if (cmd == nullptr)
    return;

  for (int i = 0; cmd[i]; i++) {

    if (cmd[i] == '\r' || cmd[i] == '\n')
      cmd[i] = '\0';

    if (cmd[i] >= 'a' && cmd[i] <= 'z')
      cmd[i] -= 32;
  }

  if (strlen(cmd) == 0)
    return;

  Serial.print("[COMMAND] ");
  Serial.println(cmd);

  if (strcmp(cmd, "PING") == 0) {

    sendToENV("PING");
    sendToESP8266("PING");

  } else if (strcmp(cmd, "STATUS") == 0) {

    sendToENV("STATUS");

  } else if (strcmp(cmd, "DATA") == 0) {

    sendToENV("DATA");

  } else if (strcmp(cmd, "SCAN") == 0) {

    sendToENV("SCAN");

  } else if (strcmp(cmd, "HELP") == 0) {

    Serial.println();
    Serial.println("AVAILABLE COMMANDS");
    Serial.println("------------------");
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

// =====================================================
// PHONE BRIDGE - SoftAP + WebSocket + JSON
// =====================================================
// Implements docs/protocol.md v2 from the app repository:
// the box is the server, the phone joins its SoftAP and
// opens ws://192.168.4.1:81/ws.
//
// JSON is built with snprintf rather than ArduinoJson. Every
// frame here has a fixed shape and only ever contains numbers,
// booleans and null, so there is nothing to escape -- and the
// ecg frame carries 62 samples four times a second, which is
// exactly the sort of repeated allocation worth not doing on
// a board that is also driving an OLED and an ADC.

constexpr uint16_t WS_PORT       = 81;
constexpr uint8_t  SOFTAP_CHANNEL = 1;

// ESP-NOW peers use channel 0 ("whatever channel we are on"),
// so the SoftAP channel becomes the channel for everything.
// If the band or ENV unit ever pins a channel explicitly, it
// has to be this one.
WebSocketsServer webSocket = WebSocketsServer(WS_PORT);

bool phoneConnected = false;
unsigned long lastStatusTxTime = 0;
constexpr unsigned long STATUS_TX_INTERVAL = 1000;   // 1 Hz

// ---- wall clock -------------------------------------
// The box has no RTC, so millis() is all it has. Rather than
// shipping millis() and making the app guess, we learn the
// epoch from the phone: every `ping` carries the phone's own
// `ts`, so the first one gives us an offset good to the link
// latency. Until then `ts` is millis() and the app is told
// nothing is calibrated.
int64_t epochOffsetMs = 0;
bool epochKnown = false;

static int64_t boxNowMs() {
  return epochKnown ? ((int64_t)millis() + epochOffsetMs) : (int64_t)millis();
}

// ---- box-owned fall / panic latch (protocol section 4) ----
// The band clears its own flags after 5 s. The protocol makes
// the *box* the owner of a 30 s latch, so that the alert
// survives the phone being asleep, absent or reconnecting.
constexpr unsigned long BOX_LATCH_MS = 30000;

bool boxFall = false, boxPanic = false;
unsigned long boxFallUntil = 0, boxPanicUntil = 0;
bool prevBandFall = false, prevBandPanic = false;

static void latchUpdate() {
  unsigned long now = millis();

  bool bandFall  = latestWristband.fallDetected  != 0;
  bool bandPanic = latestWristband.panicPressed != 0;

  // Rising edge only. A flag held true across many packets is
  // one event, not one per packet.
  if (bandFall && !prevBandFall)   { boxFall  = true; boxFallUntil  = now + BOX_LATCH_MS; }
  if (bandPanic && !prevBandPanic) { boxPanic = true; boxPanicUntil = now + BOX_LATCH_MS; }

  prevBandFall  = bandFall;
  prevBandPanic = bandPanic;

  if (boxFall  && (long)(now - boxFallUntil)  >= 0) boxFall  = false;
  if (boxPanic && (long)(now - boxPanicUntil) >= 0) boxPanic = false;
}

// ---- nullable formatting ----------------------------
// An absent reading is JSON null, never 0 and never -999.
// The app renders "--" for null; a sentinel would be drawn as
// a real measurement.
static void fmtF(char *out, size_t cap, float v, int dp, bool valid) {
  if (!valid || isnan(v)) snprintf(out, cap, "null");
  else                    snprintf(out, cap, "%.*f", dp, v);
}

static void fmtI(char *out, size_t cap, int v, bool valid) {
  if (!valid) snprintf(out, cap, "null");
  else        snprintf(out, cap, "%d", v);
}

// ---- status frame -----------------------------------
static void sendStatusFrame() {
  // Band liveness. Anything sourced from the band is null when
  // the band is not currently heard from -- repeating the last
  // value would let the app draw a five-minute-old heart rate
  // as a live reading on a patient the band has fallen off.
  bool bandLive = wristbandLinkEverSeen &&
                  (millis() - lastWristbandRxTime < 3000);

  bool fingerOn = bandLive && latestWristband.fingerDetected;
  bool tempOk   = bandLive && latestWristband.temperatureF > -100.0;
  bool accelOk  = bandLive && (latestWristband.sensorHealth & HEALTH_ACCEL);

  char hr[12], spo2[12], btemp[16];
  fmtI(hr,   sizeof(hr),   latestWristband.heartRate, fingerOn);
  fmtI(spo2, sizeof(spo2), latestWristband.spo2,      fingerOn);
  fmtF(btemp, sizeof(btemp), latestWristband.temperatureF, 1, tempOk);

  // Band reports m/s^2 x100; the protocol carries g.
  char ax[12], ay[12], az[12], mag[12];
  fmtF(ax,  sizeof(ax),  (latestWristband.ax / 100.0f) / 9.80665f, 3, accelOk);
  fmtF(ay,  sizeof(ay),  (latestWristband.ay / 100.0f) / 9.80665f, 3, accelOk);
  fmtF(az,  sizeof(az),  (latestWristband.az / 100.0f) / 9.80665f, 3, accelOk);

  float magG = (latestWristband.accel / 10.0f) / 9.80665f;
  fmtF(mag, sizeof(mag), magG, 3, accelOk);

  // Provisional activity classification, done here because the
  // band does not classify yet. Thresholds are deliberately
  // crude and want tuning against real wear data -- "unknown"
  // when there is no accelerometer, rather than a guess.
  const char *activity = "unknown";
  if (accelOk) {
    float peakG = (latestWristband.accelPeak / 10.0f) / 9.80665f;
    float dev = fabs(peakG - 1.0f);
    if      (dev < 0.15f) activity = "resting";
    else if (dev < 0.60f) activity = "walking";
    else                  activity = "active";
  }

  char eTemp[12], eHum[12], ePres[12], eAlt[12], eLat[16], eLon[16];
  fmtF(eTemp, sizeof(eTemp), envTemp,     1, true);
  fmtF(eHum,  sizeof(eHum),  envHumidity, 1, true);
  fmtF(ePres, sizeof(ePres), envPressure, 1, true);
  fmtF(eAlt,  sizeof(eAlt),  envAltitude, 1, true);
  fmtF(eLat,  sizeof(eLat),  envLat, 5, envGpsFix);
  fmtF(eLon,  sizeof(eLon),  envLon, 5, envGpsFix);

  char buf[720];
  snprintf(buf, sizeof(buf),
    "{\"t\":\"status\",\"ts\":%lld,"
    "\"vitals\":{\"hr\":%s,\"spo2\":%s,\"body_temp_f\":%s},"
    "\"band\":{\"connected\":%s,\"battery\":null,\"fall\":%s,\"panic\":%s},"
    "\"motion\":{\"ax\":%s,\"ay\":%s,\"az\":%s,\"mag\":%s,"
    "\"activity\":\"%s\",\"steps\":null},"
    "\"env\":{\"temp\":%s,\"humidity\":%s,\"pressure\":%s,"
    "\"altitude\":%s,\"lat\":%s,\"lon\":%s},"
    "\"o2\":{\"valve_open\":%s,\"flow_lpm\":null,"
    "\"mode\":\"manual\",\"trigger\":%s}}",
    (long long)boxNowMs(),
    hr, spo2, btemp,
    bandLive ? "true" : "false",
    boxFall ? "true" : "false",
    boxPanic ? "true" : "false",
    ax, ay, az, mag, activity,
    eTemp, eHum, ePres, eAlt, eLat, eLon,
    oxygenSupplyActive ? "true" : "false",
    // Reported as "manual" because that is what it is: the valve is
    // opened from the OLED menu and its rate set by a potentiometer.
    // There is no SpO2-driven control path yet. Saying "auto" here
    // would tell the phone a clinical decision had been made by the
    // box when a person pressed a button.
    oxygenSupplyActive ? "\"manual\"" : "null");

  webSocket.broadcastTXT(buf);
}

// ---- ecg frame --------------------------------------
static void sendEcgFrame() {
  // 62 samples of at most 4 digits plus a comma, plus the
  // envelope. Sized with headroom and still bounds-checked
  // below, because overrunning this on an ADC value would be
  // a silent memory bug on a medical device.
  char buf[512];
  int n = snprintf(buf, sizeof(buf),
    "{\"t\":\"ecg\",\"ts\":%lld,\"fs\":%d,\"seq\":%lu,"
    "\"leads_off\":%s,\"s\":[",
    (long long)(boxNowMs() - (millis() - ecgTxFirstSampleMs)),
    ECG_SAMPLE_RATE,
    (unsigned long)ecgTxSeq++,
    ecgTxLeadOff ? "true" : "false");

  for (int i = 0; i < ecgTxCount && n < (int)sizeof(buf) - 8; i++) {
    n += snprintf(buf + n, sizeof(buf) - n, "%s%u",
                  i ? "," : "", ecgTxBuffer[i]);
  }

  snprintf(buf + n, sizeof(buf) - n, "]}");
  webSocket.broadcastTXT(buf);

  ecgTxCount = 0;
  ecgTxReady = false;
}

// ---- inbound ----------------------------------------
// Only two message types arrive, both fixed shape, so this
// reads them with strstr rather than pulling in a parser.
static void handlePhoneMessage(const char *msg) {
  if (strstr(msg, "\"ping\"")) {
    // Learn wall-clock time from the phone. The box has no RTC,
    // and the protocol's `ts` is defined as epoch milliseconds;
    // the ping carries exactly that.
    const char *tsField = strstr(msg, "\"ts\"");
    if (tsField) {
      const char *colon = strchr(tsField, ':');
      if (colon) {
        int64_t phoneTs = atoll(colon + 1);
        // Sanity: anything past 2020 is a real epoch, anything
        // smaller is a box that is itself uncalibrated.
        if (phoneTs > 1600000000000LL) {
          epochOffsetMs = phoneTs - (int64_t)millis();
          if (!epochKnown) {
            epochKnown = true;
            Serial.println("[WS] Wall clock learned from phone ping");
          }
        }
      }
    }

    char pong[64];
    snprintf(pong, sizeof(pong), "{\"t\":\"pong\",\"ts\":%lld}",
             (long long)boxNowMs());
    webSocket.broadcastTXT(pong);
    return;
  }

  if (strstr(msg, "\"ack\"")) {
    if (strstr(msg, "\"fall\"")) {
      boxFall = false;
      Serial.println("[WS] ACK fall -- latch cleared");
    }
    if (strstr(msg, "\"panic\"")) {
      boxPanic = false;
      Serial.println("[WS] ACK panic -- latch cleared");
    }
    return;
  }
}

static void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t len) {
  switch (type) {
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      phoneConnected = true;
      Serial.printf("[WS] Phone connected from %s\n", ip.toString().c_str());
      break;
    }
    case WStype_DISCONNECTED:
      phoneConnected = (webSocket.connectedClients() > 0);
      Serial.printf("[WS] Phone %u disconnected\n", num);
      break;

    case WStype_TEXT:
      // Payload is not guaranteed null-terminated by the library.
      if (len < 250) {
        char msg[251];
        memcpy(msg, payload, len);
        msg[len] = '\0';
        handlePhoneMessage(msg);
      }
      break;

    default:
      break;
  }
}

// ---- lifecycle --------------------------------------
void bridgeBegin() {
  // AP_STA, not AP: ESP-NOW to the band and the ENV unit runs
  // on the station interface and must keep working while the
  // phone is attached to the access point.
  WiFi.mode(WIFI_AP_STA);

  uint8_t mac[6];
  WiFi.macAddress(mac);

  char ssid[32];
  snprintf(ssid, sizeof(ssid), "HealthBox-%02X%02X", mac[4], mac[5]);

  // Open network. The link carries patient data and should be
  // secured before this leaves a demo bench -- see the note in
  // the app repo's protocol document.
  WiFi.softAP(ssid, nullptr, SOFTAP_CHANNEL);

  Serial.println();
  Serial.println("==============================================");
  Serial.printf("  SoftAP  : %s\n", ssid);
  Serial.printf("  Box IP  : %s\n", WiFi.softAPIP().toString().c_str());
  Serial.printf("  Endpoint: ws://%s:%u/ws\n",
                WiFi.softAPIP().toString().c_str(), WS_PORT);
  Serial.println("==============================================");

  webSocket.begin();
  webSocket.onEvent(onWsEvent);
}

void bridgeUpdate() {
  webSocket.loop();
  latchUpdate();

  if (webSocket.connectedClients() == 0) {
    // Nothing is listening. Keep draining the ECG buffer so it
    // does not go stale and then flush 62 old samples the
    // instant a phone attaches.
    if (ecgTxReady) { ecgTxCount = 0; ecgTxReady = false; }
    return;
  }

  if (ecgTxReady) sendEcgFrame();

  unsigned long now = millis();
  if (now - lastStatusTxTime >= STATUS_TX_INTERVAL) {
    lastStatusTxTime = now;
    sendStatusFrame();
  }
}


void setup() {

  Serial.begin(115200);

  delay(1500);

  // ===================================================
  // GPIO INIT -- ECG / BP / OXYGEN / BUTTONS
  // ===================================================
  pinMode(ECG_LO_PLUS, INPUT);
  pinMode(ECG_LO_MINUS, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(ECG_PIN, ADC_11db);

  pinMode(BP_RELAY_PIN, OUTPUT);
  digitalWrite(BP_RELAY_PIN, LOW);

  pinMode(O2_POT_PIN, INPUT);
  pinMode(O2_RELAY_PIN, OUTPUT);
  digitalWrite(O2_RELAY_PIN, LOW); // Oxygen off by default at boot

  pinMode(PIN_BTN_UP, INPUT_PULLUP);
  pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
  pinMode(PIN_BTN_SELECT, INPUT_PULLUP);
  pinMode(PIN_BTN_BACK, INPUT_PULLUP);

  Serial.println("[NAV] Buttons, ECG, BP relay, and O2 relay pins initialized");
  Serial.println("[O2] Oxygen supply OFF at boot (default)");

  // ===================================================
  // OLED INITIALIZATION
  // ===================================================
  Wire.begin(21, 22);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 OLED NOT FOUND!");
    while (true) {
      delay(1000);
    }
  }

  display.clearDisplay();
  display.display();

  randomSeed(esp_random());

  // ===================================================
  // OLED STARTUP ANIMATION SEQUENCE (unchanged)
  // ===================================================
  showVentwiseFade();
  showCircuitStormByText();
  playCircuitStormIntro();

  // ===================================================
  // SERIAL OUTPUT
  // ===================================================
  Serial.println();
  Serial.println();
  Serial.println("==============================================");
  Serial.println("        VENTWISE V3 ESP32 MASTER / OBC");
  Serial.println("        ESP32 DOIT DEVKIT V1");
  Serial.println("==============================================");

  // bridgeBegin() puts the radio in AP_STA and raises the SoftAP.
  // It runs before esp_now_init() so the channel is settled before
  // any peer is added -- the ESP-NOW peers here use channel 0,
  // meaning "whatever channel the interface is already on".
  bridgeBegin();

  delay(100);

  Serial.print("ACTUAL ESP32 MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.println("CONFIGURED MASTER MAC: 8C:94:DF:6D:86:F4");
  Serial.println("CONFIGURED ENV MAC   : 00:70:07:E2:22:E0");
  Serial.println("CONFIGURED ESP8266 MAC: 40:91:51:58:D3:33");

  // ===================================================
  // ESP-NOW
  // ===================================================

  Serial.println();
  Serial.println("Initializing ESP-NOW...");

  if (esp_now_init() != ESP_OK) {

    Serial.println("ESP-NOW INIT FAILED");

  } else {

    Serial.println("ESP-NOW INIT OK");

    esp_now_register_recv_cb(onDataRecv);

    esp_now_register_send_cb(onDataSent);

    if (!addEnvPeer()) {
      Serial.println("ENV PEER ADD FAILED");
    } else {
      Serial.println("ENV PEER READY");
    }

    if (!addESP8266Peer()) {
      Serial.println("ESP8266 PEER ADD FAILED");
    } else {
      Serial.println("ESP8266 PEER READY");
    }

    delay(500);

    sendToENV("TYPE=COMMAND;CMD=MASTER_READY");
    sendToESP8266("TYPE=COMMAND;CMD=MASTER_READY");
  }

  Serial.println();
  Serial.println("==============================================");
  Serial.println("             MASTER READY");
  Serial.println("==============================================");

  // ===================================================
  // NAV MENU FIRST DRAW
  // ===================================================
  currentScreen = SCREEN_MENU;
  screenNeedsRedraw = true;
  drawMenuScreen();
  screenNeedsRedraw = false;
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  // Phone bridge first: the WebSocket library needs servicing every
  // pass, and the ECG batch is flushed the moment it is full rather
  // than waiting behind the display code.
  bridgeUpdate();

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

  // Oxygen runs in the background regardless of which screen is active,
  // per spec ("supply persists across menu navigation").
  oxygenUpdate();

  handleNavigation();
  updateActiveScreenLogic();

  // Only ECG needs a tight loop (250 Hz sampling); everything else
  // is comfortable with a short yield.
  if (currentScreen != SCREEN_ECG) {
    delay(5);
  }
}
