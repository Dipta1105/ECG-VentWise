#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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
typedef struct __attribute__((packed)) {
  int16_t heartRate;
  int16_t spo2;
  uint8_t fingerDetected;
  uint8_t fallDetected;
  int16_t accel;
  uint8_t panicPressed;
  float temperatureF;
} WristbandPacket;

WristbandPacket latestWristband = { 0, 0, 0, 0, 0, 0, -127.0 };
unsigned long lastWristbandRxTime = 0;
bool wristbandLinkEverSeen = false;

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

void ecgProcessSample() {
  int raw = analogRead(ECG_PIN);

  bool leadOff = digitalRead(ECG_LO_PLUS) || digitalRead(ECG_LO_MINUS);

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
  if (len == sizeof(WristbandPacket)) {
    memcpy(&latestWristband, data, sizeof(WristbandPacket));
    lastWristbandRxTime = millis();
    wristbandLinkEverSeen = true;

    Serial.print("[WRIST] RX HR=");
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
    Serial.println(latestWristband.panicPressed);

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

  WiFi.mode(WIFI_STA);

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
