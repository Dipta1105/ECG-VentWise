#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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
//
// ESP8266 MAC (ADDED):
// 40:91:51:58:D3:33
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
// MAC ADDRESSES
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
// LOGO GRAPHIC - 128 x 48 PIXELS
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
// PARTICLE SYSTEM
// =====================================================

struct Particle {
  float x;
  float y;
  float vx;
  uint8_t life;
  uint8_t maxLife;
};

Particle particles[28];

// =====================================================
// RESET PARTICLES
// =====================================================

void resetParticles() {
  for (int i = 0; i < 28; i++) {
    particles[i].x = random(-30, 10);
    particles[i].y = random(2, 47);
    particles[i].vx = random(8, 22) / 10.0;
    particles[i].life = random(15, 50);
    particles[i].maxLife = particles[i].life;
  }
}

// =====================================================
// UPDATE PARTICLES
// =====================================================

void updateParticles(float sweepX) {
  for (int i = 0; i < 28; i++) {
    particles[i].x += particles[i].vx;
    
    // Keep particles around advancing sweep
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

// =====================================================
// DRAW GRAPHIC
// =====================================================

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

// =====================================================
// DRAW PARTICLES
// =====================================================

void drawParticles(float sweepX) {
  for (int i = 0; i < 28; i++) {
    if (particles[i].life == 0)
      continue;
      
    int x = (int)particles[i].x;
    int y = (int)particles[i].y;
    
    // Particle head
    if (x >= 0 && x < 128 && y >= 0 && y < 48) {
      display.drawPixel(x, y, SSD1306_WHITE);
    }
    
    // Particle trailing streak
    if (particles[i].life > particles[i].maxLife / 2) {
      if (x - 1 >= 0) {
        display.drawPixel(x - 1, y, SSD1306_WHITE);
      }
      if (random(0, 3) == 0 && x - 2 >= 0) {
        display.drawPixel(x - 2, y, SSD1306_WHITE);
      }
    }
  }
  
  // Bright scanning edge
  if (sweepX >= 0 && sweepX < 128) {
    for (int y = 4; y < 45; y += 3) {
      if (random(0, 4) != 0) {
        display.drawPixel(sweepX, y, SSD1306_WHITE);
      }
    }
  }
}

// =====================================================
// DRAW CIRCUITSTORM TEXT
// =====================================================

void drawCircuitStormText(int charsToShow, bool cursorVisible) {
  const char *text = "CIRCUITSTORM";
  const int len = 12;
  
  // Default Adafruit font - 6 pixels per character
  const int textWidth = len * 6;
  const int startX = (128 - textWidth) / 2;
  const int textY = 54;
  
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(startX, textY);
  
  // Draw typed characters
  for (int i = 0; i < charsToShow && i < len; i++) {
    display.write(text[i]);
  }
  
  // Cursor
  if (cursorVisible && charsToShow < len) {
    display.fillRect(startX + charsToShow * 6, textY + 6, 4, 1, SSD1306_WHITE);
  }
}

// =====================================================
// PLAY CIRCUITSTORM LOGO INTRO
// GRAPHIC + TEXT RUN SIMULTANEOUSLY
// =====================================================

void playCircuitStormIntro() {
  resetParticles();
  
  // Graphic timing
  const unsigned long graphicTime = 1900;
  
  // Text
  const char *text = "CIRCUITSTORM";
  const int textLen = 12;
  
  // Unequal typing delays
  int typingDelays[] = {
    75, 155, 55, 210, 90, 135, 60, 185, 80, 240, 70, 170
  };
  
  // Start everything
  unsigned long startTime = millis();
  int typedChars = 0;
  unsigned long nextTypeTime = startTime + typingDelays[0];
  
  // Simultaneous animation loop
  while (true) {
    unsigned long now = millis();
    unsigned long elapsed = now - startTime;
    
    // Text typing
    if (typedChars < textLen && now >= nextTypeTime) {
      typedChars++;
      if (typedChars < textLen) {
        nextTypeTime = now + typingDelays[typedChars];
      }
    }
    
    // Graphic sweep
    float t;
    if (elapsed >= graphicTime) {
      t = 1.0;
    } else {
      t = (float)elapsed / graphicTime;
    }
    
    // Smooth ease-out
    float eased = 1.0 - pow(1.0 - t, 3.0);
    float sweepX = eased * 128.0;
    
    // Draw frame
    display.clearDisplay();
    
    // Graphic
    drawGraphic((uint8_t)sweepX);
    
    // Particles
    updateParticles(sweepX);
    drawParticles(sweepX);
    
    // Text
    bool cursorVisible = ((now / 180) % 2) == 0;
    drawCircuitStormText(typedChars, cursorVisible);
    
    // Push frame to OLED
    display.display();
    
    // Check completion
    bool graphicFinished = elapsed >= graphicTime;
    bool textFinished = typedChars >= textLen;
    
    if (graphicFinished && textFinished) {
      break;
    }
    
    delay(25);
  }
  
  // Complete logo
  display.clearDisplay();
  
  // Full graphic
  drawGraphic(128);
  
  // Full text
  drawCircuitStormText(textLen, false);
  
  display.display();
  
  // Final logo hold
  delay(3000);
}

// =====================================================
// VENTWISE V3 FADE ANIMATION
// =====================================================

void showVentwiseFade() {
  display.clearDisplay();
  display.display();
  
  // Fade in animation using progressive drawing
  for (int step = 0; step <= 20; step++) {
    display.clearDisplay();
    
    // Draw text with varying intensity
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 18);
    
    // Simulate fade by drawing text multiple times with different thickness
    for (int thickness = 0; thickness < step / 4; thickness++) {
      display.setCursor(10 + thickness, 18 + thickness);
      display.println("VENTWISE");
      display.setCursor(30 + thickness, 36 + thickness);
      display.println("V3");
    }
    
    // Main text
    display.setCursor(10, 18);
    display.println("VENTWISE");
    display.setCursor(30, 36);
    display.println("V3");
    
    display.display();
    delay(50);
  }
  
  // Hold for 3 seconds
  delay(3000);
}

// =====================================================
// CIRCUITSTORM BY TEXT ANIMATION
// One-by-one text fade appear
// =====================================================

void showCircuitStormByText() {
  const char* text = "CIRCUITSTORM";
  int textLen = strlen(text);
  
  // Clear display
  display.clearDisplay();
  display.display();
  
  // Show "by" with fade
  for (int fadeStep = 0; fadeStep <= 10; fadeStep++) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(52, 14);
    
    // Simulate fade by drawing multiple times
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
  
  // One-by-one fade appear for CIRCUITSTORM
  for (int i = 0; i < textLen; i++) {
    display.clearDisplay();
    
    // "by" at top center
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(52, 14);
    display.println("by");
    
    // CIRCUITSTORM characters one by one
    display.setTextSize(1);
    display.setCursor(20, 36);
    
    // Draw all characters up to current
    for (int j = 0; j <= i; j++) {
      display.write(text[j]);
    }
    
    // Fade effect for the newest character
    if (i > 0) {
      // Blink animation for the latest character
      for (int blink = 0; blink < 4; blink++) {
        display.display();
        delay(35);
        // Redraw to keep visible
        display.setCursor(20 + (i * 6), 36);
        display.write(text[i]);
        display.display();
        delay(35);
      }
    }
    
    display.display();
    delay(80);
  }
  
  // Hold final state
  delay(3000);
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

  Serial.println();
  Serial.println("================================================");
  Serial.println("              ESP-NOW DATA RECEIVED");
  Serial.println("================================================");

  // ---------------------------------------------------
  // SOURCE MAC
  // ---------------------------------------------------

  Serial.print("FROM MAC : ");

  for (int i = 0; i < 6; i++) {

    if (info->src_addr[i] < 16)
      Serial.print("0");

    Serial.print(info->src_addr[i], HEX);

    if (i < 5)
      Serial.print(":");
  }

  Serial.println();

  // ---------------------------------------------------
  // COPY PACKET
  // ---------------------------------------------------

  char message[250];

  int copyLen = len;

  if (copyLen >= sizeof(message))
    copyLen = sizeof(message) - 1;

  memcpy(message, data, copyLen);

  message[copyLen] = '\0';

  Serial.println();
  Serial.println("RAW PACKET:");
  Serial.println(message);

  // ---------------------------------------------------
  // DETERMINE MESSAGE TYPE
  // ---------------------------------------------------

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

// =====================================================
// ADD ESP8266 PEER
// =====================================================

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

// =====================================================
// SEND TO ENV
// =====================================================

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

// =====================================================
// SEND TO ESP8266
// =====================================================

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

// =====================================================
// COMMAND PROCESSING
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
  
  // Initialize random seed
  randomSeed(esp_random());
  
  // ===================================================
  // OLED STARTUP ANIMATION SEQUENCE
  // ===================================================
  
  // 1. "VENTWISE V3" with fade animation, bold font - stays 3 seconds
  showVentwiseFade();
  
  // 2. "by /n CIRCUITSTORM" center, Montserrat style, one-by-one fade - stays 3 seconds
  showCircuitStormByText();
  
  // 3. Full CircuitStorm Logo animation with particles
  playCircuitStormIntro();
  
  // ===================================================
  // SERIAL OUTPUT
  // ===================================================
  Serial.println();
  Serial.println();
  Serial.println("==============================================");
  Serial.println("        HYDROS ESP32 MASTER / OBC");
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

  // ===================================================
  // ADD ESP8266 PEER
  // ===================================================

  if (!addESP8266Peer()) {

    Serial.println("ESP8266 PEER ADD FAILED");

  } else {

    Serial.println("ESP8266 PEER READY");
  }

  Serial.println();
  Serial.println("==============================================");
  Serial.println("             MASTER READY");
  Serial.println("==============================================");

  delay(500);

  sendToENV("TYPE=COMMAND;CMD=MASTER_READY");
  sendToESP8266("TYPE=COMMAND;CMD=MASTER_READY");
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
