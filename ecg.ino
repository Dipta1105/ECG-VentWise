#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================================================
// ESP32 DEVKIT V1 + AD8232 + 0.91" 128x32 OLED
// ECG DISPLAY - LIGHT FILTER / WIDE TIME SCALE
// ============================================================


// ============================================================
// OLED CONFIGURATION
// ============================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

#define OLED_SDA 21
#define OLED_SCL 22

#define OLED_ADDRESS 0x3C
#define OLED_RESET -1

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  OLED_RESET
);


// ============================================================
// AD8232
// ============================================================

#define ECG_PIN     34
#define LO_PLUS     32
#define LO_MINUS    33


// ============================================================
// ECG SAMPLING
// ============================================================

// Actual ADC acquisition rate
// 250 samples/sec is suitable for ECG morphology.

#define SAMPLE_RATE 250

const unsigned long SAMPLE_INTERVAL =
  1000000UL / SAMPLE_RATE;

unsigned long lastSampleTime;


// ============================================================
// OLED TIME SCALE
// ============================================================

// We don't need to draw all 250 samples/sec.
//
// Display approximately 100 samples/sec.
//
// 128 pixels / 100 samples/sec
// = approximately 1.28 seconds visible.
//
// This gives the ECG much more horizontal space.

#define DISPLAY_RATE 100

const int DISPLAY_DIVIDER =
  SAMPLE_RATE / DISPLAY_RATE;

int displayCounter = 0;


// ============================================================
// CALIBRATION
// ============================================================

#define CALIBRATION_TIME 5000

bool calibrated = false;

unsigned long calibrationStart;

float dcBaseline = 0;

float calibrationSum = 0;

unsigned long calibrationSamples = 0;


// ============================================================
// ECG FILTER
// ============================================================

// LIGHT FILTER
//
// This is deliberately gentle.
//
// Previous version:
// 0.5 Hz HP + 40 Hz LP + 50 Hz notch
//
// That could make the waveform overly smooth.
//
// Here we use:
//
// 1. DC/baseline removal
// 2. Gentle low-pass smoothing
//
// The actual ECG morphology is retained much better.

float baseline = 2048.0;

float filteredSignal = 0;


// ============================================================
// DISPLAY SCALING
// ============================================================

// ECG center

#define ECG_CENTER 16

// Maximum distance from center

#define MAX_AMPLITUDE 14


// Current automatic scale

float displayScale = 300.0;


// ============================================================
// OLED WAVEFORM BUFFER
// ============================================================

int waveform[SCREEN_WIDTH];

int writePosition = 0;


// ============================================================
// DISPLAY TIMER
// ============================================================

unsigned long lastOLEDUpdate = 0;


// ============================================================
// FILTER FUNCTION
// ============================================================

float processECG(float raw)
{
  // ----------------------------------------------------------
  // BASELINE TRACKING
  // ----------------------------------------------------------

  // Very slow baseline tracker.
  //
  // This removes the large DC offset from AD8232
  // without destroying the ECG waveform.

  baseline =
    baseline +
    0.0015f *
    (raw - baseline);


  // AC component

  float ac =
    raw - baseline;


  // ----------------------------------------------------------
  // LIGHT LOW-PASS
  // ----------------------------------------------------------

  // Gentle smoothing.
  //
  // Higher value = less filtering.
  //
  // Previous code used 0.20.
  //
  // This uses 0.45 to retain QRS detail.

  filteredSignal =
    filteredSignal +
    0.45f *
    (ac - filteredSignal);


  return filteredSignal;
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(1000);


  // ==========================================================
  // ADC
  // ==========================================================

  analogReadResolution(12);

  analogSetPinAttenuation(
    ECG_PIN,
    ADC_11db
  );


  // ==========================================================
  // LEAD OFF
  // ==========================================================

  pinMode(
    LO_PLUS,
    INPUT
  );

  pinMode(
    LO_MINUS,
    INPUT
  );


  // ==========================================================
  // I2C
  // ==========================================================

  Wire.begin(
    OLED_SDA,
    OLED_SCL
  );


  // ==========================================================
  // OLED
  // ==========================================================

  if (!display.begin(
        SSD1306_SWITCHCAPVCC,
        OLED_ADDRESS))
  {
    Serial.println(
      "OLED FAILED"
    );

    while (true)
    {
      delay(1000);
    }
  }


  // ==========================================================
  // INITIALIZE WAVEFORM BUFFER
  // ==========================================================

  for (
    int i = 0;
    i < SCREEN_WIDTH;
    i++
  )
  {
    waveform[i] =
      ECG_CENTER;
  }


  // ==========================================================
  // CALIBRATION SCREEN
  // ==========================================================

  display.clearDisplay();

  display.setTextColor(
    SSD1306_WHITE
  );

  display.setTextSize(1);


  display.setCursor(
    0,
    0
  );

  display.println(
    "ECG CALIBRATION"
  );


  display.setCursor(
    0,
    10
  );

  display.println(
    "Keep still..."
  );


  display.setCursor(
    0,
    20
  );

  display.println(
    "5 seconds"
  );


  display.display();


  // ==========================================================
  // SERIAL
  // ==========================================================

  Serial.println();
  Serial.println(
    "================================"
  );

  Serial.println(
    "ECG CALIBRATION"
  );

  Serial.println(
    "Keep electrodes attached"
  );

  Serial.println(
    "Keep body still"
  );

  Serial.println(
    "================================"
  );


  calibrationStart =
    millis();


  lastSampleTime =
    micros();
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  unsigned long now =
    micros();


  // ==========================================================
  // FIXED 250 Hz SAMPLE RATE
  // ==========================================================

  if (
    (unsigned long)(
      now - lastSampleTime
    ) >= SAMPLE_INTERVAL
  )
  {
    lastSampleTime +=
      SAMPLE_INTERVAL;


    processSample();
  }
}


// ============================================================
// SAMPLE PROCESSING
// ============================================================

void processSample()
{
  int raw =
    analogRead(
      ECG_PIN
    );


  // ==========================================================
  // LEAD OFF
  // ==========================================================

  bool leadOff =
    digitalRead(LO_PLUS) ||
    digitalRead(LO_MINUS);


  // ==========================================================
  // CALIBRATION
  // ==========================================================

  if (!calibrated)
  {
    calibrate(
      raw,
      leadOff
    );

    return;
  }


  // ==========================================================
  // FILTER
  // ==========================================================

  float ecg =
    processECG(
      (float)raw
    );


  // ==========================================================
  // SERIAL PLOTTER
  // ==========================================================

  if (leadOff)
  {
    Serial.println(0);
  }
  else
  {
    Serial.println(
      (int)ecg
    );
  }


  // ==========================================================
  // DISPLAY DECIMATION
  // ==========================================================

  displayCounter++;


  if (
    displayCounter <
    DISPLAY_DIVIDER
  )
  {
    return;
  }


  displayCounter = 0;


  // ==========================================================
  // IGNORE LEAD OFF
  // ==========================================================

  if (leadOff)
  {
    drawLeadOff();

    return;
  }


  // ==========================================================
  // ADAPTIVE VERTICAL SCALE
  // ==========================================================

  float magnitude =
    fabs(ecg);


  // If signal exceeds current scale,
  // increase scale quickly.

  if (
    magnitude >
    displayScale
  )
  {
    displayScale =
      magnitude * 1.15f;
  }
  else
  {
    // Slowly reduce scale again.
    //
    // This prevents the ECG from becoming
    // permanently tiny after one large spike.

    displayScale *=
      0.9995f;
  }


  // Keep sensible limits.

  if (
    displayScale < 100
  )
  {
    displayScale = 100;
  }


  if (
    displayScale > 1000
  )
  {
    displayScale = 1000;
  }


  // ==========================================================
  // NORMALIZE
  // ==========================================================

  float normalized =
    ecg /
    displayScale;


  normalized =
    constrain(
      normalized,
      -1.0f,
      1.0f
    );


  // ==========================================================
  // OLED Y POSITION
  // ==========================================================

  int y =
    ECG_CENTER -
    (
      normalized *
      MAX_AMPLITUDE
    );


  y =
    constrain(
      y,
      1,
      30
    );


  // ==========================================================
  // STORE POINT
  // ==========================================================

  waveform[
    writePosition
  ] = y;


  writePosition++;


  if (
    writePosition >=
    SCREEN_WIDTH
  )
  {
    writePosition = 0;
  }


  // ==========================================================
  // DRAW
  // ==========================================================

  drawWaveform();
}


// ============================================================
// CALIBRATION
// ============================================================

void calibrate(
  int raw,
  bool leadOff
)
{
  // ----------------------------------------------------------
  // Baseline estimate
  // ----------------------------------------------------------

  calibrationSum +=
    raw;

  calibrationSamples++;


  // ----------------------------------------------------------
  // Initialize baseline
  // ----------------------------------------------------------

  if (
    calibrationSamples == 1
  )
  {
    baseline =
      raw;

    filteredSignal =
      0;
  }


  // ----------------------------------------------------------
  // Run filter so it settles
  // ----------------------------------------------------------

  processECG(
    (float)raw
  );


  // ----------------------------------------------------------
  // CALIBRATION TIMER
  // ----------------------------------------------------------

  unsigned long elapsed =
    millis() -
    calibrationStart;


  // ----------------------------------------------------------
  // OLED UPDATE
  // ----------------------------------------------------------

  static unsigned long lastCalDisplay =
    0;


  if (
    millis() -
    lastCalDisplay >=
    250
  )
  {
    lastCalDisplay =
      millis();


    int seconds =
      elapsed / 1000;


    display.clearDisplay();


    display.setTextSize(1);

    display.setTextColor(
      SSD1306_WHITE
    );


    display.setCursor(
      0,
      0
    );

    display.println(
      "ECG CALIBRATION"
    );


    display.setCursor(
      0,
      10
    );

    if (leadOff)
    {
      display.println(
        "LEAD OFF!"
      );
    }
    else
    {
      display.println(
        "Stay still..."
      );
    }


    display.setCursor(
      0,
      20
    );

    display.print(
      seconds
    );

    display.println(
      " / 5 sec"
    );


    display.display();
  }


  // ----------------------------------------------------------
  // FINISH
  // ----------------------------------------------------------

  if (
    elapsed >=
    CALIBRATION_TIME
  )
  {
    dcBaseline =
      calibrationSum /
      calibrationSamples;


    baseline =
      dcBaseline;


    // Reset signal

    filteredSignal =
      0;


    calibrated =
      true;


    // Reset waveform

    for (
      int i = 0;
      i < SCREEN_WIDTH;
      i++
    )
    {
      waveform[i] =
        ECG_CENTER;
    }


    writePosition =
      0;


    displayScale =
      300;


    // --------------------------------------------------------
    // READY SCREEN
    // --------------------------------------------------------

    display.clearDisplay();


    display.setTextSize(1);

    display.setTextColor(
      SSD1306_WHITE
    );


    display.setCursor(
      0,
      0
    );

    display.println(
      "CALIBRATION DONE"
    );


    display.setCursor(
      0,
      10
    );

    display.println(
      "ECG READY"
    );


    display.setCursor(
      0,
      20
    );

    display.println(
      "Starting trace..."
    );


    display.display();


    Serial.println();
    Serial.println(
      "================================"
    );

    Serial.println(
      "CALIBRATION COMPLETE"
    );

    Serial.print(
      "Baseline = "
    );

    Serial.println(
      dcBaseline
    );

    Serial.println(
      "ECG TRACE STARTED"
    );

    Serial.println(
      "================================"
    );


    delay(1000);
  }
}


// ============================================================
// DRAW ECG WAVEFORM
// ============================================================

void drawWaveform()
{
  display.clearDisplay();


  // ==========================================================
  // CENTER LINE
  // ==========================================================

  // Very subtle reference line.

  for (
    int x = 0;
    x < SCREEN_WIDTH;
    x += 8
  )
  {
    display.drawPixel(
      x,
      ECG_CENTER,
      SSD1306_WHITE
    );
  }


  // ==========================================================
  // DRAW TRACE
  // ==========================================================

  for (
    int x = 1;
    x < SCREEN_WIDTH;
    x++
  )
  {
    int index1 =
      (
        writePosition +
        x -
        1
      ) %
      SCREEN_WIDTH;


    int index2 =
      (
        writePosition +
        x
      ) %
      SCREEN_WIDTH;


    int y1 =
      waveform[index1];


    int y2 =
      waveform[index2];


    display.drawLine(
      x - 1,
      y1,
      x,
      y2,
      SSD1306_WHITE
    );
  }


  // ==========================================================
  // DISPLAY
  // ==========================================================

  display.display();
}


// ============================================================
// LEAD OFF SCREEN
// ============================================================

void drawLeadOff()
{
  display.clearDisplay();


  display.setTextColor(
    SSD1306_WHITE
  );

  display.setTextSize(1);


  display.setCursor(
    0,
    5
  );

  display.println(
    "ECG"
  );


  display.setCursor(
    0,
    17
  );

  display.println(
    "LEAD OFF"
  );


  display.display();
}
