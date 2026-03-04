#include <Arduino.h>
#include <FastLED.h>
#include "robot_logic.h"

#define FW_VERSION "0.1.0"

//
// Motor pins (TB6612FNG)
//
const int PWMA = 5; // Left speed
const int PWMB = 6; // Right speed
const int AIN1 = 7; // Left direction
const int BIN1 = 8; // Right direction
const int STBY = 3; // Standby
//
// Ultrasonic sensor pins (HC-SR04)
//
// pin 13 is wired to the onboard LED, and also to the ultrasonic
// connector on the shield, so that particular led cannot be used
// independently without impacting the ultrasonic sensor
const int TRIG = 13; // Trigger (output)
const int ECHO = 12; // Echo (input)
//
// Battery voltage pin
//
// Elegoo shield has voltage divider on A3
// 2x 18650 = 7.4V nominal (6.0V empty, 8.4V full)
// Voltage divider ratio assumed 1:2 (needs calibration)
const int BATTERY_PIN = A3;
float BATTERY_DIVIDER_RATIO = 2.0; // adjust based on actual divider
//
// RGB LED
//
const int LED_DATA_PIN = 4;
const int NUM_LEDS = 1;

CRGB leds[NUM_LEDS];

int motorSpeed = 150; // Global speed setting (0-255)

// Motor bias correction (for drift compensation)
float leftMotorBias = 1.0;  // 0.8-1.2 typical
float rightMotorBias = 1.0;

// Current state tracking (actual motor output, ramped)
int currentLeftSpeed = 0;  // -255 to 255
int currentRightSpeed = 0; // -255 to 255

// Target state (what the PC requested, before ramping)
int targetLeftSpeed = 0;
int targetRightSpeed = 0;

uint8_t currentLedR = 0;
uint8_t currentLedG = 0;
uint8_t currentLedB = 0;
uint8_t currentBrightness = 50;

// Motor watchdog
unsigned long lastMotorCommand = 0; // initialized to millis() in setup()

// Battery EMA filter
float batterySmoothed = -1.0; // sentinel: first read initializes
unsigned long watchdogTimeout = 1000; // 1s default — allows ~10 missed cmds at 10Hz

// Acceleration curve config
int maxAccelPerTick = 20;         // max speed change per 20ms tick (0-255 range)

// Safety loop timing
unsigned long lastSafetyTick = 0;
const int SAFETY_INTERVAL_MS = 20; // 50Hz

// Cached distance from safety loop (avoids double sonar reads)
int lastDistanceCm = 0;

// Sensor streaming
unsigned long streamIntervalMs = 0; // 0 = disabled
unsigned long lastStreamTime = 0;

// Forward declarations
void stop();
void tankDrive(int leftSpeed, int rightSpeed);
int getDistance();
float getBatteryVoltage();
void processCommand(const char *cmd);

void setup()
{
  Serial.begin(115200);
  // setup fastled
  FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(50); // 0-255

  // Set all motor pins as outputs
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Enable motor driver
  digitalWrite(STBY, HIGH);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  lastMotorCommand = millis();
}

char inputBuffer[128];
int bufferIndex = 0;

void loop()
{
  // === 50Hz motor ramp loop ===
  if (millis() - lastSafetyTick >= SAFETY_INTERVAL_MS) {
    lastSafetyTick = millis();

    // Ramp current speed toward target
    currentLeftSpeed = rampSpeed(currentLeftSpeed, targetLeftSpeed, maxAccelPerTick);
    currentRightSpeed = rampSpeed(currentRightSpeed, targetRightSpeed, maxAccelPerTick);

    // Apply motor output
    tankDrive(currentLeftSpeed, currentRightSpeed);
  }

  // Motor watchdog - stop motors if no command received within timeout
  if (watchdogTimeout > 0 && (targetLeftSpeed != 0 || targetRightSpeed != 0))
  {
    if (millis() - lastMotorCommand > watchdogTimeout)
    {
      targetLeftSpeed = 0;
      targetRightSpeed = 0;
      currentLeftSpeed = 0;
      currentRightSpeed = 0;
      stop();
    }
  }

  // Sensor streaming
  if (streamIntervalMs > 0 && millis() - lastStreamTime >= streamIntervalMs) {
    lastStreamTime = millis();
    processCommand("{\"N\":100}");
  }

  // Serial command processing
  if (Serial.available())
  {
    char c = Serial.read();

    if (c == '\n' || c == '\r')
    {
      if (bufferIndex > 0)
      {
        inputBuffer[bufferIndex] = '\0';
        processCommand(inputBuffer);
        bufferIndex = 0;
      }
    }
    else if (bufferIndex < 127)
    {
      inputBuffer[bufferIndex++] = c;
    }
  }
}

void processCommand(const char *cmd)
{
  int n = getJsonInt(cmd, "\"N\":", -1);
  if (n == -1)
  {
    Serial.println("{\"err\":\"no N\"}");
    return;
  }

  int d1 = getJsonInt(cmd, "\"D1\":");
  int d2 = getJsonInt(cmd, "\"D2\":");
  int d3 = getJsonInt(cmd, "\"D3\":");

  switch (n)
  {
  // === General ===
  case 1: // Ping
    Serial.println("{\"ok\":true}");
    break;

  // === Motor Control ===
  // All motor commands set TARGET speed; the safety loop ramps and applies.
  case 2: // Forward (D1=speed, optional)
  {
    int spd = d1 > 0 ? d1 : motorSpeed;
    targetLeftSpeed = spd;
    targetRightSpeed = spd;
    lastMotorCommand = millis();
    Serial.println("{\"cmd\":\"fwd\"}");
    break;
  }
  case 3: // Backward
  {
    int spd = d1 > 0 ? d1 : motorSpeed;
    targetLeftSpeed = -spd;
    targetRightSpeed = -spd;
    lastMotorCommand = millis();
    Serial.println("{\"cmd\":\"bwd\"}");
    break;
  }
  case 4: // Turn left
  {
    int spd = d1 > 0 ? d1 : motorSpeed;
    targetLeftSpeed = -spd;
    targetRightSpeed = spd;
    lastMotorCommand = millis();
    Serial.println("{\"cmd\":\"left\"}");
    break;
  }
  case 5: // Turn right
  {
    int spd = d1 > 0 ? d1 : motorSpeed;
    targetLeftSpeed = spd;
    targetRightSpeed = -spd;
    lastMotorCommand = millis();
    Serial.println("{\"cmd\":\"right\"}");
    break;
  }
  case 6: // Stop — instant, zeroes BOTH targets AND current speeds
    targetLeftSpeed = 0;
    targetRightSpeed = 0;
    currentLeftSpeed = 0;
    currentRightSpeed = 0;
    stop();
    Serial.println("{\"cmd\":\"stop\"}");
    break;
  case 7: // Tank control: D1=left(-255 to 255), D2=right(-255 to 255)
    targetLeftSpeed = d1;
    targetRightSpeed = d2;
    lastMotorCommand = millis();
    // No serial ack — this fires at 10Hz and would saturate the TX line
    break;
  case 8: // Set default speed
  {
    motorSpeed = constrain(d1, 0, 255);
    char buf[16];
    snprintf(buf, sizeof(buf), "{\"s\":%d}", motorSpeed);
    Serial.println(buf);
    break;
  }

  // === Sensors ===
  case 10: // Get distance (fresh read)
  {
    lastDistanceCm = getDistance();
    char buf[16];
    snprintf(buf, sizeof(buf), "{\"d\":%d}", lastDistanceCm);
    Serial.println(buf);
    break;
  }
  case 13: // Get battery voltage
  {
    float voltage = getBatteryVoltage();
    int raw = analogRead(BATTERY_PIN);
    char vStr[8];
    dtostrf(voltage, 1, 2, vStr);
    char buf[32];
    snprintf(buf, sizeof(buf), "{\"b\":%s,\"raw\":%d}", vStr, raw);
    Serial.println(buf);
    break;
  }
  case 14: // Calibrate battery: D1=actual voltage * 100 (e.g., 740 = 7.40V)
  {
    float actualVoltage = d1 / 100.0;
    int raw = analogRead(BATTERY_PIN);
    float adcVoltage = raw * (5.0 / 1023.0);
    if (adcVoltage > 0.1) {
      BATTERY_DIVIDER_RATIO = actualVoltage / adcVoltage;
      char brStr[8];
      dtostrf(BATTERY_DIVIDER_RATIO, 1, 3, brStr);
      char buf[24];
      snprintf(buf, sizeof(buf), "{\"br\":%s}", brStr);
      Serial.println(buf);
    } else {
      Serial.println("{\"err\":\"voltage too low\"}");
    }
    break;
  }
  case 15: // Set motor bias: D1=left% (80-120), D2=right% (80-120)
  {
    leftMotorBias = constrain(d1, 80, 120) / 100.0;
    rightMotorBias = constrain(d2, 80, 120) / 100.0;
    char lStr[8], rStr[8];
    dtostrf(leftMotorBias, 1, 2, lStr);
    dtostrf(rightMotorBias, 1, 2, rStr);
    char buf[32];
    snprintf(buf, sizeof(buf), "{\"mb\":[%s,%s]}", lStr, rStr);
    Serial.println(buf);
    break;
  }

  // === LED ===
  case 20: // Set LED color: D1=R, D2=G, D3=B (0-255 each)
  {
    currentLedR = d1;
    currentLedG = d2;
    currentLedB = d3;
    leds[0] = CRGB(d1, d2, d3);
    FastLED.show();
    char buf[24];
    snprintf(buf, sizeof(buf), "{\"l\":[%d,%d,%d]}", d1, d2, d3);
    Serial.println(buf);
    break;
  }
  case 21: // Set LED brightness: D1=brightness (0-255)
  {
    currentBrightness = constrain(d1, 0, 255);
    FastLED.setBrightness(currentBrightness);
    FastLED.show();
    char buf[16];
    snprintf(buf, sizeof(buf), "{\"B\":%u}", currentBrightness);
    Serial.println(buf);
    break;
  }

  // === Status ===
  case 100: // Get all sensors
  {
    unsigned long startTime = micros();
    unsigned long t = millis();
    lastDistanceCm = getDistance();
    float batt = getBatteryVoltage();
    unsigned long execTime = micros() - startTime;

    char battStr[8];
    dtostrf(batt, 1, 2, battStr);

    char buf[64];
    snprintf(buf, sizeof(buf),
      "{\"t\":%lu,\"u\":%lu,\"d\":%d,\"b\":%s}",
      t, execTime, lastDistanceCm, battStr);
    Serial.println(buf);
    break;
  }
  case 101: // Get current state
  {
    char lbStr[8], rbStr[8], brStr[8];
    dtostrf(leftMotorBias, 1, 2, lbStr);
    dtostrf(rightMotorBias, 1, 2, rbStr);
    dtostrf(BATTERY_DIVIDER_RATIO, 1, 3, brStr);
    char buf[160];
    snprintf(buf, sizeof(buf),
      "{\"t\":%lu,\"M\":[%d,%d],\"T\":[%d,%d],\"l\":[%u,%u,%u],\"B\":%u,"
      "\"s\":%d,\"w\":%lu,\"mb\":[%s,%s],\"br\":%s,\"ma\":%d,\"st\":%lu}",
      millis(),
      currentLeftSpeed, currentRightSpeed,
      targetLeftSpeed, targetRightSpeed,
      currentLedR, currentLedG, currentLedB,
      currentBrightness,
      motorSpeed,
      watchdogTimeout,
      lbStr, rbStr,
      brStr,
      maxAccelPerTick,
      streamIntervalMs);
    Serial.println(buf);
    break;
  }
  case 102: // Set watchdog timeout: D1=timeout_ms (0=disable, max 30s)
  {
    watchdogTimeout = (unsigned long)constrain(d1, 0, 30000);
    char buf[16];
    snprintf(buf, sizeof(buf), "{\"w\":%lu}", watchdogTimeout);
    Serial.println(buf);
    break;
  }
  case 103: // Set sensor stream interval: D1=interval_ms (0=disable)
  {
    streamIntervalMs = d1;
    lastStreamTime = millis();
    char buf[16];
    snprintf(buf, sizeof(buf), "{\"st\":%lu}", streamIntervalMs);
    Serial.println(buf);
    break;
  }
  case 104: // Set acceleration: D1=maxAccelPerTick
  {
    if (d1 > 0) maxAccelPerTick = d1;
    char buf[16];
    snprintf(buf, sizeof(buf), "{\"ma\":%d}", maxAccelPerTick);
    Serial.println(buf);
    break;
  }
  case 105: // Get firmware version
  {
    char buf[24];
    snprintf(buf, sizeof(buf), "{\"fv\":\"%s\"}", FW_VERSION);
    Serial.println(buf);
    break;
  }

  default:
  {
    char buf[32];
    snprintf(buf, sizeof(buf), "{\"err\":\"unknown %d\"}", n);
    Serial.println(buf);
  }
  }
}

float getBatteryVoltage()
{
  int raw = analogRead(BATTERY_PIN);
  float voltage = adcToBatteryVoltage(raw, BATTERY_DIVIDER_RATIO);
  if (batterySmoothed < 0) {
    batterySmoothed = voltage;
  } else {
    batterySmoothed = 0.1f * voltage + 0.9f * batterySmoothed;
  }
  return batterySmoothed;
}

static int singleSonarReading()
{
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duration = pulseIn(ECHO, HIGH, 10000);
  return pulseToDistanceCm(duration);
}

int getDistance()
{
  // 3-sample median filter for noise reduction
  int a = singleSonarReading();
  int b = singleSonarReading();
  int c = singleSonarReading();
  return median3(a, b, c);
}

void stop()
{
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void tankDrive(int leftSpeed, int rightSpeed)
{
  // Apply bias correction
  int leftAdjusted = clampMotorSpeed(leftSpeed, leftMotorBias);
  int rightAdjusted = clampMotorSpeed(rightSpeed, rightMotorBias);

  // Left motor
  digitalWrite(AIN1, leftAdjusted >= 0 ? HIGH : LOW);
  analogWrite(PWMA, abs(leftAdjusted));

  // Right motor
  digitalWrite(BIN1, rightAdjusted >= 0 ? HIGH : LOW);
  analogWrite(PWMB, abs(rightAdjusted));
}
