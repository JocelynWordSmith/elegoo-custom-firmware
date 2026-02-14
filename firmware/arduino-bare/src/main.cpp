#include <Arduino.h>
#include <Wire.h>
#include <FastLED.h>

// custom datatypes
struct MPUData
{
  int16_t ax; // Accelerometer
  int16_t ay;
  int16_t az;
  float tempC; // Temperature
  int16_t gx;  // Gyroscope
  int16_t gy;
  int16_t gz;
  bool valid;  // true if read succeeded
};

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
// MPU-6050 (I2C)
//
const int MPU_ADDR = 0x68;
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
unsigned long lastMotorCommand = 0;
unsigned long watchdogTimeout = 500; // 500ms default (was 0/disabled)

// MPU-6050 status
bool mpuPresent = false;

// Emergency stop config and tracking
int stopDistanceCm = 15;          // obstacle distance threshold
unsigned long lastEstopMs = 0;    // millis() of last estop
unsigned int estopCount = 0;      // total estops since boot
char estopSource[8] = "";         // "front", "test", etc.

// Acceleration curve config
int maxAccelPerTick = 10;         // max speed change per 20ms tick (0-255 range)
float tipThresholdDeg = 25.0;     // pitch angle that limits forward acceleration

// Safety loop timing
unsigned long lastSafetyTick = 0;
const int SAFETY_INTERVAL_MS = 20; // 50Hz

// Cached distance from safety loop (avoids double sonar reads)
int lastDistanceCm = 0;

// Sensor streaming
unsigned long streamIntervalMs = 0; // 0 = disabled
unsigned long lastStreamTime = 0;

// Forward declarations
void forward(int speed);
void backward(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void stop();
void tankDrive(int leftSpeed, int rightSpeed);
int getDistance();
bool testMPU();
MPUData getMPUData();
float getBatteryVoltage();
void processCommand(const char *cmd);
int getJsonInt(const char *cmd, const char *field, int defaultVal = 0);
void doEstop(const char *source, int dist);

void setup()
{
  Serial.begin(115200);
  // setup fastled
  FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(50); // 0-255

  // Initialize I2C
  Wire.begin();

  // Test if MPU-6050 is present
  mpuPresent = testMPU();

  if (mpuPresent) {
    // Wake up MPU-6050 (it starts in sleep mode)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0);    // wake up (set to 0)
    Wire.endTransmission(true);

    // Flash LED green to indicate MPU is ready
    leds[0] = CRGB(0, 255, 0);
    FastLED.show();
    delay(200);
    leds[0] = CRGB(0, 0, 0);
    FastLED.show();
  } else {
    // Flash LED red to indicate MPU not found
    leds[0] = CRGB(255, 0, 0);
    FastLED.show();
    delay(200);
    leds[0] = CRGB(0, 0, 0);
    FastLED.show();
  }

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
}

char inputBuffer[64];
int bufferIndex = 0;

void loop()
{
  // === 50Hz safety loop ===
  if (millis() - lastSafetyTick >= SAFETY_INTERVAL_MS) {
    lastSafetyTick = millis();

    // --- Emergency stop check ---
    lastDistanceCm = getDistance();

    // movingForward: at least one wheel driving forward
    bool movingForward = (targetLeftSpeed > 0 || targetRightSpeed > 0);
    // tooClose: valid reading (>0, since 0 = timeout/out-of-range) AND below threshold
    bool tooClose = (lastDistanceCm > 0 && lastDistanceCm < stopDistanceCm);

    if (movingForward && tooClose) {
      doEstop("front", lastDistanceCm);
    }

    // --- Acceleration ramping with tilt protection ---
    if (mpuPresent) {
      MPUData mpu = getMPUData();
      if (mpu.valid) {
        // Calculate pitch from accelerometer
        // Positive pitch = tilting forward (front-heavy)
        float pitch = atan2((float)mpu.ax, (float)mpu.az) * 180.0 / PI;

        // If tilting forward beyond threshold, don't allow further forward acceleration
        if (pitch > tipThresholdDeg) {
          if (targetLeftSpeed > currentLeftSpeed) targetLeftSpeed = currentLeftSpeed;
          if (targetRightSpeed > currentRightSpeed) targetRightSpeed = currentRightSpeed;
        }
        // If tilting backward beyond threshold, don't allow further backward acceleration
        if (pitch < -tipThresholdDeg) {
          if (targetLeftSpeed < currentLeftSpeed) targetLeftSpeed = currentLeftSpeed;
          if (targetRightSpeed < currentRightSpeed) targetRightSpeed = currentRightSpeed;
        }
      }
    }

    // Ramp current speed toward target
    currentLeftSpeed += constrain(targetLeftSpeed - currentLeftSpeed, -maxAccelPerTick, maxAccelPerTick);
    currentRightSpeed += constrain(targetRightSpeed - currentRightSpeed, -maxAccelPerTick, maxAccelPerTick);

    // --- Apply motor output ---
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
    else if (bufferIndex < 63)
    {
      inputBuffer[bufferIndex++] = c;
    }
  }
}

// Emergency stop helper - used by both real estop and test command
void doEstop(const char *source, int dist) {
  targetLeftSpeed = 0;
  targetRightSpeed = 0;
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
  stop();

  lastEstopMs = millis();
  estopCount++;
  strncpy(estopSource, source, sizeof(estopSource) - 1);
  estopSource[sizeof(estopSource) - 1] = '\0';

  // Emit estop event
  Serial.print("{\"estop\":true,\"source\":\"");
  Serial.print(estopSource);
  Serial.print("\",\"dist\":");
  Serial.print(dist);
  Serial.print(",\"ts\":");
  Serial.print(lastEstopMs);
  Serial.println("}");
}

// Helper to extract integer value from JSON field
int getJsonInt(const char *cmd, const char *field, int defaultVal)
{
  const char *found = strstr(cmd, field);
  if (!found)
    return defaultVal;

  const char *start = found + strlen(field);
  return atoi(start);
}

void processCommand(const char *cmd)
{
  int n = getJsonInt(cmd, "\"N\":", -1);
  if (n == -1)
  {
    Serial.println("{\"error\":\"no N field\"}");
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
  // All motor commands now set TARGET speed; the safety loop ramps and applies.
  case 2: // Forward (D1=speed, optional)
  {
    int spd = d1 > 0 ? d1 : motorSpeed;
    targetLeftSpeed = spd;
    targetRightSpeed = spd;
    lastMotorCommand = millis();
    Serial.println("{\"cmd\":\"forward\"}");
    break;
  }
  case 3: // Backward
  {
    int spd = d1 > 0 ? d1 : motorSpeed;
    targetLeftSpeed = -spd;
    targetRightSpeed = -spd;
    lastMotorCommand = millis();
    Serial.println("{\"cmd\":\"backward\"}");
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
  {
    targetLeftSpeed = d1;
    targetRightSpeed = d2;
    lastMotorCommand = millis();
    Serial.print("{\"tank\":[");
    Serial.print(d1);
    Serial.print(",");
    Serial.print(d2);
    Serial.println("]}");
    break;
  }
  case 8: // Set default speed
    motorSpeed = constrain(d1, 0, 255);
    Serial.print("{\"speed\":");
    Serial.print(motorSpeed);
    Serial.println("}");
    break;

  // === Sensors ===
  case 10: // Get distance (cached from safety loop — avoids TRIG/ECHO conflict)
  {
    Serial.print("{\"distance\":");
    Serial.print(lastDistanceCm);
    Serial.println("}");
    break;
  }
  case 12: // Get MPU data
  {
    MPUData mpu = getMPUData();
    Serial.print("{\"accel\":[");
    Serial.print(mpu.ax);
    Serial.print(",");
    Serial.print(mpu.ay);
    Serial.print(",");
    Serial.print(mpu.az);
    Serial.print("],\"temp\":");
    Serial.print(mpu.tempC, 2);
    Serial.print(",\"gyro\":[");
    Serial.print(mpu.gx);
    Serial.print(",");
    Serial.print(mpu.gy);
    Serial.print(",");
    Serial.print(mpu.gz);
    Serial.println("]}");
    break;
  }
  case 13: // Get battery voltage
  {
    float voltage = getBatteryVoltage();
    int raw = analogRead(BATTERY_PIN);
    Serial.print("{\"battery\":");
    Serial.print(voltage, 2);
    Serial.print(",\"raw\":");
    Serial.print(raw);
    Serial.println("}");
    break;
  }
  case 14: // Calibrate battery: D1=actual voltage * 100 (e.g., 740 = 7.40V)
  {
    float actualVoltage = d1 / 100.0;
    int raw = analogRead(BATTERY_PIN);
    float adcVoltage = raw * (5.0 / 1023.0);
    if (adcVoltage > 0.1) {
      BATTERY_DIVIDER_RATIO = actualVoltage / adcVoltage;
      Serial.print("{\"calibrated_ratio\":");
      Serial.print(BATTERY_DIVIDER_RATIO, 3);
      Serial.print(",\"actual\":");
      Serial.print(actualVoltage, 2);
      Serial.print(",\"adc\":");
      Serial.print(adcVoltage, 2);
      Serial.print(",\"raw\":");
      Serial.print(raw);
      Serial.println("}");
    } else {
      Serial.println("{\"error\":\"voltage too low\"}");
    }
    break;
  }
  case 15: // Set motor bias: D1=left% (80-120), D2=right% (80-120)
  {
    leftMotorBias = constrain(d1, 80, 120) / 100.0;
    rightMotorBias = constrain(d2, 80, 120) / 100.0;
    Serial.print("{\"motor_bias\":[");
    Serial.print(leftMotorBias, 2);
    Serial.print(",");
    Serial.print(rightMotorBias, 2);
    Serial.println("]}");
    break;
  }

  // === LED ===
  case 20: // Set LED color: D1=R, D2=G, D3=B (0-255 each)
    currentLedR = d1;
    currentLedG = d2;
    currentLedB = d3;
    leds[0] = CRGB(d1, d2, d3);
    FastLED.show();
    Serial.print("{\"led\":[");
    Serial.print(d1);
    Serial.print(",");
    Serial.print(d2);
    Serial.print(",");
    Serial.print(d3);
    Serial.println("]}");
    break;
  case 21: // Set LED brightness: D1=brightness (0-255)
    currentBrightness = constrain(d1, 0, 255);
    FastLED.setBrightness(currentBrightness);
    FastLED.show();
    Serial.print("{\"brightness\":");
    Serial.print(d1);
    Serial.println("}");
    break;

  // === Status ===
  case 100: // Get all sensors (uses cached distance from safety loop)
  {
    unsigned long startTime = micros();
    unsigned long t = millis();
    MPUData mpu = getMPUData();
    float batt = getBatteryVoltage();
    unsigned long execTime = micros() - startTime;

    Serial.print("{\"ts\":");
    Serial.print(t);
    Serial.print(",\"execUs\":");
    Serial.print(execTime);
    Serial.print(",\"dist_f\":");
    Serial.print(lastDistanceCm);
    Serial.print(",\"accel\":[");
    Serial.print(mpu.ax);
    Serial.print(",");
    Serial.print(mpu.ay);
    Serial.print(",");
    Serial.print(mpu.az);
    Serial.print("],\"gyro\":[");
    Serial.print(mpu.gx);
    Serial.print(",");
    Serial.print(mpu.gy);
    Serial.print(",");
    Serial.print(mpu.gz);
    Serial.print("],\"temp\":");
    Serial.print(mpu.tempC, 1);
    Serial.print(",\"battery\":");
    Serial.print(batt, 2);
    Serial.print(",\"mpuValid\":");
    Serial.print(mpu.valid ? 1 : 0);
    Serial.println("}");
    break;
  }
  case 101: // Get current state
  {
    Serial.print("{\"ts\":");
    Serial.print(millis());
    Serial.print(",\"motors\":[");
    Serial.print(currentLeftSpeed);
    Serial.print(",");
    Serial.print(currentRightSpeed);
    Serial.print("],\"targets\":[");
    Serial.print(targetLeftSpeed);
    Serial.print(",");
    Serial.print(targetRightSpeed);
    Serial.print("],\"led\":[");
    Serial.print(currentLedR);
    Serial.print(",");
    Serial.print(currentLedG);
    Serial.print(",");
    Serial.print(currentLedB);
    Serial.print("],\"brightness\":");
    Serial.print(currentBrightness);
    Serial.print(",\"speed\":");
    Serial.print(motorSpeed);
    Serial.print(",\"watchdog\":");
    Serial.print(watchdogTimeout);
    Serial.print(",\"motorBias\":[");
    Serial.print(leftMotorBias, 2);
    Serial.print(",");
    Serial.print(rightMotorBias, 2);
    Serial.print("],\"batteryRatio\":");
    Serial.print(BATTERY_DIVIDER_RATIO, 3);
    Serial.print(",\"mpuPresent\":");
    Serial.print(mpuPresent ? 1 : 0);
    Serial.print(",\"safety\":{\"stopDist\":");
    Serial.print(stopDistanceCm);
    Serial.print(",\"maxAccel\":");
    Serial.print(maxAccelPerTick);
    Serial.print(",\"tipDeg\":");
    Serial.print((int)tipThresholdDeg);
    Serial.print("},\"estop\":{\"lastMs\":");
    Serial.print(lastEstopMs);
    Serial.print(",\"count\":");
    Serial.print(estopCount);
    Serial.print(",\"source\":\"");
    Serial.print(estopSource);
    Serial.print("\"},\"stream\":");
    Serial.print(streamIntervalMs);
    Serial.println("}");
    break;
  }
  case 102: // Set watchdog timeout: D1=timeout_ms (0=disable)
    watchdogTimeout = d1;
    Serial.print("{\"watchdog\":");
    Serial.print(watchdogTimeout);
    Serial.println("}");
    break;

  // === New Commands ===
  case 103: // Set sensor stream interval: D1=interval_ms (0=disable)
    streamIntervalMs = d1;
    lastStreamTime = millis();
    Serial.print("{\"stream\":");
    Serial.print(streamIntervalMs);
    Serial.println("}");
    break;

  case 104: // Set safety config: D1=stopDistanceCm, D2=maxAccelPerTick, D3=tipThresholdDeg
  {
    if (d1 > 0) stopDistanceCm = d1;
    if (d2 > 0) maxAccelPerTick = d2;
    if (d3 > 0) tipThresholdDeg = (float)d3;
    Serial.print("{\"safety\":{\"stopDist\":");
    Serial.print(stopDistanceCm);
    Serial.print(",\"maxAccel\":");
    Serial.print(maxAccelPerTick);
    Serial.print(",\"tipDeg\":");
    Serial.print((int)tipThresholdDeg);
    Serial.println("}}");
    break;
  }

  case 105: // Emergency stop test
    doEstop("test", 0);
    break;

  default:
    Serial.print("{\"error\":\"unknown cmd ");
    Serial.print(n);
    Serial.println("\"}");
  }
}

bool testMPU()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75); // WHO_AM_I register
  if (Wire.endTransmission(false) != 0) {
    return false; // I2C error
  }

  if (Wire.requestFrom(MPU_ADDR, 1, true) != 1) {
    return false; // Failed to read
  }

  uint8_t whoami = Wire.read();
  return (whoami == 0x68); // MPU-6050 device ID
}

MPUData getMPUData()
{
  MPUData result = {0, 0, 0, 0.0, 0, 0, 0, false};

  if (!mpuPresent) {
    return result;
  }

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting register (ACCEL_XOUT_H)
  if (Wire.endTransmission(false) != 0) {
    return result; // I2C error
  }

  if (Wire.requestFrom(MPU_ADDR, 14, true) != 14) {
    return result; // Failed to read all bytes
  }

  int16_t high, low;

  high = Wire.read();
  low = Wire.read();
  result.ax = (high << 8) | low;

  high = Wire.read();
  low = Wire.read();
  result.ay = (high << 8) | low;

  high = Wire.read();
  low = Wire.read();
  result.az = (high << 8) | low;

  high = Wire.read();
  low = Wire.read();
  int16_t temperature = (high << 8) | low;
  result.tempC = (temperature / 340.0) + 36.53;

  high = Wire.read();
  low = Wire.read();
  result.gx = (high << 8) | low;

  high = Wire.read();
  low = Wire.read();
  result.gy = (high << 8) | low;

  high = Wire.read();
  low = Wire.read();
  result.gz = (high << 8) | low;

  result.valid = true;
  return result;
}

float getBatteryVoltage()
{
  int raw = analogRead(BATTERY_PIN);
  float adcVoltage = raw * (5.0 / 1023.0);
  return adcVoltage * BATTERY_DIVIDER_RATIO;
}

int getDistance()
{
  // clear trigger
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  // send 10 microsecond pulse
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  // measure echo duration (30ms timeout)
  long duration = pulseIn(ECHO, HIGH, 30000);

  // 0 duration = timeout (nothing detected / out of range)
  // This returns 0, which the estop check treats as "no obstacle"
  int distance = duration / 58;

  return distance;
}

void forward(int speed)
{
  tankDrive(speed, speed);
}

void backward(int speed)
{
  tankDrive(-speed, -speed);
}

void turnLeft(int speed)
{
  tankDrive(-speed, speed);
}

void turnRight(int speed)
{
  tankDrive(speed, -speed);
}

void stop()
{
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void tankDrive(int leftSpeed, int rightSpeed)
{
  // Apply bias correction
  int leftAdjusted = constrain((int)(leftSpeed * leftMotorBias), -255, 255);
  int rightAdjusted = constrain((int)(rightSpeed * rightMotorBias), -255, 255);

  // Left motor
  digitalWrite(AIN1, leftAdjusted >= 0 ? HIGH : LOW);
  analogWrite(PWMA, abs(leftAdjusted));

  // Right motor
  digitalWrite(BIN1, rightAdjusted >= 0 ? HIGH : LOW);
  analogWrite(PWMB, abs(rightAdjusted));
}
