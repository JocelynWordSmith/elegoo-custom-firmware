#include <Arduino.h>
#include <Wire.h>
#include <FastLED.h>
#include <Servo.h>

// custom datatypes
struct IRLineReadData
{
  int left;
  int middle;
  int right;
};

struct MPUData
{
  int16_t ax; // Accelerometer
  int16_t ay;
  int16_t az;
  float tempC; // Temperature
  int16_t gx;  // Gyroscope
  int16_t gy;
  int16_t gz;
};

//
// Motor pins
//
const int PWMA = 5; // Left speed
const int PWMB = 6; // Right speed
const int AIN1 = 7; // Left direction
const int BIN1 = 8; // Right direction
const int STBY = 3; // Standby
//
// ultrasonic sensor pins
//
// pin 13 is wired to the onboard LED, and also to the ultrasonic
// connector on the shield, so that particular led cannot be used
// independently without impacting the ultrasonic sensor
const int TRIG = 13; // Trigger (output)
const int ECHO = 12; // Echo (input)
//
// IR line tracking sensor pins
//
// IR sensors output analog voltage based on reflectance
// black surfaces absorb IR → low reading
// white surfaces reflect IR → high reading
// threshold varies by surface; typically 500-700
const int IR_LEFT = A2;   // left IR sensor
const int IR_MIDDLE = A1; // middle IR sensor
const int IR_RIGHT = A0;  // right IR sensor
//
// Battery voltage pin
//
// Elegoo shield has voltage divider on A3
// 2x 18650 = 7.4V nominal (6.0V empty, 8.4V full)
// Voltage divider ratio assumed 1:2 (needs calibration)
const int BATTERY_PIN = A3;
const float BATTERY_DIVIDER_RATIO = 2.0; // adjust based on actual divider
//
// MPU-6050 pins (I2C)
//
// const int MPU_SDA = A4; // I2C data
// const int MPU_SCL = A5; // I2C clock
const int MPU_ADDR = 0x68;
//
// RGB LED
//
const int LED_DATA_PIN = 4;
const int NUM_LEDS = 1;
//
// Servo pins
const int SERVO_Z_PIN = 10;
const int SERVO_Y_PIN = 11;

CRGB leds[NUM_LEDS];

Servo servoZ;         // Pan
Servo servoY;         // Tilt
int motorSpeed = 150; // Global speed setting (0-255)

// Current state tracking
int currentLeftSpeed = 0;  // -255 to 255
int currentRightSpeed = 0; // -255 to 255
int currentPan = 90;       // servo angle
int currentTilt = 90;      // servo angle
uint8_t currentLedR = 0;
uint8_t currentLedG = 0;
uint8_t currentLedB = 0;
uint8_t currentBrightness = 50;

// Motor watchdog
unsigned long lastMotorCommand = 0;
unsigned long watchdogTimeout = 0; // 0 = disabled

// motor control
void forward(int speed);
void backward(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void stop();
// ultrasonic sensor
int getDistance();
// IR line sensors
IRLineReadData getIRLineReadData();
// MPU-6050
MPUData getMPUData();
// battery
float getBatteryVoltage();
// JSON parser
void processCommand(const char *cmd);
int getJsonInt(const char *cmd, const char *field, int defaultVal = 0);

void setup()
{
  Serial.begin(115200);
  // setup fastled
  FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(50); // 0-255

  // setup servos
  servoZ.attach(SERVO_Z_PIN);
  servoY.attach(SERVO_Y_PIN);

  // center position
  servoZ.write(90);
  servoY.write(90);

  // Initialize I2C
  Wire.begin();

  // Wake up MPU-6050 (it starts in sleep mode)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // wakr up (set to 0)
  Wire.endTransmission(true);

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
  // Motor watchdog - stop motors if timeout exceeded
  if (watchdogTimeout > 0 && (currentLeftSpeed != 0 || currentRightSpeed != 0))
  {
    if (millis() - lastMotorCommand > watchdogTimeout)
    {
      stop();
      currentLeftSpeed = 0;
      currentRightSpeed = 0;
    }
  }

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

// Helper to extract integer value from JSON field
int getJsonInt(const char *cmd, const char *field, int defaultVal = 0)
{
  // Find field in string - returns pointer to start of match, or NULL
  const char *found = strstr(cmd, field);
  if (!found)
    return defaultVal;

  // Move pointer past the field name to where the number starts
  const char *start = found + strlen(field);

  // atoi converts string to int, automatically stops at comma/brace/etc
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
  case 2: // Forward (D1=speed, optional)
  {
    int spd = d1 > 0 ? d1 : motorSpeed;
    forward(spd);
    currentLeftSpeed = spd;
    currentRightSpeed = spd;
    lastMotorCommand = millis();
    Serial.println("{\"cmd\":\"forward\"}");
    break;
  }
  case 3: // Backward
  {
    int spd = d1 > 0 ? d1 : motorSpeed;
    backward(spd);
    currentLeftSpeed = -spd;
    currentRightSpeed = -spd;
    lastMotorCommand = millis();
    Serial.println("{\"cmd\":\"backward\"}");
    break;
  }
  case 4: // Turn left
  {
    int spd = d1 > 0 ? d1 : motorSpeed;
    turnLeft(spd);
    currentLeftSpeed = -spd;
    currentRightSpeed = spd;
    lastMotorCommand = millis();
    Serial.println("{\"cmd\":\"left\"}");
    break;
  }
  case 5: // Turn right
  {
    int spd = d1 > 0 ? d1 : motorSpeed;
    turnRight(spd);
    currentLeftSpeed = spd;
    currentRightSpeed = -spd;
    lastMotorCommand = millis();
    Serial.println("{\"cmd\":\"right\"}");
    break;
  }
  case 6: // Stop
    stop();
    currentLeftSpeed = 0;
    currentRightSpeed = 0;
    Serial.println("{\"cmd\":\"stop\"}");
    break;
  case 7: // Tank control: D1=left(-255 to 255), D2=right(-255 to 255)
  {
    // Left motor
    digitalWrite(AIN1, d1 >= 0 ? HIGH : LOW);
    analogWrite(PWMA, abs(d1));
    // Right motor
    digitalWrite(BIN1, d2 >= 0 ? HIGH : LOW);
    analogWrite(PWMB, abs(d2));
    currentLeftSpeed = d1;
    currentRightSpeed = d2;
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
  case 10: // Get distance
  {
    int dist = getDistance();
    Serial.print("{\"distance\":");
    Serial.print(dist);
    Serial.println("}");
    break;
  }
  case 11: // Get IR readings
  {
    IRLineReadData ir = getIRLineReadData();
    Serial.print("{\"ir\":[");
    Serial.print(ir.left);
    Serial.print(",");
    Serial.print(ir.middle);
    Serial.print(",");
    Serial.print(ir.right);
    Serial.println("]}");
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
    Serial.print("{\"battery\":");
    Serial.print(voltage, 2);
    Serial.println("}");
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

  // === Servos ===
  case 30: // Pan servo: D1=angle (0-180)
    currentPan = constrain(d1, 0, 180);
    servoZ.write(currentPan);
    Serial.print("{\"pan\":");
    Serial.print(currentPan);
    Serial.println("}");
    break;
  case 31: // Tilt servo: D1=angle (0-180)
    currentTilt = constrain(d1, 0, 180);
    servoY.write(currentTilt);
    Serial.print("{\"tilt\":");
    Serial.print(currentTilt);
    Serial.println("}");
    break;

  // === Status ===
  case 100: // Get all sensors
  {
    unsigned long t = millis();
    int dist = getDistance();
    IRLineReadData ir = getIRLineReadData();
    MPUData mpu = getMPUData();
    float batt = getBatteryVoltage();

    Serial.print("{\"ts\":");
    Serial.print(t);
    Serial.print(",\"distance\":");
    Serial.print(dist);
    Serial.print(",\"ir\":[");
    Serial.print(ir.left);
    Serial.print(",");
    Serial.print(ir.middle);
    Serial.print(",");
    Serial.print(ir.right);
    Serial.print("],\"accel\":[");
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
    Serial.print("],\"servos\":[");
    Serial.print(currentPan);
    Serial.print(",");
    Serial.print(currentTilt);
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
    Serial.println("}");
    break;
  }
  case 102: // Set watchdog timeout: D1=timeout_ms (0=disable)
    watchdogTimeout = d1;
    Serial.print("{\"watchdog\":");
    Serial.print(watchdogTimeout);
    Serial.println("}");
    break;

  default:
    Serial.print("{\"error\":\"unknown cmd ");
    Serial.print(n);
    Serial.println("\"}");
  }
}

MPUData getMPUData()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting register (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // request 14 bytes

  MPUData result;
  // NOTE: C/C++ doesn't guarantee which Wire.read() executes first.
  // It usually works left-to-right, but it's technically undefined.
  // The safer way is:
  //    int16_t high = Wire.read();
  //    int16_t low = Wire.read();
  //    result.ax = (high << 8) | low;
  //
  // read accelerometer (6 bytes)
  result.ax = Wire.read() << 8 | Wire.read();
  result.ay = Wire.read() << 8 | Wire.read();
  result.az = Wire.read() << 8 | Wire.read();
  // read temperature data
  int16_t temperature = Wire.read() << 8 | Wire.read();
  // convert temperature to celcius and save
  result.tempC = (temperature / 340.0) + 36.53;
  // read gyroscope data
  result.gx = Wire.read() << 8 | Wire.read();
  result.gy = Wire.read() << 8 | Wire.read();
  result.gz = Wire.read() << 8 | Wire.read();

  return result;
}

IRLineReadData getIRLineReadData()
{
  int left = analogRead(IR_LEFT);
  int middle = analogRead(IR_MIDDLE);
  int right = analogRead(IR_RIGHT);

  return IRLineReadData{left, middle, right};
}

float getBatteryVoltage()
{
  int raw = analogRead(BATTERY_PIN);
  // Convert ADC reading to voltage (5V reference, 10-bit ADC)
  float adcVoltage = raw * (5.0 / 1023.0);
  // Apply voltage divider ratio to get actual battery voltage
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

  // measure echo duration
  long duration = pulseIn(ECHO, HIGH, 30000); // 30ms timeout

  // convert to cm (speed of sound = 343 m/s = 0.0343 cm/microsecond)
  // distance = duration * 0.0343 / 2
  int distance = duration / 58; // simplified formula

  return distance;
}

void forward(int speed)
{
  digitalWrite(AIN1, HIGH); // left forward
  digitalWrite(BIN1, HIGH); // right forward
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
}

void backward(int speed)
{
  digitalWrite(AIN1, LOW); // left backward
  digitalWrite(BIN1, LOW); // right backward
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
}

void turnLeft(int speed)
{
  digitalWrite(AIN1, LOW);  // left backward
  digitalWrite(BIN1, HIGH); // right forward
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
}

void turnRight(int speed)
{
  digitalWrite(AIN1, HIGH); // left forward
  digitalWrite(BIN1, LOW);  // right backward
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
}

void stop()
{
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}
