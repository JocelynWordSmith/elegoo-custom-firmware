#ifndef ROBOT_LOGIC_H
#define ROBOT_LOGIC_H

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

// ── Pure logic functions (no hardware dependencies) ─────────────────────────

// Extract an integer value from a JSON-like string.
// Searches for `field` (e.g. "\"N\":") and returns the integer that follows.
// Returns defaultVal if the field is not found.
inline int getJsonInt(const char *cmd, const char *field, int defaultVal = 0)
{
  const char *found = strstr(cmd, field);
  if (!found)
    return defaultVal;

  const char *start = found + strlen(field);
  return atoi(start);
}

// Compute one step of speed ramping.
// Moves `current` toward `target` by at most `maxAccel` per tick.
inline int rampSpeed(int current, int target, int maxAccel)
{
  int delta = target - current;
  if (delta > maxAccel) delta = maxAccel;
  if (delta < -maxAccel) delta = -maxAccel;
  return current + delta;
}

// Apply motor bias and clamp to [-255, 255].
inline int clampMotorSpeed(int speed, float bias)
{
  int adjusted = (int)(speed * bias);
  if (adjusted > 255) adjusted = 255;
  if (adjusted < -255) adjusted = -255;
  return adjusted;
}

// Convert raw ADC reading to battery voltage.
// adcRef is the ADC reference voltage (5.0V for Arduino Uno).
// adcMax is the max ADC value (1023 for 10-bit).
inline float adcToBatteryVoltage(int raw, float dividerRatio, float adcRef = 5.0f, int adcMax = 1023)
{
  float adcVoltage = raw * (adcRef / adcMax);
  return adcVoltage * dividerRatio;
}

// Convert raw MPU-6050 temperature register to Celsius.
inline float rawToTempC(int16_t raw)
{
  return (raw / 340.0f) + 36.53f;
}

// Convert ultrasonic pulse duration (microseconds) to distance in cm.
// Returns 0 if duration is 0 (timeout / no echo).
inline int pulseToDistanceCm(long durationUs)
{
  return (int)(durationUs / 58);
}

#endif // ROBOT_LOGIC_H
