#ifndef ROBOT_LOGIC_H
#define ROBOT_LOGIC_H

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

// Extract an integer value from a JSON-like string.
// Searches for `field` (e.g. "\"N\":") and returns the integer that follows.
inline int getJsonInt(const char *cmd, const char *field, int defaultVal = 0)
{
  const char *found = strstr(cmd, field);
  if (!found)
    return defaultVal;
  const char *start = found + strlen(field);
  return atoi(start);
}

// Compute one step of speed ramping toward target.
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
inline float adcToBatteryVoltage(int raw, float dividerRatio, float adcRef = 5.0f, int adcMax = 1023)
{
  float adcVoltage = raw * (adcRef / adcMax);
  return adcVoltage * dividerRatio;
}

// Convert ultrasonic pulse duration (microseconds) to distance in cm.
// Returns -1 if duration is 0 (timeout / no echo).
inline int pulseToDistanceCm(long durationUs)
{
  if (durationUs == 0) return -1;
  return (int)(durationUs / 58);
}

// Return median of three integer values (for sensor filtering).
inline int median3(int a, int b, int c)
{
  if (a > b) { int tmp = a; a = b; b = tmp; }
  if (b > c) { int tmp = b; b = c; c = tmp; }
  if (a > b) { int tmp = a; a = b; b = tmp; }
  return b;
}

#endif // ROBOT_LOGIC_H
