#include <Arduino.h>
#include <WiFi.h>
#include "arduino_bridge.h"

#define ARDUINO_RX_PIN 3
#define ARDUINO_TX_PIN 40
#define ARDUINO_BAUD 115200
#define CONTROL_PORT 100
#define HEARTBEAT_INTERVAL 10000

static WiFiServer controlServer(CONTROL_PORT);
static WiFiClient controlClient;
static unsigned long lastHeartbeat = 0;

// Mutex to protect Serial1 access from concurrent HTTP queries and TCP relay
static SemaphoreHandle_t serialMutex = NULL;

void arduinoBridgeInit()
{
  // create mutex for serial access
  serialMutex = xSemaphoreCreateMutex();

  // initialize uart to arduino
  Serial1.begin(ARDUINO_BAUD, SERIAL_8N1, ARDUINO_RX_PIN, ARDUINO_TX_PIN);

  // start TCP server
  controlServer.begin();
  controlServer.setNoDelay(true);

  Serial.print("Arduino bridge started on port ");
  Serial.println(CONTROL_PORT);
}

void arduinoBridgeLoop()
{
  // accept new client
  if (!controlClient || !controlClient.connected())
  {
    controlClient = controlServer.available();
    if (controlClient)
    {
      Serial.println("Control client connected");
    }
  }

  if (!controlClient || !controlClient.connected())
  {
    // No client - handle factory detection
    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
      if (Serial1.available())
      {
        String msg = Serial1.readStringUntil('\n');
        msg.trim();

        if (msg == "{BT_detection}")
        {
          Serial.println("{BT_OK}");
        }
        else if (msg == "{WA_detection}")
        {
          Serial.println("{ESP32S3_CAM}");
        }
      }
      xSemaphoreGive(serialMutex);
    }
    return;
  }

  // client connected - relay data (with mutex protection)
  if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(10)) == pdTRUE)
  {
    // TCP -> UART (buffered)
    if (controlClient.available())
    {
      uint8_t buf[128];
      int len = controlClient.read(buf, sizeof(buf));
      if (len > 0)
      {
        Serial1.write(buf, len);
      }
    }

    // UART -> TCP (buffered)
    if (Serial1.available())
    {
      uint8_t buf[128];
      int len = Serial1.read(buf, sizeof(buf));
      if (len > 0)
      {
        controlClient.write(buf, len);
      }
    }

    xSemaphoreGive(serialMutex);
  }

  // send heartbeat
  unsigned long now = millis();
  if (now - lastHeartbeat > HEARTBEAT_INTERVAL)
  {
    controlClient.println("{Heartbeat}");
    lastHeartbeat = now;
  }
}

bool arduinoBridgeClientConnected()
{
  return controlClient && controlClient.connected();
}

bool arduinoBridgeQuery(const char *cmd, char *response, size_t maxLen, unsigned long timeoutMs)
{
  // Take mutex - wait up to timeoutMs
  if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(timeoutMs)) != pdTRUE)
  {
    response[0] = '\0';
    return false;
  }

  // clear any pending data
  while (Serial1.available())
  {
    Serial1.read();
  }

  // send command
  Serial1.println(cmd);

  // wait for response
  unsigned long start = millis();
  size_t idx = 0;

  while (millis() - start < timeoutMs)
  {
    if (Serial1.available())
    {
      char c = Serial1.read();
      if (c == '\n' || c == '\r')
      {
        if (idx > 0)
        {
          response[idx] = '\0';
          xSemaphoreGive(serialMutex);
          return true;
        }
      }
      else if (idx < maxLen - 1)
      {
        response[idx++] = c;
      }
    }
  }

  response[0] = '\0';
  xSemaphoreGive(serialMutex);
  return false;
}
