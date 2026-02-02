#include <Arduino.h>
#include <WiFi.h>
#include "arduino_bridge.h"

#define ARDUINO_RX_PIN 3
#define ARDUINO_TX_PIN 40
#define ARDUINO_BAUD 9600
#define CONTROL_PORT 100
#define HEARTBEAT_INTERVAL 10000

static WiFiServer controlServer(CONTROL_PORT);
static WiFiClient controlClient;
static unsigned long lastHeartbeat = 0;

void arduinoBridgeInit()
{
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
    // No client - handle factory deletion
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
    return;
  }

  // client connected - relay data

  // TCP -> UART
  while (controlClient.available())
  {
    char c = controlClient.read();
    Serial1.write(c);

    if (c == '\n')
    {
      Serial.println("TX to arduino");
    }
  }

  // UART -> TCP
  while (Serial1.available())
  {
    char c = Serial1.read();
    controlClient.write(c);

    if (c == '\n')
    {
      Serial.println("RX from arduino");
    }
  }

  // send heartbeat
  unsigned long now = millis();
  if (now - lastHeartbeat > HEARTBEAT_INTERVAL)
  {
    controlClient.println("{Heartbeat}");
    lastHeartbeat = now;
  }
}
