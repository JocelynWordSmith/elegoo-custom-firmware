#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "esp_camera.h"
#include "camera_pins.h"
#include "credentials.h"
#include "arduino_bridge.h"
#include "logging.h"
#include "http_server.h"

const char *FIRMWARE_VERSION = "0.0.1";

void setup()
{
  Serial.begin(115200);
  delay(3000); // long delay here so USB can enumerate/monitor can connect

  addLog("ESP32-S3 starting...");
  Serial.print("CPU Frequency: ");
  Serial.print(getCpuFrequencyMhz());
  Serial.println(" MHz");

  addLog("ESP32-S3 Camera Init...");

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;      // PWM channel for XCLK generation
  config.ledc_timer = LEDC_TIMER_0;          // PWM timer for XCLK generation
  config.pin_d0 = Y2_GPIO_NUM;               // camera data bus bit 0
  config.pin_d1 = Y3_GPIO_NUM;               // camera data bus bit 1
  config.pin_d2 = Y4_GPIO_NUM;               // camera data bus bit 2
  config.pin_d3 = Y5_GPIO_NUM;               // camera data bus bit 3
  config.pin_d4 = Y6_GPIO_NUM;               // camera data bus bit 4
  config.pin_d5 = Y7_GPIO_NUM;               // camera data bus bit 5
  config.pin_d6 = Y8_GPIO_NUM;               // camera data bus bit 6
  config.pin_d7 = Y9_GPIO_NUM;               // camera data bus bit 7
  config.pin_xclk = XCLK_GPIO_NUM;           // master clock output to camera
  config.pin_pclk = PCLK_GPIO_NUM;           // pixel clock input from camera
  config.pin_vsync = VSYNC_GPIO_NUM;         // vertical sync (frame start)
  config.pin_href = HREF_GPIO_NUM;           // horizontal ref (line valid)
  config.pin_sccb_sda = SIOD_GPIO_NUM;       // I2C data for camera control
  config.pin_sccb_scl = SIOC_GPIO_NUM;       // I2C clock for camera control
  config.pin_pwdn = PWDN_GPIO_NUM;           // power down pin (-1 = unused)
  config.pin_reset = RESET_GPIO_NUM;         // hardware reset pin (-1 = unused)
  config.xclk_freq_hz = 20000000;            // 20MHz
  config.frame_size = FRAMESIZE_UXGA;        // init with largest for buffer allocation
  config.pixel_format = PIXFORMAT_JPEG;      // hardware jpeg compression
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY; // wait for buffer to be free
  config.fb_location = CAMERA_FB_IN_PSRAM;   // store frames in PSRAM
  config.jpeg_quality = 12;                  // 0-63, lower = better quality
  config.fb_count = 1;                       // number of frame buffers

  // Check for PSRAM
  if (psramFound())
  {
    Serial.print("PSRAM Size: ");
    Serial.print(ESP.getPsramSize() / 1024 / 1024);
    Serial.println(" MB, using 2 frame buffers");
    // psram optimization
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  }

  // initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.print("Camera init failed with error 0x");
    Serial.println(err, HEX);
    return;
  }

  addLog("Camera initialized successfully");

  // get sensor info
  sensor_t *s = esp_camera_sensor_get();
  Serial.print("Camera PID: 0x");
  Serial.println(s->id.PID, HEX);

  // set initial working resolution (buffers sized for UXGA, but start at QVGA)
  s->set_framesize(s, FRAMESIZE_QVGA);
  // flip vertically; camera is mounted on the bottom, this makes the top of the
  // image the front of the chassis
  s->set_vflip(s, 1);

  Serial.print("Connecting to ");
  addLog(ssid);

  WiFi.begin(ssid, password);
  WiFi.setSleep(false); // disable power saving for lower latency

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30)
  {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println();
    char msg[64];
    snprintf(msg, sizeof(msg), "Wifi connected, IP: %s", WiFi.localIP().toString().c_str());
    addLog(msg);
    addLog("Starting server...");
    startServer();
  }
  else
  {
    Serial.println();
    addLog("Wifi connection failed");
  }

  // start arduino bridge
  arduinoBridgeInit();

  // initialize OTA
  ArduinoOTA.setHostname(otaHostName);
  ArduinoOTA.setPassword(otaPassword);

  ArduinoOTA.onStart([]()
                     { addLog("OTA update starting..."); });

  ArduinoOTA.onEnd([]()
                   { addLog("OTA update complete"); });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        {
    static int lastPercent = -1;
    int percent = (progress * 100) / total;
    if (percent != lastPercent && percent %10 == 0) {
      char buf[32];
      snprintf(buf, sizeof(buf), "OTA progress: %d%%", percent);
      addLog(buf);
      lastPercent = percent;
    } });

  ArduinoOTA.onError([](ota_error_t error)
                     {
    char buf[64];
    snprintf(buf, sizeof(buf), "OTA error: %d", error);
    addLog(buf); });

  ArduinoOTA.begin();
  addLog("OTA initialized");
  char buf[48];
  snprintf(buf, sizeof(buf), "FIRMWARE_VERSION: %s", FIRMWARE_VERSION);
  addLog(buf);
}

void loop()
{
  ArduinoOTA.handle();
  arduinoBridgeLoop();
  delay(1);
}
