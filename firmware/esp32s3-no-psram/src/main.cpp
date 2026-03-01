#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "esp_camera.h"
#include "camera_pins.h"
#include "credentials.h"
#include "logging.h"
#include "http_server.h"
#include "hid_handlers.h"
#include "serial_relay.h"

const char *FIRMWARE_VERSION = "0.0.1";

void setup()
{
  Serial.begin(115200);
  delay(1000); // brief delay for USB enumeration

  // Initialize HID before any other USB activity so keyboard and mouse
  // are registered with TinyUSB before the first USB enumeration.
  initHID();

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
  config.xclk_freq_hz = 24000000;            // 24MHz - faster pixel clock
  config.frame_size = FRAMESIZE_VGA;         // max buffer size for DRAM (no PSRAM)
  config.pixel_format = PIXFORMAT_JPEG;      // hardware jpeg compression
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY; // wait for buffer to be free
  config.fb_location = CAMERA_FB_IN_DRAM;    // no PSRAM - use internal DRAM
  config.jpeg_quality = 12;                  // 0-63, lower = better quality
  config.fb_count = 1;                       // single buffer to conserve DRAM

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

  // set initial working resolution (VGA = max for DRAM, no PSRAM)
  s->set_framesize(s, FRAMESIZE_VGA);
  // flip vertically; camera is mounted on the bottom, this makes the top of the
  // image the front of the chassis
  s->set_vflip(s, 1);

  // tune sensor for reading a monitor (bright, consistent lighting)
  s->set_ae_level(s, -1);     // reduce auto-exposure for bright screens
  s->set_lenc(s, 1);          // lens correction (reduces edge blur)
  s->set_raw_gma(s, 1);       // gamma correction (improves contrast)
  s->set_wpc(s, 1);           // white pixel correction
  s->set_bpc(s, 1);           // black pixel correction

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

  initSerialRelay();
  char buf[48];
  snprintf(buf, sizeof(buf), "FIRMWARE_VERSION: %s", FIRMWARE_VERSION);
  addLog(buf);
}

// Track WiFi state for reconnection
static unsigned long lastWifiCheck = 0;
static const unsigned long WIFI_CHECK_INTERVAL = 10000;  // check every 10s

void loop()
{
  ArduinoOTA.handle();
  pollSerialRelay();

  // WiFi auto-reconnect
  unsigned long now = millis();
  if (now - lastWifiCheck > WIFI_CHECK_INTERVAL)
  {
    lastWifiCheck = now;
    if (WiFi.status() != WL_CONNECTED)
    {
      addLog("WiFi disconnected, reconnecting...");
      WiFi.reconnect();
    }
  }

  delay(10);
}
