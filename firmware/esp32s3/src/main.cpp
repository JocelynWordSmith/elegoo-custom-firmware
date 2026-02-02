#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "camera_pins.h"
#include "credentials.h"
#include "arduino_bridge.h"
#include "logging.h"

//
// camera settings
//
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static const char *FIRMWARE_VERSION = "0.0.01-OTATEST";

httpd_handle_t server = NULL;
static bool streamActive = false;

esp_err_t streamHandler(httpd_req_t *req)
{
  // only allow one stream at a time
  if (streamActive)
  {
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_send(req, "Stream already active", HTTPD_RESP_USE_STRLEN);
    return ESP_FAIL;
  }
  streamActive = true;

  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  char part_buf[64];

  // set content type for multipart stream
  res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK)
  {
    return res;
  }

  // disable response buffering for streaming
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  addLog("Stream started");

  while (true)
  {
    fb = esp_camera_fb_get();
    if (!fb)
    {
      addLog("Camera capture failed");
      res = ESP_FAIL;
      break;
    }

    // send boundary
    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res != ESP_OK)
    {
      esp_camera_fb_return(fb);
      break;
    }

    // send part header with content length
    size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);
    res = httpd_resp_send_chunk(req, part_buf, hlen);
    if (res != ESP_OK)
    {
      esp_camera_fb_return(fb);
      break;
    }

    // send jpeg data
    res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);

    if (res != ESP_OK)
    {
      break;
    }

    // yield to RTOS - helps detect client disconnect faster
    delay(10);
  }

  addLog("Stream ended");
  streamActive = false;
  return res;
}

// handler for camera control
// examples:
//    set quality: `http://<IP>/control?var=quality&val=5`
//    set resolution: `http://<IP>/control?var=framesize&val=8` (VGA)
//    flip image: `http://<IP>/control?var=vflip&val=1`
esp_err_t controlHandler(httpd_req_t *req)
{
  char buf[128];
  char var[32] = {0};
  char val[32] = {0};

  // parse query string
  if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) != ESP_OK)
  {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  // extract var and val parameters
  if (httpd_query_key_value(buf, "var", var, sizeof(var)) != ESP_OK ||
      httpd_query_key_value(buf, "val", val, sizeof(val)) != ESP_OK)
  {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing var or val");
    return ESP_FAIL;
  }

  int value = atoi(val);
  sensor_t *s = esp_camera_sensor_get();
  int res = 0;

  if (!strcmp(var, "framesize"))
  {
    if (value >= 0 && value <= 13)
    {
      res = s->set_framesize(s, (framesize_t)value);
    }
  }
  else if (!strcmp(var, "quality"))
  {
    //
    // Frame Size Reference
    //
    // | Value | Resolution | Name  |
    // | ----- | ---------- | ----- |
    // | 0     | 96x96      | QQVGA |
    // | 3     | 240x176    | HQVGA |
    // | 5     | 320x240    | QVGA  |
    // | 8     | 640x480    | VGA   |
    // | 9     | 800x600    | SVGA  |
    // | 10    | 1024x768   | XGA   |
    // | 12    | 1280x1024  | SXGA  |
    // | 13    | 1600x1200  | UXGA  |
    res = s->set_quality(s, value);
  }
  else if (!strcmp(var, "brightness"))
  {
    res = s->set_brightness(s, value);
  }
  else if (!strcmp(var, "contrast"))
  {
    res = s->set_contrast(s, value);
  }
  else if (!strcmp(var, "saturation"))
  {
    res = s->set_saturation(s, value);
  }
  else if (!strcmp(var, "vflip"))
  {
    res = s->set_vflip(s, value);
  }
  else if (!strcmp(var, "hmirror"))
  {
    res = s->set_hmirror(s, value);
  }
  else
  {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Unknown variable");
    return ESP_FAIL;
  }

  char response[64];
  snprintf(response, sizeof(response), "{\"var\":\"%s\",\"val\":%d,\"result\":%d}",
           var, value, res);

  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, response, strlen(response));
}

// handler for root endpoint
esp_err_t indexHandler(httpd_req_t *req)
{
  const char *html =
      "<html><body>"
      "<h1>ESP32-S3 Camera Server</h1>"
      "<p>Endpoints:</p>"
      "<ul>"
      "<li><a href='/status'>/status</a> - Server status</li>"
      "<li><a href='/capture'>/capture</a> - Single image</li>"
      "<li><a href='/stream'>/stream</a> - MJPEG stream</li>"
      "</ul>"
      "</body></html>";

  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, html, strlen(html));
}

// handler for status endpoint
esp_err_t statusHandler(httpd_req_t *req)
{
  char json[256];
  snprintf(json, sizeof(json),
           "{\"uptime_ms\":%lu,\"free_heap\":%u,\"wifi_rssi\":%d}",
           millis(),
           ESP.getFreeHeap(),
           WiFi.RSSI());

  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, json, strlen(json));
}

// handler for single image capture
esp_err_t captureHandler(httpd_req_t *req)
{
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb)
  {
    addLog("camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  // set response headers
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");

  // add timestamp header
  char ts[32];
  snprintf(ts, sizeof(ts), "%lu", millis());
  httpd_resp_set_hdr(req, "X-Timestamp", ts);

  // send the jpeg data
  esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);

  // save length before returning buffer
  size_t len = fb->len;
  esp_camera_fb_return(fb);

  Serial.print("sent image: ");
  Serial.print(len);
  Serial.println(" bytes");

  return res;
}

void startServer()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_open_sockets = 7;  // allow more connections (default is 4)
  config.stack_size = 8192;     // larger stack for camera operations
  config.recv_wait_timeout = 5; // 5 second timeout
  config.send_wait_timeout = 5;
  config.lru_purge_enable = true; // close oldest connection when out of sockets

  if (httpd_start(&server, &config) == ESP_OK)
  {
    // register uri handlers
    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = indexHandler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &index_uri);

    httpd_uri_t status_uri = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = statusHandler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &status_uri);

    httpd_uri_t capture_uri = {
        .uri = "/capture",
        .method = HTTP_GET,
        .handler = captureHandler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &capture_uri);

    httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = streamHandler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &stream_uri);

    httpd_uri_t control_uri = {
        .uri = "/control",
        .method = HTTP_GET,
        .handler = controlHandler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &control_uri);

    httpd_uri_t logs_uri = {
        .uri = "/logs",
        .method = HTTP_GET,
        .handler = logsHandler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &logs_uri);

    addLog("HTTP server started on port 80");
  }
  else
  {
    addLog("Failed to start HTTP server");
  }
}

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
