#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "camera_pins.h"
#include "credentials.h"

httpd_handle_t server = NULL;

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
    Serial.println("camera capture failed");
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
  config.stack_size = 8192;        // larger stack for camera operations
  config.recv_wait_timeout = 10;   // 10 second timeout
  config.send_wait_timeout = 10;

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

    Serial.println("HTTP server started on port 80");
  }
  else
  {
    Serial.println("Failed to start HTTP server");
  }
}

void setup()
{
  Serial.begin(115200);
  delay(3000); // long delay here so USB can enumerate/monitor can connect

  Serial.println("ESP32-S3 starting...");
  Serial.print("CPU Frequency: ");
  Serial.print(getCpuFrequencyMhz());
  Serial.println(" MHz");

  Serial.println("ESP32-S3 Camera Init...");

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
  config.frame_size = FRAMESIZE_QVGA;        // 320x240
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

  Serial.println("Camera initialized successfully");

  // get sensor info
  sensor_t *s = esp_camera_sensor_get();
  Serial.print("Camera PID: 0x");
  Serial.println(s->id.PID, HEX);

  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);  // disable power saving for lower latency

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
    Serial.println("Wifi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("Starting server...");
    startServer();
  }
  else
  {
    Serial.println();
    Serial.println("Wifi connection failed");
  }
}

void loop()
{
  delay(1000);
}
