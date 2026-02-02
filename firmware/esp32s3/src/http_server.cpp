#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "http_server.h"
#include "camera_handlers.h"
#include "arduino_bridge.h"
#include "logging.h"

extern const char *FIRMWARE_VERSION;

static httpd_handle_t server = NULL;

// framesize names for /info endpoint
static const char *framesizeNames[] = {
    "QQVGA", "HQVGA", "QVGA", "", "", "CIF", "", "",
    "VGA", "SVGA", "XGA", "HD", "SXGA", "UXGA"};

// handler for root endpoint
static esp_err_t indexHandler(httpd_req_t *req)
{
  const char *html =
      "<html><body>"
      "<h1>ESP32-S3 Camera Server</h1>"
      "<p>Endpoints:</p>"
      "<ul>"
      "<li><a href='/info'>/info</a> - Firmware, WiFi, camera info</li>"
      "<li><a href='/status'>/status</a> - Uptime, heap, RSSI</li>"
      "<li><a href='/capture'>/capture</a> - Single JPEG image</li>"
      "<li><a href='/stream'>/stream</a> - MJPEG stream</li>"
      "<li>/control?var=X&val=Y - Camera settings</li>"
      "<li><a href='/sensors'>/sensors</a> - Arduino sensors (distance, IR, IMU, battery)</li>"
      "<li><a href='/state'>/state</a> - Arduino state (motors, servos, LED)</li>"
      "<li><a href='/logs'>/logs</a> - Recent log messages</li>"
      "<li>/restart - Reboot ESP32</li>"
      "</ul>"
      "<p>TCP port 100: Arduino UART bridge</p>"
      "</body></html>";

  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, html, strlen(html));
}

// handler for status endpoint
static esp_err_t statusHandler(httpd_req_t *req)
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

// handler for all sensors endpoint (queries Arduino)
static esp_err_t sensorsHandler(httpd_req_t *req)
{
  char response[256];

  if (arduinoBridgeQuery("{\"N\":100}", response, sizeof(response)))
  {
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, response, strlen(response));
  }
  else
  {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Arduino timeout");
    return ESP_FAIL;
  }
}

// handler for current state endpoint (queries Arduino)
static esp_err_t stateHandler(httpd_req_t *req)
{
  char response[256];

  if (arduinoBridgeQuery("{\"N\":101}", response, sizeof(response)))
  {
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, response, strlen(response));
  }
  else
  {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Arduino timeout");
    return ESP_FAIL;
  }
}

// handler for info endpoint (firmware, wifi, camera settings)
static esp_err_t infoHandler(httpd_req_t *req)
{
  char json[512];
  sensor_t *s = esp_camera_sensor_get();

  int framesize = s->status.framesize;
  const char *framesizeName = (framesize >= 0 && framesize <= 13) ? framesizeNames[framesize] : "?";

  snprintf(json, sizeof(json),
           "{"
           "\"firmware\":\"%s\","
           "\"wifi\":{"
           "\"ssid\":\"%s\","
           "\"ip\":\"%s\","
           "\"mac\":\"%s\","
           "\"rssi\":%d"
           "},"
           "\"camera\":{"
           "\"framesize\":%d,"
           "\"framesize_name\":\"%s\","
           "\"quality\":%d,"
           "\"brightness\":%d,"
           "\"contrast\":%d,"
           "\"saturation\":%d,"
           "\"vflip\":%d,"
           "\"hmirror\":%d"
           "},"
           "\"heap\":%u,"
           "\"psram\":%u"
           "}",
           FIRMWARE_VERSION,
           WiFi.SSID().c_str(),
           WiFi.localIP().toString().c_str(),
           WiFi.macAddress().c_str(),
           WiFi.RSSI(),
           framesize,
           framesizeName,
           s->status.quality,
           s->status.brightness,
           s->status.contrast,
           s->status.saturation,
           s->status.vflip,
           s->status.hmirror,
           ESP.getFreeHeap(),
           ESP.getFreePsram());

  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, json, strlen(json));
}

// handler for restart endpoint
static esp_err_t restartHandler(httpd_req_t *req)
{
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, "{\"restarting\":true}", HTTPD_RESP_USE_STRLEN);

  addLog("Restart requested via HTTP");
  delay(500);
  ESP.restart();

  return ESP_OK;
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
  config.max_uri_handlers = 12;   // need additional endpoints

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

    httpd_uri_t sensors_uri = {
        .uri = "/sensors",
        .method = HTTP_GET,
        .handler = sensorsHandler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &sensors_uri);

    httpd_uri_t state_uri = {
        .uri = "/state",
        .method = HTTP_GET,
        .handler = stateHandler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &state_uri);

    httpd_uri_t info_uri = {
        .uri = "/info",
        .method = HTTP_GET,
        .handler = infoHandler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &info_uri);

    httpd_uri_t restart_uri = {
        .uri = "/restart",
        .method = HTTP_GET,
        .handler = restartHandler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &restart_uri);

    addLog("HTTP server started on port 80");
  }
  else
  {
    addLog("Failed to start HTTP server");
  }
}
