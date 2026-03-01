#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "http_server.h"
#include "camera_handlers.h"
#include "hid_handlers.h"
#include "serial_relay.h"
#include "logging.h"

extern const char *FIRMWARE_VERSION;
extern int activeStreams;  // from camera_handlers.cpp

static httpd_handle_t server = NULL;

// helper: set common JSON response headers (CORS + content type)
static void setJsonResponseHeaders(httpd_req_t *req)
{
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
}

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
      "<p><b>Camera</b></p><ul>"
      "<li><a href='/capture'>/capture</a> - Single JPEG snapshot</li>"
      "<li><a href='/stream'>/stream</a> - MJPEG stream</li>"
      "<li>/control?var=X&val=Y - Camera settings</li>"
      "</ul>"
      "<p><b>HID (USB keyboard + mouse)</b></p><ul>"
      "<li>POST /hid/type - Body: raw text to type (plain text)</li>"
      "<li>/hid/key?key=enter - Send key (dash-separated combos: ctrl-c, alt-f4)</li>"
      "<li>/hid/mouse?dx=0&dy=0&click=left&scroll=0 - Mouse movement/click</li>"
      "</ul>"
      "<p><b>Terminal relay (USB serial &lt;-&gt; WiFi)</b></p><ul>"
      "<li>POST /terminal/write - Body: text to send to /dev/ttyACM0 on host</li>"
      "<li><a href='/terminal/read'>/terminal/read</a> - Read buffered serial input (clears buffer)</li>"
      "<li>On Ubuntu: <code>socat /dev/ttyACM0,raw,echo=0 EXEC:'bash -i',pty,setsid,ctty</code></li>"
      "</ul>"
      "<p><b>System</b></p><ul>"
      "<li><a href='/info'>/info</a> - Firmware, WiFi, camera info</li>"
      "<li><a href='/status'>/status</a> - Uptime, heap, RSSI</li>"
      "<li><a href='/logs'>/logs</a> - Recent log messages</li>"
      "<li>/restart - Reboot ESP32</li>"
      "</ul>"
      "</body></html>";

  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, html, strlen(html));
}

// handler for status endpoint
static esp_err_t statusHandler(httpd_req_t *req)
{
  char json[384];
  snprintf(json, sizeof(json),
           "{"
           "\"uptime_ms\":%lu,"
           "\"free_heap\":%u,"
           "\"min_free_heap\":%u,"
           "\"max_alloc_heap\":%u,"
           "\"wifi_rssi\":%d,"
           "\"active_streams\":%d,"
           "\"cpu_freq_mhz\":%u"
           "}",
           millis(),
           ESP.getFreeHeap(),
           ESP.getMinFreeHeap(),
           ESP.getMaxAllocHeap(),
           WiFi.RSSI(),
           activeStreams,
           getCpuFrequencyMhz());

  setJsonResponseHeaders(req);
  return httpd_resp_send(req, json, strlen(json));
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
           "\"max_alloc_heap\":%u"
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
           ESP.getMaxAllocHeap());

  setJsonResponseHeaders(req);
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
  config.max_open_sockets = 4;  // conserve DRAM (no PSRAM)
  config.stack_size = 6144;     // reduced from 8192 to conserve DRAM
  config.recv_wait_timeout = 5; // 5 second timeout
  config.send_wait_timeout = 5;
  config.lru_purge_enable = true; // close oldest connection when out of sockets
  config.max_uri_handlers = 14;

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

    // HID endpoints
    httpd_uri_t hid_type_uri = {
        .uri = "/hid/type",
        .method = HTTP_POST,
        .handler = hidTypeHandler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &hid_type_uri);

    httpd_uri_t hid_key_uri = {
        .uri = "/hid/key",
        .method = HTTP_GET,
        .handler = hidKeyHandler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &hid_key_uri);

    httpd_uri_t hid_mouse_uri = {
        .uri = "/hid/mouse",
        .method = HTTP_GET,
        .handler = hidMouseHandler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &hid_mouse_uri);

    // Serial relay endpoints
    httpd_uri_t terminal_write_uri = {
        .uri = "/terminal/write",
        .method = HTTP_POST,
        .handler = terminalWriteHandler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &terminal_write_uri);

    httpd_uri_t terminal_read_uri = {
        .uri = "/terminal/read",
        .method = HTTP_GET,
        .handler = terminalReadHandler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &terminal_read_uri);

    addLog("HTTP server started on port 80");
  }
  else
  {
    addLog("Failed to start HTTP server");
  }
}
