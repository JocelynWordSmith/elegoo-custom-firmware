#include <Arduino.h>
#include <WiFi.h>
#include "esp_http_server.h"
#include "http_server.h"
#include "camera_handlers.h"
#include "logging.h"

static httpd_handle_t server = NULL;

// handler for root endpoint
static esp_err_t indexHandler(httpd_req_t *req)
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
