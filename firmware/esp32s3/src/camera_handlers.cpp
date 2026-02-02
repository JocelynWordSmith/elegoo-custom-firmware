#include <Arduino.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "camera_handlers.h"
#include "logging.h"

//
// camera settings
//
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

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
