#include <Arduino.h>
#include "serial_relay.h"
#include "logging.h"

// Ring buffer for data received from the USB serial port (Ubuntu → ESP32).
// 1 KB is enough for typical shell command output between polls.
#define RELAY_BUF_SIZE 1024

static char  relay_buf[RELAY_BUF_SIZE];
static int   relay_head = 0;  // next write position
static int   relay_count = 0; // bytes currently stored

// Append a byte to the ring buffer (oldest byte dropped if full)
static void relayPush(char c)
{
  if (relay_count < RELAY_BUF_SIZE) {
    relay_buf[(relay_head + relay_count) % RELAY_BUF_SIZE] = c;
    relay_count++;
  } else {
    // Buffer full: overwrite oldest byte
    relay_buf[relay_head] = c;
    relay_head = (relay_head + 1) % RELAY_BUF_SIZE;
  }
}

void initSerialRelay()
{
  relay_head  = 0;
  relay_count = 0;
  addLog("Serial relay ready");
}

void pollSerialRelay()
{
  while (Serial.available()) {
    relayPush((char)Serial.read());
  }
}

// POST /terminal/write
esp_err_t terminalWriteHandler(httpd_req_t *req)
{
  if (req->content_len == 0) {
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"error\":\"empty body\"}", HTTPD_RESP_USE_STRLEN);
  }

  size_t cap = (req->content_len < 512) ? req->content_len : 512;
  char body[513] = {0};

  int got = httpd_req_recv(req, body, cap);
  if (got <= 0) {
    httpd_resp_set_status(req, "500 Internal Server Error");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"error\":\"recv failed\"}", HTTPD_RESP_USE_STRLEN);
  }

  // Write to the USB CDC serial port — Ubuntu reads this from /dev/ttyACM0
  Serial.write((uint8_t *)body, got);

  char resp[48];
  snprintf(resp, sizeof(resp), "{\"sent\":%d}", got);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
}

// GET /terminal/read
esp_err_t terminalReadHandler(httpd_req_t *req)
{
  // Snapshot and clear the buffer atomically enough for single-core use
  int count = relay_count;
  char snapshot[RELAY_BUF_SIZE + 1];

  for (int i = 0; i < count; i++) {
    snapshot[i] = relay_buf[(relay_head + i) % RELAY_BUF_SIZE];
  }
  snapshot[count] = '\0';

  relay_head  = 0;
  relay_count = 0;

  // Build JSON response: {"data":"<escaped>","bytes":<n>}
  // Max escaped size: 6 bytes per char (\uXXXX) + quotes + key + count field
  // Use chunked send to avoid a large stack allocation
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char header[48];
  snprintf(header, sizeof(header), "{\"bytes\":%d,\"data\":\"", count);
  httpd_resp_sendstr_chunk(req, header);

  // Escape the data for JSON
  char esc[8];
  for (int i = 0; i < count; i++) {
    unsigned char c = (unsigned char)snapshot[i];
    if (c == '"')       { httpd_resp_sendstr_chunk(req, "\\\""); }
    else if (c == '\\') { httpd_resp_sendstr_chunk(req, "\\\\"); }
    else if (c == '\n') { httpd_resp_sendstr_chunk(req, "\\n");  }
    else if (c == '\r') { httpd_resp_sendstr_chunk(req, "\\r");  }
    else if (c == '\t') { httpd_resp_sendstr_chunk(req, "\\t");  }
    else if (c >= 0x20 && c < 0x7F) {
      esc[0] = (char)c; esc[1] = '\0';
      httpd_resp_sendstr_chunk(req, esc);
    } else {
      // Non-printable: hex escape
      snprintf(esc, sizeof(esc), "\\u%04x", c);
      httpd_resp_sendstr_chunk(req, esc);
    }
  }

  httpd_resp_sendstr_chunk(req, "\"}");
  return httpd_resp_send_chunk(req, NULL, 0);
}
