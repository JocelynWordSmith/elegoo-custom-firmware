#include <Arduino.h>
#include <WiFi.h>
#include "esp_http_server.h"

#define LOG_BUFFER_SIZE 50
#define LOG_ENTRY_SIZE 128

static char log_buffer[LOG_BUFFER_SIZE][LOG_ENTRY_SIZE];
static int log_write_index = 0;
static int log_count = 0;

void addLog(const char *message)
{
  // add timestamp prefix
  snprintf(log_buffer[log_write_index], LOG_ENTRY_SIZE,
           "[%lu] %s", millis(), message);

  log_write_index = (log_write_index + 1) % LOG_BUFFER_SIZE;
  if (log_count < LOG_BUFFER_SIZE)
  {
    log_count++;
  }

  // also print to serial
  Serial.println(message);
}

esp_err_t logsHandler(httpd_req_t *req)
{
  // start json array
  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr_chunk(req, "[");

  // calculate oldest entry index
  int start = (log_count < LOG_BUFFER_SIZE) ? 0 : log_write_index;

  for (int i = 0; i < log_count; i++)
  {
    int idx = (start + i) % LOG_BUFFER_SIZE;

    // escape quotes in log message
    char escaped[LOG_ENTRY_SIZE * 2];
    char *dst = escaped;
    *dst++ = '"';
    for (char *src = log_buffer[idx]; *src; src++)
    {
      if (*src == '"' || *src == '\\')
      {
        *dst++ = '\\';
      }
      *dst++ = *src;
    }
    *dst++ = '"';
    *dst = '\0';

    // add comma separator (except first)
    if (i > 0)
    {
      httpd_resp_sendstr_chunk(req, ",");
    }
    httpd_resp_sendstr_chunk(req, escaped);
  }

  httpd_resp_sendstr_chunk(req, "]");
  return httpd_resp_send_chunk(req, NULL, 0); // end chunked response
}
