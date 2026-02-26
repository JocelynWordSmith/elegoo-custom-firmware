#ifndef LOGGING_H
#define LOGGING_H

#include "esp_http_server.h"

void addLog(const char *message);
esp_err_t logsHandler(httpd_req_t *req);

#endif
