#pragma once

#include "esp_http_server.h"

// Initialize the serial relay RX ring buffer.
void initSerialRelay();

// Poll Serial.available() and buffer any incoming bytes.
// Call this from loop() on every iteration.
void pollSerialRelay();

// POST /terminal/write
// Body: raw text to send to the USB serial port (i.e. to the Ubuntu host).
// On Ubuntu, expose a shell over the serial port with:
//   socat /dev/ttyACM0,raw,echo=0 EXEC:'bash -i',pty,setsid,ctty
// or interactively: screen /dev/ttyACM0 115200
//
// NOTE: existing firmware debug output (Serial.println) also flows through
// this port. Debug lines are prefixed "[DBG]" so agents can filter them.
esp_err_t terminalWriteHandler(httpd_req_t *req);

// GET /terminal/read
// Returns buffered bytes received from the Ubuntu host (JSON string),
// then clears the buffer.
// Response: {"data":"<escaped string>","bytes":<n>}
esp_err_t terminalReadHandler(httpd_req_t *req);
