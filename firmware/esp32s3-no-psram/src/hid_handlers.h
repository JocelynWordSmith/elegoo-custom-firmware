#pragma once

#include "esp_http_server.h"

// Initialize USB HID keyboard and mouse.
// Must be called in setup() before any USB activity.
void initHID();

// POST /hid/type
// Body: raw text to type (plain text, up to 256 bytes per call)
esp_err_t hidTypeHandler(httpd_req_t *req);

// GET /hid/key?key=<combo>
// Single key or dash-separated modifier combo.
// Examples: key=enter  key=ctrl-c  key=ctrl-shift-t  key=alt-f4
// Supported modifiers: ctrl, shift, alt, super/cmd/win
// Supported keys: enter, tab, esc, backspace, delete, insert,
//                 up, down, left, right, home, end, pageup, pagedown,
//                 space, f1-f12, and any single printable character
esp_err_t hidKeyHandler(httpd_req_t *req);

// GET /hid/mouse?dx=<n>&dy=<n>&click=<btn>&scroll=<n>
// All params optional. dx/dy: relative movement (-127..127).
// click: left | right | middle   scroll: negative=down, positive=up
esp_err_t hidMouseHandler(httpd_req_t *req);
