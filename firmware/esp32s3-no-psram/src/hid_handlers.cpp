#include <Arduino.h>
#include <USB.h>
#include <USBHIDKeyboard.h>
#include <USBHIDMouse.h>
#include "hid_handlers.h"
#include "logging.h"

// Declare at global scope so constructors run before USB.begin() is called
// by the CDC-on-boot initialization. Both classes self-register with TinyUSB
// in their constructors, and will be included in the USB descriptor.
static USBHIDKeyboard keyboard;
static USBHIDMouse    mouse;

void initHID()
{
  keyboard.begin();
  mouse.begin();
  // USB.begin() is called automatically by CDC-on-boot, but call it here
  // as a no-op safety net in case the boot sequence changes.
  USB.begin();
  addLog("HID ready (keyboard + mouse)");
}

// Key name -> HID keycode. Accepts lowercase names.
static uint8_t lookupKey(const char *name)
{
  // Special keys
  if (strcmp(name, "enter")    == 0 || strcmp(name, "return") == 0) return KEY_RETURN;
  if (strcmp(name, "tab")      == 0)                                 return KEY_TAB;
  if (strcmp(name, "esc")      == 0 || strcmp(name, "escape") == 0) return KEY_ESC;
  if (strcmp(name, "backspace")== 0)                                 return KEY_BACKSPACE;
  if (strcmp(name, "delete")   == 0)                                 return KEY_DELETE;
  if (strcmp(name, "insert")   == 0)                                 return KEY_INSERT;
  if (strcmp(name, "up")       == 0)                                 return KEY_UP_ARROW;
  if (strcmp(name, "down")     == 0)                                 return KEY_DOWN_ARROW;
  if (strcmp(name, "left")     == 0)                                 return KEY_LEFT_ARROW;
  if (strcmp(name, "right")    == 0)                                 return KEY_RIGHT_ARROW;
  if (strcmp(name, "home")     == 0)                                 return KEY_HOME;
  if (strcmp(name, "end")      == 0)                                 return KEY_END;
  if (strcmp(name, "pageup")   == 0)                                 return KEY_PAGE_UP;
  if (strcmp(name, "pagedown") == 0)                                 return KEY_PAGE_DOWN;
  if (strcmp(name, "space")    == 0)                                 return ' ';
  if (strcmp(name, "capslock") == 0)                                 return KEY_CAPS_LOCK;
  // Modifiers
  if (strcmp(name, "ctrl")  == 0 || strcmp(name, "control") == 0)   return KEY_LEFT_CTRL;
  if (strcmp(name, "shift") == 0)                                    return KEY_LEFT_SHIFT;
  if (strcmp(name, "alt")   == 0)                                    return KEY_LEFT_ALT;
  if (strcmp(name, "super") == 0 || strcmp(name, "cmd") == 0 ||
      strcmp(name, "win")   == 0)                                    return KEY_LEFT_GUI;
  // Function keys
  if (strcmp(name, "f1")  == 0) return KEY_F1;
  if (strcmp(name, "f2")  == 0) return KEY_F2;
  if (strcmp(name, "f3")  == 0) return KEY_F3;
  if (strcmp(name, "f4")  == 0) return KEY_F4;
  if (strcmp(name, "f5")  == 0) return KEY_F5;
  if (strcmp(name, "f6")  == 0) return KEY_F6;
  if (strcmp(name, "f7")  == 0) return KEY_F7;
  if (strcmp(name, "f8")  == 0) return KEY_F8;
  if (strcmp(name, "f9")  == 0) return KEY_F9;
  if (strcmp(name, "f10") == 0) return KEY_F10;
  if (strcmp(name, "f11") == 0) return KEY_F11;
  if (strcmp(name, "f12") == 0) return KEY_F12;
  // Single printable character
  if (strlen(name) == 1)        return (uint8_t)name[0];
  return 0;
}

// Lowercase a string in place
static void strToLower(char *s)
{
  for (; *s; s++) {
    if (*s >= 'A' && *s <= 'Z') *s += 32;
  }
}

// Parse "ctrl-shift-t" into an array of keycodes.
// Returns count of valid codes found.
static int parseCombo(const char *combo, uint8_t *codes, int maxCodes)
{
  char buf[64];
  strncpy(buf, combo, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';
  strToLower(buf);

  int count = 0;
  char *tok = strtok(buf, "-");
  while (tok && count < maxCodes) {
    uint8_t code = lookupKey(tok);
    if (code != 0) codes[count++] = code;
    tok = strtok(NULL, "-");
  }
  return count;
}

// POST /hid/type  (body = raw text, up to 256 bytes)
esp_err_t hidTypeHandler(httpd_req_t *req)
{
  if (req->content_len == 0) {
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"error\":\"empty body\"}", HTTPD_RESP_USE_STRLEN);
  }

  size_t cap = (req->content_len < 256) ? req->content_len : 256;
  char body[257] = {0};

  int got = httpd_req_recv(req, body, cap);
  if (got <= 0) {
    httpd_resp_set_status(req, "500 Internal Server Error");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"error\":\"recv failed\"}", HTTPD_RESP_USE_STRLEN);
  }
  body[got] = '\0';

  keyboard.print(body);

  char resp[48];
  snprintf(resp, sizeof(resp), "{\"typed\":%d}", got);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
}

// GET /hid/key?key=ctrl-c
esp_err_t hidKeyHandler(httpd_req_t *req)
{
  char query[128] = {0};
  char keyVal[64] = {0};

  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "key", keyVal, sizeof(keyVal)) != ESP_OK) {
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"error\":\"missing ?key=...\"}", HTTPD_RESP_USE_STRLEN);
  }

  uint8_t codes[6] = {0};
  int count = parseCombo(keyVal, codes, 6);

  if (count == 0) {
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"error\":\"unknown key\"}", HTTPD_RESP_USE_STRLEN);
  }

  for (int i = 0; i < count; i++) {
    keyboard.press(codes[i]);
    delay(5);
  }
  delay(50);
  keyboard.releaseAll();

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

// GET /hid/mouse?dx=10&dy=5&click=left&scroll=-1
esp_err_t hidMouseHandler(httpd_req_t *req)
{
  char query[128] = {0};
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"error\":\"missing query\"}", HTTPD_RESP_USE_STRLEN);
  }

  char val[32];
  int dx = 0, dy = 0, scroll = 0;
  uint8_t clickBtn = 0;

  if (httpd_query_key_value(query, "dx",     val, sizeof(val)) == ESP_OK) dx     = atoi(val);
  if (httpd_query_key_value(query, "dy",     val, sizeof(val)) == ESP_OK) dy     = atoi(val);
  if (httpd_query_key_value(query, "scroll", val, sizeof(val)) == ESP_OK) scroll = atoi(val);
  if (httpd_query_key_value(query, "click",  val, sizeof(val)) == ESP_OK) {
    strToLower(val);
    if (strcmp(val, "left")   == 0) clickBtn = MOUSE_LEFT;
    else if (strcmp(val, "right")  == 0) clickBtn = MOUSE_RIGHT;
    else if (strcmp(val, "middle") == 0) clickBtn = MOUSE_MIDDLE;
  }

  // Clamp to int8_t movement range
  if (dx >  127) dx =  127;
  if (dx < -127) dx = -127;
  if (dy >  127) dy =  127;
  if (dy < -127) dy = -127;

  if (dx != 0 || dy != 0 || scroll != 0) {
    mouse.move((int8_t)dx, (int8_t)dy, (int8_t)scroll);
  }
  if (clickBtn != 0) {
    mouse.click(clickBtn);
  }

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}
