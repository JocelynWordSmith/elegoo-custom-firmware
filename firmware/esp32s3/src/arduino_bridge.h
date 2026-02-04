#ifndef ARDUINO_BRIDGE_H
#define ARDUINO_BRIDGE_H

void arduinoBridgeInit();
void arduinoBridgeLoop();

// Send command to Arduino and wait for response (thread-safe)
// Returns true if response received, false on timeout
// Response is written to buffer (null-terminated)
bool arduinoBridgeQuery(const char *cmd, char *response, size_t maxLen, unsigned long timeoutMs = 1000);

// Check if a TCP control client is currently connected
bool arduinoBridgeClientConnected();

#endif
