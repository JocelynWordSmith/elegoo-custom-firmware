#include <Arduino.h>
#include <NimBLEDevice.h>

// Nordic UART Service UUIDs
#define NUS_SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_CHARACTERISTIC   "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // Write (iPhone → ESP32)
#define NUS_TX_CHARACTERISTIC   "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // Notify (ESP32 → iPhone)

// UART to Arduino
#define ARDUINO_RX_PIN 3
#define ARDUINO_TX_PIN 40
#define ARDUINO_BAUD 115200

// BLE device name
#define DEVICE_NAME "ElegooRelay"

// UART receive buffer (Arduino → BLE)
static char uartBuf[256];
static size_t uartBufIdx = 0;
static bool uartOverflowWarned = false;

static NimBLECharacteristic *pTxChar = nullptr;
static bool clientConnected = false;

// --- BLE Callbacks ---

class ServerCallbacks : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer *pServer, ble_gap_conn_desc *desc) override
  {
    clientConnected = true;
    Serial.print("BLE client connected: ");
    Serial.println(NimBLEAddress(desc->peer_ota_addr).toString().c_str());

    // Request fast connection parameters for low-latency control
    // min interval 7.5ms, max interval 15ms, latency 0, timeout 2s
    pServer->updateConnParams(desc->conn_handle, 6, 12, 0, 200);
  }

  void onDisconnect(NimBLEServer *pServer, ble_gap_conn_desc *desc) override
  {
    clientConnected = false;
    Serial.println("BLE client disconnected — sending stop to Arduino");

    // Safety: stop motors on disconnect
    Serial1.println("{\"N\":6}");

    // Restart advertising
    NimBLEDevice::startAdvertising();
    Serial.println("Advertising restarted");
  }
};

class RxCallbacks : public NimBLECharacteristicCallbacks
{
  void onWrite(NimBLECharacteristic *pChar) override
  {
    std::string value = pChar->getValue();
    if (value.length() > 0)
    {
      // Debug log incoming BLE data
      Serial.print("BLE -> : ");
      Serial.println(value.c_str());

      // Forward BLE data to Arduino via UART
      Serial1.write((const uint8_t *)value.data(), value.length());

      // Ensure newline termination (Arduino expects \n-delimited commands)
      if (value.back() != '\n')
      {
        Serial1.write('\n');
      }
    }
  }
};

void setup()
{
  // USB serial for debug
  Serial.begin(115200);
  delay(500);
  Serial.println("ESP32-S3 BLE Relay starting...");

  // UART to Arduino
  Serial1.begin(ARDUINO_BAUD, SERIAL_8N1, ARDUINO_RX_PIN, ARDUINO_TX_PIN);
  Serial.println("UART initialized (GPIO3 RX, GPIO40 TX @ 115200)");

  // Initialize NimBLE
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setMTU(247); // Allow larger packets for JSON responses

  NimBLEServer *pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Create Nordic UART Service
  NimBLEService *pService = pServer->createService(NUS_SERVICE_UUID);

  // TX characteristic (ESP32 → iPhone, notify)
  pTxChar = pService->createCharacteristic(
      NUS_TX_CHARACTERISTIC,
      NIMBLE_PROPERTY::NOTIFY);

  // RX characteristic (iPhone → ESP32, write)
  NimBLECharacteristic *pRxChar = pService->createCharacteristic(
      NUS_RX_CHARACTERISTIC,
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  pRxChar->setCallbacks(new RxCallbacks());

  pService->start();

  // Configure advertising
  NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
  pAdv->addServiceUUID(NUS_SERVICE_UUID);
  pAdv->setScanResponse(true);
  pAdv->start();

  Serial.print("BLE advertising as \"");
  Serial.print(DEVICE_NAME);
  Serial.println("\"");
}

void loop()
{
  // Read UART data from Arduino, buffer until newline, then notify BLE client
  while (Serial1.available())
  {
    char c = Serial1.read();

    if (c == '\n' || c == '\r')
    {
      if (uartBufIdx > 0)
      {
        uartBuf[uartBufIdx] = '\0';

        if (clientConnected && pTxChar)
        {
          pTxChar->setValue((uint8_t *)uartBuf, uartBufIdx);
          pTxChar->notify();
        }

        // Debug echo
        Serial.print("-> BLE: ");
        Serial.println(uartBuf);

        uartBufIdx = 0;
        uartOverflowWarned = false;
      }
    }
    else if (uartBufIdx < sizeof(uartBuf) - 1)
    {
      uartBuf[uartBufIdx++] = c;
    }
    else
    {
      if (!uartOverflowWarned)
      {
        Serial.println("WARN: UART buf overflow");
        uartOverflowWarned = true;
      }
    }
  }
}
