#include <Arduino.h>

// ── Modem ────────────────────────────────────────────
#define TINY_GSM_MODEM_SIM7600

// ── Serial ───────────────────────────────────────────
#define SerialMon   Serial
#define SerialAT    Serial1

// ── GSM Pins ─────────────────────────────────────────
#define GSM_RX      26
#define GSM_TX      27

// ── W5500 Ethernet Pins ──────────────────────────────
#define W5500_MOSI  23
#define W5500_MISO  19
#define W5500_SCLK  18
#define W5500_CS    21
#define W5500_RST   13
#define W5500_INT   4

// ── TFT Display Pins ─────────────────────────────────
#define TFT_MOSI       23
#define TFT_MISO       19
#define TFT_SCLK       18
#define TFT_CS         5
#define TFT_DC         4
#define TFT_RST        -1
#define MCP_CS         27
#define TFT_LED_PIN    1
#define TFT_RESET_PIN  9

// ── RS485 Pins ───────────────────────────────────────
#define S0          25
#define S1          26
#define DIR         15
#define RXD2        16
#define TXD2        17

// ── Button Pins ──────────────────────────────────────
#define BUTTON_PIN          35
#define resetbutton         25
#define WIFI_RESET_PIN      3
#define DEBOUNCE_DELAY      50

// ── NTP ──────────────────────────────────────────────
#define GMT_OFFSET_SEC       19800
#define DAYLIGHT_OFFSET_SEC  0

// ── GSM Retry ────────────────────────────────────────
#define MAX_GSM_RETRY_COUNT   10
#define GSM_INITIAL_RETRY_MS  5000
#define GSM_MAX_RETRY_MS      60000

// ── MQTT Topics ──────────────────────────────────────
#define FIRMWARE_TOPIC         "0679ag3/firmware_update"
#define CONFIG_TOPIC           "0679ag3/config"
#define CONFIG_RESPONSE_TOPIC  "0679ag3/config/response"

// ── Default MQTT Credentials ─────────────────────────
#define DEFAULT_MQTT_SERVER  "mqtt-dashboard.com"
#define DEFAULT_MQTT_PORT    "1883"
#define DEFAULT_MQTT_USER    ""
#define DEFAULT_MQTT_PASS    ""
#define DEFAULT_MQTT_METHOD  "wifi"

// ── Default RS485 Settings ───────────────────────────
#define DEFAULT_SLAVE_BAUD     "9600"
#define DEFAULT_SLAVE_DATALEN  "8"
#define DEFAULT_SLAVE_PARITY   "N"
#define DEFAULT_SLAVE_STOPBIT  "2"
#define DEFAULT_PROTOCOL       "rs485"
#define DEFAULT_INTERVAL_MS    "500"

// ── Default Ethernet Settings ────────────────────────
#define DEFAULT_ETH_MAC    "02:00:03:00:00:01"
#define DEFAULT_STATIC_IP  "192.168.1.100"
#define DEFAULT_GATEWAY    "192.168.1.1"
#define DEFAULT_SUBNET     "255.255.255.0"
#define DEFAULT_IP_MODE    "Auto"


#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>
#include "FS.h"
#include <LittleFS.h>
#include <MQTT.h>
#include "esp_partition.h"
#include <ArduinoHttpClient.h>
#include "SSLClient.h"


#include <SPI.h>
#include "Ucglib.h"
#include <Adafruit_MCP23X17.h>
#include "logo565.h"
#include <Ethernet.h>

#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <ModbusEthernet.h>
#include <ModbusIP_ESP8266.h>
#include <ModbusRTU.h>
#include <ModbusTCP.h>
#include <TinyGsmClient.h>

#include <vector>
#include <IPAddress.h>
#include <map>
#include <String>
#include <time.h>
#include <Ticker.h>
#include <numeric>
#include <algorithm>
#include <Update.h>
#include "CRC32.h"


// ══════════════════════════════════════════════════════
//  MODEM & GSM
// ══════════════════════════════════════════════════════
TinyGsm modem(SerialAT);                          // GSM modem instance on Serial1
TinyGsmClient client(modem);                       // Generic GSM TCP client
TinyGsmClient base_client(modem, 0);               // GSM client for OTA firmware download

// Use this instead:
WiFiClientSecure wifiSecureClient;
// In setup() or before OTA call:
// wifiSecureClient.setCACert(root_ca);  // uses your cert.h PEM string
// OR
// wifiSecureClient.setInsecure();       // skip cert check (less secure, but works)
            // SSL wrapper over GSM client for HTTPS OTA
TinyGsmClient gsmClient(modem);                    // GSM client for MQTT communication
MQTTClient gsmClientPubSub(2048, 2048);            // MQTT client over GSM (2KB send/recv buffer)

char apn[] = "";                                   // SIM card APN (set via config or hardcode)
char gprsUser[] = "";                              // GPRS username (usually empty)
char gprsPass[] = "";                              // GPRS password (usually empty)

// ══════════════════════════════════════════════════════
//  WiFi
// ══════════════════════════════════════════════════════
WiFiManager wm;                                    // WiFiManager for captive portal AP setup
WiFiClient espClient232;                           // Plain WiFi TCP client for MQTT
MQTTClient wifiClientPubSub(2048, 2048);           // MQTT client over WiFi (2KB send/recv buffer)

// ══════════════════════════════════════════════════════
//  ETHERNET (W5500)
// ══════════════════════════════════════════════════════
EthernetClient ethClient;                          // W5500 TCP client for MQTT
MQTTClient ethernetClientPubSub(2048, 2048);       // MQTT client over Ethernet (2KB send/recv buffer)
byte mac[6];                                       // MAC address bytes parsed from ethMacAddr string

// ══════════════════════════════════════════════════════
//  TFT DISPLAY
// ══════════════════════════════════════════════════════
Adafruit_MCP23X17 mcp;
Ucglib_ILI9341_18x240x320_HWSPI ucg(TFT_DC, TFT_CS, TFT_RST);

enum DisplayState {
  LOGO_PAGE,
  SENSOR_PAGE
};

struct DisplaySensorData {
  float frequency;
  float vlnAvg;
  float vllAvg;
  bool dataValid;
};

DisplayState currentDisplayState = LOGO_PAGE;
DisplaySensorData displaySensorData = {0.0f, 0.0f, 0.0f, false};
unsigned long displayStateChangeTime = 0;
const unsigned long LOGO_DURATION = 2000;
const unsigned long SENSOR_DURATION = 12000;
bool displayReady = false;
bool displayPageNeedsRedraw = true;
int bottleAnimFrame = 0;

// ══════════════════════════════════════════════════════
//  CONFIG CHAR ARRAYS (saved/loaded from LittleFS)
// ══════════════════════════════════════════════════════
char mqttserver[40]      = DEFAULT_MQTT_SERVER;    // MQTT broker hostname
char mqttport[6]         = DEFAULT_MQTT_PORT;      // MQTT broker port
char mqttusername[40]    = DEFAULT_MQTT_USER;      // MQTT login username
char mqttpassword[40]    = DEFAULT_MQTT_PASS;      // MQTT login password
char mqttMethod[10]      = DEFAULT_MQTT_METHOD;    // Connection method: "wifi" / "gsm" / "ethernet"

char slavebaudrate[40]   = DEFAULT_SLAVE_BAUD;     // RS485 baud rate
char slavedatalength[40] = DEFAULT_SLAVE_DATALEN;  // RS485 data bits (7 or 8)
char slaveparity[40]     = DEFAULT_SLAVE_PARITY;   // RS485 parity: "N"/"E"/"O"
char slavestopbit[40]    = DEFAULT_SLAVE_STOPBIT;  // RS485 stop bits (1 or 2)

char protocolType[20]    = DEFAULT_PROTOCOL;       // Modbus protocol: "rs485" / "tcpethernet" / "tcpwifi"
char intervalssParam[10] = DEFAULT_INTERVAL_MS;    // Modbus polling interval in milliseconds

char ethMacAddr[18]      = DEFAULT_ETH_MAC;        // W5500 MAC address string "XX:XX:XX:XX:XX:XX"
char staticIP[16]        = DEFAULT_STATIC_IP;      // Static IP for Ethernet (Manual mode)
char gatewayIPs[16]      = DEFAULT_GATEWAY;        // Gateway IP for Ethernet
char subnetMasks[16]     = DEFAULT_SUBNET;         // Subnet mask for Ethernet
char ipMode[10]          = DEFAULT_IP_MODE;        // "Auto" = DHCP, "Manual" = static IP

char slaveconfigREAD[10240];                       // 10KB buffer for Modbus READ config JSON from MQTT
char slaveconfigWRITE[10240];                      // 10KB buffer for Modbus WRITE config JSON from MQTT
char baseTopicPath[40]       = "";                // Base MQTT topic for grouped device publishing

// ══════════════════════════════════════════════════════
//  GROUPED DEVICE DATA (for JSON publishing)
// ══════════════════════════════════════════════════════
std::map<uint8_t, std::map<uint16_t, float>> deviceRegisterValues;  // deviceId -> (address -> value)
Ticker publishGroupedTicker;                       // Timer for 10-second grouped publish interval
bool useGroupedPublishing = false;                 // Flag: use grouped JSON publishing vs individual topics

// ══════════════════════════════════════════════════════
//  NTP (Network Time Protocol)
// ══════════════════════════════════════════════════════
const int ntpMaxRetries = 5;                       // Max NTP sync attempts before giving up
const int ntpRetryDelay = 5000;                    // Delay between NTP retries (ms)

// ══════════════════════════════════════════════════════
//  TICKER & TIME
// ══════════════════════════════════════════════════════
Ticker midnightTicker;                             // Timer to trigger daily midnight reset logic
int lastResetDay = -1;                             // Tracks last day a midnight reset occurred

// ══════════════════════════════════════════════════════
//  OTA (Over-The-Air Update)
// ══════════════════════════════════════════════════════
bool updating = false;                             // True while OTA firmware update is in progress
int totalSize = 0;                                 // Total firmware size in bytes
int receivedSize = 0;                              // Bytes received so far during OTA

// ══════════════════════════════════════════════════════
//  MODBUS
// ══════════════════════════════════════════════════════
ModbusRTU mb;                                      // Modbus RTU master (RS485)
ModbusIP modbusTCPWiFi;                            // Modbus TCP master over WiFi
ModbusEthernet modbusTCPEthernet;                  // Modbus TCP master over Ethernet
std::vector<IPAddress> connectedSlaveIPs;          // List of connected Modbus TCP slave IPs
bool callbackResult = false;                       // Result flag set by Modbus callback
bool requestProcessed = false;                     // Flag: current Modbus request completed
bool requestInProgress = false;                    // Flag: a Modbus request is currently pending

// ══════════════════════════════════════════════════════
//  DATA MAPS (MQTT publish & storage tracking)
// ══════════════════════════════════════════════════════
std::map<String, String> topicValueMap;            // Latest value for each MQTT topic
std::map<String, String> configMap;                // Parsed config key-value pairs
std::map<String, unsigned long> lastSendTime;      // Last MQTT publish timestamp per topic
std::map<String, String> lastSendValue;            // Last published value per topic
std::map<String, std::map<int, String>> lastSentTimes;  // Scheduled send tracking (date-based)
std::map<String, std::vector<float>> dataBuffer;   // Rolling buffer of recent values per topic
std::map<String, unsigned long> lastBufferUpdate;  // Last time dataBuffer was updated per topic
std::map<String, unsigned long> lastStorageTime;   // Last LittleFS storage timestamp per topic
std::map<String, String> lastStoredValue;          // Last value stored to LittleFS per topic
std::map<String, std::map<int, bool>> lastStoredTimes;  // Scheduled storage tracking (flag-based)
std::map<String, std::vector<float>> storedataBuffer;   // Rolling buffer for storage values
std::map<String, unsigned long> storelastBufferUpdate;  // Last time storedataBuffer was updated
std::map<String, JsonObject> dynamicConfigs;       // Parsed Modbus slave configs from MQTT
std::map<String, unsigned long> lastPublishTimes;  // Tracks last publish time to avoid duplicates

// ══════════════════════════════════════════════════════
//  MISC STATE
// ══════════════════════════════════════════════════════
bool shouldSaveConfig = false;                     // Flag: WiFiManager config changed, needs save
int count;                                         // Number of READ config substrings after split
int count1;                                        // Number of WRITE config substrings after split
char specialCharacter = '@';                       // Delimiter used to split config strings
String substringsREAD[50];                         // Array holding split READ config entries (max 50)
String substringsWRITE[50];                        // Array holding split WRITE config entries (max 50)

// ══════════════════════════════════════════════════════
//  GSM RETRY & RECONNECT
// ══════════════════════════════════════════════════════
int gsmRetryCount = 0;                             // Current GSM connection retry attempt count
int retryDelay = GSM_INITIAL_RETRY_MS;             // Current backoff delay (increases on failure)
unsigned long connectionStartTime = 0;             // Timestamp when GSM connection attempt started
const unsigned long connectionTimeout = 60000;     // GSM connection timeout (60 seconds)
unsigned long previousReconnectAttempt = 0;        // Last MQTT reconnect attempt timestamp
const long reconnectInterval = 5000;               // Delay between MQTT reconnect attempts (ms)
bool gsmMQTTConnected = false;                     // True when AT+CMQTT broker connection is active


long lastMsg = 0;
char msg[50];
int value = 0;
unsigned long previousMillisss = 0;
long intervalss = 100;
int modbusIndex = 0;
String splitmqttaddwrite[50];
String splitmqttaddread[50];
RTC_DATA_ATTR bool skipConfigResetAfterClear = false;
bool ignoreConfigResetUntilRelease = false;



// ══════════════════════════════════════════════════════
//  FUNCTION PROTOTYPES
// ══════════════════════════════════════════════════════

// -- Network & GSM --
bool setupGSM();
void setupNetwork();
void initW5500();                                  // NEW: W5500 Ethernet reset + SPI init
String gsmSendAT(String cmd, uint32_t timeoutMs = 5000, String expected1 = "OK", String expected2 = "ERROR", String desc = ""); // raw AT helper

// -- MQTT --
void connectMQTTGSM();
void reconnect();
void reconnectMQTTGSM();
void handleWiFiMQTT();
void handleEthernetMQTT();
void handleGSMMQTT();
void sendMQTTViaGSM(const char* topic, const char* payload);
void subscribeMQTTGSM(const char* topic, int qos);
void publishMessage(const char* mqttTopic, const char* msg, int qos);
void callback(char* topic, byte* payload, unsigned int length);
void checkReconnect();

// -- GSM Retry --
void increaseBackoffDelay();
void resetBackoffDelay();

// -- WiFiManager & Config --
void saveConfigCallback();
void saveConfigToFS();
void printSavedConfig();
void printStoredConfig();

// -- Config Processing --
void getConfigAndStoreIt(String topic, String message);
void storeConfig(const JsonObject& config, String filename);
bool loadConfig(String filename, JsonDocument& config);
void printDynamicConfigs();
void listFilesAndDirectories(const char* dirname);
void listStoredFiles();
void listPaths(const char* path = "/");

// -- Modbus --
void addSlaveIPToList(const IPAddress& slaveIP);
bool isSlaveIPConnected(const IPAddress& slaveIP);
bool cbWrite(Modbus::ResultCode event, uint16_t transactionId, void* data);
void readprocessModbusRequest(uint8_t slaveId, uint16_t functionCode, uint16_t address, uint16_t count, const String& dataType, const char* mqttTopic);
void readprocessModbusRequestTCPWiFi(const IPAddress& slaveIP, uint16_t functionCode, uint16_t address, uint16_t count, const String& dataType, const char* mqttTopic);
void readprocessModbusRequestTCPEthernet(const IPAddress& slaveIP, uint16_t functionCode, uint16_t address, uint16_t count, const String& dataType, const char* mqttTopic);
void publishGroupedDeviceData();
void storeGroupedValue(uint8_t deviceId, uint16_t address, float value);
uint16_t modbusRTUCRC(uint8_t* buf, uint8_t len);
bool readModbusRTUDirect(uint8_t slaveId, uint8_t fc, uint16_t addr, uint16_t count, uint16_t* regs, bool* bools);
bool writeModbusRTUDirect(uint8_t slaveId, uint8_t fc, uint16_t addr, uint16_t* regs, uint8_t regCount, bool boolVal);
void preTransmission();
void postTransmission();

// -- Data Conversion --
void split32BitTo16Bit(uint32_t value, uint16_t* output);
void floatTo16Bit(float value, uint16_t* output);
uint32_t combineToUint32(uint16_t highOrder, uint16_t lowOrder);
int32_t combineToInt32(uint16_t highOrder, uint16_t lowOrder);
float combineToFloat(uint16_t highOrder, uint16_t lowOrder);
int16_t uint16ToInt16(uint16_t value);
int32_t uint32ToInt32(uint32_t value);
void floatToRegisters(float value, uint16_t registers[2]);
void splitAsciiRegister(uint16_t reg, char& ascii1, char& ascii2);
String extractAsciiFromRegisters(const uint16_t* registers, int numRegisters);

// -- Utility --
String base36Encode(uint32_t value);
String getStaticDeviceID();
void parseMACAddress(const char* macStr, byte* macBytes);
int splitString(String input, char delimiter, String substrings[]);
int timeToSeconds(const String& timeStr);

// -- Notification & Storage --
void processNotification(const String& topic, const String& value, const String& dataType);
void processStorage(const String& topic, const String& value, const String& dataType);

// -- Button --
int checkButton();
bool configResetPressed();

// -- Loopback Test --
void testLoopbackMQTT();

// -- TFT Display --
void initDisplay();
void updateDisplayState();
void updateDisplaySensorValue(uint8_t slaveId, uint16_t address, float value);
void readDisplaySensors();
void drawRGB565Image(int x, int y, int w, int h, int scaled_w, int scaled_h, const uint16_t *img);
void drawBottle(int x, int y);
void drawConveyor();
void displayLogoPage();
void displaySensorPage();


// ══════════════════════════════════════════════════════
//  TFT DISPLAY FUNCTIONS
// ══════════════════════════════════════════════════════

void updateDisplaySensorValue(uint8_t slaveId, uint16_t address, float value) {
  (void)slaveId;

  if (address == 3037) {
    displaySensorData.frequency = value;
    displaySensorData.dataValid = true;
    displayPageNeedsRedraw = (currentDisplayState == SENSOR_PAGE);
  } else if (address == 3027) {
    displaySensorData.vlnAvg = value;
    displaySensorData.dataValid = true;
    displayPageNeedsRedraw = (currentDisplayState == SENSOR_PAGE);
  } else if (address == 3023) {
    displaySensorData.vllAvg = value;
    displaySensorData.dataValid = true;
    displayPageNeedsRedraw = (currentDisplayState == SENSOR_PAGE);
  }
}

void readDisplaySensors() {
  if (requestInProgress || strcmp(protocolType, "rs485") != 0) return;

  const uint8_t slaveId = 52;
  uint16_t regs[2] = {0, 0};
  bool bools[2] = {false, false};

  requestInProgress = true;

  // Default to zero for every fresh display read cycle.
  // Successful Modbus reads below will overwrite these values.
  displaySensorData.frequency = 0.0f;
  displaySensorData.vlnAvg = 0.0f;
  displaySensorData.vllAvg = 0.0f;
  displaySensorData.dataValid = false;

  if (readModbusRTUDirect(slaveId, 3, 3037, 2, regs, bools)) {
    updateDisplaySensorValue(slaveId, 3037, combineToFloat(regs[0], regs[1]));
  }
  delay(50);

  regs[0] = 0;
  regs[1] = 0;
  if (readModbusRTUDirect(slaveId, 3, 3027, 2, regs, bools)) {
    updateDisplaySensorValue(slaveId, 3027, combineToFloat(regs[0], regs[1]));
  }
  delay(50);

  regs[0] = 0;
  regs[1] = 0;
  if (readModbusRTUDirect(slaveId, 3, 3023, 2, regs, bools)) {
    updateDisplaySensorValue(slaveId, 3023, combineToFloat(regs[0], regs[1]));
  }

  displayPageNeedsRedraw = (currentDisplayState == SENSOR_PAGE);
  requestInProgress = false;
}

void drawRGB565Image(int x, int y, int w, int h, int scaled_w, int scaled_h, const uint16_t *img) {
  if (!displayReady) return;

  float scale_x = (float)w / scaled_w;
  float scale_y = (float)h / scaled_h;

  for (int j = 0; j < scaled_h; j++) {
    for (int i = 0; i < scaled_w; i++) {
      int src_i = (int)(i * scale_x);
      int src_j = (int)(j * scale_y);

      if (src_i >= w) src_i = w - 1;
      if (src_j >= h) src_j = h - 1;

      uint16_t color = pgm_read_word(&img[src_j * w + src_i]);
      uint8_t r = ((color >> 11) & 0x1F) << 3;
      uint8_t g = ((color >> 5) & 0x3F) << 2;
      uint8_t b = (color & 0x1F) << 3;

      ucg.setColor(r, g, b);
      ucg.drawPixel(x + i, y + j);
    }
  }
}

void drawBottle(int x, int y) {
  if (!displayReady) return;

  ucg.setColor(30, 120, 255);
  ucg.drawFrame(x, y + 5, 12, 22);

  ucg.setColor(120, 180, 255);
  ucg.drawBox(x + 2, y + 7, 8, 16);

  ucg.setColor(180, 220, 255);
  ucg.drawBox(x + 4, y + 2, 4, 6);

  ucg.setColor(220, 220, 220);
  ucg.drawBox(x + 3, y, 6, 2);

  ucg.setColor(255, 255, 255);
  ucg.drawVLine(x + 3, y + 8, 12);
}

void drawConveyor() {
  if (!displayReady) return;

  ucg.setColor(245, 245, 245);
  ucg.drawBox(0, 188, 320, 52);

  ucg.setColor(50, 50, 50);
  ucg.drawBox(0, 214, 320, 16);

  ucg.setColor(120, 120, 120);
  ucg.drawLine(0, 214, 320, 214);

  for (int i = 0; i < 320; i += 22) {
    ucg.setColor(80, 80, 80);
    ucg.drawDisc(i + 10, 222, 3, UCG_DRAW_ALL);
  }

  for (int i = 0; i < 10; i++) {
    int x = (i * 34) - bottleAnimFrame;
    if (x < -20) x += 360;
    drawBottle(x, 188);
  }

  ucg.setColor(0, 120, 255);
  ucg.drawLine(286, 198, 306, 198);
  ucg.drawLine(306, 198, 300, 192);
  ucg.drawLine(306, 198, 300, 204);
}

void displayLogoPage() {
  if (!displayReady) return;

  ucg.setColor(255, 255, 255);
  ucg.drawBox(0, 0, 320, 240);

  int scaled_width = (IMAGE_WIDTH * 60) / 100;
  int scaled_height = (IMAGE_HEIGHT * 60) / 100;
  int img_x = (320 - scaled_width) / 2;
  int img_y = 15;

  drawRGB565Image(img_x, img_y, IMAGE_WIDTH, IMAGE_HEIGHT, scaled_width, scaled_height, logo565);

  ucg.setColor(0, 0, 0);
  ucg.setFont(ucg_font_ncenR14_hr);
  ucg.setPrintPos(135, img_y + scaled_height + 28);
  ucg.print("BIT");

  ucg.setFont(ucg_font_ncenR18_hr);
  ucg.setPrintPos(18, img_y + scaled_height + 63);
  ucg.print("Industrial IoT Gateway");
}

void displaySensorPage() {
  if (!displayReady) return;

  ucg.setColor(245, 245, 245);
  ucg.drawBox(0, 0, 320, 240);

  ucg.setColor(15, 40, 90);
  ucg.drawBox(0, 0, 320, 32);
  ucg.setColor(0, 140, 255);
  ucg.drawLine(0, 31, 320, 31);

  ucg.setColor(255, 255, 255);
  ucg.setFont(ucg_font_ncenR14_hr);
  ucg.setPrintPos(15, 22);
  ucg.print("STATION DATA");

  ucg.setColor(0, 220, 0);
  ucg.drawCircle(255, 15, 2, UCG_DRAW_ALL);
  ucg.drawCircle(255, 15, 6, UCG_DRAW_UPPER_RIGHT);
  ucg.drawCircle(255, 15, 6, UCG_DRAW_UPPER_LEFT);
  ucg.drawCircle(255, 15, 10, UCG_DRAW_UPPER_RIGHT);
  ucg.drawCircle(255, 15, 10, UCG_DRAW_UPPER_LEFT);

  ucg.setColor(120, 120, 120);
  ucg.drawBox(285, 18, 4, 6);
  ucg.drawBox(292, 14, 4, 10);
  ucg.drawBox(299, 10, 4, 14);
  ucg.drawBox(306, 6, 4, 18);

  ucg.setColor(0, 180, 70);
  ucg.drawRFrame(8, 42, 118, 95, 4);
  ucg.setColor(0, 255, 0);
  ucg.setFont(ucg_font_6x13_tf);
  ucg.setPrintPos(18, 58);
  ucg.print("BOTTLE FEEDER");
  ucg.setPrintPos(40, 75);
  ucg.print("SYSTEM");

  ucg.setColor(0, 180, 0);
  ucg.setFont(ucg_font_logisoso20_tf);
  ucg.setPrintPos(28, 112);
  ucg.print("LIVE");

  ucg.setColor(0, 255, 80);
  ucg.drawCircle(68, 118, 14, UCG_DRAW_ALL);
  ucg.setColor(0, 180, 60);
  ucg.drawDisc(68, 118, 8, UCG_DRAW_ALL);

  ucg.setColor(0, 110, 255);
  ucg.drawRFrame(135, 42, 85, 70, 4);
  ucg.setColor(20, 20, 20);
  ucg.setFont(ucg_font_6x13_tf);
  ucg.setPrintPos(145, 58);
  ucg.print("FREQUENCY");

  char freq[10];
  dtostrf(displaySensorData.frequency, 4, 1, freq);
  ucg.setFont(ucg_font_logisoso20_tf);
  ucg.setPrintPos(148, 95);
  ucg.print(freq);
  ucg.setFont(ucg_font_6x13_tf);
  ucg.setPrintPos(185, 105);
  ucg.print("Hz");

  ucg.setColor(0, 110, 255);
  ucg.drawRFrame(228, 42, 85, 70, 4);
  ucg.setColor(20, 20, 20);
  ucg.setFont(ucg_font_6x13_tf);
  ucg.setPrintPos(248, 58);
  ucg.print("VLN AVG");

  char vln[12];
  dtostrf(displaySensorData.vlnAvg, 4, 1, vln);
  ucg.setFont(ucg_font_logisoso20_tf);
  ucg.setPrintPos(236, 95);
  ucg.print(vln);

  ucg.setColor(0, 110, 255);
  ucg.drawRFrame(135, 122, 178, 58, 4);
  ucg.setColor(20, 20, 20);
  ucg.setFont(ucg_font_6x13_tf);
  ucg.setPrintPos(145, 140);
  ucg.print("VLL AVG");

  char vll[12];
  dtostrf(displaySensorData.vllAvg, 4, 1, vll);
  ucg.setFont(ucg_font_logisoso24_tf);
  ucg.setPrintPos(188, 165);
  ucg.print(vll);
  ucg.setColor(0, 120, 255);
  ucg.setFont(ucg_font_6x13_tf);
  ucg.setPrintPos(276, 165);
  ucg.print("V");

  drawConveyor();
}

void updateDisplayState() {
  if (!displayReady) return;

  unsigned long currentTime = millis();
  unsigned long currentDuration = (currentDisplayState == LOGO_PAGE) ? LOGO_DURATION : SENSOR_DURATION;

  if (currentTime - displayStateChangeTime >= currentDuration) {
    currentDisplayState = (currentDisplayState == LOGO_PAGE) ? SENSOR_PAGE : LOGO_PAGE;
    if (currentDisplayState == SENSOR_PAGE) {
      readDisplaySensors();
    }
    displayStateChangeTime = currentTime;
    displayPageNeedsRedraw = true;
  }

  if (displayPageNeedsRedraw) {
    if (currentDisplayState == LOGO_PAGE) {
      displayLogoPage();
    } else {
      displaySensorPage();
      bottleAnimFrame = 0;
    }
    displayPageNeedsRedraw = false;
  } else if (currentDisplayState == SENSOR_PAGE) {
    static unsigned long lastAnimTime = 0;
    static unsigned long lastDisplayReadTime = 0;

    if (millis() - lastDisplayReadTime >= 1000) {
      lastDisplayReadTime = millis();
      readDisplaySensors();
    }

    if (millis() - lastAnimTime > 150) {
      lastAnimTime = millis();
      bottleAnimFrame++;
      if (bottleAnimFrame > 340) bottleAnimFrame = 0;
      drawConveyor();
    }
  }
}

void initDisplay() {
  pinMode(MCP_CS, OUTPUT);
  digitalWrite(MCP_CS, HIGH);

  pinMode(W5500_CS, OUTPUT);
  digitalWrite(W5500_CS, HIGH);

  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);

  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI);

  if (!mcp.begin_SPI(MCP_CS)) {
    Serial.println("[DISPLAY] MCP23S17 Failed");
    displayReady = false;
    return;
  }

  mcp.pinMode(TFT_LED_PIN, OUTPUT);
  mcp.pinMode(TFT_RESET_PIN, OUTPUT);
  mcp.digitalWrite(TFT_LED_PIN, HIGH);

  mcp.digitalWrite(TFT_RESET_PIN, LOW);
  delay(50);
  mcp.digitalWrite(TFT_RESET_PIN, HIGH);
  delay(200);

  ucg.begin(UCG_FONT_MODE_TRANSPARENT);
  ucg.setRotate90();
  ucg.clearScreen();

  displayReady = true;
  currentDisplayState = LOGO_PAGE;
  displayStateChangeTime = millis();
  displayPageNeedsRedraw = true;

  Serial.println("[DISPLAY] TFT ready");
  displayLogoPage();
  displayPageNeedsRedraw = false;
}

// ══════════════════════════════════════════════════════
//  UTILITY FUNCTIONS
// ══════════════════════════════════════════════════════

// Converts a 32-bit value to a 6-character Base36 encoded string
String base36Encode(uint32_t value) {
  const char base36Chars[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
  char encoded[7];
  for (int i = 5; i >= 0; i--) {
    encoded[i] = base36Chars[value % 36];
    value /= 36;
  }
  encoded[6] = '\0';
  return String(encoded);
}

// Generate a static 6-char device ID from ESP32 MAC address
String getStaticDeviceID() {
  uint64_t mac = ESP.getEfuseMac();
  uint32_t shortId = (uint32_t)(mac & 0xFFFFFF);
  return base36Encode(shortId);
}

// Convert MAC address string "XX:XX:XX:XX:XX:XX" to byte array
void parseMACAddress(const char* macStr, byte* macBytes) {
  sscanf(macStr, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
         &macBytes[0], &macBytes[1], &macBytes[2],
         &macBytes[3], &macBytes[4], &macBytes[5]);
}

// Split string by delimiter into array, returns number of parts
int splitString(String input, char delimiter, String substrings[]) {
  int count = 0;
  int startIndex = 0;
  for (int i = 0; i <= input.length(); i++) {
    if (i == input.length() || input.charAt(i) == delimiter) {
      substrings[count++] = input.substring(startIndex, i);
      startIndex = i + 1;
    }
  }
  return count;
}

// Convert "HH:MM" or "HH:MM:SS" string to total seconds since midnight
int timeToSeconds(const String& timeStr) {
  int hours = 0, minutes = 0, seconds = 0;
  if (timeStr.length() < 5) {
    Serial.printf("[ERROR] Invalid time format: %s\n", timeStr.c_str());
    return -1;
  }
  hours = timeStr.substring(0, 2).toInt();
  minutes = timeStr.substring(3, 5).toInt();
  if (timeStr.length() >= 8) {
    seconds = timeStr.substring(6, 8).toInt();
  }
  if (hours < 0 || hours > 23 || minutes < 0 || minutes > 59 || seconds < 0 || seconds > 59) {
    Serial.printf("[ERROR] Invalid time values in: %s\n", timeStr.c_str());
    return -1;
  }
  return (hours * 3600) + (minutes * 60) + seconds;
}

// Get current time as "HH:MM:SS" string from RTC
String getCurrentTimeString() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("[ERROR] Failed to obtain time.");
    return "00:00:00";
  }
  char timeStr[9];
  if (strftime(timeStr, sizeof(timeStr), "%H:%M:%S", &timeinfo) == 0) {
    Serial.println("[ERROR] Failed to format the time.");
    return "00:00:00";
  }
  return String(timeStr);
}

// Get current date as "YYYY-MM-DD" string from RTC
String getCurrentDate() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("[ERROR] Failed to obtain time");
    return "";
  }
  char dateStr[11];
  snprintf(dateStr, sizeof(dateStr), "%04d-%02d-%02d",
           timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday);
  return String(dateStr);
}

// Reset scheduled send tracking at midnight
void resetLastSentTimes() {
  lastSentTimes.clear();
  Serial.println("[INFO] Reset lastSentTimes for the new day.");
}

// Check if day changed and trigger midnight reset
void checkForMidnightReset() {
  struct tm timeinfo;
  if (strcmp(mqttMethod, "gsm") == 0) return;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("[ERROR] Failed to obtain time for midnight check.");
    return;
  }
  int currentDay = timeinfo.tm_mday;
  if (currentDay != lastResetDay) {
    resetLastSentTimes();
    lastResetDay = currentDay;
  }
}

// Set ESP32 internal RTC from individual time components
void setTimeOnESP32(int year, int month, int day, int hour, int minute, int second) {
  struct tm timeinfo = {};
  timeinfo.tm_year = year - 1900;
  timeinfo.tm_mon = month - 1;
  timeinfo.tm_mday = day;
  timeinfo.tm_hour = hour;
  timeinfo.tm_min = minute;
  timeinfo.tm_sec = second;
  time_t t = mktime(&timeinfo);
  struct timeval tv = { .tv_sec = t, .tv_usec = 0 };
  settimeofday(&tv, nullptr);
  SerialMon.println("[INFO] RTC updated with network time.");
}

// Fetch time from GSM network and update RTC
void fetchNetworkTime() {
  SerialMon.println("[INFO] Fetching network time...");
  int ntp_year = 0, ntp_month = 0, ntp_day = 0;
  int ntp_hour = 0, ntp_min = 0, ntp_sec = 0;
  float ntp_timezone = 0;
  String res = gsmSendAT("AT+CCLK?", 2000);
  if (res.indexOf("+CCLK:") != -1) {
    SerialMon.println("[INFO] Network Time: " + res);
    // Parsing can be added here if RTC update is critical, for now we log it.
  } else {
    SerialMon.println("[ERROR] Failed to fetch network time!");
  }
}

// Check if network is connected (WiFi/Ethernet/GSM) with retries
bool isNetworkConnected() {
  const int maxRetries = 5;
  const int initialDelay = 1000;
  int retryCount = 0;
  int currentDelay = initialDelay;
  while (retryCount < maxRetries) {
    if (strcmp(mqttMethod, "wifi") == 0) {
      if (WiFi.status() == WL_CONNECTED) return true;
    } else if (strcmp(mqttMethod, "ethernet") == 0) {
      if (Ethernet.linkStatus() == LinkON) return true;
    } else if (strcmp(mqttMethod, "gsm") == 0) {
      String res = gsmSendAT("AT+CREG?", 1000);
      if (res.indexOf(",1") != -1 || res.indexOf(",5") != -1) return true;
      reconnectMQTTGSM();
    } else {
      Serial.println("[ERROR] Unsupported network method.");
      return false;
    }
    retryCount++;
    Serial.printf("[RETRY] Network attempt %d/%d\n", retryCount, maxRetries);
    delay(currentDelay);
    currentDelay = min(currentDelay * 2, 8000);
  }
  Serial.println("[FAILURE] Network connection failed after retries.");
  return false;
}

// Retry NTP sync across multiple servers with backoff
bool retryNTP() {
  // ── Ethernet mode: configTime() uses ESP32 IDF SNTP/lwIP which has NO interface
  // when WiFi is not started. Use EthernetUDP to query NTP directly over the W5500
  // hardware stack, then set the ESP32 RTC manually via settimeofday().
  if (strcmp(mqttMethod, "ethernet") == 0) {
    // Well-known NTP server IPs — avoids DNS dependency (DNS may also use lwIP).
    // time.google.com = 216.239.35.0 / 216.239.35.4
    // time.cloudflare.com = 162.159.200.123
    // time-a-g.nist.gov = 129.6.15.28
    const IPAddress ntpIPs[] = {
      IPAddress(216, 239, 35,  0),   // time.google.com
      IPAddress(216, 239, 35,  4),   // time.google.com alt
      IPAddress(162, 159, 200, 123), // time.cloudflare.com
      IPAddress(129,   6,  15,  28), // time-a-g.nist.gov
    };
    const int ntpIPCount = sizeof(ntpIPs) / sizeof(ntpIPs[0]);

    for (int attempt = 1; attempt <= ntpMaxRetries; attempt++) {
      for (int s = 0; s < ntpIPCount; s++) {
        Serial.printf("[DEBUG] NTP attempt %d/%d via Ethernet UDP → %s\n",
                      attempt, ntpMaxRetries, ntpIPs[s].toString().c_str());

        EthernetUDP udp;
        udp.begin(2390);

        // Build 48-byte NTP request: LI=0, VN=4, Mode=3 (client)
        uint8_t pkt[48] = {0};
        pkt[0] = 0b00100011;  // LI=0, VN=4, Mode=3

        udp.beginPacket(ntpIPs[s], 123);
        udp.write(pkt, 48);
        udp.endPacket();

        // Wait up to 3 seconds for a 48-byte response
        unsigned long t0 = millis();
        bool got = false;
        while (millis() - t0 < 3000) {
          if (udp.parsePacket() >= 48) {
            uint8_t resp[48];
            udp.read(resp, 48);
            // Transmit timestamp is at bytes 40-43 (NTP seconds since 1900-01-01)
            uint32_t ntpSecs = ((uint32_t)resp[40] << 24) | ((uint32_t)resp[41] << 16)
                             | ((uint32_t)resp[42] <<  8) |  (uint32_t)resp[43];
            const uint32_t NTP_TO_UNIX = 2208988800UL;  // seconds between 1900 and 1970
            time_t unixTime = (time_t)(ntpSecs - NTP_TO_UNIX) + GMT_OFFSET_SEC;
            struct timeval tv = { .tv_sec = unixTime, .tv_usec = 0 };
            settimeofday(&tv, NULL);
            got = true;
            break;
          }
          delay(10);
        }
        udp.stop();

        if (got) {
          struct tm timeinfo;
          if (getLocalTime(&timeinfo)) {
            char timeStr[64];
            strftime(timeStr, sizeof(timeStr), "%c", &timeinfo);
            Serial.printf("[INFO] NTP sync OK (Ethernet UDP): %s\n", timeStr);
            return true;
          }
        }
        Serial.printf("[DEBUG] NTP timeout for %s\n", ntpIPs[s].toString().c_str());
      }
      delay(ntpRetryDelay);
    }
    Serial.println("[ERROR] NTP sync failed after retries.");
    return false;
  }

  // ── WiFi mode: use ESP32 IDF SNTP via configTime() (routes through lwIP/WiFi).
  // NOTE: Do NOT call WiFiUDP::begin() — crashes when WiFi stack is not initialised.
  const char* ntpServers[] = { "pool.ntp.org", "time.google.com", "time.nist.gov", "0.pool.ntp.org", "1.pool.ntp.org" };
  const int ntpServerCount = sizeof(ntpServers) / sizeof(ntpServers[0]);
  for (int attempt = 1; attempt <= ntpMaxRetries; attempt++) {
    for (int s = 0; s < ntpServerCount; s++) {
      Serial.printf("[DEBUG] NTP attempt %d/%d server: %s\n", attempt, ntpMaxRetries, ntpServers[s]);
      configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, ntpServers[s]);
      delay(3000);
      struct tm timeinfo;
      if (getLocalTime(&timeinfo)) {
        char timeStr[64];
        strftime(timeStr, sizeof(timeStr), "%c", &timeinfo);
        Serial.printf("[INFO] NTP sync OK: %s\n", timeStr);
        return true;
      }
    }
    delay(ntpRetryDelay);
  }
  Serial.println("[ERROR] NTP sync failed after retries.");
  return false;
}

// Setup time: NTP for WiFi/Ethernet, GSM network time for GSM
void setupNTP() {
  Serial.println("[INFO] Configuring time...");
  if (!isNetworkConnected()) {
    Serial.println("[ERROR] No network. Cannot sync time.");
    return;
  }
  if (strcmp(mqttMethod, "wifi") == 0 || strcmp(mqttMethod, "ethernet") == 0) {
    if (!retryNTP()) {
      Serial.println("[ERROR] NTP sync failed.");
      return;
    }
    Serial.println("[INFO] Time synced via NTP.");
  } else if (strcmp(mqttMethod, "gsm") == 0) {
    fetchNetworkTime();
  }
}

// Print current RTC time to serial
void printCurrentTime() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    char timeString[64];
    strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo);
    SerialMon.printf("[INFO] Current RTC Time: %s\n", timeString);
  } else {
    SerialMon.println("[ERROR] Failed to retrieve RTC time!");
  }
}

// ══════════════════════════════════════════════════════
//  MODBUS DATA CONVERSION
// ══════════════════════════════════════════════════════

// Combine two 16-bit registers into unsigned 32-bit
uint32_t combineToUint32(uint16_t highOrder, uint16_t lowOrder) {
  return ((uint32_t)highOrder << 16) | lowOrder;
}

// Combine two 16-bit registers into signed 32-bit
int32_t combineToInt32(uint16_t highOrder, uint16_t lowOrder) {
  return ((int32_t)(int16_t)highOrder << 16) | lowOrder;
}

// Combine two 16-bit registers into IEEE 754 float
float combineToFloat(uint16_t highOrder, uint16_t lowOrder) {
  uint32_t combined = ((uint32_t)highOrder << 16) | lowOrder;
  float result;
  memcpy(&result, &combined, sizeof(result));
  return result;
}

// Reinterpret unsigned 16-bit as signed 16-bit
int16_t uint16ToInt16(uint16_t value) {
  return static_cast<int16_t>(value);
}

// Reinterpret unsigned 32-bit as signed 32-bit
int32_t uint32ToInt32(uint32_t value) {
  return static_cast<int32_t>(value);
}

// Convert float to two 16-bit Modbus registers [high, low]
void floatToRegisters(float value, uint16_t registers[2]) {
  uint32_t asInt = *(uint32_t*)&value;
  registers[0] = (uint16_t)(asInt >> 16);
  registers[1] = (uint16_t)(asInt & 0xFFFF);
}

// Split 32-bit unsigned into two 16-bit parts [high, low]
void split32BitTo16Bit(uint32_t value, uint16_t* output) {
  output[0] = (value >> 16) & 0xFFFF;
  output[1] = value & 0xFFFF;
}

// Convert float to two 16-bit parts via IEEE 754 union
void floatTo16Bit(float value, uint16_t* output) {
  union { float f; uint32_t i; } converter;
  converter.f = value;
  split32BitTo16Bit(converter.i, output);
}

// Extract MSB and LSB ASCII characters from a 16-bit register
void splitAsciiRegister(uint16_t reg, char &ascii1, char &ascii2) {
  ascii1 = (char)((reg >> 8) & 0xFF);
  ascii2 = (char)(reg & 0xFF);
}

// Extract ASCII string from array of 16-bit registers (2 chars per register)
String extractAsciiFromRegisters(const uint16_t* registers, int numRegisters) {
  String result = "";
  for (int i = 0; i < numRegisters; i++) {
    char ascii1, ascii2;
    splitAsciiRegister(registers[i], ascii1, ascii2);
    result += ascii1;
    result += ascii2;
  }
  return result;
}


// ══════════════════════════════════════════════════════
//  MODBUS CALLBACKS
// ══════════════════════════════════════════════════════

// Add a Modbus TCP slave IP to the connected list
void addSlaveIPToList(const IPAddress& slaveIP) {
  connectedSlaveIPs.push_back(slaveIP);
}

// Check if a slave IP is already in the connected list
bool isSlaveIPConnected(const IPAddress& slaveIP) {
  for (const auto& ip : connectedSlaveIPs) {
    if (ip == slaveIP) return true;
  }
  return false;
}

// Modbus transaction result callback — sets callbackResult flag
bool cbWrite(Modbus::ResultCode event, uint16_t transactionId, void* data) {
  Serial.printf_P("Request result: 0x%02X, Mem: %d\n", event, ESP.getFreeHeap());
  switch (event) {
    case Modbus::EX_SUCCESS:
      Serial.println(" - Success");
      callbackResult = true;
      break;
    case Modbus::EX_ILLEGAL_FUNCTION:
      Serial.println(" - Illegal function: Function code not supported");
      callbackResult = false;
      break;
    case Modbus::EX_ILLEGAL_ADDRESS:
      Serial.println(" - Illegal address: Output address does not exist");
      callbackResult = false;
      break;
    case Modbus::EX_ILLEGAL_VALUE:
      Serial.println(" - Illegal value: Output value not in range");
      callbackResult = false;
      break;
    case Modbus::EX_SLAVE_FAILURE:
      Serial.println(" - Slave failure: Device failed to process request");
      callbackResult = false;
      break;
    case Modbus::EX_ACKNOWLEDGE:
      Serial.println(" - Acknowledge (Not used)");
      callbackResult = false;
      break;
    case Modbus::EX_SLAVE_DEVICE_BUSY:
      Serial.println(" - Slave device busy (Not used)");
      callbackResult = false;
      break;
    case Modbus::EX_MEMORY_PARITY_ERROR:
      Serial.println(" - Memory parity error (Not used)");
      callbackResult = false;
      break;
    case Modbus::EX_PATH_UNAVAILABLE:
      Serial.println(" - Path unavailable (Not used)");
      callbackResult = false;
      break;
    case Modbus::EX_DEVICE_FAILED_TO_RESPOND:
      Serial.println(" - Device failed to respond (Not used)");
      callbackResult = false;
      break;
    case Modbus::EX_GENERAL_FAILURE:
      Serial.println(" - General failure: Unexpected master error");
      callbackResult = false;
      break;
    case Modbus::EX_DATA_MISMACH:
      Serial.println(" - Data mismatch: Input data size mismatch");
      callbackResult = false;
      break;
    case Modbus::EX_UNEXPECTED_RESPONSE:
      Serial.println(" - Unexpected response: Result doesn't match transaction");
      callbackResult = false;
      break;
    case Modbus::EX_TIMEOUT:
      Serial.println(" - Timeout: Operation not finished within reasonable time");
      callbackResult = false;
      break;
    case Modbus::EX_CONNECTION_LOST:
      Serial.println(" - Connection lost: Connection with device lost");
      callbackResult = false;
      break;
    case Modbus::EX_CANCEL:
      Serial.println(" - Cancel: Transaction/request canceled");
      callbackResult = false;
      break;
    case Modbus::EX_PASSTHROUGH:
      Serial.println(" - Passthrough: Raw callback, normal processing on callback exit");
      callbackResult = false;
      break;
    case Modbus::EX_FORCE_PROCESS:
      Serial.println(" - Force process: Raw callback, force processing on callback exit");
      callbackResult = false;
      break;
    default:
      Serial.println(" - Unknown error");
      callbackResult = false;
      break;
  }
  return true;
}

// ══════════════════════════════════════════════════════
//  W5500 ETHERNET INIT (NEW for new PCB)
// ══════════════════════════════════════════════════════

// Hardware reset W5500, init SPI bus, and configure Ethernet CS
void initW5500() {
  Serial.println("[W5500] Initialising...");
  pinMode(W5500_RST, OUTPUT);
  digitalWrite(W5500_RST, LOW);       // Hold reset LOW
  delay(1000);
  digitalWrite(W5500_RST, HIGH);      // Release reset
  delay(2000);                         // Wait for W5500 to boot
  pinMode(W5500_CS, OUTPUT);
  digitalWrite(W5500_CS, HIGH);       // Deselect W5500
  delay(100);
  SPI.begin(W5500_SCLK, W5500_MISO, W5500_MOSI, W5500_CS);
  SPI.setFrequency(14000000);         // 14 MHz SPI clock
  SPI.setDataMode(SPI_MODE0);
  delay(500);
  Ethernet.init(W5500_CS);            // Tell Ethernet library which CS pin
  delay(500);
  Serial.println("[W5500] Init complete");
}


// ══════════════════════════════════════════════════════
//  WIFIMANAGER & CONFIG (LittleFS)
// ══════════════════════════════════════════════════════

// Called by WiFiManager when config portal saves new values
void saveConfigCallback() {
  Serial.println("Should save config");
  shouldSaveConfig = true;
  Serial.println("Values to be saved:");
  Serial.print("MQTT Server: "); Serial.println(mqttserver);
  Serial.print("MQTT Port: "); Serial.println(mqttport);
  Serial.print("MQTT Username: "); Serial.println(mqttusername);
  Serial.print("MQTT Password: "); Serial.println(mqttpassword);
  Serial.print("MQTT Method: "); Serial.println(mqttMethod);
  Serial.print("ethMacAddress: "); Serial.println(ethMacAddr);
  Serial.print("staticIP: "); Serial.println(staticIP);
  Serial.print("gatewayIPs: "); Serial.println(gatewayIPs);
  Serial.print("subnetMasks: "); Serial.println(subnetMasks);
  Serial.print("ipMode: "); Serial.println(ipMode);
  Serial.print("apn: "); Serial.println(apn);
  Serial.print("gprsUser: "); Serial.println(gprsUser);
  Serial.print("gprsPass: "); Serial.println(gprsPass);
  Serial.print("Protocol Type: "); Serial.println(protocolType);
  Serial.print("intervalssParam ms: "); Serial.println(intervalssParam);
  Serial.print("Slave Baudrate: "); Serial.println(slavebaudrate);
  Serial.print("Slave Data Length: "); Serial.println(slavedatalength);
  Serial.print("Slave Parity: "); Serial.println(slaveparity);
  Serial.print("Slave Stop Bit: "); Serial.println(slavestopbit);
  Serial.print("Slave Config READ: "); Serial.println(slaveconfigREAD);
  Serial.print("Slave Config WRITE: "); Serial.println(slaveconfigWRITE);
}

// Save all config parameters to /config.json on LittleFS
void saveConfigToFS() {
  if (shouldSaveConfig) {
    Serial.println("Saving config...");
    Serial.printf("[HEAP] saveConfigToFS enter: %u bytes\n", ESP.getFreeHeap());
    if (!LittleFS.begin()) {
      Serial.println("Failed to mount LittleFS");
      return;
    }
    // DynamicJsonDocument allocates on HEAP instead of stack (avoids stack overflow)
    DynamicJsonDocument json(4096);  // 4KB is enough for these fields
    json["mqttserver"] = mqttserver;
    json["mqttport"] = mqttport;
    json["mqttusername"] = mqttusername;
    json["mqttpassword"] = mqttpassword;
    json["mqttMethod"] = mqttMethod;
    json["ethMacAddr"] = ethMacAddr;
    json["staticIP"] = staticIP;
    json["gatewayIP"] = gatewayIPs;
    json["subnetMask"] = subnetMasks;
    json["ipMode"] = ipMode;
    json["apn"] = apn;
    json["gprsUser"] = gprsUser;
    json["gprsPass"] = gprsPass;
    json["protocolType"] = protocolType;
    json["intervalss"] = intervalssParam;
    json["slavebaudrate"] = slavebaudrate;
    json["slavedatalength"] = slavedatalength;
    json["slaveparity"] = slaveparity;
    json["slavestopbit"] = slavestopbit;
    json["slaveconfigREAD"] = slaveconfigREAD;
    json["slaveconfigWRITE"] = slaveconfigWRITE;
    json["baseTopicPath"] = baseTopicPath;  // SAVE THIS TOO!
    Serial.printf("[CONFIG-SAVE] Saving baseTopicPath: '%s'\n", baseTopicPath);
    yield();
    File configFile = LittleFS.open("/config.json", "w", true);  // true = create if missing
    if (!configFile) {
      Serial.println("Failed to open config file for writing");
      return;
    }
    if (serializeJson(json, configFile) == 0) {
      Serial.println("Failed to write to file");
    }
    configFile.close();
    shouldSaveConfig = false;
    Serial.printf("[HEAP] saveConfigToFS done: %u bytes\n", ESP.getFreeHeap());
    Serial.println("Config saved successfully.");
  } else {
    Serial.println("No config changes to save.");
  }
}

// Print stored config variables to serial
void printStoredConfig() {
  Serial.println("\n--- Stored Configuration ---");
  Serial.print("MQTT Server: "); Serial.println(mqttserver);
  Serial.print("MQTT Port: "); Serial.println(mqttport);
  Serial.print("MQTT Username: "); Serial.println(mqttusername);
  Serial.print("MQTT Password: "); Serial.println(mqttpassword);
  Serial.print("mqttMethod: "); Serial.println(mqttMethod);
  Serial.print("ethMacAddr: "); Serial.println(ethMacAddr);
  Serial.print("staticIP: "); Serial.println(staticIP);
  Serial.print("gatewayIPs: "); Serial.println(gatewayIPs);
  Serial.print("subnetMasks: "); Serial.println(subnetMasks);
  Serial.print("ipMode: "); Serial.println(ipMode);
  Serial.print("apn: "); Serial.println(apn);
  Serial.print("gprsUser: "); Serial.println(gprsUser);
  Serial.print("gprsPass: "); Serial.println(gprsPass);
  Serial.print("Protocol Type: "); Serial.println(protocolType);
  Serial.print("intervalssParam ms: "); Serial.println(intervalssParam);
  Serial.print("Slave Baudrate: "); Serial.println(slavebaudrate);
  Serial.print("Slave Data Length: "); Serial.println(slavedatalength);
  Serial.print("Slave Parity: "); Serial.println(slaveparity);
  Serial.print("Slave Stop Bit: "); Serial.println(slavestopbit);
  Serial.print("Slave Config READ: "); Serial.println(slaveconfigREAD);
  Serial.print("Slave Config WRITE: "); Serial.println(slaveconfigWRITE);
  Serial.print("Base MQTT Topic (Grouped): "); Serial.println(baseTopicPath);
  Serial.println("----------------------------");
}

// Read and print /config.json raw contents from LittleFS
void printSavedConfig() {
  if (!LittleFS.begin()) {
    Serial.println("Failed to mount LittleFS");
    return;
  }
  if (LittleFS.exists("/config.json")) {
    Serial.println("Reading config file...");
    File configFile = LittleFS.open("/config.json", "r", false);
    if (!configFile) {
      Serial.println("Failed to open config file for reading");
      return;
    }
    Serial.println("Saved config file contents:");
    while (configFile.available()) {
      Serial.write(configFile.read());
    }
    Serial.println();
    configFile.close();
  } else {
    Serial.println("Config file does not exist.");
  }
}

// ══════════════════════════════════════════════════════
//  LITTLEFS FILE HELPERS
// ══════════════════════════════════════════════════════

// List all files in LittleFS root
void listFiles() {
  File root = LittleFS.open("/");
  if (!root || !root.isDirectory()) {
    Serial.println("[ERROR] Failed to open root directory.");
    return;
  }
  File file = root.openNextFile();
  while (file) {
    Serial.printf("[INFO] File found: %s\n", file.name());
    file = root.openNextFile();
  }
}

// Format LittleFS (erase all files)
void clearLittleFS() {
  Serial.println("[DEBUG] Starting reformatLittleFS...");
  LittleFS.end();  // unmount first — format fails if filesystem is still mounted
  if (LittleFS.format()) {
    Serial.println("[INFO] LittleFS reformatted successfully.");
  } else {
    Serial.println("[ERROR] Failed to reformat LittleFS.");
  }
  Serial.println("[DEBUG] Completed reformatLittleFS.");
}

bool configResetPressed() {
  if (digitalRead(WIFI_RESET_PIN) == HIGH) {
    ignoreConfigResetUntilRelease = false;
    return false;
  }
  return !ignoreConfigResetUntilRelease;
}

// Print ESP32 partition table to serial
void printPartitionTable() {
  Serial.println("\nPartition Table:");
  Serial.println("----------------------------------------------------");
  Serial.println("| Name       | Type | SubType | Address  | Size   |");
  Serial.println("----------------------------------------------------");
  esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, NULL);
  while (it != NULL) {
    const esp_partition_t *partition = esp_partition_get(it);
    Serial.printf("| %-10s | APP  | %-7d | 0x%06x | %-7d |\n",
                  partition->label, partition->subtype, partition->address, partition->size);
    it = esp_partition_next(it);
  }
  it = esp_partition_find(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, NULL);
  while (it != NULL) {
    const esp_partition_t *partition = esp_partition_get(it);
    Serial.printf("| %-10s | DATA | %-7d | 0x%06x | %-7d |\n",
                  partition->label, partition->subtype, partition->address, partition->size);
    it = esp_partition_next(it);
  }
  Serial.println("----------------------------------------------------");
}

// Print all file names and contents from LittleFS
void printFileNamesAndContents() {
  if (!LittleFS.begin()) {
    Serial.println("[ERROR] Failed to initialize LittleFS.");
    return;
  }
  Serial.println("[INFO] Listing all files and their contents:");
  File root = LittleFS.open("/");
  if (!root || !root.isDirectory()) {
    Serial.println("[ERROR] Failed to open root directory.");
    return;
  }
  File file = root.openNextFile();
  while (file) {
    Serial.printf("[FILE] Name: %s\n", file.name());
    Serial.print("[CONTENT] ");
    while (file.available()) {
      Serial.write(file.read());
    }
    Serial.println();
    file = root.openNextFile();
  }
  Serial.println("[INFO] Finished listing files.");
}

// Print configMap contents to serial
void printConfigMap() {
  Serial.println("[DEBUG] Printing configMap:");
  for (const auto& pair : configMap) {
    Serial.printf("Topic: %s, JSON: %s\n", pair.first.c_str(), pair.second.c_str());
  }
}

// ══════════════════════════════════════════════════════
//  TOPIC <-> FILENAME ENCODING (collision-safe)
// ══════════════════════════════════════════════════════

// Encode MQTT topic to safe LittleFS filename
String encodeTopicToFilename(const String& topic) {
  String encoded = "";
  String cleanTopic = topic;
  cleanTopic.replace("/configs", "");
  for (int i = 0; i < cleanTopic.length(); i++) {
    char c = cleanTopic[i];
    if (c == '/')      encoded += "_sl_";
    else if (c == '_') encoded += "_un_";
    else if (c == '.') encoded += "_dt_";
    else if (c == ' ') encoded += "_sp_";
    else               encoded += c;
  }
  return "/" + encoded + ".json";
}

// Decode safe filename back to MQTT topic
String decodeFilenameToTopic(const String& filename) {
  String decoded = filename;
  if (decoded.startsWith("/")) decoded = decoded.substring(1);
  if (decoded.endsWith(".json")) decoded = decoded.substring(0, decoded.length() - 5);
  decoded.replace("_sl_", "/");
  decoded.replace("_un_", "_");
  decoded.replace("_dt_", ".");
  decoded.replace("_sp_", " ");
  return decoded;
}

// ══════════════════════════════════════════════════════
//  ATOMIC FILE WRITE (safe write with backup + verify)
// ══════════════════════════════════════════════════════

// Write file atomically: write to .tmp, verify, rename to final
bool atomicWriteFile(const String& filename, const String& content) {
  String tempFilename = filename + ".tmp";
  String backupFilename = filename + ".bak";
  Serial.printf("[DEBUG] Atomic write starting for: %s\n", filename.c_str());

  // Backup existing file
  if (LittleFS.exists(filename)) {
    if (LittleFS.exists(backupFilename)) LittleFS.remove(backupFilename);
    if (!LittleFS.rename(filename, backupFilename)) {
      Serial.println("[ERROR] Failed to create backup file.");
      return false;
    }
  }

  // Write to temp file
  File tempFile = LittleFS.open(tempFilename, "w");
  if (!tempFile) {
    Serial.printf("[ERROR] Failed to open temp file: %s\n", tempFilename.c_str());
    if (LittleFS.exists(backupFilename)) LittleFS.rename(backupFilename, filename);
    return false;
  }
  size_t bytesWritten = tempFile.print(content);
  tempFile.flush();
  tempFile.close();

  // Verify write size
  if (bytesWritten != content.length()) {
    Serial.printf("[ERROR] Write size mismatch: expected %d, got %d\n", content.length(), bytesWritten);
    LittleFS.remove(tempFilename);
    if (LittleFS.exists(backupFilename)) LittleFS.rename(backupFilename, filename);
    return false;
  }

  // Verify readback
  File verifyFile = LittleFS.open(tempFilename, "r");
  if (!verifyFile) {
    LittleFS.remove(tempFilename);
    if (LittleFS.exists(backupFilename)) LittleFS.rename(backupFilename, filename);
    return false;
  }
  String readBack = verifyFile.readString();
  verifyFile.close();
  if (readBack != content) {
    Serial.println("[ERROR] Verification failed.");
    LittleFS.remove(tempFilename);
    if (LittleFS.exists(backupFilename)) LittleFS.rename(backupFilename, filename);
    return false;
  }

  // Atomic rename
  if (LittleFS.exists(filename)) LittleFS.remove(filename);
  if (!LittleFS.rename(tempFilename, filename)) {
    if (LittleFS.exists(backupFilename)) LittleFS.rename(backupFilename, filename);
    return false;
  }

  // Cleanup backup
  if (LittleFS.exists(backupFilename)) LittleFS.remove(backupFilename);
  Serial.printf("[SUCCESS] Atomic write completed for: %s\n", filename.c_str());
  return true;
}

// ══════════════════════════════════════════════════════
//  CONFIG FILE SAVE / LOAD (per-topic config files)
// ══════════════════════════════════════════════════════

// Save a JSON config string to a topic-encoded file on LittleFS
void saveConfigToFile(const String& jsonString, String filename) {
  Serial.printf("[DEBUG] saveConfigToFile for topic: %s\n", filename.c_str());
  String originalTopic = filename;
  originalTopic.replace("/configs", "");
  String safeFilename = encodeTopicToFilename(filename);
  Serial.printf("[DEBUG] Encoded filename: %s\n", safeFilename.c_str());

  DynamicJsonDocument config(2048);
  DeserializationError error = deserializeJson(config, jsonString);
  if (error) {
    Serial.printf("[ERROR] Invalid JSON: %s. Skipping save.\n", error.c_str());
    return;
  }

  if (!atomicWriteFile(safeFilename, jsonString)) {
    Serial.printf("[ERROR] Failed to write file: %s\n", safeFilename.c_str());
    return;
  }

  configMap[originalTopic] = jsonString;
  Serial.printf("[INFO] Config stored for topic: %s\n", originalTopic.c_str());
  printConfigMap();
}

// Load all topic config files from LittleFS into configMap
void loadConfigFromFiles() {
  Serial.println("[DEBUG] Starting loadConfigFromFiles...");
  String excludedFiles[] = {"/config.json", "/exclude_this.json", "config.json"};
  size_t excludedFileCount = sizeof(excludedFiles) / sizeof(excludedFiles[0]);

  File root = LittleFS.open("/");
  if (!root || !root.isDirectory()) {
    Serial.println("[ERROR] Failed to open root directory.");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    String filename = file.name();
    bool isExcluded = false;
    for (size_t i = 0; i < excludedFileCount; i++) {
      if (filename == excludedFiles[i]) { isExcluded = true; break; }
    }
    if (isExcluded) {
      file.close();
      file = root.openNextFile();
      continue;
    }
    if (filename.endsWith(".json") && !filename.endsWith(".tmp") && !filename.endsWith(".bak")) {
      String fileContent = file.readString();
      file.close();
      DynamicJsonDocument config(2048);
      DeserializationError error = deserializeJson(config, fileContent);
      if (error) {
        Serial.printf("[ERROR] Bad JSON in %s: %s\n", filename.c_str(), error.c_str());
      } else {
        String topic = decodeFilenameToTopic(filename);
        configMap[topic] = fileContent;
        Serial.printf("[INFO] Loaded config for topic: %s\n", topic.c_str());
      }
    } else {
      file.close();
    }
    file = root.openNextFile();
  }
  Serial.println("[DEBUG] Finished loadConfigFromFiles.");
}

// ══════════════════════════════════════════════════════
//  NETWORK SETUP (Modbus TCP over WiFi or Ethernet)
// ══════════════════════════════════════════════════════

// Setup WiFi or Ethernet for Modbus TCP based on protocolType
void setupNetwork() {
  if (strcmp(protocolType, "tcpwifi") == 0) {
    Serial.println("Initializing Wi-Fi for Modbus TCP...");
    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    if (strcmp(ipMode, "manual") == 0) {
      IPAddress ip, gateway, subnet;
      ip.fromString(staticIP);
      gateway.fromString(gatewayIPs);
      subnet.fromString(subnetMasks);
      if (!WiFi.config(ip, gateway, subnet)) {
        Serial.println("Failed to configure static IP for Wi-Fi.");
        return;
      }
      Serial.print("Static IP: "); Serial.println(ip);
      Serial.print("Gateway: "); Serial.println(gateway);
      Serial.print("Subnet: "); Serial.println(subnet);
    } else {
      Serial.println("Using DHCP (Auto IP Mode) for Wi-Fi.");
    }
    WiFi.begin();
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 30) {
      Serial.print(".");
      delay(500);
      retries++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWi-Fi connected.");
      Serial.print("IP Address: "); Serial.println(WiFi.localIP());
    } else {
      Serial.println("\nFailed to connect to Wi-Fi.");
    }

  } else if (strcmp(protocolType, "tcpethernet") == 0) {
    Serial.println("Initializing Ethernet for Modbus TCP...");
    initW5500();                                   // NEW: replaces Ethernet.init(ETH_CS_PIN)
    parseMACAddress(ethMacAddr, mac);
    if (strcasecmp(ipMode, "manual") == 0) {
      IPAddress ip, gateway, subnet;
      ip.fromString(staticIP);
      gateway.fromString(gatewayIPs);
      subnet.fromString(subnetMasks);
      Ethernet.begin(mac, ip, gateway, subnet);
      Serial.print("Static IP: "); Serial.println(ip);
      Serial.print("Gateway: "); Serial.println(gateway);
      Serial.print("Subnet: "); Serial.println(subnet);
    } else {
      Serial.println("Using DHCP (Auto IP Mode) for Ethernet.");
      Ethernet.begin(mac);
      if (Ethernet.localIP() == INADDR_NONE) {
        Serial.println("DHCP failed, falling back to static IP.");
        IPAddress ip, gateway, subnet;
        ip.fromString(staticIP);
        gateway.fromString(gatewayIPs);
        subnet.fromString(subnetMasks);
        Ethernet.begin(mac, ip, gateway, subnet);
        Serial.print("Static IP: "); Serial.println(ip);
      }
    }
    int retries = 0;
    while (Ethernet.linkStatus() == LinkOFF && retries < 10) {
      Serial.println("Waiting for Ethernet connection...");
      delay(1000);
      retries++;
    }
    if (Ethernet.linkStatus() != LinkOFF) {
      Serial.println("Ethernet connected.");
      Serial.print("IP Address: "); Serial.println(Ethernet.localIP());
    } else {
      Serial.println("Failed to connect via Ethernet.");
    }

  } else {
    Serial.println("Invalid protocol type.");
  }
}

// ══════════════════════════════════════════════════════
//  BUTTON HANDLER
// ══════════════════════════════════════════════════════

unsigned long buttonPressTime = 0;       // Time when button was pressed
bool buttonPressed = false;              // Tracks if button is currently pressed
bool longPressActive = false;            // Tracks if long press is active
int displayIndex = 0;                    // Index for scrolling through parameters
unsigned long lastScrollTime = 0;        // Time of the last scroll
int buttonClickCount = 0;               // Counter for short button presses

// Check button press, return click count on short press, -1 if no press
int checkButton() {
  int buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == LOW) {
    if (!buttonPressed) {
      buttonPressed = true;
      buttonPressTime = millis();
    }
  } else if (buttonPressed) {
    if (millis() - buttonPressTime < 1000) {
      buttonClickCount++;
      Serial.print("Button short press count: ");
      Serial.println(buttonClickCount);
      buttonPressed = false;
      return buttonClickCount;
    }
    buttonPressed = false;
  }
  return -1;
}

// ══════════════════════════════════════════════════════
//  MQTT PUBLISH
// ══════════════════════════════════════════════════════

// Publish MQTT message via the active connection method (WiFi/GSM/Ethernet)
void publishMessage(const char* mqttTopic, const char* msg, int qos) {
  if (strcmp(mqttMethod, "gsm") == 0) {
    if (gsmMQTTConnected) {
      sendMQTTViaGSM(mqttTopic, msg);
    } else {
      Serial.println("GSM MQTT not connected. Reconnecting...");
      reconnect();
    }
  } else if (strcmp(mqttMethod, "wifi") == 0) {
    if (wifiClientPubSub.connected()) {
      wifiClientPubSub.publish(mqttTopic, msg, strlen(msg), false, qos);
      Serial.println("Sending via Wi-Fi");
    } else {
      Serial.println("Wi-Fi MQTT not connected. Reconnecting...");
      reconnect();
    }
  } else if (strcmp(mqttMethod, "ethernet") == 0) {
    if (ethernetClientPubSub.connected()) {
      ethernetClientPubSub.publish(mqttTopic, msg, strlen(msg), false, qos);
      Serial.println("Sending via Ethernet");
    } else {
      Serial.println("Ethernet MQTT not connected. Reconnecting...");
      reconnect();
    }
  } else {
    Serial.println("Unknown MQTT method.");
  }
}


// ══════════════════════════════════════════════════════
//  GSM SETUP & BACKOFF
// ══════════════════════════════════════════════════════

// Setup GSM modem: init UART, restart modem, connect to GPRS
// Interruptible delay for GSM setup — checks GPIO33 every 200 ms.
// If GPIO33 goes LOW during wait, clears all config and reboots.
static void gsmWait(unsigned long ms) {
  unsigned long end = millis() + ms;
  while (millis() < end) {
    if (configResetPressed()) {
      Serial.println("[RESET] GPIO33 pressed. Clearing config...");
      WiFiManager wmReset;
      wmReset.resetSettings();
      clearLittleFS();
      skipConfigResetAfterClear = true;
      delay(200);
      ESP.restart();
    }
    delay(200);
  }
}

bool setupGSM() {
  connectionStartTime = millis();
  while (true) {
    // Check reset button at top of every retry iteration
    if (configResetPressed()) {
      Serial.println("[RESET] GPIO33 pressed in GSM mode. Clearing config...");
      WiFiManager wmReset;
      wmReset.resetSettings();
      clearLittleFS();
      skipConfigResetAfterClear = true;
      delay(200);
      ESP.restart();
    }

    if (gsmRetryCount > MAX_GSM_RETRY_COUNT) {
      Serial.println("Exceeded maximum retries, resetting modem...");
      gsmSendAT("AT+CFUN=1,1", 10000);
      gsmWait(10000);
      gsmRetryCount = 0;
      retryDelay = GSM_INITIAL_RETRY_MS;
    }

    SerialAT.begin(115200, SERIAL_8N1, GSM_RX, GSM_TX);
    gsmWait(3000);  // modem UART stabilise
    Serial.println("Initializing modem (Raw AT)...");

    if (gsmSendAT("AT", 2000, "OK", "ERROR", "Modem Test").indexOf("OK") != -1) {
      Serial.println("Modem is alive, running init...");
      gsmSendAT("ATE0", 2000, "OK", "ERROR", "Echo Off"); // Disable Echo
    } else {
      Serial.println("Modem not responding, attempting restart...");
      gsmSendAT("AT+CFUN=1,1", 10000, "OK", "ERROR", "Hard Reboot"); // Reset modem
      gsmWait(10000);
      continue;
    }

    Serial.println("Waiting for network...");
    {
      unsigned long netStart = millis();
      bool networkOk = false;
      while (millis() - netStart < 60000) {
        if (configResetPressed()) {
          Serial.println("[RESET] GPIO33 pressed while waiting for network. Clearing config...");
          WiFiManager wmReset;
          wmReset.resetSettings();
          clearLittleFS();
          skipConfigResetAfterClear = true;
          delay(200);
          ESP.restart();
        }
        // A7672S is LTE — check EPS (CEREG), GPRS (CGREG), and GSM (CREG)
        String regs[] = {"AT+CEREG?", "AT+CGREG?", "AT+CREG?"};
        for (int i = 0; i < 3; i++) {
            String regResp = gsmSendAT(regs[i], 1000, "OK", "ERROR", "Network Check");
            if (regResp.indexOf(",1") >= 0 || regResp.indexOf(",5") >= 0) {
                networkOk = true;
                break;
            }
        }
        if (networkOk) break;
        delay(500);
      }
      if (!networkOk) {
        Serial.println("Network connection failed. Retrying in " + String(retryDelay / 1000) + " seconds...");
        gsmWait(retryDelay);
        increaseBackoffDelay();
        continue;
      }
    }

    // Log signal quality
    String csqRes = gsmSendAT("AT+CSQ", 2000);
    Serial.println("[GSM] Signal: " + csqRes);

    Serial.println("[GSM] Activating APN Context...");
   
    // Explicitly use "airtelgprs.com" as per the working code
    String activeApn = (apn[0] != '\0') ? String(apn) : "airtelgprs.com";

    // Matching the user's working initNetwork logic exactly
    gsmSendAT("AT+CGATT=1", 5000, "OK", "ERROR", "Attach GPRS");
    gsmSendAT("AT+CGDCONT=1,\"IP\",\"" + activeApn + "\"", 5000, "OK", "ERROR", "Set APN");
    gsmSendAT("AT+CGACT=1,1", 5000, "OK", "ERROR", "Activate Context");

    // Final check for connection status
    String cgactRes = gsmSendAT("AT+CGACT?", 3000, "OK", "ERROR", "Activation Check");
    if (cgactRes.indexOf(",1") == -1) {
      Serial.println("GPRS activation failed. Retrying...");
      gsmWait(retryDelay);
      increaseBackoffDelay();
      continue;
    }

    Serial.println("GSM setup successful.");
    // Manual IP check
    gsmSendAT("AT+CGPADDR=1", 2000, "OK", "ERROR", "Check IP");
    resetBackoffDelay();
    return true;
  }
}

// Exponentially increase retry delay, capped at max
void increaseBackoffDelay() {
  retryDelay = min(retryDelay * 2, (int)GSM_MAX_RETRY_MS);
}

// Reset retry delay to initial value
void resetBackoffDelay() {
  retryDelay = GSM_INITIAL_RETRY_MS;
}

// ══════════════════════════════════════════════════════
//  GSM MQTT  (Pure Raw AT Commands - No TinyGSM MQTT lib)
// ══════════════════════════════════════════════════════

// Track AT-based MQTT connection state (declared in global scope above)

// Internal helper: raw AT sendAT wrapper (blocking with expected response)
String gsmSendAT(String cmd, uint32_t timeoutMs, String expected1, String expected2, String desc) {
    Serial.print("\n[GSM-AT]");
    if (desc != "") { Serial.print(" ("); Serial.print(desc); Serial.print(")"); }
    Serial.print(">> ");
    Serial.println(cmd);
    while(SerialAT.available()) SerialAT.read(); // flush
    SerialAT.println(cmd);
    uint32_t t_start = millis();
    String response = "";
    while (millis() - t_start < timeoutMs) {
        while (SerialAT.available()) response += (char)SerialAT.read();
        if (expected1 != "" && response.indexOf(expected1) != -1) break;
        if (expected2 != "" && response.indexOf(expected2) != -1) break;
        delay(10);
    }
    Serial.print(response);
    return response;
}

// Connect to the MQTT broker using AT+CMQTT commands
void connectMQTTGSM() {
    Serial.println("[GSM-MQTT] Connecting via AT+CMQTT...");
    gsmMQTTConnected = false;

    // Aggressive cleanup before starting
    gsmSendAT("AT+CMQTTDISC=0,120", 1000, "OK", "ERROR");
    gsmSendAT("AT+CMQTTREL=0", 1000, "OK", "ERROR");
    gsmSendAT("AT+CMQTTSTOP", 1000, "OK", "ERROR");
    gsmWait(500);

    // Start MQTT service (ignore error if already running)
    gsmSendAT("AT+CMQTTSTART", 3000, "OK", "ERROR", "MQTT Start");

    // Reverting to the requested client ID
    String clientId = "clientId-AhOtDapiJG";
    Serial.println("[GSM-MQTT] Using ClientID: " + clientId);
   
    String accqCmd = "AT+CMQTTACCQ=0,\"" + clientId + "\",0";
    String accqRes = gsmSendAT(accqCmd, 3000, "OK", "ERROR", "MQTT Acquire");
    if (accqRes.indexOf("ERROR") != -1) {
        Serial.println("[GSM-MQTT] Acquire failed. Retrying in next loop.");
        increaseBackoffDelay();
        return;
    }

    // Build connect command with no-auth format for brokers like HiveMQ
    String broker = "tcp://" + String(mqttserver) + ":" + String(mqttport);
    String connCmd = "AT+CMQTTCONNECT=0,\"" + broker + "\",60,1";
   
    // Only add credentials if they are NOT empty
    if (strlen(mqttusername) > 0 && strcmp(mqttusername, "") != 0) {
        connCmd += ",\"" + String(mqttusername) + "\",\"" + String(mqttpassword) + "\"";
        Serial.println("[GSM-MQTT] Using authenticated connection.");
    } else {
        Serial.println("[GSM-MQTT] Using anonymous connection (no user/pass).");
    }

    String res = gsmSendAT(connCmd, 30000, "+CMQTTCONNECT: 0,0", "ERROR", "Broker Connect");

    if (res.indexOf("0,0") != -1) {
        Serial.println("[GSM-MQTT] Connected to broker!");
        gsmMQTTConnected = true;
        sendMQTTViaGSM("bit/values", "hi from here");
        resetBackoffDelay();
    } else {
        Serial.println("[GSM-MQTT] Connection failed (19=Network/Reject). Cleaning up...");
        gsmSendAT("AT+CMQTTDISC=0,120", 2000, "OK", "ERROR");
        gsmSendAT("AT+CMQTTREL=0", 2000, "OK", "ERROR");
        gsmSendAT("AT+CMQTTSTOP", 2000, "OK", "ERROR");
        gsmWait(2000); // 2s delay to let broker clear session
        increaseBackoffDelay();
        gsmMQTTConnected = false;
    }
}

// Publish MQTT message via GSM using raw AT commands
void sendMQTTViaGSM(const char* topic, const char* payload) {
    if (!gsmMQTTConnected) {
        connectMQTTGSM();
    }
    if (!gsmMQTTConnected) {
        Serial.println("[GSM-MQTT] Cannot publish — not connected.");
        return;
    }

    String topicStr = String(topic);
    String payloadStr = String(payload);

    // Set topic
    gsmSendAT("AT+CMQTTTOPIC=0," + String(topicStr.length()), 2000, ">");
    gsmSendAT(topicStr, 2000, "OK");

    // Set payload
    gsmSendAT("AT+CMQTTPAYLOAD=0," + String(payloadStr.length()), 2000, ">");
    gsmSendAT(payloadStr, 2000, "OK");

    // Publish with QoS 1
    String pubRes = gsmSendAT("AT+CMQTTPUB=0,1,60", 5000, "+CMQTTPUB: 0,0");
    if (pubRes.indexOf("0,0") != -1) {
        Serial.println("[GSM-MQTT] Published successfully.");
    } else {
        Serial.println("[GSM-MQTT] Publish failed.");
        gsmMQTTConnected = false;
    }
}

// Subscribe to MQTT topic via GSM using raw AT commands
void subscribeMQTTGSM(const char* topic, int qos) {
    if (!gsmMQTTConnected) {
        connectMQTTGSM();
    }
    if (!gsmMQTTConnected) return;

    String topicStr = String(topic);
    gsmSendAT("AT+CMQTTSUB=0," + String(topicStr.length()) + "," + String(qos), 2000, ">");
    gsmSendAT(topicStr, 2000, "OK");
    Serial.println("[GSM-MQTT] Subscribed to: " + topicStr);
}

// Handle GSM MQTT loop (poll SerialAT for incoming MQTT msgs)
void handleGSMMQTT() {
    if (!gsmMQTTConnected) {
        unsigned long currentMillis = millis();
        if (currentMillis - previousReconnectAttempt >= reconnectInterval) {
            Serial.println("[GSM-MQTT] Reconnecting...");
            previousReconnectAttempt = currentMillis;
            connectMQTTGSM();
        }
        return;
    }
    // Poll for incoming AT URC messages from the modem
    while (SerialAT.available()) {
        String line = SerialAT.readStringUntil('\n');
        line.trim();
        if (line.startsWith("+CMQTTRXSTART") || line.startsWith("+CMQTTRXTOPIC") || line.startsWith("+CMQTTRXPAYLOAD")) {
            Serial.println("[GSM-MQTT] Incoming: " + line);
        }
        if (line.indexOf("+CMQTTNONET") != -1 || line.indexOf("NO CARRIER") != -1) {
            gsmMQTTConnected = false;
        }
    }
}

// Reconnect GSM network + MQTT
void reconnectMQTTGSM() {
    Serial.println("[GSM-MQTT] Checking network before reconnect...");
    // Use AT+CREG to check network instead of modem.isNetworkConnected()
    String regRes = gsmSendAT("AT+CREG?", 2000);
    if (regRes.indexOf(",1") == -1 && regRes.indexOf(",5") == -1) {
        Serial.println("[GSM] Network lost — re-running setupGSM()...");
        setupGSM();
    }
    connectMQTTGSM();
}

// ══════════════════════════════════════════════════════
//  WIFI & ETHERNET MQTT HANDLERS
// ══════════════════════════════════════════════════════

// Handle WiFi MQTT loop, reconnect WiFi and MQTT if needed
void handleWiFiMQTT() {
  wifiClientPubSub.loop();
  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long lastWifiAttemptMs = 0;
    static int wifiFailCount = 0;
    unsigned long now = millis();
    if (now - lastWifiAttemptMs < 10000) return;  // throttle: try every 10s only
    lastWifiAttemptMs = now;
    Serial.println("Reconnecting to WiFi...");
    WiFi.reconnect();
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20) { delay(500); retries++; yield(); }
    if (WiFi.status() != WL_CONNECTED) {
      wifiFailCount++;
      Serial.printf("[WARN] WiFi reconnect failed (%d/10)\n", wifiFailCount);
      if (wifiFailCount >= 10) {
        Serial.println("[ERROR] WiFi failed 10 consecutive times. Restarting...");
        delay(500);
        ESP.restart();
      }
      return;
    }
    wifiFailCount = 0;  // reset on success
  }
  if (!wifiClientPubSub.connected()) {
    reconnect();
  }
}

// Handle Ethernet MQTT loop, reconnect if link is down
void handleEthernetMQTT() {
  ethernetClientPubSub.loop();
  if (Ethernet.linkStatus() == LinkOFF) {
    static unsigned long lastEthLinkMs = 0;
    unsigned long now = millis();
    if (now - lastEthLinkMs >= 10000) {  // throttle link-down retries to every 10s
      lastEthLinkMs = now;
      Serial.println("Ethernet link is down, reconnecting...");
      Ethernet.begin(mac);
    }
    return;  // no point trying MQTT while link is down
  }
  if (!ethernetClientPubSub.connected()) {
    static unsigned long lastEthMqttAttemptMs = 0;
    unsigned long now = millis();
    if (now - lastEthMqttAttemptMs < 10000) return;  // throttle to every 10s
    lastEthMqttAttemptMs = now;
    reconnect();
  }
}

// ══════════════════════════════════════════════════════
//  MQTT RECONNECT (unified for all methods)
// ══════════════════════════════════════════════════════

// Reconnect MQTT and resubscribe to all topics based on active method
void reconnect() {
  if (strcmp(mqttMethod, "gsm") == 0) {
    Serial.println("Attempting GSM MQTT connection...");
    connectMQTTGSM();
    delay(500);
    if (gsmMQTTConnected) {
      Serial.println("GSM MQTT connected");
      subscribeMQTTGSM(FIRMWARE_TOPIC, 1);
      subscribeMQTTGSM(CONFIG_TOPIC, 1);
      testLoopbackMQTT();
      for (int i = 0; i < count1; i++) {
        if (splitmqttaddwrite[i].length() > 0) {
          subscribeMQTTGSM(splitmqttaddwrite[i].c_str(), 1);
          delay(10);
        }
      }
      for (int i = 0; i < count; i++) {
        if (splitmqttaddread[i].length() > 0) {
          String topic = splitmqttaddread[i] + "/configs";
          subscribeMQTTGSM(topic.c_str(), 1);
          delay(10);
        }
      }
    } else {
      Serial.println("Failed to connect GSM MQTT");
    }

  } else if (strcmp(mqttMethod, "wifi") == 0) {
    // Only reconnect WiFi if actually disconnected
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected, reconnecting...");
      WiFi.reconnect();
      int retries = 0;
      while (WiFi.status() != WL_CONNECTED && retries < 20) { delay(500); retries++; yield(); }
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi reconnect failed, skipping MQTT.");
        return;
      }
    }
    // Non-blocking: single attempt per call, throttled by reconnectInterval
    unsigned long now = millis();
    if (now - previousReconnectAttempt < reconnectInterval) return;
    previousReconnectAttempt = now;

    Serial.println("Attempting Wi-Fi MQTT connection...");
    Serial.printf("[HEAP] reconnect wifi: %u bytes\n", ESP.getFreeHeap());
    yield();
    String clientId = "RoloblogWifi-";
    clientId += String(random(0xffff), HEX);
    if (wifiClientPubSub.connect(clientId.c_str(), mqttusername, mqttpassword)) {
      Serial.println("Wi-Fi MQTT connected");
      yield();
      wifiClientPubSub.subscribe(FIRMWARE_TOPIC, 1);
      wifiClientPubSub.subscribe(CONFIG_TOPIC, 1);
      testLoopbackMQTT();
      for (int i = 0; i < count1; i++) {
        if (splitmqttaddwrite[i].length() > 0) {
          wifiClientPubSub.subscribe(splitmqttaddwrite[i].c_str(), 1);
          delay(10);
        }
      }
      for (int i = 0; i < count; i++) {
        if (splitmqttaddread[i].length() > 0) {
          String topic = splitmqttaddread[i] + "/configs";
          wifiClientPubSub.subscribe(topic.c_str(), 1);
          delay(10);
        }
      }
    } else {
      Serial.println("Wi-Fi MQTT connection failed, will retry.");
    }

  } else if (strcmp(mqttMethod, "ethernet") == 0) {
    Serial.println("Attempting Ethernet MQTT connection...");
    // Always call begin() before connect() — if Ethernet link was down at boot,
    // begin() may have been skipped in setup(), leaving a null network client
    // inside MQTTClient. Calling begin() here is safe to repeat on every reconnect.
    // onMessage() is NOT called here — it was already registered once in setup()
    // and persists across reconnects. Re-registering here would fail to compile
    // because the String& String& overload is defined later in the file.
    ethernetClientPubSub.begin(mqttserver, atoi(mqttport), ethClient);
    String clientId = "RobologEthernetClient-";
    clientId += String(random(0xffff), HEX);
    if (ethernetClientPubSub.connect(clientId.c_str(), mqttusername, mqttpassword)) {
      Serial.println("Ethernet MQTT connected");
      ethernetClientPubSub.subscribe(FIRMWARE_TOPIC, 1);
      ethernetClientPubSub.subscribe(CONFIG_TOPIC, 1);
      testLoopbackMQTT();
      for (int i = 0; i < count1; i++) {
        if (splitmqttaddwrite[i].length() > 0) {
          ethernetClientPubSub.subscribe(splitmqttaddwrite[i].c_str(), 1);
          delay(10);
        }
      }
      for (int i = 0; i < count; i++) {
        if (splitmqttaddread[i].length() > 0) {
          String topic = splitmqttaddread[i] + "/configs";
          ethernetClientPubSub.subscribe(topic.c_str(), 1);
          delay(10);
        }
      }
    } else {
      Serial.println("Failed to connect to Ethernet MQTT");
    }

  } else {
    Serial.println("Unknown MQTT method.");
  }
}

// Check and reconnect Modbus TCP network (WiFi/Ethernet/RS485)
void checkReconnect() {
  if (strcmp(protocolType, "tcpwifi") == 0) {
    if (WiFi.status() != WL_CONNECTED) {
      static unsigned long lastTcpWifiAttemptMs = 0;
      unsigned long now = millis();
      if (now - lastTcpWifiAttemptMs < 10000) return;  // throttle to every 10s
      lastTcpWifiAttemptMs = now;
      Serial.println("Wi-Fi connection lost, reconnecting...");
      WiFi.reconnect();
      int retries = 0;
      while (WiFi.status() != WL_CONNECTED && retries < 20) {
        Serial.print(".");
        delay(500);
        retries++;
      }
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWi-Fi reconnected.");
        Serial.print("IP Address: "); Serial.println(WiFi.localIP());
      } else {
        Serial.println("\nFailed to reconnect to Wi-Fi.");
      }
    }
  } else if (strcmp(protocolType, "tcpethernet") == 0) {
    if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet connection lost, reconnecting...");
      Ethernet.begin(mac);
      int retries = 0;
      while (Ethernet.linkStatus() == LinkOFF && retries < 10) {
        Serial.println("Waiting for Ethernet reconnection...");
        delay(1000);
        retries++;
      }
      if (Ethernet.linkStatus() != LinkOFF) {
        Serial.println("Ethernet reconnected.");
        Serial.print("IP Address: "); Serial.println(Ethernet.localIP());
      } else {
        Serial.println("Failed to reconnect via Ethernet.");
      }
    }
  } else if (strcmp(protocolType, "rs485") == 0) {
    // RS485 is always connected, nothing to check
  } else {
    Serial.println("Unknown protocol type.");
  }
}

// Disconnect MQTT from active method
void disconnectMQTT() {
  Serial.println("[INFO] Disconnecting MQTT...");
  if (strcmp(mqttMethod, "gsm") == 0) {
    if (gsmMQTTConnected) {
      gsmSendAT("AT+CMQTTDISC=0,60", 3000, "+CMQTTDISC");
      gsmSendAT("AT+CMQTTREL=0", 2000, "OK");
      gsmSendAT("AT+CMQTTSTOP", 2000, "OK");
      gsmMQTTConnected = false;
      Serial.println("[INFO] GSM MQTT disconnected.");
    }
  } else if (strcmp(mqttMethod, "wifi") == 0) {
    if (wifiClientPubSub.connected()) {
      wifiClientPubSub.disconnect();
      Serial.println("[INFO] Wi-Fi MQTT disconnected.");
    }
  } else if (strcmp(mqttMethod, "ethernet") == 0) {
    if (ethernetClientPubSub.connected()) {
      ethernetClientPubSub.disconnect();
      Serial.println("[INFO] Ethernet MQTT disconnected.");
    }
  }
}

// ══════════════════════════════════════════════════════
//  OTA FIRMWARE UPDATE
// ══════════════════════════════════════════════════════

// Extract file path from full URL (e.g. "http://host/path/file.bin" -> "/path/file.bin")
String extractPathFromURL(const String &url) {
  int protocolPos = url.indexOf("://");
  if (protocolPos != -1) {
    int pathPos = url.indexOf("/", protocolPos + 3);
    if (pathPos != -1) return url.substring(pathPos);
  }
  return "/firmware.bin";
}

// Download firmware over HTTP and flash via OTA
void downloadFirmwareHTTP(String firmware_url, Client &client) {
  Serial.println("[INFO] Starting HTTP firmware download...");

  String filePath = extractPathFromURL(firmware_url);
  String host = firmware_url.substring(7);
  int slashIndex = host.indexOf("/");
  if (slashIndex != -1) host = host.substring(0, slashIndex);

  Serial.print("[INFO] Connecting to: "); Serial.println(host);
  HttpClient httpClient(client, host, 80);
  httpClient.get(filePath);

  int status_code = httpClient.responseStatusCode();
  if (status_code != 200) {
    Serial.println("[ERROR] Failed to retrieve firmware.");
    return;
  }

  long content_length = httpClient.contentLength();
  if (content_length <= 0 || content_length > ESP.getFreeSketchSpace()) {
    Serial.println("[ERROR] Invalid content length or not enough space.");
    return;
  }

  if (!Update.begin(content_length)) {
    Serial.println("[ERROR] Not enough space for OTA!");
    return;
  }

  int totalBytesRead = 0;
  uint8_t buffer[512];
  while (totalBytesRead < content_length) {
    int bytesRead = httpClient.readBytes(buffer, sizeof(buffer));
    if (bytesRead > 0) {
      if (!Update.write(buffer, bytesRead)) {
        Serial.println("[ERROR] Flash write failed!");
        return;
      }
      totalBytesRead += bytesRead;
      int percentage = (totalBytesRead * 100) / content_length;
      Serial.printf("[OTA] %d/%d (%d%%)\n", totalBytesRead, content_length, percentage);
    } else {
      Serial.println("[ERROR] Timeout or no data.");
      break;
    }
  }

  if (Update.end()) {
    Serial.println("[SUCCESS] Firmware update completed! Rebooting...");
    delay(2000);
    ESP.restart();
  } else {
    Serial.println("[ERROR] Update failed!");
  }
}

// Route firmware update to correct transport (GSM/WiFi/Ethernet)
void handleFirmwareUpdate(String &url) {
  if (!url.startsWith("http://")) {
    Serial.println("[ERROR] Invalid firmware URL: " + url);
    return;
  }
  Serial.println("Firmware URL received: " + url);

  if (strcmp(mqttMethod, "gsm") == 0) {
    TinyGsmClient httpClient(modem);
    downloadFirmwareHTTP(url, httpClient);
  } else if (strcmp(mqttMethod, "wifi") == 0) {
    WiFiClient wifiClient;
    downloadFirmwareHTTP(url, wifiClient);
  } else if (strcmp(mqttMethod, "ethernet") == 0) {
    EthernetClient ethOtaClient;
    downloadFirmwareHTTP(url, ethOtaClient);
  } else {
    Serial.println("[ERROR] Unknown network method.");
  }
}


// ══════════════════════════════════════════════════════
//  LITTLEFS FILE LISTING
// ══════════════════════════════════════════════════════

// Recursively list files and directories
void listFilesAndDirectories(const char *dirname) {
  Serial.print("Listing paths under: "); Serial.println(dirname);
  File root = LittleFS.open(dirname);
  if (!root || !root.isDirectory()) { Serial.println("[ERROR] Not a directory."); return; }
  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("DIR : "); Serial.println(file.name());
      listFilesAndDirectories(file.name());
    } else {
      Serial.print("FILE: "); Serial.print(file.name());
      Serial.print("  SIZE: "); Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

// List all stored files with contents
void listStoredFiles() {
  Serial.println("Listing files in LittleFS:");
  File root = LittleFS.open("/");
  File file = root.openNextFile();
  if (!file) { Serial.println("No files found."); return; }
  while (file) {
    Serial.print("File: "); Serial.print(file.name());
    Serial.print(" - Size: "); Serial.print(file.size()); Serial.println(" bytes");
    Serial.println("File content:");
    while (file.available()) Serial.print((char)file.read());
    Serial.println();
    file = root.openNextFile();
  }
  Serial.println("Finished listing files.");
}

// List all paths recursively
void listPaths(const char* path) {
  Serial.printf("Listing paths under: %s\n", path);
  File root = LittleFS.open(path);
  if (!root || !root.isDirectory()) { Serial.println("Not a directory."); return; }
  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.printf("DIR : %s\n", file.name());
      listPaths(file.name());
    } else {
      Serial.printf("FILE: %s\tSIZE: %u bytes\n", file.name(), file.size());
    }
    file = root.openNextFile();
  }
  Serial.println("Finished listing paths.");
}

// ══════════════════════════════════════════════════════
//  SUBSCRIBE HELPER & LOOPBACK TEST
// ══════════════════════════════════════════════════════

// Subscribe to all configured read/write topics
template <typename MQTTClientType>
void subscribeToTopics(MQTTClientType &client) {
  for (int i = 0; i < count1; i++) {
    if (splitmqttaddwrite[i].length() > 0) {
      if (client.connected() && client.subscribe(splitmqttaddwrite[i].c_str(), 1)) {
        Serial.print("Subscribed to: "); Serial.println(splitmqttaddwrite[i]);
        delay(20);
      }
    }
  }
  for (int i = 0; i < count; i++) {
    if (splitmqttaddread[i].length() > 0) {
      String topic = splitmqttaddread[i] + "/configs";
      if (client.connected() && client.subscribe(topic.c_str(), 1)) {
        Serial.print("Subscribed to: "); Serial.println(topic);
        delay(20);
      }
    }
  }
}

// MQTT loopback test to verify connection
void testLoopbackMQTT() {
  const char* testTopic = "esp32max/loopback_test";
  const char* testMessage = "Loopback test message from ESP32";
  Serial.println("Publishing loopback test...");
  publishMessage(testTopic, testMessage, 1);
}

// ══════════════════════════════════════════════════════
//  CONFIG PROCESSING
// ══════════════════════════════════════════════════════

// Load config from file into JsonDocument
bool loadConfig(String filename, JsonDocument& config) {
  File file = LittleFS.open(filename, "r");
  if (!file) { Serial.println("Configuration file not found."); return false; }
  DeserializationError error = deserializeJson(config, file);
  file.close();
  if (error) { Serial.print("Failed to parse JSON: "); Serial.println(error.c_str()); return false; }
  return true;
}

// Ensure path starts with /
String ensureValidPath(String filename) {
  if (filename.startsWith("/")) return filename;
  return "/" + filename;
}

// Parse config JSON from MQTT and store it
void getConfigAndStoreIt(String topic, String message) {
  Serial.println("==== getConfigAndStoreIt START ====");
  StaticJsonDocument<2048> docs;
  DeserializationError error = deserializeJson(docs, message);
  if (error) {
    Serial.print("[ERROR] Failed to parse JSON: "); Serial.println(error.c_str());
    return;
  }
  JsonObject newConfigs = docs.as<JsonObject>();
  storeConfig(newConfigs, topic);
  Serial.println("==== getConfigAndStoreIt END ====");
}

// Store config (stub - uses saveConfigToFile in practice)
void storeConfig(const JsonObject& config, String filename) {
  String jsonStr;
  serializeJson(config, jsonStr);
  saveConfigToFile(jsonStr, filename);
}

// Create nested directories on LittleFS
void createDirectories(String path) {
  if (!path.startsWith("/")) path = "/" + path;
  String dirPath = "/";
  for (int i = 1; i < path.length(); i++) {
    if (path[i] == '/') {
      if (!LittleFS.exists(dirPath)) LittleFS.mkdir(dirPath);
    }
    dirPath += path[i];
  }
  if (!LittleFS.exists(dirPath)) LittleFS.mkdir(dirPath);
}

// Print dynamic configs map (stub — dynamicConfigs not populated elsewhere)
void printDynamicConfigs() {
  Serial.println("Printing Dynamic Configurations:");
  for (const auto& pair : dynamicConfigs) {
    Serial.print("Topic: "); Serial.println(pair.first);
    serializeJsonPretty(pair.second, Serial);
    Serial.println();
  }
}

// ══════════════════════════════════════════════════════
//  SMS
// ══════════════════════════════════════════════════════

// Send SMS via GSM modem
bool sendSMS(const String &phoneNumber, const String &message) {
  Serial.println("[GSM] Sending SMS via Raw AT...");
  gsmSendAT("AT+CMGF=1", 2000); // Set Text Mode
 
  // Clear buffer
  while(SerialAT.available()) SerialAT.read();

  // Send command
  SerialAT.print("AT+CMGS=\"");
  SerialAT.print(9042416640);
  SerialAT.println("\"");
 
  // Wait for prompt '>'
  uint32_t start = millis();
  bool prompt = false;
  while(millis() - start < 5000) {
    if(SerialAT.read() == '>') { prompt = true; break; }
    delay(10);
  }

  if (prompt) {
    SerialAT.print(message);
    SerialAT.write(26); // CTRL+Z
    Serial.println("[GSM] Message sent, waiting for OK...");
    String res = "";
    start = millis();
    while(millis() - start < 10000) {
      if(SerialAT.available()) res += (char)SerialAT.read();
      if(res.indexOf("OK") != -1) {
        Serial.printf("[SUCCESS] SMS sent to: %s\n", phoneNumber.c_str());
        return true;
      }
      delay(10);
    }
  }

  Serial.printf("[ERROR] Failed to send SMS to: %s\n", phoneNumber.c_str());
  return false;
}

// ══════════════════════════════════════════════════════
//  AGGREGATION
// ══════════════════════════════════════════════════════

// Aggregate data buffer values (average/max/min) for notification
float aggregateData(const String& topic, const String& method) {
  if (dataBuffer.find(topic) == dataBuffer.end() || dataBuffer[topic].empty()) return 0.0;
  const auto& values = dataBuffer[topic];
  if (method == "average") return std::accumulate(values.begin(), values.end(), 0.0f) / values.size();
  else if (method == "max") return *std::max_element(values.begin(), values.end());
  else if (method == "min") return *std::min_element(values.begin(), values.end());
  return 0.0;
}

// Aggregate data buffer values for storage
float storeaggregateData(const String& topic, const String& method) {
  if (dataBuffer.find(topic) == dataBuffer.end() || dataBuffer[topic].empty()) return 0.0;
  const auto& values = dataBuffer[topic];
  if (method == "average") return std::accumulate(values.begin(), values.end(), 0.0f) / values.size();
  else if (method == "max") return *std::max_element(values.begin(), values.end());
  else if (method == "min") return *std::min_element(values.begin(), values.end());
  return 0.0;
}

// ══════════════════════════════════════════════════════
//  NESTED CONDITION EVALUATOR
// ══════════════════════════════════════════════════════

// Evaluate nested notification/storage condition against topicValueMap
bool evaluateNestedCondition(JsonObject nestedConfig, const String& parentTopic) {
  if (!nestedConfig.containsKey("tp") || !nestedConfig.containsKey("data")) return false;
  String nestedType = nestedConfig["tp"].as<String>();
  String nestedTopic = nestedConfig["data"].as<String>();
  if (topicValueMap.find(nestedTopic) == topicValueMap.end()) return false;

  String nestedValue = topicValueMap[nestedTopic];
  float currentValue = 0.0;
  if (nestedValue.equalsIgnoreCase("true")) currentValue = 1.0;
  else if (nestedValue.equalsIgnoreCase("false")) currentValue = 0.0;
  else if (nestedValue.toFloat() != 0.0 || nestedValue == "0") currentValue = nestedValue.toFloat();
  else return false;

  if (nestedType == "above_thr") return currentValue > (nestedConfig["thr"] | 0.0f);
  else if (nestedType == "below_thr") return currentValue < (nestedConfig["thr"] | 0.0f);
  else if (nestedType == "within_rng") return currentValue >= (nestedConfig["min"] | 0.0f) && currentValue <= (nestedConfig["max"] | 0.0f);
  else if (nestedType == "exclude_rng") return currentValue < (nestedConfig["min"] | 0.0f) || currentValue > (nestedConfig["max"] | 0.0f);
  else if (nestedType == "match_val") {
    if (nestedConfig["val"].is<float>()) return currentValue == nestedConfig["val"].as<float>();
    else if (nestedConfig["val"].is<String>()) return nestedValue == nestedConfig["val"].as<String>();
    else if (nestedConfig["val"].is<bool>()) return (nestedValue.equalsIgnoreCase("true")) == nestedConfig["val"].as<bool>();
  }
  return false;
}



// ══════════════════════════════════════════════════════
//  NOTIFICATION PROCESSING (with wait/stabilization)
// ══════════════════════════════════════════════════════

// Wrapper: handles wait timer before calling processNotification
void processNotificationWithWait(const String& topic, const String& value, const String& dataType) {
  if (configMap.find(topic) == configMap.end()) return;

  DynamicJsonDocument config(2048);
  deserializeJson(config, configMap[topic]);
  JsonObject notificationConfig = config["notification"];
  if (!notificationConfig.containsKey("tp")) return;

  String notificationType = notificationConfig["tp"].as<String>();
  unsigned long waitTime = notificationConfig["wt"] | 0;
  waitTime *= 1000;
  unsigned long currentMillis = millis();

  float currentValue = 0;
  if (dataType == "int16" || dataType == "uint16") currentValue = value.toInt();
  else if (dataType == "float") currentValue = value.toFloat();
  else if (dataType == "bool") currentValue = (value == "true") ? 1 : 0;

  // Check nested conditions
  if (notificationConfig.containsKey("nested")) {
    JsonObject nestedConfig = notificationConfig["nested"].as<JsonObject>();
    if (!evaluateNestedCondition(nestedConfig, topic)) return;
  }

  // Handle aggregation
  if (notificationConfig.containsKey("agg")) {
    JsonObject aggregationConfig = notificationConfig["agg"];
    String method = aggregationConfig["method"] | "average";
    unsigned long aggregationInterval = aggregationConfig["int"] | 60;
    aggregationInterval *= 1000;
    if (currentMillis - lastBufferUpdate[topic] >= aggregationInterval) {
      float aggregatedValue = aggregateData(topic, method);
      dataBuffer[topic].clear();
      lastBufferUpdate[topic] = currentMillis;
      currentValue = aggregatedValue;
    }
    dataBuffer[topic].push_back(currentValue);
  }

  // Bypass wait for these types
  if (notificationType == "always_notify" || notificationType == "time_based" ||
      notificationType == "val_change" || notificationType == "match_val") {
    processNotification(topic, value, dataType);
    return;
  }

  // Static maps for stabilization
  static std::map<String, unsigned long> waitStartTime;
  static std::map<String, bool> isWaiting;
  bool shouldNotify = false;

  if (notificationType == "above_thr") {
    float threshold = notificationConfig["thr"] | 0.0;
    if (currentValue > threshold) {
      if (!isWaiting[topic]) { waitStartTime[topic] = currentMillis; isWaiting[topic] = true; return; }
      if (currentMillis - waitStartTime[topic] >= waitTime) shouldNotify = true;
    } else { isWaiting[topic] = false; }
  }
  else if (notificationType == "below_thr") {
    float threshold = notificationConfig["thr"] | 0.0;
    if (currentValue < threshold) {
      if (!isWaiting[topic]) { waitStartTime[topic] = currentMillis; isWaiting[topic] = true; return; }
      if (currentMillis - waitStartTime[topic] >= waitTime) shouldNotify = true;
    } else { isWaiting[topic] = false; }
  }
  else if (notificationType == "within_rng") {
    float minVal = notificationConfig["min"] | 0.0;
    float maxVal = notificationConfig["max"] | 0.0;
    if (currentValue >= minVal && currentValue <= maxVal) {
      if (!isWaiting[topic]) { waitStartTime[topic] = currentMillis; isWaiting[topic] = true; return; }
      if (currentMillis - waitStartTime[topic] >= waitTime) shouldNotify = true;
    } else { isWaiting[topic] = false; }
  }
  else if (notificationType == "exclude_rng") {
    float minVal = notificationConfig["min"] | 0.0;
    float maxVal = notificationConfig["max"] | 0.0;
    if (currentValue < minVal || currentValue > maxVal) {
      if (!isWaiting[topic]) { waitStartTime[topic] = currentMillis; isWaiting[topic] = true; return; }
      if (currentMillis - waitStartTime[topic] >= waitTime) shouldNotify = true;
    } else { isWaiting[topic] = false; }
  }

  if (shouldNotify) {
    processNotification(topic, value, dataType);
    isWaiting[topic] = false;
  }
}

// ══════════════════════════════════════════════════════
//  PROCESS NOTIFICATION (evaluate + publish)
// ══════════════════════════════════════════════════════

void processNotification(const String& topic, const String& value, const String& dataType) {
  if (configMap.find(topic) == configMap.end()) return;

  DynamicJsonDocument config(2048);
  deserializeJson(config, configMap[topic]);
  JsonObject notificationConfig = config["notification"];
  if (!notificationConfig.containsKey("tp")) return;

  bool notify = notificationConfig["nfy"] | true;
  String notificationType = notificationConfig["tp"].as<String>();
  unsigned long interval = notificationConfig["int"] | 0;
  interval *= 1000;
  unsigned long currentTimeMillis = millis();

  if (currentTimeMillis - lastSendTime[topic] < interval) return;

  String onValue = notificationConfig["on_val"] | "true";
  String offValue = notificationConfig["off_val"] | "false";
  String priority = notificationConfig["priority"] | "";

  static std::map<String, bool> isInitialized;
  if (!isInitialized[topic]) {
    lastSendValue[topic] = value;
    isInitialized[topic] = true;
    return;
  }

  float currentValue = 0;
  if (dataType == "int16" || dataType == "uint16") currentValue = value.toInt();
  else if (dataType == "float") currentValue = value.toFloat();
  else if (dataType == "bool") currentValue = (value == "true" || value == onValue) ? 1 : 0;

  bool shouldNotify = false;

  if (notificationType == "always_notify") {
    shouldNotify = true;
  }
  else if (notificationType == "above_thr") {
    if (currentValue > (notificationConfig["thr"] | 0.0)) shouldNotify = true;
  }
  else if (notificationType == "below_thr") {
    if (currentValue < (notificationConfig["thr"] | 0.0)) shouldNotify = true;
  }
  else if (notificationType == "within_rng") {
    float minVal = notificationConfig["min"] | 0.0;
    float maxVal = notificationConfig["max"] | 0.0;
    if (currentValue >= minVal && currentValue <= maxVal) shouldNotify = true;
  }
  else if (notificationType == "exclude_rng") {
    float minVal = notificationConfig["min"] | 0.0;
    float maxVal = notificationConfig["max"] | 0.0;
    if (currentValue < minVal || currentValue > maxVal) shouldNotify = true;
  }
  else if (notificationType == "val_change") {
    if (dataType == "bool") {
      if (value != lastSendValue[topic]) shouldNotify = true;
    } else {
      if (currentValue != lastSendValue[topic].toFloat()) shouldNotify = true;
    }
  }
  else if (notificationType == "match_val") {
    if (dataType == "bool") {
      String matchValue = notificationConfig["match_value"] | "true";
      if (value == matchValue || (value == onValue && matchValue == "true") || (value == offValue && matchValue == "false"))
        shouldNotify = true;
    } else {
      if (currentValue == (notificationConfig["match_value"] | 0.0)) shouldNotify = true;
    }
  }
  else if (notificationType == "time_based") {
    String currentTimeStr = getCurrentTimeString();
    int currentSeconds = timeToSeconds(currentTimeStr);
    if (currentSeconds == -1) return;
    String currentDate = getCurrentDate();
    if (currentDate == "0000-00-00") return;

    String timeConfig = notificationConfig["time"] | "";
    timeConfig.trim();
    if (timeConfig.length() == 0) return;

    int timeWindow = 180;
    bool timeMatched = false;
    int startIndex = 0;
    while (startIndex < timeConfig.length()) {
      int commaIndex = timeConfig.indexOf(',', startIndex);
      if (commaIndex == -1) commaIndex = timeConfig.length();
      String scheduledTime = timeConfig.substring(startIndex, commaIndex);
      scheduledTime.trim();
      if (scheduledTime.length() >= 5) {
        int scheduledSeconds = timeToSeconds(scheduledTime);
        if (scheduledSeconds != -1) {
          int timeDifference = abs(currentSeconds - scheduledSeconds);
          if (timeDifference > 43200) timeDifference = 86400 - timeDifference;
          if (timeDifference <= timeWindow) {
            if (lastSentTimes[topic].find(scheduledSeconds) == lastSentTimes[topic].end() ||
                lastSentTimes[topic][scheduledSeconds] != currentDate) {
              timeMatched = true;
              lastSentTimes[topic][scheduledSeconds] = currentDate;
              break;
            }
          }
        }
      }
      startIndex = commaIndex + 1;
    }
    if (timeMatched) shouldNotify = true;
  }

  // Priority logic
  if (shouldNotify && !priority.isEmpty() && dataType == "bool") {
    if (priority == "on" && value != onValue) { lastSendValue[topic] = value; return; }
    if (priority == "off" && value != offValue) { lastSendValue[topic] = value; return; }
  }

  if (shouldNotify) {
    String payload = value;
    if (dataType == "bool") {
      payload = (value == "true" || currentValue == 1 || value == onValue) ? onValue : offValue;
    }

    if (notify) {
      String notificationTopic = topic + "/not";
      publishMessage(notificationTopic.c_str(), payload.c_str(), 1);
    }

    if (notificationConfig.containsKey("top")) {
      JsonArray additionalTopics = notificationConfig["top"].as<JsonArray>();
      for (JsonVariant t : additionalTopics) {
        publishMessage(t.as<String>().c_str(), payload.c_str(), 1);
      }
    }

    // SMS via GSM
    if (strcmp(mqttMethod, "gsm") == 0) {
      if (notificationConfig.containsKey("sms") && notificationConfig["sms"].as<bool>()) {
        if (notificationConfig.containsKey("pn") && notificationConfig.containsKey("msg")) {
          JsonArray phoneNumbers = notificationConfig["pn"].as<JsonArray>();
          String baseMessage = notificationConfig["msg"].as<String>();
          for (JsonVariant phoneVariant : phoneNumbers) {
            String messageToSend = baseMessage + " Value: " + payload;
            sendSMS(phoneVariant.as<String>(), messageToSend);
          }
        }
      }
    }

    lastSendTime[topic] = currentTimeMillis;
    lastSendValue[topic] = value;
  } else {
    lastSendValue[topic] = value;
  }
}

// ══════════════════════════════════════════════════════
//  STORAGE PROCESSING (with wait/stabilization)
// ══════════════════════════════════════════════════════

// Wrapper: handles wait timer before calling processStorage
void processStorageWithWait(const String& topic, const String& value, const String& dataType) {
  if (configMap.find(topic) == configMap.end()) return;

  DynamicJsonDocument config(2048);
  deserializeJson(config, configMap[topic]);
  JsonObject storageConfig = config["storage"];
  if (!storageConfig.containsKey("tp")) return;

  String storageType = storageConfig["tp"].as<String>();
  unsigned long waitTime = storageConfig["wt"] | 0;
  waitTime *= 1000;
  unsigned long currentMillis = millis();

  float currentValue = 0;
  if (dataType == "int16" || dataType == "uint16") currentValue = value.toInt();
  else if (dataType == "float") currentValue = value.toFloat();
  else if (dataType == "bool") currentValue = (value == "true") ? 1 : 0;

  if (storageConfig.containsKey("nested")) {
    JsonObject nestedConfig = storageConfig["nested"].as<JsonObject>();
    if (!evaluateNestedCondition(nestedConfig, topic)) return;
  }

  if (storageType == "always_notify" || storageType == "time_based" ||
      storageType == "val_change" || storageType == "match_val") {
    processStorage(topic, value, dataType);
    return;
  }

  static std::map<String, unsigned long> storageWaitStartTime;
  static std::map<String, bool> storageIsWaiting;
  bool shouldStore = false;

  if (storageType == "above_thr") {
    float threshold = storageConfig["thr"] | 0.0;
    if (currentValue > threshold) {
      if (!storageIsWaiting[topic]) { storageWaitStartTime[topic] = currentMillis; storageIsWaiting[topic] = true; return; }
      if (currentMillis - storageWaitStartTime[topic] >= waitTime) shouldStore = true;
    } else { storageIsWaiting[topic] = false; }
  }
  else if (storageType == "below_thr") {
    float threshold = storageConfig["thr"] | 0.0;
    if (currentValue < threshold) {
      if (!storageIsWaiting[topic]) { storageWaitStartTime[topic] = currentMillis; storageIsWaiting[topic] = true; return; }
      if (currentMillis - storageWaitStartTime[topic] >= waitTime) shouldStore = true;
    } else { storageIsWaiting[topic] = false; }
  }
  else if (storageType == "within_rng") {
    float minVal = storageConfig["min"] | 0.0;
    float maxVal = storageConfig["max"] | 0.0;
    if (currentValue >= minVal && currentValue <= maxVal) {
      if (!storageIsWaiting[topic]) { storageWaitStartTime[topic] = currentMillis; storageIsWaiting[topic] = true; return; }
      if (currentMillis - storageWaitStartTime[topic] >= waitTime) shouldStore = true;
    } else { storageIsWaiting[topic] = false; }
  }
  else if (storageType == "exclude_rng") {
    float minVal = storageConfig["min"] | 0.0;
    float maxVal = storageConfig["max"] | 0.0;
    if (currentValue < minVal || currentValue > maxVal) {
      if (!storageIsWaiting[topic]) { storageWaitStartTime[topic] = currentMillis; storageIsWaiting[topic] = true; return; }
      if (currentMillis - storageWaitStartTime[topic] >= waitTime) shouldStore = true;
    } else { storageIsWaiting[topic] = false; }
  }

  if (storageConfig.containsKey("agg")) {
    JsonObject aggregationConfig = storageConfig["agg"];
    String method = aggregationConfig["method"] | "average";
    unsigned long aggregationInterval = aggregationConfig["int"] | 60;
    aggregationInterval *= 1000;
    if (currentMillis - storelastBufferUpdate[topic] >= aggregationInterval) {
      float aggregatedValue = storeaggregateData(topic, method);
      storedataBuffer[topic].clear();
      storelastBufferUpdate[topic] = currentMillis;
      currentValue = aggregatedValue;
    }
    storedataBuffer[topic].push_back(currentValue);
  }

  if (shouldStore) {
    processStorage(topic, value, dataType);
    storageIsWaiting[topic] = false;
  }
}

// ══════════════════════════════════════════════════════
//  PROCESS STORAGE (evaluate + publish to /store)
// ══════════════════════════════════════════════════════

void processStorage(const String& topic, const String& value, const String& dataType) {
  if (configMap.find(topic) == configMap.end()) return;

  DynamicJsonDocument config(2048);
  deserializeJson(config, configMap[topic]);
  JsonObject storageConfig = config["storage"];
  if (!storageConfig.containsKey("tp")) return;

  bool notify = storageConfig["nfy"] | true;
  String storageType = storageConfig["tp"].as<String>();
  unsigned long interval = storageConfig["int"] | 0;
  interval *= 1000;
  unsigned long currentTimeMillis = millis();

  if (currentTimeMillis - lastStorageTime[topic] < interval) return;

  float currentValue = 0;
  if (dataType == "int16" || dataType == "uint16") currentValue = value.toInt();
  else if (dataType == "float") currentValue = value.toFloat();
  else if (dataType == "bool") currentValue = (value == "true") ? 1 : 0;

  bool shouldStore = false;

  if (storageType == "always_store") shouldStore = true;
  else if (storageType == "above_thr") { if (currentValue > (storageConfig["thr"] | 0.0)) shouldStore = true; }
  else if (storageType == "below_thr") { if (currentValue < (storageConfig["thr"] | 0.0)) shouldStore = true; }
  else if (storageType == "within_rng") {
    float minVal = storageConfig["min"] | 0.0, maxVal = storageConfig["max"] | 0.0;
    if (currentValue >= minVal && currentValue <= maxVal) shouldStore = true;
  }
  else if (storageType == "exclude_rng") {
    float minVal = storageConfig["min"] | 0.0, maxVal = storageConfig["max"] | 0.0;
    if (currentValue < minVal || currentValue > maxVal) shouldStore = true;
  }
  else if (storageType == "val_change") {
    if (currentValue != lastStoredValue[topic].toFloat()) shouldStore = true;
  }
  else if (storageType == "match_val") {
    if (dataType == "bool") {
      bool matchBool;
      if (storageConfig["match_value"].is<bool>()) matchBool = storageConfig["match_value"].as<bool>();
      else if (storageConfig["match_value"].is<const char*>()) {
        String mv = storageConfig["match_value"].as<const char*>(); mv.toLowerCase();
        matchBool = (mv == "true");
      } else return;
      if ((currentValue != 0) == matchBool) shouldStore = true;
    } else {
      if (currentValue == (storageConfig["match_value"] | 0.0)) shouldStore = true;
    }
  }
  else if (storageType == "time_based") {
    String currentTimeStr = getCurrentTimeString();
    int currentSeconds = timeToSeconds(currentTimeStr);
    if (currentSeconds == -1) return;

    String timeConfig = storageConfig["time"] | "";
    timeConfig.trim();
    int timeWindow = 30;
    bool timeMatched = false;
    int startIndex = 0;
    while (startIndex < timeConfig.length()) {
      int commaIndex = timeConfig.indexOf(',', startIndex);
      if (commaIndex == -1) commaIndex = timeConfig.length();
      String scheduledTime = timeConfig.substring(startIndex, commaIndex);
      scheduledTime.trim();
      int scheduledSeconds = timeToSeconds(scheduledTime);
      if (scheduledSeconds != -1 && abs(currentSeconds - scheduledSeconds) <= timeWindow &&
          !lastStoredTimes[topic][scheduledSeconds]) {
        timeMatched = true;
        lastStoredTimes[topic][scheduledSeconds] = true;
        break;
      }
      startIndex = commaIndex + 1;
    }
    if (timeMatched) shouldStore = true;
  }

  if (shouldStore) {
    String payload = value;
    if (notify) {
      String storageTopic = topic + "/store";
      publishMessage(storageTopic.c_str(), payload.c_str(), 1);
    }
    if (storageConfig.containsKey("top")) {
      JsonArray additionalTopics = storageConfig["top"].as<JsonArray>();
      for (JsonVariant t : additionalTopics) {
        publishMessage(t.as<String>().c_str(), payload.c_str(), 1);
      }
    }
    lastStorageTime[topic] = currentTimeMillis;
    lastStoredValue[topic] = value;
  }
}

// ══════════════════════════════════════════════════════
//  ERASE CONFIGURATION FUNCTIONS
// ══════════════════════════════════════════════════════

// Erase all topic-specific configs (preserves /config.json)
void eraseAllTopicConfigs() {
  Serial.println("[WARNING] Starting ERASE ALL TOPIC CONFIGS");
  publishMessage(CONFIG_RESPONSE_TOPIC, "STARTING: Erase all topic configs", 0);
  delay(100);

  int filesErased = 0, filesSkipped = 0, errors = 0;
  String preservedFiles[] = {"/config.json", "config.json"};
  size_t preservedCount = sizeof(preservedFiles) / sizeof(preservedFiles[0]);
  std::vector<String> filesToDelete;

  File root = LittleFS.open("/");
  if (!root || !root.isDirectory()) {
    publishMessage(CONFIG_RESPONSE_TOPIC, "ERROR: Cannot access filesystem", 0);
    return;
  }

  File file = root.openNextFile();
  while (file) {
    String filename = String(file.name());
    size_t fileSize = file.size();
    bool isDir = file.isDirectory();
    file.close();

    if (isDir) { file = root.openNextFile(); continue; }

    String normalizedFilename = filename;
    if (!normalizedFilename.startsWith("/")) normalizedFilename = "/" + normalizedFilename;

    bool shouldPreserve = false;
    for (size_t i = 0; i < preservedCount; i++) {
      String pn = preservedFiles[i];
      if (!pn.startsWith("/")) pn = "/" + pn;
      if (normalizedFilename == pn) { shouldPreserve = true; break; }
    }

    if (shouldPreserve) {
      filesSkipped++;
    } else if (normalizedFilename.endsWith(".json") && !normalizedFilename.endsWith(".tmp") && !normalizedFilename.endsWith(".bak")) {
      filesToDelete.push_back(normalizedFilename);
    } else if (normalizedFilename.endsWith(".tmp") || normalizedFilename.endsWith(".bak")) {
      filesToDelete.push_back(normalizedFilename);
    }
    file = root.openNextFile();
  }

  for (size_t i = 0; i < filesToDelete.size(); i++) {
    String fn = filesToDelete[i];
    bool deleted = LittleFS.remove(fn.c_str());
    if (!deleted) {
      String noSlash = fn;
      if (noSlash.startsWith("/")) noSlash = noSlash.substring(1);
      deleted = LittleFS.remove(noSlash.c_str());
    }
    if (deleted) filesErased++;
    else errors++;
    delay(50);
  }

  configMap.clear(); lastSendTime.clear(); lastSendValue.clear();
  lastSentTimes.clear(); lastStorageTime.clear(); lastStoredValue.clear();
  lastStoredTimes.clear(); dataBuffer.clear(); lastBufferUpdate.clear();
  storedataBuffer.clear(); storelastBufferUpdate.clear(); topicValueMap.clear();

  char finalMsg[250];
  snprintf(finalMsg, sizeof(finalMsg), "COMPLETE: Erased=%d, Skipped=%d, Errors=%d", filesErased, filesSkipped, errors);
  publishMessage(CONFIG_RESPONSE_TOPIC, finalMsg, 0);
}

// Erase config for a single topic
void eraseTopicConfig(const String& topic) {
  Serial.printf("[WARNING] Erasing config for topic: %s\n", topic.c_str());
  String cleanTopic = topic;
  cleanTopic.replace("/configs", "");
  String filename = encodeTopicToFilename(topic);

  if (!LittleFS.exists(filename.c_str())) {
    char msg[200];
    snprintf(msg, sizeof(msg), "ERROR: Config file not found for topic %s", cleanTopic.c_str());
    publishMessage(CONFIG_RESPONSE_TOPIC, msg, 0);
    return;
  }

  if (LittleFS.remove(filename.c_str())) {
    configMap.erase(cleanTopic);
    lastSendTime.erase(cleanTopic); lastSendValue.erase(cleanTopic);
    lastSentTimes.erase(cleanTopic); lastStorageTime.erase(cleanTopic);
    lastStoredValue.erase(cleanTopic); lastStoredTimes.erase(cleanTopic);
    dataBuffer.erase(cleanTopic); lastBufferUpdate.erase(cleanTopic);
    storedataBuffer.erase(cleanTopic); storelastBufferUpdate.erase(cleanTopic);
    topicValueMap.erase(cleanTopic);

    char msg[200];
    snprintf(msg, sizeof(msg), "COMPLETE: Erased config for %s", cleanTopic.c_str());
    publishMessage(CONFIG_RESPONSE_TOPIC, msg, 0);
  } else {
    char msg[200];
    snprintf(msg, sizeof(msg), "ERROR: Failed to delete file for topic %s", cleanTopic.c_str());
    publishMessage(CONFIG_RESPONSE_TOPIC, msg, 0);
  }
}

// List all topic config files via MQTT
void listTopicConfigs() {
  publishMessage(CONFIG_RESPONSE_TOPIC, "STARTING: List all topic configs", 0);
  delay(100);

  int mapCount = 0;
  for (const auto& pair : configMap) {
    mapCount++;
    char msg[200];
    snprintf(msg, sizeof(msg), "MAP [%d]: %s (%d bytes)", mapCount, pair.first.c_str(), pair.second.length());
    publishMessage(CONFIG_RESPONSE_TOPIC, msg, 0);
    delay(50);
  }

  String excludedFiles[] = {"/config.json", "config.json"};
  size_t excludedCount = 2;
  File root = LittleFS.open("/");
  if (!root || !root.isDirectory()) {
    publishMessage(CONFIG_RESPONSE_TOPIC, "ERROR: Cannot access filesystem", 0);
    return;
  }

  int fileCount = 0;
  DynamicJsonDocument doc(4096);
  JsonArray configs = doc.createNestedArray("configs");

  File file = root.openNextFile();
  while (file) {
    String filename = String(file.name());
    size_t fileSize = file.size();
    bool isDir = file.isDirectory();
    file.close();
    if (isDir) { file = root.openNextFile(); continue; }

    bool isExcluded = false;
    for (size_t i = 0; i < excludedCount; i++) {
      if (filename.equals(excludedFiles[i])) { isExcluded = true; break; }
    }
    if (isExcluded) { file = root.openNextFile(); continue; }

    if (filename.endsWith(".json") && !filename.endsWith(".tmp") && !filename.endsWith(".bak")) {
      String topic = decodeFilenameToTopic(filename);
      fileCount++;
      JsonObject configObj = configs.createNestedObject();
      configObj["file"] = filename;
      configObj["topic"] = topic;
      configObj["size"] = fileSize;
    }
    file = root.openNextFile();
  }

  char summary[150];
  snprintf(summary, sizeof(summary), "COMPLETE: %d config files found", fileCount);
  publishMessage(CONFIG_RESPONSE_TOPIC, summary, 0);

  if (fileCount > 0) {
    String response;
    serializeJson(doc, response);
    publishMessage(CONFIG_RESPONSE_TOPIC, response.c_str(), 0);
  }
}

// ══════════════════════════════════════════════════════
//  MQTT CALLBACK (message handler)
// ══════════════════════════════════════════════════════

void callback(String &topic, String &payload) {
  Serial.println("========================================");
  Serial.print("[CALLBACK] Topic: ["); Serial.print(topic); Serial.println("]");
  String message = payload;
  Serial.print("[CALLBACK] Payload: "); Serial.println(message);

  String debugTopic = topic + "/debug";
  String debugMessage = "Received on [" + topic + "]: " + message;
  publishMessage(debugTopic.c_str(), debugMessage.c_str(), 0);

  // FIRMWARE UPDATE
  if (topic == FIRMWARE_TOPIC) {
    Serial.println("[INFO] Firmware update requested");
    disconnectMQTT();
    handleFirmwareUpdate(payload);
    return;
  }

  // MAIN CONFIG TOPIC
  if (topic == CONFIG_TOPIC) {
    publishMessage(CONFIG_RESPONSE_TOPIC, "configure entered", 0);

    if (payload == "0") {
      if (!LittleFS.exists("/config.json")) {
        publishMessage(CONFIG_RESPONSE_TOPIC, "ERROR: config.json not found", 0);
        return;
      }
      File configFile = LittleFS.open("/config.json", "r");
      if (!configFile) {
        publishMessage(CONFIG_RESPONSE_TOPIC, "ERROR: Failed to read config.json", 0);
        return;
      }
      String configData = configFile.readString();
      configFile.close();
      publishMessage(CONFIG_RESPONSE_TOPIC, configData.c_str(), 0);
      return;
    }
    else if (payload == "LIST_CONFIGS") { listTopicConfigs(); return; }
    else if (payload == "ERASE_ALL_CONFIGS") { eraseAllTopicConfigs(); publishMessage(CONFIG_RESPONSE_TOPIC, "erased", 0); return; }
    else if (payload.startsWith("ERASE:")) {
      String topicToErase = payload.substring(6);
      topicToErase.trim();
      eraseTopicConfig(topicToErase);
      return;
    }
    else {
      DynamicJsonDocument testDoc(2048);
      DeserializationError error = deserializeJson(testDoc, payload);
      if (error) {
        publishMessage(CONFIG_RESPONSE_TOPIC, "ERROR: Invalid JSON format", 0);
        return;
      }
      if (LittleFS.exists("/config.json")) LittleFS.remove("/config.json");
      File configFile = LittleFS.open("/config.json", "w");
      if (!configFile) {
        publishMessage(CONFIG_RESPONSE_TOPIC, "ERROR: Failed to write config.json", 0);
        return;
      }
      size_t bytesWritten = configFile.print(payload);
      configFile.close();
      if (bytesWritten != payload.length()) {
        publishMessage(CONFIG_RESPONSE_TOPIC, "ERROR: Incomplete write", 0);
        return;
      }
      publishMessage(CONFIG_RESPONSE_TOPIC, "SUCCESS: Config updated, restarting...", 0);
      delay(2000);
      ESP.restart();
    }
    return;
  }

  // TOPIC-SPECIFIC CONFIG
  if (topic.endsWith("/configs")) {
    DynamicJsonDocument testDoc(2048);
    DeserializationError error = deserializeJson(testDoc, message);
    if (error) {
      String responseTopic = topic + "/response";
      publishMessage(responseTopic.c_str(), "ERROR: Invalid JSON format", 0);
      return;
    }
    saveConfigToFile(message, topic);
    String responseTopic = topic + "/response";
    char responseMsg[150];
    snprintf(responseMsg, sizeof(responseMsg), "SUCCESS: Config saved for %s (%d bytes)", topic.c_str(), message.length());
    publishMessage(responseTopic.c_str(), responseMsg, 0);
    return;
  }

  // MODBUS WRITE COMMAND
  int topicIndex = -1;
  for (int i = 0; i < count1; i++) {
    if (String(topic) == splitmqttaddwrite[i]) {
      topicIndex = i;
      bool boolvalue = false;
      int16_t intvalue = 0;
      float floatvalue = 0.0;
      long longvalue = 0;

      String splitwrite[6];
      char spccar = ',';
      int splitCount = splitString(substringsWRITE[topicIndex], spccar, splitwrite);

      if (splitwrite[4] == "int16") intvalue = message.toInt();
      else if (splitwrite[4] == "int32") longvalue = message.toInt();
      else if (splitwrite[4] == "bool") boolvalue = (message.equalsIgnoreCase("true") || message.equalsIgnoreCase("yes") || message == "1");
      else if (splitwrite[4] == "float") floatvalue = message.toFloat();

      // RS485
      if (strcmp(protocolType, "rs485") == 0) {
        bool result = false;
        uint16_t regValues[2] = {0, 0};
        if (splitwrite[1].toInt() == 5) {
          result = writeModbusRTUDirect(splitwrite[0].toInt(), 5, splitwrite[2].toInt(), nullptr, 0, boolvalue);
        } else if (splitwrite[1].toInt() == 6) {
          if (splitwrite[4] == "int16") {
            regValues[0] = (uint16_t)intvalue;
            result = writeModbusRTUDirect(splitwrite[0].toInt(), 6, splitwrite[2].toInt(), regValues, 1, false);
          } else if (splitwrite[4] == "int32") {
            split32BitTo16Bit(static_cast<uint32_t>(longvalue), regValues);
            result = writeModbusRTUDirect(splitwrite[0].toInt(), 16, splitwrite[2].toInt(), regValues, 2, false);
          } else if (splitwrite[4] == "float") {
            floatTo16Bit(floatvalue, regValues);
            result = writeModbusRTUDirect(splitwrite[0].toInt(), 16, splitwrite[2].toInt(), regValues, 2, false);
          }
        }
        Serial.printf("[MODBUS RTU] Write %s\n", result ? "OK" : "FAILED");
        return;
      }

      // TCP WiFi
      else if (strcmp(protocolType, "tcpwifi") == 0) {
        IPAddress slaveIP;
        if (!slaveIP.fromString(splitwrite[0])) return;
        if (!modbusTCPWiFi.isConnected(slaveIP)) {
          if (!modbusTCPWiFi.connect(slaveIP)) return;
        }
        uint16_t transId = 0;
        if (splitwrite[1].toInt() == 5) {
          transId = modbusTCPWiFi.writeCoil(slaveIP, splitwrite[2].toInt(), boolvalue);
        } else if (splitwrite[1].toInt() == 6) {
          uint16_t regValues[2];
          if (splitwrite[4] == "int16") transId = modbusTCPWiFi.writeHreg(slaveIP, splitwrite[2].toInt(), intvalue);
          else if (splitwrite[4] == "int32") { split32BitTo16Bit(static_cast<uint32_t>(longvalue), regValues); transId = modbusTCPWiFi.writeHreg(slaveIP, splitwrite[2].toInt(), regValues, 2); }
          else if (splitwrite[4] == "float") { floatTo16Bit(floatvalue, regValues); transId = modbusTCPWiFi.writeHreg(slaveIP, splitwrite[2].toInt(), regValues, 2); }
        }
        unsigned long startWait = millis();
        while (modbusTCPWiFi.isTransaction(transId)) { modbusTCPWiFi.task(); delay(10); if (millis() - startWait > 5000) break; }
        return;
      }

      // TCP Ethernet
      else if (strcmp(protocolType, "tcpethernet") == 0) {
        IPAddress slaveIP;
        if (!slaveIP.fromString(splitwrite[0])) return;
        if (!modbusTCPEthernet.isConnected(slaveIP)) {
          if (!modbusTCPEthernet.connect(slaveIP)) return;
        }
        uint16_t transId = 0;
        if (splitwrite[1].toInt() == 5) {
          transId = modbusTCPEthernet.writeCoil(slaveIP, splitwrite[2].toInt(), boolvalue);
        } else if (splitwrite[1].toInt() == 6) {
          uint16_t regValues[2];
          if (splitwrite[4] == "int16") transId = modbusTCPEthernet.writeHreg(slaveIP, splitwrite[2].toInt(), intvalue);
          else if (splitwrite[4] == "int32") { split32BitTo16Bit(static_cast<uint32_t>(longvalue), regValues); transId = modbusTCPEthernet.writeHreg(slaveIP, splitwrite[2].toInt(), regValues, 2); }
          else if (splitwrite[4] == "float") { floatTo16Bit(floatvalue, regValues); transId = modbusTCPEthernet.writeHreg(slaveIP, splitwrite[2].toInt(), regValues, 2); }
        }
        unsigned long startWait = millis();
        while (modbusTCPEthernet.isTransaction(transId)) { modbusTCPEthernet.task(); delay(10); if (millis() - startWait > 5000) break; }
        return;
      }
      break;
    }
  }

  Serial.println("[DEBUG] Topic not matched to any handler");
  Serial.println("========================================");
}


// ══════════════════════════════════════════════════════
//  MODBUS RTU DIRECT — Serial2 helpers
// ══════════════════════════════════════════════════════

uint16_t modbusRTUCRC(uint8_t* buf, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
      else crc >>= 1;
    }
  }
  return crc;
}

void preTransmission()
{
  digitalWrite(DIR, LOW);
  delayMicroseconds(100);
}

void postTransmission()
{
  Serial2.flush();
  delayMicroseconds(300);
  digitalWrite(DIR, HIGH);
}

// Returns true on success; fills regs[] for FC3/FC4, bools[] for FC1/FC2
bool readModbusRTUDirect(uint8_t slaveId, uint8_t fc, uint16_t addr, uint16_t count, uint16_t* regs, bool* bools) {
  uint8_t req[8];
  req[0] = slaveId;
  req[1] = fc;
  req[2] = (addr >> 8) & 0xFF;
  req[3] = addr & 0xFF;
  req[4] = (count >> 8) & 0xFF;
  req[5] = count & 0xFF;
  uint16_t crc = modbusRTUCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = (crc >> 8) & 0xFF;

  while (Serial2.available()) Serial2.read();  // flush stale bytes from previous reads
  preTransmission();
  Serial2.write(req, 8);
  postTransmission();

  // Print TX frame for verification
  Serial.printf("[MODBUS RTU] TX: ");
  for (int i = 0; i < 8; i++) Serial.printf("%02X ", req[i]);
  Serial.println();

  uint16_t expectedBytes = (fc == 1 || fc == 2) ? (5 + ((count + 7) / 8)) : (5 + count * 2);
  Serial.printf("[MODBUS RTU] waiting for %d bytes, fifo=%d\n", expectedBytes, Serial2.available());
  uint8_t resp[256];
  uint16_t rLen = 0;
  // After delay(20) most bytes are already in FIFO; use 3 ms inter-byte gap to stop reading.
  // Deadline is 1000 ms to handle WiFi/MQTT stretching the delay() call beyond 200 ms.
  unsigned long deadline = millis() + 1000;
  unsigned long lastByteTime = millis();
  while (millis() < deadline && rLen < sizeof(resp)) {
    if (Serial2.available()) {
      resp[rLen++] = Serial2.read();
      lastByteTime = millis();
    } else if (rLen > 0 && (millis() - lastByteTime) >= 3) {
      break;  // 3 ms silence: no more bytes coming
    }
  }

  // Print whatever was received
  if (rLen > 0) {
    Serial.printf("[MODBUS RTU] RX[%d]: ", rLen);
    for (uint16_t i = 0; i < rLen; i++) Serial.printf("%02X ", resp[i]);
    Serial.println();
  }
  Serial.printf("[MODBUS RTU] FC%d slave=%d addr=%d count=%d  got=%d expected=%d\n", fc, slaveId, addr, count, rLen, expectedBytes);

  // Scan for a valid Modbus frame: find [slaveId][fc] and confirm with CRC.
  // This handles TX echo, gap bytes between echo and response, and trailing noise robustly.
  uint8_t* frame = nullptr;
  for (uint16_t i = 0; i + expectedBytes <= rLen; i++) {
    if (resp[i] != slaveId || resp[i + 1] != fc) continue;
    uint16_t calcCRC = modbusRTUCRC(resp + i, expectedBytes - 2);
    uint16_t recvCRC = (uint16_t)resp[i + expectedBytes - 2] | ((uint16_t)resp[i + expectedBytes - 1] << 8);
    if (calcCRC == recvCRC) { frame = resp + i; break; }
  }
  if (!frame) {
    // Also check for exception response (5 bytes: slaveId, fc|0x80, code, CRC×2)
    for (uint16_t i = 0; i + 5 <= rLen; i++) {
      if (resp[i] != slaveId || resp[i + 1] != (fc | 0x80)) continue;
      uint16_t calcCRC = modbusRTUCRC(resp + i, 3);
      uint16_t recvCRC = (uint16_t)resp[i + 3] | ((uint16_t)resp[i + 4] << 8);
      if (calcCRC == recvCRC) { Serial.printf("[MODBUS RTU] Exception code: 0x%02X\n", resp[i + 2]); return false; }
    }
    Serial.println("[MODBUS RTU] No valid frame found");
    return false;
  }

  if (fc == 1 || fc == 2) {
    for (uint16_t i = 0; i < count; i++) bools[i] = (frame[3 + i / 8] >> (i % 8)) & 0x01;
  } else {
    for (uint16_t i = 0; i < count; i++) regs[i] = ((uint16_t)frame[3 + i * 2] << 8) | frame[4 + i * 2];
  }
  return true;
}

// Returns true on success; fc=5 uses boolVal, fc=6 uses regs[0], fc=16 uses regs[0..regCount-1]
bool writeModbusRTUDirect(uint8_t slaveId, uint8_t fc, uint16_t addr, uint16_t* regs, uint8_t regCount, bool boolVal) {
  uint8_t req[256];
  uint8_t reqLen = 0;
  req[reqLen++] = slaveId;
  req[reqLen++] = fc;
  req[reqLen++] = (addr >> 8) & 0xFF;
  req[reqLen++] = addr & 0xFF;

  if (fc == 5) {
    uint16_t v = boolVal ? 0xFF00 : 0x0000;
    req[reqLen++] = (v >> 8) & 0xFF;
    req[reqLen++] = v & 0xFF;
  } else if (fc == 6) {
    req[reqLen++] = (regs[0] >> 8) & 0xFF;
    req[reqLen++] = regs[0] & 0xFF;
  } else if (fc == 16) {
    req[reqLen++] = (regCount >> 8) & 0xFF;
    req[reqLen++] = regCount & 0xFF;
    req[reqLen++] = regCount * 2;
    for (uint8_t i = 0; i < regCount; i++) {
      req[reqLen++] = (regs[i] >> 8) & 0xFF;
      req[reqLen++] = regs[i] & 0xFF;
    }
  }
  uint16_t crc = modbusRTUCRC(req, reqLen);
  req[reqLen++] = crc & 0xFF;
  req[reqLen++] = (crc >> 8) & 0xFF;

  while (Serial2.available()) Serial2.read();
  preTransmission();
  Serial2.write(req, reqLen);
  postTransmission();

  uint8_t resp[16];
  uint8_t rLen = 0;
  unsigned long deadline = millis() + 1000;
  while (millis() < deadline) {
    while (Serial2.available() && rLen < sizeof(resp)) resp[rLen++] = Serial2.read();
    if (rLen >= 8) break;
    delay(1);
  }

  Serial.printf("[MODBUS RTU] Write FC%d slave=%d addr=%d  got=%d bytes\n", fc, slaveId, addr, rLen);
  if (rLen < 6) return false;
  if (resp[0] != slaveId) return false;
  if (resp[1] & 0x80) { Serial.printf("[MODBUS RTU] Write exception: 0x%02X\n", resp[2]); return false; }
  uint16_t respCRC = modbusRTUCRC(resp, rLen - 2);
  uint16_t recvCRC = (uint16_t)resp[rLen - 2] | ((uint16_t)resp[rLen - 1] << 8);
  return respCRC == recvCRC;
}

// ══════════════════════════════════════════════════════
//  GROUPED DEVICE DATA PUBLISHING (JSON per device every 10s)
// ══════════════════════════════════════════════════════

// Store a single register value for a device
void storeGroupedValue(uint8_t deviceId, uint16_t address, float value) {
  Serial.printf("[GROUPED-STORE] Device=%d, Addr=%u, Value=%.2f (useGroupedPublishing=%s, baseTopic='%s')\n", 
    deviceId, address, value, useGroupedPublishing ? "true" : "false", baseTopicPath);
  deviceRegisterValues[deviceId][address] = value;
  Serial.printf("[GROUPED-STORE] Map size: %d devices, Device %d now has %d registers\n", 
    deviceRegisterValues.size(), deviceId, deviceRegisterValues[deviceId].size());
}

// Publish all grouped device data as concatenated string values
// Format: address1:value1,address2:value2,address3:value3,...
// Topic format: baseTopicPath/deviceId
void publishGroupedDeviceData() {
  Serial.printf("[GROUPED-PUBLISH] Called. useGroupedPublishing=%s, baseTopicPath='%s', mqttMethod=%s\n", 
    useGroupedPublishing ? "true" : "false", baseTopicPath, mqttMethod);
  
  if (!useGroupedPublishing) {
    Serial.println("[GROUPED-PUBLISH] SKIPPED: useGroupedPublishing is FALSE");
    return;
  }
  
  if (baseTopicPath[0] == '\0') {
    Serial.println("[GROUPED-PUBLISH] SKIPPED: baseTopicPath is EMPTY");
    return;
  }
  
  if (strcmp(mqttMethod, "wifi") != 0 && strcmp(mqttMethod, "ethernet") != 0 && strcmp(mqttMethod, "gsm") != 0) {
    Serial.printf("[GROUPED-PUBLISH] SKIPPED: Invalid mqttMethod '%s'\n", mqttMethod);
    return;
  }

  bool hasData = false;
  Serial.printf("[GROUPED-PUBLISH] Starting publish cycle. Total devices in map: %d\n", deviceRegisterValues.size());

  // Iterate over all devices
  for (auto& devicePair : deviceRegisterValues) {
    uint8_t deviceId = devicePair.first;
    auto& registerMap = devicePair.second;
    
    Serial.printf("[GROUPED-PUBLISH] Processing Device %d with %d registers\n", deviceId, registerMap.size());
    
    if (registerMap.empty()) {
      Serial.printf("[GROUPED-PUBLISH] Device %d is EMPTY, skipping\n", deviceId);
      continue;
    }
    
    hasData = true;
    
    // Build concatenated string of register values
    // Format: address1:value1,address2:value2,address3:value3
    char valueBuffer[2048] = {0};
    int bufferPos = 0;
    bool isFirst = true;
    
    Serial.printf("[GROUPED-PUBLISH] Building payload for Device %d: ", deviceId);
    for (auto& regPair : registerMap) {
      uint16_t address = regPair.first;
      float value = regPair.second;
      
      if (!isFirst) {
        bufferPos += snprintf(&valueBuffer[bufferPos], sizeof(valueBuffer) - bufferPos, ",");
      }
      
      // Format: address:value (e.g., 3110:123.45)
      bufferPos += snprintf(&valueBuffer[bufferPos], sizeof(valueBuffer) - bufferPos, "%u:%.2f", address, value);
      Serial.printf("[%u:%.2f] ", address, value);
      isFirst = false;
    }
    Serial.println();
    
    // Publish to topic: baseTopicPath/deviceId
    String topic = String(baseTopicPath) + "/" + String(deviceId);
    Serial.printf("[GROUPED-PUBLISH] Topic: %s, Payload: %s\n", topic.c_str(), valueBuffer);
    Serial.printf("[GROUPED-PUBLISH] Grouped MQTT Publishing Message: %s\n", valueBuffer);
    Serial.printf("[GROUPED-PUBLISH] Method=%s, WiFi=%s, Ethernet=%s, GSM=%s\n", 
      mqttMethod, 
      strcmp(mqttMethod, "wifi") == 0 ? "YES" : "NO",
      strcmp(mqttMethod, "ethernet") == 0 ? "YES" : "NO",
      strcmp(mqttMethod, "gsm") == 0 ? "YES" : "NO");
    
    if (strcmp(mqttMethod, "gsm") == 0) {
      if (gsmMQTTConnected) {
        Serial.println("[GROUPED-PUBLISH] Publishing via GSM...");
        sendMQTTViaGSM(topic.c_str(), valueBuffer);
        Serial.println("[GROUPED-PUBLISH] Published to Hive MQTT via GSM");
      } else {
        Serial.println("[GROUPED-PUBLISH] GSM selected but NOT connected!");
      }
    } else if (strcmp(mqttMethod, "wifi") == 0) {
      if (wifiClientPubSub.connected()) {
        Serial.println("[GROUPED-PUBLISH] Publishing via WiFi...");
        bool published = wifiClientPubSub.publish(topic.c_str(), valueBuffer, false, 0);
        Serial.printf("[GROUPED-PUBLISH] WiFi publish result: %s\n", published ? "SUCCESS" : "FAILED");
        if (published) {
          Serial.println("[GROUPED-PUBLISH] Published to Hive MQTT via WiFi");
        }
      } else {
        Serial.println("[GROUPED-PUBLISH] WiFi selected but NOT connected!");
      }
    } else if (strcmp(mqttMethod, "ethernet") == 0) {
      if (ethernetClientPubSub.connected()) {
        Serial.println("[GROUPED-PUBLISH] Publishing via Ethernet...");
        bool published = ethernetClientPubSub.publish(topic.c_str(), valueBuffer, false, 0);
        Serial.printf("[GROUPED-PUBLISH] Ethernet publish result: %s\n", published ? "SUCCESS" : "FAILED");
        if (published) {
          Serial.println("[GROUPED-PUBLISH] Published to Hive MQTT via Ethernet");
        }
      } else {
        Serial.println("[GROUPED-PUBLISH] Ethernet selected but NOT connected!");
      }
    }
    
    Serial.printf("[GROUPED] Published to %s: %s\n", topic.c_str(), valueBuffer);
  }

  if (!hasData) {
    Serial.println("[GROUPED-PUBLISH] No device data to publish");
  }
}

// ══════════════════════════════════════════════════════
//  MODBUS READ — RS485 (RTU)
// ══════════════════════════════════════════════════════

void readprocessModbusRequest(uint8_t slaveId, uint16_t functionCode, uint16_t address, uint16_t count, const String& dataType, const char* mqttTopic) {
  if (requestInProgress) return;
  requestInProgress = true;
  callbackResult = false;

  uint16_t* buffers = new uint16_t[count];
  bool* bufferbool = new bool[count];
  bool result = false;

  switch (functionCode) {
    case 1:
      result = readModbusRTUDirect(slaveId, 1, address, count, buffers, bufferbool);
      break;
    case 2:
      result = readModbusRTUDirect(slaveId, 2, address, count, buffers, bufferbool);
      break;
    case 3:
      result = readModbusRTUDirect(slaveId, 3, address, count, buffers, bufferbool);
      break;
    case 4:
      result = readModbusRTUDirect(slaveId, 4, address, count, buffers, bufferbool);
      break;
    default:
      delete[] buffers; delete[] bufferbool; requestInProgress = false; return;
  }

  if (!result) { delete[] buffers; delete[] bufferbool; requestInProgress = false; return; }
  callbackResult = true;  // direct read succeeded — data is valid

  char msg[512];
  if (dataType == "float" && count == 2) {
    float v = combineToFloat(buffers[0], buffers[1]);
    updateDisplaySensorValue(slaveId, address, v);
    snprintf(msg, sizeof(msg), "%f", v);
  }
  else if (dataType == "int16" && count == 1) { snprintf(msg, sizeof(msg), "%d", (int16_t)buffers[0]); }
  else if (dataType == "uint16" && count == 1) { snprintf(msg, sizeof(msg), "%u", buffers[0]); }
  else if (dataType == "int32" && count == 2) { snprintf(msg, sizeof(msg), "%ld", (long)combineToInt32(buffers[0], buffers[1])); }
  else if (dataType == "uint32" && count == 2) { snprintf(msg, sizeof(msg), "%lu", (unsigned long)combineToUint32(buffers[0], buffers[1])); }
  else if (dataType == "bool" && count == 1) { snprintf(msg, sizeof(msg), "%s", (buffers[0] != 0) ? "true" : "false"); }
  else if (dataType == "ascii") {
    String asciiString = "";
    for (uint16_t i = 0; i < count; i++) { asciiString += (char)((buffers[i] >> 8) & 0xFF); asciiString += (char)(buffers[i] & 0xFF); }
    asciiString.trim();
    snprintf(msg, sizeof(msg), "%s", asciiString.c_str());
  } else { snprintf(msg, sizeof(msg), "Unsupported data type"); }

  int qos = 0;
  int payloadLength = strlen(msg);
 

  if (callbackResult) {
    processNotificationWithWait(mqttTopic, msg, dataType);
    processStorageWithWait(mqttTopic, msg, dataType);
    topicValueMap[mqttTopic] = msg;

    // Store grouped value if grouped publishing is enabled
    if (useGroupedPublishing && dataType == "float") {
      float floatVal = strtof(msg, nullptr);
      storeGroupedValue(slaveId, address, floatVal);
    }

    if (strcmp(mqttMethod, "gsm") == 0) {
        if (gsmMQTTConnected) {
            sendMQTTViaGSM(mqttTopic, msg);
        } else reconnect();
    } else if (strcmp(mqttMethod, "wifi") == 0) {
      if (wifiClientPubSub.connected()) wifiClientPubSub.publish(mqttTopic, msg, payloadLength, false, qos);
      else reconnect();
    } else if (strcmp(mqttMethod, "ethernet") == 0) {
      if (ethernetClientPubSub.connected()) ethernetClientPubSub.publish(mqttTopic, msg, payloadLength, false, qos);
      else reconnect();
    }
  }

  delete[] buffers; delete[] bufferbool;
  requestInProgress = false;
}

// ══════════════════════════════════════════════════════
//  MODBUS READ — TCP over WiFi
// ══════════════════════════════════════════════════════

void readprocessModbusRequestTCPWiFi(const IPAddress& slaveIP, uint16_t functionCode, uint16_t address, uint16_t count, const String& dataType, const char* mqttTopic) {
  if (requestInProgress) return;
  requestInProgress = true;
  callbackResult = false;

  if (!modbusTCPWiFi.isConnected(slaveIP)) {
    if (!modbusTCPWiFi.connect(slaveIP)) { requestInProgress = false; return; }
  }

  uint16_t* buffers = new uint16_t[count];
  bool* bufferbool = new bool[count];
  uint16_t transId = 0;

  switch (functionCode) {
    case 1: transId = modbusTCPWiFi.readCoil(slaveIP, address, bufferbool, count, cbWrite); break;
    case 2: transId = modbusTCPWiFi.readIsts(slaveIP, address, bufferbool, count, cbWrite); break;
    case 3: transId = modbusTCPWiFi.readHreg(slaveIP, address, buffers, count, cbWrite); break;
    case 4: transId = modbusTCPWiFi.readIreg(slaveIP, address, buffers, count, cbWrite); break;
    default: delete[] buffers; delete[] bufferbool; requestInProgress = false; return;
  }

  while (modbusTCPWiFi.isTransaction(transId)) { modbusTCPWiFi.task(); delay(10); }

  if (transId == 0) {
    modbusTCPWiFi.disconnect(slaveIP);
    delete[] buffers; delete[] bufferbool; requestInProgress = false; return;
  }

  char msg[50];
  if (dataType == "float" && count == 2) {
    float v = combineToFloat(buffers[0], buffers[1]);
    updateDisplaySensorValue(0, address, v);
    snprintf(msg, sizeof(msg), "%f", v);
  }
  else if (dataType == "int16" && count == 1) snprintf(msg, sizeof(msg), "%d", (int16_t)buffers[0]);
  else if (dataType == "uint16" && count == 1) snprintf(msg, sizeof(msg), "%u", buffers[0]);
  else if (dataType == "int32" && count == 2) snprintf(msg, sizeof(msg), "%ld", (long)combineToInt32(buffers[0], buffers[1]));
  else if (dataType == "uint32" && count == 2) snprintf(msg, sizeof(msg), "%lu", (unsigned long)combineToUint32(buffers[0], buffers[1]));
  else if (dataType == "bool" && count == 1) snprintf(msg, sizeof(msg), "%s", (buffers[0] != 0) ? "true" : "false");
  else snprintf(msg, sizeof(msg), "Unsupported");

  unsigned long currentMillis = millis();
  if (String(mqttTopic).endsWith("/str")) {
    if (lastPublishTimes.find(mqttTopic) != lastPublishTimes.end() && (currentMillis - lastPublishTimes[mqttTopic]) < 1800000) {
      delete[] buffers; delete[] bufferbool; requestInProgress = false; return;
    }
  }

  if (callbackResult) {
    publishMessage(mqttTopic, msg, 0);
    if (String(mqttTopic).endsWith("/str")) lastPublishTimes[mqttTopic] = currentMillis;
  }

  delete[] buffers; delete[] bufferbool;
  requestInProgress = false;
}

// ══════════════════════════════════════════════════════
//  MODBUS READ — TCP over Ethernet
// ══════════════════════════════════════════════════════

void readprocessModbusRequestTCPEthernet(const IPAddress& slaveIP, uint16_t functionCode, uint16_t address, uint16_t count, const String& dataType, const char* mqttTopic) {
  if (requestInProgress) return;
  requestInProgress = true;
  callbackResult = false;

  if (!modbusTCPEthernet.isConnected(slaveIP)) {
    if (!modbusTCPEthernet.connect(slaveIP)) { requestInProgress = false; return; }
    addSlaveIPToList(slaveIP);
  }

  uint16_t* buffers = new uint16_t[count];
  bool* bufferbool = new bool[count];
  uint16_t transId = 0;

  switch (functionCode) {
    case 1: transId = modbusTCPEthernet.readCoil(slaveIP, address, bufferbool, count, cbWrite); break;
    case 2: transId = modbusTCPEthernet.readIsts(slaveIP, address, bufferbool, count, cbWrite); break;
    case 3: transId = modbusTCPEthernet.readHreg(slaveIP, address, buffers, count, cbWrite); break;
    case 4: transId = modbusTCPEthernet.readIreg(slaveIP, address, buffers, count, cbWrite); break;
    default: delete[] buffers; delete[] bufferbool; requestInProgress = false; return;
  }

  while (modbusTCPEthernet.isTransaction(transId)) { modbusTCPEthernet.task(); delay(10); }

  if (transId == 0) {
    modbusTCPEthernet.disconnect(slaveIP);
    delete[] buffers; delete[] bufferbool; requestInProgress = false; return;
  }

  char msg[50];
  if (dataType == "float" && count == 2) {
    float v = combineToFloat(buffers[0], buffers[1]);
    updateDisplaySensorValue(0, address, v);
    snprintf(msg, sizeof(msg), "%f", v);
  }
  else if (dataType == "int16" && count == 1) snprintf(msg, sizeof(msg), "%d", (int16_t)buffers[0]);
  else if (dataType == "uint16" && count == 1) snprintf(msg, sizeof(msg), "%u", buffers[0]);
  else if (dataType == "int32" && count == 2) snprintf(msg, sizeof(msg), "%ld", (long)combineToInt32(buffers[0], buffers[1]));
  else if (dataType == "uint32" && count == 2) snprintf(msg, sizeof(msg), "%lu", (unsigned long)combineToUint32(buffers[0], buffers[1]));
  else if (dataType == "bool" && count == 1) snprintf(msg, sizeof(msg), "%s", (buffers[0] != 0) ? "true" : "false");
  else snprintf(msg, sizeof(msg), "Unsupported");

  String topicStr = String(mqttTopic);
  if (topicStr.endsWith("/str")) {
    unsigned long currentTime = millis();
    if (lastPublishTimes.find(topicStr) != lastPublishTimes.end() && (currentTime - lastPublishTimes[topicStr]) < 1800000) {
      delete[] buffers; delete[] bufferbool; requestInProgress = false; return;
    }
    lastPublishTimes[topicStr] = currentTime;
  }

  if (callbackResult) {
    publishMessage(mqttTopic, msg, 0);
  }

  delete[] buffers; delete[] bufferbool;
  requestInProgress = false;
}

// ══════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════

String deviceID;  // Global device ID

void setup() {
  Serial.begin(115200);
  Serial.printf("[HEAP] Boot start: %u bytes free, min ever: %u bytes\n", ESP.getFreeHeap(), ESP.getMinFreeHeap());
  deviceID = getStaticDeviceID();
  Serial.print("deviceID: "); Serial.println(deviceID);

  // ── Early reset check (runs before any network/GSM init) ──────────────
  // GPIO33 held LOW on boot → clear all config and open portal.
  // This works even in GSM mode where loop() is never reached.
  pinMode(WIFI_RESET_PIN, INPUT_PULLUP);
  delay(100);  // let pin settle
  if (skipConfigResetAfterClear) {
    ignoreConfigResetUntilRelease = true;
    skipConfigResetAfterClear = false;
    Serial.println("[RESET] GPIO33 still LOW after config clear; ignoring until release.");
  } else if (configResetPressed()) {
    Serial.println("[RESET] GPIO33 held on boot. Clearing config...");
    WiFi.mode(WIFI_STA);
    WiFiManager wmReset;
    wmReset.resetSettings();
    if (LittleFS.begin(true)) {
      LittleFS.end();
      LittleFS.format();
      Serial.println("[RESET] LittleFS formatted.");
    }
    skipConfigResetAfterClear = true;
    Serial.println("[RESET] Done. Rebooting into portal...");
    delay(500);
    ESP.restart();
  }
  // ─────────────────────────────────────────────────────────────────────

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Mount LittleFS
  Serial.println("mounting FS...");
  Serial.println("firmware version 1.13");
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
    return;
  }
  Serial.println("mounted LittleFS filesystem");
  Serial.printf("[HEAP] After LittleFS mount: %u bytes\n", ESP.getFreeHeap());

  // Load config.json
  if (LittleFS.exists("/config.json")) {
    Serial.println("reading config file");
    File configFile = LittleFS.open("/config.json", "r", false);
    if (configFile) {
      size_t size = configFile.size();
      std::unique_ptr<char[]> buf(new char[size]);
      configFile.readBytes(buf.get(), size);
      configFile.close();

      DynamicJsonDocument json(2048);
      auto deserializeError = deserializeJson(json, buf.get());
      if (!deserializeError) {
        Serial.println("parsed json");
        strcpy(mqttserver, json["mqttserver"]);
        strcpy(mqttport, json["mqttport"]);
        strcpy(mqttusername, json["mqttusername"]);
        strcpy(mqttpassword, json["mqttpassword"]);
        strcpy(mqttMethod, json["mqttMethod"]);
        strcpy(ethMacAddr, json["ethMacAddr"]);
        strcpy(staticIP, json["staticIP"]);
        strcpy(gatewayIPs, json["gatewayIP"]);
        strcpy(subnetMasks, json["subnetMask"]);
        strcpy(ipMode, json["ipMode"]);
        strcpy(apn, json["apn"]);
        strcpy(gprsUser, json["gprsUser"]);
        strcpy(gprsPass, json["gprsPass"]);
        strcpy(protocolType, json["protocolType"]);
        strcpy(intervalssParam, json["intervalss"]);
        strcpy(slavebaudrate, json["slavebaudrate"]);
        strcpy(slavedatalength, json["slavedatalength"]);
        strcpy(slaveparity, json["slaveparity"]);
        strcpy(slavestopbit, json["slavestopbit"]);
        strcpy(slaveconfigREAD, json["slaveconfigREAD"]);
        strcpy(slaveconfigWRITE, json["slaveconfigWRITE"]);
        
        // Load baseTopicPath from JSON (THIS WAS MISSING!)
        if (json.containsKey("baseTopicPath")) {
          strcpy(baseTopicPath, json["baseTopicPath"]);
          Serial.printf("[CONFIG-LOAD] baseTopicPath loaded from JSON: '%s'\n", baseTopicPath);
        } else {
          Serial.println("[CONFIG-LOAD] baseTopicPath NOT found in JSON (field missing)");
          baseTopicPath[0] = '\0';
        }
        
        printStoredConfig();
      } else {
        Serial.println("failed to load json config");
      }
    }
  } else {
    Serial.println("/config.json does not exist. Using macros.");
  }

  // FORCE SYNC: Ensure new macros take precedence over STALE saved config
  if (strcmp(mqttserver, DEFAULT_MQTT_SERVER) != 0 ||
      strcmp(mqttusername, DEFAULT_MQTT_USER) != 0) {
      Serial.println("[CONFIG] Overriding saved MQTT config with new macros...");
      strcpy(mqttserver, DEFAULT_MQTT_SERVER);
      strcpy(mqttusername, DEFAULT_MQTT_USER);
      strcpy(mqttpassword, DEFAULT_MQTT_PASS);
      shouldSaveConfig = true; // Save this to FS
  }

  intervalss = strtoul(intervalssParam, NULL, 10);

  // Reset button check (existing — GPIO 25)
  pinMode(resetbutton, INPUT_PULLUP);
  if (digitalRead(resetbutton) == LOW) {
    wm.resetSettings();
    clearLittleFS();
    if (LittleFS.begin()) {
      if (LittleFS.exists("/config.json")) LittleFS.remove("/config.json");
      LittleFS.end();
    }
    ESP.restart();
  }

  // WiFi config reset button (GPIO 33) — hold LOW on boot to clear WiFi + config
  pinMode(WIFI_RESET_PIN, INPUT_PULLUP);
  if (configResetPressed()) {
    Serial.println("[RESET] WiFi reset button (GPIO 33) held — clearing WiFi config...");
    wm.resetSettings();
    if (LittleFS.begin()) {
      if (LittleFS.exists("/config.json")) {
        LittleFS.remove("/config.json");
        Serial.println("[RESET] config.json deleted.");
      }
      LittleFS.end();
    }
    skipConfigResetAfterClear = true;
    Serial.println("[RESET] Restarting...");
    delay(500);
    ESP.restart();
  }

  initDisplay();

  // Init GSM UART
  SerialAT.begin(115200, SERIAL_8N1, GSM_RX, GSM_TX);  // NEW: A7672S pins
  delay(1000);
  Serial.println("Sending AT command...");
  SerialAT.println("AT");
  Serial.setDebugOutput(false);  // Disable WiFi stack noise — keeps serial clean
  delay(500);

  // WiFiManager parameters
  WiFiManagerParameter customoutput("Mqttserver", "mqtt server IP host", mqttserver, 40);
  WiFiManagerParameter customoutputa("mqttport", "mqtt port", mqttport, 6);
  WiFiManagerParameter customoutputb("Mqttusername", "mqtt server username", mqttusername, 40);
  WiFiManagerParameter customoutputc("Mqttpassword", "mqtt server password", mqttpassword, 40);
  WiFiManagerParameter custommqttMethod("mqttMethod", "MQTT via wifi or gsm or ethernet", mqttMethod, 10);
  WiFiManagerParameter customMacParam("ethMacAddr", "Ethernet MAC Address", ethMacAddr, 18);
  WiFiManagerParameter customStaticIPParam("staticIP", "Static IP Address", staticIP, 16);
  WiFiManagerParameter customGatewayIPParam("gatewayIP", "Gateway IP Address", gatewayIPs, 16);
  WiFiManagerParameter customSubnetMaskParam("subnetMask", "Subnet Mask", subnetMasks, 16);
  WiFiManagerParameter customIPModeParam("ipMode", "IP Mode (Auto/Manual)", ipMode, 10);
  WiFiManagerParameter customAPN("apn", "GSM APN", apn, 40);
  WiFiManagerParameter customGprsUser("gprsUser", "GPRS Username", gprsUser, 40);
  WiFiManagerParameter customGprsPass("gprsPass", "GPRS Password", gprsPass, 40);
  WiFiManagerParameter customProtocol("protocolType", "Protocol (rs485/tcpwifi/tcpethernet)", protocolType, 20);
  WiFiManagerParameter customIntervalss("intervalss", "Modbus Interval (ms)", intervalssParam, 10);
  WiFiManagerParameter customoutputd("RS485Baudrate", "system Baudrate", slavebaudrate, 40);
  WiFiManagerParameter customoutpute("RS485DataLength", "Data Length", slavedatalength, 40);
  WiFiManagerParameter customoutputf("RS485parity", "parity E or N or O", slaveparity, 40);
  WiFiManagerParameter customoutputg("RS485Stopbit", "Stopbit", slavestopbit, 40);
  WiFiManagerParameter customoutputREAD("slavejsonconfigREAD", "Slave Config READ (e.g. 1,3,0,2,float,test/temp)", slaveconfigREAD, 5120);
  WiFiManagerParameter customoutputWRITE("slavejsonconfigWRITE", "Slave Config WRITE", slaveconfigWRITE, 5120);
  WiFiManagerParameter custombaseTopicPath("baseTopicPath", "Base MQTT Topic for Grouped Publishing (e.g. dhaya/test)", baseTopicPath, 40);

  Serial.printf("[HEAP] Before WM params: %u bytes\n", ESP.getFreeHeap());
  wm.addParameter(&customoutput);
  wm.addParameter(&customoutputa);
  wm.addParameter(&customoutputb);
  wm.addParameter(&customoutputc);
  wm.addParameter(&custommqttMethod);
  wm.addParameter(&customMacParam);
  wm.addParameter(&customStaticIPParam);
  wm.addParameter(&customGatewayIPParam);
  wm.addParameter(&customSubnetMaskParam);
  wm.addParameter(&customIPModeParam);
  wm.addParameter(&customAPN);
  wm.addParameter(&customGprsUser);
  wm.addParameter(&customGprsPass);
  wm.addParameter(&customProtocol);
  wm.addParameter(&customIntervalss);
  wm.addParameter(&customoutputd);
  wm.addParameter(&customoutpute);
  wm.addParameter(&customoutputf);
  wm.addParameter(&customoutputg);
  wm.addParameter(&customoutputREAD);
  wm.addParameter(&customoutputWRITE);
  wm.addParameter(&custombaseTopicPath);

  Serial.printf("[HEAP] After WM params: %u bytes\n", ESP.getFreeHeap());
  wm.setConfigPortalTimeout(120);  // 2 min timeout
  wm.setConnectTimeout(20);
  wm.setBreakAfterConfig(true);    // Exit portal after config saved
  wm.setSaveConfigCallback(saveConfigCallback);
  wm.setRemoveDuplicateAPs(true);  // Skip duplicate SSIDs in scan list
  wm.setMinimumSignalQuality(10);  // Skip very weak networks (speeds up scan)

  // ── Connect based on method ──
  if (strcmp(mqttMethod, "wifi") == 0) {
    // Set AP+STA mode — Samsung phones detect this better than pure AP mode
    WiFi.mode(WIFI_AP_STA);

    // Keep retrying portal until WiFi connects, but bail out after 2 attempts
    // if a saved config already exists (prevents permanent boot deadlock).
    bool res = false;
    int portalAttempts = 0;
    bool savedConfigExists = LittleFS.exists("/config.json");
    while (WiFi.status() != WL_CONNECTED) {
      Serial.printf("[HEAP] Before autoConnect: %u bytes\n", ESP.getFreeHeap());
      res = wm.autoConnect("AutoConnectAP");
      Serial.printf("[HEAP] After autoConnect: %u bytes\n", ESP.getFreeHeap());

      if (res && WiFi.status() == WL_CONNECTED) {
        Serial.println("Connected to Wi-Fi");
        break;
      }
      portalAttempts++;
      if (savedConfigExists && portalAttempts >= 2) {
        Serial.println("[WARN] Wi-Fi unavailable after 2 portal attempts. Continuing with saved config.");
        break;
      }
      Serial.println("Failed to connect to Wi-Fi, reopening config portal...");
      WiFi.mode(WIFI_AP_STA);  // Re-set mode for next attempt
    }

    strcpy(mqttserver, customoutput.getValue());
    strcpy(mqttport, customoutputa.getValue());
    strcpy(mqttusername, customoutputb.getValue());
    strcpy(mqttpassword, customoutputc.getValue());
    strcpy(mqttMethod, custommqttMethod.getValue());
    strcpy(ethMacAddr, customMacParam.getValue());
    strcpy(staticIP, customStaticIPParam.getValue());
    strcpy(gatewayIPs, customGatewayIPParam.getValue());
    strcpy(subnetMasks, customSubnetMaskParam.getValue());
    strcpy(ipMode, customIPModeParam.getValue());
    strcpy(apn, customAPN.getValue());
    strcpy(gprsUser, customGprsUser.getValue());
    strcpy(gprsPass, customGprsPass.getValue());
    strcpy(protocolType, customProtocol.getValue());
    strcpy(intervalssParam, customIntervalss.getValue());
    intervalss = strtoul(intervalssParam, NULL, 10);
    strcpy(slavebaudrate, customoutputd.getValue());
    strcpy(slavedatalength, customoutpute.getValue());
    strcpy(slaveparity, customoutputf.getValue());
    strcpy(slavestopbit, customoutputg.getValue());
    strcpy(slaveconfigREAD, customoutputREAD.getValue());
    strcpy(slaveconfigWRITE, customoutputWRITE.getValue());
    strcpy(baseTopicPath, custombaseTopicPath.getValue());
    Serial.printf("[WM-CONFIG] baseTopicPath loaded from WiFiManager: '%s' (len=%d)\n", baseTopicPath, strlen(baseTopicPath));
    parseMACAddress(ethMacAddr, mac);

    if (strcmp(ipMode, "manual") == 0) {
      IPAddress ip, gateway, subnet;
      ip.fromString(staticIP); gateway.fromString(gatewayIPs); subnet.fromString(subnetMasks);
      WiFi.config(ip, gateway, subnet);
      Serial.println("Using manual IP configuration");
    } else {
      Serial.println("Using DHCP (Auto IP Mode)");
    }

    // WiFi is already connected from autoConnect — no need to call WiFi.begin() again
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Successfully connected to Wi-Fi");
      Serial.print("IP Address: "); Serial.println(WiFi.localIP());

      // Stop WiFiManager portal to free ~40-50KB heap
      wm.stopWebPortal();
      WiFi.softAPdisconnect(true);  // Turn off AP mode
      WiFi.mode(WIFI_STA);          // Station mode only
      Serial.printf("[HEAP] After stopping portal: %u bytes\n", ESP.getFreeHeap());

      Serial.printf("[HEAP] Before MQTT begin: %u bytes\n", ESP.getFreeHeap());

      wifiClientPubSub.begin(mqttserver, atoi(mqttport), espClient232);
      wifiClientPubSub.setKeepAlive(120);
      wifiClientPubSub.onMessage(callback);
      yield();  // Feed watchdog

      Serial.printf("[HEAP] Before MQTT connect: %u bytes\n", ESP.getFreeHeap());
      if (!wifiClientPubSub.connected()) {
        Serial.println("Connecting to MQTT broker...");
        String clientId = "ESP32Client-" + String(random(0xffff), HEX);
        bool mqttOk = false;
        if (strlen(mqttusername) > 0) mqttOk = wifiClientPubSub.connect(clientId.c_str(), mqttusername, mqttpassword);
        else mqttOk = wifiClientPubSub.connect(clientId.c_str());
        yield();  // Feed watchdog
        if (mqttOk) Serial.println("Connected to MQTT broker");
        else Serial.println("[WARN] MQTT connect failed, will retry in loop");
      }
      Serial.printf("[HEAP] After MQTT connect: %u bytes\n", ESP.getFreeHeap());
    }

  } else if (strcmp(mqttMethod, "ethernet") == 0) {
    initW5500();  // NEW: W5500 reset + SPI init
    parseMACAddress(ethMacAddr, mac);
    if (strcmp(ipMode, "manual") == 0) {
      IPAddress ip, gateway, subnet;
      ip.fromString(staticIP); gateway.fromString(gatewayIPs); subnet.fromString(subnetMasks);
      Ethernet.begin(mac, ip, gateway, subnet);
    } else {
      Serial.println("Using DHCP (Auto IP Mode) for Ethernet");
      Ethernet.begin(mac);
    }
    int retries = 0;
    while (Ethernet.linkStatus() == LinkOFF && retries < 10) { Serial.println("Waiting for Ethernet..."); delay(1000); retries++; }
    if (Ethernet.linkStatus() != LinkOFF) {
      Serial.println("Ethernet connected");
      Serial.print("IP Address: "); Serial.println(Ethernet.localIP());
      ethernetClientPubSub.begin(mqttserver, atoi(mqttport), ethClient);
      ethernetClientPubSub.onMessage(callback);
      if (!ethernetClientPubSub.connected()) {
        String clientId = "ESP32Client-" + String(random(0xffff), HEX);
        if (strlen(mqttusername) > 0) ethernetClientPubSub.connect(clientId.c_str(), mqttusername, mqttpassword);
        else ethernetClientPubSub.connect(clientId.c_str());
      }
    }

  } else if (strcmp(mqttMethod, "gsm") == 0) {
    if (setupGSM()) {
      // AT+CMQTT subscriptions done via subscribeMQTTGSM() inside reconnect()
      reconnect();
      // Send "hello from board" to MQTT immediately after connect
      delay(500);
      sendMQTTViaGSM("bit/values", "{\"msg\":\"hello from board\"}");
      Serial.println("[GSM] Hello message sent to MQTT!");
    } else {
      Serial.println("Failed to initialize GSM.");
    }
  }

  Serial.printf("[HEAP] Before split configs: %u bytes\n", ESP.getFreeHeap());
  yield();

  // Split READ/WRITE configs
  count = splitString(slaveconfigREAD, specialCharacter, substringsREAD);
  count1 = splitString(slaveconfigWRITE, specialCharacter, substringsWRITE);
  Serial.printf("[SETUP] Modbus READ count: %d, WRITE count: %d\n", count, count1);
  if (count > 0) Serial.printf("[SETUP] First READ topic: %s\n", substringsREAD[0].c_str());
  printDynamicConfigs();

  // Init Modbus based on protocol
  if (strcmp(protocolType, "rs485") == 0) {
    Serial.printf("[HEAP] Before RS485 init: %u bytes\n", ESP.getFreeHeap());
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(DIR, OUTPUT);
    digitalWrite(DIR, HIGH);
    digitalWrite(S0, HIGH);
    digitalWrite(S1, LOW);
    Serial.println("RS485 mode initialized (direct RTU).");
    Serial.printf("[HEAP] After RS485 init: %u bytes\n", ESP.getFreeHeap());
  } else if (strcmp(protocolType, "tcpwifi") == 0) {
    setupNetwork();
    if (WiFi.status() == WL_CONNECTED) { modbusTCPWiFi.client(); Serial.println("Modbus TCP WiFi initialized."); }
  } else if (strcmp(protocolType, "tcpethernet") == 0) {
    setupNetwork();
    if (Ethernet.linkStatus() != LinkOFF) { modbusTCPEthernet.client(); Serial.println("Modbus TCP Ethernet initialized."); }
  }

  printSavedConfig();

  // Extract MQTT topics from WRITE config
  for (int i = 0; i < count1; i++) {
    String splitwrite[6]; char spccar = ',';
    splitString(substringsWRITE[i], spccar, splitwrite);
    splitmqttaddwrite[i] = splitwrite[5].c_str();
  }

  // Extract MQTT topics from READ config
  for (int i = 0; i < count; i++) {
    String splitwrite[6]; char spccar = ',';
    splitString(substringsREAD[i], spccar, splitwrite);
    splitmqttaddread[i] = splitwrite[5].c_str();
  }

  // Force save if config.json doesn't exist yet (first boot after portal config)
  if (!LittleFS.exists("/config.json")) {
    shouldSaveConfig = true;
    Serial.println("[INFO] No config.json found, forcing save.");
  }

  Serial.printf("[HEAP] Before saveConfigToFS: %u bytes\n", ESP.getFreeHeap());
  yield();
  saveConfigToFS();
  Serial.printf("[HEAP] After saveConfigToFS: %u bytes\n", ESP.getFreeHeap());
  yield();

  bool networkReady = false;
  if (strcmp(mqttMethod, "wifi") == 0 && WiFi.status() == WL_CONNECTED) networkReady = true;
  else if (strcmp(mqttMethod, "ethernet") == 0 && Ethernet.linkStatus() != LinkOFF) networkReady = true;
  else if (strcmp(mqttMethod, "gsm") == 0) {
    String res = gsmSendAT("AT+CREG?", 1000);
    if (res.indexOf(",1") != -1 || res.indexOf(",5") != -1) networkReady = true;
  }

  if (networkReady) {
    Serial.printf("[HEAP] Before reconnect: %u bytes\n", ESP.getFreeHeap());
    Serial.println("Reconnecting...");
    yield();
    reconnect();
    Serial.printf("[HEAP] After reconnect: %u bytes\n", ESP.getFreeHeap());
  } else {
    Serial.println("[WARN] Network not connected, skipping MQTT reconnect.");
  }

  yield();
  Serial.printf("[HEAP] Before listStoredFiles: %u bytes\n", ESP.getFreeHeap());
  listStoredFiles();
  printFileNamesAndContents();
  loadConfigFromFiles();
  printConfigMap();
  setupNTP();

  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) lastResetDay = timeinfo.tm_mday;
  midnightTicker.attach(60, checkForMidnightReset);

  // Initialize grouped device data publisher (10-second interval)
  Serial.printf("[GROUPED-SETUP] Checking grouped publishing setup. baseTopicPath='%s' (len=%d)\n", baseTopicPath, strlen(baseTopicPath));
  if (baseTopicPath[0] != '\0') {
    useGroupedPublishing = true;
    publishGroupedTicker.attach(10, publishGroupedDeviceData);
    Serial.printf("[GROUPED-SETUP] ENABLED grouped publishing to base topic: %s\n", baseTopicPath);
    Serial.printf("[GROUPED-SETUP] Ticker attached (10 second interval)\n");
  } else {
    useGroupedPublishing = false;
    Serial.println("[GROUPED-SETUP] DISABLED grouped publishing (empty baseTopicPath)");
  }

  printPartitionTable();
 
  Serial.println("==========================================");
  Serial.printf("[SETUP DONE] Protocol: %s, READ Count: %d, Interval: %ld ms\n", protocolType, count, intervalss);
  Serial.println("[SETUP DONE] Entering Main Loop...");
  Serial.println("==========================================");
}

// ══════════════════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════════════════

void loop() {
  static unsigned long lastLoopHeartbeat = 0;
  if (millis() - lastLoopHeartbeat > 5000) {
      Serial.printf("[HEARTBEAT] Loop running. Method: %s, GSM Connected: %s\n", mqttMethod, gsmMQTTConnected ? "YES" : "NO");
      lastLoopHeartbeat = millis();
  }
  int pressCount = checkButton();
  // Button press logging (OLED display code removed)
  if (pressCount > 0) {
    Serial.printf("[BUTTON] Press count: %d\n", pressCount);
    if (pressCount > 26) pressCount = 1;
  }

  checkReconnect();

  // Only run WiFiManager portal when WiFi is not connected (saves ~40-50KB heap)
  if (strcmp(mqttMethod, "wifi") == 0 && WiFi.status() != WL_CONNECTED) {
    wm.process();
  }

  if (strcmp(mqttMethod, "gsm") == 0) {
    handleGSMMQTT();
  } else if (strcmp(mqttMethod, "ethernet") == 0) {
    handleEthernetMQTT();
  } else {
    handleWiFiMQTT();
  }

  static unsigned long lastRequestStart = 0;
  if (requestInProgress && (millis() - lastRequestStart > 10000)) {
      Serial.println("[WARN] Modbus request timeout! Forcing requestInProgress = false");
      requestInProgress = false;
  }

  // Modbus polling
  unsigned long currentMillisss = millis();
  long intsval = (intervalss > 0) ? intervalss : 1000;
  if (currentMillisss - previousMillisss >= intsval && !requestProcessed && !requestInProgress) {
      lastRequestStart = millis();
    if (count > 0) {
      if (modbusIndex < count) {
        String splitread[6]; char spccar = ',';
        splitString(substringsREAD[modbusIndex], spccar, splitread);
       
        // Skip if slave ID is empty or 0
        if (splitread[0] == "" || splitread[0] == "0") {
          modbusIndex++;
        } else {
          Serial.printf("[MODBUS] Requesting idx=%d/%d slave=%s fc=%s addr=%s count=%s type=%s\n",
            modbusIndex, count, splitread[0].c_str(), splitread[1].c_str(), splitread[2].c_str(),
            splitread[3].c_str(), splitread[4].c_str());

          if (strcmp(protocolType, "rs485") == 0) {
            readprocessModbusRequest(splitread[0].toInt(), splitread[1].toInt(), splitread[2].toInt(),
                                     splitread[3].toInt(), splitread[4], splitread[5].c_str());
          } else if (strcmp(protocolType, "tcpwifi") == 0) {
            IPAddress slaveIP; slaveIP.fromString(splitread[0]);
            readprocessModbusRequestTCPWiFi(slaveIP, splitread[1].toInt(), splitread[2].toInt(),
                                            splitread[3].toInt(), splitread[4], splitread[5].c_str());
          } else if (strcmp(protocolType, "tcpethernet") == 0) {
            IPAddress slaveIP; slaveIP.fromString(splitread[0]);
            readprocessModbusRequestTCPEthernet(slaveIP, splitread[1].toInt(), splitread[2].toInt(),
                                                splitread[3].toInt(), splitread[4], splitread[5].c_str());
          }
          modbusIndex++;
        }
      } else {
        modbusIndex = 0;
        Serial.printf("[MODBUS] Cycle complete. Free heap: %u\n", ESP.getFreeHeap());
      }
    } else {
      static unsigned long lastZeroLog = 0;
      if (millis() - lastZeroLog > 10000) {
        Serial.println("[MODBUS] No registers configured (count=0).");
        lastZeroLog = millis();
      }
    }
    previousMillisss = millis();
    requestProcessed = false;
  }

  updateDisplayState();

  // Reset button in loop (existing — GPIO 25)
  if (digitalRead(resetbutton) == LOW) {
    wm.resetSettings();
    clearLittleFS();
    if (LittleFS.begin()) {
      if (LittleFS.exists("/config.json")) LittleFS.remove("/config.json");
      LittleFS.end();
    }
    ESP.restart();
  }

  // Config reset button (GPIO 33) — clears all settings and opens portal
  if (configResetPressed()) {
    Serial.println("[RESET] WiFi reset button (GPIO 33) pressed. Clearing config...");
    wm.resetSettings();
    clearLittleFS();
    skipConfigResetAfterClear = true;
    delay(500);
    ESP.restart();
  }
}
