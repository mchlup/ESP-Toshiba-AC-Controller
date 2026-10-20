#include <Arduino.h>
#ifndef USE_MQTT_TLS
  #define USE_MQTT_TLS false
#endif

// ===== Platforma (ESP8266/ESP32): WiFi + WebServer =====
#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <ESP8266WebServer.h>
  using WebSrv = ESP8266WebServer;
#elif defined(ESP32)
  #include <WiFi.h>
  #include <WebServer.h>
  using WebSrv = WebServer;
#else
  #error "Unsupported platform (need ESP8266 or ESP32)"
#endif

#include <WiFiManager.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <WiFiClient.h>
#if (USE_MQTT_TLS)
  #include <WiFiClientSecure.h>
#endif
#include "ToshibaCarrierHvac.h"
#include "FeatureConfig.h"
#include "LocaleCs.h"
#include "LocaleEn.h"

#if defined(ESP32)
  #include <HardwareSerial.h>
#endif

// ===== Modbus knihovna (název se liší podle platformy) =====
#if defined(ESP8266)
  #include <ModbusIP_ESP8266.h>
#elif defined(ESP32)
  //#include <ModbusIP_ESP32.h>
  #include <ModbusIP_ESP8266.h>
#endif

#include <math.h>
#include <limits.h>

// ===== OTA/HTTP UPDATE rozdíly =====
#if defined(ESP8266)
  #include <ESP8266httpUpdate.h>
  #include <Updater.h>   // třída Update (ESP8266 core)
#elif defined(ESP32)
  // ESP32 nepotřebuje ESP8266httpUpdate.h
  #include <Update.h>    // třída Update (ESP32 core)
#endif

struct DeviceConfig;

// Minimalní implicitní implementace „raw“ hooku, aby nepadal linker.
// Můžeš si z toho udělat logger – níže je ukázka s volitelným výpisem.
// Defined after diagnostics storage (no dynamic allocation).
void HvacRawAnalyzer_onFrame(bool tx, uint8_t cmd, const uint8_t* payload, uint8_t plen);

// ================== HW & BUILD CONFIG ==================
#if defined(ESP8266)
  // NodeMCU / Wemos D1 mini: osvědčené piny pro SW/HW serial
  #define RX_PIN         15      // GPIO15; test wiring (D8 on D1 mini)
  #define TX_PIN         13      // GPIO13; test wiring (D7 on D1 mini)
  #define LED_PIN         2      // D4 (GPIO2), active LOW
  #define FLASH_BTN_PIN   0      // D3 (GPIO0) – FLASH
#elif defined(ESP32)
  // Jemnější rozlišení podle cílové varianty ESP32
  #if defined(CONFIG_IDF_TARGET_ESP32C3)
    // ESP32-C3 má pouze UART0/1. Doporučené piny pro UART1:
    // RX=GPIO20, TX=GPIO21 (na většině DevKitM-1 vyvedené)
    #define RX_PIN       3
    #define TX_PIN       4
    #define FLASH_BTN_PIN 9      // BOOT na C3 bývá GPIO9
    #define LED_PIN       8      // ESP32-C3 SuperMini built-in active-low LED
  #elif defined(CONFIG_IDF_TARGET_ESP32S2)
    // S2: běžně dostupné piny pro HW serial
    #define RX_PIN       16
    #define TX_PIN       17
    #define FLASH_BTN_PIN 0
    #define LED_PIN       2
  #elif defined(CONFIG_IDF_TARGET_ESP32S3)
    // S3 DevKit: UART2 na 16/17, BOOT na GPIO0, LED obvykle na 2
    #define RX_PIN       16
    #define TX_PIN       17
    #define FLASH_BTN_PIN 0
    #define LED_PIN       2
  #else
    // Klasické ESP32 (WROOM/DEVKIT V1): UART2 -> 16/17
    #define RX_PIN       16
    #define TX_PIN       17
    #define FLASH_BTN_PIN 0
    #define LED_PIN       2      // GPIO2 (na většině DevKit je LED na 2)
  #endif
#endif

#ifndef LED_PIN
  #define LED_PIN 2
#endif

#ifndef RED_LED_PIN
  #define RED_LED_PIN   -1       // Dejte -1, pokud LED není připojena.
#endif

#define BTN_DEBOUNCE_MS 30
#define BTN_SHORT_MIN   50
#define BTN_SHORT_MAX   2000
// USE_MQTT_TLS is configured before includes.
#define EEPROM_SIZE  1024
#define CONFIG_MAGIC 0x42
#define CONFIG_VER   8   // +Telnet enable/pass; OTA gating; další drobnosti

struct SwingFixMap;
static bool HvacRawAnalyzer_guessSwingFixMap(SwingFixMap &out);
static const uint32_t STATE_PUBLISH_MS   = 10000UL;
static const uint32_t HEARTBEAT_MS       = 30000UL;
static const uint32_t HVAC_TIMEOUT_MS    = 30000UL;
static const uint32_t MQTT_RETRY_MS      = 5000UL;
static const uint32_t WIFI_PORTAL_MANUAL_LIMIT_MS = 300000UL;
// Boot: HVAC is independent of Wi-Fi. The station association is asynchronous;
// WiFiManager captive portal is processed cooperatively in loop().
static const uint32_t WIFI_BOOT_STA_GRACE_MS     = 20000UL;
static const uint32_t WIFI_PORTAL_RETRY_DELAY_MS = 30000UL;
static const uint32_t WIFI_STA_RECONNECT_MS      = 20000UL;
static const uint32_t MQTT_BOOT_GUARD_MS         = 15000UL;
static const uint16_t WFM_STA_TIMEOUT_S  = 180;

// Struktura konfigurace + CRC16 (X25)
struct DeviceConfig {
  //uint8_t  magic;
  uint16_t magic;              // rychlá detekce platnosti
  uint8_t  version;
  char     mqttServer[64];
  uint16_t mqttPort;                 // 1883/8883
  char     mqttUser[40];
  char     mqttPassword[40];
  char     mqttCmdTopic[64];         // "toshiba/{id}/control"
  char     mqttStateTopic[64];       // "toshiba/{id}/state"
  char     mqttHeartbeatTopic[64];   // "toshiba/{id}/heartbeat"
  uint8_t  mqttEnable;               // 0/1
  char     webUser[20];
  char     webPassword[20];
  uint8_t  useCustomId;              // 0/1
  char     customDeviceId[32];       // A-Za-z0-9_-
  uint8_t  modbusEnable;             // 0/1
  uint16_t modbusPort;               // (informativní) default 502
  uint8_t  modbusUnitId;             // (informativní)
  uint8_t  otaEnable;                // 0/1
  char     otaPassword[32];          // prázdné = bez hesla
  uint8_t  telnetEnable;             // 0/1
  char     telnetPassword[16];       // prázdné = bez hesla
  uint16_t crc16;
  char     wifiSsid[32];
  char     wifiPass[64];
  char     hostname[32];       // jméno zařízení (OTA/mDNS/UI)
  uint8_t  wifiMgrTimeoutMin;  // kolik minut čekat v STA, než AP
  uint16_t reserved;           // zarovnání / budoucí použití
  char     room[24];            // např. "Obyvak" (bez diakritiky doporučeno)
  char     unitName[24];        // volitelný název jednotky, např. "AC Obyvak"
};

static const uint16_t CFG_MAGIC = 0xABCD;
static const uint8_t  CFG_VERSION = 1;

// === JEDINÁ DEFINICE CRC (bez dalšího prototypu později) ===
static uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t seed = 0xFFFF) {
  uint16_t crc = seed;
  while (len--) {
    crc ^= (uint16_t)(*data++) << 8;
    for (uint8_t i = 0; i < 8; i++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// === VÝCHOZÍ NAPLNĚNÍ KONFIGU ===
static void fillDefaultConfig(DeviceConfig &cfg) {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic   = CFG_MAGIC;
  cfg.version = CFG_VERSION;

  // SSID/PASS prázdné => WiFiManager otevře portál

  #if defined(strlcpy)
    strlcpy(cfg.hostname, "Toshiba-HVAC", sizeof(cfg.hostname));
    strlcpy(cfg.webUser,     "admin", sizeof(cfg.webUser));
    strlcpy(cfg.webPassword, "admin", sizeof(cfg.webPassword));
    // MQTT topics s {id}
    strlcpy(cfg.mqttCmdTopic,       "toshiba/{id}/control",   sizeof(cfg.mqttCmdTopic));
    strlcpy(cfg.mqttStateTopic,     "toshiba/{id}/state",     sizeof(cfg.mqttStateTopic));
    strlcpy(cfg.mqttHeartbeatTopic, "toshiba/{id}/heartbeat", sizeof(cfg.mqttHeartbeatTopic));
  #else
    strncpy(cfg.hostname, "Toshiba-HVAC", sizeof(cfg.hostname) - 1);
    cfg.hostname[sizeof(cfg.hostname) - 1] = '\0';
    strncpy(cfg.webUser, "admin", sizeof(cfg.webUser) - 1);
    cfg.webUser[sizeof(cfg.webUser) - 1] = '\0';
    strncpy(cfg.webPassword, "admin", sizeof(cfg.webPassword) - 1);
    cfg.webPassword[sizeof(cfg.webPassword) - 1] = '\0';
    strncpy(cfg.mqttCmdTopic, "toshiba/{id}/control", sizeof(cfg.mqttCmdTopic) - 1);
    cfg.mqttCmdTopic[sizeof(cfg.mqttCmdTopic) - 1] = '\0';
    strncpy(cfg.mqttStateTopic, "toshiba/{id}/state", sizeof(cfg.mqttStateTopic) - 1);
    cfg.mqttStateTopic[sizeof(cfg.mqttStateTopic) - 1] = '\0';
    strncpy(cfg.mqttHeartbeatTopic, "toshiba/{id}/heartbeat", sizeof(cfg.mqttHeartbeatTopic) - 1);
    cfg.mqttHeartbeatTopic[sizeof(cfg.mqttHeartbeatTopic) - 1] = '\0';
  #endif

  // MQTT: default vypnuto, ale port nastav
  cfg.mqttEnable = 0;
  cfg.mqttPort   = USE_MQTT_TLS ? 8883 : 1883;

  // Modbus: ON, info-only port/unit
  cfg.modbusEnable = 1;
  cfg.modbusPort   = 502;
  cfg.modbusUnitId = 1;

  // OTA/Telnet: výchozí OFF (hesla prázdná = žádné)
  cfg.otaEnable    = 0;
  cfg.otaPassword[0] = '\0';
  cfg.telnetEnable = 0;
  cfg.telnetPassword[0] = '\0';

  // Device ID: default z MAC (useCustomId=0)
  cfg.useCustomId = 0;
  cfg.customDeviceId[0] = '\0';
  cfg.room[0] = '\0';
  cfg.unitName[0] = '\0';

  cfg.wifiMgrTimeoutMin = 2;
  cfg.reserved = 0;

  // CRC
  cfg.crc16 = 0;
  cfg.crc16 = crc16_ccitt(reinterpret_cast<const uint8_t*>(&cfg), sizeof(cfg), 0xFFFF);
}

// === PŘEPOČET A ULOŽENÍ CRC ===
static void computeAndStoreCRC(DeviceConfig &cfg) {
  cfg.crc16 = 0;
  cfg.crc16 = crc16_ccitt(reinterpret_cast<const uint8_t*>(&cfg), sizeof(cfg), 0xFFFF);
}

// === VALIDACE KONFIGU ===
static bool isConfigValid(const DeviceConfig &cfg) {
  if (cfg.magic   != CFG_MAGIC)   return false;
  if (cfg.version != CFG_VERSION) return false;

  DeviceConfig tmp = cfg;
  tmp.crc16 = 0;
  const uint16_t c = crc16_ccitt(reinterpret_cast<const uint8_t*>(&tmp), sizeof(tmp), 0xFFFF);
  return (c == cfg.crc16);
}

// Extended feature settings: separate record, original DeviceConfig is unchanged.
static FeatureOptions featureOptions {};
static bool featureDirty = false;
static bool haDiscoveryPublished = false;
static bool haDiscoveryEverPublished = false;
static uint32_t haLastPublishMs = 0;
static uint32_t mqttReconnectCount = 0;
static uint32_t modbusClientWrites = 0;
static uint32_t modbusRejectedWrites = 0;
static uint32_t restartRequestedMs = 0;
static uint32_t wifiPortalStartedMs = 0;
static bool wifiPortalManual = false;
static bool modbusInitializedOnce = false;
static uint32_t bootReasonCode = 0;

static void defaultFeatureOptions() {
  memset(&featureOptions, 0, sizeof(featureOptions));
  featureOptions.magic = FEATURE_MAGIC;
  featureOptions.version = FEATURE_SCHEMA;
}
static bool validFeatureOptions(const FeatureOptions& value) {
  if (value.magic != FEATURE_MAGIC || value.version != FEATURE_SCHEMA) return false;
  FeatureOptions tmp = value;
  tmp.crc = 0;
  if (crc16_ccitt(reinterpret_cast<const uint8_t*>(&tmp), sizeof(tmp)) != value.crc) return false;
  for (uint8_t i = 0; i < CAP_COUNT; ++i) if (value.capability[i] > CAP_UNSUPPORTED) return false;
  return value.haDiscovery <= 1;
}
static void loadFeatureOptions() {
  EEPROM.get(FEATURE_EEPROM_OFFSET, featureOptions);
  if (!validFeatureOptions(featureOptions)) defaultFeatureOptions();
}
static void saveFeatureOptions() {
  FeatureOptions tmp = featureOptions;
  tmp.crc = 0;
  featureOptions.crc = crc16_ccitt(reinterpret_cast<const uint8_t*>(&tmp), sizeof(tmp));
  EEPROM.put(FEATURE_EEPROM_OFFSET, featureOptions);
  EEPROM.commit();
}
static bool capabilityEnabled(CapabilityId id) {
  return featureOptions.capability[id] != CAP_UNSUPPORTED;
}
static const char* capabilityStateName(uint8_t setting) {
  switch (setting) { case CAP_SUPPORTED: return "supported"; case CAP_UNSUPPORTED: return "unsupported"; default: return "auto"; }
}
static bool commandAllowedByCapabilities(const char* cmd) {
  if (!cmd) return false;
  if (!strncasecmp(cmd, "SWING_FIX_", 10) && !capabilityEnabled(CAP_SWING_POSITIONS)) return false;
  if (!strcasecmp(cmd,"SWING_OFF") && !capabilityEnabled(CAP_SWING_FIXED)) return false;
  if (!strcasecmp(cmd,"SWING_V") && !capabilityEnabled(CAP_SWING_VERTICAL)) return false;
  if (!strcasecmp(cmd,"SWING_H") && !capabilityEnabled(CAP_SWING_HORIZONTAL)) return false;
  if (!strcasecmp(cmd,"SWING_VH") && !capabilityEnabled(CAP_SWING_BOTH)) return false;
  return true;
}
char deviceLabel[32] = {0};  // zobrazený název (name → room-AC → AC-<id>)
DeviceConfig deviceConfig;

// ================== Globals ==================
// HVAC instance:
#if defined(ESP8266)
  ToshibaCarrierHvac hvac(RX_PIN, TX_PIN);       // SW serial uvnitř knihovny
#elif defined(ESP32)
  #if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32S2)
  HardwareSerial HVAC_SERIAL(1);   // C3/S2 mají jen UART0/1 -> použij UART1
#else
  HardwareSerial HVAC_SERIAL(2);   // klasické ESP32/ESP32-S3 -> UART2
#endif
  // HW UART2
  ToshibaCarrierHvac hvac(&HVAC_SERIAL);         // knihovna si sama volá begin()
#endif

WiFiManager wifiManager;
WiFiClient        wifiClientPlain;
#if (USE_MQTT_TLS)
  WiFiClientSecure  wifiClientTLS;
#endif
PubSubClient      mqttClient;
//ESP8266WebServer server(80);
WebSrv server(80);
static char csrfToken[33] = {0};
static bool validCsrf() {
  if (server.header("X-Requested-With") != "XMLHttpRequest") return false;
  return server.header("X-CSRF-Token") == csrfToken && csrfToken[0];
}
static void initCsrf() {
  uint32_t keys[4];
  for (unsigned i=0; i<4; ++i) keys[i] = (uint32_t)random(0x7FFFFFFF) ^ (micros() * (i+1));
  snprintf(csrfToken,sizeof(csrfToken),"%08lx%08lx%08lx%08lx",
    (unsigned long)keys[0],(unsigned long)keys[1],
    (unsigned long)keys[2],(unsigned long)keys[3]);
}


WiFiServer       telnetServer(23);
WiFiClient       telnetClient;
static bool telnetEnabled = true;
static bool telnetAuthenticated = false;
char deviceID[13] = {0};  // 12 + '\0'
volatile bool shouldSaveConfig = false;
uint32_t lastHvacResponse = 0;
uint32_t lastHeartbeat = 0;
uint32_t lastStatePub = 0;
bool mqttEnabled = false;          // výchozí OFF
int  mqttFailCount = 0;
uint32_t lastMqttAttempt = 0;
volatile bool otaActive = false;   // << oprava: start na false
uint32_t      otaStartMs = 0;
static bool   otaStarted = false;  // hlídá, zda už běží OTA (pro runtime zapnutí)
// The HTTP/Modbus listeners cannot start while the WiFiManager AP owns port 80.
static bool networkServicesStarted = false;
static bool wifiHadCredentials = false;
static bool wifiWasConnected = false;
static bool wifiHadPortal = false;
static uint32_t wifiAttemptStartMs = 0;
static uint32_t wifiPortalLastClosedMs = 0;
static uint32_t wifiLastReconnectMs = 0;
static uint32_t firmwareBootMs = 0;
static const uint32_t BOOT_RED_LED_MS = 2000UL;
bool     bootRedActive = false;
uint32_t bootRedUntil  = 0;
ModbusIP mb;
bool modbusActive = true;
static uint32_t g_stateRev = 1;
static uint32_t g_lastHash = 0;

#if defined(ESP8266)
  static const int PWM_MAX = 1023;
#else
  static const int PWM_MAX = 255;
#endif

static inline void pwmWriteCompat(int pin, int value01k){
  // value01k očekává 0..PWM_MAX
  int v = value01k;
  if (v < 0) v = 0;
  if (v > PWM_MAX) v = PWM_MAX;
  analogWrite(pin, v);
}


// Mapování Holding registrů (HREG)
enum : uint16_t {
  HREG_POWER      = 0,   // 0/1
  HREG_SETPOINT   = 1,   // 17..30 (°C)
  HREG_MODE       = 2,   // viz mapování níže
  HREG_FAN        = 3,   // viz mapování níže
  HREG_SWING      = 4,   // viz mapování níže
  // >>> NOVÉ TELEMETRIE
  HREG_TIN_X10    = 5,   // int16 (°C*10), 32767 = N/A
  HREG_TOUT_X10   = 6,   // int16 (°C*10), 32767 = N/A
  HREG_OPER       = 7,   // 0=off,1=auto,2=heat,3=cool,4=dry,5=fan_only
  HREG_FLAGS      = 8,   // bitové příznaky (0 pokud neznámé)
  HREG_ERROR      = 9,   // error code (0xFFFF pokud neznámý)
  HREG_STATUS     = 10,  // bit0=HVAC UART, bit1=MQTT, bit2=WiFi
  HREG_UPTIME_S   = 11,  // uptime (low 16b)
  HREG__COUNT
};

// Cache pro detekci zápisů klientem
uint16_t hregCache[HREG__COUNT] = {0};

// WiFiManager parametry
WiFiManagerParameter p_mqtt_en   ("mqtt_en", "MQTT Enable (0/1)", "", 2);
WiFiManagerParameter p_mqtt_server("server", "MQTT Server", "", 64);
WiFiManagerParameter p_mqtt_port  ("port", "MQTT Port", "", 6);
WiFiManagerParameter p_mqtt_user  ("user", "MQTT Username", "", 40);
WiFiManagerParameter p_mqtt_pass  ("pass", "MQTT Password", "", 40);
WiFiManagerParameter p_mqtt_cmd   ("cmdtopic", "MQTT Command Topic", "", 64);
WiFiManagerParameter p_mqtt_state ("statetopic", "MQTT State Topic", "", 64);
WiFiManagerParameter p_mqtt_hb    ("hbtopic", "MQTT Heartbeat Topic", "", 64);
WiFiManagerParameter p_dev_id_en("dev_id_en", "Use custom Device ID (0/1)", "", 2);
WiFiManagerParameter p_dev_id   ("dev_id",    "Custom Device ID (A-Za-z0-9_-)", "", 32);
WiFiManagerParameter p_mb_enable("mb_en",   "Modbus TCP Enable (0/1)", "", 2);
WiFiManagerParameter p_mb_port  ("mb_port", "Modbus TCP Port (info)", "", 6);
WiFiManagerParameter p_mb_uid   ("mb_uid",  "Modbus TCP Unit ID (info)", "", 4);
WiFiManagerParameter p_ota_en    ("ota_en",   "OTA Enable (0/1)", "", 2);
WiFiManagerParameter p_ota_pass  ("ota_pass", "OTA Password (blank = none)", "", 32);
WiFiManagerParameter p_telnet_en   ("telnet_en",   "Telnet Enable (0/1)", "", 2);
WiFiManagerParameter p_telnet_pass ("telnet_pass", "Telnet Password (blank = none)", "", 16);
WiFiManagerParameter p_room   ("room",   "Room/Location (bez diakritiky)", "", 24);
WiFiManagerParameter p_name   ("name",   "Unit name/Label", "", 24);

// ================== Utils ==================
static uint16_t crc16_x25(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; ++i) {
      crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : (crc >> 1);
    }
  }
  crc = ~crc;
  uint16_t lo = (crc & 0xFF);
  uint16_t hi = (crc >> 8);
  return (lo << 8) | hi;
}

static void loadConfig() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, deviceConfig);
  if (!isConfigValid(deviceConfig)) {
    fillDefaultConfig(deviceConfig);
    computeAndStoreCRC(deviceConfig);
    EEPROM.put(0, deviceConfig);
    EEPROM.commit();
    Serial.println(F("Použita výchozí konfigurace (EEPROM init)."));
  } else {
    Serial.println(F("Konfigurace načtena z EEPROM."));
  }
  loadFeatureOptions();
}

static void saveConfig() {
  computeAndStoreCRC(deviceConfig);
  EEPROM.put(0, deviceConfig);
  EEPROM.commit();
  Serial.println(F("Konfigurace uložena do EEPROM."));
}

static void saveConfigCallback() { shouldSaveConfig = true; }

// === FACTORY RESET (smazání EEPROM + Wi-Fi kredencí) ===
static void doFactoryReset() {
  Serial.println(F("[FACTORY] Reset to defaults requested"));

  // Zastav služby, ať nepíšou do sítě během mazání
  if (mqttClient.connected()) mqttClient.disconnect();
  modbusActive = false;
  server.stop();
  if (telnetClient) telnetClient.stop();
  telnetServer.stop();

  // 1) EEPROM: kompletní wipe + naplnění výchozího configu
  EEPROM.begin(EEPROM_SIZE);
  for (int i = 0; i < EEPROM_SIZE; ++i) EEPROM.write(i, 0xFF);
  EEPROM.commit();

  DeviceConfig fresh;
  fillDefaultConfig(fresh);
  computeAndStoreCRC(fresh);
  EEPROM.put(0, fresh);
  EEPROM.commit();

  // 2) Wi-Fi credentials / WiFiManager portál
  wifiManager.resetSettings();     // vyčistí uložené SSID/PASS (WiFiManager)

#if defined(ESP8266)
  WiFi.persistent(true);
  WiFi.disconnect(true);           // zapomene STA i AP
  ESP.eraseConfig();               // smaže NVS s Wi-Fi (ESP8266)
  WiFi.persistent(false);
#elif defined(ESP32)
  // true,true = zapomenout i uložené sítě
  WiFi.disconnect(true, true);
  // (volitelně) esp_wifi_restore() – není nutné, WiFi.disconnect(true,true) stačí
#endif

  delay(200);
  Serial.println(F("[FACTORY] Rebooting..."));
  ESP.restart();
  // (nedostaneme se sem)
}

// --- Device ID helpers ---
static void getMacNoColons(char* out12) {
  uint8_t mac[6]; WiFi.macAddress(mac);
  snprintf(out12, 13, "%02X%02X%02X%02X%02X%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void sanitizeDeviceId(char* s) {
  for (char* p = s; *p; ++p) {
    char c = *p;
    bool ok = (c>='A'&&c<='Z') || (c>='a'&&c<='z') || (c>='0'&&c<='9') || c=='_' || c=='-';
    if (!ok) *p = '-';
  }
}

static void computeDeviceID() {
  if (deviceConfig.useCustomId && deviceConfig.customDeviceId[0]) {
    char tmp[32]; strlcpy(tmp, deviceConfig.customDeviceId, sizeof(tmp));
    sanitizeDeviceId(tmp);
    char mac12[13]; getMacNoColons(mac12);
    char padded[13] = {0};
    size_t L = strnlen(tmp, 12);
    memcpy(padded, tmp, L);
    if (L < 12) memcpy(padded + L, mac12, 12 - L);
    strlcpy(deviceID, padded, sizeof(deviceID));
  } else {
    getMacNoColons(deviceID);
  }
}

static void computeDeviceLabel() {
  if (deviceConfig.unitName[0]) {
    strlcpy(deviceLabel, deviceConfig.unitName, sizeof(deviceLabel));
  } else if (deviceConfig.room[0]) {
    // Když je pouze místnost, udělej čitelný label
    snprintf(deviceLabel, sizeof(deviceLabel), "%s-AC", deviceConfig.room);
  } else {
    snprintf(deviceLabel, sizeof(deviceLabel), "AC-%s", deviceID);
  }
}

// Expanduje {id} v MQTT topicu
static void expandTopic(const char* in, char* out, size_t outLen) {
  const char* p = in;
  size_t used = 0;

  auto emitStr = [&](const char* s){
    while (*s && used + 1 < outLen) out[used++] = *s++;
  };
  auto emitChar = [&](char c){ if (used + 1 < outLen) out[used++] = c; };

  while (*p && used + 1 < outLen) {
    if (*p == '{') {
      const char* q = strchr(p, '}');
      if (q) {
        size_t len = (size_t)(q - (p + 1));
        auto eqTok = [&](const char* lit)->bool {
          size_t L = strlen(lit);
          return (L == len) && (strncmp(p + 1, lit, L) == 0);
        };

        if (eqTok("id")) {
          emitStr(deviceID);
        } else if (eqTok("room")) {
          emitStr(deviceConfig.room[0] ? deviceConfig.room : deviceID);
        } else if (eqTok("name")) {
          emitStr(deviceLabel[0] ? deviceLabel : deviceID);
        } else {
          // neznámý token – vytiskni '{' a pokračuj po znacích
          emitChar(*p++);
          continue;
        }
        p = q + 1;
        continue;
      }
    }
    emitChar(*p++);
  }
  out[used] = '\0';
}

// Null-safe helper pro JSON
static const char* nz(const char* s) { return s ? s : ""; }

// Escape user-configurable labels before inserting into HTML.
static String htmlEscape(const char* raw) {
  String escaped;
  const char* p = raw ? raw : "";
  escaped.reserve(strlen(p) + 8);
  for (; *p; ++p) {
    switch (*p) {
      case '&': escaped += F("&amp;"); break;
      case '<': escaped += F("&lt;"); break;
      case '>': escaped += F("&gt;"); break;
      case '"': escaped += F("&quot;"); break;
      case '\'': escaped += F("&#39;"); break;
      default: escaped += *p; break;
    }
  }
  return escaped;
}

// ---------- HVAC RAW ANALYZER (TX/RX logger + heuristika pro FIX1..5) ----------
struct RawEvt {
  uint32_t ts_ms;
  bool     tx;        // true=TX (my -> jednotka), false=RX (jednotka -> my)
  uint8_t  cmd;       // např. 163 = Swing
  uint8_t  len;       // délka payloadu
  uint8_t  data[8];   // první bajty payloadu
};

static const uint16_t RAW_RING_MAX = 128;
static RawEvt g_rawRing[RAW_RING_MAX];
static volatile uint16_t g_rawHead = 0;
static volatile bool g_sniffEnabled = true;

struct CmdStats {
  uint8_t  cmd;       // 1 B (+ zarovnání)
  uint32_t seen;      // 4 B
  uint32_t vcount[64]; // 256 * 4 B = 1024 B
};
// ~1030 B na položku => 16× ≈ 16,5 kB staticky!

static CmdStats g_cmdStats[4];
static uint8_t  g_cmdStatsUsed = 0;

static int _findStats(uint8_t cmd) {
  for (uint8_t i=0;i<g_cmdStatsUsed;i++) if (g_cmdStats[i].cmd == cmd) return i;
  if (g_cmdStatsUsed < (sizeof(g_cmdStats)/sizeof(g_cmdStats[0]))) {
    g_cmdStats[g_cmdStatsUsed] = {};
    g_cmdStats[g_cmdStatsUsed].cmd = cmd;
    return g_cmdStatsUsed++;
  }
  return -1;
}

static void HvacRawAnalyzer_reset() {
  noInterrupts();
  g_rawHead = 0;
  for (uint16_t i=0;i<RAW_RING_MAX;i++){ g_rawRing[i] = {}; }
  for (uint8_t i=0;i<g_cmdStatsUsed;i++){
    g_cmdStats[i].seen = 0;
    memset(g_cmdStats[i].vcount, 0, sizeof(g_cmdStats[i].vcount));
  }
  g_cmdStatsUsed = 0;
  interrupts();
}

static void HvacRawAnalyzer_enable(bool en){ g_sniffEnabled = en; }
static volatile uint32_t g_rxFrames = 0, g_txFrames = 0;
static volatile uint32_t g_lastRawMs = 0;
void HvacRawAnalyzer_onFrame(bool tx, uint8_t cmd, const uint8_t* payload, uint8_t plen) {
  if (!g_sniffEnabled) return;
  RawEvt &e = g_rawRing[g_rawHead % RAW_RING_MAX];
  e.ts_ms = millis();
  e.tx = tx;
  e.cmd = cmd;
  e.len = (plen > sizeof(e.data)) ? sizeof(e.data) : plen;
  memset(e.data, 0, sizeof(e.data));
  if (payload && e.len) memcpy(e.data, payload, e.len);
  g_rawHead = (g_rawHead + 1) % RAW_RING_MAX;
  g_lastRawMs = e.ts_ms;
  if (tx) ++g_txFrames; else ++g_rxFrames;
  const int idx = _findStats(cmd);
  if (idx >= 0) {
    ++g_cmdStats[idx].seen;
    if (payload && plen) {
      const uint8_t bin = payload[0] & 63;
      ++g_cmdStats[idx].vcount[bin];
    }
  }
}


// --- Výstupní struktura pro odhad fix pozic (1..5) z RX armatur (cmd=163)
struct SwingFixMap {
  bool     have;
  uint8_t  valForFix[6]; // index 1..5, hodnota data[0] (0=neznámé)
};

// Heuristika: hledá RX rámce (cmd=163) 0–800 ms po našich TX nastavovacích rámcích
// Vrací true pokud se podařilo něco odhadnout; výstup je v 'out'
static bool HvacRawAnalyzer_guessSwingFixMap(SwingFixMap &out) {
  SwingFixMap m = {}; m.have = false;
  uint16_t head = g_rawHead;
  auto prev = [&](uint16_t idx)->uint16_t { return (idx==0)? (RAW_RING_MAX-1):(idx-1); };

  uint8_t found = 0;
  uint16_t idx = (head==0)? (RAW_RING_MAX-1) : (head-1);

  for (uint16_t scan=0; scan<RAW_RING_MAX; scan++, idx=prev(idx)) {
    const RawEvt &e = g_rawRing[idx];
    if (e.ts_ms == 0) break;
    if (!(e.tx && e.cmd==163)) continue;    // hledej TX swing
    uint32_t winStart = e.ts_ms, winEnd = winStart + 800;

    uint16_t j = (idx+1) % RAW_RING_MAX;
    for (uint16_t ahead=0; ahead<RAW_RING_MAX; ahead++, j=(j+1)%RAW_RING_MAX) {
      const RawEvt &r = g_rawRing[j];
      if (r.ts_ms == 0) break;
      if (r.ts_ms < winStart) continue;
      if (r.ts_ms > winEnd)   break;
      if (!r.tx && r.cmd==163 && r.len>0) {
        // prostý mapping “první nalezená hodnota” => další volné místo 1..5
        for (uint8_t pos=1; pos<=5; pos++) {
          if (m.valForFix[pos] == 0) { m.valForFix[pos] = r.data[0]; found++; break; }
        }
        break;
      }
    }
    if (found >= 5) break;
  }
  m.have = (found >= 2);
  out = m;
  return m.have;
}

static void HvacRawAnalyzer_dump(Print &out, uint16_t maxRows=64) {
  out.println(F("--- RAW RING (nejnovější nahoře) ---"));
  uint16_t head = g_rawHead;
  uint16_t idx  = (head==0)? (RAW_RING_MAX-1) : (head-1);
  for (uint16_t i=0; i<RAW_RING_MAX && i<maxRows; i++) {
    const RawEvt &e = g_rawRing[idx];
    if (e.ts_ms == 0) break;
    out.printf("#%03u t=%lu %s cmd=%u len=%u data=",
               i, (unsigned long)e.ts_ms, e.tx?"TX":"RX", e.cmd, e.len);
    for (uint8_t k=0;k<e.len;k++) { out.printf("%s%u", (k?",":""), e.data[k]); }
    out.print("\r\n");
    idx = (idx==0)? (RAW_RING_MAX-1) : (idx-1);
  }
}

static void HvacRawAnalyzer_stats(Print &out) {
  out.println(F("--- CMD STATS ---"));
  for (uint8_t i=0;i<g_cmdStatsUsed;i++){
    const auto &s = g_cmdStats[i];
    if (s.seen==0) continue;
    out.printf("cmd=%u seen=%lu hot_bins:", s.cmd, (unsigned long)s.seen);
    uint8_t printed=0;
    for (int v=0; v<64 && printed<8; v++){       // <<< 64
      if (s.vcount[v]>0){
        out.printf(" %d(%lu)", v, (unsigned long)s.vcount[v]);
        printed++;
      }
    }
    out.print("\r\n");
  }
}

// ---------- /HVAC RAW ANALYZER ----------

// ================== HVAC TELEMETRY HELPERS ==================
static const char* hvacOperatingState() {
  const char* st = hvac.getState();        // "on"/"off"
  if (!st || !strcmp(st, "off")) return "off";
  const char* m = hvac.getMode();          // "auto"|"cool"|"heat"|"dry"|"fan_only"
  return (m && *m) ? m : "auto";
}

static bool hvacReadRoomTemp(float& tC) {
  hvacStatus st = hvac.getStatus();
  if (st.roomTemperature == INT8_MIN) { tC = NAN; return false; }
  tC = (float)st.roomTemperature;
  return true;
}

static bool hvacReadOutdoorTemp(float& tC) {
  hvacStatus st = hvac.getStatus();
  if (st.outsideTemperature == INT8_MIN) { tC = NAN; return false; }
  tC = (float)st.outsideTemperature;
  return true;
}

static uint16_t hvacReadFlags() { return 0; }
static uint16_t hvacReadErrorCode() { return 0xFFFF; }

// ================== REV/hash pro auto-refresh ==================
static uint16_t modeToReg(const char* m);
static uint16_t fanToReg(const char* f);
static uint16_t swingToReg(const char* s);

// --- Shadow (požadované) hodnoty pro okamžitý export do Modbusu
// Pending writes are separate from the confirmed HVAC state.
struct PendingHreg {
  uint16_t value = 0;
  uint32_t startedMs = 0;
  bool active = false;
};
static PendingHreg g_pendingHreg[5];       // HREG_POWER through HREG_SWING
static const uint32_t MODBUS_CONFIRM_TIMEOUT_MS = 10000UL;
static void rememberRequestedHreg(uint16_t reg, uint16_t value) {
  if (reg > HREG_SWING) return;
  g_pendingHreg[reg].value = value;
  g_pendingHreg[reg].startedMs = millis();
  g_pendingHreg[reg].active = true;
}
static uint16_t exportedHreg(uint16_t reg, uint16_t actual) {
  if (reg > HREG_SWING) return actual;
  PendingHreg &p = g_pendingHreg[reg];
  if (!p.active) return actual;
  if (actual == p.value || millis() - p.startedMs >= MODBUS_CONFIRM_TIMEOUT_MS) {
    p.active = false;
    return actual;
  }
  return p.value;
}
static void setHoldingFromHvac(uint16_t reg, uint16_t value) {
  if (mb.Hreg(reg) != value) mb.Hreg(reg, value);
  hregCache[reg] = value; // local updates must NOT be interpreted as Modbus writes
}

static uint32_t computeStateHash() {
  uint32_t h = 2166136261u;
  auto mix = [&](uint32_t v){ h ^= v; h *= 16777619u; };

  mix((uint32_t)hvac.getSetpoint());
  mix((uint32_t)modeToReg(nz(hvac.getMode())));
  mix((uint32_t)fanToReg(nz(hvac.getFanMode())));
  mix((uint32_t)swingToReg(nz(hvac.getSwing())));
  mix((uint32_t)(strcmp(nz(hvac.getState()),"on")==0));

  float tin, tout;
  bool hasTin  = hvacReadRoomTemp(tin);
  bool hasTout = hvacReadOutdoorTemp(tout);
  mix(hasTin  && !isnan(tin)  ? (uint32_t)lroundf(tin*10)  : 0xFFFFFFFFu);
  mix(hasTout && !isnan(tout) ? (uint32_t)lroundf(tout*10) : 0xFFFFFFFFu);

  mix((uint32_t)hvacReadFlags());
  mix((uint32_t)hvacReadErrorCode());
  return h;
}

static void bumpRevIfChanged() {
  uint32_t h = computeStateHash();
  if (h != g_lastHash) { g_lastHash = h; ++g_stateRev; }
}

// ================== HVAC command parsing ==================
// ================== HVAC command parsing ==================
static bool processCommand(const char* cmd) {
  if (!cmd || !*cmd || !commandAllowedByCapabilities(cmd)) return false;

  // Pseudo-kódy pro TX logování (drž je unikátní mimo reálné cmd)
  static const uint8_t PSEUDO_CMD_POWER = 200;
  static const uint8_t PSEUDO_CMD_TEMP  = 201;
  static const uint8_t PSEUDO_CMD_MODE  = 202;
  static const uint8_t PSEUDO_CMD_FAN   = 203;
  // Pozn.: SWING používá reálný cmd=163 kvůli heuristice mapování fix pozic

  // POWER
  if (!strcasecmp(cmd, "STATE_ON"))  {
    uint8_t v = 1;
    HvacRawAnalyzer_onFrame(true, PSEUDO_CMD_POWER, &v, 1);
    hvac.setState("on");
    rememberRequestedHreg(HREG_POWER, 1);
    // Request queued; last valid UART reception is tracked by the HVAC driver.
    bumpRevIfChanged();
    return true;
  }
  if (!strcasecmp(cmd, "STATE_OFF")) {
    uint8_t v = 0;
    HvacRawAnalyzer_onFrame(true, PSEUDO_CMD_POWER, &v, 1);
    hvac.setState("off");
    rememberRequestedHreg(HREG_POWER, 0);
    // Request queued; last valid UART reception is tracked by the HVAC driver.
    bumpRevIfChanged();
    return true;
  }

  // TEMP_xx (17..30)
  if (!strncasecmp(cmd, "TEMP_", 5)) {
    char* end = nullptr;
    const long t = strtol(cmd + 5, &end, 10);
    if (end == cmd + 5 || !end || *end != '\0' || t < 17 || t > 30) {
      Serial.println(F("TEMP: expected integer 17..30"));
      return false;
    }
    // TX log (payload = cílová teplota)
    uint8_t tv = (uint8_t)t;
    HvacRawAnalyzer_onFrame(true, PSEUDO_CMD_TEMP, &tv, 1);

    hvac.setSetpoint((uint8_t)t);
    rememberRequestedHreg(HREG_SETPOINT, (uint16_t)t);
    // Request queued; last valid UART reception is tracked by the HVAC driver.
    bumpRevIfChanged();
    return true;
  }

  // MODE_xxx
  if (!strncasecmp(cmd, "MODE_", 5)) {
    const char* p = cmd + 5;
    const char* m = nullptr;
    if      (!strcasecmp(p, "AUTO"))     m = "auto";
    else if (!strcasecmp(p, "COOL"))     m = "cool";
    else if (!strcasecmp(p, "HEAT"))     m = "heat";
    else if (!strcasecmp(p, "DRY"))      m = "dry";
    else if (!strcasecmp(p, "FAN"))      m = "fan_only";
    else if (!strcasecmp(p, "FAN_ONLY")) m = "fan_only";
    else { Serial.println(F("MODE: nepodporovany alias")); return false; }

    // TX log (payload = náš mapovaný kód režimu)
    uint8_t mv = (uint8_t)modeToReg(m);
    HvacRawAnalyzer_onFrame(true, PSEUDO_CMD_MODE, &mv, 1);

    hvac.setMode(m);
    rememberRequestedHreg(HREG_MODE, modeToReg(m));
    // Request queued; last valid UART reception is tracked by the HVAC driver.
    bumpRevIfChanged();
    return true;
  }

  // FAN_xxx (0=auto, 1..5)
  if (!strncasecmp(cmd, "FAN_", 4)) {
    const char* p = cmd + 4;
    const char* f = nullptr;
    if (!strcasecmp(p, "AUTO"))        f = "auto";
    else if (!strcasecmp(p, "QUIET"))  f = "lvl_1";
    else if (!strcasecmp(p, "LOW"))    f = "lvl_1";
    else if (!strcasecmp(p, "MEDIUM")) f = "lvl_3";
    else if (!strcasecmp(p, "HIGH"))   f = "lvl_5";
    else if (!strcasecmp(p, "TURBO"))  f = "lvl_5";
    else if (p[0]>='1' && p[0]<='5' && p[1]==0) {
      static const char* const levels[] = {"lvl_1","lvl_2","lvl_3","lvl_4","lvl_5"};
      f = levels[p[0] - '1'];
    } else {
      Serial.println(F("FAN: nepodporovany alias"));
      return false;
    }

    // TX log (payload = náš mapovaný kód ventilátoru)
    uint8_t fv = (uint8_t)fanToReg(f);
    HvacRawAnalyzer_onFrame(true, PSEUDO_CMD_FAN, &fv, 1);

    hvac.setFanMode(f);
    rememberRequestedHreg(HREG_FAN, fanToReg(f));
    // Request queued; last valid UART reception is tracked by the HVAC driver.
    bumpRevIfChanged();
    return true;
  }

  // SWING (reálný cmd=163 – logujeme TX; RX hook případně pasivně/snifferem)
  if (!strcasecmp(cmd, "SWING_OFF")) {
    uint8_t v = 0;   // marker pro FIX
    HvacRawAnalyzer_onFrame(true, 163, &v, 1);
    hvac.setSwing("fix");
    rememberRequestedHreg(HREG_SWING, swingToReg("fix"));
    bumpRevIfChanged();
    return true;
  }
  if (!strcasecmp(cmd, "SWING_V"))   {
    uint8_t v = 100; // marker pro V-swing
    HvacRawAnalyzer_onFrame(true, 163, &v, 1);
    hvac.setSwing("v_swing");
    rememberRequestedHreg(HREG_SWING, swingToReg("v_swing"));
    bumpRevIfChanged();
    return true;
  }
  if (!strcasecmp(cmd, "SWING_H"))   {
    uint8_t v = 101; // marker pro H-swing
    HvacRawAnalyzer_onFrame(true, 163, &v, 1);
    hvac.setSwing("h_swing");
    rememberRequestedHreg(HREG_SWING, swingToReg("h_swing"));
    bumpRevIfChanged();
    return true;
  }
  if (!strcasecmp(cmd, "SWING_VH"))  {
    uint8_t v = 102; // marker pro VH-swing
    HvacRawAnalyzer_onFrame(true, 163, &v, 1);
    hvac.setSwing("hv_swing");
    rememberRequestedHreg(HREG_SWING, swingToReg("hv_swing"));
    bumpRevIfChanged();
    return true;
  }
  if (!strncasecmp(cmd, "SWING_FIX_", 10)) {
    const char* p = cmd + 10; // "1".."5"
    if (p[0]>='1' && p[0]<='5' && p[1]==0) {
      uint8_t pos = (uint8_t)(p[0]-'0');            // 1..5
      HvacRawAnalyzer_onFrame(true, 163, &pos, 1);  // TX pro heuristiku
      static const char* const fixed[] = {
        "fix_pos_1","fix_pos_2","fix_pos_3","fix_pos_4","fix_pos_5"
      };
      hvac.setSwing(fixed[pos - 1]);
      rememberRequestedHreg(HREG_SWING, swingToReg(fixed[pos - 1]));
      bumpRevIfChanged();
    return true;
    }
    Serial.println(F("SWING: nepodporovany alias"));
    return false;
  }
    // FACTORY RESET (nevratné!) – použij pouze pokud MQTT je důvěryhodné
  if (!strcasecmp(cmd, "FACTORY_RESET") || !strcasecmp(cmd, "RESET_FACTORY")) {
    Serial.println(F("Factory reset denied via generic HVAC command"));
    return false;
  }

  Serial.print(F("Neznamy prikaz: ")); Serial.println(cmd);
  return false;
}


/*
 * DŮLEŽITÉ (doporučení pro lepší odhad mapy fix1..5):
 *  V knihovně ToshibaCarrierHvac přidej při PŘÍJMU rámce s cmd=163 ještě:
 *      HvacRawAnalyzer_onFrame(false, 163, payload, plen);
 *  -> analyzer pak uvidí i RX “echo/ack” a funkce HvacRawAnalyzer_guessSwingFixMap()
 *     bude schopná přiřazovat reálné hodnoty fix pozic.
 */


#include "HomeAssistantDiscovery.h"

// ================== MQTT ==================
static void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (!topic || !payload || !length || length >= 64) {
    Serial.println(F("MQTT: invalid or oversized command rejected")); return;
  }
  char cmd[64];
  memcpy(cmd, payload, length);
  cmd[length] = '\0';
  char baseTopic[96];
  expandTopic(deviceConfig.mqttCmdTopic, baseTopic, sizeof(baseTopic));
  if (!strcmp(topic, baseTopic)) {
    if (!processCommand(cmd)) Serial.printf("MQTT: rejected command: %s\n",cmd);
  } else if (!haCommand(topic,cmd)) {
    Serial.println(F("MQTT: nonmatching or invalid Home Assistant command"));
  }
}

static bool isMqttHostConfigured() {
  return deviceConfig.mqttServer[0] != '\0';
}

static void configureMqttClient() {
  #if (USE_MQTT_TLS)
    mqttClient.setClient(wifiClientTLS);
  #else
    mqttClient.setClient(wifiClientPlain);
  #endif
  mqttClient.setServer(deviceConfig.mqttServer, deviceConfig.mqttPort);
  // Bound MQTT CONNACK wait; an unresponsive broker must not starve UART.
  mqttClient.setSocketTimeout(3);
  if (!mqttClient.setBufferSize(2048)) Serial.println(F("MQTT: failed to allocate 2048 byte buffer"));
  mqttClient.setCallback(mqttCallback);
}

static void connectToMQTT() {
  if (!mqttEnabled || WiFi.status() != WL_CONNECTED || otaActive) return;
  if (mqttClient.connected()) return;
  if (!isMqttHostConfigured()) return;
  uint32_t now = millis();
  if (now - lastMqttAttempt < MQTT_RETRY_MS) return; // throttle
  lastMqttAttempt = now;

  char statusTopic[96];  expandTopic("toshiba/{id}/status", statusTopic, sizeof(statusTopic));
  char clientId[20];     snprintf(clientId, sizeof(clientId), "ac-%s", deviceID);

  Serial.print(F("MQTT připojení na ")); Serial.print(deviceConfig.mqttServer);
  Serial.print(F(":")); Serial.println(deviceConfig.mqttPort);

  bool ok = mqttClient.connect(
    clientId,
    deviceConfig.mqttUser[0] ? deviceConfig.mqttUser : nullptr,
    deviceConfig.mqttPassword[0] ? deviceConfig.mqttPassword : nullptr,
    statusTopic, 1, true, "offline"
  );

  if (ok) {
    mqttFailCount = 0;
    ++mqttReconnectCount;
    mqttClient.publish(statusTopic, "online", true);
    char cmdTopic[96]; expandTopic(deviceConfig.mqttCmdTopic, cmdTopic, sizeof(cmdTopic));
    mqttClient.subscribe(cmdTopic, 1);
    if (haPublishDiscovery(featureOptions.haDiscovery)) haDiscoveryEverPublished = featureOptions.haDiscovery;
    Serial.println(F("MQTT připojeno."));
  } else {
    ++mqttFailCount;
    Serial.print(F("MQTT chyba, state: ")); Serial.println(mqttClient.state());
    if (mqttFailCount >= 5) mqttFailCount = 0;
  }
}

static void publishState() {
  char topic[96]; expandTopic(deviceConfig.mqttStateTopic, topic, sizeof(topic));
  StaticJsonDocument<448> doc;

  doc["power"] = nz(hvac.getState());
  doc["temp"]  = (int)hvac.getSetpoint();
  doc["mode"]  = nz(hvac.getMode());
  doc["fan"]   = nz(hvac.getFanMode());
  doc["swing"] = nz(hvac.getSwing());

  // UART/komunikace s jednotkou
  doc["hvac"]  = hvac.isConnected() ? "Active" : "Inactive";

  // doplněná telemetrie
  doc["oper"]  = hvacOperatingState();   // off/auto/cool/heat/dry/fan_only

  doc["id"]   = deviceID;
  if (deviceConfig.room[0]) doc["room"] = deviceConfig.room;
  if (deviceLabel[0])       doc["name"] = deviceLabel;

  float tin, tout;
  if (hvacReadRoomTemp(tin)  && !isnan(tin))  doc["t_in"]  = tin;
  if (hvacReadOutdoorTemp(tout) && !isnan(tout)) doc["t_out"] = tout;

  uint16_t flags = hvacReadFlags();
  uint16_t err   = hvacReadErrorCode();
  if (flags) doc["flags"] = flags;
  if (err != 0xFFFF) doc["err"] = err;

  size_t need = measureJson(doc);
  String payload; payload.reserve(need + 8);
  serializeJson(doc, payload);
  Serial.print(F("Publikovaný stav: ")); Serial.println(payload);

  if (mqttEnabled && mqttClient.connected()) {
    if (!mqttClient.publish(topic, reinterpret_cast<const uint8_t*>(payload.c_str()),payload.length(),false))
      Serial.printf("[MQTT] publish failed (%u bytes)\n",(unsigned)payload.length());
  }
  bumpRevIfChanged();
}

// ================== Modbus: mapování helpery ==================
// ====== CUSTOM MODE MAP (HREG_MODE) ======
// Nastav požadované kódy pro Modbus HREG_MODE:
#define MODE_CODE_AUTO      1
#define MODE_CODE_COOL      3
#define MODE_CODE_HEAT      2
#define MODE_CODE_DRY       4
#define MODE_CODE_FAN_ONLY  5

// ====== CUSTOM FAN MAP (HREG_FAN) ======
// Nastav požadované kódy pro Modbus HREG_FAN:
#define FAN_CODE_AUTO   1
#define FAN_CODE_LVL_1  2
#define FAN_CODE_LVL_2  3
#define FAN_CODE_LVL_3  4
#define FAN_CODE_LVL_4  5
#define FAN_CODE_LVL_5  6

// ====== CUSTOM SWING MAP (HREG_SWING) ======
// Nastav požadované kódy pro Modbus HREG_SWING:
#define SWING_CODE_FIX        8
#define SWING_CODE_V_SWING    7
#define SWING_CODE_H_SWING    6
#define SWING_CODE_VH_SWING   5
#define SWING_CODE_FIX_POS_1  11
#define SWING_CODE_FIX_POS_2  12
#define SWING_CODE_FIX_POS_3  13
#define SWING_CODE_FIX_POS_4  14
#define SWING_CODE_FIX_POS_5  15

// Pomocné konstanty/validátor
static const uint16_t kModeCodes[] = {
  MODE_CODE_AUTO, MODE_CODE_COOL, MODE_CODE_HEAT, MODE_CODE_DRY, MODE_CODE_FAN_ONLY
};

// Seznam povolených kódů + validátor
static const uint16_t kSwingCodes[] = {
  SWING_CODE_FIX, SWING_CODE_V_SWING, SWING_CODE_H_SWING, SWING_CODE_VH_SWING,
  SWING_CODE_FIX_POS_1, SWING_CODE_FIX_POS_2, SWING_CODE_FIX_POS_3,
  SWING_CODE_FIX_POS_4, SWING_CODE_FIX_POS_5
};

static uint16_t modeCodeMax() {
  uint16_t m = 0;
  for (size_t i=0;i<sizeof(kModeCodes)/sizeof(kModeCodes[0]);++i)
    if (kModeCodes[i] > m) m = kModeCodes[i];
  return m;
}

static bool isValidModeCode(uint16_t v){
  for (size_t i=0;i<sizeof(kModeCodes)/sizeof(kModeCodes[0]);++i)
    if (kModeCodes[i] == v) return true;
  return false;
}

static bool isValidSwingCode(uint16_t v){
  for (size_t i=0;i<sizeof(kSwingCodes)/sizeof(kSwingCodes[0]);++i)
    if (kSwingCodes[i] == v) return true;
  return false;
}

// Pomůcky/validátor
static const uint16_t kFanCodes[] = {
  FAN_CODE_AUTO, FAN_CODE_LVL_1, FAN_CODE_LVL_2,
  FAN_CODE_LVL_3, FAN_CODE_LVL_4, FAN_CODE_LVL_5
};

static bool isValidFanCode(uint16_t v){
  for (size_t i=0;i<sizeof(kFanCodes)/sizeof(kFanCodes[0]);++i)
    if (kFanCodes[i] == v) return true;
  return false;
}

static uint16_t modeToReg(const char* m) {
  if (!m) return MODE_CODE_AUTO;
  if (!strcmp(m,"auto"))     return MODE_CODE_AUTO;
  if (!strcmp(m,"cool"))     return MODE_CODE_COOL;
  if (!strcmp(m,"heat"))     return MODE_CODE_HEAT;
  if (!strcmp(m,"dry"))      return MODE_CODE_DRY;
  if (!strcmp(m,"fan_only")) return MODE_CODE_FAN_ONLY;
  return MODE_CODE_AUTO;
}

static const char* regToMode(uint16_t r) {
  if (r == MODE_CODE_AUTO)     return "auto";
  if (r == MODE_CODE_COOL)     return "cool";
  if (r == MODE_CODE_HEAT)     return "heat";
  if (r == MODE_CODE_DRY)      return "dry";
  if (r == MODE_CODE_FAN_ONLY) return "fan_only";
  return "auto";
}

// 0=auto, 1..5 = lvl_1..lvl_5  (quiet mapujeme na lvl_1)
static uint16_t fanToReg(const char* f) {
  if (!f) return FAN_CODE_AUTO;
  if (!strcmp(f,"auto"))  return FAN_CODE_AUTO;
  if (!strcmp(f,"quiet")) return FAN_CODE_LVL_1; // alias
  if (!strcmp(f,"lvl_1")) return FAN_CODE_LVL_1;
  if (!strcmp(f,"lvl_2")) return FAN_CODE_LVL_2;
  if (!strcmp(f,"lvl_3")) return FAN_CODE_LVL_3;
  if (!strcmp(f,"lvl_4")) return FAN_CODE_LVL_4;
  if (!strcmp(f,"lvl_5")) return FAN_CODE_LVL_5;
  return FAN_CODE_AUTO;
}

// Převod kódu z HREG_FAN na text pro knihovnu
static const char* regToFan(uint16_t r) {
  if (r == FAN_CODE_AUTO)   return "auto";
  if (r == FAN_CODE_LVL_1)  return "lvl_1";
  if (r == FAN_CODE_LVL_2)  return "lvl_2";
  if (r == FAN_CODE_LVL_3)  return "lvl_3";
  if (r == FAN_CODE_LVL_4)  return "lvl_4";
  if (r == FAN_CODE_LVL_5)  return "lvl_5";
  return "auto";
}

static uint16_t swingToReg(const char* s) {
  if (!s) return SWING_CODE_FIX;
  if (!strcmp(s,"fix"))        return SWING_CODE_FIX;
  if (!strcmp(s,"v_swing"))    return SWING_CODE_V_SWING;
  if (!strcmp(s,"h_swing"))    return SWING_CODE_H_SWING;
  if (!strcmp(s,"hv_swing") || !strcmp(s,"vh_swing")) return SWING_CODE_VH_SWING;
  if (!strcmp(s,"fix_pos_1"))  return SWING_CODE_FIX_POS_1;
  if (!strcmp(s,"fix_pos_2"))  return SWING_CODE_FIX_POS_2;
  if (!strcmp(s,"fix_pos_3"))  return SWING_CODE_FIX_POS_3;
  if (!strcmp(s,"fix_pos_4"))  return SWING_CODE_FIX_POS_4;
  if (!strcmp(s,"fix_pos_5"))  return SWING_CODE_FIX_POS_5;
  return SWING_CODE_FIX;
}

static const char* regToSwing(uint16_t r) {
  if (r == SWING_CODE_FIX)        return "fix";
  if (r == SWING_CODE_V_SWING)    return "v_swing";
  if (r == SWING_CODE_H_SWING)    return "h_swing";
  if (r == SWING_CODE_VH_SWING)   return "hv_swing";
  if (r == SWING_CODE_FIX_POS_1)  return "fix_pos_1";
  if (r == SWING_CODE_FIX_POS_2)  return "fix_pos_2";
  if (r == SWING_CODE_FIX_POS_3)  return "fix_pos_3";
  if (r == SWING_CODE_FIX_POS_4)  return "fix_pos_4";
  if (r == SWING_CODE_FIX_POS_5)  return "fix_pos_5";
  return "fix";
}

static uint16_t buildStatusReg() {
  uint16_t s = 0;
  // bit0 = HVAC UART linka / běžící CDU (dle knihovny)
  if (hvac.isConnected()) s |= (1 << 0);
  // bit1 = MQTT připojeno
  if (mqttEnabled && mqttClient.connected()) s |= (1 << 1);
  // bit2 = Wi-Fi připojeno
  if (WiFi.status() == WL_CONNECTED) s |= (1 << 2);
  return s;
}

// mapování "oper" do registru (včetně OFF)
static uint16_t operToReg(const char* oper) {
  if (!oper) return 0;
  if (!strcmp(oper, "off")) return 0;
  return modeToReg(oper);
}

// převod teploty na x10 int16 (32767 = N/A)
static int16_t tempToX10_orNA(float tC, bool ok) {
  if (!ok || isnan(tC)) return 32767;
  long v = lroundf(tC * 10.0f);
  if (v < -32768) v = -32768;
  if (v >  32767) v =  32767;
  return (int16_t)v;
}

static void modbusSyncFromHVAC() {
  if (!modbusActive) return;
  const uint16_t power = !strcmp(nz(hvac.getState()), "on") ? 1 : 0;
  setHoldingFromHvac(HREG_POWER, exportedHreg(HREG_POWER, power));
  uint16_t setpoint = hvac.getSetpoint();
  if (setpoint < 17) setpoint = 17;
  if (setpoint > 30) setpoint = 30;
  setHoldingFromHvac(HREG_SETPOINT, exportedHreg(HREG_SETPOINT, setpoint));
  setHoldingFromHvac(HREG_MODE, exportedHreg(HREG_MODE, modeToReg(hvac.getMode())));
  setHoldingFromHvac(HREG_FAN, exportedHreg(HREG_FAN, fanToReg(hvac.getFanMode())));
  setHoldingFromHvac(HREG_SWING, exportedHreg(HREG_SWING, swingToReg(hvac.getSwing())));

  setHoldingFromHvac(HREG_OPER, operToReg(hvacOperatingState()));
  float tin=NAN, tout=NAN;
  const bool tinOk=hvacReadRoomTemp(tin), toutOk=hvacReadOutdoorTemp(tout);
  const int16_t in = tempToX10_orNA(tin, tinOk);
  const int16_t out = tempToX10_orNA(tout, toutOk);
  setHoldingFromHvac(HREG_TIN_X10, static_cast<uint16_t>(in));
  setHoldingFromHvac(HREG_TOUT_X10, static_cast<uint16_t>(out));
  setHoldingFromHvac(HREG_FLAGS, hvacReadFlags());
  setHoldingFromHvac(HREG_ERROR, hvacReadErrorCode());
  setHoldingFromHvac(HREG_STATUS, buildStatusReg());
  setHoldingFromHvac(HREG_UPTIME_S, (uint16_t)((millis()/1000UL) & 0xFFFF));
}

static void modbusApplyWrites() {
  if (!modbusActive) return;
  uint16_t v = mb.Hreg(HREG_POWER);
  if (v != hregCache[HREG_POWER]) {
    if (v <= 1) {
      ++modbusClientWrites;
      hvac.setState(v ? "on" : "off");
      rememberRequestedHreg(HREG_POWER, v);
      hregCache[HREG_POWER] = v;
    } else { ++modbusRejectedWrites; mb.Hreg(HREG_POWER, hregCache[HREG_POWER]); }
  }

  v = mb.Hreg(HREG_SETPOINT);
  if (v != hregCache[HREG_SETPOINT]) {
    if (v >= 17 && v <= 30) {
      ++modbusClientWrites;
      hvac.setSetpoint((uint8_t)v);
      rememberRequestedHreg(HREG_SETPOINT, v);
      hregCache[HREG_SETPOINT] = v;
    } else { ++modbusRejectedWrites; mb.Hreg(HREG_SETPOINT, hregCache[HREG_SETPOINT]); }
  }

  v = mb.Hreg(HREG_MODE);
  if (v != hregCache[HREG_MODE]) {
    if (isValidModeCode(v)) {
      ++modbusClientWrites;
      hvac.setMode(regToMode(v));
      rememberRequestedHreg(HREG_MODE, v);
      hregCache[HREG_MODE] = v;
    } else { ++modbusRejectedWrites; mb.Hreg(HREG_MODE, hregCache[HREG_MODE]); }
  }

  v = mb.Hreg(HREG_FAN);
  if (v != hregCache[HREG_FAN]) {
    if (isValidFanCode(v)) {
      ++modbusClientWrites;
      hvac.setFanMode(regToFan(v));
      rememberRequestedHreg(HREG_FAN, v);
      hregCache[HREG_FAN] = v;
    } else { ++modbusRejectedWrites; mb.Hreg(HREG_FAN, hregCache[HREG_FAN]); }
  }

  v = mb.Hreg(HREG_SWING);
  if (v != hregCache[HREG_SWING]) {
    if (isValidSwingCode(v)) {
      ++modbusClientWrites;
      hvac.setSwing(regToSwing(v));
      rememberRequestedHreg(HREG_SWING, v);
      hregCache[HREG_SWING] = v;
    } else { ++modbusRejectedWrites; mb.Hreg(HREG_SWING, hregCache[HREG_SWING]); }
  }
}

static void modbusSetup() {
  if (!deviceConfig.modbusEnable) { modbusActive = false; return; }
  if (modbusInitializedOnce) { modbusActive = true; modbusSyncFromHVAC(); return; }

  // Stabilní režim – naslouchat na implicitním portu 502 (nezpřístupňujeme změnu portu)
  mb.server();  // TCP 502

  for (uint16_t r = 0; r < HREG__COUNT; ++r) {
    mb.addHreg(r, 0);
  }
  for (uint16_t reg=100; reg<=112; ++reg) mb.addHreg(reg,0);
  modbusInitializedOnce = true;
  modbusActive = true;
  modbusSyncFromHVAC();
  for (uint16_t r = 0; r < HREG__COUNT; ++r) {
    hregCache[r] = mb.Hreg(r);
  }
  Serial.printf("Modbus TCP server ON (port 502, unit %u - info only)\n", deviceConfig.modbusUnitId);
}

static void updateModbusDiagnostics() {
  if (!modbusActive) return;
  // Read-only in application logic: refreshed on each heartbeat.
  mb.Hreg(100, (uint16_t)(hvac.validPackets() & 0xFFFFu));
  mb.Hreg(101, (uint16_t)(hvac.invalidPackets() & 0xFFFFu));
  mb.Hreg(102, (uint16_t)(hvac.handshakeAttempts() & 0xFFFFu));
  const uint32_t lastRx=hvac.lastValidRxMs();
  mb.Hreg(103, lastRx ? (uint16_t)(((millis()-lastRx)/1000UL)>65534UL?65534UL:(millis()-lastRx)/1000UL) : 65535);
  mb.Hreg(104, (uint16_t)(int16_t)(WiFi.status()==WL_CONNECTED ? WiFi.RSSI() : -127));
  mb.Hreg(105, (uint16_t)min((uint32_t)65535, (uint32_t)ESP.getFreeHeap()/1024u));
  mb.Hreg(106, (uint16_t)(mqttReconnectCount & 0xFFFF));
  mb.Hreg(107, (uint16_t)(modbusClientWrites & 0xFFFF));
  mb.Hreg(108, (uint16_t)(modbusRejectedWrites & 0xFFFF));
  mb.Hreg(109, featureOptions.haDiscovery);
  mb.Hreg(110, (uint16_t)hvac.isConnected());
  uint16_t supported=0,unsupported=0;
  for(uint8_t i=0;i<CAP_COUNT;i++){
    if(featureOptions.capability[i]==CAP_SUPPORTED)supported|=(1u<<i);
    if(featureOptions.capability[i]==CAP_UNSUPPORTED)unsupported|=(1u<<i);
  }
  mb.Hreg(111,supported); mb.Hreg(112,unsupported);
}

// ================== Web ==================
#include "WebAssets.h"

static bool handleAuthentication() {
  if (!server.authenticate(deviceConfig.webUser, deviceConfig.webPassword)) {
    server.requestAuthentication();
    return false;
  }
  return true;
}

static void handleAppCss() {
  if (!handleAuthentication()) return;
  server.sendHeader("Cache-Control","no-cache");
  server.send(200, "text/css", FPSTR(APP_CSS));
}

static void handleAppJs() {
  if (!handleAuthentication()) return;
  server.sendHeader("Cache-Control","no-cache");
  server.send(200, "application/javascript", FPSTR(APP_JS));
}

// Root dashboard: page is stored in flash, not built in heap String.
#include "DashboardAssets.h"
static void handleRoot() {
  if (!handleAuthentication()) return;
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "text/html; charset=utf-8", FPSTR(DASHBOARD_HTML));
}


// Feature API is separated from legacy EEPROM DeviceConfig.
static void handleLocaleCs() {
  if (!handleAuthentication()) return;
  server.sendHeader("Cache-Control","public, max-age=3600");
  server.send(200,"application/json; charset=utf-8",FPSTR(LOCALE_CS));
}
static void handleLocaleEn() {
  if (!handleAuthentication()) return;
  server.sendHeader("Cache-Control","public, max-age=3600");
  server.send(200,"application/json; charset=utf-8",FPSTR(LOCALE_EN));
}

static void handleConfigBackup() {
  if (!handleAuthentication()) return;
  DynamicJsonDocument doc(1536);
  doc["schema"] = 1;
  doc["device_id"] = deviceID;
  doc["server"] = deviceConfig.mqttServer;
  doc["port"] = deviceConfig.mqttPort;
  doc["mqtt_en"] = deviceConfig.mqttEnable;
  doc["user"] = deviceConfig.mqttUser;
  doc["cmdtopic"] = deviceConfig.mqttCmdTopic;
  doc["statetopic"] = deviceConfig.mqttStateTopic;
  doc["hbtopic"] = deviceConfig.mqttHeartbeatTopic;
  doc["webuser"] = deviceConfig.webUser;
  doc["dev_id_en"] = deviceConfig.useCustomId;
  doc["dev_id"] = deviceConfig.customDeviceId;
  doc["room"] = deviceConfig.room;
  doc["name"] = deviceConfig.unitName;
  doc["mb_en"] = deviceConfig.modbusEnable;
  doc["ota_en"] = deviceConfig.otaEnable;
  doc["telnet_en"] = deviceConfig.telnetEnable;
  doc["ha_discovery"] = featureOptions.haDiscovery;
  JsonObject cap = doc.createNestedObject("capabilities");
  for (uint8_t i=0;i<CAP_COUNT;++i)
    cap[CAPABILITY_KEYS[i]] = capabilityStateName(featureOptions.capability[i]);
  String out;serializeJson(doc,out);
  server.sendHeader("Content-Disposition", "attachment; filename=toshiba-settings.json");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200,"application/json",out);
}
static void handleConfigRestore() {
  if (!handleAuthentication()) return;
  if (!validCsrf()) { server.send(403,"text/plain","CSRF token required");return; }
  if (!server.hasArg("plain") || server.arg("plain").length() > 3000) {
    server.send(400,"text/plain","Missing or oversized JSON body");return;
  }
  DynamicJsonDocument doc(2048);
  auto error=deserializeJson(doc,server.arg("plain"));
  if (error || doc["schema"] != 1) {
    server.send(400,"text/plain","Unsupported backup schema");return;
  }
  DeviceConfig next=deviceConfig;
  FeatureOptions nextFeatures=featureOptions;
  const char* strKeys[]={"server","user","cmdtopic","statetopic","hbtopic","webuser","dev_id","room","name"};
  char* destinations[]={next.mqttServer,next.mqttUser,next.mqttCmdTopic,
    next.mqttStateTopic,next.mqttHeartbeatTopic,next.webUser,next.customDeviceId,
    next.room,next.unitName};
  const size_t limits[]={sizeof(next.mqttServer),sizeof(next.mqttUser),sizeof(next.mqttCmdTopic),
    sizeof(next.mqttStateTopic),sizeof(next.mqttHeartbeatTopic),sizeof(next.webUser),
    sizeof(next.customDeviceId),sizeof(next.room),sizeof(next.unitName)};
  for (size_t i=0;i<sizeof(strKeys)/sizeof(strKeys[0]);++i) {
    if (!doc.containsKey(strKeys[i])) continue;
    if (!doc[strKeys[i]].is<const char*>()) {
      server.send(400,"text/plain","Invalid text field");return;
    }
    const char* value=doc[strKeys[i]].as<const char*>();
    if (strlen(value) >= limits[i]) {
      server.send(400,"text/plain","Backup field too long");return;
    }
    strlcpy(destinations[i],value,limits[i]);
  }
  const char* names[]={"port","mqtt_en","dev_id_en","mb_en","ota_en","telnet_en","ha_discovery"};
  for (const char* name:names) {
    if (!doc.containsKey(name)) continue;
    if (!doc[name].is<int>() || doc[name].as<int>() < 0) {
      server.send(400,"text/plain","Invalid numeric field");return;
    }
  }
  if (doc.containsKey("port")) {
    const int p=doc["port"].as<int>();
    if (p < 1 || p > 65535) {server.send(400,"text/plain","Invalid MQTT port");return;}
    next.mqttPort=(uint16_t)p;
  }
  if (doc.containsKey("mqtt_en")) next.mqttEnable=doc["mqtt_en"].as<int>()!=0;
  if (doc.containsKey("dev_id_en")) next.useCustomId=doc["dev_id_en"].as<int>()!=0;
  if (doc.containsKey("mb_en")) next.modbusEnable=doc["mb_en"].as<int>()!=0;
  if (doc.containsKey("ota_en")) next.otaEnable=doc["ota_en"].as<int>()!=0;
  if (doc.containsKey("telnet_en")) next.telnetEnable=doc["telnet_en"].as<int>()!=0;
  if (doc.containsKey("ha_discovery")) nextFeatures.haDiscovery=doc["ha_discovery"].as<int>()!=0;
  if (doc.containsKey("capabilities")) {
    JsonObject caps=doc["capabilities"].as<JsonObject>();
    if (caps.isNull()) {server.send(400,"text/plain","Invalid capabilities");return;}
    for (uint8_t i=0;i<CAP_COUNT;i++) {
      if (!caps.containsKey(CAPABILITY_KEYS[i]))continue;
      const char* value=caps[CAPABILITY_KEYS[i]];
      if (!value) {server.send(400,"text/plain","Invalid capability");return;}
      if (!strcmp(value,"auto"))nextFeatures.capability[i]=CAP_AUTO;
      else if (!strcmp(value,"supported"))nextFeatures.capability[i]=CAP_SUPPORTED;
      else if (!strcmp(value,"unsupported"))nextFeatures.capability[i]=CAP_UNSUPPORTED;
      else {server.send(400,"text/plain","Invalid capability setting");return;}
    }
  }
  sanitizeDeviceId(next.customDeviceId);
  deviceConfig=next;
  featureOptions=nextFeatures;
  saveConfig();
  saveFeatureOptions();
  featureDirty=true;
  computeDeviceID();computeDeviceLabel();
  mqttEnabled=deviceConfig.mqttEnable!=0;
  if (mqttClient.connected()) mqttClient.disconnect();
  if (mqttEnabled && isMqttHostConfigured()) configureMqttClient();
  // Reboot only after response; user can inspect restored settings before restart.
  server.send(200,"application/json","{\"restored\":true,\"restart_recommended\":true}");
}

static void handleSecurityToken() {
  if (!handleAuthentication()) return;
  StaticJsonDocument<80> doc;doc["csrf"] = csrfToken;
  String response;serializeJson(doc,response);
  server.sendHeader("Cache-Control","no-store");
  server.send(200,"application/json",response);
}
static void handleFeaturesGet() {
  if (!handleAuthentication()) return;
  StaticJsonDocument<768> doc;
  doc["ha_discovery"] = featureOptions.haDiscovery != 0;
  doc["ha_available"] = mqttEnabled && mqttClient.connected();
  JsonObject capabilities=doc.createNestedObject("capabilities");
  JsonObject observed=doc.createNestedObject("observed");
  const char* swing = nz(hvac.getSwing());
  const char* pure = nz(hvac.getPure());
  const char* psel = nz(hvac.getPowerSelect());
  const char* op = nz(hvac.getOperation());
  for (uint8_t i=0;i<CAP_COUNT;i++) {
    capabilities[CAPABILITY_KEYS[i]]=capabilityStateName(featureOptions.capability[i]);
    bool known = false;
    switch (i) {
      case CAP_SWING_FIXED: known = !strcmp(swing,"fix");break;
      case CAP_SWING_VERTICAL: known = !strcmp(swing,"v_swing");break;
      case CAP_SWING_HORIZONTAL: known = !strcmp(swing,"h_swing");break;
      case CAP_SWING_BOTH: known = !strcmp(swing,"hv_swing");break;
      case CAP_SWING_POSITIONS: known = !strncmp(swing,"fix_pos_",8);break;
      case CAP_PURE: known = strcmp(pure,"UNKNOWN") && *pure;break;
      case CAP_POWER_SELECT: known = strcmp(psel,"UNKNOWN") && *psel;break;
      case CAP_OPERATION: known = strcmp(op,"UNKNOWN") && *op;break;
      case CAP_WIFI_LED: known = strcmp(nz(hvac.getWifiLed()),"UNKNOWN") != 0 && *nz(hvac.getWifiLed());break;
    }
    observed[CAPABILITY_KEYS[i]]=known;
  }
  String out;
  serializeJson(doc,out);
  server.sendHeader("Cache-Control","no-store");
  server.send(200,"application/json; charset=utf-8",out);
}
static void handleFeaturesSave() {
  if (!handleAuthentication()) return;
  if (!validCsrf()) {
    server.send(403,"text/plain","Invalid request");return;
  }
  FeatureOptions next=featureOptions;
  if (server.hasArg("ha_discovery")) {
    String val=server.arg("ha_discovery");
    if (val != "0" && val != "1") { server.send(400,"text/plain","Invalid HA option");return; }
    next.haDiscovery=val=="1";
  }
  for (uint8_t i=0;i<CAP_COUNT;i++) {
    if (!server.hasArg(CAPABILITY_KEYS[i])) continue;
    const String val=server.arg(CAPABILITY_KEYS[i]);
    if (val=="auto") next.capability[i]=CAP_AUTO;
    else if (val=="supported") next.capability[i]=CAP_SUPPORTED;
    else if (val=="unsupported") next.capability[i]=CAP_UNSUPPORTED;
    else { server.send(400,"text/plain","Invalid capability"); return; }
  }
  featureOptions=next;
  saveFeatureOptions();
  featureDirty=true;
  server.send(200,"application/json","{\"saved\":true}");
}
static void handleRestart() {
  if (!handleAuthentication()) return;
  if (!validCsrf() ||
      !server.hasArg("confirm") || server.arg("confirm") != "YES") {
    server.send(403,"text/plain","Restart requires explicit confirmation");
    return;
  }
  restartRequestedMs = millis();
  server.send(202,"application/json","{\"restarting\":true}");
}

static void handleCommand() {
  if (!handleAuthentication()) return;
  if (!validCsrf()) {
    server.send(403,"text/plain","Invalid request"); return;
  }
  if (server.hasArg("cmd")) {
    String c = server.arg("cmd");
    c.trim();
    if (c.length() == 0 || c.length() >= 64) {
      server.send(400, "text/plain", "Neplatná délka příkazu");
      return;
    }
    char tmp[64];
    c.toCharArray(tmp, sizeof(tmp));
    const bool accepted = processCommand(tmp);
    server.send(accepted ? 200 : 400, "text/plain",
                accepted ? String("Příkaz přijat: ") + c : String("Neplatný příkaz: ") + c);
  } else {
    server.send(400, "text/plain", "Chybí parametr cmd");
  }
}

static void handleMQTTConfig() {
  if (!handleAuthentication()) return;
  if (!validCsrf()) {
    server.send(403,"text/plain","Invalid request"); return;
  }

  if (server.hasArg("mqtt_en")) deviceConfig.mqttEnable = (uint8_t)server.arg("mqtt_en").toInt();
  if (server.hasArg("server")) server.arg("server").toCharArray(deviceConfig.mqttServer, sizeof(deviceConfig.mqttServer));
  if (server.hasArg("port"))   deviceConfig.mqttPort = (uint16_t)server.arg("port").toInt();
  if (server.hasArg("user"))   server.arg("user").toCharArray(deviceConfig.mqttUser, sizeof(deviceConfig.mqttUser));
  if (server.hasArg("pass") && server.arg("pass").length())   server.arg("pass").toCharArray(deviceConfig.mqttPassword, sizeof(deviceConfig.mqttPassword));
  if (server.hasArg("cmdtopic")) server.arg("cmdtopic").toCharArray(deviceConfig.mqttCmdTopic, sizeof(deviceConfig.mqttCmdTopic));
  if (server.hasArg("statetopic")) server.arg("statetopic").toCharArray(deviceConfig.mqttStateTopic, sizeof(deviceConfig.mqttStateTopic));
  if (server.hasArg("hbtopic")) server.arg("hbtopic").toCharArray(deviceConfig.mqttHeartbeatTopic, sizeof(deviceConfig.mqttHeartbeatTopic));

  if (server.hasArg("webuser")) server.arg("webuser").toCharArray(deviceConfig.webUser, sizeof(deviceConfig.webUser));
  if (server.hasArg("webpass") && server.arg("webpass").length()) server.arg("webpass").toCharArray(deviceConfig.webPassword, sizeof(deviceConfig.webPassword));

  // OTA
  if (server.hasArg("ota_en"))   deviceConfig.otaEnable = (uint8_t)server.arg("ota_en").toInt();
  if (server.hasArg("ota_pass") && server.arg("ota_pass").length()) server.arg("ota_pass").toCharArray(deviceConfig.otaPassword, sizeof(deviceConfig.otaPassword));

  // Device ID
  if (server.hasArg("dev_id_en")) deviceConfig.useCustomId = (uint8_t)server.arg("dev_id_en").toInt();
  if (server.hasArg("dev_id")) {
    server.arg("dev_id").toCharArray(deviceConfig.customDeviceId, sizeof(deviceConfig.customDeviceId));
    sanitizeDeviceId(deviceConfig.customDeviceId);
  }
  // Identifikace (místnost + název)
  if (server.hasArg("room"))
    server.arg("room").toCharArray(deviceConfig.room, sizeof(deviceConfig.room));
  if (server.hasArg("name"))
    server.arg("name").toCharArray(deviceConfig.unitName, sizeof(deviceConfig.unitName));

  // Modbus
  if (server.hasArg("mb_en"))   deviceConfig.modbusEnable = (uint8_t)server.arg("mb_en").toInt();
  // mb_port/mb_uid jsou jen informativní – port neměníme

  // Telnet
  if (server.hasArg("telnet_en"))   deviceConfig.telnetEnable = (uint8_t)server.arg("telnet_en").toInt();
  if (server.hasArg("telnet_pass") && server.arg("telnet_pass").length()) server.arg("telnet_pass").toCharArray(deviceConfig.telnetPassword, sizeof(deviceConfig.telnetPassword));

  saveConfig();

  // Aplikace změn bez resetu:
  computeDeviceID();
  computeDeviceLabel();

  mqttEnabled = deviceConfig.mqttEnable != 0;
  if (mqttClient.connected()) mqttClient.disconnect();
  if (mqttEnabled && isMqttHostConfigured()) configureMqttClient();

  // OTA runtime přepnutí
  if (deviceConfig.otaEnable && !otaStarted) {
    char host[24]; snprintf(host, sizeof(host), "toshiba-%s", deviceID);
    ArduinoOTA.setHostname(host);
    if (deviceConfig.otaPassword[0]) { ArduinoOTA.setPassword(deviceConfig.otaPassword); }
    ArduinoOTA.onStart([](){ Serial.println(F("OTA start")); otaActive = true; otaStartMs = millis(); });
    ArduinoOTA.onEnd([](){ Serial.println(F("OTA end")); otaActive = false; if (RED_LED_PIN >= 0) analogWrite(RED_LED_PIN, 0); });
    ArduinoOTA.onProgress([](unsigned int p, unsigned int t){ uint32_t pct = (t ? (p*100U/t) : 0U); Serial.printf("OTA: %u%%\r\n", pct); });
    ArduinoOTA.onError([](ota_error_t e){ Serial.printf("OTA err %u\r\n", e); otaActive = false; if (RED_LED_PIN >= 0) analogWrite(RED_LED_PIN, 0); });
    ArduinoOTA.begin();
    otaStarted = true;
  } else if (!deviceConfig.otaEnable) {
    otaActive = false; // handle() se volat nebude
    otaStarted = false; // povolí případnou re-inicializaci po dalším zapnutí
  }

  // Restart Modbusu dle nastavení
  // Avoid repeatedly calling mb.server()/addHreg() for an already active server.
  if (deviceConfig.modbusEnable) {
    if (!modbusActive) modbusSetup();
  } else {
    modbusActive = false;
  }

  // Telnet runtime přepnutí
  telnetEnabled = (deviceConfig.telnetEnable != 0);
  if (!telnetEnabled && telnetClient) { telnetClient.stop(); }
  if (telnetEnabled) { telnetServer.begin(); telnetServer.setNoDelay(true); }

  String resp;
  resp.reserve(256);
  resp += F("<!doctype html><meta charset='utf-8'><title>OK</title><pre>Konfigurace aktualizována.\n</pre><a href='/'>Zpět</a>");
  server.send(200, "text/html", resp);
}

static void handleDebug() {
  if (!handleAuthentication()) return;

  const bool hvacConn = (hvac.isConnected() || hvac.isCduRunning());

  String dbg;
  dbg.reserve(1600);
  dbg += F("<!doctype html><meta charset='utf-8'><title>Debug</title><pre>");
  //dbg += F("Device ID: "); dbg += deviceID; dbg += deviceConfig.useCustomId ? " (custom)" : " (MAC)"; dbg += F("\n");
  dbg += F("Device: ");
  dbg += htmlEscape(deviceLabel[0] ? deviceLabel : "(unnamed)");
  dbg += F(" | ID: "); dbg += deviceID; dbg += deviceConfig.useCustomId ? " (custom)" : " (MAC)";
  dbg += F("\nRoom: ");
  dbg += htmlEscape(deviceConfig.room[0] ? deviceConfig.room : "(n/a)");
  dbg += F("\n");
  dbg += F("WiFi status: "); dbg += (WiFi.status()==WL_CONNECTED?"Connected":"Not connected"); dbg += F("\n");
  dbg += F("MQTT: "); dbg += (deviceConfig.mqttEnable ? (mqttClient.connected() ? "Enabled & Connected" : "Enabled, not connected") : "Disabled"); dbg += F("\n");
  dbg += F("MQTT Host: "); dbg += (deviceConfig.mqttServer[0] ? deviceConfig.mqttServer : "(empty)"); dbg += F(":"); dbg += deviceConfig.mqttPort; dbg += F("\n");
  dbg += F("MQTT Fail Count (session): "); dbg += mqttFailCount; dbg += F("\n");
  dbg += F("IP: "); dbg += WiFi.localIP().toString(); dbg += F("\n");
  dbg += F("HVAC: "); dbg += (hvacConn ? "Active" : "Inactive"); dbg += F("\n");
  dbg += F("Modbus: "); dbg += (deviceConfig.modbusEnable ? (modbusActive ? "Enabled (running)" : "Enabled (not running)") : "Disabled");
  dbg += F(" (port 502, unit "); dbg += deviceConfig.modbusUnitId; dbg += F(" - info only)\n");
  dbg += F("OTA: "); dbg += (deviceConfig.otaEnable ? "Enabled" : "Disabled"); dbg += F(", pass: ");
  dbg += (deviceConfig.otaPassword[0] ? "(set)" : "(empty)"); dbg += F("\n");
  dbg += F("Telnet: "); dbg += (deviceConfig.telnetEnable ? "Enabled" : "Disabled");
  dbg += deviceConfig.telnetPassword[0] ? " (password set)\n" : " (no password)\n";
  dbg += F("UART handshake attempts: "); dbg += hvac.handshakeAttempts(); dbg += F("\n");
  dbg += F("UART RX accepted batches: "); dbg += hvac.validPackets(); dbg += F("\n");
  dbg += F("UART RX rejected batches: "); dbg += hvac.invalidPackets(); dbg += F("\n");
  dbg += F("UART last valid RX (ms): "); dbg += hvac.lastValidRxMs(); dbg += F("\n");
  dbg += F("RAW: tx="); dbg += g_txFrames;
  dbg += F(" rx="); dbg += g_rxFrames;
  dbg += F(" last_ms="); dbg += g_lastRawMs; dbg += F("\n");
  dbg += F("</pre><a href='/'>Zpět</a> | <a href='/service'>Servis HVAC</a>");
  server.send(200, "text/html", dbg);
}


// ================== HVAC Service console ==================
// NOTE: Deliberately does not modify HVAC serial timing or raw UART TX/RX.
#include "ServiceAssets.h"

struct ServiceSwingTest {
  bool active = false;
  uint32_t startedMs = 0;
  uint32_t receivedAtStart = 0;
  char requested[16] = {0};
  char before[16] = {0};
  const char* observation = "unknown"; // RAM only, reset at boot
};
static ServiceSwingTest serviceSwingTest;
static uint32_t serviceLastRefreshRequestMs = 0;
static const uint32_t SERVICE_SWING_TEST_MS = 8000UL;
static const uint32_t SERVICE_REFRESH_MIN_MS = 30000UL;

static const char* serviceKnownText(const char* p) {
  return (p && *p) ? p : "UNKNOWN";
}

static void serviceAddSettings(JsonObject dst, const hvacSettings& s) {
  dst["state"] = serviceKnownText(s.state);
  if (s.setpoint >= 17 && s.setpoint <= 30) dst["setpoint"] = s.setpoint;
  else dst["setpoint"] = nullptr;
  dst["mode"] = serviceKnownText(s.mode);
  dst["fan"] = serviceKnownText(s.fanMode);
  dst["swing"] = serviceKnownText(s.swing);
  dst["pure"] = serviceKnownText(s.pure);
  dst["powerSelect"] = serviceKnownText(s.powerSelect);
  dst["operation"] = serviceKnownText(s.operation);
  dst["wifiLed"] = serviceKnownText(s.wifiLed);
}

// All setters below pass permanent string literals to the HVAC library,
// because ToshibaCarrierHvac stores const char* pointers (not String copies).
static bool serviceSetProperty(const String& field, const String& value) {
  if (field == "power") {
    if (value == "on") return processCommand("STATE_ON");
    if (value == "off") return processCommand("STATE_OFF");
    return false;
  }
  if (field == "setpoint") {
    if (value.length() < 2 || value.length() > 2) return false;
    for (unsigned int i=0; i<value.length(); ++i) {
      if (value[i] < '0' || value[i] > '9') return false;
    }
    const int t = value.toInt();
    if (t < 17 || t > 30) return false;
    char cmd[12];
    snprintf(cmd, sizeof(cmd), "TEMP_%d", t);
    return processCommand(cmd);
  }
  if (field == "mode") {
    if (value == "auto") return processCommand("MODE_AUTO");
    if (value == "cool") return processCommand("MODE_COOL");
    if (value == "heat") return processCommand("MODE_HEAT");
    if (value == "dry") return processCommand("MODE_DRY");
    if (value == "fan_only") return processCommand("MODE_FAN_ONLY");
    return false;
  }
  if (field == "fan") {
    if (value == "auto") return processCommand("FAN_AUTO");
    if (value == "quiet") return processCommand("FAN_QUIET");
    if (value == "lvl_1") return processCommand("FAN_1");
    if (value == "lvl_2") return processCommand("FAN_2");
    if (value == "lvl_3") return processCommand("FAN_3");
    if (value == "lvl_4") return processCommand("FAN_4");
    if (value == "lvl_5") return processCommand("FAN_5");
    return false;
  }
  if (field == "swing") {
    if (value == "fix") return processCommand("SWING_OFF");
    if (value == "v_swing") return processCommand("SWING_V");
    if (value == "h_swing") return processCommand("SWING_H");
    if (value == "hv_swing") return processCommand("SWING_VH");
    if (value == "fix_pos_1") return processCommand("SWING_FIX_1");
    if (value == "fix_pos_2") return processCommand("SWING_FIX_2");
    if (value == "fix_pos_3") return processCommand("SWING_FIX_3");
    if (value == "fix_pos_4") return processCommand("SWING_FIX_4");
    if (value == "fix_pos_5") return processCommand("SWING_FIX_5");
    return false;
  }
  if (field == "pure") {
    if (!capabilityEnabled(CAP_PURE)) return false;
    if (value == "on") hvac.setPure("on");
    else if (value == "off") hvac.setPure("off");
    else return false;
  } else if (field == "powerSelect") {
    if (!capabilityEnabled(CAP_POWER_SELECT)) return false;
    if (value == "50%") hvac.setPowerSelect("50%");
    else if (value == "75%") hvac.setPowerSelect("75%");
    else if (value == "100%") hvac.setPowerSelect("100%");
    else return false;
  } else if (field == "operation") {
    if (!capabilityEnabled(CAP_OPERATION)) return false;
    if (value == "normal") hvac.setOperation("normal");
    else if (value == "high_power") hvac.setOperation("high_power");
    else if (value == "silent_1") hvac.setOperation("silent_1");
    else if (value == "silent_2") hvac.setOperation("silent_2");
    else if (value == "eco") hvac.setOperation("eco");
    else if (value == "eight_deg") hvac.setOperation("eight_deg");
    else if (value == "fireplace_1") hvac.setOperation("fireplace_1");
    else if (value == "fireplace_2") hvac.setOperation("fireplace_2");
    else return false;
  } else if (field == "wifiLed") {
    if (!capabilityEnabled(CAP_WIFI_LED)) return false;
    if (value == "on") hvac.setWifiLed("on");
    else if (value == "off") hvac.setWifiLed("off");
    else return false;
  } else {
    return false;
  }
  // Advanced features are queued by the original UART driver's sync loop.
  bumpRevIfChanged();
  return true;
}

static void handleServicePage() {
  if (!handleAuthentication()) return;
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/html; charset=utf-8", FPSTR(SERVICE_HTML));
}

static void handleServiceJs() {
  if (!handleAuthentication()) return;
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/javascript; charset=utf-8", FPSTR(SERVICE_JS));
}

static void handleServiceState() {
  if (!handleAuthentication()) return;
  // Dynamic allocation avoids adding a 3-4kB JSON object to the ESP8266 stack.
  DynamicJsonDocument doc(4096);
  const uint32_t now = millis();
  doc["connected"] = hvac.isConnected();
  doc["build"] = __DATE__ " " __TIME__;
  doc["uptime_ms"] = now;
  doc["id"] = deviceID;

  serviceAddSettings(doc.createNestedObject("actual"), hvac.getSettings());
  serviceAddSettings(doc.createNestedObject("requested"), hvac.getRequestedSettings());
  serviceAddSettings(doc.createNestedObject("sent"), hvac.getLastSentSettings());

  hvacStatus st = hvac.getStatus();
  JsonObject telemetry = doc.createNestedObject("telemetry");
  if (st.roomTemperature != INT8_MIN) telemetry["room_temp_c"] = st.roomTemperature;
  else telemetry["room_temp_c"] = nullptr;
  if (st.outsideTemperature != INT8_MIN && st.outsideTemperature != 127)
    telemetry["outdoor_temp_c"] = st.outsideTemperature;
  else telemetry["outdoor_temp_c"] = nullptr;
  telemetry["on_timer"] = serviceKnownText(st.onTimer);
  telemetry["off_timer"] = serviceKnownText(st.offTimer);
  telemetry["cdu_running"] = st.running;

  JsonObject diag = doc.createNestedObject("diag");
  diag["rx_ok"] = hvac.validPackets();
  diag["rx_invalid"] = hvac.invalidPackets();
  diag["handshakes"] = hvac.handshakeAttempts();
  uint32_t lastRx = hvac.lastValidRxMs();
  if (lastRx) diag["last_rx_age_ms"] = now - lastRx;
  else diag["last_rx_age_ms"] = nullptr;
  diag["wifi_connected"] = WiFi.status() == WL_CONNECTED;
  if (WiFi.status() == WL_CONNECTED) diag["rssi_dbm"] = WiFi.RSSI();
  else diag["rssi_dbm"] = nullptr;
  diag["mqtt_connected"] = mqttEnabled && mqttClient.connected();
  diag["modbus_active"] = modbusActive;
  diag["modbus_writes"] = modbusClientWrites;
  diag["modbus_invalid_writes"] = modbusRejectedWrites;
  diag["mqtt_reconnects"] = mqttReconnectCount;
  diag["free_heap"] = ESP.getFreeHeap();
  diag["sniff_enabled"] = g_sniffEnabled;
  diag["raw_tx"] = g_txFrames;
  diag["raw_rx"] = g_rxFrames;

  JsonObject test = doc.createNestedObject("swing_test");
  test["active"] = serviceSwingTest.active;
  if (serviceSwingTest.active) {
    const char* actual = serviceKnownText(hvac.getSwing());
    const bool freshRx = hvac.validPackets() > serviceSwingTest.receivedAtStart;
    const uint32_t elapsed = now - serviceSwingTest.startedMs;
    const char* result = "pending";
    if (elapsed >= SERVICE_SWING_TEST_MS) {
      if (!freshRx) result = "no_rx";
      else if (strcmp(actual, serviceSwingTest.requested) == 0) {
        result = strcmp(serviceSwingTest.before, actual) == 0 ? "already" : "reported";
      } else result = "unconfirmed";
    }
    test["before"] = serviceSwingTest.before;
    test["target"] = serviceSwingTest.requested;
    test["actual"] = actual;
    test["elapsed_ms"] = elapsed;
    test["status"] = result;
    test["observation"] = serviceSwingTest.observation;
  }

  if (doc.overflowed()) {
    server.send(500, "text/plain", "Service JSON memory overflow");
    return;
  }
  String out;
  out.reserve(measureJson(doc) + 8);
  serializeJson(doc, out);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", out);
}

static void handleServiceAction() {
  if (!handleAuthentication()) return;
  if (!validCsrf()) {
    server.send(403, "text/plain", "Missing X-Requested-With header");
    return;
  }
  if (!server.hasArg("action")) {
    server.send(400, "text/plain", "Missing action");
    return;
  }
  String action = server.arg("action");
  String field  = server.hasArg("field") ? server.arg("field") : "";
  String value  = server.hasArg("value") ? server.arg("value") : "";
  if (action.length() > 20 || field.length() > 20 || value.length() > 24) {
    server.send(400, "text/plain", "Value too long");
    return;
  }

  if (action == "sniff") {
    if (value == "on") HvacRawAnalyzer_enable(true);
    else if (value == "off") HvacRawAnalyzer_enable(false);
    else if (value == "clear") HvacRawAnalyzer_reset();
    else { server.send(400, "text/plain", "Invalid sniff command"); return; }
    server.send(200, "text/plain", "RAW log updated");
    return;
  }
  if (action == "observe_swing") {
    if (!serviceSwingTest.active) {
      server.send(409, "text/plain", "Start a swing test first"); return;
    }
    if (value == "yes") {
      serviceSwingTest.observation = "yes";
      if (featureOptions.capability[CAP_SWING_POSITIONS] == CAP_AUTO) {
        featureOptions.capability[CAP_SWING_POSITIONS] = CAP_SUPPORTED;
        saveFeatureOptions();
        featureDirty = true;
      }
    }
    else if (value == "no") serviceSwingTest.observation = "no";
    else if (value == "unknown") serviceSwingTest.observation = "unknown";
    else { server.send(400, "text/plain", "Invalid observation"); return; }
    server.send(200, "text/plain", "Observation recorded in RAM");
    return;
  }
  if (!hvac.isConnected()) {
    server.send(409, "text/plain", "HVAC UART is not connected");
    return;
  }
  if (action == "refresh") {
    const uint32_t now = millis();
    if (serviceLastRefreshRequestMs &&
        now - serviceLastRefreshRequestMs < SERVICE_REFRESH_MIN_MS) {
      server.send(429, "text/plain", "Refresh limited to one per 30 seconds");
      return;
    }
    serviceLastRefreshRequestMs = now;
    hvac.forceQueryAllData();
    server.send(202, "text/plain", "Full query requested; executes with original HVAC timing");
    return;
  }
  if (action == "set") {
    if (!serviceSetProperty(field, value)) {
      server.send(400, "text/plain", "Unsupported property or invalid value"); return;
    }
    server.send(202, "text/plain", "Command queued; wait for HVAC-reported state");
    return;
  }
  if (action == "test_swing") {
    if (strcmp(serviceKnownText(hvac.getState()), "on") != 0) {
      server.send(409, "text/plain", "Turn HVAC on before testing swing positions");
      return;
    }
    if (value != "fix_pos_1" && value != "fix_pos_2" && value != "fix_pos_3" &&
        value != "fix_pos_4" && value != "fix_pos_5") {
      server.send(400, "text/plain", "Invalid test position"); return;
    }
    // Taking the initial snapshot before dispatch allows readback comparison.
    ServiceSwingTest pending;
    pending.active = true;
    pending.startedMs = millis();
    pending.receivedAtStart = hvac.validPackets();
    strlcpy(pending.before, serviceKnownText(hvac.getSwing()), sizeof(pending.before));
    strlcpy(pending.requested, value.c_str(), sizeof(pending.requested));
    if (!serviceSetProperty("swing", value)) {
      server.send(400, "text/plain", "Failed to queue swing command");
      return;
    }
    serviceSwingTest = pending;
    server.send(202, "text/plain", "Swing test started; check state and actual movement after 8s");
    return;
  }
  server.send(400, "text/plain", "Unknown service action");
}


// ================== Telnet ==================
static void printTelnetHelp() {
  telnetClient.println(F("=== Toshiba HVAC Control Telnet ==="));
  telnetClient.println(F("help         - napoveda"));
  telnetClient.println(F("status       - stav wifi/mqtt/hvac/modbus"));
  telnetClient.println(F("test_led     - bliknout LED"));
  telnetClient.println(F("ota          - info k OTA"));
  telnetClient.println(F("reset        - restart MCU"));
  telnetClient.println(F("HVAC prikazy: STATE_ON, STATE_OFF, TEMP_, MODE_, FAN_, SWING_*"));
  telnetClient.println(F("-- raw sniff --"));
  telnetClient.println(F("sniff on     - zapnout logovani cmd/payload"));
  telnetClient.println(F("sniff off    - vypnout logovani"));
  telnetClient.println(F("sniff clear  - smazat buffer/statistiky"));
  telnetClient.println(F("sniff dump   - vypsat posledni udalosti"));
  telnetClient.println(F("sniff stats  - statistiky hodnot (data[0]) podle cmd"));
  telnetClient.println(F("sniff guess  - heuristicky odhad mapy SWING_FIX_1..5 (z RX)"));
}

static void handleTelnet() {
  if (!telnetEnabled) return;

  if (!telnetClient || !telnetClient.connected()) {
    WiFiClient c = telnetServer.available();
    if (c) {
      telnetClient.stop();
      telnetClient = c;
      telnetClient.setNoDelay(true);
      telnetAuthenticated = false;
      if (deviceConfig.telnetPassword[0]) {
        telnetClient.println(F("Telnet (password protected)"));
        telnetClient.print(F("Password: "));
      } else {
        telnetClient.println(F("Vitejte v Telnet Debug"));
        printTelnetHelp();
        telnetAuthenticated = true;
        telnetClient.print(F("> "));
      }
    }
    return;
  }

  while (telnetClient.available()) {
    static char line[96];
    static uint8_t idx = 0;
    char ch = telnetClient.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      line[idx] = 0;
      idx = 0;

      if (!telnetAuthenticated) {
        if (!strcmp(line, deviceConfig.telnetPassword)) {
          telnetAuthenticated = true;
          telnetClient.println(F("\r\nOK"));
          printTelnetHelp();
          telnetClient.print(F("> "));
        } else {
          telnetClient.println(F("\r\nBad password."));
          telnetClient.stop();
        }
        continue;
      }

      if (!line[0]) { telnetClient.print(F("> ")); continue; }

      // ---- příkazy ----
      if (!strcasecmp(line, "help")) {
        printTelnetHelp();

      } else if (!strcasecmp(line, "status")) {
        const bool hvacConn = (hvac.isConnected() || hvac.isCduRunning());
        float tin, tout;
        bool hasTin  = hvacReadRoomTemp(tin);
        bool hasTout = hvacReadOutdoorTemp(tout);
        uint16_t flags = hvacReadFlags();
        uint16_t err   = hvacReadErrorCode();

        telnetClient.printf("DeviceID: %s%s\r\n", deviceID, deviceConfig.useCustomId ? " (custom)" : " (MAC)");
        telnetClient.printf("Name: %s\r\n", deviceLabel[0] ? deviceLabel : "(unnamed)");
        telnetClient.printf("Room: %s\r\n", deviceConfig.room[0] ? deviceConfig.room : "(n/a)");
        telnetClient.printf("WiFi: %s\r\n", WiFi.status() == WL_CONNECTED ? "Connected" : "Not Connected");
        telnetClient.printf("IP: %s\r\n", WiFi.localIP().toString().c_str());
        telnetClient.printf("MQTT: %s\r\n", deviceConfig.mqttEnable ? (mqttClient.connected() ? "Enabled & Connected" : "Enabled, not connected") : "Disabled");
        telnetClient.printf("HVAC UART: %s\r\n", hvacConn ? "Active" : "Inactive");
        telnetClient.printf("HVAC UART handshake=%lu rx_ok=%lu rx_invalid=%lu last_rx_ms=%lu\r\n",
          (unsigned long)hvac.handshakeAttempts(),
          (unsigned long)hvac.validPackets(),
          (unsigned long)hvac.invalidPackets(),
          (unsigned long)hvac.lastValidRxMs());
        telnetClient.printf("Modbus TCP: %s (port 502, unit %u - info only)\r\n",
          deviceConfig.modbusEnable ? (modbusActive ? "Enabled (running)" : "Enabled (not running)") : "Disabled",
          deviceConfig.modbusUnitId);

        telnetClient.printf("Power: %s\r\n", nz(hvac.getState()));
        telnetClient.printf("Setpoint: %u C\r\n", (unsigned)hvac.getSetpoint());
        telnetClient.printf("Mode: %s\r\n", nz(hvac.getMode()));
        telnetClient.printf("Fan: %s\r\n", nz(hvac.getFanMode()));
        telnetClient.printf("Swing: %s\r\n", nz(hvac.getSwing()));
        telnetClient.printf("Operating: %s\r\n", hvacOperatingState());
        telnetClient.printf("Temp indoor: %s\r\n", (hasTin && !isnan(tin)) ? String(tin,1).c_str() : "N/A");
        telnetClient.printf("Temp outdoor: %s\r\n", (hasTout && !isnan(tout)) ? String(tout,1).c_str() : "N/A");
        telnetClient.printf("Flags: 0x%04X\r\n", flags);
        telnetClient.printf("Error: %s\r\n", (err == 0xFFFF ? "N/A" : String(err).c_str()));
        //telnetClient.printf("RAW: tx="); dbg += g_txFrames;
        //telnetClient.printf(" rx="); dbg += g_rxFrames;
        //telnetClient.printf(" last_ms="); dbg += g_lastRawMs; dbg += F("\n");
      } else if (!strcasecmp(line, "test_led")) {
        for (int i = 0; i < 3; ++i) {
          if (LED_PIN >= 0) { digitalWrite(LED_PIN, LOW); delay(150);
          digitalWrite(LED_PIN, HIGH); delay(150); }
        }
        telnetClient.println(F("OK"));

      } else if (!strcasecmp(line, "ota")) {
        telnetClient.printf("OTA: %s, pass:%s\r\n", deviceConfig.otaEnable ? "enabled" : "disabled", (deviceConfig.otaPassword[0] ? "set" : "empty"));

      } else if (!strcasecmp(line, "reset")) {
        telnetClient.println(F("Restartuji..."));
        telnetClient.flush();
        ESP.restart();

      // ---- RAW SNIFF ----
      } else if (!strcasecmp(line, "sniff on")) {
        HvacRawAnalyzer_enable(true);
        telnetClient.println(F("sniff: ON"));
      } else if (!strcasecmp(line, "sniff off")) {
        HvacRawAnalyzer_enable(false);
        telnetClient.println(F("sniff: OFF"));
      } else if (!strcasecmp(line, "sniff clear")) {
        HvacRawAnalyzer_reset();
        telnetClient.println(F("sniff: cleared"));
      } else if (!strcasecmp(line, "sniff dump")) {
        HvacRawAnalyzer_dump(telnetClient, 80);
      } else if (!strcasecmp(line, "sniff stats")) {
        HvacRawAnalyzer_stats(telnetClient);
      } else if (!strcasecmp(line, "sniff guess")) {
        SwingFixMap m;
        if (!HvacRawAnalyzer_guessSwingFixMap(m)) {
          telnetClient.println(F("guess: not enough evidence (dopln RX hook v knihovne)"));
        } else {
          telnetClient.printf("fix1=%u fix2=%u fix3=%u fix4=%u fix5=%u\r\n",
            m.valForFix[1], m.valForFix[2], m.valForFix[3], m.valForFix[4], m.valForFix[5]);
        }

      } else if (!strcasecmp(line, "factory_reset")) {
        telnetClient.println(F("Provadim factory reset a restart..."));
        telnetClient.flush();
        doFactoryReset();
      } else {
        telnetClient.println(processCommand(line) ? F("OK") : F("ERR: invalid command"));
      }

      telnetClient.print(F("> "));
    } else {
      if (idx < sizeof(line) - 1) line[idx++] = ch;
    }
  }
}

// ================== FLASH button: vyvolání WiFiManager portálu ==================
static void enterConfigPortal() {
  if (wifiManager.getConfigPortalActive()) return;
  Serial.println(F("[FLASH] Nonblocking WiFiManager portal requested"));
  // Stop port-80 listener while the captive portal owns it.
  if (networkServicesStarted) {
    server.stop();
    if (telnetClient) telnetClient.stop();
    telnetServer.stop();
    if (mqttClient.connected()) mqttClient.disconnect();
    modbusActive = false;
    networkServicesStarted = false;
  }
  wifiPortalManual = true;
  wifiPortalStartedMs = millis();
  wifiManager.setConfigPortalBlocking(false);
  wifiManager.setConfigPortalTimeout(300);
  char apName[40];
  snprintf(apName,sizeof(apName),"Toshiba-%s-HVAC-Setup",deviceID);
  // Unprotected AP intentionally retained by project policy.
  wifiManager.startConfigPortal(apName);
  if (!wifiManager.getConfigPortalActive()) {
    wifiPortalManual = false;
    wifiPortalLastClosedMs = millis();
  }
}

static void handleFlashButton() {
  static uint32_t tDown = 0;
  static uint32_t tDeb  = 0;
  static bool wasDown = false;

  uint32_t now = millis();
  bool rawDown = (digitalRead(FLASH_BTN_PIN) == LOW);
  static bool down = false;

  if (now - tDeb >= BTN_DEBOUNCE_MS) {
    if (rawDown != down) {
      down = rawDown;
      tDeb = now;
    }
  }

  if (down && !wasDown) { tDown = now; }
  if (!down && wasDown) {
    uint32_t dur = now - tDown;
    if (dur >= BTN_SHORT_MIN && dur <= BTN_SHORT_MAX) {
      enterConfigPortal();
    }
  }
  wasDown = down;
}

// >>> Boot RED LED – drží LED 5 s po startu (pokud neprobíhá OTA)
static void bootRedLedHandle() {
  if (RED_LED_PIN < 0) return;
  if (!bootRedActive) return;
  if (otaActive) return;            // OTA má prioritu

  uint32_t now = millis();
  if (now < bootRedUntil) {
    pwmWriteCompat(RED_LED_PIN, PWM_MAX);
  } else {
    bootRedActive = false;
    pwmWriteCompat(RED_LED_PIN, 0);
  }
}

// ================== OTA LED fade ==================
static void otaLedHandle() {
  if (!otaActive || (RED_LED_PIN < 0) || bootRedActive) return; // během boot-LED nefadeovat
  const uint32_t period = 1200;
uint32_t t = (millis() - otaStartMs) % period;
uint16_t val = (t < period/2)
               ? (uint16_t)((t * PWM_MAX) / (period/2))
               : (uint16_t)(((period - t) * PWM_MAX) / (period/2));
pwmWriteCompat(RED_LED_PIN, val);
}

// ======== FW UPDATE (upload .bin přes web) ========
static volatile bool fw_in_progress = false;
static String fw_result;

static void handleFwPage() {
  if (!handleAuthentication()) return;

  String html;
  html.reserve(5000);
  html += F("<!doctype html><html lang='cs'><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>FW Update</title>"
            "<link rel='stylesheet' href='/app.css'>"
            "</head><body><div class='container'>"
            "<div class='card' style='grid-column:span 12'>"
            "<h3>Aktualizace firmware</h3>"
            "<form method='POST' action='/fwupdate?csrf=");
  html += csrfToken;
  html += F("' enctype='multipart/form-data'>"
            "<input class='btn' type='file' name='fw' accept='.bin' required>"
            " <button class='btn primary' type='submit'>Nahrát a aktualizovat</button>"
            "</form>"
            "<div class='help' style='margin-top:8px'>Po úspěšném nahrání se zařízení samo restartuje.</div>"
            "</div>"
            "<div class='card' style='grid-column:span 12'>"
            "<h3>Servis</h3>"
            "<p><a class='link' href='/factory-reset'>Tovární reset…</a></p>"
            "</div>");
  html += F("<div style='margin-top:12px'><a class='link' href='/'>⟵ Zpět na hlavní stránku</a></div>");
  html += F("</div></body></html>");

  server.send(200, "text/html", html);
}

// callback z ESP8266WebServer při POST /fwupdate – streamuje data do Update.write()
static void handleFwUpload() {
  if (!handleAuthentication()) return;
  if (!server.hasArg("csrf") || server.arg("csrf") != csrfToken) {
    server.send(403,"text/plain","Invalid firmware upload token");return;
  }
  HTTPUpload &up = server.upload();

  switch (up.status) {
    case UPLOAD_FILE_START: {
  fw_in_progress = true;
  fw_result = "";

  if (mqttClient.connected()) mqttClient.disconnect();
  modbusActive = false;

  Serial.printf("FW: start '%s'\n", up.filename.c_str());

  bool okBegin = false;
  #if defined(ESP8266)
    size_t maxSketch = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
    Update.runAsync(true);
    okBegin = Update.begin(maxSketch);
  #elif defined(ESP32)
    okBegin = Update.begin(UPDATE_SIZE_UNKNOWN);
  #endif

      if (!okBegin) {
        Update.printError(Serial);
      }
    } break;

    case UPLOAD_FILE_WRITE: {
      if (Update.isRunning()) {
        if (Update.write(up.buf, up.currentSize) != up.currentSize) {
          Update.printError(Serial);
        }
      }
    } break;

    case UPLOAD_FILE_END: {
      if (Update.end(true)) {
        fw_result = "OK";
        Serial.printf("FW: success, %u bytes\n", up.totalSize);
      } else {
        Serial.print("FW: error end: ");
        Update.printError(Serial);
        fw_result = "ERROR";
      }
      fw_in_progress = false;
    } break;

    case UPLOAD_FILE_ABORTED: {
      Update.end();
      fw_in_progress = false;
      fw_result = "ABORT";
      Serial.println("FW: aborted.");
    } break;
  }
}

// finální odpověď po POST (zde už víme výsledek). Restart děláme až po odeslání stránky.
static void handleFwDone() {
  if (!handleAuthentication()) return;
  if (!server.hasArg("csrf") || server.arg("csrf") != csrfToken) {
    server.send(403,"text/plain","Invalid firmware upload token");return;
  }

  String html;
  html.reserve(2000);
  if (fw_result == "OK") {
    html += F("<!doctype html><meta charset='utf-8'><title>FW OK</title>"
              "<div class='container'><div class='card'><h3>FW nahrán</h3>"
              "<p>Proběhlo úspěšně. Zařízení se nyní restartuje…</p>"
              "<p><a class='link' href='/'>Zpět</a></p></div></div>");
    server.send(200, "text/html", html);
    server.client().stop();
    delay(250);
    ESP.restart();
  } else if (fw_result == "ABORT") {
    html += F("<!doctype html><meta charset='utf-8'><title>FW zrušen</title>"
              "<div class='container'><div class='card'><h3>Nahrávání zrušeno</h3>"
              "<p>Aktualizace byla zrušena.</p>"
              "<p><a class='link' href='/fw'>Zkusit znovu</a></p></div></div>");
    server.send(500, "text/html", html);
  } else if (fw_result == "ERROR") {
    html += F("<!doctype html><meta charset='utf-8'><title>FW chyba</title>"
              "<div class='container'><div class='card'><h3>Chyba při aktualizaci</h3>"
              "<p>Zkontroluj, že nahráváš správný <code>.bin</code> pro ESP8266 a že je dostatek volné flash.</p>"
              "<p>Detail chyby je v Serial logu.</p>"
              "<p><a class='link' href='/fw'>Zkusit znovu</a></p></div></div>");
    server.send(500, "text/html", html);
  } else {
    html += F("<!doctype html><meta charset='utf-8'><title>FW stav</title>"
              "<div class='container'><div class='card'><h3>Neznámý stav</h3>"
              "<p>Zkus prosím znovu nahrát firmware.</p>"
              "<p><a class='link' href='/fw'>Zpět</a></p></div></div>");
    server.send(500, "text/html", html);
  }
  if (fw_result != "OK") {
    // Restore traffic without re-adding already registered holding registers.
    modbusActive = deviceConfig.modbusEnable && modbusInitializedOnce;
    if (modbusActive) modbusSyncFromHVAC();
    lastMqttAttempt = 0;
    fw_in_progress = false;
  }
}

static void handleFactoryResetPage() {
  if (!handleAuthentication()) return;
  String html;
  html.reserve(3000);
  html += F("<!doctype html><meta charset='utf-8'><title>Factory Reset</title>"
            "<link rel='stylesheet' href='/app.css'>"
            "<div class='container'><div class='card'>"
            "<h3>Tovární nastavení</h3>"
            "<p>Tato akce <b>nevratně</b> smaže konfiguraci zařízení (EEPROM) a Wi-Fi přihlašovací údaje."
            " Po restartu se otevře konfigurační AP portál.</p>"
            "<form method='POST' action='/factory-reset'>"
            "<input type='hidden' name='csrf' value='");
  html += csrfToken;
  html += F("'>"
            "<button class='btn bad' type='submit' name='confirm' value='YES'>Provést Factory Reset</button>"
            " <a class='btn' href='/'>Zpět</a>"
            "</form>"
            "</div></div>");
  server.send(200, "text/html", html);
}

static void handleFactoryResetPost() {
  if (!handleAuthentication()) return;
  if (!server.hasArg("csrf") || server.arg("csrf") != csrfToken) {
    server.send(403,"text/plain","Invalid CSRF token");return;
  }
  if (server.hasArg("confirm") && server.arg("confirm") == "YES") {
    server.send(200, "text/plain", "Factory reset… restartuji.");
    server.client().stop();
    delay(200);
    doFactoryReset();
  } else {
    server.send(400, "text/plain", "Chybí potvrzení.");
  }
}

// Saving portal configuration is deferred until after WiFiManager.process(),
// outside its callback, so HVAC servicing remains the first loop activity.
static void applySavedPortalParameters() {
  if (!shouldSaveConfig) return;
  shouldSaveConfig = false;
  Serial.println(F("Applying WiFiManager parameters (EEPROM)..."));
  deviceConfig.mqttEnable = (uint8_t)atoi(p_mqtt_en.getValue());
  strlcpy(deviceConfig.mqttServer, p_mqtt_server.getValue(), sizeof(deviceConfig.mqttServer));
  deviceConfig.mqttPort = (uint16_t)atoi(p_mqtt_port.getValue());
  strlcpy(deviceConfig.mqttUser, p_mqtt_user.getValue(), sizeof(deviceConfig.mqttUser));
  if (p_mqtt_pass.getValue()[0])
    strlcpy(deviceConfig.mqttPassword, p_mqtt_pass.getValue(), sizeof(deviceConfig.mqttPassword));
  strlcpy(deviceConfig.mqttCmdTopic, p_mqtt_cmd.getValue(), sizeof(deviceConfig.mqttCmdTopic));
  strlcpy(deviceConfig.mqttStateTopic, p_mqtt_state.getValue(), sizeof(deviceConfig.mqttStateTopic));
  strlcpy(deviceConfig.mqttHeartbeatTopic, p_mqtt_hb.getValue(), sizeof(deviceConfig.mqttHeartbeatTopic));

  deviceConfig.useCustomId = (uint8_t)atoi(p_dev_id_en.getValue());
  strlcpy(deviceConfig.customDeviceId, p_dev_id.getValue(), sizeof(deviceConfig.customDeviceId));
  sanitizeDeviceId(deviceConfig.customDeviceId);
  strlcpy(deviceConfig.room, p_room.getValue(), sizeof(deviceConfig.room));
  strlcpy(deviceConfig.unitName, p_name.getValue(), sizeof(deviceConfig.unitName));

  deviceConfig.modbusEnable = (uint8_t)atoi(p_mb_enable.getValue());
  { // pouze info
    uint16_t mp = (uint16_t)atoi(p_mb_port.getValue());
    deviceConfig.modbusPort = (mp == 0 ? 502 : mp);
  }
  { // pouze info
    int uid = atoi(p_mb_uid.getValue());
    if (uid < 1) uid = 1; if (uid > 247) uid = 247;
    deviceConfig.modbusUnitId = (uint8_t)uid;
  }

  deviceConfig.otaEnable = (uint8_t)atoi(p_ota_en.getValue());
  if (p_ota_pass.getValue()[0])
    strlcpy(deviceConfig.otaPassword, p_ota_pass.getValue(), sizeof(deviceConfig.otaPassword));

  deviceConfig.telnetEnable = (uint8_t)atoi(p_telnet_en.getValue());
  if (p_telnet_pass.getValue()[0])
    strlcpy(deviceConfig.telnetPassword, p_telnet_pass.getValue(), sizeof(deviceConfig.telnetPassword));

  saveConfig();
  computeDeviceID();
  computeDeviceLabel();
  mqttEnabled = deviceConfig.mqttEnable != 0;
  if (mqttClient.connected()) mqttClient.disconnect();
  if (mqttEnabled && isMqttHostConfigured()) configureMqttClient();
  telnetEnabled = deviceConfig.telnetEnable != 0;
  Serial.printf("[BOOT %lu ms] WiFiManager config saved\n", (unsigned long)millis());
}

// Configure OTA only once and only when the network is ready. This also
// covers OTA enabled through the WiFiManager portal on the first boot.
static void startArduinoOTAAfterWiFi() {
  if (!deviceConfig.otaEnable || otaStarted) return;
  char host[24];
  snprintf(host, sizeof(host), "toshiba-%s", deviceID);
  ArduinoOTA.setHostname(host);
  if (deviceConfig.otaPassword[0]) ArduinoOTA.setPassword(deviceConfig.otaPassword);
  ArduinoOTA.onStart([]() { Serial.println(F("OTA start")); otaActive = true; otaStartMs = millis(); });
  ArduinoOTA.onEnd([]()   { Serial.println(F("OTA end")); otaActive = false; if (RED_LED_PIN >= 0) analogWrite(RED_LED_PIN, 0); });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) { uint32_t pct = (t ? (p * 100U / t) : 0U); Serial.printf("OTA: %u%%\r\n", pct); });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf("OTA err %u\r\n", e); otaActive = false; if (RED_LED_PIN >= 0) analogWrite(RED_LED_PIN, 0); });
  ArduinoOTA.begin();
  otaStarted = true;
  Serial.print(F("[WiFi] OTA ready, hostname: ")); Serial.println(host);
}

// Start listeners once STA has received an IP. Never compete with the
// WiFiManager captive portal HTTP server on port 80.
static void startNetworkServicesAfterWiFi() {
  if (networkServicesStarted || WiFi.status() != WL_CONNECTED ||
      wifiManager.getConfigPortalActive()) return;

  server.begin();
  Serial.println(F("[WiFi] Web server started (80)."));
  if (telnetEnabled) {
    telnetServer.begin();
    telnetServer.setNoDelay(true);
    Serial.println(F("[WiFi] Telnet server started (23)."));
  }
  startArduinoOTAAfterWiFi();
  modbusSetup();
  networkServicesStarted = true;
  Serial.printf("[BOOT %lu ms] HTTP/Modbus services ready: %s\n",
                (unsigned long)millis(), WiFi.localIP().toString().c_str());
}

static void startNonblockingConfigPortal() {
  if (wifiManager.getConfigPortalActive() || networkServicesStarted) return;
  wifiPortalStartedMs = millis();
  wifiPortalManual = false;
  wifiManager.setConfigPortalBlocking(false);
  wifiManager.setConfigPortalTimeout(300);
  char apName[40];
  snprintf(apName, sizeof(apName), "Toshiba-%s-HVAC-Setup", deviceID);
  Serial.printf("[BOOT %lu ms] Starting nonblocking AP: %s\n",
                (unsigned long)millis(), apName);
  wifiManager.startConfigPortal(apName);
  if (wifiManager.getConfigPortalActive()) {
    wifiHadPortal = true;
    Serial.printf("[WiFi] Config portal address: %s\n", WiFi.softAPIP().toString().c_str());
  } else {
    // Avoid tight-loop retry when the ESP cannot enable the AP.
    wifiPortalLastClosedMs = millis();
    Serial.println(F("[WiFi] Failed to start portal; retry later."));
  }
}

static void maintainWiFiBoot() {
  const uint32_t now = millis();
  const bool wasPortalActive = wifiManager.getConfigPortalActive();
  if (wasPortalActive) {
    wifiManager.process();
    // Enforce timeout also in nonblocking mode.
    if (wifiPortalStartedMs && millis() - wifiPortalStartedMs >= WIFI_PORTAL_MANUAL_LIMIT_MS) {
      wifiManager.stopConfigPortal();
      wifiPortalStartedMs = 0;
      wifiPortalManual = false;
      Serial.println(F("[WiFi] Captive portal timeout"));
    }
  }

  // Parameters were written by WiFiManager callback; commit outside callbacks.
  if (shouldSaveConfig) applySavedPortalParameters();

  const bool portalActive = wifiManager.getConfigPortalActive();
  if (wasPortalActive && !portalActive) {
    wifiPortalStartedMs = 0;
    wifiPortalManual = false;
    wifiPortalLastClosedMs = millis();
    wifiAttemptStartMs = millis();
    Serial.printf("[BOOT %lu ms] WiFiManager portal closed\n", (unsigned long)millis());
    if (WiFi.status() != WL_CONNECTED && wifiManager.getWiFiIsSaved()) {
      WiFi.mode(WIFI_STA);
      WiFi.begin(); // resume STA using newly saved credentials
      wifiHadCredentials = true;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiWasConnected) {
      wifiWasConnected = true;
      Serial.printf("[BOOT %lu ms] Wi-Fi CONNECTED: %s\n",
                    (unsigned long)millis(), WiFi.localIP().toString().c_str());
    }
    if (!portalActive) startNetworkServicesAfterWiFi();
    return;
  }

  if (wifiWasConnected) {
    wifiWasConnected = false;
    wifiAttemptStartMs = now;
    wifiLastReconnectMs = now;
    Serial.printf("[WiFi %lu ms] STA disconnected; HVAC remains active\n",
                  (unsigned long)now);
  }

  if (portalActive) return;

  if (networkServicesStarted) {
    // On a running controller do not start another AP on TCP/80 while the
    // dashboard server owns that port. Let the station reconnect instead.
    if (now - wifiLastReconnectMs >= WIFI_STA_RECONNECT_MS) {
      wifiLastReconnectMs = now;
      WiFi.reconnect();
      Serial.println(F("[WiFi] Reconnecting STA in background."));
    }
    return;
  }

  // If no saved credentials, show captive portal immediately. Otherwise
  // allow a full 20s for DHCP/authentication, WITHOUT blocking HVAC.
  if (wifiHadCredentials && now - wifiAttemptStartMs < WIFI_BOOT_STA_GRACE_MS) return;
  // After a timeout/closed portal, avoid opening an AP in a tight loop.
  if (wifiHadPortal && wifiPortalLastClosedMs &&
      now - wifiPortalLastClosedMs < WIFI_PORTAL_RETRY_DELAY_MS) return;
  startNonblockingConfigPortal();
}

// ================== Setup/Loop ==================
void setup() {
  // LEDy
  if (LED_PIN >= 0) {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // active LOW
  }
  pinMode(FLASH_BTN_PIN, INPUT_PULLUP);

  if (RED_LED_PIN >= 0) {
    pinMode(RED_LED_PIN, OUTPUT);
    #if defined(ESP8266)
    analogWriteRange(PWM_MAX);
    #endif
    pwmWriteCompat(RED_LED_PIN, 0);
  }

  Serial.begin(115200);
  randomSeed(micros() ^ (uint32_t)ESP.getCycleCount());
  initCsrf();
  Serial.println();
  Serial.println(F("== Boot =="));

  // >>> HVAC UART init (důležité pro zprovoznění komunikace)
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  //hvac.begin(); // << povolit linku podle knihovny

  Serial.println(F("HVAC UART init done."));
  #if defined(ESP32)
    // Přemapuj HW UART na zvolené piny a drž 9600 8E1 (jak používá knihovna)
    HVAC_SERIAL.end();
    HVAC_SERIAL.begin(9600, SERIAL_8E1, RX_PIN, TX_PIN);
  #endif

  // CRITICAL: this uses the original proven 6-frame Toshiba handshake,
  // unchanged 9600/8E1 pins and timings.  Call it before ANY Wi-Fi work.
  firmwareBootMs = millis();
  Serial.printf("[BOOT %lu ms] HVAC handshake start\n", (unsigned long)millis());
  hvac.handleHvac();
  Serial.printf("[BOOT %lu ms] HVAC handshake first pass completed\n", (unsigned long)millis());

  loadConfig();

  // DeviceID pro SSID již teď (before Wi-Fi STA start)
  computeDeviceID();
  computeDeviceLabel();

  // >>> Boot red LED na 5 s
  if (RED_LED_PIN >= 0) {
    bootRedActive = true;
    bootRedUntil  = millis() + BOOT_RED_LED_MS;
    pwmWriteCompat(RED_LED_PIN, PWM_MAX); // platform-neutral PWM
  }

  // WiFiManager parametry – předvyplnit z EEPROM
  char mqttEnStr[2]; snprintf(mqttEnStr, sizeof(mqttEnStr), "%u", (unsigned)deviceConfig.mqttEnable);
  p_mqtt_en.setValue(mqttEnStr, sizeof(mqttEnStr));
  p_mqtt_server.setValue(deviceConfig.mqttServer, sizeof(deviceConfig.mqttServer));
  char portStr[6]; snprintf(portStr, sizeof(portStr), "%u", deviceConfig.mqttPort);
  p_mqtt_port.setValue(portStr, sizeof(portStr));
  p_mqtt_user.setValue(deviceConfig.mqttUser, sizeof(deviceConfig.mqttUser));
  p_mqtt_pass.setValue("", sizeof(deviceConfig.mqttPassword)); // never render a saved password on the open configuration AP
  p_mqtt_cmd.setValue(deviceConfig.mqttCmdTopic, sizeof(deviceConfig.mqttCmdTopic));
  p_mqtt_state.setValue(deviceConfig.mqttStateTopic, sizeof(deviceConfig.mqttStateTopic));
  p_mqtt_hb.setValue(deviceConfig.mqttHeartbeatTopic, sizeof(deviceConfig.mqttHeartbeatTopic));

  char devEnStr[2]; snprintf(devEnStr, sizeof(devEnStr), "%u", (unsigned)deviceConfig.useCustomId);
  p_dev_id_en.setValue(devEnStr, sizeof(devEnStr));
  p_dev_id.setValue(deviceConfig.customDeviceId, sizeof(deviceConfig.customDeviceId));

  p_room.setValue(deviceConfig.room, sizeof(deviceConfig.room));
  p_name.setValue(deviceConfig.unitName, sizeof(deviceConfig.unitName));

  char mbEnStr[2]; snprintf(mbEnStr, sizeof(mbEnStr), "%u", (unsigned)deviceConfig.modbusEnable);
  p_mb_enable.setValue(mbEnStr, sizeof(mbEnStr));
  char mbPortStr[6]; snprintf(mbPortStr, sizeof(mbPortStr), "%u", deviceConfig.modbusPort);
  p_mb_port.setValue(mbPortStr, sizeof(mbPortStr));
  char mbUidStr[4]; snprintf(mbUidStr, sizeof(mbUidStr), "%u", deviceConfig.modbusUnitId);
  p_mb_uid.setValue(mbUidStr, sizeof(mbUidStr));

  // OTA parametry
  char otaEnStr[2]; snprintf(otaEnStr, sizeof(otaEnStr), "%u", (unsigned)deviceConfig.otaEnable);
  p_ota_en.setValue(otaEnStr, sizeof(otaEnStr));
  p_ota_pass.setValue("", sizeof(deviceConfig.otaPassword)); // never render a saved password on the open configuration AP

  // Telnet parametry
  char tEnStr[2]; snprintf(tEnStr, sizeof(tEnStr), "%u", (unsigned)deviceConfig.telnetEnable);
  p_telnet_en.setValue(tEnStr, sizeof(tEnStr));
  p_telnet_pass.setValue("", sizeof(deviceConfig.telnetPassword)); // never render a saved password on the open configuration AP

  wifiManager.setSaveConfigCallback(saveConfigCallback);

  // Do not call autoConnect(): it can block up to 30s x 2 before HVAC starts.
  // Failed STA association is handled by our state machine; portal is nonblocking.
  wifiManager.setConfigPortalBlocking(false);
  wifiManager.setConnectTimeout(8);     // only used during portal credential save
  wifiManager.setSaveConnectTimeout(8);
  wifiManager.setConnectRetries(1);
  wifiManager.setConfigPortalTimeout(300);
  wifiManager.setBreakAfterConfig(true);
  wifiManager.setDisableConfigPortal(true); // close AP after successful Wi-Fi save
  wifiManager.setSaveParamsCallback(saveConfigCallback);

  wifiManager.addParameter(&p_mqtt_en);
  wifiManager.addParameter(&p_mqtt_server);
  wifiManager.addParameter(&p_mqtt_port);
  wifiManager.addParameter(&p_mqtt_user);
  wifiManager.addParameter(&p_mqtt_pass);
  wifiManager.addParameter(&p_mqtt_cmd);
  wifiManager.addParameter(&p_mqtt_state);
  wifiManager.addParameter(&p_mqtt_hb);
  wifiManager.addParameter(&p_dev_id_en);
  wifiManager.addParameter(&p_dev_id);
  wifiManager.addParameter(&p_room);
  wifiManager.addParameter(&p_name);
  wifiManager.addParameter(&p_mb_enable);
  wifiManager.addParameter(&p_mb_port);
  wifiManager.addParameter(&p_mb_uid);
  wifiManager.addParameter(&p_ota_en);
  wifiManager.addParameter(&p_ota_pass);
  wifiManager.addParameter(&p_telnet_en);
  wifiManager.addParameter(&p_telnet_pass);

  // STA connect happens in the background; do not wait here.
  // Stored credentials belong to the core (as saved by WiFiManager).
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  wifiHadCredentials = wifiManager.getWiFiIsSaved();
  wifiAttemptStartMs = millis();
  if (wifiHadCredentials) {
    WiFi.begin();
    Serial.printf("[BOOT %lu ms] STA connecting asynchronously\n", (unsigned long)millis());
  } else {
    Serial.printf("[BOOT %lu ms] No saved Wi-Fi; config AP will open in loop()\n",
                  (unsigned long)millis());
  }

  // DeviceID dle configu (znovu pro případ změny custom ID)
  computeDeviceID();

  // MQTT – povol dle configu, jen pokud je host nastaven
  mqttEnabled = deviceConfig.mqttEnable != 0;
  if (mqttEnabled && isMqttHostConfigured()) {
    #if (USE_MQTT_TLS)
      #error "MQTT TLS requires certificate configuration; unverified TLS is disabled."
    #endif
    configureMqttClient();
  }

  // Web
#if defined(ESP8266)
  server.collectHeaders("X-Requested-With", "X-CSRF-Token");
#else
  const char* customHeaders[] = {"X-Requested-With", "X-CSRF-Token"};
  server.collectHeaders(customHeaders, 2);
#endif
  server.on("/", handleRoot);
  server.on("/command", HTTP_POST, handleCommand);
  server.on("/api/security/token", HTTP_GET, handleSecurityToken);
  server.on("/api/restart", HTTP_POST, handleRestart);
  server.on("/locale/cs.json", HTTP_GET, handleLocaleCs);
  server.on("/locale/en.json", HTTP_GET, handleLocaleEn);
  server.on("/api/backup", HTTP_GET, handleConfigBackup);
  server.on("/api/restore", HTTP_POST, handleConfigRestore);
  server.on("/api/features", HTTP_GET, handleFeaturesGet);
  server.on("/api/features", HTTP_POST, handleFeaturesSave);
  server.on("/mqttconfig", HTTP_POST, handleMQTTConfig);
  server.on("/debug", handleDebug);
  server.on("/service", HTTP_GET, handleServicePage);
  server.on("/service.js", HTTP_GET, handleServiceJs);
  server.on("/api/service/state", HTTP_GET, handleServiceState);
  server.on("/api/service/action", HTTP_POST, handleServiceAction);
  server.on("/app.css", handleAppCss);
  server.on("/app.js",  handleAppJs);

    // --- Factory reset (chráněno web auth) ---
  server.on("/factory-reset", HTTP_GET,  handleFactoryResetPage);
  server.on("/factory-reset", HTTP_POST, handleFactoryResetPost);

  // --- API pro auto-refresh a stav ---
  server.on("/api/rev", [](){
    if (!handleAuthentication()) return;
    server.send(200, "text/plain", String(g_stateRev));
  });

  // --- FW update přes web ---
  server.on("/fw", HTTP_GET, handleFwPage);
  // POST s upload streamem
  server.on("/fwupdate", HTTP_POST, handleFwDone, handleFwUpload);

  server.on("/api/config.json", [](){
    if (!handleAuthentication()) return;
    StaticJsonDocument<640> doc;
    doc["mqtt_en"] = deviceConfig.mqttEnable;
    doc["server"]  = deviceConfig.mqttServer;
    doc["port"]    = deviceConfig.mqttPort;
    doc["user"]    = deviceConfig.mqttUser;
    doc["pass"]    = "";
    doc["cmdtopic"]= deviceConfig.mqttCmdTopic;
    doc["statetopic"] = deviceConfig.mqttStateTopic;
    doc["hbtopic"] = deviceConfig.mqttHeartbeatTopic;

    doc["webuser"] = deviceConfig.webUser;
    doc["webpass"] = "";

    doc["dev_id_en"] = deviceConfig.useCustomId;
    doc["dev_id"]    = deviceConfig.customDeviceId;
    doc["room"] = deviceConfig.room;
    doc["name"] = deviceConfig.unitName;

    doc["mb_en"]   = deviceConfig.modbusEnable;
    doc["mb_port"] = deviceConfig.modbusPort;   // info
    doc["mb_uid"]  = deviceConfig.modbusUnitId; // info

    doc["ota_en"]  = deviceConfig.otaEnable;
    doc["ota_pass"]= "";

    doc["telnet_en"]   = deviceConfig.telnetEnable;
    doc["telnet_pass"] = "";

    String out; out.reserve(measureJson(doc)+8);
    serializeJson(doc, out);
    server.send(200, "application/json", out);
  });

  server.on("/api/state.json", [](){
    if (!handleAuthentication()) return;
    StaticJsonDocument<768> doc;
    doc["power"] = nz(hvac.getState());
    doc["temp"]  = (int)hvac.getSetpoint();
    doc["mode"]  = nz(hvac.getMode());
    doc["fan"]   = nz(hvac.getFanMode());
    doc["swing"] = nz(hvac.getSwing());

    // UART/komunikace s jednotkou
    doc["hvac"]  = hvac.isConnected() ? "Active" : "Inactive";
    doc["uart_rx_ok"] = hvac.validPackets();
    doc["uart_rx_invalid"] = hvac.invalidPackets();
    doc["uart_handshakes"] = hvac.handshakeAttempts();
    doc["uart_last_rx_ms"] = hvac.lastValidRxMs();
    const uint32_t lastRx = hvac.lastValidRxMs();
    if (lastRx) doc["uart_last_rx_age_ms"] = millis() - lastRx;
    else doc["uart_last_rx_age_ms"] = nullptr;
    doc["wifi_connected"] = WiFi.status() == WL_CONNECTED;
    doc["mqtt_connected"] = mqttEnabled && mqttClient.connected();
    doc["ha_discovery"] = featureOptions.haDiscovery != 0;
    doc["oper"]  = hvacOperatingState();
    float tin, tout;
    if (hvacReadRoomTemp(tin)  && !isnan(tin))  doc["t_in"]  = tin;
    if (hvacReadOutdoorTemp(tout) && !isnan(tout)) doc["t_out"] = tout;

    uint16_t flags = hvacReadFlags();
    uint16_t err   = hvacReadErrorCode();
    if (flags) doc["flags"] = flags;
    if (err != 0xFFFF) doc["err"] = err;

    doc["id"] = deviceID;
    if (deviceConfig.room[0]) doc["room"] = deviceConfig.room;
    if (deviceLabel[0])       doc["name"] = deviceLabel;

    doc["rev"] = g_stateRev;
    String out; out.reserve(measureJson(doc)+8);
    serializeJson(doc, out);
    server.send(200, "application/json", out);
  });
  server.on("/api/raw.txt", [](){
    if (!handleAuthentication()) return;
    String s; s.reserve(8192);
    s += F("--- RAW RING (newest first) ---\n");
    uint16_t head = g_rawHead;
    for (uint16_t i=0;i<RAW_RING_MAX;i++){
      const RawEvt &e = g_rawRing[(head+RAW_RING_MAX-1-i)%RAW_RING_MAX];
      if (!e.ts_ms) break;
      s += "#"; s += String(i); s += " t="; s += String(e.ts_ms);
      s += e.tx? " TX":" RX"; s += " cmd="; s += String(e.cmd);
      s += " len="; s += String(e.len); s += " data=";
      for (uint8_t k=0;k<e.len;k++){ if(k) s+=','; s+=String(e.data[k]); }
      s += "\n";
  }
  server.send(200,"text/plain",s);
  });

  // Routes registered. Do not bind port 80 yet: WiFiManager needs it if
  // STA credentials are missing or connection fails during bootstrap.
  Serial.println(F("Web routes ready (listener deferred until STA connected)."));
  telnetEnabled = (deviceConfig.telnetEnable != 0);

  // The actual OTA setup is deferred until Wi-Fi connects, including
  // passwords/settings newly provided by the first-run config portal.
  otaStarted = false;

  // Modbus server starts with the other listeners after STA association.
  modbusActive = false;
  lastHvacResponse = millis();
  Serial.printf("[BOOT %lu ms] setup() completed; HVAC continues regardless of Wi-Fi\n",
                (unsigned long)millis());
}

void loop() {
  if (restartRequestedMs && millis() - restartRequestedMs >= 800) {
    Serial.println(F("[SYS] Restart requested through WebUI"));
    ESP.restart();
  }
  // Highest priority: the original Toshiba handshake and RX collector.
  hvac.handleHvac();
  static bool hvacConnectLogged = false;
  if (hvac.isConnected() && !hvacConnectLogged) {
    hvacConnectLogged = true;
    Serial.printf("[BOOT %lu ms] HVAC UART CONNECTED (rx_ok=%lu)\n",
                  (unsigned long)millis(), (unsigned long)hvac.validPackets());
  }

  // STA and WiFiManager AP are serviced cooperatively in loop().
  maintainWiFiBoot();

  if (networkServicesStarted && WiFi.status() == WL_CONNECTED &&
      deviceConfig.otaEnable && otaStarted) {
    ArduinoOTA.handle();
  }

  // >>> nejdřív obsluha 5s boot LED, pak případně OTA fade
  bootRedLedHandle();
  otaLedHandle(); // animace červené LED během OTA

  // Do not allow a blocking MQTT TCP attempt during the first handshake and
  // full initial Toshiba query. UART is serviced before this block.
  if (networkServicesStarted && WiFi.status() == WL_CONNECTED && mqttEnabled &&
      (millis() - firmwareBootMs >= MQTT_BOOT_GUARD_MS)) {
    if (!mqttClient.connected()) connectToMQTT();
    if (mqttClient.connected()) {
      mqttClient.loop();
      if (featureDirty && millis() - haLastPublishMs >= 5000UL) {
        haLastPublishMs = millis();
        if (haPublishDiscovery(featureOptions.haDiscovery)) {
          haDiscoveryEverPublished = featureOptions.haDiscovery;
          featureDirty = false;
        }
      }
    }
  }

  if (networkServicesStarted && WiFi.status() == WL_CONNECTED) {
    server.handleClient();
    handleTelnet();
  }
  handleFlashButton(); // manual portal remains accessible as before

  // Modbus obsluha
  if (modbusActive) {
    mb.task();
    modbusApplyWrites();
  }

  uint32_t now = millis();

  if (now - lastStatePub >= STATE_PUBLISH_MS) {
    publishState();
    if (modbusActive) modbusSyncFromHVAC();
    lastStatePub = now;
  }

  if (now - lastHeartbeat >= HEARTBEAT_MS) {
    // Heartbeat přes MQTT
    if (mqttEnabled && mqttClient.connected()) {
      StaticJsonDocument<160> doc;
      doc["status"] = "online";
      doc["uptime"] = (uint32_t)(millis() / 1000UL);
      doc["id"] = deviceID;
      if (deviceConfig.room[0]) doc["room"] = deviceConfig.room;
      if (deviceLabel[0])       doc["name"] = deviceLabel;
      String payload; payload.reserve(64);
      serializeJson(doc, payload);
      char topic[96]; expandTopic(deviceConfig.mqttHeartbeatTopic, topic, sizeof(topic));
      if (!mqttClient.publish(topic, reinterpret_cast<const uint8_t*>(payload.c_str()),payload.length(),false))
      Serial.printf("[MQTT] publish failed (%u bytes)\n",(unsigned)payload.length());
      Serial.print(F("Heartbeat: ")); Serial.println(payload);
    }
    if (modbusActive) {
      updateModbusDiagnostics();
      setHoldingFromHvac(HREG_STATUS, buildStatusReg());
      setHoldingFromHvac(HREG_UPTIME_S, (uint16_t)((millis()/1000UL) & 0xFFFF));
    }
    lastHeartbeat = now;
  }

  // LED diagnostika (LOW = svítí); mimo OTA fade
  if (!otaActive && LED_PIN >= 0) {
    bool hvacConn = hvac.isConnected() && (now - hvac.lastValidRxMs()) < HVAC_TIMEOUT_MS;
    if (!mqttEnabled) {
      digitalWrite(LED_PIN, (now / 500) % 2 ? LOW : HIGH);         // pomalé blikání
    } else if (!mqttClient.connected()) {
      digitalWrite(LED_PIN, (now / 200) % 2 ? LOW : HIGH);         // rychlejší blikání
    } else if (!hvacConn) {
      digitalWrite(LED_PIN, (now / 500) % 2 ? LOW : HIGH);
    } else {
      if (LED_PIN >= 0) digitalWrite(LED_PIN, LOW);                                 // OK = trvale svítí
    }
  }

  yield();
}
