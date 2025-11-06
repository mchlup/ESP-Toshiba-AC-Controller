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
void HvacRawAnalyzer_onFrame(bool tx, uint8_t cmd, const uint8_t* payload, uint8_t plen) {
  // Odkomentuj, chceš-li logovat
  /*
  Serial.print(tx ? F("TX ") : F("RX "));
  Serial.print(F("CMD=")); Serial.print(cmd);
  Serial.print(F(" PLEN=")); Serial.print(plen);
  Serial.print(F(" PAYLOAD:"));
  for (uint8_t i = 0; i < plen; i++) { Serial.print(' '); Serial.print(payload[i]); }
  Serial.println();
  */
}

// ================== HW & BUILD CONFIG ==================
#if defined(ESP8266)
  // NodeMCU / Wemos D1 mini: osvědčené piny pro SW/HW serial
  #define RX_PIN         12      // D6 (GPIO12)
  #define TX_PIN         14      // D5 (GPIO14)
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
    #define LED_PIN      -1      // většina C3 nemá klasickou LED na GPIO2
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
#define USE_MQTT_TLS  false   // true = WiFiClientSecure + 8883; false = WiFiClient + 1883
#define EEPROM_SIZE  1024
#define CONFIG_MAGIC 0x42
#define CONFIG_VER   8   // +Telnet enable/pass; OTA gating; další drobnosti

struct SwingFixMap;
static bool HvacRawAnalyzer_guessSwingFixMap(SwingFixMap &out);
static const uint32_t STATE_PUBLISH_MS   = 10000UL;
static const uint32_t HEARTBEAT_MS       = 30000UL;
static const uint32_t HVAC_TIMEOUT_MS    = 30000UL;
static const uint32_t MQTT_RETRY_MS      = 5000UL;
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
  cfg.otaEnable    = 1;
  cfg.otaPassword[0] = '\0';
  cfg.telnetEnable = 1;
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
  HREG_OPER       = 7,   // 0=auto,1=cool,2=heat,3=dry,4=fan_only,5=off
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
static uint16_t g_reqModeReg = 0;  // 0..4  (auto,cool,heat,dry,fan_only)
static uint16_t g_reqFanReg  = 0;  // 0..5  (0=auto, 1..5=lvl_1..lvl_5)

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
static void processCommand(const char* cmd) {
  if (!cmd || !*cmd) return;

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
    lastHvacResponse = millis();
    bumpRevIfChanged();
    return;
  }
  if (!strcasecmp(cmd, "STATE_OFF")) {
    uint8_t v = 0;
    HvacRawAnalyzer_onFrame(true, PSEUDO_CMD_POWER, &v, 1);
    hvac.setState("off");
    lastHvacResponse = millis();
    bumpRevIfChanged();
    return;
  }

  // TEMP_xx (17..30)
  if (!strncasecmp(cmd, "TEMP_", 5)) {
    int t = atoi(cmd + 5);
    if (t < 17) t = 17;
    if (t > 30) t = 30;
    // TX log (payload = cílová teplota)
    uint8_t tv = (uint8_t)t;
    HvacRawAnalyzer_onFrame(true, PSEUDO_CMD_TEMP, &tv, 1);

    hvac.setSetpoint((uint8_t)t);
    lastHvacResponse = millis();
    bumpRevIfChanged();
    return;
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
    else { Serial.println(F("MODE: nepodporovany alias")); return; }

    // TX log (payload = náš mapovaný kód režimu)
    uint8_t mv = (uint8_t)modeToReg(m);
    HvacRawAnalyzer_onFrame(true, PSEUDO_CMD_MODE, &mv, 1);

    hvac.setMode(m);
    g_reqModeReg = modeToReg(m);
    lastHvacResponse = millis();
    bumpRevIfChanged();
    return;
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
      static char buf[8]; snprintf(buf,sizeof(buf),"lvl_%c",p[0]); f = buf;
    } else {
      Serial.println(F("FAN: nepodporovany alias"));
      return;
    }

    // TX log (payload = náš mapovaný kód ventilátoru)
    uint8_t fv = (uint8_t)fanToReg(f);
    HvacRawAnalyzer_onFrame(true, PSEUDO_CMD_FAN, &fv, 1);

    hvac.setFanMode(f);
    g_reqFanReg = fanToReg(f);
    lastHvacResponse = millis();
    bumpRevIfChanged();
    return;
  }

  // SWING (reálný cmd=163 – logujeme TX; RX hook případně pasivně/snifferem)
  if (!strcasecmp(cmd, "SWING_OFF")) {
    uint8_t v = 0;   // marker pro FIX
    HvacRawAnalyzer_onFrame(true, 163, &v, 1);
    hvac.setSwing("fix");
    lastHvacResponse = millis(); bumpRevIfChanged(); return;
  }
  if (!strcasecmp(cmd, "SWING_V"))   {
    uint8_t v = 100; // marker pro V-swing
    HvacRawAnalyzer_onFrame(true, 163, &v, 1);
    hvac.setSwing("v_swing");
    lastHvacResponse = millis(); bumpRevIfChanged(); return;
  }
  if (!strcasecmp(cmd, "SWING_H"))   {
    uint8_t v = 101; // marker pro H-swing
    HvacRawAnalyzer_onFrame(true, 163, &v, 1);
    hvac.setSwing("h_swing");
    lastHvacResponse = millis(); bumpRevIfChanged(); return;
  }
  if (!strcasecmp(cmd, "SWING_VH"))  {
    uint8_t v = 102; // marker pro VH-swing
    HvacRawAnalyzer_onFrame(true, 163, &v, 1);
    hvac.setSwing("vh_swing");
    lastHvacResponse = millis(); bumpRevIfChanged(); return;
  }
  if (!strncasecmp(cmd, "SWING_FIX_", 10)) {
    const char* p = cmd + 10; // "1".."5"
    if (p[0]>='1' && p[0]<='5' && p[1]==0) {
      uint8_t pos = (uint8_t)(p[0]-'0');            // 1..5
      HvacRawAnalyzer_onFrame(true, 163, &pos, 1);  // TX pro heuristiku
      char buf[16]; snprintf(buf,sizeof(buf),"fix_pos_%c",p[0]);
      hvac.setSwing(buf);
      lastHvacResponse = millis(); bumpRevIfChanged(); return;
    }
    Serial.println(F("SWING: nepodporovany alias"));
    return;
  }
    // FACTORY RESET (nevratné!) – použij pouze pokud MQTT je důvěryhodné
  if (!strcasecmp(cmd, "FACTORY_RESET") || !strcasecmp(cmd, "RESET_FACTORY")) {
    // volitelně: publikovat upozornění na status topic
    Serial.println(F("[FACTORY] Trigger via MQTT"));
    do {
      // malá odpověď HTTP/MQTT může ještě odejít, ale nečekáme dlouho
      if (mqttClient.connected()) mqttClient.loop();
      delay(50);
    } while (false);
    doFactoryReset();
    return;
  }

  Serial.print(F("Neznamy prikaz: ")); Serial.println(cmd);
}


/*
 * DŮLEŽITÉ (doporučení pro lepší odhad mapy fix1..5):
 *  V knihovně ToshibaCarrierHvac přidej při PŘÍJMU rámce s cmd=163 ještě:
 *      HvacRawAnalyzer_onFrame(false, 163, payload, plen);
 *  -> analyzer pak uvidí i RX “echo/ack” a funkce HvacRawAnalyzer_guessSwingFixMap()
 *     bude schopná přiřazovat reálné hodnoty fix pozic.
 */


// ================== MQTT ==================
static void mqttCallback(char* topic, byte* payload, unsigned int length) {
  static char buf[128];
  unsigned int n = (length < sizeof(buf) - 1) ? length : (sizeof(buf) - 1);
  memcpy(buf, payload, n);
  buf[n] = '\0';

  Serial.print(F("MQTT zpráva (")); Serial.print(topic); Serial.print(F("): "));
  Serial.println(buf);
  processCommand(buf);
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
  mqttClient.setCallback(mqttCallback);
}

static void connectToMQTT() {
  if (!mqttEnabled) return;
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
    mqttClient.publish(statusTopic, "online", true);
    char cmdTopic[96]; expandTopic(deviceConfig.mqttCmdTopic, cmdTopic, sizeof(cmdTopic));
    mqttClient.subscribe(cmdTopic, 1);
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
  doc["hvac"]  = (hvac.isConnected() || hvac.isCduRunning()) ? "Active" : "Inactive";

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
    mqttClient.publish(topic, reinterpret_cast<const uint8_t*>(payload.c_str()), payload.length(), false);
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
  if (!strcmp(s,"vh_swing"))   return SWING_CODE_VH_SWING;
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
  if (r == SWING_CODE_VH_SWING)   return "vh_swing";
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
  if (hvac.isConnected() || hvac.isCduRunning()) s |= (1 << 0);
  // bit1 = MQTT připojeno
  if (mqttEnabled && mqttClient.connected()) s |= (1 << 1);
  // bit2 = Wi-Fi připojeno
  if (WiFi.status() == WL_CONNECTED) s |= (1 << 2);
  return s;
}

// mapování "oper" do registru (včetně OFF)
static uint16_t operToReg(const char* oper) {
  if (!oper) return 0;
  if (!strcmp(oper, "off")) return 5;
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

  // POWER
  uint16_t pwr = (!strcmp(nz(hvac.getState()), "on")) ? 1 : 0;
  if (mb.Hreg(HREG_POWER) != pwr) mb.Hreg(HREG_POWER, pwr);

  // SETPOINT
  uint16_t sp = (uint16_t)hvac.getSetpoint();
  if (sp < 17) sp = 17; if (sp > 30) sp = 30;
  if (mb.Hreg(HREG_SETPOINT) != sp) mb.Hreg(HREG_SETPOINT, sp);

  // MODE – preferuj požadovanou (shadow) hodnotu
  uint16_t mdCur  = modeToReg(nz(hvac.getMode()));
  if (g_reqModeReg != mdCur && g_reqModeReg <= 4) {
    // čekáme na potvrzení z jednotky – do Modbusu držíme požadavek
    if (mb.Hreg(HREG_MODE) != g_reqModeReg) mb.Hreg(HREG_MODE, g_reqModeReg);
  } else {
    // potvrzeno nebo bez požadavku
    if (mb.Hreg(HREG_MODE) != mdCur) mb.Hreg(HREG_MODE, mdCur);
  }

  // FAN – analogicky (0..5)
  uint16_t fnCur = fanToReg(nz(hvac.getFanMode()));
  if (g_reqFanReg != fnCur && isValidFanCode(g_reqFanReg)) {
    if (mb.Hreg(HREG_FAN) != g_reqFanReg) mb.Hreg(HREG_FAN, g_reqFanReg);
    } else {
    if (mb.Hreg(HREG_FAN) != fnCur) mb.Hreg(HREG_FAN, fnCur);
  }

  // SWING
  uint16_t sw = swingToReg(nz(hvac.getSwing()));
  if (mb.Hreg(HREG_SWING) != sw) mb.Hreg(HREG_SWING, sw);

  // OPER (H7)
  const char* oper = hvacOperatingState();
  uint16_t opr = operToReg(oper);
  if (mb.Hreg(HREG_OPER) != opr) mb.Hreg(HREG_OPER, opr);

  // TEPLOTY (H5/H6)
  float tin = NAN, tout = NAN;
  bool okTin  = hvacReadRoomTemp(tin);
  bool okTout = hvacReadOutdoorTemp(tout);
  int16_t tin_x10  = tempToX10_orNA(tin,  okTin);
  int16_t tout_x10 = tempToX10_orNA(tout, okTout);
  if ((int16_t)mb.Hreg(HREG_TIN_X10)  != tin_x10)  mb.Hreg(HREG_TIN_X10,  (uint16_t)tin_x10);
  if ((int16_t)mb.Hreg(HREG_TOUT_X10) != tout_x10) mb.Hreg(HREG_TOUT_X10, (uint16_t)tout_x10);

  // FLAGS / ERROR
  uint16_t flags = hvacReadFlags();
  uint16_t err   = hvacReadErrorCode();
  if (mb.Hreg(HREG_FLAGS) != flags) mb.Hreg(HREG_FLAGS, flags);
  if (mb.Hreg(HREG_ERROR) != err)   mb.Hreg(HREG_ERROR, err);

  // STATUS + UPTIME
  mb.Hreg(HREG_STATUS,   buildStatusReg());
  mb.Hreg(HREG_UPTIME_S, (uint16_t)((millis()/1000UL) & 0xFFFF));
}

static void modbusApplyWrites() {
  if (!modbusActive) return;

  bool changed = false;

  uint16_t v = mb.Hreg(HREG_POWER);
  if (v != hregCache[HREG_POWER]) {
    hvac.setState(v ? "on" : "off");
    hregCache[HREG_POWER] = v;
    lastHvacResponse = millis();
    changed = true;
  }

  v = mb.Hreg(HREG_SETPOINT);
  if (v < 17) v = 17; if (v > 30) v = 30;
  if (v != hregCache[HREG_SETPOINT]) {
    hvac.setSetpoint((uint8_t)v);
    hregCache[HREG_SETPOINT] = v;
    lastHvacResponse = millis();
    changed = true;
  }

  v = mb.Hreg(HREG_MODE);
  if (!isValidModeCode(v)) {
    // Pokud někdo zapíše mimo mapu, vrať na výchozí (třeba AUTO)
    v = MODE_CODE_AUTO;
  }
  if (v != hregCache[HREG_MODE]) {
    hvac.setMode(regToMode(v));
    g_reqModeReg = v;
    hregCache[HREG_MODE] = v;
    lastHvacResponse = millis();
    changed = true;
  }

  v = mb.Hreg(HREG_FAN);
  if (!isValidFanCode(v)) {
    v = FAN_CODE_AUTO; // mimo mapu -> bezpečně na AUTO
  }
  if (v != hregCache[HREG_FAN]) {
    hvac.setFanMode(regToFan(v));
    g_reqFanReg = v;
    hregCache[HREG_FAN] = v;
    lastHvacResponse = millis();
    changed = true;
  }

  v = mb.Hreg(HREG_SWING);
  if (!isValidSwingCode(v)) {
    v = SWING_CODE_FIX;  // mimo mapu -> bezpečně na FIX
  }
  if (v != hregCache[HREG_SWING]) {
    hvac.setSwing(regToSwing(v));
    hregCache[HREG_SWING] = v;
    lastHvacResponse = millis();
    changed = true;
  }

  if (changed) {
    bumpRevIfChanged();
    modbusSyncFromHVAC();   // <<< přidej
  }
}

static void modbusSetup() {
  if (!deviceConfig.modbusEnable) { modbusActive = false; return; }

  // Stabilní režim – naslouchat na implicitním portu 502 (nezpřístupňujeme změnu portu)
  mb.server();  // TCP 502

  for (uint16_t r = 0; r < HREG__COUNT; ++r) {
    mb.addHreg(r, 0);
  }
  modbusSyncFromHVAC();
  for (uint16_t r = 0; r < HREG__COUNT; ++r) {
    hregCache[r] = mb.Hreg(r);
  }
  modbusActive = true;
  Serial.printf("Modbus TCP server ON (port 502, unit %u - info only)\n", deviceConfig.modbusUnitId);
}

// ================== Web ==================
// ============ Embedded UI assets (modern responsive) ============
static const char APP_CSS[] PROGMEM = R"CSS(
:root{
  --bg: #0b1020; --fg:#e9eefb; --muted:#aab3c5; --card:#131a2e; --accent:#4f8cff;
  --ok:#26c281; --warn:#ffb020; --bad:#ff6b6b; --border:#223052;
  --gap: 12px; --radius: 14px; --pad: 14px; --shadow: 0 8px 24px rgba(0,0,0,.25);
  --font: system-ui,-apple-system,Segoe UI,Roboto,Ubuntu,Arial;
}
@media (prefers-color-scheme: light){
  :root{ --bg:#f5f7fb; --fg:#0f172a; --muted:#465066; --card:#ffffff; --accent:#2563eb; --border:#dbe1f1; }
}
*{box-sizing:border-box} html,body{height:100%}
body{margin:0;background:var(--bg);color:var(--fg);font-family:var(--font);-webkit-font-smoothing:antialiased}
.container{max-width:1024px;margin:0 auto;padding:16px}
.header{display:flex;gap:var(--gap);align-items:center;justify-content:space-between;margin-bottom:16px}
.hgroup{display:flex;gap:10px;align-items:center}
.badge{font-size:12px;padding:4px 8px;border-radius:999px;background:var(--card);border:1px solid var(--border);color:var(--muted)}
.brand{font-weight:700;font-size:18px;letter-spacing:.3px}
.grid{display:grid;gap:var(--gap);grid-template-columns:repeat(12,1fr)}
.card{background:var(--card);border:1px solid var(--border);border-radius:var(--radius);padding:var(--pad);box-shadow:var(--shadow)}
.card h3{margin:.1rem 0 .6rem 0;font-size:14px;color:var(--muted);font-weight:600}
.kv{display:grid;grid-template-columns:1fr auto;gap:8px;font-size:14px}
.kv span:nth-child(2){color:var(--fg);font-weight:600}
.status-dot{width:10px;height:10px;border-radius:50%;display:inline-block;vertical-align:middle;margin-right:6px}
.ok{background:var(--ok)} .bad{background:var(--bad)} .warn{background:var(--warn)}
.btn{appearance:none;border:1px solid var(--border);background:var(--card);color:var(--fg);padding:10px 12px;border-radius:12px;cursor:pointer;font-weight:600}
.btn:active{transform:translateY(1px)} .btn-group{display:flex;flex-wrap:wrap;gap:8px}
.btn.primary{background:var(--accent);border-color:transparent;color:#fff}
.segment{display:flex;background:var(--card);border:1px solid var(--border);border-radius:999px;overflow:hidden}
.segment button{flex:1;border:none;background:transparent;color:var(--fg);padding:10px 12px;font-weight:600;cursor:pointer}
.segment button.active{background:var(--accent);color:#fff}

/* >>> DOPLNĚNO: zvýraznění aktivních tlačítek i mimo .segment (Ventilátor / Lamel) */
.btn-group .btn.active, 
.btn.active{
  background: var(--accent);
  color: #fff;
  border-color: transparent;
  box-shadow: inset 0 0 0 1px rgba(255,255,255,.08);
}

.row{display:flex;gap:12px;align-items:center}
.value{font-size:22px;font-weight:800;letter-spacing:.3px}
.slider{width:100%}
/* --- form fields for config --- */
.form{display:grid;grid-template-columns:repeat(12,1fr);gap:12px}
.field{grid-column:span 6;display:flex;flex-direction:column;gap:6px}
.field.sm{grid-column:span 3}
.field.lg{grid-column:span 12}
label{font-size:12px;color:var(--muted);font-weight:600}
input[type=text],input[type=number],input[type=password]{
  width:100%;padding:10px;border-radius:10px;border:1px solid var(--border);
  background:var(--bg);color:var(--fg)
}
input[disabled]{opacity:.7}
.switchrow{display:flex;gap:10px;align-items:center}
.help{font-size:12px;color:var(--muted)}
small.muted{color:var(--muted)}
footer.sticky{
  position:sticky;bottom:0;z-index:5;background:linear-gradient(180deg, rgba(0,0,0,0), rgba(0,0,0,.2) 8px, var(--bg) 12px);
  padding-top:6px;margin-top:14px
}
.footerbar{display:flex;gap:var(--gap);align-items:center;padding:10px;border:1px solid var(--border);background:var(--card);
  border-radius:16px;box-shadow:var(--shadow);flex-wrap:wrap}
.footerbar .grow{flex:1;min-width:180px}
hr.sep{border:none;border-top:1px dashed var(--border);margin:10px 0}
a.link{color:var(--accent);text-decoration:none;font-weight:600}
code.tag{background:#00000020;border:1px solid var(--border);padding:2px 6px;border-radius:8px;font-family:ui-monospace,Consolas;font-size:12px}
@media (max-width:900px){ .grid {grid-template-columns:repeat(6,1fr)} }
@media (max-width:640px){ .grid {grid-template-columns:repeat(4,1fr)} .header{flex-wrap:wrap} }
)CSS";

static const char APP_JS[] PROGMEM = R"JS(
const $ = sel => document.querySelector(sel);

function setActive(group, value){
  if(!group) return;
  group.querySelectorAll('button').forEach(b=>{
    b.classList.toggle('active', b.dataset.val===value);
  });
}

async function apiState(){
  const r = await fetch('/api/state.json',{cache:'no-store'});
  if(!r.ok) throw new Error('state http '+r.status);
  return await r.json();
}

async function send(cmd){
  try{
    const r = await fetch('/command?cmd='+encodeURIComponent(cmd),{cache:'no-store'});
    if(!r.ok) throw 0;
  }catch(e){}
}

function fmt(val, unit=''){ return (val==null?'—':val)+unit; }

async function refresh(){
  try{
    const s = await apiState();
    // headline
    const oper = $('#oper');
    if(oper) oper.textContent = s.oper || '—';
    const pwrDot = $('#pwrDot');
    if(pwrDot) pwrDot.className = 'status-dot '+((s.power==='on')?'ok':'bad');

    // temps
    const setVal = $('#setVal');
    const slider = $('#temp');
    if(typeof s.temp==='number'){
      if(setVal) setVal.textContent = s.temp;
      if(slider && !document.activeElement.isSameNode(slider)){ slider.value = s.temp; }
    }
    const tin = $('#tin'), tout = $('#tout');
    if(tin)  tin.textContent  = (s.t_in!=null)? (s.t_in.toFixed(1)+' °C') : '—';
    if(tout) tout.textContent = (s.t_out!=null)? (s.t_out.toFixed(1)+' °C') : '—';

    // segments
    setActive($('#modeGroup'), s.mode||'auto');
    setActive($('#fanGroup'),  s.fan||'auto');
    setActive($('#swingGroup'), s.swing||'fix');

    // badges
    const hvacBadge = $('#hvacBadge');
    if(hvacBadge) hvacBadge.innerHTML = `<span class="status-dot ${s.hvac==='Active'?'ok':'bad'}"></span>${s.hvac||'—'}`;
  }catch(e){}
}

function bind(){
  // Config UI
  loadCfgUI();
  const rld = $('#btnCfgReload'), sv = $('#btnCfgSave');
  if(rld) rld.onclick = loadCfgUI;
  if(sv)  sv.onclick  = saveCfgUI;

  // power
  const btnOn = $('#btnOn'), btnOff = $('#btnOff');
  if(btnOn)  btnOn.onclick  = ()=>send('STATE_ON');
  if(btnOff) btnOff.onclick = ()=>send('STATE_OFF');

  // temp
  const slid = $('#temp');
  const lbl  = $('#setVal');
  if(slid){
    slid.addEventListener('input', e=>{ if(lbl) lbl.textContent = e.target.value; });
    let tmr=null;
    slid.addEventListener('change', e=>{
      const v = Math.max(17, Math.min(30, parseInt(e.target.value||24,10)));
      send('TEMP_'+v);
      if(tmr) clearTimeout(tmr);
      tmr=setTimeout(refresh,300);
    });
  }

  // segments
  const mg = $('#modeGroup');
  if(mg) mg.querySelectorAll('button').forEach(b=>{
    b.onclick = ()=>{
      const map = { auto:'AUTO', cool:'COOL', heat:'HEAT', dry:'DRY', fan_only:'FAN' }; // <<< FAN => server alias
      const val = map[b.dataset.val] || b.dataset.val.toUpperCase();
      send('MODE_'+val);
    };
  });
  const fg = $('#fanGroup');
  if(fg) fg.querySelectorAll('button').forEach(b=>{
    b.onclick=()=>{
      const v=b.dataset.val;
      const map={low:'LOW',medium:'MEDIUM',high:'HIGH','lvl_1':'1','lvl_2':'2','lvl_3':'3','lvl_4':'4','lvl_5':'5'};
      send('FAN_'+(map[v]||v.toUpperCase()));
    }
  });
  const sg = $('#swingGroup');
  if(sg) sg.querySelectorAll('button').forEach(b=>{
    b.onclick=()=>{
      const v=b.dataset.val;
      const map={fix:'SWING_OFF',v_swing:'SWING_V',h_swing:'SWING_H',vh_swing:'SWING_VH',
                 fix_pos_1:'SWING_FIX_1',fix_pos_2:'SWING_FIX_2',fix_pos_3:'SWING_FIX_3',fix_pos_4:'SWING_FIX_4',fix_pos_5:'SWING_FIX_5'};
      send(map[v]||'SWING_OFF');
    }
  });

  // polling
  refresh();
  setInterval(refresh, 2000);
}

async function apiConfig(){
  const r = await fetch('/api/config.json',{cache:'no-store'});
  if(!r.ok) throw new Error('config http '+r.status);
  return await r.json();
}

function getVal(id){ const el=$(id); return el?el.value:''; }
function setVal(id,v){ const el=$(id); if(el!=null) el.value = (v==null?'':v); }

async function loadCfgUI(){
  try{
    const c = await apiConfig();
    setVal('#cfg_mqtt_en', c.mqtt_en); setVal('#cfg_server', c.server); setVal('#cfg_port', c.port);
    setVal('#cfg_user', c.user); setVal('#cfg_pass', c.pass);
    setVal('#cfg_cmdtopic', c.cmdtopic); setVal('#cfg_statetopic', c.statetopic); setVal('#cfg_hbtopic', c.hbtopic);
    setVal('#cfg_webuser', c.webuser); setVal('#cfg_webpass', c.webpass);
    setVal('#cfg_dev_id_en', c.dev_id_en); setVal('#cfg_dev_id', c.dev_id);
    setVal('#cfg_room', c.room);
    setVal('#cfg_name', c.name);
    setVal('#cfg_mb_en', c.mb_en); setVal('#cfg_mb_port', c.mb_port); setVal('#cfg_mb_uid', c.mb_uid);
    setVal('#cfg_ota_en', c.ota_en); setVal('#cfg_ota_pass', c.ota_pass);
    setVal('#cfg_telnet_en', c.telnet_en); setVal('#cfg_telnet_pass', c.telnet_pass);
    const msg = $('#cfgMsg'); if(msg) msg.textContent = '';
  }catch(e){
    const msg = $('#cfgMsg'); if(msg) msg.textContent = 'Nepodařilo se načíst konfiguraci.';
  }
}

async function saveCfgUI(){
  const q = new URLSearchParams({
    mqtt_en: getVal('#cfg_mqtt_en'),
    server:  getVal('#cfg_server'),
    port:    getVal('#cfg_port'),
    user:    getVal('#cfg_user'),
    pass:    getVal('#cfg_pass'),
    cmdtopic: getVal('#cfg_cmdtopic'),
    statetopic: getVal('#cfg_statetopic'),
    hbtopic: getVal('#cfg_hbtopic'),
    webuser: getVal('#cfg_webuser'),
    webpass: getVal('#cfg_webpass'),
    dev_id_en: getVal('#cfg_dev_id_en'),
    dev_id:    getVal('#cfg_dev_id'),
    room:    getVal('#cfg_room'),
    name:    getVal('#cfg_name'),
    mb_en: getVal('#cfg_mb_en'),
    ota_en:  getVal('#cfg_ota_en'),
    ota_pass:getVal('#cfg_ota_pass'),
    telnet_en:  getVal('#cfg_telnet_en'),
    telnet_pass:getVal('#cfg_telnet_pass'),
  });
  try{
    const r = await fetch('/mqttconfig?'+q.toString(), {cache:'no-store'});
    const msg = $('#cfgMsg');
    if(r.ok){
      if(msg) msg.textContent = 'Konfigurace uložena. Aplikuji…';
      setTimeout(()=>{ loadCfgUI(); refresh(); }, 500);
    }else{
      if(msg) msg.textContent = 'Chyba při ukládání ('+r.status+').';
    }
  }catch(e){
    const msg = $('#cfgMsg'); if(msg) msg.textContent = 'Chyba při ukládání.';
  }
}

document.addEventListener('DOMContentLoaded', bind);
)JS";

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

static void handleRoot() {
  if (!handleAuthentication()) return;

  String html;
  html.reserve(16000);

  // --- Head + header ---
  html += F("<!doctype html><html lang='cs'><head><meta charset='utf-8'>");
  html += F("<meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<title>Toshiba HVAC</title>");
  html += F("<link rel='preload' href='/app.css' as='style'><link rel='stylesheet' href='/app.css'>");
  html += F("</head><body><div class='container'>");

  html += F("<div class='header'>"
            "  <div class='hgroup'>"
            "    <span class='brand'>");
  html += (deviceLabel[0] ? deviceLabel : "Toshiba HVAC");
  html += F("</span>"
            "    <span class='badge'>ID: ");
  html += deviceID;
  html += deviceConfig.useCustomId ? F(" · custom") : F(" · mac");
  html += F("</span>");
  if (deviceConfig.room[0]) {
    html += F("<span class='badge'>Room: ");
    html += deviceConfig.room;
    html += F("</span>");
  }
   html += F("  </div>"
            "  <div class='hgroup'>"
            "    <span id='pwrDot' class='status-dot bad'></span>"
            "    <span id='oper' class='badge'>—</span>"
            "  </div>"
            "</div>");

  html += F("<div class='grid'>");

  // --- Stav zařízení ---
  html += F("<div class='card' style='grid-column:span 6'>"
            "  <h3>Stav zařízení</h3>"
            "  <div class='kv'>"
            "    <span>Wi-Fi</span><span>");
  html += (WiFi.status()==WL_CONNECTED) ? F("Connected") : F("Not connected");
  html += F("</span><span>IP adresa</span><span>");
  html += WiFi.localIP().toString();
  html += F("</span><span>MQTT</span><span>");
  html += deviceConfig.mqttEnable ? (mqttClient.connected() ? "Enabled & Connected" : "Enabled, not connected") : "Disabled";
  html += F("</span><span>HVAC UART</span><span id='hvacBadge'><span class='status-dot bad'></span>—</span>"
            "<span>Modbus TCP</span><span>");
  html += deviceConfig.modbusEnable ? (modbusActive ? "Enabled (running)" : "Enabled (not running)") : "Disabled";
  html += F("</span><span>Telnet</span><span>");
  html += deviceConfig.telnetEnable ? (deviceConfig.telnetPassword[0] ? "Enabled (password set)" : "Enabled") : "Disabled";
  html += F("</span></div></div>");

  // --- Teploty ---
  html += F("<div class='card' style='grid-column:span 6'>"
            "  <h3>Teploty</h3>"
            "  <div class='row' style='justify-content:space-between'>"
            "    <div><small class='muted'>Vnitřní</small><div id='tin' class='value'>—</div></div>"
            "    <div><small class='muted'>Venkovní</small><div id='tout' class='value'>—</div></div>"
            "  </div>"
            "  <hr class='sep'>"
            "  <div class='row'>"
            "    <button id='btnOff' class='btn'>Vypnout</button>"
            "    <button id='btnOn' class='btn primary'>Zapnout</button>"
            "    <div class='grow'></div>"
            "    <div class='row'>"
            "      <small class='muted'>Nastaveno</small>"
            "      <div id='setVal' class='value'>—</div>"
            "      <small>°C</small>"
            "    </div>"
            "  </div>"
            "  <input id='temp' class='slider' type='range' min='17' max='30' step='1' value='24'/>"
            "</div>");

  // --- Režim ---
  html += F("<div class='card' style='grid-column:span 12'>"
            "  <h3>Režim</h3>"
            "  <div id='modeGroup' class='segment'>"
            "    <button data-val='auto' class='active'>Auto</button>"
            "    <button data-val='heat'>Topení</button>"
            "    <button data-val='cool'>Chlazení</button>"
            "    <button data-val='dry'>Dry</button>"
            "    <button data-val='fan_only'>Ventilátor</button>"
            "  </div>"
            "</div>");

  // --- Ventilátor ---
  html += F("<div class='card' style='grid-column:span 6'>"
            "  <h3>Ventilátor</h3>"
            "  <div id='fanGroup' class='btn-group'>"
            "    <button class='btn' data-val='auto'>Auto</button>"
            "    <button class='btn' data-val='quiet'>Quiet</button>"
            "    <button class='btn' data-val='low'>Low</button>"
            "    <button class='btn' data-val='medium'>Medium</button>"
            "    <button class='btn' data-val='high'>High</button>"
            "    <button class='btn' data-val='lvl_1'>Tichá</button>"
            "    <button class='btn' data-val='lvl_2'>Nízká</button>"
            "    <button class='btn' data-val='lvl_3'>Střední</button>"
            "    <button class='btn' data-val='lvl_4'>Vyšší</button>"
            "    <button class='btn' data-val='lvl_5'>Vysoká</button>"
            "  </div>"
            "</div>");

  // --- Natáčení lamel ---
  html += F("<div class='card' style='grid-column:span 6'>"
            "  <h3>Natáčení lamel</h3>"
            "  <div id='swingGroup' class='btn-group'>"
            "    <button class='btn' data-val='fix'>Fix</button>"
            "    <button class='btn' data-val='v_swing'>V-swing</button>"
            "    <button class='btn' data-val='h_swing'>H-swing</button>"
            "    <button class='btn' data-val='vh_swing'>VH-swing</button>"
            "    <button class='btn' data-val='fix_pos_1'>Fix 1</button>"
            "    <button class='btn' data-val='fix_pos_2'>Fix 2</button>"
            "    <button class='btn' data-val='fix_pos_3'>Fix 3</button>"
            "    <button class='btn' data-val='fix_pos_4'>Fix 4</button>"
            "    <button class='btn' data-val='fix_pos_5'>Fix 5</button>"
            "  </div>"
            "</div>");

  // --- Konfigurace zařízení ---
  html += F("<div class='card' style='grid-column:span 12'>"
            "  <h3>Konfigurace zařízení</h3>"
            "  <div class='form'>"
            "    <div class='field sm'><label>MQTT enable (0/1)</label><input id='cfg_mqtt_en' type='number' min='0' max='1'></div>"
            "    <div class='field lg'><label>MQTT server</label><input id='cfg_server' type='text' placeholder='broker.local'></div>"
            "    <div class='field sm'><label>MQTT port</label><input id='cfg_port' type='number' min='1' max='65535'></div>"
            "    <div class='field sm'><label>MQTT user</label><input id='cfg_user' type='text'></div>"
            "    <div class='field sm'><label>MQTT pass</label><input id='cfg_pass' type='password'></div>"
            "    <div class='field lg'><label>Cmd topic</label><input id='cfg_cmdtopic' type='text' placeholder='toshiba/{id}/control'></div>"
            "    <div class='field lg'><label>State topic</label><input id='cfg_statetopic' type='text' placeholder='toshiba/{id}/state'></div>"
            "    <div class='field lg'><label>Heartbeat topic</label><input id='cfg_hbtopic' type='text' placeholder='toshiba/{id}/heartbeat'></div>"
            "    <div class='field sm'><label>Web uživatel</label><input id='cfg_webuser' type='text'></div>"
            "    <div class='field sm'><label>Web heslo</label><input id='cfg_webpass' type='password'></div>"
            "    <div class='field sm'><label>Místnost (room)</label><input id='cfg_room' type='text' placeholder='Obyvak'></div>"
            "    <div class='field sm'><label>Název jednotky (name)</label><input id='cfg_name' type='text' placeholder='AC Obyvak'></div>"
            "    <div class='field sm'><label>Use custom ID (0/1)</label><input id='cfg_dev_id_en' type='number' min='0' max='1'></div>"
            "    <div class='field sm'><label>Custom ID</label><input id='cfg_dev_id' type='text' placeholder='A-Za-z0-9_- (max 12 -> doplní MAC)'></div>"
            "    <div class='field sm'><label>Modbus enable (0/1)</label><input id='cfg_mb_en' type='number' min='0' max='1'></div>"
            "    <div class='field sm'><label>Modbus port (info)</label><input id='cfg_mb_port' type='number' disabled></div>"
            "    <div class='field sm'><label>Modbus Unit ID (info)</label><input id='cfg_mb_uid' type='number' disabled></div>"
            "    <div class='field sm'><label>OTA enable (0/1)</label><input id='cfg_ota_en' type='number' min='0' max='1'></div>"
            "    <div class='field sm'><label>OTA password</label><input id='cfg_ota_pass' type='password'></div>"
            "    <div class='field sm'><label>Telnet enable (0/1)</label><input id='cfg_telnet_en' type='number' min='0' max='1'></div>"
            "    <div class='field sm'><label>Telnet password</label><input id='cfg_telnet_pass' type='password'></div>"
            "    <div class='field lg'><span class='help'>Pozn.: MQTT topics podporují <code class='tag'>{id}</code>. Změny se aplikují okamžitě, bez restartu.</span></div>"
            "  </div>"
            "  <div class='row' style='margin-top:10px;justify-content:flex-end'>"
            "    <button id='btnCfgReload' class='btn'>Načíst znovu</button>"
            "    <button id='btnCfgSave' class='btn primary'>Uložit konfiguraci</button>"
            "  </div>"
            "  <div id='cfgMsg' class='help' style='margin-top:6px'></div>"
            "</div>");

    // --- Firmware update (krátký formulář přímo na Dashboardu) ---
  html += F("<div class='card' style='grid-column:span 12'>"
            "  <h3>Firmware</h3>"
            "  <form method='POST' action='/fwupdate' enctype='multipart/form-data' class='row' style='flex-wrap:wrap'>"
            "    <input class='btn' type='file' name='fw' accept='.bin' required>"
            "    <button class='btn primary' type='submit'>Nahrát & aktualizovat</button>"
            "    <div class='grow'></div>"
            "    <a class='link' href='/fw'>Pokročilá stránka…</a>"
            "  </form>"
            "  <div class='help' style='margin-top:6px'>Po úspěšném nahrání proběhne automatický restart.</div>"
            "</div>");

  // --- konec gridu + footer + skripty ---
  html += F("</div>");  // grid

  html += F("<footer class='sticky'>"
            "  <div class='footerbar'>"
            "    <div class='row' style='gap:8px'>"
            "      <button id='btnOff2' class='btn'>OFF</button>"
            "      <button id='btnOn2' class='btn primary'>ON</button>"
            "    </div>"
            "    <div class='grow'>"
            "      <input id='temp2' class='slider' type='range' min='17' max='30' step='1' value='24'/>"
            "    </div>"
            "    <div class='row'><small class='muted'>Set</small><div id='setVal2' class='value'>—</div><small>°C</small></div>"
            "  </div>"
            "</footer>");

  html += F("<div style='margin-top:14px'>"
            "  <small class='muted'>Konfigurace najdeš na původní URL <code class='tag'>/mqttconfig</code> a diagnostiku na "
            "  <a class='link' href='/debug'>/debug</a>. Výchozí hesla <code class='tag'>admin/admin</code> prosím hned změň.</small>"
            "</div>");

  html += F("<script src='/app.js'></script>"
            "<script>"
            "document.addEventListener('DOMContentLoaded',()=>{"
            " const t=document.getElementById('temp'), t2=document.getElementById('temp2');"
            " const v=document.getElementById('setVal'), v2=document.getElementById('setVal2');"
            " const sync=(a,b,va,vb)=>{a.addEventListener('input',e=>{b.value=e.target.value; va.textContent=vb.textContent=e.target.value;});"
            " a.addEventListener('change',()=>{b.dispatchEvent(new Event('change'));});};"
            " if(t&&t2&&v&&v2){sync(t,t2,v,v2); sync(t2,t,v2,v);}"
            " const on=()=>{const x=document.getElementById('btnOn'); if(x) x.click();};"
            " const off=()=>{const x=document.getElementById('btnOff'); if(x) x.click();};"
            " const on2=document.getElementById('btnOn2'), off2=document.getElementById('btnOff2');"
            " if(on2) on2.onclick=on; if(off2) off2.onclick=off;"
            "});"
            "</script>");

  html += F("</div></body></html>");

  server.send(200, "text/html", html);
}

static void handleCommand() {
  if (!handleAuthentication()) return;
  if (server.hasArg("cmd")) {
    String c = server.arg("cmd");
    c.trim();
    char tmp[64]; c.toCharArray(tmp, sizeof(tmp));
    processCommand(tmp);
    server.send(200, "text/plain", String("Příkaz proveden: ") + c);
  } else {
    server.send(400, "text/plain", "Chybí parametr cmd");
  }
}

static void handleMQTTConfig() {
  if (!handleAuthentication()) return;

  if (server.hasArg("mqtt_en")) deviceConfig.mqttEnable = (uint8_t)server.arg("mqtt_en").toInt();
  if (server.hasArg("server")) server.arg("server").toCharArray(deviceConfig.mqttServer, sizeof(deviceConfig.mqttServer));
  if (server.hasArg("port"))   deviceConfig.mqttPort = (uint16_t)server.arg("port").toInt();
  if (server.hasArg("user"))   server.arg("user").toCharArray(deviceConfig.mqttUser, sizeof(deviceConfig.mqttUser));
  if (server.hasArg("pass"))   server.arg("pass").toCharArray(deviceConfig.mqttPassword, sizeof(deviceConfig.mqttPassword));
  if (server.hasArg("cmdtopic")) server.arg("cmdtopic").toCharArray(deviceConfig.mqttCmdTopic, sizeof(deviceConfig.mqttCmdTopic));
  if (server.hasArg("statetopic")) server.arg("statetopic").toCharArray(deviceConfig.mqttStateTopic, sizeof(deviceConfig.mqttStateTopic));
  if (server.hasArg("hbtopic")) server.arg("hbtopic").toCharArray(deviceConfig.mqttHeartbeatTopic, sizeof(deviceConfig.mqttHeartbeatTopic));

  if (server.hasArg("webuser")) server.arg("webuser").toCharArray(deviceConfig.webUser, sizeof(deviceConfig.webUser));
  if (server.hasArg("webpass")) server.arg("webpass").toCharArray(deviceConfig.webPassword, sizeof(deviceConfig.webPassword));

  // OTA
  if (server.hasArg("ota_en"))   deviceConfig.otaEnable = (uint8_t)server.arg("ota_en").toInt();
  if (server.hasArg("ota_pass")) server.arg("ota_pass").toCharArray(deviceConfig.otaPassword, sizeof(deviceConfig.otaPassword));

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
  if (server.hasArg("telnet_pass")) server.arg("telnet_pass").toCharArray(deviceConfig.telnetPassword, sizeof(deviceConfig.telnetPassword));

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
  if (deviceConfig.modbusEnable) modbusSetup(); else modbusActive = false;

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
  dbg += (deviceLabel[0] ? deviceLabel : "(unnamed)");
  dbg += F(" | ID: "); dbg += deviceID; dbg += deviceConfig.useCustomId ? " (custom)" : " (MAC)";
  dbg += F("\nRoom: ");
  dbg += (deviceConfig.room[0] ? deviceConfig.room : "(n/a)");
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
  dbg += F("RAW: tx="); dbg += g_txFrames;
  dbg += F(" rx="); dbg += g_rxFrames;
  dbg += F(" last_ms="); dbg += g_lastRawMs; dbg += F("\n");
  dbg += F("</pre><a href='/'>Zpět</a>");
  server.send(200, "text/html", dbg);
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
          digitalWrite(LED_PIN, LOW); delay(150);
          digitalWrite(LED_PIN, HIGH); delay(150);
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
        processCommand(line);
        telnetClient.println(F("OK"));
      }

      telnetClient.print(F("> "));
    } else {
      if (idx < sizeof(line) - 1) line[idx++] = ch;
    }
  }
}

// ================== FLASH button: vyvolání WiFiManager portálu ==================
static void enterConfigPortal() {
  Serial.println(F("[FLASH] Request to open WiFiManager config portal"));

  // Stop služeb, aby portál měl volné porty a AP naskočilo spolehlivě
  server.stop();
  if (mqttClient.connected()) mqttClient.disconnect();
  bool modbusWasActive = modbusActive;
  modbusActive = false;

  // čistý AP mód kvůli stabilitě portálu
  WiFi.disconnect();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_AP);

  wifiManager.setConfigPortalBlocking(true);
  wifiManager.setBreakAfterConfig(true);
  wifiManager.setConfigPortalTimeout(300); // 5 min

  char apName[40];
  snprintf(apName, sizeof(apName), "Toshiba-%s-HVAC-Setup", deviceID);
  Serial.printf("[FLASH] Starting portal SSID: %s (connect and open http://192.168.4.1)\n", apName);

  bool saved = wifiManager.startConfigPortal(apName);
  Serial.printf("[FLASH] Portal finished: %s\n", saved ? "SAVED/CONNECTED" : "TIMEOUT/EXIT");

  if (shouldSaveConfig) {
    Serial.println(F("[FLASH] Applying params from portal..."));
    deviceConfig.mqttEnable = (uint8_t)atoi(p_mqtt_en.getValue());
    strlcpy(deviceConfig.mqttServer, p_mqtt_server.getValue(), sizeof(deviceConfig.mqttServer));
    deviceConfig.mqttPort = (uint16_t)atoi(p_mqtt_port.getValue());
    strlcpy(deviceConfig.mqttUser, p_mqtt_user.getValue(), sizeof(deviceConfig.mqttUser));
    strlcpy(deviceConfig.mqttPassword, p_mqtt_pass.getValue(), sizeof(deviceConfig.mqttPassword));
    strlcpy(deviceConfig.mqttCmdTopic, p_mqtt_cmd.getValue(), sizeof(deviceConfig.mqttCmdTopic));
    strlcpy(deviceConfig.mqttStateTopic, p_mqtt_state.getValue(), sizeof(deviceConfig.mqttStateTopic));
    strlcpy(deviceConfig.mqttHeartbeatTopic, p_mqtt_hb.getValue(), sizeof(deviceConfig.mqttHeartbeatTopic));

    deviceConfig.useCustomId = (uint8_t)atoi(p_dev_id_en.getValue());
    strlcpy(deviceConfig.customDeviceId, p_dev_id.getValue(), sizeof(deviceConfig.customDeviceId));
    sanitizeDeviceId(deviceConfig.customDeviceId);

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

    // OTA (WiFiManager parametry)
    deviceConfig.otaEnable = (uint8_t)atoi(p_ota_en.getValue());
    strlcpy(deviceConfig.otaPassword, p_ota_pass.getValue(), sizeof(deviceConfig.otaPassword));

    // Telnet
    deviceConfig.telnetEnable = (uint8_t)atoi(p_telnet_en.getValue());
    strlcpy(deviceConfig.telnetPassword, p_telnet_pass.getValue(), sizeof(deviceConfig.telnetPassword));

    saveConfig();
    shouldSaveConfig = false;
    computeDeviceID();
  }

  // návrat do STA + restart služeb
  WiFi.mode(WIFI_STA);

  mqttEnabled = deviceConfig.mqttEnable != 0;
  if (mqttEnabled && deviceConfig.mqttServer[0]) {
    configureMqttClient();
  }

  server.begin();
  if (modbusWasActive || deviceConfig.modbusEnable) {
    modbusSetup();
  }

  // Telnet
  telnetEnabled = (deviceConfig.telnetEnable != 0);
  if (telnetEnabled) { telnetServer.begin(); telnetServer.setNoDelay(true); }

  Serial.print(F("[FLASH] Current IP (STA): "));
  Serial.println(WiFi.localIP());
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
            "<form method='POST' action='/fwupdate' enctype='multipart/form-data'>"
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
  if (!handleAuthentication()) return; // jistota – i během streamu
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
            "<button class='btn bad' type='submit' name='confirm' value='YES'>Provést Factory Reset</button>"
            " <a class='btn' href='/'>Zpět</a>"
            "</form>"
            "</div></div>");
  server.send(200, "text/html", html);
}

static void handleFactoryResetPost() {
  if (!handleAuthentication()) return;
  if (server.hasArg("confirm") && server.arg("confirm") == "YES") {
    server.send(200, "text/plain", "Factory reset… restartuji.");
    server.client().stop();
    delay(200);
    doFactoryReset();
  } else {
    server.send(400, "text/plain", "Chybí potvrzení.");
  }
}

// ================== Setup/Loop ==================
void setup() {
  // LEDy
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // zhasnuto (active LOW)
  pinMode(FLASH_BTN_PIN, INPUT_PULLUP);

  if (RED_LED_PIN >= 0) {
    pinMode(RED_LED_PIN, OUTPUT);
    #if defined(ESP8266)
    analogWriteRange(PWM_MAX);
    #endif
    pwmWriteCompat(RED_LED_PIN, 0);
  }

  Serial.begin(115200);
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

  loadConfig();

  // DeviceID pro SSID již teď (před autoConnect)
  computeDeviceID();
  computeDeviceLabel();

  // >>> Boot red LED na 5 s
  if (RED_LED_PIN >= 0) {
    bootRedActive = true;
    bootRedUntil  = millis() + BOOT_RED_LED_MS;
    analogWrite(RED_LED_PIN, 1023);   // rozsvítit hned
  }

  // WiFiManager parametry – předvyplnit z EEPROM
  char mqttEnStr[2]; snprintf(mqttEnStr, sizeof(mqttEnStr), "%u", (unsigned)deviceConfig.mqttEnable);
  p_mqtt_en.setValue(mqttEnStr, sizeof(mqttEnStr));
  p_mqtt_server.setValue(deviceConfig.mqttServer, sizeof(deviceConfig.mqttServer));
  char portStr[6]; snprintf(portStr, sizeof(portStr), "%u", deviceConfig.mqttPort);
  p_mqtt_port.setValue(portStr, sizeof(portStr));
  p_mqtt_user.setValue(deviceConfig.mqttUser, sizeof(deviceConfig.mqttUser));
  p_mqtt_pass.setValue(deviceConfig.mqttPassword, sizeof(deviceConfig.mqttPassword));
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
  p_ota_pass.setValue(deviceConfig.otaPassword, sizeof(deviceConfig.otaPassword));

  // Telnet parametry
  char tEnStr[2]; snprintf(tEnStr, sizeof(tEnStr), "%u", (unsigned)deviceConfig.telnetEnable);
  p_telnet_en.setValue(tEnStr, sizeof(tEnStr));
  p_telnet_pass.setValue(deviceConfig.telnetPassword, sizeof(deviceConfig.telnetPassword));

  wifiManager.setSaveConfigCallback(saveConfigCallback);

  // Přesnější chování připojení: kratší pokusy o STA, pak AP portál
  wifiManager.setConnectTimeout(30);  // sekund pokus o STA
  wifiManager.setConnectRetries(2);   // kolikrát zkusit
  wifiManager.setTimeout(300);        // sekund běží portál (když se otevře)

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

  // Připojení / AP fallback (standardní režim STA; do AP se přepíná jen pro portál)
  WiFi.mode(WIFI_STA);
  {
    char apName[40];
    snprintf(apName, sizeof(apName), "Toshiba-%s-HVAC-Setup", deviceID);
    wifiManager.autoConnect(apName);
  }
  Serial.print(F("WiFi IP: ")); Serial.println(WiFi.localIP());

  if (shouldSaveConfig) {
    Serial.println(F("Ukladam WiFiManager parametry..."));
    deviceConfig.mqttEnable = (uint8_t)atoi(p_mqtt_en.getValue());
    strlcpy(deviceConfig.mqttServer, p_mqtt_server.getValue(), sizeof(deviceConfig.mqttServer));
    deviceConfig.mqttPort = (uint16_t)atoi(p_mqtt_port.getValue());
    strlcpy(deviceConfig.mqttUser, p_mqtt_user.getValue(), sizeof(deviceConfig.mqttUser));
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
    strlcpy(deviceConfig.otaPassword, p_ota_pass.getValue(), sizeof(deviceConfig.otaPassword));

    deviceConfig.telnetEnable = (uint8_t)atoi(p_telnet_en.getValue());
    strlcpy(deviceConfig.telnetPassword, p_telnet_pass.getValue(), sizeof(deviceConfig.telnetPassword));

    saveConfig();
  }

  // DeviceID dle configu (znovu pro případ změny custom ID)
  computeDeviceID();

  // MQTT – povol dle configu, jen pokud je host nastaven
  mqttEnabled = deviceConfig.mqttEnable != 0;
  if (mqttEnabled && isMqttHostConfigured()) {
    #if (USE_MQTT_TLS)
      wifiClientTLS.setInsecure(); // volitelně (vývoj)
    #endif // volitelně (vývoj)
    configureMqttClient();
  }

  // Web
  server.on("/", handleRoot);
  server.on("/command", handleCommand);
  server.on("/mqttconfig", handleMQTTConfig);
  server.on("/debug", handleDebug);
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
    doc["pass"]    = deviceConfig.mqttPassword;
    doc["cmdtopic"]= deviceConfig.mqttCmdTopic;
    doc["statetopic"] = deviceConfig.mqttStateTopic;
    doc["hbtopic"] = deviceConfig.mqttHeartbeatTopic;

    doc["webuser"] = deviceConfig.webUser;
    doc["webpass"] = deviceConfig.webPassword;

    doc["dev_id_en"] = deviceConfig.useCustomId;
    doc["dev_id"]    = deviceConfig.customDeviceId;
    doc["room"] = deviceConfig.room;
    doc["name"] = deviceConfig.unitName;

    doc["mb_en"]   = deviceConfig.modbusEnable;
    doc["mb_port"] = deviceConfig.modbusPort;   // info
    doc["mb_uid"]  = deviceConfig.modbusUnitId; // info

    doc["ota_en"]  = deviceConfig.otaEnable;
    doc["ota_pass"]= deviceConfig.otaPassword;

    doc["telnet_en"]   = deviceConfig.telnetEnable;
    doc["telnet_pass"] = deviceConfig.telnetPassword;

    String out; out.reserve(measureJson(doc)+8);
    serializeJson(doc, out);
    server.send(200, "application/json", out);
  });

  server.on("/api/state.json", [](){
    if (!handleAuthentication()) return;
    StaticJsonDocument<448> doc;
    doc["power"] = nz(hvac.getState());
    doc["temp"]  = (int)hvac.getSetpoint();
    doc["mode"]  = nz(hvac.getMode());
    doc["fan"]   = nz(hvac.getFanMode());
    doc["swing"] = nz(hvac.getSwing());

    // UART/komunikace s jednotkou
    doc["hvac"]  = (hvac.isConnected() || hvac.isCduRunning()) ? "Active" : "Inactive";
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

  server.begin();
  Serial.println(F("Web server OK."));

  // Telnet
  telnetEnabled = (deviceConfig.telnetEnable != 0);
  if (telnetEnabled) {
    telnetServer.begin();
    telnetServer.setNoDelay(true);
    Serial.println(F("Telnet server OK (23)."));
  } else {
    Serial.println(F("Telnet server disabled."));
  }

  // OTA (hostname = "toshiba-" + deviceID) – pouze když povoleno
  if (deviceConfig.otaEnable) {
    char host[24]; snprintf(host, sizeof(host), "toshiba-%s", deviceID);
    ArduinoOTA.setHostname(host);
    if (deviceConfig.otaPassword[0]) { ArduinoOTA.setPassword(deviceConfig.otaPassword); }
    ArduinoOTA.onStart([]() { Serial.println(F("OTA start")); otaActive = true; otaStartMs = millis(); });
    ArduinoOTA.onEnd([]()   { Serial.println(F("OTA end")); otaActive = false; if (RED_LED_PIN >= 0) analogWrite(RED_LED_PIN, 0); });
    ArduinoOTA.onProgress([](unsigned int p, unsigned int t) { uint32_t pct = (t ? (p * 100U / t) : 0U); Serial.printf("OTA: %u%%\r\n", pct); });
    ArduinoOTA.onError([](ota_error_t e) { Serial.printf("OTA err %u\r\n", e); otaActive = false; if (RED_LED_PIN >= 0) analogWrite(RED_LED_PIN, 0); });
    ArduinoOTA.begin();
    otaStarted = true;
    Serial.print(F("OTA ready, hostname: ")); Serial.println(host);
  } else {
    otaStarted = false;
    Serial.println(F("OTA disabled."));
  }

  // Modbus TCP – spustit jen když povoleno
  modbusSetup();

  lastHvacResponse = millis();

  // Volitelný rychlý “probe” po startu pro debug linky
  hvac.handleHvac();
  delay(10);
  hvacStatus s = hvac.getStatus();
  Serial.printf("HVAC probe: room=%d, out=%d\n", (int)s.roomTemperature, (int)s.outsideTemperature);
}

void loop() {
  if (deviceConfig.otaEnable) {
    ArduinoOTA.handle();
  }

  // >>> nejdřív obsluha 5s boot LED, pak případně OTA fade
  bootRedLedHandle();
  otaLedHandle(); // animace červené LED během OTA

  if (mqttEnabled) {
    if (!mqttClient.connected()) connectToMQTT();
    if (mqttClient.connected()) mqttClient.loop();
  }

  server.handleClient();
  handleTelnet();
  handleFlashButton();  // krátký stisk FLASH otevře WiFiManager portál

  // HVAC servisní funkce
  hvac.handleHvac();

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
      mqttClient.publish(topic, reinterpret_cast<const uint8_t*>(payload.c_str()), payload.length(), false);
      Serial.print(F("Heartbeat: ")); Serial.println(payload);
    }
    if (modbusActive) {
      mb.Hreg(HREG_STATUS, buildStatusReg());
      mb.Hreg(HREG_UPTIME_S, (uint16_t)((millis()/1000UL) & 0xFFFF));
    }
    lastHeartbeat = now;
  }

  // LED diagnostika (LOW = svítí); mimo OTA fade
  if (!otaActive) {
    bool hvacConn = (now - lastHvacResponse) < HVAC_TIMEOUT_MS;
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
