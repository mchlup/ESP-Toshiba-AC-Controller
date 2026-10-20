#pragma once
#include <Arduino.h>

// Persisted separately from DeviceConfig to preserve CONFIG_VERSION and CRC.
struct ExtensionConfig {
  uint32_t magic;
  uint8_t version;
  uint8_t discovery;            // 0=off, 1=on
  uint8_t caps[7];              // 0=unknown, 1=supported, 2=unsupported
  uint8_t reserved[8];
  uint16_t crc;
};
static constexpr uint32_t EXT_MAGIC = 0x54484332UL;
static constexpr uint8_t EXT_VERSION = 1;
static constexpr size_t EXT_EEPROM_OFFSET = 896;
static_assert(EXT_EEPROM_OFFSET + sizeof(ExtensionConfig) <= EEPROM_SIZE,
              "Extension config does not fit in EEPROM");
static ExtensionConfig extCfg {};
static bool discoveryPending = false;
static const char* const CAP_NAMES[7] = {
  "swing_v", "swing_h", "swing_both", "swing_fixed",
  "pure", "power_select", "special_operation"
};
static void extensionDefaults() {
  memset(&extCfg, 0, sizeof(extCfg));
  extCfg.magic = EXT_MAGIC;
  extCfg.version = EXT_VERSION;
  extCfg.discovery = 0; // opt-in for existing installations
}
static uint16_t extensionCrc() {
  ExtensionConfig copy = extCfg;
  copy.crc = 0;
  return crc16_ccitt(reinterpret_cast<const uint8_t*>(&copy), sizeof(copy));
}
static void saveExtensions() {
  extCfg.crc = extensionCrc();
  EEPROM.put(EXT_EEPROM_OFFSET, extCfg);
  EEPROM.commit();
}
static void loadExtensions() {
  EEPROM.get(EXT_EEPROM_OFFSET, extCfg);
  bool valid = extCfg.magic == EXT_MAGIC && extCfg.version == EXT_VERSION;
  if (valid) valid = (extCfg.crc == extensionCrc());
  for (unsigned i=0; valid && i<7; ++i) valid = extCfg.caps[i] <= 2;
  if (!valid) { extensionDefaults(); saveExtensions(); }
}
static int capIndex(const String& name) {
  for (int i=0; i<7; ++i) if (name == CAP_NAMES[i]) return i;
  return -1;
}
static const char* capText(uint8_t code) {
  return code == 1 ? "supported" : code == 2 ? "unsupported" : "unknown";
}
static void addCapabilities(JsonObject object) {
  for (unsigned i=0; i<7; ++i) object[CAP_NAMES[i]] = capText(extCfg.caps[i]);
}
static bool capAllowed(unsigned i) {
  return i<7 && extCfg.caps[i] != 2;
}
