#pragma once
#include <Arduino.h>

// Extended settings live at an isolated EEPROM offset, preserving the binary
// layout/CRC of DeviceConfig already installed on users' ESP boards.
// Never silently migrate or overwrite the existing DeviceConfig.
enum CapabilityId : uint8_t {
  CAP_SWING_FIXED = 0,
  CAP_SWING_VERTICAL,
  CAP_SWING_HORIZONTAL,
  CAP_SWING_BOTH,
  CAP_SWING_POSITIONS,
  CAP_PURE,
  CAP_POWER_SELECT,
  CAP_OPERATION,
  CAP_WIFI_LED,
  CAP_COUNT
};
static const char* const CAPABILITY_KEYS[CAP_COUNT] = {
  "swing_fixed", "swing_vertical", "swing_horizontal", "swing_both",
  "swing_positions", "pure", "power_select", "operation", "wifi_led"
};
enum CapabilitySetting : uint8_t {
  CAP_AUTO = 0, CAP_SUPPORTED = 1, CAP_UNSUPPORTED = 2
};

// Versioned sidecar; extend with a new schema instead of reinterpreting bytes.
static const uint32_t FEATURE_MAGIC = 0x544F5348UL;  // TOSH
static const uint8_t FEATURE_SCHEMA = 1;
static const size_t FEATURE_EEPROM_OFFSET = 768;

struct FeatureOptions {
  uint32_t magic;
  uint8_t version;
  uint8_t haDiscovery;  // 0/1, explicitly enabled by user
  uint8_t capability[CAP_COUNT];  // 0 auto, 1 supported, 2 unsupported
  uint16_t crc;
};

static inline int featureKeyToIndex(const char* key) {
  if (!key) return -1;
  for (unsigned i = 0; i < CAP_COUNT; ++i)
    if (strcmp(key, CAPABILITY_KEYS[i]) == 0) return static_cast<int>(i);
  return -1;
}
