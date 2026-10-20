#pragma once
#include <Arduino.h>

// Additional HOLDING registers. Existing 0..11 addresses stay unchanged.
// Counters are split into 16-bit words (low at 100/101/102, high at 110/111/112).
static uint32_t maxLoopDurationMs = 0;
static uint16_t resetReasonCode() {
#if defined(ESP32)
  return (uint16_t)esp_reset_reason();
#else
  const rst_info* info=ESP.getResetInfoPtr();
  return info?(uint16_t)info->reason:0xFFFF;
#endif
}
static uint16_t diagLimitU16(uint32_t value) { return value > 65535U ? 65535U : (uint16_t)value; }
static const uint16_t DIAG_START = 100;
static const uint16_t DIAG_END   = 112;
static void modbusAddDiagnostics() {
  for(uint16_t address=DIAG_START; address<=DIAG_END; ++address) {
    mb.addHreg(address,0);
  }
}
static void modbusUpdateDiagnostics() {
  if(!modbusActive) return;
  const uint32_t good=hvac.validPackets();
  const uint32_t bad=hvac.invalidPackets();
  const uint32_t handshakes=hvac.handshakeAttempts();
  const uint32_t lastRx=hvac.lastValidRxMs();
  mb.Hreg(100,static_cast<uint16_t>(good));
  mb.Hreg(101,static_cast<uint16_t>(bad));
  mb.Hreg(102,static_cast<uint16_t>(handshakes));
  mb.Hreg(103,lastRx ? diagLimitU16((millis()-lastRx)/1000UL) : 65535);
  mb.Hreg(104,WiFi.status()==WL_CONNECTED ? static_cast<uint16_t>((int16_t)WiFi.RSSI()) : 32767);
  mb.Hreg(105,diagLimitU16(ESP.getFreeHeap()/1024UL));
  mb.Hreg(106,static_cast<uint16_t>(mqttConnectCount));
  mb.Hreg(107,resetReasonCode()); // platform-specific reset reason code
  mb.Hreg(108,lastCommandStatus);
  mb.Hreg(109,diagLimitU16(maxLoopDurationMs));
  mb.Hreg(110,static_cast<uint16_t>(good>>16));
  mb.Hreg(111,static_cast<uint16_t>(bad>>16));
  mb.Hreg(112,static_cast<uint16_t>(handshakes>>16));
}
static void handleMetrics() {
  if(!server.authenticate(deviceConfig.webUser,deviceConfig.webPassword)){server.requestAuthentication();return;}
  StaticJsonDocument<768> doc;
  doc["uptime_s"]=millis()/1000UL;
  doc["wifi_connected"]=WiFi.status()==WL_CONNECTED;
  if(WiFi.status()==WL_CONNECTED)doc["wifi_rssi_dbm"]=WiFi.RSSI();
  doc["mqtt_connected"]=mqttEnabled && mqttClient.connected();
  doc["mqtt_connections"]=mqttConnectCount;
  doc["hvac_connected"]=hvac.isConnected();
  doc["rx_batches_valid"]=hvac.validPackets();
  doc["rx_batches_invalid"]=hvac.invalidPackets();
  doc["state_changes"]=hvac.stateChanges();
  doc["handshakes"]=hvac.handshakeAttempts();
  doc["uart_age_ms"]=hvac.lastValidRxMs()?millis()-hvac.lastValidRxMs():0;
  doc["free_heap_bytes"]=ESP.getFreeHeap();
  doc["max_loop_ms"]=maxLoopDurationMs;
  doc["last_command_status"]=lastCommandStatus;
  doc["reset_reason_code"]=resetReasonCode();
  if(doc.overflowed()){server.send(500,"text/plain","JSON overflow");return;}
  String result;serializeJson(doc,result);
  server.send(200,"application/json",result);
}
