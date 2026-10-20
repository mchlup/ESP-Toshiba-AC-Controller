#pragma once
#include <Arduino.h>

// Included after processCommand() and the global MQTT state in the main sketch.
// Discovery publishes one retained climate config for this controller only.
// No UART timing/handshake changes.
static void haTopic(char* out, size_t cap, const char* suffix) {
  char prefix[80];
  snprintf(prefix, sizeof(prefix), "toshiba/%s", deviceID);
  snprintf(out, cap, "%s/%s", prefix, suffix);
}
static bool haCommand(const char* topic, const char* payload) {
  if (!featureOptions.haDiscovery || !topic || !payload) return false;
  char expected[96];
  haTopic(expected, sizeof(expected), "ha/mode/set");
  if (!strcmp(topic, expected)) {
    if (!strcasecmp(payload, "off")) return processCommand("STATE_OFF");
    if (!strcasecmp(payload, "auto")) return processCommand("MODE_AUTO") && processCommand("STATE_ON");
    if (!strcasecmp(payload, "cool")) return processCommand("MODE_COOL") && processCommand("STATE_ON");
    if (!strcasecmp(payload, "heat")) return processCommand("MODE_HEAT") && processCommand("STATE_ON");
    if (!strcasecmp(payload, "dry")) return processCommand("MODE_DRY") && processCommand("STATE_ON");
    if (!strcasecmp(payload, "fan_only")) return processCommand("MODE_FAN_ONLY") && processCommand("STATE_ON");
    return false;
  }
  haTopic(expected, sizeof(expected), "ha/temp/set");
  if (!strcmp(topic, expected)) {
    char* end = nullptr;
    const float val = strtof(payload, &end);
    if (end == payload || !end || *end || !isfinite(val) || val < 17 || val > 30) return false;
    const int target = (int)lroundf(val);
    char cmd[16]; snprintf(cmd, sizeof(cmd), "TEMP_%d", target);
    return processCommand(cmd);
  }
  haTopic(expected, sizeof(expected), "ha/fan/set");
  if (!strcmp(topic, expected)) {
    if (!strcasecmp(payload, "auto")) return processCommand("FAN_AUTO");
    for (int i=1; i<=5; ++i) {
      char mode[8], cmd[8];
      snprintf(mode, sizeof(mode), "lvl_%d", i);
      snprintf(cmd, sizeof(cmd), "FAN_%d", i);
      if (!strcasecmp(mode, payload)) return processCommand(cmd);
    }
    return false;
  }
  haTopic(expected, sizeof(expected), "ha/swing/set");
  if (!strcmp(topic, expected)) {
    if (!strcmp(payload, "fix")) return processCommand("SWING_OFF");
    if (!strcmp(payload, "v_swing")) return processCommand("SWING_V");
    if (!strcmp(payload, "h_swing")) return processCommand("SWING_H");
    if (!strcmp(payload, "hv_swing")) return processCommand("SWING_VH");
    for (int i=1;i<=5;i++){
      char pos[14], cmd[16];
      snprintf(pos,sizeof(pos),"fix_pos_%d",i);
      snprintf(cmd,sizeof(cmd),"SWING_FIX_%d",i);
      if (!strcmp(payload,pos)) return processCommand(cmd);
    }
  }
  return false;
}
static void haSubscribe() {
  if (!featureOptions.haDiscovery || !mqttClient.connected()) return;
  const char* paths[] = {"ha/mode/set", "ha/temp/set", "ha/fan/set", "ha/swing/set"};
  for (const char* suffix: paths) {
    char topic[96]; haTopic(topic,sizeof(topic),suffix);
    mqttClient.subscribe(topic, 1);
  }
}
static void haDiscoveryTopic(char* topic, size_t len) {
  snprintf(topic,len,"homeassistant/climate/toshiba_%s/config",deviceID);
}
static bool haPublishDiscovery(bool enabled) {
  if (!mqttClient.connected()) return false;
  char discoveryTopic[110];
  haDiscoveryTopic(discoveryTopic,sizeof(discoveryTopic));
  if (!enabled) {
    const bool ok = mqttClient.publish(discoveryTopic, "", true);
    if (ok) haDiscoveryPublished = false;
    return ok;
  }
  DynamicJsonDocument doc(2048);
  doc["name"] = deviceLabel[0] ? deviceLabel : "Toshiba HVAC";
  char unique[40]; snprintf(unique,sizeof(unique),"toshiba_hvac_%s",deviceID);
  doc["uniq_id"] = unique;
  char topic[96];
  expandTopic(deviceConfig.mqttStateTopic,topic,sizeof(topic));
  doc["mode_stat_t"] = topic;
  doc["mode_stat_tpl"] = "{{ 'off' if value_json.power == 'off' else value_json.mode }}";
  doc["temp_stat_t"] = topic;
  doc["temp_stat_tpl"] = "{{ value_json.temp }}";
  doc["curr_temp_t"] = topic;
  doc["curr_temp_tpl"] = "{{ value_json.t_in | default(none) }}";
  doc["fan_mode_stat_t"] = topic;
  doc["fan_mode_stat_tpl"] = "{{ value_json.fan }}";
  doc["swing_mode_stat_t"] = topic;
  doc["swing_mode_stat_tpl"] = "{{ value_json.swing }}";
  haTopic(topic,sizeof(topic),"ha/mode/set");
  doc["mode_cmd_t"] = topic;
  haTopic(topic,sizeof(topic),"ha/temp/set");
  doc["temp_cmd_t"] = topic;
  haTopic(topic,sizeof(topic),"ha/fan/set");
  doc["fan_mode_cmd_t"] = topic;
  haTopic(topic,sizeof(topic),"ha/swing/set");
  doc["swing_mode_cmd_t"] = topic;
  haTopic(topic,sizeof(topic),"status");
  doc["avty_t"] = topic;
  doc["pl_avail"] = "online";
  doc["pl_not_avail"] = "offline";
  doc["min_temp"] = 17;
  doc["max_temp"] = 30;
  doc["temp_step"] = 1;
  doc["temp_unit"] = "C";
  doc["precision"] = 1.0;
  JsonArray modes=doc.createNestedArray("modes");
  for (const char* val: {"off","auto","cool","heat","dry","fan_only"}) modes.add(val);
  JsonArray fans=doc.createNestedArray("fan_modes");
  for (const char* val: {"auto","lvl_1","lvl_2","lvl_3","lvl_4","lvl_5"}) fans.add(val);
  JsonArray swings=doc.createNestedArray("swing_modes");
  if (capabilityEnabled(CAP_SWING_FIXED)) swings.add("fix");
  if (capabilityEnabled(CAP_SWING_VERTICAL)) swings.add("v_swing");
  if (capabilityEnabled(CAP_SWING_HORIZONTAL)) swings.add("h_swing");
  if (capabilityEnabled(CAP_SWING_BOTH)) swings.add("hv_swing");
  if (featureOptions.capability[CAP_SWING_POSITIONS] == CAP_SUPPORTED)
    for(int i=1;i<=5;++i) { char t[16]; snprintf(t,sizeof(t),"fix_pos_%d",i); swings.add(t); }
  if (swings.size()==0) {
    doc.remove("swing_modes");
    doc.remove("swing_mode_cmd_t");
    doc.remove("swing_mode_stat_t");
    doc.remove("swing_mode_stat_tpl");
  }
  JsonObject dev=doc.createNestedObject("dev");
  JsonArray ids=dev.createNestedArray("ids");ids.add(unique);
  dev["name"] = deviceLabel[0]?deviceLabel:"Toshiba HVAC";
  dev["mf"] = "Toshiba / ESP";
  dev["mdl"] = "ESP-Toshiba-AC";
  dev["sw"] = "2.0";
  if (doc.overflowed()) return false;
  String payload;payload.reserve(measureJson(doc)+1);
  serializeJson(doc,payload);
  if (!mqttClient.publish(discoveryTopic, reinterpret_cast<const uint8_t*>(payload.c_str()),payload.length(),true)){
    Serial.println(F("[HA] MQTT discovery publish failed"));
    return false;
  }
  haDiscoveryPublished=true;
  haSubscribe();
  Serial.printf("[HA] Discovery published: %s (%u bytes)\n",discoveryTopic,(unsigned)payload.length());
  return true;
}
