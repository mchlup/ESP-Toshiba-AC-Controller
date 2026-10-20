#pragma once
#include <Arduino.h>
// This header is included after publishState(). Existing MQTT topic/command
// names remain unchanged. Configuration is retained over broker reconnects.
static void publishHaDiscovery() {
  if (!extCfg.discovery || !mqttEnabled || !mqttClient.connected()) return;
  char topic[96],stateTopic[96],cmdTopic[96],statusTopic[96];
  snprintf(topic,sizeof(topic),"homeassistant/climate/toshiba_%s/config",deviceID);
  expandTopic(deviceConfig.mqttStateTopic,stateTopic,sizeof(stateTopic));
  expandTopic(deviceConfig.mqttCmdTopic,cmdTopic,sizeof(cmdTopic));
  expandTopic("toshiba/{id}/status",statusTopic,sizeof(statusTopic));
  DynamicJsonDocument doc(2400);
  doc["name"]=deviceLabel;
  doc["unique_id"]=String("toshiba_")+deviceID;
  doc["availability_topic"]=statusTopic;
  doc["payload_available"]="online";
  doc["payload_not_available"]="offline";
  doc["mode_state_topic"]=stateTopic;
  doc["mode_state_template"]="{{ 'off' if value_json.power == 'off' else value_json.mode }}";
  doc["mode_command_topic"]=cmdTopic;
  doc["mode_command_template"]="{% if value == 'off' %}STATE_OFF{% else %}HA_MODE_{{ value | upper }}{% endif %}";
  doc["temperature_state_topic"]=stateTopic;
  doc["temperature_state_template"]="{{ value_json.temp }}";
  doc["temperature_command_topic"]=cmdTopic;
  doc["temperature_command_template"]="TEMP_{{ value | int }}";
  doc["current_temperature_topic"]=stateTopic;
  doc["current_temperature_template"]="{{ value_json.t_in | default(none) }}";
  doc["fan_mode_state_topic"]=stateTopic;
  doc["fan_mode_state_template"]="{{ value_json.fan }}";
  doc["fan_mode_command_topic"]=cmdTopic;
  doc["fan_mode_command_template"]="{% if value == 'auto' %}FAN_AUTO{% else %}FAN_{{ value | replace('lvl_', '') }}{% endif %}";
  JsonArray modes=doc.createNestedArray("modes");
  modes.add("off");modes.add("auto");modes.add("heat");modes.add("cool");modes.add("dry");modes.add("fan_only");
  JsonArray fans=doc.createNestedArray("fan_modes");
  fans.add("auto");for(int i=1;i<=5;++i)fans.add(String("lvl_")+i);
  if(capAllowed(0)||capAllowed(1)||capAllowed(2)||extCfg.caps[3]==1) {
    doc["swing_mode_state_topic"]=stateTopic;
    doc["swing_mode_state_template"]="{{ value_json.swing }}";
    doc["swing_mode_command_topic"]=cmdTopic;
    doc["swing_mode_command_template"]="{% if value == 'fix' %}SWING_OFF{% elif value.startswith('fix_pos_') %}SWING_FIX_{{ value | replace('fix_pos_', '') }}{% elif value == 'v_swing' %}SWING_V{% elif value == 'h_swing' %}SWING_H{% else %}SWING_VH{% endif %}";
    JsonArray swings=doc.createNestedArray("swing_modes");
    swings.add("fix");
    if(capAllowed(0))swings.add("v_swing");
    if(capAllowed(1))swings.add("h_swing");
    if(capAllowed(2))swings.add("hv_swing");
    if(extCfg.caps[3]==1)for(int p=1;p<=5;++p)swings.add(String("fix_pos_")+p);
  }
  doc["min_temp"]=17;doc["max_temp"]=30;doc["temp_step"]=1;
  doc["temperature_unit"]="C";
  JsonObject dev=doc.createNestedObject("device");
  JsonArray id=dev.createNestedArray("identifiers");id.add(String("toshiba_")+deviceID);
  dev["name"]=deviceLabel;dev["manufacturer"]="Toshiba";
  dev["model"]="ESP Toshiba HVAC";
  if(doc.overflowed()){Serial.println(F("HA discovery JSON too small"));return;}
  String json;json.reserve(measureJson(doc)+1);serializeJson(doc,json);
  if(!mqttClient.publish(topic,(const uint8_t*)json.c_str(),json.length(),true))
    Serial.println(F("HA discovery MQTT publish failed"));
}
static void publishHaRemoval() {
  if(!mqttClient.connected())return;
  char topic[96];snprintf(topic,sizeof(topic),"homeassistant/climate/toshiba_%s/config",deviceID);
  mqttClient.publish(topic,"",true);
}
static void handleExtensionsGet() {
  if(!server.authenticate(deviceConfig.webUser,deviceConfig.webPassword)){server.requestAuthentication();return;}
  StaticJsonDocument<400> doc;doc["discovery"]=extCfg.discovery;
  addCapabilities(doc.createNestedObject("capabilities"));
  String response;serializeJson(doc,response);
  server.send(200,"application/json",response);
}
static void handleExtensionsPost() {
  if(!server.authenticate(deviceConfig.webUser,deviceConfig.webPassword)){server.requestAuthentication();return;}
  if(server.header("X-Requested-With")!="XMLHttpRequest"){server.send(403,"text/plain","Invalid request");return;}
  if(!requireCsrf())return;
  // Stage the entire update, reject malformed fields before modifying EEPROM.
  ExtensionConfig next=extCfg;
  if(server.hasArg("discovery")) {
    const String value=server.arg("discovery");
    if(value!="0"&&value!="1"){server.send(400,"text/plain","Invalid discovery value");return;}
    next.discovery=(value=="1");
  }
  if(server.hasArg("feature")) {
    const int idx=capIndex(server.arg("feature"));
    const String value=server.arg("status");
    if(idx<0||(value!="0"&&value!="1"&&value!="2")){
      server.send(400,"text/plain","Invalid capability");return;
    }
    next.caps[idx]=(uint8_t)value.toInt();
  }
  for(int i=0;i<7;++i){
    const String field=String("cap_")+CAP_NAMES[i];
    if(!server.hasArg(field))continue;
    const String value=server.arg(field);
    if(value!="0"&&value!="1"&&value!="2"){
      server.send(400,"text/plain","Invalid capability");return;
    }
    next.caps[i]=(uint8_t)value.toInt();
  }
  bool changed=(next.discovery!=extCfg.discovery);
  for(int i=0;i<7;++i)changed=changed||(next.caps[i]!=extCfg.caps[i]);
  if(changed){
    if(extCfg.discovery && !next.discovery)publishHaRemoval();
    extCfg=next;saveExtensions();
    discoveryPending=true;
  }
  server.send(200,"text/plain",changed?"Saved":"Unchanged");
}
static uint32_t restartAfterMs=0;
static void handleRestart() {
  if(!server.authenticate(deviceConfig.webUser,deviceConfig.webPassword)){server.requestAuthentication();return;}
  if(server.header("X-Requested-With")!="XMLHttpRequest"){server.send(403,"text/plain","Invalid request");return;}
  if(!requireCsrf())return;
  server.send(202,"text/plain","ESP restarting");
  restartAfterMs=millis()+1200;
}
