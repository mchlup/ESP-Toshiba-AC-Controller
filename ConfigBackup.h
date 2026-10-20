#pragma once
#include <Arduino.h>

// Export/import intentionally excludes all stored passwords and Wi-Fi secrets;
// those are neither returned in plaintext nor silently overwritten.
static void handleConfigBackup() {
  if(!server.authenticate(deviceConfig.webUser,deviceConfig.webPassword)){server.requestAuthentication();return;}
  DynamicJsonDocument doc(1400);
  doc["schema"]=1;
  doc["name"]=deviceConfig.unitName; doc["room"]=deviceConfig.room;
  doc["dev_id_en"]=deviceConfig.useCustomId;
  doc["dev_id"]=deviceConfig.customDeviceId;
  doc["mqtt_en"]=deviceConfig.mqttEnable;
  doc["server"]=deviceConfig.mqttServer;
  doc["port"]=deviceConfig.mqttPort;
  doc["user"]=deviceConfig.mqttUser;
  doc["cmdtopic"]=deviceConfig.mqttCmdTopic;
  doc["statetopic"]=deviceConfig.mqttStateTopic;
  doc["hbtopic"]=deviceConfig.mqttHeartbeatTopic;
  doc["webuser"]=deviceConfig.webUser;
  doc["mb_en"]=deviceConfig.modbusEnable;
  doc["ota_en"]=deviceConfig.otaEnable;
  doc["telnet_en"]=deviceConfig.telnetEnable;
  doc["ha_discovery"]=extCfg.discovery;
  addCapabilities(doc.createNestedObject("capabilities"));
  if(doc.overflowed()){server.send(500,"text/plain","Backup too large");return;}
  String output;output.reserve(measureJson(doc)+8);serializeJsonPretty(doc,output);
  server.sendHeader("Cache-Control","no-store");
  server.sendHeader("Content-Disposition","attachment; filename=toshiba-config.json");
  server.send(200,"application/json",output);
}
static void handleConfigImport() {
  if(!server.authenticate(deviceConfig.webUser,deviceConfig.webPassword)){server.requestAuthentication();return;}
  if(server.header("X-Requested-With")!="XMLHttpRequest"||!requireCsrf())return;
  if(!server.hasArg("backup")||server.arg("backup").length()>3400){
    server.send(400,"text/plain","Missing/large backup");return;
  }
  DynamicJsonDocument doc(2000);
  if(deserializeJson(doc,server.arg("backup"))||doc["schema"].as<int>()!=1){
    server.send(400,"text/plain","Invalid backup schema");return;
  }
  DeviceConfig staged=deviceConfig;
  ExtensionConfig stageExt=extCfg;
  auto textField=[&](const char* field,char* dest,size_t maxLen)->bool {
    if(doc[field].isNull())return true;
    if(!doc[field].is<const char*>())return false;
    const char* input=doc[field].as<const char*>();
    if(!input||strlen(input)>=maxLen)return false;
    strlcpy(dest,input,maxLen);
    return true;
  };
  bool ok=true;
  ok &= textField("name",staged.unitName,sizeof(staged.unitName));
  ok &= textField("room",staged.room,sizeof(staged.room));
  ok &= textField("dev_id",staged.customDeviceId,sizeof(staged.customDeviceId));
  ok &= textField("server",staged.mqttServer,sizeof(staged.mqttServer));
  ok &= textField("user",staged.mqttUser,sizeof(staged.mqttUser));
  ok &= textField("cmdtopic",staged.mqttCmdTopic,sizeof(staged.mqttCmdTopic));
  ok &= textField("statetopic",staged.mqttStateTopic,sizeof(staged.mqttStateTopic));
  ok &= textField("hbtopic",staged.mqttHeartbeatTopic,sizeof(staged.mqttHeartbeatTopic));
  ok &= textField("webuser",staged.webUser,sizeof(staged.webUser));
  auto u8=[&](const char* field,uint8_t& dest)->bool {
    if(doc[field].isNull())return true;
    if(!doc[field].is<int>())return false;
    int value=doc[field].as<int>();
    if(value<0||value>1)return false;
    dest=static_cast<uint8_t>(value);
    return true;
  };
  ok &= u8("dev_id_en",staged.useCustomId);
  ok &= u8("mqtt_en",staged.mqttEnable);
  ok &= u8("mb_en",staged.modbusEnable);
  ok &= u8("ota_en",staged.otaEnable);
  ok &= u8("telnet_en",staged.telnetEnable);
  ok &= u8("ha_discovery",stageExt.discovery);
  if(!doc["port"].isNull()){
    if(!doc["port"].is<int>())ok=false;
    else {int port=doc["port"].as<int>();if(port<1||port>65535)ok=false;else staged.mqttPort=port;}
  }
  if(doc["capabilities"].is<JsonObject>()){
    JsonObject caps=doc["capabilities"].as<JsonObject>();
    for(int i=0;i<7;++i){
      if(caps[CAP_NAMES[i]].isNull())continue;
      const char* status=caps[CAP_NAMES[i]].as<const char*>();
      if(!status)ok=false;
      else if(!strcmp(status,"supported"))stageExt.caps[i]=1;
      else if(!strcmp(status,"unsupported"))stageExt.caps[i]=2;
      else if(!strcmp(status,"unknown"))stageExt.caps[i]=0;
      else ok=false;
    }
  }
  if(!ok){server.send(400,"text/plain","Backup contains invalid fields");return;}
  sanitizeDeviceId(staged.customDeviceId);
  const bool oldHa=extCfg.discovery!=0;
  if(oldHa && !stageExt.discovery)publishHaRemoval();
  deviceConfig=staged;extCfg=stageExt;
  saveConfig();saveExtensions();
  computeDeviceID();computeDeviceLabel();
  mqttEnabled=deviceConfig.mqttEnable!=0;
  if(mqttClient.connected())mqttClient.disconnect();
  if(mqttEnabled && isMqttHostConfigured())configureMqttClient();
  telnetEnabled=deviceConfig.telnetEnable!=0;
  if(!telnetEnabled&&telnetClient)telnetClient.stop();
  discoveryPending=true;
  server.send(200,"text/plain","Backup applied, passwords retained");
}
