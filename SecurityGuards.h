#pragma once
#include <Arduino.h>
#if defined(ESP8266)
  #include <user_interface.h>
#else
  #include <esp_system.h>
#endif

// Per-boot CSRF nonce. HTTP Basic authorization remains unchanged, including
// user-requested admin/admin defaults. Captive portal remains passwordless.
static char csrfNonce[25]={0};
static void initializeCsrfNonce() {
#if defined(ESP8266)
  uint32_t a=os_random(),b=os_random(),c=os_random();
#else
  uint32_t a=esp_random(),b=esp_random(),c=esp_random();
#endif
  snprintf(csrfNonce,sizeof(csrfNonce),"%08lx%08lx%08lx",
          (unsigned long)a,(unsigned long)b,(unsigned long)c);
}
static bool requestCsrfValid() {
  return server.hasArg("csrf") && server.arg("csrf") == csrfNonce;
}
static bool requireCsrf() {
  if(requestCsrfValid())return true;
  server.send(403,"text/plain","Invalid or missing CSRF token");
  return false;
}
static void handleCsrfToken() {
  if(!server.authenticate(deviceConfig.webUser,deviceConfig.webPassword)) {
    server.requestAuthentication();return;
  }
  StaticJsonDocument<80> doc;
  doc["csrf"]=csrfNonce;
  String out;serializeJson(doc,out);
  server.sendHeader("Cache-Control","no-store");
  server.send(200,"application/json",out);
}
