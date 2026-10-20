# Firmware candidate: features, localization and stability

Based on `main@81947df18d02d54ac21dee27db3ce2e657000959` (20 Oct 2026).

## Preserved deliberately

* **WiFiManager captive AP stays open/no password**, per project request.
* WebUI credentials remain **admin/admin** until changed manually.
* Existing Toshiba UART 9600 8E1, RX/TX wiring, 6-frame handshake and original
  polling delays. The only UART-library behavior change treats a duplicate
  but correctly structured response as a received frame for diagnostics.
* EEPROM DeviceConfig layout/CRC remains byte-for-byte compatible. Additional
  HA/capability options use a versioned record at EEPROM offset 768.
* Existing Modbus registers 0..11 unchanged.

## Added

1. Home Assistant MQTT Discovery (disabled by default): retained climate
   announcement, availability LWT, mode/temp/fan/swing commands, command
   validation, retained announcement removal on disable. Only deliberately
   marked supported manual swing positions appear in Home Assistant.
2. Capability Manager: each feature auto/unverified, supported, unsupported.
   Manual service observation may confirm physical support of swing positions.
   Unsupported controls are rejected in API and omitted from HA discovery.
3. English and Czech localization via individual JSON language files:
   `locale/cs.json`, `locale/en.json`, with flash-ready copies
   `LocaleCs.h` and `LocaleEn.h`. `/locale/{cs,en}.json` serve
   their text from flash, without CDN.
4. Password-redacted backup/restore `/api/backup` and `/api/restore`.
   Restore does not alter Wi-Fi or credential secrets; validate inputs first.
5. WebUI restart `POST /api/restart` with explicit user confirmation.
6. CSRF protection for browser controls/service/config/update/reset with
   per-boot token from `GET /api/security/token`.
7. Nonblocking BOOT-button captive portal; expiry via explicit
   `stopConfigPortal()`, with no UART polling interruption.
8. Restore services after failed/aborted HTTP firmware update.
9. MQTT buffer enlarged to 2048 bytes, publish failures are logged.
10. Modbus diagnostics 100-112 and expanded console diagnostics.
11. Build workflow for ESP8266, ESP32 and ESP32-C3.

## Limitations and precautions

* No HVAC hardware tests or real unit captures are available remotely;
  communication needs validation on the actual device.
* Wi-Fi AP and default WebUI password intentionally remain insecure.
  Run in a segregated trusted LAN; **do not expose ports publicly**.
* Token protects against ordinary cross-site form submission but Basic auth
  over HTTP does **not** encrypt traffic.
* MQTT connection is still synchronous; socket timeout limits but cannot
  fully eliminate network blocking. A future isolated task requires UART tests.
* Physical capability cannot be reliably inferred from UART acknowledgements.
  Therefore unverified features remain unknown until confirmed in service.
* Incoming HVAC frame checksum verification is deliberately deferred pending
  known-good raw frame samples to avoid another UART regression.
* Saved WiFiManager parameter values can include passwords on the open AP.
  Keep AP availability limited and never operate it on a public network.
* Import/export intentionally excludes all credentials and Wi-Fi settings.
* Not all subsidiary debug / firmware pages are fully localized.
* Successful CI builds are necessary, but not sufficient, for production.

## Verification checklist

- First boot with no Wi-Fi: UART handshake starts before any network work.
- Saved Wi-Fi connects without blocking UART; manual BOOT config portal closes
  after timeout; HVAC continues answering queries during portal operation.
- WebUI Czech/English selection persists across browser reload.
- WebUI restart asks for confirmation, responds HTTP 202, reboots.
- HA disabled: no entity; enable and check retained MQTT config and live
  `climate` entity; disable and verify tombstone.
- Capability Manager disabled features cannot be controlled through WebUI,
  HA or service API. Verify FIX 1..5 manually on hardware.
- MQTT unavailable: HVAC stable, eventual recovery; state packet size under
  configured 2048 bytes, retained availability online/offline works.
- Loxone Modbus registers 0-11 unchanged; 100-112 return telemetry.
- Invalid OTA/aborted transfer reactivates Modbus and MQTT.
- Password-redacted backup export and restore, with preserved credentials.
- Confirm all 3 CI builds and reproduce UART RX counters on real AC.
