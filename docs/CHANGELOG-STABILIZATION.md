# HVAC stability and security branch

This is an **experimental review branch**. Do not flash production devices without
a backup and bench testing of UART traffic and WebUI/Modbus operation.

## Changes implemented

- Corrected dangling stack pointer from SWING_FIX_x and mutable FAN_x alias buffer.
- Canonical HVAC swing literal is `hv_swing`; legacy `vh_swing` read alias remains accepted.
- Fixed Modbus mode 0 shadow register and FAN_ONLY code-5 handling.
- Separate pending control values (10-second confirmation window) from
  confirmed HVAC state; Modbus synchronizer updates internal register cache.
- Reject invalid Modbus control values rather than silently writing AUTO/FIX.
- Distinguish OFF (0) from FAN_ONLY (5) in operating state. This modifies
  HREG7 semantics; clients relying on the old mapping must be updated.
- Initialise temperatures as unknown (`INT8_MIN`) to avoid false 0 Celsius.
- Fix memory safety in HVAC receive header scanning and bounded feedback/reply copying.
- Replace blocking handshake, serial receive and initial polling with scheduled steps.
- Add UART valid/invalid packet counters in driver.
- Enable ESP32-C3 SuperMini GPIO8 active-low status LED.
- Correct stale heartbeat proxy: commands no longer imply received HVAC data.
- Move TLS feature flag before include; refuse TLS without certificate validation.
- Disable default Telnet and ArduinoOTA; web firmware upload remains authenticated.
- Web configuration and commands use POST instead of query parameters; the
  handler checks the custom X-Requested-With header and no longer publishes
  persisted passwords via /api/config.json.

## Known limitations / validation still required

- No real Toshiba unit UART captures or hardware acceptance tests were available.
- This driver has not yet implemented receive checksum verification because
  checksum validation must be established against known-good frame samples.
- UART connection does not prove individual command acknowledgement.
- `X-Requested-With` is a CSRF mitigation only in browsers; use a real
  session-bound CSRF token and stronger authentication for exposed installations.
- Factory-reset controls via MQTT and Telnet should only be enabled on a trusted network.
- Default WebUI password is still admin/admin for backward compatibility;
  change immediately after installation. Provisioning a unique first-run
  credential remains necessary.
- Modbus register writes still require testing for burst writes and concurrency.
- Firmware is not yet validated across all Arduino core versions.
- WiFiManager initial STA/AP boot sequence remains blocking.
- An optional raw analyzer hook requires additional behavioral verification.

## Build and test

Install PlatformIO and run:
```sh
pio run -e esp8266_d1mini
pio run -e esp32dev
pio run -e esp32c3
```
After a successful build, test HVAC serial handshake and commands with live
unit before production use. Verify each control path (WebUI, MQTT, Modbus).
