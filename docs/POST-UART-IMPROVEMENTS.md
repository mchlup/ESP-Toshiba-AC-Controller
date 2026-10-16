# Post-UART regression: conservative hardening pass

Branch: `improvement/post-uart-safe-fixes-2026-10`, based on
`improvement/hvac-stability-security-2026` at commit `1beae330`.

The previously nonworking HVAC UART has been confirmed **working** by the
device owner. Therefore this iteration intentionally leaves **unchanged**:

- ToshibaCarrierHvac.h and ToshibaCarrierHvac.cpp
- UART 9600 baud, 8E1, physical RX/TX GPIO mappings
- Original 6-frame handshake + ACK and 200 ms delays
- Original 250 ms packet collection and 11-command query cadence
- Modbus TCP addresses, values and timing of actual HVAC writes

## Changes

1. **Swing command regression:** restore independent return statements and
   state revision update after `SWING_OFF`, `SWING_V`, `SWING_H`,
   `SWING_VH`, and `SWING_FIX_1..5`. The prior inline comment had
   accidentally commented out `bumpRevIfChanged(); return;`, causing fallthrough.
2. **Strict temperature command validation:** reject `TEMP_abc`,
   `TEMP_999` and partially numeric values, instead of `atoi()` and
   clamping an invalid command to a valid setpoint.
3. **HTTP status:** unknown or invalid commands now return HTTP 400 instead
   of always reporting success; valid requests return HTTP 200.
4. **MQTT message safety:** reject empty and oversized messages instead of
   silently truncating; check the subscribed command topic; safely
   terminate a copied payload before command parsing.
5. **Factory-reset safety:** remove factory reset from generic HVAC commands,
   including MQTT and direct WebUI commands. Dedicated authenticated
   `/factory-reset` web form remains available.
6. **UI alignment:** the combined swing value `hv_swing` now maps to the
   correct `SWING_VH` command. Legacy `vh_swing` is also understood.
7. **HTML escaping:** escape user-provided room and device names on the main
   page and debug HTML to prevent markup/script injection.
8. **Modbus runtime:** avoid re-registering active Modbus server/reg mappings
   after a configuration save. Preserve synchronous startup registration.
9. **Consistency:** fix ESP32 red LED PWM range and avoid bypassing the
   Modbus holding-register cache during heartbeat updates.
10. **UART diagnostics:** add counters and last valid RX time to the
    authenticated debug page, state API and Telnet status to help diagnose
    protocol behavior without altering the working UART implementation.

## Verification required

Build with all three PlatformIO targets in GitHub Actions. Then smoke test
UART, WebUI commands, MQTT and Modbus TCP on the real device, especially:
`SWING_OFF`, `SWING_FIX_5`, `SWING_VH` and malformed temperature commands.

A successful build is **not** evidence of successful hardware operation.
The new draft PR intentionally does not merge into the working branch or
`main`.

## Deliberately deferred

Do not add another asynchronous UART parser or change packet framing
without UART captures, tests and a hardware acceptance plan. Changing
WiFiManager to asynchronous operation is likewise deferred because it
would interfere with the startup path that now communicates with the AC.
WebUI authentication / full CSRF token rollout is a separate
compatibility-sensitive security project.
