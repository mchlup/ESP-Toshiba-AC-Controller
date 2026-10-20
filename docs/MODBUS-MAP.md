# Modbus TCP map – firmware 2026-10

Addresses are **zero-based Modbus HOLDING registers**. Existing 0..11 addresses
are preserved, including their read/write behavior. Modbus TCP port is 502.

| Address | Value | Access | Format |
|---:|---|---|---|
| 0 | Power | RW | 0 off, 1 on |
| 1 | Setpoint | RW | 17..30 °C |
| 2 | Mode | RW | 1 auto, 2 heat, 3 cool, 4 dry, 5 fan-only |
| 3 | Fan | RW | 1 auto, 2..6 fan level 1..5 |
| 4 | Swing | RW | 5 both, 6 horizontal, 7 vertical, 8 fixed, 11..15 fixed pos 1..5 |
| 5 | Room temperature | RO (firmware) | signed int16 ×10; 32767 unknown |
| 6 | Outdoor temperature | RO (firmware) | signed int16 ×10; 32767 unknown |
| 7 | Operating mode | RO (firmware) | 0 off; 1 auto, 2 heat, 3 cool, 4 dry, 5 fan only |
| 8 | Flags | RO (firmware) | placeholder 0 |
| 9 | Error | RO (firmware) | placeholder 0xFFFF |
| 10 | Status | RO (firmware) | bit0 UART connected, bit1 MQTT, bit2 WiFi |
| 11 | Uptime | RO (firmware) | seconds, low 16 bits |

New diagnostics – firmware refresh every second:

| Address | Value | Format |
|---:|---|---|
| 100 | Successful UART RX batches low word | unsigned 16-bit |
| 101 | Rejected UART RX batches low word | unsigned 16-bit |
| 102 | Handshake attempts low word | unsigned 16-bit |
| 103 | Last UART RX age | seconds; 65535 = unknown, saturated at 65535 |
| 104 | Wi-Fi RSSI | signed int16, dBm; 32767 unknown |
| 105 | Free heap memory | kB, saturated |
| 106 | Successful MQTT connections | unsigned 16-bit, wraps |
| 107 | Reset reason code | platform-dependent enum |
| 108 | Last control request state | 0 none, 1 pending, 2 matched HVAC feedback, 3 timed out |
| 109 | Worst measured loop duration | milliseconds, saturated |
| 110 | UART RX successful high word | unsigned 16-bit |
| 111 | UART RX rejected high word | unsigned 16-bit |
| 112 | Handshake attempts high word | unsigned 16-bit |

A 32-bit counter is `(high << 16) | low`. Registers 100..112 are
**firmware read-only**: Modbus Holding registers are technically writable at
protocol level, but the firmware refreshes these values periodically.

Existing HREG0..4 writes still have pending/confirmed behavior and should be
tested against actual Toshiba feedback before switching Loxone logic.

## Known limitations
- The RX valid/invalid counters distinguish recognized frames from
  unrecognized frames, but do **not** validate the incoming checksum until
  confirmed against known-good Toshiba UART traces.
- Temperature values are derived from the Toshiba driver's integer readings,
  not a calibrated independent sensor.
- Reset reason numeric values differ between ESP8266 and ESP32.
- The code does not implement live, user-selectable Modbus TCP port despite an
  informational field in the configuration.
