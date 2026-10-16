# Actual firmware Modbus TCP map

All addresses are **0-based holding register offsets**. Modbus TCP listens on 502.
Read/write means an external Modbus client may write; note that the firmware
publishes last requested value for up to 10 seconds before restoring actual
state if there is no confirmation.

| Register | Function | Access | Encoding |
|---|---|---|---|
| 0 | Power | RW | 0 off, 1 on |
| 1 | Setpoint | RW | integer 17..30 Celsius |
| 2 | Mode | RW | 1 auto, 2 heat, 3 cool, 4 dry, 5 fan-only |
| 3 | Fan | RW | 1 auto, 2..6 level 1..5 |
| 4 | Swing | RW | 8 fixed, 7 vertical, 6 horizontal, 5 both, 11..15 fixed position 1..5 |
| 5 | Room temperature | RO | signed int16 degrees C times 10; 32767 unknown |
| 6 | Outdoor temperature | RO | signed int16 degrees C times 10; 32767 unknown |
| 7 | Operating mode | RO | 0 off, 1 auto, 2 heat, 3 cool, 4 dry, 5 fan-only |
| 8 | Flags | RO | currently 0 (not implemented) |
| 9 | Error | RO | currently 0xFFFF unknown (not implemented) |
| 10 | Status | RO | bit0 HVAC connected, bit1 MQTT, bit2 Wi-Fi |
| 11 | Uptime | RO | seconds, low 16 bits, rollover at 65536 |

These values match the firmware's current maps; this map differs from
some historical README examples. Existing Loxone installations should be
tested before replacing the controller firmware.
