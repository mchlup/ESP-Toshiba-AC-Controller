# Modbus TCP register map (firmware v2 candidate)

All offsets 0-based holding registers, Modbus TCP port **502**.

| Address | Name | Access | Mapping |
|---|---|---|---|
|0|Power|RW|0 off / 1 on|
|1|Setpoint|RW|17..30 °C|
|2|HVAC mode|RW|1 auto, 2 heat, 3 cool, 4 dry, 5 fan only|
|3|Fan|RW|1 auto, 2..6 levels 1..5|
|4|Swing|RW|8 fixed, 7 vertical, 6 horizontal, 5 both, 11..15 fixed positions|
|5|Room temperature|RO|signed 16bit °C×10; 32767 unknown|
|6|Outdoor temperature|RO|signed 16bit °C×10; 32767 unknown|
|7|Operating mode|RO|0 off, 1 auto, 2 heat, 3 cool, 4 dry, 5 fan only|
|8|Flags|RO|currently 0 (not implemented)|
|9|HVAC error|RO|currently 0xFFFF (not implemented)|
|10|Connectivity flags|RO|bit0 HVAC, bit1 MQTT, bit2 WiFi|
|11|Uptime|RO|lower 16 bits, seconds|
|100|UART accepted RX batches|RO|lower 16 bits|
|101|UART rejected RX batches|RO|lower 16 bits|
|102|UART handshake attempts|RO|lower 16 bits|
|103|Age of last UART RX|RO|seconds, 65535 = unknown|
|104|RSSI|RO|signed int16 dBm (-127 if no WiFi)|
|105|Free heap|RO|KiB, clamped to 65535|
|106|MQTT reconnects|RO|lower 16 bits|
|107|Accepted Modbus writes|RO|lower 16 bits|
|108|Rejected Modbus writes|RO|lower 16 bits|
|109|Home Assistant Discovery|RO|0 disabled / 1 enabled|
|110|UART connected|RO|0/1|
|111|Explicitly supported capabilities|RO|bitmask (bits 0..8 in FeatureConfig.h)|
|112|Explicitly unsupported capabilities|RO|bitmask (bits 0..8 in FeatureConfig.h)|

**Important:** Holding registers 0-4 publish a requested value for at most
10 seconds before reverting to the actual HVAC state if not confirmed.
Registers 100-112 are application-owned diagnostics; a Modbus client should
never write to them. Their 16-bit counters wrap.

Loxone XML uses the same offsets, with legacy 0/1 setpoint scaling.
Existing active installations should first be bench-tested before upgrading.
