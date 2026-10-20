> **Feature branch (2026-10-20):** Full change log and hardware test checklist:
> [docs/FEATURES-2026-10-20.md](docs/FEATURES-2026-10-20.md).
> Extended Modbus TCP registers 100..112:
> [docs/MODBUS-MAP.md](docs/MODBUS-MAP.md).
> WiFiManager captive AP intentionally stays passwordless and WebUI
> credentials intentionally default to admin/admin.
> Neither interface should be exposed on an untrusted network.
>
> **Experimental stabilization branch:** Firmware changes in this branch require
> successful CI builds and testing on the actual Toshiba unit before production.
> Read [the stabilization changelog](docs/CHANGELOG-STABILIZATION.md)
> and the [verified source-code register map](docs/MODBUS-MAP.md).
> Historic Modbus examples below may differ from current firmware mappings;
> use docs/MODBUS-MAP.md as the reference for registers 0-11.
>
> **Security:** Keep the device on a trusted local network. Configure a
> non-default WebUI password immediately; do not expose HTTP, Modbus, Telnet,
> MQTT command topics or OTA directly to the public Internet.
>
# 🌡️ ESP-Toshiba-AC-Modbus

**Ovládání a monitorování klimatizací Toshiba přes ESP8266/ESP32 s rozhraním Modbus TCP**

---

## 📘 Popis projektu

Tento projekt umožňuje obousměrnou komunikaci s klimatizační jednotkou **Toshiba RAS / RAV / DI / SDI / SHRM / SMMS** přes vnitřní servisní sběrnici **TCC-Link (UART 8E1, 9600 baud)** pomocí mikrokontroléru **ESP8266** nebo **ESP32**.

ESP zařízení čte a odesílá příkazy v binárním formátu podle **Toshiba Carrier HVAC protokolu**, zpracovává stavové informace a poskytuje rozhraní:

- 🌐 **WebUI** (ovládání z prohlížeče)
- 🧠 **Modbus TCP** (pro integraci s Loxone, PLC, BMS systémy)
- ☁️ **MQTT** (volitelné, pro integraci s Home Assistant / Node-RED)
- 🔧 **OTA aktualizace** a **Telnet debug rozhraní**

---

## 🧩 Funkce

| Funkce | Popis |
|:--|:--|
| 🌬️ **Ovládání klimatizace** | ON/OFF, režim, teplota, ventilátor, swing |
| 🧾 **Čtení stavů** | aktuální teplota, režim, provozní stav, chybové kódy |
| ⚙️ **Mapování Modbus TCP** | registry pro čtení/zápis hodnot (viz níže) |
| 🕹️ **Webové UI** | responzivní stránka pro ovládání přes Wi-Fi |
| 📶 **WiFiManager** | konfigurační portál pro SSID / heslo |
| 🔌 **OTA update** | aktualizace firmware přes web |
| 🪶 **Telnet** | přímé ladění, logování rámců, příkazy |
| 🧱 **EEPROM/LittleFS** | trvalé uložení konfigurace |
| 🧰 **Modularita** | snadné rozšíření o MQTT, časové plány, logování |

---

## ⚡ Hardwarové zapojení

| Signál | ESP8266 pin | Popis |
|:--|:--|:--|
| RX | GPIO12 (D6) | příjem dat z klimatizace |
| TX | GPIO14 (D5) | odesílání dat do klimatizace |
| GND | GND | společná zem |
| LED | GPIO2 | indikace stavu (volitelné) |
| BTN | GPIO0 | konfigurační tlačítko (WiFi setup) |

> 💡 Doporučeno použít **TTL ↔ RS-485 převodník** nebo **optoizolaci** (např. PC817) a **logický převodník 3.3 ↔ 5 V**, pokud se připojujete k vodičům AB (TCC-Link).

---

## 📡 Komunikační parametry

| Parametr | Hodnota |
|:--|:--|
| UART rychlost | **9600 baud** |
| Data bits | 8 |
| Parita | Even |
| Stop bit | 1 |
| Polarita | TTL (3.3 V) nebo RS-485 |
| Protokol | Toshiba Carrier HVAC |

---

## 🔠 Příkazy Toshiba Carrier HVAC

| CMD | Název | Popis | Hodnota |
|:--|:--|:--|:--|
| 128 | STATE | Zapnutí / vypnutí | `48 = ON`, `49 = OFF` |
| 135 | POWER | Výkon kompresoru | `50/75/100` % |
| 150 | COMFORT SLEEP | Režim spánku | `hodiny, 0x0F` |
| 160 | FAN MODE | Rychlost ventilátoru | `49–54`, `65=Auto` |
| 163 | SWING | Natáčení lamel | `49=OFF`, `65=ON` |
| 176 | MODE | Režim | `65=Auto`, `66=Cool`, `67=Heat`, `68=Dry`, `69=Fan` |
| 179 | SETPOINT | Nastavená teplota | °C |
| 187 | ROOM TEMP | Teplota v místnosti | °C |
| 190 | OUTDOOR TEMP | Teplota venku | °C |

---

## 🧮 Mapování Modbus TCP registrů (historická dokumentace)

**Pozor:** Následující historická tabulka nemusí odpovídat aktuálnímu firmware. Pro přesné adresy a mapování použijte [docs/MODBUS-MAP.md](docs/MODBUS-MAP.md).

| Adresa | Název | Typ | Rozsah | Popis a chování |
|:--|:--|:--|:--|:--|
| **0** | **Mode** | RW | 0–4 | Nastavení režimu. Odpovídá příkazu `CMD 176` (MODE). <br>0=Auto, 1=Cool, 2=Heat, 3=Dry, 4=Fan |
| **1** | **SetTemp** | RW | 16–30 °C | Nastavená cílová teplota (`CMD 179`). Hodnota se zaokrouhluje na celé °C. |
| **2** | **FanSpeed** | RW | 0–5 | Rychlost ventilátoru (`CMD 160`). <br>0=Quiet, 1–4=Manual, 5=Auto |
| **3** | **Swing** | RW | 0/1 | Natáčení lamel (`CMD 163`). <br>0=OFF, 1=ON |
| **4** | **Power** | RW | 0/1 | Zapnutí/vypnutí klimatizace (`CMD 128`). |
| **5** | **RoomTemp** | R | 0–50 | Aktuální teplota místnosti (`CMD 187`). |
| **6** | **OutdoorTemp** | R | −40–60 | Teplota venkovního výměníku (`CMD 190`). |
| **7** | **OperatingState** | R | 0/1 | 0=Standby, 1=Kompresor běží. |
| **8** | **ErrorCode** | R | 0–255 | Chybový kód dle servisního manuálu Toshiba. |
| **9** | **Flags** | R | bitmask | Stavová bitová maska (např. komunikace HVAC, MQTT, Wi-Fi). |
| **10** | **Uptime (s)** | R | 0–65535 | Počet sekund od restartu. |

### 🔹 Poznámky k Modbus TCP
- Komunikuje na portu **502** (standardní Modbus TCP).  
- Čtení doporučeno v intervalu **2–5 s**.  
- Zápis se provádí pouze při změně hodnoty.  
- Při neplatném zápisu (mimo rozsah) se příkaz ignoruje a vrací chyba.  
- Po ztrátě spojení s klimatizací se registry zmrazí, `ErrorCode=255`.

---

## 🧱 Integrace s Loxone

Projekt je kompatibilní se systémem **Loxone Config** a má připravenou oficiální šablonu XML:

📄 **Soubor:** `MB_Toshiba RAS-B10J2KVG-E MODBUS TCP.xml`

### 🔹 Import do Loxone Config
1. Otevři **Loxone Config** → záložka **Periferie → Modbus**  
2. Klikni pravým tlačítkem na Modbus zařízení → **Importovat šablonu**
3. Vyber přiložený soubor XML
4. Po importu se automaticky vytvoří vstupy/výstupy:
   - Power, Mode, Fan, Swing, Setpoint
   - RoomTemp, OutdoorTemp, ErrorCode, OperatingState, Flags, Uptime
5. Uprav IP adresu a port (502)

### 🔹 Doporučené adresy (z XML)
| Loxone blok | Typ | Adresa | Komentář |
|:--|:--|:--|:--|
| Power | 3/6 | 0 | 0=Off, 1=On |
| Setpoint | 3/6 | 1 | 17–30 °C |
| Mode | 3/6 | 2 | 0=Auto, 1=Cool, 2=Heat, 3=Dry, 4=Fan |
| Fan | 3/6 | 3 | 0–5 |
| Swing | 3/6 | 4 | 0=Off, 1=On |
| IndoorTemp | 3 | 5 | °C ×10 |
| OutdoorTemp | 3 | 6 | °C ×10 |
| OperatingState | 3 | 7 | 0=Standby, 1=Run |
| Flags | 3 | 8 | Bitmask |
| ErrorCode | 3 | 9 | Chybový kód |
| Uptime | 3 | 11 | Sekundy |

### 🔹 Tipy pro Loxone
- Používej **Modbus Holding Registry (function 3/6)**.  
- Hodnoty teploty (`IndoorTemp`, `OutdoorTemp`) jsou ve formátu **int16 ×10** – vyděl 10 pro zobrazení ve °C.  
- Pro komfortní integraci doporučeno doplnit vizualizační prvky (ikonky, barvy režimů).

---

## 🛠️ Instalace

1. Vyber desku **ESP8266 (NodeMCU / Wemos D1 Mini)** nebo **ESP32**
2. Přidej knihovny: `WiFiManager`, `PubSubClient`, `ArduinoOTA`, `ModbusIP`, `ToshibaCarrierHvac`
3. Nahraj `ESP-Toshiba-AC-Modbus.ino`
4. Připoj k TCC-Link (CN40) vodičům klimatizace

---

## 🧠 Reference a dokumentace

- 📘 Toshiba **AC E15-3R1 Application Control Manual**
- 📗 Toshiba **TCB-IFMB641TLE Modbus Interface Manual**
- 📙 Toshiba **Controls 2015 v1**
- 📒 Toshiba **Shorai / Signatur Service Manuals**
- 📔 **ToshibaCarrierHvac Library**
- 📄 **Loxone Modbus XML Template** (součást projektu)

---

## 👨‍🔧 Autor

**Martin Chlup**  
Integrace klimatizací Toshiba ↔ Loxone / Home Assistant / Modbus / MQTT  
📧 *mchlup (at) gmail (dot) com*
