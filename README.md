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

## 🧮 Mapování Modbus TCP registrů

| Adresa | Název | Typ | Popis |
|:--|:--|:--|:--|
| 0 | Mode | RW | 0 = Auto, 1 = Cool, 2 = Heat, 3 = Dry, 4 = Fan |
| 1 | FanSpeed | RW | 0–5 (Quiet–Auto) |
| 2 | Swing | RW | 0 = OFF, 1 = ON |
| 3 | Power | RW | 0 = OFF, 1 = ON |
| 4 | SetTemp | RW | °C |
| 10 | RoomTemp | R | °C |
| 11 | OutdoorTemp | R | °C |
| 12 | OperatingState | R | 0 = Standby, 1 = Running |
| 13 | ErrorCode | R | aktuální chybový kód |

---

## 🖥️ Webové rozhraní

WebUI je vloženo přímo do firmware a umožňuje:

- přehled aktuálních stavů (režim, teplota, ventilátor)
- okamžité ovládání všech parametrů
- aktualizaci firmware (OTA)
- reset do továrního nastavení
- diagnostiku komunikace

Přístup:  
**http://toshiba-hvac.local**  
nebo přímo IP adresa zařízení.

---

## 🛠️ Instalace

### 🔹 Arduino IDE
1. Vyber desku **ESP8266 (NodeMCU / Wemos D1 Mini)** nebo **ESP32**  
2. Naimportuj knihovny:
   - `WiFiManager`, `PubSubClient`, `ArduinoOTA`, `ArduinoJson`
   - `ModbusIP_ESP8266` nebo `ModbusIP_ESP32`
   - `ToshibaCarrierHvac`  
3. Nahraj soubor `ESP-Toshiba-AC-Modbus.ino`
4. Připoj ESP k TCC-Link vodičům klimatizace

---

## 🧪 Testování

- Po spuštění se ESP přepne do **AP režimu** `Toshiba-HVAC-Setup`
- Připoj se z mobilu nebo PC  
  a nastav Wi-Fi připojení.
- Po úspěšném připojení otevři webové rozhraní a zkontroluj data.
- Ověř komunikaci přes Modbus TCP např. v Loxone nebo QModMaster.

---

## 🔍 Diagnostika (Telnet)

| Příkaz | Funkce |
|:--|:--|
| `help` | výpis dostupných příkazů |
| `state` | výpis stavových hodnot |
| `hvac raw on/off` | zapnutí logování rámců |
| `modbus dump` | výpis aktuálních registrů |
| `factoryreset` | obnovení výchozí konfigurace |

---

## 🔧 OTA aktualizace

- WebUI → **Aktualizace firmware**
- Vyber `.bin` soubor → „Nahrát a aktualizovat“
- Po nahrání proběhne automatický restart.

---

## 🧰 Kompatibilita

Testováno s následujícími jednotkami:
- **Toshiba Shorai / Seiya / Signatur / Daisekai / Super Digital Inverter**
- **Modely RAS-25/35PKVSG, RAS-10–22N3AV2-E, RAS-M10U2MUVG-E**
- Připojení přes servisní konektor **CN40 (TCC-Link)**

---

## 📄 Licence

Tento projekt je šířen pod licencí **MIT License**.  
Závislosti (ToshibaCarrierHvac, ModbusIP, WiFiManager) si zachovávají své vlastní licence.

---

## 🧠 Reference a dokumentace

- 📘 Toshiba **AC E15-3R1 Application Control Manual**  
- 📗 Toshiba **TCB-IFMB641TLE Modbus Interface Manual**  
- 📙 Toshiba **Controls 2015 v1**  
- 📒 Toshiba **Shorai / Signatur Service Manuals**  
- 📔 **ToshibaCarrierHvac Library**  
- 🧾 Datasheety převodníků RS-485 ↔ TTL a logických úrovní

---

## 👨‍🔧 Autor

**Martin Chlup**  
Projekt ESP-Toshiba-AC-Modbus  
Integrace klimatizací Toshiba ↔ Loxone / Home Assistant / Modbus / MQTT  
📧 *mchlup (at) gmail (dot) com*
