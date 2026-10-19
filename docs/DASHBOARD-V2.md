# Dashboard v2 — Moderní webové rozhraní

## Základ
Tato větev vznikla přímo z `main` na commitu `4ba88de` (2026-10-16).
Neobsahuje změny protokolu Toshiba, pinů UART, Modbus registrů ani
nastavení handshake či časování dotazů. Reaguje na existující HTTP API.

## Implementováno
- Responzivní rozhraní pro telefon, tablet i desktop.
- Moderní čistý vzhled, světlý/tmavý režim s preferencí systému.
- Rychlý dashboard: požadovaná teplota, zapnutí/vypnutí, aktuální
  vnitřní/venkovní teplota, režim, ventilátor, základní pohyb lamel.
- Manuální fixní polohy lamel jsou dostupné v servisním testu,
  protože jejich podpora není prokázána.
- Detailní síťové stavy Wi-Fi/MQTT a stáří poslední přijaté UART dávky.
- Konfigurace zařízení, MQTT, WebUI, Modbus, OTA a Telnet.
- Zachované odkazy na servis `/service`, diagnostiku `/debug` a firmware `/fw`.
- Všechen HTML/CSS/JS kód je uložený v `PROGMEM`, bez CDN
  či externích knihoven. Nová hlavní stránka se odesílá přímo z flash,
  místo opakovaného spojování velkého `String` v haldě.
- Obnova stavu každé 3 s, při skryté záložce se pozastavuje.
- HTTP POST se stávající hlavičkou `X-Requested-With`;
  validace chyb serveru a zobrazení chyb uživateli.
- Formulář zachovává původní hesla při odeslání prázdných polí.
- Automatizované buildy pro ESP8266 D1 mini, ESP32 a ESP32-C3 (PlatformIO).

## Změny souborů
- `DashboardAssets.h`: nový HTML dashboard.
- `WebAssets.h`: nový styl a vanilla JS klient.
- `ESP-Toshiba-AC-Modbus.ino`: nový způsob obsluhy `/`;
  doplnění `wifi_connected`, `mqtt_connected` a `uart_last_rx_age_ms`
  do již existujícího `/api/state.json`.
- `platformio.ini`, `.github/workflows/build.yml`: ověřovací buildy.

## Kompatibilita
HTTP endpointy a datové příkazy jsou zachovány:
`GET /api/state.json`, `GET /api/config.json`,
`POST /command`, `POST /mqttconfig`.
Knihovna `ToshibaCarrierHvac.cpp/.h` zůstává nezměněná.
Pokud zařízení nabízí již uložené heslo, WebUI jej úmyslně nečte zpět.

## Akceptační test na zařízení
1. Ověřit UART spojení, čtení teplot a provoz Modbus/MQTT.
2. Ovládat power, 17–30 °C, všech 5 režimů a stupně ventilátoru.
3. Ovládat pevné/svislé/vodorovné/kombinované natáčení lamel.
4. Ověřit dostupnost servisní stránky `/service`.
5. Ověřit změnu parametrů MQTT a zpětné načtení konfigurace.
6. Otestovat v mobilním prohlížeči v šířkách 320–390 px.
7. Po změně webového hesla ověřit nové přihlášení.

Kompilace sama o sobě nepotvrzuje funkčnost na reálné klimatizaci.
