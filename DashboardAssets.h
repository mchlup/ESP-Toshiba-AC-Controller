#pragma once
#include <Arduino.h>

// Static dashboard template: served directly from flash without large String
// concatenation and independent of slow/cloud resources.
static const char DASHBOARD_HTML[] PROGMEM = R"DASH_HTML(
<!doctype html>
<html lang="cs">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <meta name="theme-color" content="#f5f7fb">
  <meta name="description" content="Toshiba HVAC – místní ovládání klimatizace">
  <title>Toshiba HVAC · Ovládání</title>
  <link rel="stylesheet" href="/app.css?v=ui2">
  <script defer src="/app.js?v=ui2"></script>
</head>
<body>
<a class="skip-link" href="#main">Přejít k obsahu</a>
<div class="app-shell">
  <header class="topbar">
    <a class="brand" href="/" aria-label="Toshiba HVAC – úvod">
      <span class="brand-symbol" aria-hidden="true"><span class="brand-dot"></span><span class="brand-waves">≋</span></span>
      <span class="brand-name"><strong>TOSHIBA</strong><small>HVAC CONTROLLER</small></span>
    </a>
    <nav class="primary-nav" aria-label="Hlavní nabídka">
      <button class="nav-item is-active" type="button" data-view="dashboard" aria-current="page" data-i18n="dashboard">Ovládání</button>
      <button class="nav-item" type="button" data-view="settings" data-i18n="settings">Nastavení</button>
      <a class="nav-item" href="/service" data-i18n="service">Servis</a>
      <a class="nav-item" href="/debug" data-i18n="diagnostics">Diagnostika</a>
    </nav>
    <div class="top-actions">
      <label class="language-box" for="languageSelect"><span data-i18n="language">Jazyk</span>
        <select id="languageSelect" aria-label="Language / Jazyk">
          <option value="cs">CZ</option><option value="en">EN</option>
        </select>
      </label>
      <span class="connection-pill" id="connectionPill" data-state="loading" role="status" aria-live="polite">
        <span class="status-light" aria-hidden="true"></span><span id="connectionText">Připojování</span>
      </span>
      <button class="icon-button" id="themeToggle" type="button" title="Přepnout vzhled" aria-label="Přepnout světlý a tmavý vzhled">◐</button>
    </div>
  </header>

  <main id="main">
    <section id="dashboardView" data-view-panel="dashboard" aria-labelledby="dashboardTitle">
      <div class="page-heading">
        <div><p class="eyebrow">PŘEHLED ZAŘÍZENÍ <span class="crumb">/</span> KLIMATIZACE</p>
          <h1 id="dashboardTitle">Toshiba HVAC</h1>
          <p class="muted" id="roomName">Načítám údaje z klimatizace…</p>
        </div>
        <span class="last-update" id="lastUpdate" aria-live="off">Stav se aktualizuje automaticky</span>
      </div>

      <div class="dashboard-grid">
        <section class="card thermostat" aria-labelledby="targetTitle">
          <div class="card-head"><span class="section-label">TEPLOTA</span>
            <span class="chip chip-soft" id="modeCaption">—</span>
          </div>
          <div class="setpoint-top"><p id="targetTitle" data-i18n="setpoint">Požadovaná teplota</p>
            <div class="thermostat-controls">
              <button type="button" class="temp-adjust" id="tempDown" aria-label="Snížit teplotu o jeden stupeň" disabled>−</button>
              <output id="targetTemp" for="tempRange" class="target-value">--<span>°C</span></output>
              <button type="button" class="temp-adjust" id="tempUp" aria-label="Zvýšit teplotu o jeden stupeň" disabled>+</button>
            </div>
          </div>
          <div class="slider-wrap">
            <input id="tempRange" class="temp-slider" type="range" min="17" max="30" step="1" value="24" aria-label="Požadovaná teplota" disabled>
            <div class="slider-labels"><span>17 °C</span><span>30 °C</span></div>
          </div>
          <div class="thermostat-bottom">
            <div class="thermostat-note"><span class="mini-dot" aria-hidden="true"></span><span id="powerInfo">Zjišťuji stav jednotky</span></div>
            <button type="button" class="power-button" id="powerButton" aria-pressed="false">
              <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" aria-hidden="true"><path d="M12 3v9"/><path d="M6.5 5.8a8 8 0 1 0 11 0"/></svg>
              <span id="powerButtonLabel">Zapnout</span>
            </button>
          </div>
        </section>

        <section class="card overview" aria-labelledby="overviewTitle">
          <div class="card-head"><h2 id="overviewTitle" data-i18n="readings">Aktuální hodnoty</h2><span class="subtle">živá data</span></div>
          <div class="temperature-readings">
            <div class="reading"><div class="reading-icon" aria-hidden="true">↘</div><div><span class="reading-title" data-i18n="room_temp">V místnosti</span><strong id="indoorTemp">—</strong></div></div>
            <div class="reading"><div class="reading-icon outdoor" aria-hidden="true">↗</div><div><span class="reading-title" data-i18n="outdoor_temp">Venku</span><strong id="outdoorTemp">—</strong></div></div>
          </div>
          <div class="divider"></div>
          <div class="status-rows">
            <div class="status-row"><span>Provozní stav</span><strong id="operatingStatus">—</strong></div>
            <div class="status-row"><span>Klimatizace</span><span class="inline-state" id="hvacState">Čekám na spojení</span></div>
            <div class="status-row"><span>Wi-Fi / MQTT</span><span id="networkState" class="soft-text">Zjišťuji…</span></div>
            <div class="status-row"><span>Poslední UART odpověď</span><span id="uartAge" class="soft-text">—</span></div>
          </div>
        </section>

        <section class="card mode-card" aria-labelledby="modeTitle">
          <div class="card-head"><div><span class="section-label">REŽIM</span><h2 id="modeTitle">Způsob provozu</h2></div></div>
          <div class="choice-grid choice-grid-five" role="group" aria-label="Režim klimatizace">
            <button class="choice" type="button" data-cmd="MODE_AUTO" data-match="auto" aria-pressed="false"><span class="choice-symbol">A</span><span>Auto</span></button>
            <button class="choice" type="button" data-cmd="MODE_COOL" data-match="cool" aria-pressed="false"><span class="choice-symbol">❄</span><span>Chlazení</span></button>
            <button class="choice" type="button" data-cmd="MODE_HEAT" data-match="heat" aria-pressed="false"><span class="choice-symbol">☼</span><span>Topení</span></button>
            <button class="choice" type="button" data-cmd="MODE_DRY" data-match="dry" aria-pressed="false"><span class="choice-symbol">◌</span><span>Odvlhčení</span></button>
            <button class="choice" type="button" data-cmd="MODE_FAN" data-match="fan_only" aria-pressed="false"><span class="choice-symbol">✳</span><span>Ventilátor</span></button>
          </div>
        </section>

        <section class="card fan-card" aria-labelledby="fanTitle">
          <div class="card-head"><div><span class="section-label">VENTILÁTOR</span><h2 id="fanTitle" data-i18n="fan_speed">Rychlost proudění</h2></div>
            <span class="chip chip-soft" id="fanCaption">—</span>
          </div>
          <div class="choice-grid fan-grid" role="group" aria-label="Rychlost ventilátoru">
            <button type="button" class="choice compact" data-cmd="FAN_AUTO" data-match="auto" aria-pressed="false">Auto</button>
            <button type="button" class="choice compact" data-cmd="FAN_1" data-match="lvl_1" aria-pressed="false">1</button>
            <button type="button" class="choice compact" data-cmd="FAN_2" data-match="lvl_2" aria-pressed="false">2</button>
            <button type="button" class="choice compact" data-cmd="FAN_3" data-match="lvl_3" aria-pressed="false">3</button>
            <button type="button" class="choice compact" data-cmd="FAN_4" data-match="lvl_4" aria-pressed="false">4</button>
            <button type="button" class="choice compact" data-cmd="FAN_5" data-match="lvl_5" aria-pressed="false">5</button>
          </div>
        </section>

        <section class="card swing-card" aria-labelledby="swingTitle">
          <div class="card-head"><div><span class="section-label">LAMELY</span><h2 id="swingTitle" data-i18n="airflow">Směr proudění</h2></div><span class="chip chip-soft" id="swingCaption">—</span></div>
          <div class="choice-grid swing-grid" role="group" aria-label="Pohyb lamel">
            <button type="button" class="choice compact" data-cmd="SWING_OFF" data-match="fix" aria-pressed="false">Pevné</button>
            <button type="button" class="choice compact" data-cmd="SWING_V" data-match="v_swing" aria-pressed="false">Svisle</button>
            <button type="button" class="choice compact" data-cmd="SWING_H" data-match="h_swing" aria-pressed="false">Vodorovně</button>
            <button type="button" class="choice compact" data-cmd="SWING_VH" data-match="hv_swing" aria-pressed="false">Obě osy</button>
          </div>
          <p class="hint">Podpora jednotlivých poloh lamel se liší podle modelu. <a href="/service">Testovat v servisu →</a></p>
        </section>

        <section class="card quick-card" aria-labelledby="quickTitle">
          <div class="card-head"><div><span class="section-label">NÁSTROJE</span><h2 id="quickTitle" data-i18n="more_options">Další možnosti</h2></div></div>
          <div class="quick-links">
            <a href="/service"><span class="quick-icon">↗</span><span><strong data-i18n="service_diagnostics">Servisní diagnostika</strong><small>Detailní stav a testy klimatizace</small></span><span class="quick-arrow">→</span></a>
            <a href="/fw"><span class="quick-icon">↑</span><span><strong data-i18n="firmware">Aktualizace firmware</strong><small>Bezpečné nahrání nové verze</small></span><span class="quick-arrow">→</span></a>
          </div>
        </section>
      </div>
    </section>

    <section id="settingsView" data-view-panel="settings" aria-labelledby="settingsTitle" hidden>
      <div class="page-heading"><div><p class="eyebrow">SPRÁVA ZAŘÍZENÍ <span class="crumb">/</span> KONFIGURACE</p><h1 id="settingsTitle">Nastavení</h1>
        <p class="muted">Změny se ukládají do ESP. Prázdná hesla znamenají „ponechat stávající“.</p></div></div>
      <form id="settingsForm" autocomplete="off">
        <div class="settings-grid">
          <section class="card settings-card">
            <div class="card-head"><div><span class="section-label">IDENTITA</span><h2>Zařízení</h2></div></div>
            <div class="field-grid">
              <label class="field">Název zařízení<input name="name" maxlength="31" placeholder="Klimatizace obývák"></label>
              <label class="field">Místnost<input name="room" maxlength="31" placeholder="Obývák"></label>
              <label class="field">Vlastní ID<select name="dev_id_en"><option value="0">Vypnuto (MAC)</option><option value="1">Zapnuto</option></select></label>
              <label class="field">Identifikátor<input name="dev_id" maxlength="12" placeholder="AC-OBYVAK"></label>
            </div>
          </section>
          <section class="card settings-card">
            <div class="card-head"><div><span class="section-label">INTEGRACE</span><h2>MQTT broker</h2></div></div>
            <div class="field-grid">
              <label class="field">MQTT<select name="mqtt_en"><option value="0">Vypnuto</option><option value="1">Zapnuto</option></select></label>
              <label class="field">Port<input name="port" type="number" min="1" max="65535" placeholder="1883"></label>
              <label class="field wide">Adresa brokeru<input name="server" maxlength="63" placeholder="192.168.1.10"></label>
              <label class="field">Uživatelské jméno<input name="user" maxlength="31"></label>
              <label class="field">Heslo<input name="pass" type="password" maxlength="63" placeholder="Ponechat beze změny" autocomplete="new-password"></label>
              <label class="field wide">Topic příkazů<input name="cmdtopic" maxlength="95" placeholder="toshiba/{id}/control"></label>
              <label class="field wide">Topic stavu<input name="statetopic" maxlength="95" placeholder="toshiba/{id}/state"></label>
              <label class="field wide">Topic heartbeat<input name="hbtopic" maxlength="95" placeholder="toshiba/{id}/heartbeat"></label>
            </div>
          </section>
          <section class="card settings-card">
            <div class="card-head"><div><span class="section-label">PŘÍSTUP</span><h2>Zabezpečení WebUI</h2></div></div>
            <div class="field-grid">
              <label class="field">Web uživatel<input name="webuser" maxlength="31" autocomplete="username"></label>
              <label class="field">Nové webové heslo<input name="webpass" type="password" maxlength="63" placeholder="Ponechat beze změny" autocomplete="new-password"></label>
            </div>
            <p class="hint">Rozhraní používá HTTP Basic ověřování. Nepřistupujte k němu přímo z veřejného internetu.</p>
          </section>
          <section class="card settings-card">
            <div class="card-head"><div><span class="section-label">SLUŽBY</span><h2>Pokročilé funkce</h2></div></div>
            <div class="field-grid">
              <label class="field">Modbus TCP<select name="mb_en"><option value="0">Vypnuto</option><option value="1">Zapnuto</option></select></label>
              <label class="field">Port Modbus<input id="modbusPort" type="text" value="502" readonly></label>
              <label class="field">Arduino OTA<select name="ota_en"><option value="0">Vypnuto</option><option value="1">Zapnuto</option></select></label>
              <label class="field">Heslo OTA<input name="ota_pass" type="password" maxlength="63" placeholder="Ponechat beze změny"></label>
              <label class="field">Telnet<select name="telnet_en"><option value="0">Vypnuto</option><option value="1">Zapnuto</option></select></label>
              <label class="field">Heslo Telnet<input name="telnet_pass" type="password" maxlength="63" placeholder="Ponechat beze změny"></label>
            </div>
          </section>
        </div>
        <div class="settings-actions">
          <span id="settingsMessage" class="muted" role="status" aria-live="polite">Změny budou použity po uložení.</span>
          <button type="button" class="btn btn-ghost" id="reloadSettings">Znovu načíst</button>
          <button type="submit" class="btn btn-primary" id="saveSettings" data-i18n="save">Uložit nastavení →</button>
        </div>
      </form>
      <div class="settings-grid feature-grid">
        <section class="card settings-card">
          <div class="card-head"><div><span class="section-label">MQTT</span><h2 data-i18n="ha_discovery">Home Assistant MQTT Discovery</h2></div></div>
          <label class="field"><span data-i18n="ha_discovery_help">Automatické přidání klimatizace do Home Assistantu přes MQTT</span>
            <select id="haDiscovery"><option value="0">Off</option><option value="1">On</option></select>
          </label>
        </section>
        <section class="card settings-card">
          <div class="card-head"><div><span class="section-label">HVAC</span><h2 data-i18n="capabilities">Podporované funkce</h2></div></div>
          <p class="hint" data-i18n="capability_note">Auto = dosud neověřeno. Nepodporované funkce se nezobrazují v Home Assistantu.</p>
          <div id="capabilityFields" class="field-grid"></div>
        </section>
      </div>
      <div class="settings-actions">
        <span id="featureMessage" class="muted" role="status"></span>
        <button type="button" id="saveFeatures" class="btn btn-primary" data-i18n="apply_capabilities">Uložit schopnosti</button>
        <button type="button" id="restartDevice" class="btn btn-danger" data-i18n="restart">Restartovat zařízení</button>
      </div>
    </section>
  </main>
  <footer class="app-footer"><span>Toshiba HVAC · Lokální ovládání</span><span id="deviceIdentifier">—</span></footer>
</div>
<div id="toast" class="toast" role="status" aria-live="polite" hidden></div>
</body>
</html>
)DASH_HTML";
