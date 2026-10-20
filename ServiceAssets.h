#pragma once
#include <Arduino.h>

// Dedicated service UI. Served from flash, not copied into the .ino or SRAM.
static const char SERVICE_HTML[] PROGMEM = R"SERVICEHTML(
<!doctype html><html lang="cs"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Toshiba HVAC | Servis</title>
<link rel="stylesheet" href="/app.css">
<style>
.service-wrap{max-width:1160px;margin:auto;padding:16px}
.service-hero{display:flex;gap:12px;align-items:center;justify-content:space-between;flex-wrap:wrap;margin-bottom:14px}
.service-card{background:var(--card);border:1px solid var(--border);border-radius:14px;padding:16px;margin-bottom:14px}
.service-card h2{font-size:18px;margin:0 0 12px}
.service-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:14px}
.service-tbl{width:100%;border-collapse:collapse;font-size:13px;word-break:break-word}
.service-tbl th,.service-tbl td{padding:6px 7px;border-bottom:1px solid var(--border);text-align:left;vertical-align:top}
.service-tbl th{color:var(--muted);font-weight:600}
.service-tbl td:nth-child(2){font-weight:600}
.service-form{display:flex;gap:10px;flex-wrap:wrap;align-items:end;margin-bottom:10px}
.service-form>label{display:flex;flex-direction:column;gap:5px;min-width:140px;flex:1}
.service-form select,.service-form input{padding:9px;background:var(--bg);color:var(--fg);border:1px solid var(--border);border-radius:10px;min-height:39px}
.service-note{color:var(--muted);font-size:13px;line-height:1.55;margin:8px 0}
.service-warn{border-left:4px solid var(--warn);padding:10px;background:var(--bg);border-radius:4px;font-size:13px}
.service-status{min-height:24px;white-space:pre-wrap;overflow-wrap:anywhere}
.service-small{font-size:12px;color:var(--muted)}
.service-log{padding:12px;background:var(--bg);border-radius:8px;white-space:pre-wrap;word-break:break-all;max-height:400px;overflow:auto;font-size:12px}
.service-tag{display:inline-block;padding:4px 8px;border-radius:12px;background:var(--bg);border:1px solid var(--border);font-size:12px}
@media(max-width:650px){.service-grid{grid-template-columns:1fr}.service-form>label{flex-basis:100%}}
</style></head><body><div class="service-wrap">
<div class="service-hero"><div><h1 style="margin:4px 0">Servisn&#237; konzole Toshiba</h1>
<div class="service-small">Diagnostika, stav UART, funkce a ov&#283;&#345;en&#237; nat&#225;&#269;en&#237; lamel</div></div>
<div class="btn-group"><a class="btn" href="/">Dashboard</a><a class="btn" href="/debug">Debug</a></div></div>
<div class="service-warn">Zobrazen&#233; hodnoty jsou stavy hl&#225;&#353;en&#233; protokolem, nikoliv p&#345;&#237;m&#225; fyzick&#225; kontrola.
Dostupnost funkc&#237; PURE, Power Select, Operation a pevn&#253;ch poloh lamel z&#225;vis&#237; na modelu klimatizace.
Povely mohou zm&#283;nit provoz jednotky. Pou&#382;&#237;vejte pouze v d&#367;v&#283;ryhodn&#233; lok&#225;ln&#237; s&#237;ti.</div>
<div class="service-card" style="margin-top:14px"><div class="service-form">
<strong id="servConnection" class="service-tag">UART: --</strong><span id="servBuild" class="service-small"></span>
<button class="btn" id="servRefresh" type="button">Na&#269;&#237;st stav</button>
<button class="btn" id="servQuery" type="button">Vy&#382;&#225;dat pln&#253; stav</button>
</div><div class="service-status" id="servMsg" aria-live="polite"></div>
<div class="service-small">Aktualizace stav&#367; automaticky ka&#382;d&#233; 3 s. &#381;&#225;dost o pln&#253; stav pou&#382;&#237;v&#225; p&#367;vodn&#237; &#269;asov&#225;n&#237; knihovny (nen&#237; okam&#382;it&#225;).</div></div>
<div class="service-grid">
<section class="service-card"><h2>Telemetrie a komunikace</h2><table class="service-tbl"><tbody id="servTelemetry"><tr><td>Na&#269;&#237;t&#225;n&#237; ...</td></tr></tbody></table></section>
<section class="service-card"><h2>Konfigurace: potvrzeno vs. po&#382;adov&#225;no</h2>
<table class="service-tbl"><thead><tr><th>Vlastnost</th><th>Hl&#225;&#353;eno</th><th>Po&#382;adov&#225;no</th><th>Posl&#225;no</th></tr></thead><tbody id="servSettings"><tr><td>Na&#269;&#237;t&#225;n&#237; ...</td></tr></tbody></table>
<div class="service-note">Po&#382;adov&#225;no a posl&#225;no nejsou potvrzen&#233; stavy. Komunikace nemus&#237; v&#353;echny vlastnosti podporovat.</div></section>
</div>
<section class="service-card"><h2>Ovl&#225;d&#225;n&#237; a servisn&#237; nastaven&#237;</h2>
<div class="service-form"><label>Vlastnost<select id="servField">
<option value="power">Zapnout / vypnout</option><option value="setpoint">Po&#382;adovan&#225; teplota</option>
<option value="mode">Re&#382;im</option><option value="fan">Ventil&#225;tor</option><option value="swing">Lamely</option>
<option value="pure">PURE / ionizace (dle modelu)</option><option value="powerSelect">Power Select (dle modelu)</option>
<option value="operation">Operation (dle modelu)</option><option value="wifiLed">Wi-Fi LED (dle modelu)</option>
</select></label><label>Hodnota<select id="servValue"></select></label>
<button class="btn primary" id="servApply" type="button">Odeslat povel</button></div>
<label class="service-note" style="display:flex;align-items:center;gap:8px"><input type="checkbox" id="servArmed"> Povolit servisn&#237; ovl&#225;d&#225;n&#237; (odeslan&#233; povely mohou zm&#283;nit provoz)</label>
<div class="service-note">Povely jsou validov&#225;ny a odes&#237;l&#225;ny p&#367;vodn&#237; knihovnou. Str&#225;nka neumo&#382;&#328;uje odes&#237;lat libovoln&#233; syrov&#233; UART pakety.</div></section>
<section class="service-card"><h2>Test manu&#225;ln&#237;ch poloh lamel</h2>
<p class="service-note">Test odes&#237;l&#225; standardn&#237; p&#345;&#237;kaz pro pevn&#253; &#250;hel, sleduje hodnotu hl&#225;&#353;enou klimatizac&#237; a p&#345;&#237;jem UART.
Pokud se poloha po testu nevr&#225;t&#237; jako potvrzen&#225;, m&#367;&#382;e j&#237;t o nepodporovanou funkci, zpo&#382;d&#283;n&#237; nebo nep&#345;ijet&#237; p&#345;&#237;kazu.
Ani potvrzen&#253; &#250;daj automaticky nedokazuje mechanick&#253; pohyb.</p>
<div class="service-form"><label>C&#237;lov&#225; poloha<select id="servSwingPos">
<option value="fix_pos_1">FIX 1</option><option value="fix_pos_2">FIX 2</option><option value="fix_pos_3">FIX 3</option>
<option value="fix_pos_4">FIX 4</option><option value="fix_pos_5">FIX 5</option></select></label>
<button class="btn primary" type="button" id="servSwingTest">Spustit test</button></div>
<p id="servSwingReport" class="service-status">Zat&#237;m nebyl zah&#225;jen test.</p>
<div class="service-form"><button class="btn" type="button" data-observe="yes">Lamely se fyzicky pohnuly</button>
<button class="btn" type="button" data-observe="no">Bez viditeln&#233;ho pohybu</button>
<button class="btn" type="button" data-observe="unknown">Nev&#237;m / reset pozorov&#225;n&#237;</button></div>
<p class="service-note">Pozorov&#225;n&#237; pohybu se ukl&#225;d&#225; pouze do RAM ESP. Pro srovn&#225;n&#237; otestujte FIX 1 a FIX 5 a zaznamenejte skute&#269;n&#253; pohyb.</p></section>
<section class="service-card"><h2>Z&#225;znam komunikace</h2><div class="service-form">
<button class="btn" type="button" id="servRawLoad">Na&#269;&#237;st RAW z&#225;znam</button>
<button class="btn" type="button" id="servSniffOn">Z&#225;znam zapnout</button>
<button class="btn" type="button" id="servSniffOff">Z&#225;znam vypnout</button>
<button class="btn" type="button" id="servSniffClear">Vymazat z&#225;znam</button></div>
<p class="service-note">TX ud&#225;losti ovl&#225;dac&#237;ho rozhran&#237; mohou b&#253;t jen intern&#237; markery, ne p&#345;esn&#233; bajty na vodi&#269;i.
RX z&#225;znam obsahuje dek&#243;dovan&#233; payloady. Na&#269;ten&#237; logu je ru&#269;n&#237;.</p>
<pre id="servRaw" class="service-log">Stiskn&#283;te "Na&#269;&#237;st RAW z&#225;znam".</pre></section>
</div><script src="/service.js"></script></body></html>
)SERVICEHTML";

static const char SERVICE_JS[] PROGMEM = R"SERVICEJS(
'use strict';
const $serv = id => document.getElementById(id);
const configs = {
  power:[['on','ON'],['off','OFF']],
  setpoint:Array.from({length:14},(_,i)=>[String(i+17),String(i+17)+' C']),
  mode:[['auto','Auto'],['cool','Cool'],['heat','Heat'],['dry','Dry'],['fan_only','Fan only']],
  fan:[['auto','Auto'],['quiet','Quiet'],['lvl_1','Level 1'],['lvl_2','Level 2'],['lvl_3','Level 3'],['lvl_4','Level 4'],['lvl_5','Level 5']],
  swing:[['fix','Fix'],['v_swing','Vertical'],['h_swing','Horizontal'],['hv_swing','V+H'],['fix_pos_1','Fix 1'],['fix_pos_2','Fix 2'],['fix_pos_3','Fix 3'],['fix_pos_4','Fix 4'],['fix_pos_5','Fix 5']],
  pure:[['off','Off'],['on','On']],
  powerSelect:[['100%','100%'],['75%','75%'],['50%','50%']],
  operation:[['normal','Normal'],['high_power','High power'],['silent_1','Silent 1'],['eco','Eco'],['eight_deg','8 C'],['silent_2','Silent 2'],['fireplace_1','Fireplace 1'],['fireplace_2','Fireplace 2']],
  wifiLed:[['off','Off'],['on','On']]
};
const text = v => v == null || v === '' || v === 'UNKNOWN' ? '-' : String(v);
const appendCell = (row,content,isHeader=false) => {
  const cell=document.createElement(isHeader?'th':'td');
  cell.textContent=text(content);
  row.appendChild(cell);
};
const displayTable = (id,rows) => {
  const tbody=$serv(id); tbody.textContent='';
  for(const cells of rows){
    const tr=document.createElement('tr');
    cells.forEach((v,i)=>appendCell(tr,v,i===0));
    tbody.appendChild(tr);
  }
};
function message(msg,isError=false){
  const area=$serv('servMsg'); area.textContent=msg; area.style.color=isError?'var(--bad)':'var(--fg)';
}
function populate(){
  const v=$serv('servValue'),name=$serv('servField').value;
  v.textContent='';
  for(const [id,label] of configs[name]||[]){
    const op=document.createElement('option'); op.value=id; op.textContent=label; v.appendChild(op);
  }
}
async function readState(){
  const res=await fetch('/api/service/state',{credentials:'same-origin',cache:'no-store'});
  if(!res.ok) throw new Error('HTTP '+res.status);
  return await res.json();
}
async function update(){
  try {
    const x=await readState();
    $serv('servConnection').textContent='HVAC UART: '+(x.connected?'Pripojeno':'Odpojeno');
    $serv('servConnection').style.color=x.connected?'var(--ok)':'var(--bad)';
    $serv('servBuild').textContent='Verze '+text(x.build)+' | uptime '+Math.floor(x.uptime_ms/1000)+' s';
    const t=x.telemetry||{}, d=x.diag||{};
    displayTable('servTelemetry',[
      ['RX posledni odpoved',d.last_rx_age_ms==null?'-':(d.last_rx_age_ms+' ms')],
      ['UART handshake pokusy',d.handshakes],['UART RX OK davky',d.rx_ok],['UART RX chybne davky',d.rx_invalid],
      ['Room temp',t.room_temp_c==null?'-':t.room_temp_c+' C'],['Outdoor temp',t.outdoor_temp_c==null?'-':t.outdoor_temp_c+' C'],
      ['Kompresor / CDU',t.cdu_running?'Bezi':'Nebezi / nezname'],
      ['On timer',t.on_timer],['Off timer',t.off_timer],
      ['Wi-Fi',d.wifi_connected?'Pripojeno':'Odpojeno'],['RSSI',d.rssi_dbm==null?'-':d.rssi_dbm+' dBm'],
      ['MQTT',d.mqtt_connected?'Pripojeno':'Odpojeno'],['Modbus',d.modbus_active?'Zapnuto':'Vypnuto'],
      ['Volna RAM',d.free_heap+' B'],['RAW log',d.sniff_enabled?'Zapnut':'Vypnut'],
      ['RAW TX/RX',String(d.raw_tx)+' / '+String(d.raw_rx)]
    ]);
    const current=x.actual||{},requested=x.requested||{},sent=x.sent||{};
    const names={state:'Power',setpoint:'Setpoint',mode:'Mode',fan:'Fan',swing:'Swing',pure:'PURE',powerSelect:'Power Select',operation:'Operation',wifiLed:'WiFi LED'};
    displayTable('servSettings',Object.keys(names).map(k=>[names[k],current[k],requested[k],sent[k]]));
    const test=x.swing_test||{};
    const result=$serv('servSwingReport');
    if(test.active){
      const states={pending:'Test probiha',reported:'Protokol vratil pozadovanou polohu',unconfirmed:'Zmena nebyla po testu potvrzena',already:'Cilova poloha byla hlasena uz pred testem',no_rx:'Neni prijata nova UART odpoved'};
      const manual={yes:'Potvrzen fyzicky pohyb',no:'Bez viditelneho pohybu',unknown:'Bez lidskeho pozorovani'};
      result.textContent=(states[test.status]||text(test.status))+' | pred: '+text(test.before)+
        ', cil: '+text(test.target)+', nyni: '+text(test.actual)+', cas: '+String(test.elapsed_ms||0)+' ms | '+
        (manual[test.observation]||'Bez lidskeho pozorovani');
      result.style.color=test.status==='reported'?'var(--ok)':(test.status==='unconfirmed'?'var(--warn)':'var(--fg)');
    }
  } catch(e){ message('Nepodarilo se nacist servisni stav: '+e.message,true); }
}
let serviceCsrfPromise;
async function serviceCsrf(){
  if(!serviceCsrfPromise){
    serviceCsrfPromise=fetch('/api/security',{credentials:'same-origin',cache:'no-store'})
      .then(async res=>{if(!res.ok)throw new Error('HTTP '+res.status);return (await res.json()).csrf;})
      .catch(error=>{serviceCsrfPromise=null;throw error;});
  }
  return serviceCsrfPromise;
}
async function action(name,field,value){
  const data=new URLSearchParams({action:name});
  data.set('csrf',await serviceCsrf());
  if(field!==undefined)data.set('field',field);
  if(value!==undefined)data.set('value',value);
  const res=await fetch('/api/service/action',{
    method:'POST',credentials:'same-origin',cache:'no-store',
    headers:{'Content-Type':'application/x-www-form-urlencoded','X-Requested-With':'XMLHttpRequest'},
    body:data.toString()
  });
  const reply=await res.text();
  if(!res.ok) throw new Error('HTTP '+res.status+': '+reply);
  return reply;
}
async function invoke(name,field,value){
  try {
    const response=await action(name,field,value);
    message(response); await update();
  } catch(e){ message('Povel nebyl prijat: '+e.message,true); }
}
function bind(){
  $serv('servField').addEventListener('change',populate);populate();
  $serv('servRefresh').onclick=()=>update();
  $serv('servQuery').onclick=()=>invoke('refresh');
  $serv('servApply').onclick=()=>{
    if(!$serv('servArmed').checked){ message('Nejprve povolte servisni ovladani.',true);return; }
    const field=$serv('servField').value,value=$serv('servValue').value;
    if(['pure','powerSelect','operation','wifiLed'].includes(field) &&
       !confirm('Tato funkce nemusi byt klimatizaci podporovana. Odeslat '+field+' = '+value+'?'))return;
    invoke('set',field,value);
  };
  $serv('servSwingTest').onclick=()=>{
    if(!$serv('servArmed').checked){message('Nejprve povolte servisni ovladani.',true);return;}
    invoke('test_swing',undefined,$serv('servSwingPos').value);
  };
  document.querySelectorAll('[data-observe]').forEach(el=>el.onclick=()=>invoke('observe_swing',undefined,el.dataset.observe));
  $serv('servRawLoad').onclick=async()=>{
    try {
      const r=await fetch('/api/raw.txt',{credentials:'same-origin',cache:'no-store'});
      if(!r.ok) throw new Error('HTTP '+r.status);
      $serv('servRaw').textContent=await r.text();
    } catch(e){ message('RAW log: '+e.message,true); }
  };
  $serv('servSniffOn').onclick=()=>invoke('sniff',undefined,'on');
  $serv('servSniffOff').onclick=()=>invoke('sniff',undefined,'off');
  $serv('servSniffClear').onclick=()=>invoke('sniff',undefined,'clear');
  update();setInterval(update,3000);
}
document.addEventListener('DOMContentLoaded',bind);
)SERVICEJS";
