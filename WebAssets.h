#pragma once
#include <Arduino.h>

// ============ Embedded UI assets (modern responsive) ============
// Dashboard v2: standalone CSS in flash (no remote dependencies).
static const char APP_CSS[] PROGMEM = R"CSS(

:root{
  color-scheme:light;
  --bg:#f3f6fb; --surface:#fff; --surface-2:#f8faff; --ink:#17243a;
  --muted:#6a7890; --line:#e3e9f2; --accent:#3468e8; --accent-soft:#ebf1ff;
  --accent-on:#fff; --good:#11956c; --warn:#d68a32; --danger:#d84954;
  --shade:0 12px 35px rgba(28,48,88,.045),0 2px 8px rgba(28,48,88,.025);
  --radius:20px; --control-radius:12px;
  font:16px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,Arial,sans-serif;
}
@media(prefers-color-scheme:dark){
  :root:not([data-theme="light"]){color-scheme:dark;--bg:#0c1220;--surface:#161f30;--surface-2:#1c2940;--ink:#edf3ff;--muted:#a3b1c6;--line:#2c3b51;--accent:#8bacff;--accent-soft:#25385a;--accent-on:#0d1930;--good:#62dab4;--warn:#ffbd76;--danger:#ff8993;--shade:0 12px 36px rgba(0,0,0,.14)}
}
:root[data-theme="dark"]{color-scheme:dark;--bg:#0c1220;--surface:#161f30;--surface-2:#1c2940;--ink:#edf3ff;--muted:#a3b1c6;--line:#2c3b51;--accent:#8bacff;--accent-soft:#25385a;--accent-on:#0d1930;--good:#62dab4;--warn:#ffbd76;--danger:#ff8993;--shade:0 12px 36px rgba(0,0,0,.14)}
*{box-sizing:border-box}html{scroll-behavior:smooth}body{margin:0;background:var(--bg);color:var(--ink);min-height:100vh;letter-spacing:-.012em}
button,input,select{font:inherit}button{cursor:pointer}button:disabled{cursor:not-allowed;opacity:.55}a{color:inherit;text-decoration:none}a:hover{text-decoration:none}
:focus-visible{outline:3px solid var(--accent);outline-offset:3px}::selection{background:var(--accent-soft)}
[hidden]{display:none!important}.muted,.subtle,.soft-text{color:var(--muted)}.subtle{font-size:12px}.skip-link{position:absolute;top:-80px;left:16px;z-index:20;background:var(--surface);padding:10px}
.skip-link:focus{top:10px}
.app-shell{width:min(1160px,calc(100% - 52px));margin:0 auto}
.topbar{display:flex;align-items:center;gap:24px;min-height:102px;border-bottom:1px solid var(--line)}
.brand{display:inline-flex;gap:12px;align-items:center;white-space:nowrap}
.brand-symbol{width:44px;height:44px;background:var(--ink);color:var(--surface);display:grid;place-items:center;border-radius:14px;position:relative}
.brand-dot{width:7px;height:7px;border-radius:50%;background:#76dfca;position:absolute;top:12px;right:9px}
.brand-waves{font-size:30px;font-weight:800;transform:rotate(90deg);line-height:1}
.brand-name{display:flex;flex-direction:column;line-height:1.1;gap:4px;letter-spacing:.09em}.brand-name strong{font-size:16px;letter-spacing:.12em}.brand-name small{font-size:9px;color:var(--muted);font-weight:800;letter-spacing:.18em}
.primary-nav{display:flex;align-items:center;gap:6px;margin:auto}
.nav-item{white-space:nowrap;display:inline-flex;align-items:center;justify-content:center;background:none;border:0;color:var(--muted);font-size:13px;font-weight:650;padding:12px 14px;border-radius:11px}
.nav-item:hover{color:var(--ink);background:var(--surface-2)}
.nav-item.is-active{color:var(--accent);background:var(--accent-soft)}
.top-actions{display:flex;align-items:center;gap:11px}
.connection-pill{border:1px solid var(--line);border-radius:50px;background:var(--surface);display:inline-flex;align-items:center;gap:8px;padding:10px 13px;white-space:nowrap;font-size:12px;font-weight:650;color:var(--muted)}
.status-light{display:inline-block;width:8px;height:8px;border-radius:50%;background:var(--warn);box-shadow:0 0 0 3px rgba(214,138,50,.12)}
.connection-pill[data-state="ok"] .status-light{background:var(--good);box-shadow:0 0 0 3px rgba(17,149,108,.14)}
.connection-pill[data-state="bad"] .status-light{background:var(--danger);box-shadow:0 0 0 3px rgba(216,73,84,.11)}
.icon-button{width:40px;height:40px;display:grid;place-items:center;background:var(--surface);border:1px solid var(--line);border-radius:12px;color:var(--ink);font-size:21px}
.icon-button:hover{border-color:var(--accent)}
main{padding-top:35px}.page-heading{display:flex;align-items:end;justify-content:space-between;gap:16px;margin:5px 0 26px}
.eyebrow,.section-label{font-size:10px;font-weight:850;letter-spacing:.17em;color:var(--accent)}.eyebrow{margin:0 0 10px}.crumb{color:var(--muted);padding:0 8px}
h1,h2,h3,p{margin-top:0}h1{margin-bottom:5px;font-size:clamp(25px,3.5vw,36px);font-weight:750;letter-spacing:-.048em}
h2{font-size:17px;font-weight:740;letter-spacing:-.025em;margin:5px 0 0}.page-heading .muted{margin:0;font-size:13px}
.last-update{font-size:12px;color:var(--muted);white-space:nowrap;padding-bottom:4px}
.dashboard-grid{display:grid;grid-template-columns:minmax(0,1.17fr) minmax(0,.83fr);gap:18px;align-items:stretch}
.card{background:var(--surface);border:1px solid var(--line);border-radius:var(--radius);padding:23px 25px;box-shadow:var(--shade);min-width:0}
.card-head{display:flex;align-items:center;justify-content:space-between;gap:12px;min-height:23px}.card-head h2{margin:0}
.chip{font-size:11px;font-weight:740;padding:7px 10px;border-radius:9px;white-space:nowrap}.chip-soft{background:var(--surface-2);color:var(--muted)}
.thermostat{background:linear-gradient(140deg,var(--surface) 5%,var(--surface-2) 110%);display:flex;flex-direction:column}
.setpoint-top{padding:33px 0 17px;text-align:center}.setpoint-top>p{font-size:13px;color:var(--muted);margin:0 0 6px;font-weight:600}
.thermostat-controls{display:flex;align-items:center;justify-content:center;gap:19px}
.target-value{font-size:clamp(65px,8.3vw,97px);font-weight:650;letter-spacing:-.09em;white-space:nowrap;line-height:1.2;font-variant-numeric:tabular-nums;min-width:165px;text-align:center}
.target-value span{font-size:25px;color:var(--muted);font-weight:500;letter-spacing:-.04em;margin-left:4px}
.temp-adjust{width:45px;height:45px;border:1px solid var(--line);background:var(--surface);border-radius:50%;font-size:27px;color:var(--ink);display:grid;place-items:center;line-height:1;padding-bottom:4px}
.temp-adjust:hover:not(:disabled){color:var(--accent);border-color:var(--accent);background:var(--accent-soft)}
.slider-wrap{padding:7px 22px 25px}.temp-slider{width:100%;margin:0;accent-color:var(--accent);cursor:pointer;min-height:35px}
.slider-labels{display:flex;justify-content:space-between;color:var(--muted);font-size:11px;font-weight:700;padding-top:2px}
.thermostat-bottom{margin-top:auto;padding-top:18px;border-top:1px solid var(--line);display:flex;align-items:center;justify-content:space-between;gap:10px}
.thermostat-note{display:flex;align-items:center;gap:9px;font-size:12px;color:var(--muted);font-weight:600}
.mini-dot{width:7px;height:7px;border-radius:50%;background:var(--muted)}
.power-button{display:inline-flex;align-items:center;justify-content:center;gap:9px;padding:13px 19px;background:var(--accent);color:var(--accent-on);border:1px solid transparent;border-radius:12px;font-size:13px;font-weight:750;min-width:135px}
.power-button svg{height:18px;width:18px}.power-button:hover{filter:brightness(.95)}
.power-button[data-power="on"]{background:var(--accent-soft);color:var(--accent);border-color:var(--line)}
.overview .card-head{margin-bottom:24px}.temperature-readings{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.reading{min-height:142px;border-radius:15px;background:var(--surface-2);display:flex;flex-direction:column;gap:12px;padding:17px}
.reading-icon{font-size:16px;line-height:1;font-weight:800;width:32px;height:32px;background:var(--accent-soft);color:var(--accent);display:grid;place-items:center;border-radius:10px}
.reading-icon.outdoor{background:rgba(17,149,108,.1);color:var(--good)}.reading-title{display:block;font-size:12px;color:var(--muted);margin-bottom:4px}
.reading strong{font-size:31px;font-weight:690;white-space:nowrap;letter-spacing:-.05em;font-variant-numeric:tabular-nums}
.divider{height:1px;background:var(--line);margin:20px 0}
.status-rows{display:grid;gap:17px}.status-row{display:flex;align-items:center;justify-content:space-between;gap:12px;font-size:12px}.status-row>span:first-child{color:var(--muted)}
.status-row strong{font-weight:700;font-size:12px}.inline-state{font-size:12px;font-weight:700;color:var(--good)}
.mode-card,.fan-card,.swing-card,.quick-card{min-height:190px}.mode-card{grid-column:1/-1}
.mode-card .card-head,.fan-card .card-head,.swing-card .card-head,.quick-card .card-head{margin-bottom:20px}
.choice-grid{display:grid;gap:11px}.choice-grid-five{grid-template-columns:repeat(5,minmax(0,1fr))}
.choice{min-width:0;min-height:93px;border:1px solid var(--line);background:var(--surface);border-radius:13px;color:var(--muted);font-weight:730;font-size:12px;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:9px;padding:12px 5px}
.choice-symbol{font-size:25px;color:var(--ink);font-weight:650;line-height:1}
.choice:hover{border-color:var(--accent);color:var(--ink);background:var(--surface-2)}
.choice[aria-pressed="true"]{border:1.5px solid var(--accent);color:var(--accent);background:var(--accent-soft)}
.choice[aria-pressed="true"] .choice-symbol{color:var(--accent)}
.fan-grid{grid-template-columns:repeat(6,minmax(0,1fr))}.swing-grid{grid-template-columns:repeat(4,minmax(0,1fr))}
.choice.compact{min-height:49px;padding:7px 5px;font-size:12px;gap:0}
.hint{font-size:11px;color:var(--muted);margin:16px 0 0;line-height:1.7}
.hint a{color:var(--accent);font-weight:700}.hint a:hover{text-decoration:underline}
.quick-links{display:grid;gap:8px}
.quick-links>a{display:grid;grid-template-columns:38px 1fr 14px;align-items:center;gap:10px;padding:11px 10px;border-radius:12px;border:1px solid var(--line)}
.quick-links>a:hover{border-color:var(--accent);background:var(--surface-2)}.quick-links strong{display:block;font-size:12px}
.quick-links small{display:block;margin-top:3px;color:var(--muted);font-size:10px}.quick-icon{height:34px;width:34px;display:grid;place-items:center;border-radius:10px;background:var(--accent-soft);color:var(--accent);font-size:20px;font-weight:700}.quick-arrow{font-size:17px;color:var(--muted)}
.settings-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:18px}.settings-card .card-head{margin-bottom:18px}
.field-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:14px}.field{font-size:12px;font-weight:650;color:var(--muted);display:flex;flex-direction:column;gap:7px}.field.wide{grid-column:1/-1}
.field input,.field select{background:var(--surface-2);width:100%;border:1px solid var(--line);border-radius:10px;color:var(--ink);padding:12px 11px;min-width:0;font-weight:500;min-height:45px}
.field input:focus,.field select:focus{border-color:var(--accent);outline:2px solid var(--accent-soft)}
.field input[readonly]{opacity:.66;cursor:default}.field input::placeholder{color:var(--muted);opacity:.7}
.settings-actions{display:flex;align-items:center;justify-content:flex-end;gap:11px;margin-top:19px;flex-wrap:wrap}
.settings-actions>span{margin-right:auto;font-size:12px}.btn{border-radius:12px;font-size:13px;font-weight:740;min-height:45px;padding:11px 17px;border:1px solid var(--line);background:var(--surface);color:var(--ink)}
.btn-primary{background:var(--accent);color:var(--accent-on);border-color:var(--accent)}.btn-ghost:hover{border-color:var(--accent);color:var(--accent)}
.app-footer{display:flex;justify-content:space-between;color:var(--muted);border-top:1px solid var(--line);margin:35px 0 24px;padding-top:20px;font-size:11px;gap:12px}
.toast{position:fixed;bottom:25px;right:24px;z-index:50;max-width:min(400px,calc(100vw - 30px));color:var(--surface);background:var(--ink);padding:15px 20px;box-shadow:0 16px 50px rgba(10,21,38,.3);border-radius:13px;font-size:13px;font-weight:600}
.toast[data-type="error"]{background:var(--danger);color:#fff}.toast[data-type="ok"]{background:var(--good);color:#fff}
.language-box{display:flex;align-items:center;gap:7px;font-size:12px;color:var(--muted);font-weight:700}
.language-box select{padding:7px;background:var(--surface);border:1px solid var(--line);border-radius:9px;color:var(--ink)}
.feature-grid{margin-top:18px}
.btn-danger{border-color:var(--danger);color:var(--danger)}
.btn-danger:hover{background:var(--danger);color:#fff}
.field small{color:var(--muted);font-size:10px}
@media(max-width:1060px){.topbar{gap:14px}.nav-item{padding:10px 11px}.brand-name strong{font-size:13px}.brand-symbol{width:39px;height:39px}.card{padding:20px}.app-shell{width:min(100% - 36px,1160px)}}
@media(max-width:900px){.topbar{flex-wrap:wrap;padding:18px 0;min-height:0}.brand{margin-right:auto}.primary-nav{order:3;width:100%;justify-content:space-between;margin:5px 0 0}.top-actions{margin-left:auto}.dashboard-grid{grid-template-columns:1fr 1fr}.thermostat,.overview{grid-column:1/-1}.temperature-readings{grid-template-columns:1fr 1fr}.settings-grid{grid-template-columns:1fr}}
@media(max-width:620px){.app-shell{width:calc(100% - 24px)}main{padding-top:22px}.page-heading{align-items:start;flex-direction:column;margin-bottom:20px;gap:9px}.last-update{font-size:11px}.topbar{gap:12px}.top-actions{gap:8px}.connection-pill{padding:9px 10px;font-size:11px}.primary-nav{gap:0;overflow:auto;scrollbar-width:none}.nav-item{padding:10px 9px;font-size:12px}.dashboard-grid{grid-template-columns:1fr;gap:12px}.thermostat,.overview,.mode-card{grid-column:auto}.card{padding:18px;border-radius:17px}.choice-grid-five{grid-template-columns:repeat(5,minmax(0,1fr));gap:6px}.choice{min-height:78px;font-size:10px;padding:8px 2px}.choice-symbol{font-size:21px}.fan-grid{gap:7px}.temperature-readings{gap:9px}.reading{padding:13px;min-height:121px}.reading strong{font-size:27px}.setpoint-top{padding-top:26px}.thermostat-controls{gap:9px}.temp-adjust{width:43px;height:43px}.target-value{min-width:157px;font-size:72px}.slider-wrap{padding-left:8px;padding-right:8px}.power-button{min-width:115px;padding:12px 13px}.field-grid{grid-template-columns:1fr}.field.wide{grid-column:auto}.settings-actions>span{flex-basis:100%}.app-footer{margin-top:26px}}
@media(max-width:355px){.brand-name small{display:none}.brand-symbol{width:35px;height:35px}.connection-pill{padding:8px}.choice-grid-five{grid-template-columns:repeat(3,1fr)}.temp-adjust{width:37px;height:37px}.target-value{font-size:61px;min-width:130px}.thermostat-controls{gap:5px}.thermostat-note{font-size:10px}}
@media(prefers-reduced-motion:reduce){*,*:before,*:after{scroll-behavior:auto!important;animation:none!important;transition:none!important}}

)CSS";

// Dashboard v2: dependency-free event-driven controls, API compatible.
static const char APP_JS[] PROGMEM = R"JS(
'use strict';
(() => {
const $ = id => document.getElementById(id);
const $$ = selector => document.querySelectorAll(selector);
const range = $('tempRange'), target = $('targetTemp'), power = $('powerButton');
const labels = {auto:'Auto',cool:'Chlazení',heat:'Topení',dry:'Odvlhčení',fan_only:'Ventilátor',fix:'Pevné',v_swing:'Svisle',h_swing:'Vodorovně',hv_swing:'Obě osy',vh_swing:'Obě osy',quiet:'Tichý',lvl_1:'Stupeň 1',lvl_2:'Stupeň 2',lvl_3:'Stupeň 3',lvl_4:'Stupeň 4',lvl_5:'Stupeň 5'};
let state=null, polling=false, editedAt=0, pending=null, toastTimer=0, errors=0, loadedSettings=false;
const describe = value => (translations && translations[value]) || labels[value] || value || '—';
const tempText = v => typeof v==='number' && Number.isFinite(v) ? v.toFixed(1)+' °C' : '—';
function toast(message,type='ok'){
  const element=$('toast');
  element.textContent=message;element.dataset.type=type;element.hidden=false;
  clearTimeout(toastTimer);toastTimer=setTimeout(()=>element.hidden=true,4500);
}
function updatePill(status,message){
  $('connectionPill').dataset.state=status;
  $('connectionText').textContent=message;
}
function selectButtons(selector,value){
  $$(selector).forEach(button=>button.setAttribute('aria-pressed',String(button.dataset.match===value)));
}
function updateTemperature(t){
  if(!Number.isInteger(t)||t<17||t>30)return;
  range.value=String(t);
  target.innerHTML=String(t)+'<span>°C</span>';
  range.style.setProperty('--value',((t-17)*100/13)+'%');
}
function render(data){
  state=data;errors=0;
  const online=data.hvac==='Active';
  updatePill(online?'ok':'bad',online?(translations.connection_online||'HVAC online'):(translations.connection_offline||'HVAC offline'));
  const deviceName=data.name||'Toshiba HVAC';
  $('dashboardTitle').textContent=deviceName;
  $('roomName').textContent=data.room ? 'Místnost: '+data.room : 'Lokální ovládání klimatizace';
  $('deviceIdentifier').textContent='ID: '+(data.id||'—');
  $('lastUpdate').textContent='Aktualizováno '+new Date().toLocaleTimeString('cs-CZ',{hour:'2-digit',minute:'2-digit',second:'2-digit'});
  const isOn=data.power==='on';
  power.dataset.power=isOn?'on':'off';power.setAttribute('aria-pressed',String(isOn));
  $('powerButtonLabel').textContent=isOn?(translations.power_off||'Vypnout'):(translations.power_on||'Zapnout');
  $('powerInfo').textContent=isOn?(translations.power_on_state||'Klimatizace zapnutá'):(translations.power_off_state||'Klimatizace vypnutá');
  $('hvacState').textContent=online?'Komunikuje':'Bez spojení';
  $('hvacState').style.color=online?'var(--good)':'var(--danger)';
  $('indoorTemp').textContent=tempText(data.t_in);
  $('outdoorTemp').textContent=tempText(data.t_out);
  $('modeCaption').textContent=describe(data.mode);
  $('operatingStatus').textContent=describe(data.oper);
  $('fanCaption').textContent=describe(data.fan);
  $('swingCaption').textContent=describe(data.swing);
  $('networkState').textContent=(data.wifi_connected?'Wi-Fi OK':'Wi-Fi offline')+' / '+(data.mqtt_connected?'MQTT OK':'MQTT nepřipojeno');
  const rxAge=data.uart_last_rx_age_ms;
  $('uartAge').textContent=typeof rxAge==='number'?'před '+(rxAge<1000?'< 1 s':Math.round(rxAge/1000)+' s'):'Zatím bez dat';
  selectButtons('[data-cmd^="MODE_"]',data.mode);
  selectButtons('[data-cmd^="FAN_"]',data.fan);
  selectButtons('[data-cmd^="SWING_"]',data.swing==='vh_swing'?'hv_swing':data.swing);
  if(!pending||Date.now()-pending.at>6500){
    pending=null;
    if(Date.now()-editedAt>2200)updateTemperature(Number(data.temp));
  }
  range.disabled=false;
  $('tempDown').disabled=false;$('tempUp').disabled=false;
}
let csrfTokenCache='';
async function csrfTokenForRequest(){
  if(csrfTokenCache)return csrfTokenCache;
  const response=await fetch('/api/security/token',{cache:'no-store',credentials:'same-origin'});
  if(!response.ok)throw new Error('Authentication / CSRF HTTP '+response.status);
  csrfTokenCache=(await response.json()).csrf||'';
  return csrfTokenCache;
}
async function request(url,options={}){
  if(options.method && options.method.toUpperCase()==='POST'){
    options.headers=Object.assign({},options.headers,{'X-CSRF-Token':await csrfTokenForRequest()});
  }
  const response=await fetch(url,Object.assign({cache:'no-store',credentials:'same-origin'},options));
  if(!response.ok)throw new Error((await response.text().catch(()=>''))||'HTTP '+response.status);
  return response;
}
async function poll(){
  if(document.hidden||polling)return;
  polling=true;
  try{render(await (await request('/api/state.json')).json());}
  catch(err){errors++;updatePill('bad',errors>1?'Nedostupné':'Načítání selhalo');if(errors===2)toast('Nelze načíst stav ESP. Zkontrolujte síť a přihlášení.','error');}
  finally{polling=false;}
}
async function command(cmd){
  const body=new URLSearchParams({cmd});
  pending={cmd,at:Date.now()};
  try{
    await request('/command',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded','X-Requested-With':'XMLHttpRequest'},body:body.toString()});
    toast('Příkaz '+cmd.replaceAll('_',' ')+' byl přijat; čekám na potvrzení jednotky.');
    setTimeout(poll,900);
    return true;
  }catch(err){pending=null;toast('Příkaz se nepodařilo odeslat: '+err.message,'error');poll();return false;}
}
async function setTemp(value){
  const t=Math.max(17,Math.min(30,Number(value)));
  if(!Number.isInteger(t))return;
  editedAt=Date.now();
  updateTemperature(t);
  await command('TEMP_'+t);
}
range.addEventListener('input',()=>{editedAt=Date.now();updateTemperature(Number(range.value));});
range.addEventListener('change',()=>setTemp(range.value));
$('tempDown').addEventListener('click',()=>setTemp(Number(range.value)-1));
$('tempUp').addEventListener('click',()=>setTemp(Number(range.value)+1));
power.addEventListener('click',()=>command(state?.power==='on'?'STATE_OFF':'STATE_ON'));
$$('[data-cmd]').forEach(button=>button.addEventListener('click',()=>command(button.dataset.cmd)));

const themeKey='toshiba-ui-theme';
try{const saved=localStorage.getItem(themeKey);if(saved==='light'||saved==='dark')document.documentElement.dataset.theme=saved;}catch(_){}
$('themeToggle').addEventListener('click',()=>{
  const current=document.documentElement.dataset.theme;
  const next=(current==='dark'||(!current&&matchMedia('(prefers-color-scheme: dark)').matches))?'light':'dark';
  document.documentElement.dataset.theme=next;
  try{localStorage.setItem(themeKey,next);}catch(_){}
});
function showView(view){
  const setting=view==='settings';
  $('dashboardView').hidden=setting;
  $('settingsView').hidden=!setting;
  $$('.nav-item[data-view]').forEach(b=>{
    const active=b.dataset.view===view;
    b.classList.toggle('is-active',active);
    if(active)b.setAttribute('aria-current','page');else b.removeAttribute('aria-current');
  });
  if(setting){if(!loadedSettings)loadSettings();loadFeatures();}
}
$$('[data-view]').forEach(el=>el.addEventListener('click',()=>showView(el.dataset.view)));
async function loadSettings(){
  const notice=$('settingsMessage');
  notice.textContent='Načítám konfiguraci…';
  try{
    const config=await (await request('/api/config.json')).json();
    const form=$('settingsForm');
    for(const [key,value]of Object.entries(config)){
      const input=form.elements.namedItem(key);
      if(!input)continue;
      if(input.type==='password'){input.value='';continue;}
      input.value=String(value??'');
    }
    loadedSettings=true;
    notice.textContent='Konfigurace načtena. Prázdná hesla ponechají dosavadní hodnoty.';
  }catch(err){notice.textContent='Konfiguraci nelze načíst: '+err.message;toast('Načtení konfigurace selhalo.','error');}
}
$('reloadSettings').addEventListener('click',loadSettings);
$('settingsForm').addEventListener('submit',async event=>{
  event.preventDefault();
  const form=event.currentTarget,save=$('saveSettings'),notice=$('settingsMessage');
  const body=new URLSearchParams();
  for(const el of form.elements){
    if(!el.name||el.disabled||el.type==='submit')continue;
    if(el.type==='password'&&!el.value)continue;
    body.set(el.name,el.value);
  }
  save.disabled=true;notice.textContent='Ukládám nastavení…';
  try{
    await request('/mqttconfig',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded','X-Requested-With':'XMLHttpRequest'},body:body.toString()});
    loadedSettings=false;notice.textContent='Nastavení uloženo.';
    toast('Konfigurace úspěšně uložena.');
    // Password may change HTTP Basic auth. Reauthentication occurs at next fetch.
    setTimeout(loadSettings,950);
  }catch(err){notice.textContent='Chyba uložení: '+err.message;toast('Nastavení nebylo uloženo.','error');}
  finally{save.disabled=false;}
});
document.addEventListener('visibilitychange',()=>{if(!document.hidden)poll();});

const capabilityKeys = [
  'swing_fixed','swing_vertical','swing_horizontal','swing_both','swing_positions',
  'pure','power_select','operation','wifi_led'
];
const capabilityNames = {
  swing_fixed:'Pevná poloha / Fixed',swing_vertical:'Svisle / Vertical',
  swing_horizontal:'Vodorovně / Horizontal',swing_both:'Obě osy / Both',
  swing_positions:'Pevné pozice 1–5 / Fixed positions',pure:'PURE',
  power_select:'Power Select',operation:'Operation',wifi_led:'Wi-Fi LED'
};
let translations = {};
let language = 'cs';
const originalLiteralNodes = new WeakMap();
function translateLiterals(dictionary){
  const walker=document.createTreeWalker(document.body,NodeFilter.SHOW_TEXT);
  let node;
  while((node=walker.nextNode())){
    const parent=node.parentElement;
    if(!parent || parent.closest('[data-i18n]') || ['SCRIPT','STYLE'].includes(parent.tagName))continue;
    if(!originalLiteralNodes.has(node))originalLiteralNodes.set(node,node.nodeValue);
    const original=originalLiteralNodes.get(node);
    const trimmed=original.trim();
    if(!trimmed)continue;
    const newText=(dictionary.literals||{})[trimmed] || trimmed;
    if(newText!==trimmed)node.nodeValue=original.replace(trimmed,newText);
    else node.nodeValue=original;
  }
}
async function setLanguage(code){
  if(code!=='cs' && code!=='en') code='cs';
  try {
    const dictionary = await (await request('/locale/'+code+'.json')).json();
    translations=dictionary;language=code;
    document.documentElement.lang=code;
    $('languageSelect').value=code;
    try{localStorage.setItem('toshiba-language',code);}catch(_){}
    document.querySelectorAll('[data-i18n]').forEach(el=>{
      const msg=dictionary[el.dataset.i18n];
      if(typeof msg==='string')el.textContent=msg;
    });
    translateLiterals(dictionary);
    if(state)render(state);
  } catch(err){toast('Language / Jazyk: '+err.message,'error');}
}
$('languageSelect').addEventListener('change',event=>setLanguage(event.target.value));
try{language=localStorage.getItem('toshiba-language')||'cs'}catch(_){}
setLanguage(language);
const t = key => translations[key]||key;
async function loadFeatures(){
  try{
    const cfg=await (await request('/api/features')).json();
    $('haDiscovery').value=cfg.ha_discovery?'1':'0';
    const root=$('capabilityFields');root.replaceChildren();
    for(const key of capabilityKeys){
      const field=document.createElement('label');field.className='field';
      const caption=document.createElement('span');caption.textContent=capabilityNames[key];
      const select=document.createElement('select');select.dataset.capability=key;
      for(const [value,label] of [['auto',t('unknown')],['supported',t('supported')],['unsupported',t('unsupported')]]){
        const option=document.createElement('option');option.value=value;option.textContent=label;select.appendChild(option);
      }
      select.value=cfg.capabilities?.[key]||'auto';
      const observed=document.createElement('small');
      observed.textContent=cfg.observed?.[key]?'UART: observed':'UART: unverified';
      field.append(caption,select,observed);root.appendChild(field);
    }
    $('featureMessage').textContent=cfg.ha_available?'MQTT online':'MQTT offline';
  }catch(err){$('featureMessage').textContent=err.message;}
}
$('saveFeatures').addEventListener('click',async()=>{
  const data=new URLSearchParams({ha_discovery:$('haDiscovery').value});
  document.querySelectorAll('[data-capability]').forEach(el=>data.set(el.dataset.capability,el.value));
  const button=$('saveFeatures');button.disabled=true;
  try{
    await request('/api/features',{method:'POST',
      headers:{'Content-Type':'application/x-www-form-urlencoded','X-Requested-With':'XMLHttpRequest'},
      body:data.toString()});
    $('featureMessage').textContent=t('save_ok');
    toast(t('save_ok'));
  }catch(err){$('featureMessage').textContent=err.message;toast(t('save_fail'),'error');}
  finally{button.disabled=false;}
});
$('restoreFile').addEventListener('change',async e=>{
  const file=e.target.files?.[0];
  if(!file)return;
  if(file.size>3000){toast('Backup is too large','error');e.target.value='';return;}
  if(!confirm('Importovat nastavení? / Restore this configuration?')){e.target.value='';return;}
  try{
    const json=await file.text();
    await request('/api/restore',{method:'POST',
      headers:{'Content-Type':'application/json','X-Requested-With':'XMLHttpRequest'},body:json});
    toast('Konfigurace importována / Restored. Restart recommended.');
    loadedSettings=false;loadSettings();loadFeatures();
  }catch(err){toast(err.message,'error');}
  finally{e.target.value='';}
});
$('restartDevice').addEventListener('click',async()=>{
  if(!confirm(t('restart_confirm')))return;
  try{
    await request('/api/restart',{method:'POST',
      headers:{'Content-Type':'application/x-www-form-urlencoded','X-Requested-With':'XMLHttpRequest'},
      body:'confirm=YES'});
    toast(t('restart_sent'));updatePill('loading',t('restart_sent'));
  }catch(err){toast(err.message,'error');}
});

poll();setInterval(poll,3000);
})();
)JS";
