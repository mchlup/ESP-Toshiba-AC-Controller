#pragma once
#include <Arduino.h>

// ============ Embedded UI assets (modern responsive) ============
static const char APP_CSS[] PROGMEM = R"CSS(
:root{
  --bg: #0b1020; --fg:#e9eefb; --muted:#aab3c5; --card:#131a2e; --accent:#4f8cff;
  --ok:#26c281; --warn:#ffb020; --bad:#ff6b6b; --border:#223052;
  --gap: 12px; --radius: 14px; --pad: 14px; --shadow: 0 8px 24px rgba(0,0,0,.25);
  --font: system-ui,-apple-system,Segoe UI,Roboto,Ubuntu,Arial;
}
@media (prefers-color-scheme: light){
  :root{ --bg:#f5f7fb; --fg:#0f172a; --muted:#465066; --card:#ffffff; --accent:#2563eb; --border:#dbe1f1; }
}
*{box-sizing:border-box} html,body{height:100%}
body{margin:0;background:var(--bg);color:var(--fg);font-family:var(--font);-webkit-font-smoothing:antialiased}
.container{max-width:1024px;margin:0 auto;padding:16px}
.header{display:flex;gap:var(--gap);align-items:center;justify-content:space-between;margin-bottom:16px}
.hgroup{display:flex;gap:10px;align-items:center}
.badge{font-size:12px;padding:4px 8px;border-radius:999px;background:var(--card);border:1px solid var(--border);color:var(--muted)}
.brand{font-weight:700;font-size:18px;letter-spacing:.3px}
.grid{display:grid;gap:var(--gap);grid-template-columns:repeat(12,1fr)}
.card{background:var(--card);border:1px solid var(--border);border-radius:var(--radius);padding:var(--pad);box-shadow:var(--shadow)}
.card h3{margin:.1rem 0 .6rem 0;font-size:14px;color:var(--muted);font-weight:600}
.kv{display:grid;grid-template-columns:1fr auto;gap:8px;font-size:14px}
.kv span:nth-child(2){color:var(--fg);font-weight:600}
.status-dot{width:10px;height:10px;border-radius:50%;display:inline-block;vertical-align:middle;margin-right:6px}
.ok{background:var(--ok)} .bad{background:var(--bad)} .warn{background:var(--warn)}
.btn{appearance:none;border:1px solid var(--border);background:var(--card);color:var(--fg);padding:10px 12px;border-radius:12px;cursor:pointer;font-weight:600}
.btn:active{transform:translateY(1px)} .btn-group{display:flex;flex-wrap:wrap;gap:8px}
.btn.primary{background:var(--accent);border-color:transparent;color:#fff}
.segment{display:flex;background:var(--card);border:1px solid var(--border);border-radius:999px;overflow:hidden}
.segment button{flex:1;border:none;background:transparent;color:var(--fg);padding:10px 12px;font-weight:600;cursor:pointer}
.segment button.active{background:var(--accent);color:#fff}

/* >>> DOPLNĚNO: zvýraznění aktivních tlačítek i mimo .segment (Ventilátor / Lamel) */
.btn-group .btn.active, 
.btn.active{
  background: var(--accent);
  color: #fff;
  border-color: transparent;
  box-shadow: inset 0 0 0 1px rgba(255,255,255,.08);
}

.row{display:flex;gap:12px;align-items:center}
.value{font-size:22px;font-weight:800;letter-spacing:.3px}
.slider{width:100%}
/* --- form fields for config --- */
.form{display:grid;grid-template-columns:repeat(12,1fr);gap:12px}
.field{grid-column:span 6;display:flex;flex-direction:column;gap:6px}
.field.sm{grid-column:span 3}
.field.lg{grid-column:span 12}
label{font-size:12px;color:var(--muted);font-weight:600}
input[type=text],input[type=number],input[type=password]{
  width:100%;padding:10px;border-radius:10px;border:1px solid var(--border);
  background:var(--bg);color:var(--fg)
}
input[disabled]{opacity:.7}
.switchrow{display:flex;gap:10px;align-items:center}
.help{font-size:12px;color:var(--muted)}
small.muted{color:var(--muted)}
footer.sticky{
  position:sticky;bottom:0;z-index:5;background:linear-gradient(180deg, rgba(0,0,0,0), rgba(0,0,0,.2) 8px, var(--bg) 12px);
  padding-top:6px;margin-top:14px
}
.footerbar{display:flex;gap:var(--gap);align-items:center;padding:10px;border:1px solid var(--border);background:var(--card);
  border-radius:16px;box-shadow:var(--shadow);flex-wrap:wrap}
.footerbar .grow{flex:1;min-width:180px}
hr.sep{border:none;border-top:1px dashed var(--border);margin:10px 0}
a.link{color:var(--accent);text-decoration:none;font-weight:600}
code.tag{background:#00000020;border:1px solid var(--border);padding:2px 6px;border-radius:8px;font-family:ui-monospace,Consolas;font-size:12px}
@media (max-width:900px){ .grid {grid-template-columns:repeat(6,1fr)} }
@media (max-width:640px){ .grid {grid-template-columns:repeat(4,1fr)} .header{flex-wrap:wrap} }
)CSS";

static const char APP_JS[] PROGMEM = R"JS(
const $ = sel => document.querySelector(sel);

function setActive(group, value){
  if(!group) return;
  group.querySelectorAll('button').forEach(b=>{
    b.classList.toggle('active', b.dataset.val===value);
  });
}

async function apiState(){
  const r = await fetch('/api/state.json',{cache:'no-store'});
  if(!r.ok) throw new Error('state http '+r.status);
  return await r.json();
}

async function send(cmd){
  try{
    const r = await fetch('/command',{method:'POST',
      headers:{'Content-Type':'application/x-www-form-urlencoded','X-Requested-With':'XMLHttpRequest'},
      credentials:'same-origin',body:'cmd='+encodeURIComponent(cmd),cache:'no-store'});
    if(!r.ok) throw 0;
  }catch(e){}
}

function fmt(val, unit=''){ return (val==null?'—':val)+unit; }

async function refresh(){
  try{
    const s = await apiState();
    // headline
    const oper = $('#oper');
    if(oper) oper.textContent = s.oper || '—';
    const pwrDot = $('#pwrDot');
    if(pwrDot) pwrDot.className = 'status-dot '+((s.power==='on')?'ok':'bad');

    // temps
    const setVal = $('#setVal');
    const slider = $('#temp');
    if(typeof s.temp==='number'){
      if(setVal) setVal.textContent = s.temp;
      if(slider && !document.activeElement.isSameNode(slider)){ slider.value = s.temp; }
    }
    const tin = $('#tin'), tout = $('#tout');
    if(tin)  tin.textContent  = (s.t_in!=null)? (s.t_in.toFixed(1)+' °C') : '—';
    if(tout) tout.textContent = (s.t_out!=null)? (s.t_out.toFixed(1)+' °C') : '—';

    // segments
    setActive($('#modeGroup'), s.mode||'auto');
    setActive($('#fanGroup'),  s.fan||'auto');
    setActive($('#swingGroup'), s.swing||'fix');

    // badges
    const hvacBadge = $('#hvacBadge');
    if(hvacBadge) hvacBadge.innerHTML = `<span class="status-dot ${s.hvac==='Active'?'ok':'bad'}"></span>${s.hvac||'—'}`;
  }catch(e){}
}

function bind(){
  // Config UI
  loadCfgUI();
  const rld = $('#btnCfgReload'), sv = $('#btnCfgSave');
  if(rld) rld.onclick = loadCfgUI;
  if(sv)  sv.onclick  = saveCfgUI;

  // power
  const btnOn = $('#btnOn'), btnOff = $('#btnOff');
  if(btnOn)  btnOn.onclick  = ()=>send('STATE_ON');
  if(btnOff) btnOff.onclick = ()=>send('STATE_OFF');

  // temp
  const slid = $('#temp');
  const lbl  = $('#setVal');
  if(slid){
    slid.addEventListener('input', e=>{ if(lbl) lbl.textContent = e.target.value; });
    let tmr=null;
    slid.addEventListener('change', e=>{
      const v = Math.max(17, Math.min(30, parseInt(e.target.value||24,10)));
      send('TEMP_'+v);
      if(tmr) clearTimeout(tmr);
      tmr=setTimeout(refresh,300);
    });
  }

  // segments
  const mg = $('#modeGroup');
  if(mg) mg.querySelectorAll('button').forEach(b=>{
    b.onclick = ()=>{
      const map = { auto:'AUTO', cool:'COOL', heat:'HEAT', dry:'DRY', fan_only:'FAN' }; // <<< FAN => server alias
      const val = map[b.dataset.val] || b.dataset.val.toUpperCase();
      send('MODE_'+val);
    };
  });
  const fg = $('#fanGroup');
  if(fg) fg.querySelectorAll('button').forEach(b=>{
    b.onclick=()=>{
      const v=b.dataset.val;
      const map={low:'LOW',medium:'MEDIUM',high:'HIGH','lvl_1':'1','lvl_2':'2','lvl_3':'3','lvl_4':'4','lvl_5':'5'};
      send('FAN_'+(map[v]||v.toUpperCase()));
    }
  });
  const sg = $('#swingGroup');
  if(sg) sg.querySelectorAll('button').forEach(b=>{
    b.onclick=()=>{
      const v=b.dataset.val;
      const map={fix:'SWING_OFF',v_swing:'SWING_V',h_swing:'SWING_H',vh_swing:'SWING_VH',
                 fix_pos_1:'SWING_FIX_1',fix_pos_2:'SWING_FIX_2',fix_pos_3:'SWING_FIX_3',fix_pos_4:'SWING_FIX_4',fix_pos_5:'SWING_FIX_5'};
      send(map[v]||'SWING_OFF');
    }
  });

  // polling
  refresh();
  setInterval(refresh, 2000);
}

async function apiConfig(){
  const r = await fetch('/api/config.json',{cache:'no-store'});
  if(!r.ok) throw new Error('config http '+r.status);
  return await r.json();
}

function getVal(id){ const el=$(id); return el?el.value:''; }
function setVal(id,v){ const el=$(id); if(el!=null) el.value = (v==null?'':v); }

async function loadCfgUI(){
  try{
    const c = await apiConfig();
    setVal('#cfg_mqtt_en', c.mqtt_en); setVal('#cfg_server', c.server); setVal('#cfg_port', c.port);
    setVal('#cfg_user', c.user); setVal('#cfg_pass', c.pass);
    setVal('#cfg_cmdtopic', c.cmdtopic); setVal('#cfg_statetopic', c.statetopic); setVal('#cfg_hbtopic', c.hbtopic);
    setVal('#cfg_webuser', c.webuser); setVal('#cfg_webpass', c.webpass);
    setVal('#cfg_dev_id_en', c.dev_id_en); setVal('#cfg_dev_id', c.dev_id);
    setVal('#cfg_room', c.room);
    setVal('#cfg_name', c.name);
    setVal('#cfg_mb_en', c.mb_en); setVal('#cfg_mb_port', c.mb_port); setVal('#cfg_mb_uid', c.mb_uid);
    setVal('#cfg_ota_en', c.ota_en); setVal('#cfg_ota_pass', c.ota_pass);
    setVal('#cfg_telnet_en', c.telnet_en); setVal('#cfg_telnet_pass', c.telnet_pass);
    const msg = $('#cfgMsg'); if(msg) msg.textContent = '';
  }catch(e){
    const msg = $('#cfgMsg'); if(msg) msg.textContent = 'Nepodařilo se načíst konfiguraci.';
  }
}

async function saveCfgUI(){
  const q = new URLSearchParams({
    mqtt_en: getVal('#cfg_mqtt_en'),
    server:  getVal('#cfg_server'),
    port:    getVal('#cfg_port'),
    user:    getVal('#cfg_user'),
    pass:    getVal('#cfg_pass'),
    cmdtopic: getVal('#cfg_cmdtopic'),
    statetopic: getVal('#cfg_statetopic'),
    hbtopic: getVal('#cfg_hbtopic'),
    webuser: getVal('#cfg_webuser'),
    webpass: getVal('#cfg_webpass'),
    dev_id_en: getVal('#cfg_dev_id_en'),
    dev_id:    getVal('#cfg_dev_id'),
    room:    getVal('#cfg_room'),
    name:    getVal('#cfg_name'),
    mb_en: getVal('#cfg_mb_en'),
    ota_en:  getVal('#cfg_ota_en'),
    ota_pass:getVal('#cfg_ota_pass'),
    telnet_en:  getVal('#cfg_telnet_en'),
    telnet_pass:getVal('#cfg_telnet_pass'),
  });
  try{
    const r = await fetch('/mqttconfig', {method:'POST',
      headers:{'Content-Type':'application/x-www-form-urlencoded','X-Requested-With':'XMLHttpRequest'},
      credentials:'same-origin',body:q.toString(),cache:'no-store'});
    const msg = $('#cfgMsg');
    if(r.ok){
      if(msg) msg.textContent = 'Konfigurace uložena. Aplikuji…';
      setTimeout(()=>{ loadCfgUI(); refresh(); }, 500);
    }else{
      if(msg) msg.textContent = 'Chyba při ukládání ('+r.status+').';
    }
  }catch(e){
    const msg = $('#cfgMsg'); if(msg) msg.textContent = 'Chyba při ukládání.';
  }
}

document.addEventListener('DOMContentLoaded', bind);
)JS";

