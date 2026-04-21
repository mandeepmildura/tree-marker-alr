// ================================================================
//  Tree Row Marker — KinCony ALR (ESP32-S3)
//  Receives RTK position from AgOpenGPS via WiFi UDP
//  Fires on-board relay at each tree grid intersection
//
//  Repo:  https://github.com/mandeepMildura/tree-marker-alr
//  Board: KinCony ALR  https://www.kincony.com/esp32-lora-sx1278-gateway.html
//  IDE:   ESP32S3 Dev Module  (esp32 by Espressif, 2.x)
// ================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPUpdate.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Update.h>
#include <HTTPClient.h>
#include <DNSServer.h>

// ── Firmware version (bumped on each release) ─────────────────
#define FW_VERSION "1.3.2"

// ================================================================
//  COMPILED-IN DEFAULTS — overridden by Preferences after first save
// ================================================================
#define DEF_WIFI_SSID     "Mandeep"
#define DEF_WIFI_PASS     "rockyjungle161"
#define DEF_MQTT_HOST     "eb65c13ec8ab480a9c8492778fdddda8.s1.eu.hivemq.cloud"
#define DEF_MQTT_PORT     8883
#define DEF_MQTT_USER     "mandeep"
#define DEF_MQTT_PASS     "Gill@1977"
#define DEF_MQTT_CLIENT   "tree-marker-alr-01"
#define DEF_ORIGIN_LAT    -34.3166591
#define DEF_ORIGIN_LON    142.1562539
#define DEF_ROW_BEARING   180.0
#define DEF_ROW_SPACING   6.7
#define DEF_TREE_SPACING  3.0
#define DEF_NUM_ROWS      10
#define DEF_NUM_TREES     50
#define DEF_HIT_RADIUS    0.35
#define DEF_RELAY_PULSE   600

// ── AgIO UDP ──────────────────────────────────────────────────
#define UDP_PORT  8888

// ── MQTT topics ───────────────────────────────────────────────
#define T_STATUS      "treemarker/status"
#define T_HIT         "treemarker/hit"
#define T_OTA_URL     "treemarker/ota/url"
#define T_OTA_STATUS  "treemarker/ota/status"
#define T_RESTART     "treemarker/restart"

// ── Pin map ───────────────────────────────────────────────────
#define RELAY_PIN         48
#define OLED_SDA          39
#define OLED_SCL          38
#define SCREEN_W         128
#define SCREEN_H          64
#define SETUP_BUTTON_PIN   0       // DL button — hold at boot to force AP mode

// ── AP / setup mode ──────────────────────────────────────────
#define AP_SSID         "TreeMarker-Setup"
#define STA_TIMEOUT_MS  20000

// ── GitHub release API ───────────────────────────────────────
#define GH_LATEST_URL \
  "https://api.github.com/repos/mandeepMildura/tree-marker-alr/releases/latest"

// ================================================================
//  MUTABLE CONFIG (loaded from Preferences at boot)
// ================================================================
char   gWifiSsid[64];
char   gWifiPass[64];
char   gMqttHost[128];
int    gMqttPort;
char   gMqttUser[64];
char   gMqttPass[64];
double gOriginLat;
double gOriginLon;
double gRowBearing;
double gRowSpacing;
double gTreeSpacing;
int    gNumRows;
int    gNumTrees;
double gHitRadius;
int    gRelayPulse;
int    gUdpPort;

Preferences prefs;

void loadPrefs() {
  prefs.begin("tm", true);
  strlcpy(gWifiSsid,  prefs.getString("wifi_ssid",  DEF_WIFI_SSID).c_str(),  sizeof(gWifiSsid));
  strlcpy(gWifiPass,  prefs.getString("wifi_pass",  DEF_WIFI_PASS).c_str(),  sizeof(gWifiPass));
  strlcpy(gMqttHost,  prefs.getString("mqtt_host",  DEF_MQTT_HOST).c_str(),  sizeof(gMqttHost));
  gMqttPort    = prefs.getInt(   "mqtt_port",  DEF_MQTT_PORT);
  strlcpy(gMqttUser,  prefs.getString("mqtt_user",  DEF_MQTT_USER).c_str(),  sizeof(gMqttUser));
  strlcpy(gMqttPass,  prefs.getString("mqtt_pass",  DEF_MQTT_PASS).c_str(),  sizeof(gMqttPass));
  gOriginLat   = prefs.getDouble("origin_lat",  DEF_ORIGIN_LAT);
  gOriginLon   = prefs.getDouble("origin_lon",  DEF_ORIGIN_LON);
  gRowBearing  = prefs.getDouble("row_bearing", DEF_ROW_BEARING);
  gRowSpacing  = prefs.getDouble("row_spacing", DEF_ROW_SPACING);
  gTreeSpacing = prefs.getDouble("tree_space",  DEF_TREE_SPACING);
  gNumRows     = prefs.getInt(   "num_rows",    DEF_NUM_ROWS);
  gNumTrees    = prefs.getInt(   "num_trees",   DEF_NUM_TREES);
  gHitRadius   = prefs.getDouble("hit_radius",  DEF_HIT_RADIUS);
  gRelayPulse  = prefs.getInt(   "relay_pulse", DEF_RELAY_PULSE);
  gUdpPort     = prefs.getInt(   "udp_port",    UDP_PORT);
  prefs.end();
}

void saveGridPrefs() {
  prefs.begin("tm", false);
  prefs.putDouble("origin_lat",  gOriginLat);
  prefs.putDouble("origin_lon",  gOriginLon);
  prefs.putDouble("row_bearing", gRowBearing);
  prefs.putDouble("row_spacing", gRowSpacing);
  prefs.putDouble("tree_space",  gTreeSpacing);
  prefs.putInt(   "num_rows",    gNumRows);
  prefs.putInt(   "num_trees",   gNumTrees);
  prefs.putDouble("hit_radius",  gHitRadius);
  prefs.putInt(   "relay_pulse", gRelayPulse);
  prefs.end();
}

void saveWifiPrefs() {
  prefs.begin("tm", false);
  prefs.putString("wifi_ssid", gWifiSsid);
  prefs.putString("wifi_pass", gWifiPass);
  prefs.putInt(   "udp_port",  gUdpPort);
  prefs.end();
}

// ── Objects ───────────────────────────────────────────────────
WiFiUDP           udp;
WiFiClientSecure  tlsClient;
PubSubClient      mqtt(tlsClient);
Adafruit_SSD1306  oled(SCREEN_W, SCREEN_H, &Wire, -1);
WebServer         webServer(80);
DNSServer         dnsServer;
bool              apMode = false;
bool              otaRebootPending = false;
unsigned long     otaRebootAt = 0;

// ── Grid ──────────────────────────────────────────────────────
const double M_PER_DEG_LAT = 111320.0;
double mPerDegLon;
double bearingRad, perpBearingRad;

struct Point { double lat; double lon; };
Point grid[20][100];   // [row][tree]  max 20x100

// ── State ─────────────────────────────────────────────────────
bool   relayActive    = false;
unsigned long relayStart    = 0;
unsigned long lastHit       = 0;
int    lastRow   = -1, lastTree  = -1;
int    insideRow = -1, insideTree = -1;   // tree we're currently inside (debounce)
double lastDist  =  0;
double curLat    =  0, curLon   =  0;
bool   gpsFix    = false;
unsigned long lastFixMs     = 0;
int    totalHits            = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastMachineSend = 0;
bool   otaPending   = false;
String otaUrl       = "";

// ================================================================
//  WEB PAGE (served from ESP32 at http://board-ip/)
// ================================================================
const char PAGE[] PROGMEM = R"rawpage(<!DOCTYPE html>
<html lang=en>
<head>
<meta charset=UTF-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>Tree Marker ALR — Sunraysia Acres</title>
<link href="https://fonts.googleapis.com/css2?family=Space+Grotesk:wght@400;700&family=Manrope:wght@400;500;600;700&display=swap" rel=stylesheet>
<style>
:root{--bg:#fcf9f4;--sl:#f6f3ee;--sc:#f0ede8;--sh:#ebe8e3;--sw:#e5e2dd;--wh:#ffffff;--pr:#006e2f;--pb:#22c55e;--tx:#1c1c19;--tv:#3d4a3d;--mu:#6d7b6c;--ol:#bccbb9;--er:#ba1a1a}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Manrope',-apple-system,sans-serif;background:var(--bg);color:var(--tx);min-height:100vh}
.hl{font-family:'Space Grotesk',sans-serif}
header{background:rgba(252,249,244,.92);backdrop-filter:blur(12px);position:sticky;top:0;z-index:50;border-bottom:1px solid var(--ol)}
.hi{max-width:920px;margin:auto;display:flex;align-items:center;gap:12px;height:56px;padding:0 20px}
.brand{font-family:'Space Grotesk',sans-serif;font-size:15px;font-weight:700;letter-spacing:-.3px;line-height:1.2}
.brand small{display:block;font-size:10px;font-weight:400;color:var(--mu);letter-spacing:.5px;text-transform:uppercase}
.pill{display:flex;align-items:center;gap:5px;padding:3px 10px;border-radius:99px;background:#dcfce7;border:1px solid #86efac;font-size:11px;font-weight:700;color:var(--pr);transition:all .3s}
.pill.off{background:#fee2e2;border-color:#fca5a5;color:var(--er)}
.sdot{width:7px;height:7px;border-radius:50%;background:var(--pb);flex-shrink:0}
.pill.off .sdot{background:var(--er)}
nav{display:flex;gap:2px;margin-left:12px}
.nb{background:none;border:none;cursor:pointer;font-family:'Space Grotesk',sans-serif;font-size:11px;font-weight:700;text-transform:uppercase;letter-spacing:.8px;color:var(--mu);padding:6px 12px;border-radius:6px;transition:all .2s}
.nb.active{background:var(--sh);color:var(--pr)}
.nb:hover:not(.active){background:var(--sl)}
.fw{margin-left:auto;font-size:11px;color:var(--mu);font-family:'Space Grotesk',sans-serif}
main{max-width:920px;margin:0 auto;padding:22px 20px 48px}
.sec{display:none}.sec.active{display:block}
.sech{margin-bottom:18px}
.sech h2{font-size:26px;font-weight:700;letter-spacing:-.5px}
.slbl{font-size:10px;font-weight:700;text-transform:uppercase;letter-spacing:1.2px;color:var(--pr);margin-bottom:3px}
.g2{display:grid;grid-template-columns:1fr 1fr;gap:14px}
.g4{display:grid;grid-template-columns:repeat(4,1fr);gap:10px}
@media(max-width:600px){.g2,.g4{grid-template-columns:1fr 1fr}}
.card{background:var(--wh);border-radius:12px;padding:18px;border:1px solid var(--ol);margin-bottom:14px}
.clbl{font-size:10px;font-weight:700;text-transform:uppercase;letter-spacing:1px;color:var(--mu);margin-bottom:12px;display:flex;align-items:center;gap:6px}
.clbl::before{content:'';display:inline-block;width:3px;height:13px;background:var(--pb);border-radius:2px}
.sr{display:flex;align-items:center;justify-content:space-between;padding:7px 0;border-bottom:1px solid var(--sl);font-size:13px}
.sr:last-child{border:none;padding-bottom:0}
.sk{color:var(--tv);font-weight:500}
.sv{font-weight:700;font-family:'Space Grotesk',sans-serif}
.sv.ok{color:var(--pr)}.sv.bad{color:var(--er)}.sv.bl{color:#0369a1}
.bs{background:var(--sl);border-radius:8px;padding:12px 14px}
.bs .lbl{font-size:10px;font-weight:700;text-transform:uppercase;letter-spacing:.8px;color:var(--mu)}
.bs .num{font-family:'Space Grotesk',sans-serif;font-size:30px;font-weight:800;line-height:1.1}
.bs .num.gr{color:var(--pr)}
.dvd{height:1px;background:var(--ol);margin:14px 0}
.btn{display:flex;align-items:center;justify-content:space-between;width:100%;padding:13px 16px;border-radius:8px;border:none;cursor:pointer;font-family:'Space Grotesk',sans-serif;font-size:13px;font-weight:700;letter-spacing:.2px;transition:all .15s;margin-bottom:8px}
.btn:last-child{margin-bottom:0}
.btn:active{transform:scale(.98)}
.btn-p{background:var(--pr);color:#fff}.btn-p:hover{background:#005321}
.btn-g{background:#dcfce7;color:var(--pr);border:1px solid #86efac}.btn-g:hover{background:#bbf7d0}
.btn-w{background:#735a42;color:#fff}.btn-w:hover{background:#5c4534}
.bi{font-size:17px}
.fg{margin-bottom:12px}
.fg label{display:block;font-size:10px;font-weight:700;text-transform:uppercase;letter-spacing:.8px;color:var(--mu);margin-bottom:5px}
.fg input{width:100%;background:var(--sl);border:1px solid transparent;border-bottom-color:var(--ol);border-radius:6px 6px 0 0;padding:9px 12px;font-family:'Manrope',sans-serif;font-size:13px;font-weight:600;color:var(--tx);outline:none;transition:all .2s}
.fg input:focus{background:var(--wh);border-bottom-color:var(--pr);border-color:var(--ol)}
.note{font-size:11px;color:var(--mu);margin-top:8px}
</style>
</head>
<body>
<header>
<div class=hi>
  <div class=brand>Tree Marker ALR<small>Sunraysia Acres</small></div>
  <div class="pill off" id=spill><div class=sdot></div><span id=slbl>Offline</span></div>
  <nav>
    <button class="nb active" onclick="showTab('status',this)">Status</button>
    <button class=nb onclick="showTab('config',this)">Config</button>
  </nav>
  <div class=fw id=hfw></div>
</div>
</header>
<main>

<!-- STATUS -->
<div id=t-status class="sec active">
  <div class=sech>
    <div class=slbl>Live Telemetry</div>
    <h2 class=hl>System Status</h2>
  </div>
  <div class=g2>
    <div class=card>
      <div class=clbl>System</div>
      <div class=sr><span class=sk>WiFi</span><span class="sv ok" id=s-wifi>—</span></div>
      <div class=sr><span class=sk>IP Address</span><span class="sv bl" id=s-ip>—</span></div>
      <div class=sr><span class=sk>MQTT</span><span class="sv ok" id=s-mqtt>—</span></div>
      <div class=sr><span class=sk>GPS Fix</span><span class="sv bad" id=s-gps>—</span></div>
      <div class=sr><span class=sk>Position</span><span class="sv bl" id=s-pos style="font-size:11px">—</span></div>
      <div class=sr><span class=sk>Uptime</span><span class=sv id=s-up>—</span></div>
      <div class=sr><span class=sk>Firmware</span><span class=sv id=s-fw>—</span></div>
    </div>
    <div class=card>
      <div class=clbl>Actions</div>
      <button class="btn btn-g" onclick=testRelay()><span>Fire Relay (wiring test)</span><span class=bi>&#9889;</span></button>
      <div class=dvd></div>
      <div class=clbl>Active Grid</div>
      <div class=sr><span class=sk>Origin</span><span class="sv bl" id=c-origin style="font-size:11px">—</span></div>
      <div class=sr><span class=sk>Bearing</span><span class=sv id=c-brg>—</span></div>
      <div class=sr><span class=sk>Row / Tree spacing</span><span class=sv id=c-spacing>—</span></div>
      <div class=sr><span class=sk>Grid size</span><span class=sv id=c-grid>—</span></div>
      <div class=sr><span class=sk>Hit radius</span><span class=sv id=c-hr>—</span></div>
      <div class=sr><span class=sk>Relay pulse</span><span class=sv id=c-rp>—</span></div>
    </div>
  </div>
  <div class=card>
    <div class=clbl>Session Statistics</div>
    <div class=g4>
      <div class=bs><div class=lbl>Hits</div><div class="num gr" id=s-hits>0</div></div>
      <div class=bs><div class=lbl>Last Row</div><div class=num id=s-lrow>—</div></div>
      <div class=bs><div class=lbl>Last Tree</div><div class=num id=s-ltree>—</div></div>
      <div class=bs><div class=lbl>Last Dist</div><div class=num id=s-ldist style="font-size:20px">—</div></div>
    </div>
  </div>
</div>

<!-- CONFIG -->
<div id=t-config class=sec>
  <div class=sech>
    <div class=slbl>Device Configuration</div>
    <h2 class=hl>Grid Parameters</h2>
  </div>
  <div class=card>
    <div class=clbl>Field Grid</div>
    <div class=g2>
      <div class=fg><label>Origin Latitude</label><input id=f-lat type=number step=0.0000001></div>
      <div class=fg><label>Origin Longitude</label><input id=f-lon type=number step=0.0000001></div>
      <div class=fg><label>Row Bearing (&#176;) &mdash; 0=N 90=E 180=S</label><input id=f-brg type=number step=0.1></div>
      <div class=fg><label>Row Spacing (m)</label><input id=f-rs type=number step=0.1></div>
      <div class=fg><label>Tree Spacing (m)</label><input id=f-ts type=number step=0.1></div>
      <div class=fg><label>Hit Radius (m)</label><input id=f-hr type=number step=0.01></div>
      <div class=fg><label>Number of Rows (max 20)</label><input id=f-nr type=number step=1 min=1 max=20></div>
      <div class=fg><label>Trees per Row (max 100)</label><input id=f-nt type=number step=1 min=1 max=100></div>
      <div class=fg style="grid-column:1/-1"><label>Relay Pulse (ms)</label><input id=f-rp type=number step=50></div>
    </div>
    <button class="btn btn-p" onclick=saveGrid() style="margin-top:6px"><span>Save Grid + Apply Now</span><span class=bi>&#10003;</span></button>
    <p class=note>Grid rebuilds immediately &mdash; no reboot needed.</p>
  </div>
  <div class=card>
    <div class=clbl>Network (requires reboot)</div>
    <div class=g2>
      <div class=fg><label>Network Name (SSID)</label><input id=f-ssid type=text></div>
      <div class=fg><label>Password</label><input id=f-wpass type=password placeholder="(leave blank to keep current)"></div>
      <div class=fg><label>AgIO UDP Port</label><input id=f-udp type=number step=1 min=1 max=65535></div>
    </div>
    <button class="btn btn-w" onclick=saveWifi()><span>Save Network + Reboot</span><span class=bi>&#8635;</span></button>
    <p class=note>Board will reconnect after reboot. Default AgIO port is 8888.</p>
  </div>
  <div class=card>
    <div class=clbl>Firmware Update (OTA)</div>
    <button class="btn btn-g" onclick=checkLatest() style="margin-bottom:10px">
      <span id=lat-btn>Check Latest Release on GitHub</span><span class=bi>&#128269;</span></button>
    <div class=fg><label>Firmware .bin URL (from GitHub Releases)</label><input id=f-ota type=text placeholder="https://github.com/.../releases/download/vX.Y.Z/tree_marker.ino.bin"></div>
    <button class="btn btn-p" onclick=flashOta()><span>Flash From URL</span><span class=bi>&#8659;</span></button>
    <div class=dvd></div>
    <div class=fg><label>Or upload a .bin file from this computer</label>
      <input id=f-file type=file accept=".bin"></div>
    <button class="btn btn-w" onclick=uploadBin()><span>Upload &amp; Flash</span><span class=bi>&#128228;</span></button>
    <p class=note id=ota-status>Board will reboot automatically after flashing.</p>
  </div>
</div>

</main>
<script>
var showTab=(t,btn)=>{
  document.querySelectorAll('.sec').forEach(e=>e.classList.remove('active'));
  document.querySelectorAll('.nb').forEach(e=>e.classList.remove('active'));
  document.getElementById('t-'+t).classList.add('active');
  btn.classList.add('active');
};
var uptime=(s)=>Math.floor(s/3600)+'h '+Math.floor((s%3600)/60)+'m '+s%60+'s';
var setv=(id,txt,cls)=>{const e=document.getElementById(id);if(e){e.textContent=txt;if(cls!==undefined)e.className='sv '+(cls?'ok':'bad');}};
var poll=()=>{
  fetch('/api/status').then(r=>r.json()).then(d=>{
    const ok=d.wifi,mo=d.mqtt,go=d.fix;
    const p=document.getElementById('spill');
    p.className='pill'+(ok?'':' off');
    document.getElementById('slbl').textContent=ok?'Online':'Offline';
    setv('s-wifi',ok?'Connected':'Disconnected',ok);
    setv('s-ip',d.ip,true);
    setv('s-mqtt',mo?'Connected':'Disconnected',mo);
    setv('s-gps',go?'RTK Fix':'No Fix',go);
    document.getElementById('s-pos').textContent=go?d.lat.toFixed(7)+', '+d.lon.toFixed(7):'-';
    document.getElementById('s-up').textContent=uptime(d.uptime);
    document.getElementById('s-fw').textContent='v'+d.fw;
    document.getElementById('hfw').textContent='fw v'+d.fw;
    document.getElementById('s-hits').textContent=d.hits;
    document.getElementById('s-lrow').textContent=d.lastRow>=0?d.lastRow:'-';
    document.getElementById('s-ltree').textContent=d.lastTree>=0?d.lastTree:'-';
    document.getElementById('s-ldist').textContent=d.lastDist>0?d.lastDist.toFixed(3)+'m':'-';
    const c=d.cfg;
    document.getElementById('c-origin').textContent=c.lat.toFixed(7)+', '+c.lon.toFixed(7);
    document.getElementById('c-brg').textContent=c.brg+' deg';
    document.getElementById('c-spacing').textContent=c.rs+'m / '+c.ts+'m';
    document.getElementById('c-grid').textContent=c.rows+' x '+c.trees;
    document.getElementById('c-hr').textContent=c.hr+' m';
    document.getElementById('c-rp').textContent=c.rp+' ms';
    document.getElementById('f-lat').value=c.lat;
    document.getElementById('f-lon').value=c.lon;
    document.getElementById('f-brg').value=c.brg;
    document.getElementById('f-rs').value=c.rs;
    document.getElementById('f-ts').value=c.ts;
    document.getElementById('f-nr').value=c.rows;
    document.getElementById('f-nt').value=c.trees;
    document.getElementById('f-hr').value=c.hr;
    document.getElementById('f-rp').value=c.rp;
    document.getElementById('f-ssid').value=c.ssid;
    document.getElementById('f-udp').value=c.udp;
  }).catch(()=>{});
};
var testRelay=()=>{fetch('/relay',{method:'POST'}).then(()=>alert('Relay fired!'));};
var saveGrid=()=>{
  const b={lat:+document.getElementById('f-lat').value,lon:+document.getElementById('f-lon').value,
    brg:+document.getElementById('f-brg').value,rs:+document.getElementById('f-rs').value,
    ts:+document.getElementById('f-ts').value,nr:+document.getElementById('f-nr').value,
    nt:+document.getElementById('f-nt').value,hr:+document.getElementById('f-hr').value,
    rp:+document.getElementById('f-rp').value};
  fetch('/config/grid',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)})
    .then(r=>r.text()).then(t=>{alert(t);poll();});
};
var saveWifi=()=>{
  const ssid=document.getElementById('f-ssid').value;
  const pass=document.getElementById('f-wpass').value;
  const udp=+document.getElementById('f-udp').value;
  if(!ssid){alert('SSID cannot be empty');return;}
  if(!confirm('Save network settings and reboot?'))return;
  fetch('/config/wifi',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({ssid:ssid,pass:pass,udp:udp})}).then(r=>r.text()).then(t=>alert(t));
};
var flashOta=()=>{
  const url=document.getElementById('f-ota').value.trim();
  if(!url){alert('Paste a .bin URL first');return;}
  if(!confirm('Flash firmware from:\n'+url+'\n\nBoard will reboot after flashing.'))return;
  const st=document.getElementById('ota-status');
  st.textContent='Sending to board...';
  fetch('/ota',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({url:url})})
    .then(r=>r.text()).then(t=>{st.textContent=t;})
    .catch(e=>{st.textContent='Error: '+e;});
};
var checkLatest=()=>{
  const b=document.getElementById('lat-btn'),st=document.getElementById('ota-status');
  b.textContent='Checking GitHub...';
  fetch('/ota/latest').then(r=>r.json()).then(d=>{
    if(d.error){b.textContent='Check Latest Release on GitHub';st.textContent='Error: '+d.error;return;}
    b.textContent='Latest: '+d.tag+' (current v'+d.current+')';
    if(d.url)document.getElementById('f-ota').value=d.url;
    st.textContent=d.tag&&('v'+d.current)!==d.tag?'Newer release available. Click Flash From URL.':'You are on the latest release.';
  }).catch(e=>{b.textContent='Check Latest Release on GitHub';st.textContent='Error: '+e;});
};
var uploadBin=()=>{
  const f=document.getElementById('f-file').files[0];
  if(!f){alert('Pick a .bin file first');return;}
  if(!confirm('Upload and flash '+f.name+' ('+Math.round(f.size/1024)+' KB)?\nBoard will reboot.'))return;
  const st=document.getElementById('ota-status');
  st.textContent='Uploading '+f.name+'...';
  const fd=new FormData();fd.append('firmware',f,f.name);
  fetch('/ota/upload',{method:'POST',body:fd})
    .then(r=>r.text()).then(t=>{st.textContent=t;})
    .catch(e=>{st.textContent='Upload error: '+e;});
};
poll();setInterval(poll,2000);
</script>
</body></html>)rawpage";

// ================================================================
//  AP SETUP PAGE — served when board can't reach saved WiFi
//  or when DL button held at boot. Minimal, no external fonts.
// ================================================================
const char SETUP_PAGE[] PROGMEM = R"rawpage(<!DOCTYPE html>
<html lang=en>
<head>
<meta charset=UTF-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>Tree Marker — WiFi Setup</title>
<style>
*{box-sizing:border-box;margin:0;padding:0;font-family:-apple-system,Segoe UI,sans-serif}
body{background:#fcf9f4;color:#1c1c19;padding:20px;min-height:100vh}
.wrap{max-width:460px;margin:0 auto}
h1{font-size:22px;margin-bottom:4px}
.sub{color:#6d7b6c;font-size:13px;margin-bottom:20px}
.card{background:#fff;border:1px solid #bccbb9;border-radius:12px;padding:18px;margin-bottom:14px}
.lbl{font-size:10px;font-weight:700;text-transform:uppercase;letter-spacing:1px;color:#6d7b6c;margin-bottom:10px}
label{display:block;font-size:11px;font-weight:700;text-transform:uppercase;color:#6d7b6c;margin:10px 0 4px}
input,select{width:100%;padding:10px 12px;border:1px solid #bccbb9;border-radius:6px;font-size:14px;background:#f6f3ee}
input:focus,select:focus{outline:none;border-color:#006e2f;background:#fff}
button{width:100%;padding:13px;border:none;border-radius:8px;background:#006e2f;color:#fff;font-size:14px;font-weight:700;cursor:pointer;margin-top:14px}
button:hover{background:#005321}
button.sec{background:#735a42}
button.sec:hover{background:#5c4534}
.note{font-size:11px;color:#6d7b6c;margin-top:10px;line-height:1.5}
.ssid-row{display:flex;justify-content:space-between;padding:8px 10px;border-bottom:1px solid #f0ede8;font-size:13px;cursor:pointer;border-radius:4px}
.ssid-row:hover{background:#f6f3ee}
.ssid-row .rssi{color:#6d7b6c;font-size:11px}
#scan-list{max-height:220px;overflow-y:auto;margin-top:6px}
</style></head><body><div class=wrap>
<h1>Tree Marker Setup</h1>
<p class=sub>Board can't reach saved WiFi. Pick a network and enter its password.</p>
<div class=card>
  <div class=lbl>Available Networks</div>
  <div id=scan-list>Scanning...</div>
  <button class=sec onclick=rescan()>Rescan</button>
</div>
<div class=card>
  <div class=lbl>Connect</div>
  <label>Network (SSID)</label><input id=ssid placeholder="Pick above or type">
  <label>Password</label><input id=pass type=password placeholder="WiFi password">
  <label>AgIO UDP Port</label><input id=udp type=number value=8888>
  <button onclick=save()>Save &amp; Reboot</button>
  <p class=note>Board will reboot and try to connect. If it fails again the setup AP comes back up.</p>
</div>
</div>
<script>
var scan=()=>{document.getElementById('scan-list').textContent='Scanning...';
  fetch('/scan').then(r=>r.json()).then(d=>{
    var el=document.getElementById('scan-list');el.innerHTML='';
    if(!d.length){el.textContent='No networks found.';return;}
    d.forEach(n=>{
      var row=document.createElement('div');row.className='ssid-row';
      row.innerHTML='<span>'+n.ssid+(n.sec?' &#128274;':'')+'</span><span class=rssi>'+n.rssi+' dBm</span>';
      row.onclick=()=>{document.getElementById('ssid').value=n.ssid;document.getElementById('pass').focus();};
      el.appendChild(row);
    });
  }).catch(()=>{document.getElementById('scan-list').textContent='Scan failed.';});};
var rescan=scan;
var save=()=>{
  var s=document.getElementById('ssid').value.trim();
  var p=document.getElementById('pass').value;
  var u=+document.getElementById('udp').value||8888;
  if(!s){alert('Pick or type an SSID');return;}
  fetch('/config/wifi',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({ssid:s,pass:p,udp:u})})
    .then(r=>r.text()).then(t=>alert(t));
};
scan();
</script></body></html>)rawpage";

// ── AgOpenGPS PGN 100 (0x64) — NAV position from AOG ─────────
// AOG broadcasts this 30-byte packet to modules in both real-RTK
// and simulator mode. Payload is 24 bytes:
//   [0..7]   longitude  (double, little-endian)
//   [8..15]  latitude   (double, little-endian)
//   [16..23] heading / speed / flags (not used here)
// Header: 0x80 0x81 <src=0x7F> 0x64 0x18 <payload..> <CRC>
bool parsePGN100(uint8_t* buf, int len, double& lat, double& lon) {
  if (len < 30) return false;
  if (buf[0] != 0x80 || buf[1] != 0x81) return false;
  if (buf[3] != 0x64) return false;     // PGN 100
  if (buf[4] != 0x18) return false;     // 24-byte payload
  memcpy(&lon, &buf[5],  8);
  memcpy(&lat, &buf[13], 8);
  if (lat < -90.0 || lat > 90.0)   return false;
  if (lon < -180.0 || lon > 180.0) return false;
  if (lat == 0.0 && lon == 0.0)    return false;
  return true;
}

// ── NMEA GGA parser ───────────────────────────────────────────
bool parseGGA(const char* s, double& lat, double& lon) {
  if (strncmp(s, "$GPGGA", 6) != 0 && strncmp(s, "$GNGGA", 6) != 0) return false;

  char buf[128];
  strncpy(buf, s, 127); buf[127] = '\0';

  char* f[16]; int n = 0;
  f[n++] = buf;
  for (char* p = buf; *p && n < 16; p++)
    if (*p == ',') { *p = '\0'; f[n++] = p + 1; }
  if (n < 7) return false;
  if (f[6][0] == '0' || f[6][0] == '\0') return false;
  if (strlen(f[2]) < 4 || strlen(f[4]) < 5) return false;

  char d2[3] = { f[2][0], f[2][1], '\0' };
  lat = atof(d2) + atof(f[2] + 2) / 60.0;
  if (f[3][0] == 'S') lat = -lat;

  char d3[4] = { f[4][0], f[4][1], f[4][2], '\0' };
  lon = atof(d3) + atof(f[4] + 3) / 60.0;
  if (f[5][0] == 'W') lon = -lon;

  return true;
}

// ── Flat-earth distance ────────────────────────────────────────
double distM(double lat1, double lon1, double lat2, double lon2) {
  double dLat = (lat2 - lat1) * M_PER_DEG_LAT;
  double dLon = (lon2 - lon1) * mPerDegLon;
  return sqrt(dLat * dLat + dLon * dLon);
}

// ── Grid hit check ────────────────────────────────────────────
// Fires once per tree *entry*: the tractor must exit a tree's hit
// radius before it can fire the same tree again. Prevents a stopped
// tractor from re-triggering the relay every pulse+200ms.
void checkGrid(double lat, double lon) {
  if (relayActive) return;

  // Find the nearest tree within hit radius (if any).
  int nearR = -1, nearT = -1;
  double nearD = 1e9;
  for (int r = 0; r < gNumRows; r++) {
    for (int t = 0; t < gNumTrees; t++) {
      double d = distM(lat, lon, grid[r][t].lat, grid[r][t].lon);
      if (d < gHitRadius && d < nearD) {
        nearD = d; nearR = r; nearT = t;
      }
    }
  }

  // Outside every tree: clear inside-tracker so re-entry can fire again.
  if (nearR < 0) {
    insideRow = -1; insideTree = -1;
    return;
  }
  // Same tree as last fire: suppress until we leave its radius.
  if (nearR == insideRow && nearT == insideTree) return;

  // New tree entered — fire the relay.
  insideRow = nearR; insideTree = nearT;
  digitalWrite(RELAY_PIN, HIGH);
  relayActive = true;
  relayStart  = millis();
  lastHit     = millis();
  lastRow     = nearR;
  lastTree    = nearT;
  lastDist    = nearD;
  totalHits++;

  Serial.printf("HIT  Row %-2d  Tree %-3d  %.3fm\n", nearR, nearT, nearD);

  char payload[80];
  snprintf(payload, sizeof(payload),
    "{\"row\":%d,\"tree\":%d,\"dist\":%.3f,\"lat\":%.7f,\"lon\":%.7f}",
    nearR, nearT, nearD, lat, lon);
  mqtt.publish(T_HIT, payload, true);
}

// ── OLED ──────────────────────────────────────────────────────
void updateOLED() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);

  if (WiFi.status() != WL_CONNECTED) {
    oled.println("WiFi...");
  } else if (!gpsFix) {
    oled.println(WiFi.localIP().toString());
    oled.println(mqtt.connected() ? "MQTT OK" : "MQTT...");
    oled.println("Waiting for AgIO");
  } else {
    oled.printf("%.6f\n", curLat);
    oled.printf("%.6f\n", curLon);
    oled.println(mqtt.connected() ? "MQTT OK" : "MQTT--");
  }

  if (lastRow >= 0) {
    oled.printf("Last R%d T%d  #%d\n", lastRow, lastTree, totalHits);
  }

  if (relayActive) {
    oled.setTextSize(2);
    oled.setCursor(16, 48);
    oled.print("MARK!");
  }

  if (otaPending) {
    oled.setTextSize(1);
    oled.setCursor(0, 56);
    oled.print("OTA updating...");
  }

  oled.display();
}

// ── Build tree grid ───────────────────────────────────────────
void buildGrid() {
  mPerDegLon     = M_PER_DEG_LAT * cos(gOriginLat * M_PI / 180.0);
  bearingRad     = gRowBearing   * M_PI / 180.0;
  perpBearingRad = bearingRad    + M_PI / 2.0;

  for (int r = 0; r < gNumRows; r++) {
    for (int t = 0; t < gNumTrees; t++) {
      double along  = t * gTreeSpacing;
      double across = r * gRowSpacing;
      grid[r][t].lat = gOriginLat +
        (along * cos(bearingRad) + across * cos(perpBearingRad)) / M_PER_DEG_LAT;
      grid[r][t].lon = gOriginLon +
        (along * sin(bearingRad) + across * sin(perpBearingRad)) / mPerDegLon;
    }
  }
  Serial.printf("Grid: %d rows x %d trees  R=%.2fm  T=%.2fm  HIT=%.2fm\n",
    gNumRows, gNumTrees, gRowSpacing, gTreeSpacing, gHitRadius);
}

// ── PGN 239 — section state to AgOpenGPS ─────────────────────
void sendMachinePGN() {
  uint8_t sections = relayActive ? 0x01 : 0x00;
  uint8_t pkt[12];
  pkt[0]=0x80; pkt[1]=0x81; pkt[2]=0x7E; pkt[3]=239; pkt[4]=8;
  pkt[5]=sections; pkt[6]=0; pkt[7]=0; pkt[8]=0; pkt[9]=0; pkt[10]=0;
  uint8_t crc=0; for(int i=2;i<=10;i++) crc^=pkt[i]; pkt[11]=crc;

  IPAddress bcast = WiFi.localIP();
  IPAddress mask  = WiFi.subnetMask();
  for (int i = 0; i < 4; i++) bcast[i] |= ~mask[i];
  udp.beginPacket(bcast, 8888);
  udp.write(pkt, 12);
  udp.endPacket();
}

// ── MQTT callback ─────────────────────────────────────────────
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  String msg;
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
  msg.trim();
  Serial.printf("MQTT rx [%s]: %s\n", topic, msg.c_str());

  if (strcmp(topic, T_OTA_URL) == 0 && msg.length() > 10) {
    otaUrl = msg; otaPending = true;
    mqtt.publish(T_OTA_STATUS, "queued");
    return;
  }
  if (strcmp(topic, T_RESTART) == 0) {
    mqtt.publish(T_STATUS, "{\"event\":\"restarting\"}");
    delay(500); ESP.restart();
  }
}

// ── MQTT connect / reconnect ──────────────────────────────────
void mqttConnect() {
  if (mqtt.connected()) return;
  Serial.print("MQTT connecting...");
  if (mqtt.connect(DEF_MQTT_CLIENT, gMqttUser, gMqttPass,
                   T_STATUS, 0, true, "{\"online\":false}")) {
    Serial.println(" OK");
    mqtt.subscribe(T_OTA_URL);
    mqtt.subscribe(T_RESTART);
    char will[80];
    snprintf(will, sizeof(will),
      "{\"online\":true,\"fw\":\"%s\",\"ip\":\"%s\"}",
      FW_VERSION, WiFi.localIP().toString().c_str());
    mqtt.publish(T_STATUS, will, true);
  } else {
    Serial.printf(" failed rc=%d\n", mqtt.state());
  }
}

// ── OTA update ────────────────────────────────────────────────
void doOTA() {
  mqtt.publish(T_OTA_STATUS, "starting");
  updateOLED();
  WiFiClientSecure otaClient;
  otaClient.setInsecure();
  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  httpUpdate.rebootOnUpdate(true);
  Serial.printf("OTA from: %s\n", otaUrl.c_str());
  t_httpUpdate_return ret = httpUpdate.update(otaClient, otaUrl);
  switch (ret) {
    case HTTP_UPDATE_OK:
      mqtt.publish(T_OTA_STATUS, "success"); break;
    case HTTP_UPDATE_FAILED:
      mqtt.publish(T_OTA_STATUS,
        ("failed: " + String(httpUpdate.getLastErrorString())).c_str()); break;
    case HTTP_UPDATE_NO_UPDATES:
      mqtt.publish(T_OTA_STATUS, "no-update"); break;
  }
  otaPending = false; otaUrl = "";
}

// ================================================================
//  WEB SERVER HANDLERS
// ================================================================

void handleRoot() {
  if (apMode) webServer.send_P(200, "text/html", SETUP_PAGE);
  else        webServer.send_P(200, "text/html", PAGE);
}

void handleStatus() {
  char json[512];
  snprintf(json, sizeof(json),
    "{\"wifi\":%s,\"mqtt\":%s,\"fix\":%s,"
    "\"ip\":\"%s\",\"fw\":\"%s\",\"uptime\":%lu,"
    "\"hits\":%d,\"lastRow\":%d,\"lastTree\":%d,\"lastDist\":%.3f,"
    "\"lat\":%.7f,\"lon\":%.7f,"
    "\"cfg\":{\"lat\":%.7f,\"lon\":%.7f,\"brg\":%.1f,"
    "\"rs\":%.1f,\"ts\":%.1f,\"rows\":%d,\"trees\":%d,"
    "\"hr\":%.2f,\"rp\":%d,\"ssid\":\"%s\",\"udp\":%d}}",
    WiFi.status()==WL_CONNECTED?"true":"false",
    mqtt.connected()?"true":"false",
    gpsFix?"true":"false",
    WiFi.localIP().toString().c_str(),
    FW_VERSION, millis()/1000,
    totalHits, lastRow, lastTree, lastDist,
    curLat, curLon,
    gOriginLat, gOriginLon, gRowBearing,
    gRowSpacing, gTreeSpacing, gNumRows, gNumTrees,
    gHitRadius, gRelayPulse, gWifiSsid, gUdpPort
  );
  webServer.sendHeader("Access-Control-Allow-Origin", "*");
  webServer.send(200, "application/json", json);
}

void handleRelay() {
  if (webServer.method() != HTTP_POST) {
    webServer.send(405, "text/plain", "POST only"); return;
  }
  if (!relayActive) {
    digitalWrite(RELAY_PIN, HIGH);
    relayActive = true;
    relayStart  = millis();
    Serial.println("WEB: relay test fired");
  }
  webServer.send(200, "text/plain", "OK");
}

// Simple JSON field extraction helper (no full JSON parser needed)
float jsonFloat(const String& body, const char* key) {
  int i = body.indexOf(String("\"") + key + "\"");
  if (i < 0) return 0;
  i = body.indexOf(':', i) + 1;
  return body.substring(i).toFloat();
}
String jsonStr(const String& body, const char* key) {
  int i = body.indexOf(String("\"") + key + "\"");
  if (i < 0) return "";
  i = body.indexOf('"', body.indexOf(':', i)) + 1;
  int j = body.indexOf('"', i);
  return body.substring(i, j);
}

void handleConfigGrid() {
  if (webServer.method() != HTTP_POST) {
    webServer.send(405, "text/plain", "POST only"); return;
  }
  String body = webServer.arg("plain");
  gOriginLat   = jsonFloat(body, "lat");
  gOriginLon   = jsonFloat(body, "lon");
  gRowBearing  = jsonFloat(body, "brg");
  gRowSpacing  = jsonFloat(body, "rs");
  gTreeSpacing = jsonFloat(body, "ts");
  gNumRows     = (int)jsonFloat(body, "nr");
  gNumTrees    = (int)jsonFloat(body, "nt");
  gHitRadius   = jsonFloat(body, "hr");
  gRelayPulse  = (int)jsonFloat(body, "rp");

  // Clamp to safe grid array bounds
  if (gNumRows  < 1)  gNumRows  = 1;
  if (gNumRows  > 20) gNumRows  = 20;
  if (gNumTrees < 1)  gNumTrees = 1;
  if (gNumTrees > 100) gNumTrees = 100;

  saveGridPrefs();
  buildGrid();
  Serial.println("WEB: grid config updated");
  webServer.send(200, "text/plain",
    "Grid saved and rebuilt: " + String(gNumRows) + " rows x " + String(gNumTrees) + " trees");
}

void handleConfigWifi() {
  if (webServer.method() != HTTP_POST) {
    webServer.send(405, "text/plain", "POST only"); return;
  }
  String body = webServer.arg("plain");
  String ssid = jsonStr(body, "ssid");
  String pass = jsonStr(body, "pass");
  if (ssid.length() == 0) {
    webServer.send(400, "text/plain", "SSID cannot be empty"); return;
  }
  strlcpy(gWifiSsid, ssid.c_str(), sizeof(gWifiSsid));
  if (pass.length() > 0) strlcpy(gWifiPass, pass.c_str(), sizeof(gWifiPass));
  int newPort = (int)jsonFloat(body, "udp");
  if (newPort > 0 && newPort != gUdpPort) {
    gUdpPort = newPort;
    if (!apMode) { udp.stop(); udp.begin(gUdpPort); }
    Serial.printf("WEB: UDP port set to %d\n", gUdpPort);
  }
  saveWifiPrefs();
  webServer.send(200, "text/plain", "Network saved — rebooting in 2s...");
  delay(2000);
  ESP.restart();
}

void handleOtaWeb() {
  if (webServer.method() != HTTP_POST) {
    webServer.send(405, "text/plain", "POST only"); return;
  }
  String url = jsonStr(webServer.arg("plain"), "url");
  if (url.length() < 10) {
    webServer.send(400, "text/plain", "Missing url"); return;
  }
  otaUrl     = url;
  otaPending = true;
  webServer.send(200, "text/plain", "OTA queued — flashing now, board will reboot");
  Serial.println("WEB: OTA triggered: " + url);
}

// ── Setup page (served in AP mode at http://192.168.4.1/) ────
void handleSetup() {
  webServer.send_P(200, "text/html", SETUP_PAGE);
}

// ── WiFi scan JSON for setup page ────────────────────────────
void handleScan() {
  int n = WiFi.scanNetworks();
  String out = "[";
  for (int i = 0; i < n && i < 20; i++) {
    if (i) out += ",";
    out += "{\"ssid\":\"";
    String ssid = WiFi.SSID(i);
    ssid.replace("\\", "\\\\");
    ssid.replace("\"", "\\\"");
    out += ssid;
    out += "\",\"rssi\":";
    out += String(WiFi.RSSI(i));
    out += ",\"sec\":";
    out += (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "false" : "true";
    out += "}";
  }
  out += "]";
  WiFi.scanDelete();
  webServer.send(200, "application/json", out);
}

// ── Direct .bin upload OTA ────────────────────────────────────
// The ESP32 WebServer upload handler runs chunk-by-chunk during
// the POST. We also register a completion handler that fires when
// the full upload is done.
void handleOtaUploadEnd() {
  if (Update.hasError()) {
    webServer.send(500, "text/plain",
      String("Upload failed: ") + Update.errorString());
  } else {
    webServer.send(200, "text/plain",
      "Upload OK — rebooting into new firmware");
    otaRebootPending = true;
    otaRebootAt = millis() + 1000;
  }
}

void handleOtaUploadChunk() {
  HTTPUpload& up = webServer.upload();
  if (up.status == UPLOAD_FILE_START) {
    Serial.printf("OTA upload start: %s\n", up.filename.c_str());
    if (mqtt.connected()) mqtt.publish(T_OTA_STATUS, "upload-start");
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
    }
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (Update.write(up.buf, up.currentSize) != up.currentSize) {
      Update.printError(Serial);
    }
  } else if (up.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      Serial.printf("OTA upload OK: %u bytes\n", up.totalSize);
      if (mqtt.connected()) mqtt.publish(T_OTA_STATUS, "upload-success");
    } else {
      Update.printError(Serial);
      if (mqtt.connected()) mqtt.publish(T_OTA_STATUS, "upload-failed");
    }
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
    Serial.println("OTA upload aborted");
  }
}

// ── GitHub latest release lookup ─────────────────────────────
// Hits the releases/latest API, pulls tag_name and the .bin asset URL.
void handleOtaLatest() {
  if (WiFi.status() != WL_CONNECTED || apMode) {
    webServer.send(503, "application/json",
      "{\"error\":\"No internet in setup mode\"}");
    return;
  }
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setUserAgent("tree-marker-alr");
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  if (!http.begin(client, GH_LATEST_URL)) {
    webServer.send(500, "application/json",
      "{\"error\":\"http.begin failed\"}");
    return;
  }
  int code = http.GET();
  if (code != 200) {
    http.end();
    char err[80];
    snprintf(err, sizeof(err), "{\"error\":\"HTTP %d\"}", code);
    webServer.send(502, "application/json", err);
    return;
  }
  String body = http.getString();
  http.end();

  // Parse tag_name
  String tag = "";
  int ti = body.indexOf("\"tag_name\"");
  if (ti >= 0) {
    int q1 = body.indexOf('"', body.indexOf(':', ti)) + 1;
    int q2 = body.indexOf('"', q1);
    if (q1 > 0 && q2 > q1) tag = body.substring(q1, q2);
  }
  // Find first browser_download_url ending in .bin
  String binUrl = "";
  int search = 0;
  while (true) {
    int k = body.indexOf("\"browser_download_url\"", search);
    if (k < 0) break;
    int q1 = body.indexOf('"', body.indexOf(':', k)) + 1;
    int q2 = body.indexOf('"', q1);
    if (q1 > 0 && q2 > q1) {
      String u = body.substring(q1, q2);
      if (u.endsWith(".bin")) { binUrl = u; break; }
    }
    search = q2 + 1;
  }
  char out[600];
  snprintf(out, sizeof(out),
    "{\"tag\":\"%s\",\"url\":\"%s\",\"current\":\"%s\"}",
    tag.c_str(), binUrl.c_str(), FW_VERSION);
  webServer.send(200, "application/json", out);
}

// ── Start SoftAP captive-portal setup mode ───────────────────
void startAP() {
  apMode = true;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID);
  delay(200);
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("AP mode: %s @ %s\n", AP_SSID, ip.toString().c_str());

  // Catch-all DNS → AP IP (makes phone/laptop auto-open the setup page)
  dnsServer.start(53, "*", ip);

  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.setTextSize(1);
  oled.println("SETUP MODE");
  oled.println(AP_SSID);
  oled.println(ip.toString());
  oled.println("No password");
  oled.display();
}

// ── Try STA. Returns true if connected within STA_TIMEOUT_MS. ─
bool tryStationMode() {
  if (strlen(gWifiSsid) == 0) return false;
  WiFi.mode(WIFI_STA);
  WiFi.begin(gWifiSsid, gWifiPass);
  Serial.printf("WiFi STA: %s", gWifiSsid);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > STA_TIMEOUT_MS) {
      Serial.println(" TIMEOUT");
      WiFi.disconnect(true, false);
      return false;
    }
    delay(400);
    Serial.print(".");
  }
  Serial.printf("\nWiFi OK  %s\n", WiFi.localIP().toString().c_str());
  return true;
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.printf("\n=== Tree Marker  v%s ===\n", FW_VERSION);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  pinMode(SETUP_BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(OLED_SDA, OLED_SCL);
  if (oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.printf("Tree Marker v%s\n", FW_VERSION);
    oled.println("Starting...");
    oled.display();
  }

  loadPrefs();
  buildGrid();

  // If DL button is held at boot, force AP setup mode.
  bool forceAP = (digitalRead(SETUP_BUTTON_PIN) == LOW);
  if (forceAP) Serial.println("DL button held — forcing AP setup mode");

  if (forceAP || !tryStationMode()) {
    startAP();
  }

  // Web server routes — same endpoints in both modes; /scan + /setup
  // are what the AP setup page uses.
  webServer.on("/",            HTTP_GET,  handleRoot);
  webServer.on("/setup",       HTTP_GET,  handleSetup);
  webServer.on("/scan",        HTTP_GET,  handleScan);
  webServer.on("/api/status",  HTTP_GET,  handleStatus);
  webServer.on("/relay",       HTTP_POST, handleRelay);
  webServer.on("/config/grid", HTTP_POST, handleConfigGrid);
  webServer.on("/config/wifi", HTTP_POST, handleConfigWifi);
  webServer.on("/ota",         HTTP_POST, handleOtaWeb);
  webServer.on("/ota/latest",  HTTP_GET,  handleOtaLatest);
  webServer.on("/ota/upload",  HTTP_POST, handleOtaUploadEnd, handleOtaUploadChunk);
  // Captive-portal catch-all: anything unknown in AP mode redirects to setup
  webServer.onNotFound([]() {
    if (apMode) { webServer.send_P(200, "text/html", SETUP_PAGE); }
    else        { webServer.send(404, "text/plain", "Not found"); }
  });
  webServer.begin();

  if (apMode) {
    Serial.println("Web UI: http://192.168.4.1/ (setup)");
  } else {
    Serial.printf("Web UI: http://%s/\n", WiFi.localIP().toString().c_str());
    udp.begin(gUdpPort);
    Serial.printf("UDP listening on port %d\n", gUdpPort);

    tlsClient.setInsecure();
    mqtt.setServer(gMqttHost, gMqttPort);
    mqtt.setCallback(mqttCallback);
    mqtt.setBufferSize(512);
    mqttConnect();
  }
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
  // Relay off after pulse
  if (relayActive && millis() - relayStart > (unsigned long)gRelayPulse) {
    digitalWrite(RELAY_PIN, LOW);
    relayActive = false;
  }

  // OTA
  if (otaPending) doOTA();
  if (otaRebootPending && millis() >= otaRebootAt) {
    Serial.println("Rebooting after upload OTA...");
    delay(100);
    ESP.restart();
  }

  // Web server (always)
  webServer.handleClient();

  // AP setup mode: only run web + DNS. Skip GPS / MQTT / OLED updates below.
  if (apMode) {
    dnsServer.processNextRequest();
    static unsigned long lastApOled = 0;
    if (millis() - lastApOled > 500) {
      lastApOled = millis();
      oled.clearDisplay();
      oled.setCursor(0, 0);
      oled.setTextSize(1);
      oled.println("SETUP MODE");
      oled.println(AP_SSID);
      oled.println("192.168.4.1");
      oled.display();
    }
    return;
  }

  // MQTT keep-alive
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqtt.connected()) mqttConnect();
    mqtt.loop();
  }

  // Read UDP from AgIO / AgOpenGPS
  int pkt = udp.parsePacket();
  if (pkt > 0) {
    uint8_t buf[256];
    int len = udp.read(buf, sizeof(buf) - 1);
    double lat, lon;
    if (parsePGN100(buf, len, lat, lon)) {
      // AOG NAV PGN 100 — works in RTK and simulator mode
      gpsFix = true; curLat = lat; curLon = lon;
      lastFixMs = millis();
      checkGrid(lat, lon);
    } else {
      // Fallback: NMEA text ($GNGGA / $GPGGA) if AgIO NMEA-out enabled
      buf[len] = '\0';
      char* line = strtok((char*)buf, "\r\n");
      while (line) {
        if (parseGGA(line, lat, lon)) {
          gpsFix = true; curLat = lat; curLon = lon;
          lastFixMs = millis();
          checkGrid(lat, lon);
        }
        line = strtok(nullptr, "\r\n");
      }
    }
  }

  // GPS fix timeout
  if (gpsFix && millis() - lastFixMs > 3000) {
    gpsFix = false;
    Serial.println("AgIO position lost");
  }

  // MQTT heartbeat every 30s
  if (millis() - lastHeartbeat > 30000) {
    lastHeartbeat = millis();
    char hb[120];
    snprintf(hb, sizeof(hb),
      "{\"online\":true,\"fw\":\"%s\",\"uptime\":%lu,\"hits\":%d,\"fix\":%s}",
      FW_VERSION, millis()/1000, totalHits, gpsFix?"true":"false");
    mqtt.publish(T_STATUS, hb, true);
  }

  // PGN 239 to AgOpenGPS every 100ms
  if (WiFi.status() == WL_CONNECTED && millis() - lastMachineSend > 100) {
    sendMachinePGN();
    lastMachineSend = millis();
  }

  // OLED every 400ms
  static unsigned long lastOled = 0;
  if (millis() - lastOled > 400) {
    updateOLED();
    lastOled = millis();
  }
}
