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

// ── Firmware version (bumped on each release) ─────────────────
#define FW_VERSION "1.1.0"

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
#define UDP_PORT  9999

// ── MQTT topics ───────────────────────────────────────────────
#define T_STATUS      "treemarker/status"
#define T_HIT         "treemarker/hit"
#define T_OTA_URL     "treemarker/ota/url"
#define T_OTA_STATUS  "treemarker/ota/status"
#define T_RESTART     "treemarker/restart"

// ── Pin map ───────────────────────────────────────────────────
#define RELAY_PIN   48
#define OLED_SDA    39
#define OLED_SCL    38
#define SCREEN_W   128
#define SCREEN_H    64

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
  prefs.end();
}

// ── Objects ───────────────────────────────────────────────────
WiFiUDP           udp;
WiFiClientSecure  tlsClient;
PubSubClient      mqtt(tlsClient);
Adafruit_SSD1306  oled(SCREEN_W, SCREEN_H, &Wire, -1);
WebServer         webServer(80);

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
<html><head>
<meta charset=UTF-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>Tree Marker ALR</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:-apple-system,BlinkMacSystemFont,sans-serif;background:#0f172a;color:#e2e8f0}
header{background:#1e293b;padding:14px 20px;border-bottom:2px solid #334155;display:flex;align-items:center;gap:10px}
header h1{font-size:17px;flex:1}
.dot{width:10px;height:10px;border-radius:50%;background:#ef4444;flex-shrink:0}
.dot.ok{background:#22c55e;box-shadow:0 0 6px #22c55e}
main{padding:14px;max-width:560px;margin:auto}
.tabs{display:flex;gap:4px;margin-bottom:0}
.tab{background:#334155;color:#94a3b8;border:none;padding:8px 18px;border-radius:6px 6px 0 0;cursor:pointer;font-size:13px}
.tab.active{background:#1e293b;color:#e2e8f0;font-weight:600}
.section{display:none}
.section.active{display:block}
.card{background:#1e293b;border-radius:0 8px 8px 8px;padding:14px;margin:0 0 12px;border:1px solid #334155}
.card:not(:first-child){border-radius:8px}
.card h2{font-size:11px;color:#64748b;text-transform:uppercase;letter-spacing:.5px;margin-bottom:10px}
.row{display:flex;justify-content:space-between;padding:5px 0;border-bottom:1px solid #0f172a;font-size:13px}
.row:last-child{border:none}
.val{color:#38bdf8;font-weight:600}
.val.ok{color:#22c55e}
.val.bad{color:#ef4444}
.val.hit{color:#4ade80;font-size:20px}
button{background:#1d4ed8;color:#fff;border:none;padding:10px 16px;border-radius:6px;font-size:13px;cursor:pointer;width:100%;margin-top:8px}
button.green{background:#15803d}
button.red{background:#dc2626}
button:active{opacity:.7}
label{display:block;font-size:11px;color:#94a3b8;margin:8px 0 3px}
input{width:100%;background:#0f172a;border:1px solid #334155;color:#e2e8f0;padding:8px;border-radius:4px;font-size:13px}
.note{font-size:11px;color:#64748b;margin-top:6px}
</style></head>
<body>
<header>
  <div class=dot id=wdot></div>
  <h1>Tree Marker ALR</h1>
  <span id=hfw style="font-size:11px;color:#64748b"></span>
</header>
<main>
<div class=tabs>
  <button class="tab active" onclick="showTab('status',this)">Status</button>
  <button class=tab onclick="showTab('config',this)">Config</button>
</div>

<div id=t-status class="section active">
  <div class=card>
    <h2>System</h2>
    <div class=row><span>WiFi</span><span class=val id=s-wifi></span></div>
    <div class=row><span>IP Address</span><span class=val id=s-ip></span></div>
    <div class=row><span>MQTT</span><span class=val id=s-mqtt></span></div>
    <div class=row><span>GPS Fix</span><span class=val id=s-gps></span></div>
    <div class=row><span>Position</span><span class=val id=s-pos>—</span></div>
    <div class=row><span>Uptime</span><span class=val id=s-up></span></div>
    <div class=row><span>Firmware</span><span class=val id=s-fw></span></div>
  </div>
  <div class=card>
    <h2>This Session</h2>
    <div class=row><span>Hits</span><span class="val hit" id=s-hits>0</span></div>
    <div class=row><span>Last Row</span><span class=val id=s-lrow>—</span></div>
    <div class=row><span>Last Tree</span><span class=val id=s-ltree>—</span></div>
    <div class=row><span>Last distance</span><span class=val id=s-ldist>—</span></div>
  </div>
  <div class=card>
    <h2>Grid (active)</h2>
    <div class=row><span>Origin Lat</span><span class=val id=c-lat></span></div>
    <div class=row><span>Origin Lon</span><span class=val id=c-lon></span></div>
    <div class=row><span>Row Bearing</span><span class=val id=c-brg></span></div>
    <div class=row><span>Row Spacing</span><span class=val id=c-rs></span></div>
    <div class=row><span>Tree Spacing</span><span class=val id=c-ts></span></div>
    <div class=row><span>Rows x Trees</span><span class=val id=c-grid></span></div>
    <div class=row><span>Hit Radius</span><span class=val id=c-hr></span></div>
    <div class=row><span>Relay Pulse</span><span class=val id=c-rp></span></div>
  </div>
  <div class=card>
    <h2>Test</h2>
    <button class=green onclick=testRelay()>Fire Relay (wiring test)</button>
  </div>
</div>

<div id=t-config class=section>
  <div class=card>
    <h2>Grid Parameters</h2>
    <label>Origin Latitude</label><input id=f-lat type=number step=0.0000001>
    <label>Origin Longitude</label><input id=f-lon type=number step=0.0000001>
    <label>Row Bearing (deg: 0=N 90=E 180=S 270=W)</label><input id=f-brg type=number step=0.1>
    <label>Row Spacing (m)</label><input id=f-rs type=number step=0.1>
    <label>Tree Spacing (m)</label><input id=f-ts type=number step=0.1>
    <label>Number of Rows</label><input id=f-nr type=number step=1 min=1 max=20>
    <label>Trees per Row</label><input id=f-nt type=number step=1 min=1 max=100>
    <label>Hit Radius (m)</label><input id=f-hr type=number step=0.01>
    <label>Relay Pulse (ms)</label><input id=f-rp type=number step=50>
    <button onclick=saveGrid()>Save Grid + Apply Now</button>
    <p class=note>Grid rebuilds immediately — no reboot needed.</p>
  </div>
  <div class=card>
    <h2>WiFi (requires reboot)</h2>
    <label>Network Name (SSID)</label><input id=f-ssid type=text>
    <label>Password</label><input id=f-wpass type=password placeholder="(unchanged if blank)">
    <button class=red onclick=saveWifi()>Save WiFi + Reboot</button>
    <p class=note>Board will reconnect to new network after reboot.</p>
  </div>
</div>
</main>
<script>
function showTab(t,btn){
  document.querySelectorAll('.section').forEach(e=>e.classList.remove('active'));
  document.querySelectorAll('.tab').forEach(e=>e.classList.remove('active'));
  document.getElementById('t-'+t).classList.add('active');
  btn.classList.add('active');
}
function uptime(s){return Math.floor(s/3600)+'h '+Math.floor((s%3600)/60)+'m '+s%60+'s'}
function poll(){
  fetch('/api/status').then(r=>r.json()).then(d=>{
    const ok=d.wifi,mo=d.mqtt,go=d.fix;
    document.getElementById('wdot').className='dot'+(ok?' ok':'');
    set('s-wifi',ok?'Connected':'Disconnected',ok);
    set('s-ip',d.ip,true);
    set('s-mqtt',mo?'Connected':'Disconnected',mo);
    set('s-gps',go?'RTK Fix':'No Fix',go);
    document.getElementById('s-pos').textContent=go?d.lat.toFixed(7)+', '+d.lon.toFixed(7):'—';
    document.getElementById('s-up').textContent=uptime(d.uptime);
    document.getElementById('s-fw').textContent='v'+d.fw;
    document.getElementById('hfw').textContent='v'+d.fw;
    document.getElementById('s-hits').textContent=d.hits;
    document.getElementById('s-lrow').textContent=d.lastRow>=0?'Row '+d.lastRow:'—';
    document.getElementById('s-ltree').textContent=d.lastTree>=0?'Tree '+d.lastTree:'—';
    document.getElementById('s-ldist').textContent=d.lastDist>0?d.lastDist.toFixed(3)+' m':'—';
    const c=d.cfg;
    document.getElementById('c-lat').textContent=c.lat.toFixed(7);
    document.getElementById('c-lon').textContent=c.lon.toFixed(7);
    document.getElementById('c-brg').textContent=c.brg+'\u00b0';
    document.getElementById('c-rs').textContent=c.rs+' m';
    document.getElementById('c-ts').textContent=c.ts+' m';
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
  }).catch(()=>{});
}
function set(id,txt,ok){
  const e=document.getElementById(id);
  e.textContent=txt;
  e.className='val '+(ok?'ok':'bad');
}
function testRelay(){
  fetch('/relay',{method:'POST'}).then(()=>alert('Relay fired for '+document.getElementById('c-rp').textContent));
}
function saveGrid(){
  const b={lat:+document.getElementById('f-lat').value,lon:+document.getElementById('f-lon').value,
    brg:+document.getElementById('f-brg').value,rs:+document.getElementById('f-rs').value,
    ts:+document.getElementById('f-ts').value,nr:+document.getElementById('f-nr').value,
    nt:+document.getElementById('f-nt').value,hr:+document.getElementById('f-hr').value,
    rp:+document.getElementById('f-rp').value};
  fetch('/config/grid',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)})
    .then(r=>r.text()).then(t=>{alert(t);poll();});
}
function saveWifi(){
  const ssid=document.getElementById('f-ssid').value;
  const pass=document.getElementById('f-wpass').value;
  if(!ssid){alert('SSID cannot be empty');return;}
  if(!confirm('Save WiFi "'+ssid+'" and reboot?'))return;
  fetch('/config/wifi',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({ssid:ssid,pass:pass})})
    .then(r=>r.text()).then(t=>alert(t));
}
poll();setInterval(poll,2000);
</script>
</body></html>)rawpage";

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
void checkGrid(double lat, double lon) {
  if (relayActive) return;
  if (millis() - lastHit < (unsigned long)gRelayPulse + 200) return;

  for (int r = 0; r < gNumRows; r++) {
    for (int t = 0; t < gNumTrees; t++) {
      double d = distM(lat, lon, grid[r][t].lat, grid[r][t].lon);
      if (d < gHitRadius) {
        digitalWrite(RELAY_PIN, HIGH);
        relayActive = true;
        relayStart  = millis();
        lastHit     = millis();
        lastRow     = r;
        lastTree    = t;
        lastDist    = d;
        totalHits++;

        Serial.printf("HIT  Row %-2d  Tree %-3d  %.3fm\n", r, t, d);

        char payload[80];
        snprintf(payload, sizeof(payload),
          "{\"row\":%d,\"tree\":%d,\"dist\":%.3f,\"lat\":%.7f,\"lon\":%.7f}",
          r, t, d, lat, lon);
        mqtt.publish(T_HIT, payload, true);
        return;
      }
    }
  }
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
  tlsClient.setInsecure();
  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  httpUpdate.rebootOnUpdate(true);
  Serial.printf("OTA from: %s\n", otaUrl.c_str());
  t_httpUpdate_return ret = httpUpdate.update(tlsClient, otaUrl);
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
  webServer.send_P(200, "text/html", PAGE);
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
    "\"hr\":%.2f,\"rp\":%d,\"ssid\":\"%s\"}}",
    WiFi.status()==WL_CONNECTED?"true":"false",
    mqtt.connected()?"true":"false",
    gpsFix?"true":"false",
    WiFi.localIP().toString().c_str(),
    FW_VERSION, millis()/1000,
    totalHits, lastRow, lastTree, lastDist,
    curLat, curLon,
    gOriginLat, gOriginLon, gRowBearing,
    gRowSpacing, gTreeSpacing, gNumRows, gNumTrees,
    gHitRadius, gRelayPulse, gWifiSsid
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
  saveWifiPrefs();
  webServer.send(200, "text/plain", "WiFi saved — rebooting in 2s...");
  delay(2000);
  ESP.restart();
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

  WiFi.mode(WIFI_STA);
  WiFi.begin(gWifiSsid, gWifiPass);
  Serial.print("WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\nWiFi OK  %s\n", WiFi.localIP().toString().c_str());

  // Web server routes
  webServer.on("/",            HTTP_GET,  handleRoot);
  webServer.on("/api/status",  HTTP_GET,  handleStatus);
  webServer.on("/relay",       HTTP_POST, handleRelay);
  webServer.on("/config/grid", HTTP_POST, handleConfigGrid);
  webServer.on("/config/wifi", HTTP_POST, handleConfigWifi);
  webServer.begin();
  Serial.printf("Web UI: http://%s/\n", WiFi.localIP().toString().c_str());

  udp.begin(UDP_PORT);
  Serial.printf("UDP listening on port %d\n", UDP_PORT);

  tlsClient.setInsecure();
  mqtt.setServer(gMqttHost, gMqttPort);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(512);
  mqttConnect();
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

  // Web server
  webServer.handleClient();

  // MQTT keep-alive
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqtt.connected()) mqttConnect();
    mqtt.loop();
  }

  // Read UDP from AgIO
  int pkt = udp.parsePacket();
  if (pkt > 0) {
    char buf[256];
    int len = udp.read(buf, sizeof(buf) - 1);
    buf[len] = '\0';
    char* line = strtok(buf, "\r\n");
    while (line) {
      double lat, lon;
      if (parseGGA(line, lat, lon)) {
        gpsFix = true; curLat = lat; curLon = lon;
        lastFixMs = millis();
        checkGrid(lat, lon);
      }
      line = strtok(nullptr, "\r\n");
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
