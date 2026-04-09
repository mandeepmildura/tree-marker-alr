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
#include <WiFiUDP.h>
#include <WiFiClientSecure.h>
#include <HTTPUpdate.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ── Firmware version (bumped on each release) ─────────────────
#define FW_VERSION "1.0.0"

// ================================================================
//  USER CONFIG — edit these before first flash
// ================================================================

// WiFi (must be same network as AgOpenGPS tablet / phone hotspot)
const char* WIFI_SSID     = "YourHotspotName";
const char* WIFI_PASSWORD = "YourPassword";

// HiveMQ broker
const char* MQTT_HOST     = "your-id.s1.eu.hivemq.cloud";
const int   MQTT_PORT     = 8883;           // TLS port
const char* MQTT_USER     = "your-mqtt-user";
const char* MQTT_PASS     = "your-mqtt-pass";
const char* MQTT_CLIENT   = "tree-marker-alr-01";

// AgIO UDP — AIO Teensy broadcasts NMEA here
#define UDP_PORT  9999

// Grid — set from field survey, leave as-is for simulation
const double ORIGIN_LAT   = -34.200000;   // First tree (tree 0, row 0)
const double ORIGIN_LON   =  142.150000;
const double ROW_BEARING  =   90.0;       // Degrees: 0=N, 90=E, 180=S, 270=W
const double ROW_SPACING  =    6.7;       // Metres between rows
const double TREE_SPACING =    3.0;       // Metres between trees in a row
const int    NUM_ROWS     =   10;
const int    NUM_TREES    =   50;

// Trigger tuning
const double HIT_RADIUS       =  0.35;    // Metres — RTK allows tight radius
const int    RELAY_PULSE_MS   =  600;     // How long relay stays on (ms)
const unsigned long COOLDOWN_MS = 2000;   // Min ms between hits on same spot

// ================================================================
//  ALR BOARD PIN MAP (do not change)
// ================================================================
#define RELAY_PIN   48
#define OLED_SDA    39
#define OLED_SCL    38
#define SCREEN_W   128
#define SCREEN_H    64

// ================================================================
//  MQTT TOPICS
// ================================================================
#define T_STATUS      "treemarker/status"
#define T_HIT         "treemarker/hit"
#define T_OTA_URL     "treemarker/ota/url"
#define T_OTA_STATUS  "treemarker/ota/status"
#define T_RESTART     "treemarker/restart"

// ── Objects ───────────────────────────────────────────────────
WiFiUDP           udp;
WiFiClientSecure  tlsClient;
PubSubClient      mqtt(tlsClient);
Adafruit_SSD1306  oled(SCREEN_W, SCREEN_H, &Wire, -1);

// ── Grid ──────────────────────────────────────────────────────
const double M_PER_DEG_LAT = 111320.0;
double mPerDegLon;
double bearingRad, perpBearingRad;

struct Point { double lat; double lon; };
Point grid[20][100];   // [row][tree]  max 20×100

// ── State ─────────────────────────────────────────────────────
bool   relayActive   = false;
unsigned long relayStart   = 0;
unsigned long lastHit      = 0;
int    lastRow  = -1, lastTree  = -1;
double curLat   =  0, curLon   =  0;
bool   gpsFix   = false;
unsigned long lastFixMs    = 0;
int    totalHits           = 0;
unsigned long lastHeartbeat = 0;
bool   otaPending  = false;
String otaUrl      = "";

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

// ── Flat-earth distance (fine for <500m) ──────────────────────
double distM(double lat1, double lon1, double lat2, double lon2) {
  double dLat = (lat2 - lat1) * M_PER_DEG_LAT;
  double dLon = (lon2 - lon1) * mPerDegLon;
  return sqrt(dLat * dLat + dLon * dLon);
}

// ── Grid hit check ────────────────────────────────────────────
void checkGrid(double lat, double lon) {
  if (relayActive) return;
  if (millis() - lastHit < COOLDOWN_MS) return;

  for (int r = 0; r < NUM_ROWS; r++) {
    for (int t = 0; t < NUM_TREES; t++) {
      double d = distM(lat, lon, grid[r][t].lat, grid[r][t].lon);
      if (d < HIT_RADIUS) {
        digitalWrite(RELAY_PIN, HIGH);
        relayActive = true;
        relayStart  = millis();
        lastHit     = millis();
        lastRow     = r;
        lastTree    = t;
        totalHits++;

        Serial.printf("HIT  Row %-2d  Tree %-3d  %.3fm\n", r, t, d);

        // Publish hit event to MQTT
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

// ── MQTT callback ─────────────────────────────────────────────
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  String msg;
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
  msg.trim();

  Serial.printf("MQTT rx [%s]: %s\n", topic, msg.c_str());

  if (strcmp(topic, T_OTA_URL) == 0 && msg.length() > 10) {
    otaUrl     = msg;
    otaPending = true;
    mqtt.publish(T_OTA_STATUS, "queued");
    Serial.printf("OTA URL queued: %s\n", otaUrl.c_str());
    return;
  }

  if (strcmp(topic, T_RESTART) == 0) {
    mqtt.publish(T_STATUS, "{\"event\":\"restarting\"}");
    delay(500);
    ESP.restart();
  }
}

// ── MQTT connect / reconnect ──────────────────────────────────
void mqttConnect() {
  if (mqtt.connected()) return;
  Serial.print("MQTT connecting...");
  if (mqtt.connect(MQTT_CLIENT, MQTT_USER, MQTT_PASS,
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

// ── OTA update (called from loop, not callback) ───────────────
void doOTA() {
  mqtt.publish(T_OTA_STATUS, "starting");
  updateOLED();

  tlsClient.setInsecure();   // GitHub redirects — skip cert verify for OTA

  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  httpUpdate.rebootOnUpdate(true);

  Serial.printf("OTA from: %s\n", otaUrl.c_str());
  t_httpUpdate_return ret = httpUpdate.update(tlsClient, otaUrl);

  switch (ret) {
    case HTTP_UPDATE_OK:
      mqtt.publish(T_OTA_STATUS, "success — rebooting");
      break;
    case HTTP_UPDATE_FAILED:
      mqtt.publish(T_OTA_STATUS,
        ("failed: " + String(httpUpdate.getLastErrorString())).c_str());
      Serial.printf("OTA failed: %s\n", httpUpdate.getLastErrorString().c_str());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      mqtt.publish(T_OTA_STATUS, "no-update");
      break;
  }
  otaPending = false;
  otaUrl     = "";
}

// ── Build tree grid ───────────────────────────────────────────
void buildGrid() {
  mPerDegLon     = M_PER_DEG_LAT * cos(ORIGIN_LAT * M_PI / 180.0);
  bearingRad     = ROW_BEARING   * M_PI / 180.0;
  perpBearingRad = bearingRad    + M_PI / 2.0;

  for (int r = 0; r < NUM_ROWS; r++) {
    for (int t = 0; t < NUM_TREES; t++) {
      double along  = t * TREE_SPACING;
      double across = r * ROW_SPACING;
      grid[r][t].lat = ORIGIN_LAT +
        (along * cos(bearingRad) + across * cos(perpBearingRad)) / M_PER_DEG_LAT;
      grid[r][t].lon = ORIGIN_LON +
        (along * sin(bearingRad) + across * sin(perpBearingRad)) / mPerDegLon;
    }
  }
  Serial.printf("Grid: %d rows x %d trees  R=%.2fm  T=%.2fm  HIT=%.2fm\n",
    NUM_ROWS, NUM_TREES, ROW_SPACING, TREE_SPACING, HIT_RADIUS);
}

// ── setup ─────────────────────────────────────────────────────
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
    oled.setCursor(0, 0);
    oled.printf("Tree Marker v%s\n", FW_VERSION);
    oled.println("Starting...");
    oled.display();
  }

  buildGrid();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\nWiFi OK  %s\n", WiFi.localIP().toString().c_str());

  udp.begin(UDP_PORT);
  Serial.printf("UDP listening on port %d\n", UDP_PORT);

  tlsClient.setInsecure();   // HiveMQ Cloud uses TLS — cert pinning optional
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(512);
  mqttConnect();
}

// ── loop ──────────────────────────────────────────────────────
void loop() {
  // Relay off after pulse
  if (relayActive && millis() - relayStart > RELAY_PULSE_MS) {
    digitalWrite(RELAY_PIN, LOW);
    relayActive = false;
  }

  // Process OTA outside of MQTT callback
  if (otaPending) {
    doOTA();
  }

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
        gpsFix  = true;
        curLat  = lat;
        curLon  = lon;
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

  // Heartbeat to MQTT every 30s
  if (millis() - lastHeartbeat > 30000) {
    lastHeartbeat = millis();
    char hb[120];
    snprintf(hb, sizeof(hb),
      "{\"online\":true,\"fw\":\"%s\",\"uptime\":%lu,\"hits\":%d,\"fix\":%s}",
      FW_VERSION, millis() / 1000, totalHits, gpsFix ? "true" : "false");
    mqtt.publish(T_STATUS, hb, true);
  }

  // OLED every 400ms
  static unsigned long lastOled = 0;
  if (millis() - lastOled > 400) {
    updateOLED();
    lastOled = millis();
  }
}
