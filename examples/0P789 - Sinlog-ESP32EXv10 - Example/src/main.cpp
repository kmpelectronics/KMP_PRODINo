// 0P789 - Sinlog-ESP32EXv10 - Example  (PHASE 3: Ethernet + WiFi + web app)
//
// Board: Sinlog-ESP32EXv10 (ESP32-WROOM). W5500 Ethernet and WiFi both run on the
// native lwIP stack, so a single web server is reachable over whichever interface
// is up. The web page offers:
//   - live status of the 4 opto-isolated inputs
//   - a small RS485 terminal (send + receive)
//   - a text box that pushes text onto the OLED
//   - a WiFi setup wizard (scan / connect / forget), reachable over Ethernet too
//
// Network behaviour:
//   - Ethernet (DHCP) comes up automatically.
//   - Saved WiFi credentials (NVS) are tried in STA mode.
//   - If NO network is present a short while after boot, an AP + captive portal is
//     raised (SSID "Sinlog-XXXX") so WiFi can be provisioned from a phone.
//   - Status LED: RED = no network, BLUE = AP/portal, GREEN = connected.
//
// Pinout (Sinlog-ESP32EXv10 schematic):
//   W5500 SPI : CS=IO15  SCLK=IO18  MISO=IO19  MOSI=IO23  RST=IO17  INT=IO5
//   RS485     : RX(RO)=IO4  TX(DI)=IO16  DE/RE=IO2
//   Opto in   : IN1=IO35  IN2=IO34  IN3=IO39  IN4=IO36  (active = LOW)
//   I2C/Grove : SDA=IO21  SCL=IO22   (OLED @ 0x3C)
//   Status LED: WS2812 DIN=IO32
//
// Requires ESP32 Arduino core 3.x (see platformio.ini / pioarduino platform).

#include <Arduino.h>
#include <ETH.h>
#include <WiFi.h>
#include <SPI.h>
#include <Wire.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NeoPixelBus.h>

// ---------------- Pin map ----------------
#define ETH_PHY_TYPE  ETH_PHY_W5500
#define ETH_PHY_ADDR  1
#define ETH_CS_PIN    15
#define ETH_IRQ_PIN   5
#define ETH_RST_PIN   17
#define ETH_SCK_PIN   18
#define ETH_MISO_PIN  19
#define ETH_MOSI_PIN  23
#define ETH_SPI_MHZ   16

#define RS485_RX_PIN  4
#define RS485_TX_PIN  16
#define RS485_DE_PIN  2
#define RS485_BAUD    9600

#define I2C_SDA_PIN   21
#define I2C_SCL_PIN   22
#define OLED_ADDR     0x3C
#define OLED_W        128
#define OLED_H        64

#define STATUS_LED_PIN 32

#define HOSTNAME "sinlog"
#define AP_FALLBACK_MS 12000  // raise the AP portal if no network by this time

const uint8_t OPTO_PINS[4] = { 35, 34, 39, 36 };  // IN1..IN4, active LOW

// ---------------- Peripherals ----------------
Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, -1);
bool oledOk = false;

NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt0Ws2812xMethod> statusLed(1, STATUS_LED_PIN);
RgbColor cRed(32, 0, 0), cGreen(0, 32, 0), cBlue(0, 0, 32);

HardwareSerial RS485(1);
WebServer server(80);
DNSServer dns;
Preferences prefs;

bool ethConnected = false;
bool wifiConnected = false;
bool apActive = false;
IPAddress apIP(192, 168, 4, 1);
String apSsid;

String staSsid, staPass;      // saved WiFi credentials
String rs485Log;              // rolling RS485 receive log
String oledMsg;               // text pushed from the web page
unsigned long bootMs = 0;
bool apDecided = false;

// ---------------- Helpers ----------------
bool haveNetwork() { return ethConnected || wifiConnected; }

void setLed(RgbColor c) { statusLed.SetPixelColor(0, c); statusLed.Show(); }

void refreshStatus() {
  if (haveNetwork())   setLed(cGreen);
  else if (apActive)   setLed(cBlue);
  else                 setLed(cRed);
}

bool optoActive(uint8_t i) { return digitalRead(OPTO_PINS[i]) == LOW; }  // active LOW

String macSuffix() {
  uint8_t m[6];
  WiFi.macAddress(m);
  char b[5];
  sprintf(b, "%02X%02X", m[4], m[5]);
  return String(b);
}

void oledRender() {
  if (!oledOk) return;
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.println("Sinlog-ESP32EXv10");
  oled.print("ETH : ");
  oled.println(ethConnected ? ETH.localIP().toString() : String("-"));
  oled.print("WiFi: ");
  if (wifiConnected)   oled.println(WiFi.localIP().toString());
  else if (apActive)   oled.println("AP " + apSsid);
  else                 oled.println("-");
  oled.drawFastHLine(0, 28, OLED_W, SSD1306_WHITE);
  oled.setCursor(0, 34);
  oled.println(oledMsg);
  oled.display();
}

void rs485AppendRx() {
  while (RS485.available()) rs485Log += (char)RS485.read();
  if (rs485Log.length() > 2000)
    rs485Log.remove(0, rs485Log.length() - 2000);
}

void rs485Send(const String& s) {
  digitalWrite(RS485_DE_PIN, HIGH);
  delayMicroseconds(50);
  RS485.print(s);
  RS485.flush();
  delayMicroseconds(50);
  digitalWrite(RS485_DE_PIN, LOW);
  Serial.printf("[rs485 tx] %s\n", s.c_str());
}

// ---------------- WiFi ----------------
void loadCreds() {
  prefs.begin("wifi", true);
  staSsid = prefs.getString("ssid", "");
  staPass = prefs.getString("pass", "");
  prefs.end();
}

void saveCreds(const String& ssid, const String& pass) {
  prefs.begin("wifi", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.end();
  staSsid = ssid;
  staPass = pass;
}

void clearCreds() {
  prefs.begin("wifi", false);
  prefs.clear();
  prefs.end();
  staSsid = "";
  staPass = "";
  WiFi.disconnect(true, true);
  wifiConnected = false;
}

bool wifiConnectSta(const String& ssid, const String& pass, uint32_t timeoutMs) {
  WiFi.setHostname(HOSTNAME);
  WiFi.mode(apActive ? WIFI_AP_STA : WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());
  Serial.printf("[wifi] connecting to '%s' ...\n", ssid.c_str());
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < timeoutMs) delay(100);
  return WiFi.status() == WL_CONNECTED;
}

void startAP() {
  if (apActive) return;
  apSsid = "Sinlog-" + macSuffix();
  WiFi.mode(staSsid.length() ? WIFI_AP_STA : WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(apSsid.c_str());
  dns.start(53, "*", apIP);
  apActive = true;
  Serial.printf("[wifi] AP portal up: SSID='%s'  http://%s\n",
                apSsid.c_str(), apIP.toString().c_str());
  refreshStatus();
  oledRender();
}

// ---------------- Web handlers ----------------
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Sinlog-ESP32EXv10</title>
<style>
 body{font-family:system-ui,Arial,sans-serif;margin:0;background:#0f1620;color:#e6edf3}
 header{background:#16202c;padding:14px 18px;font-size:18px;font-weight:600;border-bottom:1px solid #263241}
 .wrap{max-width:760px;margin:0 auto;padding:16px}
 .card{background:#16202c;border:1px solid #263241;border-radius:10px;padding:16px;margin:14px 0}
 h2{margin:0 0 12px;font-size:15px;color:#8fb7e0}
 .opto{display:flex;gap:10px;flex-wrap:wrap}
 .in{flex:1;min-width:110px;text-align:center;padding:12px;border-radius:8px;background:#0f1620;border:1px solid #263241}
 .in .lbl{font-size:12px;color:#9db2c8}.in .st{font-size:20px;font-weight:700;margin-top:4px}
 .on{color:#39d353}.off{color:#5a6b7d}
 textarea{width:100%;box-sizing:border-box;height:150px;background:#0b1017;color:#cfe;border:1px solid #263241;border-radius:8px;padding:8px;font-family:monospace;font-size:13px}
 input,select{background:#0b1017;color:#e6edf3;border:1px solid #263241;border-radius:8px;padding:9px}
 input[type=text],input[type=password]{flex:1}
 .row{display:flex;gap:8px;margin-top:8px;flex-wrap:wrap}
 button{background:#2563eb;color:#fff;border:0;border-radius:8px;padding:9px 16px;font-weight:600;cursor:pointer}
 button:hover{background:#1d4ed8}
 .muted{color:#9db2c8;font-size:13px}
</style></head><body>
<header>Sinlog-ESP32EXv10 <span class="muted" id="net"></span></header>
<div class="wrap">
 <div class="card"><h2>Opto inputs</h2><div class="opto" id="opto"></div></div>
 <div class="card"><h2>RS485 terminal</h2>
  <textarea id="rxlog" readonly></textarea>
  <div class="row"><input type="text" id="txt" placeholder="text to send over RS485">
   <button onclick="sendRs()">Send</button><button onclick="clearLog()">Clear</button></div></div>
 <div class="card"><h2>OLED display</h2>
  <div class="row"><input type="text" id="oled" placeholder="text to show on the OLED">
   <button onclick="sendOled()">Show</button></div></div>
 <div class="card"><h2>WiFi setup</h2>
  <div class="muted" id="wifist">...</div>
  <div class="row"><button onclick="scan()">Scan</button>
   <select id="ssids" onchange="pick()" style="flex:1"></select></div>
  <div class="row"><input type="text" id="wssid" placeholder="SSID">
   <input type="password" id="wpass" placeholder="password"><button onclick="wconn()">Connect</button></div>
  <div class="row"><button onclick="wforget()" style="background:#3a4553">Forget</button></div></div>
</div>
<script>
let lastRx="";
async function poll(){try{
 const d=await(await fetch('/api/status')).json();
 let n='ETH '+(d.eth?d.eth_ip:'-')+' | WiFi '+(d.wifi?d.wifi_ip:(d.ap?'AP '+d.ap_ip:'-'));
 document.getElementById('net').textContent=n;
 let h='';for(let i=0;i<d.opto.length;i++)h+=`<div class="in"><div class="lbl">IN${i+1}</div><div class="st ${d.opto[i]?'on':'off'}">${d.opto[i]?'ON':'off'}</div></div>`;
 document.getElementById('opto').innerHTML=h;
 if(d.rs485!==lastRx){lastRx=d.rs485;const t=document.getElementById('rxlog');
  const b=t.scrollTop+t.clientHeight>=t.scrollHeight-4;t.value=d.rs485;if(b)t.scrollTop=t.scrollHeight}
}catch(e){}}
async function sendRs(){const v=document.getElementById('txt').value;
 await fetch('/api/rs485',{method:'POST',body:v});document.getElementById('txt').value=''}
async function clearLog(){await fetch('/api/rs485/clear',{method:'POST'});lastRx=''}
async function sendOled(){await fetch('/api/oled',{method:'POST',body:document.getElementById('oled').value})}
async function wifiStat(){const d=await(await fetch('/api/wifi')).json();
 let s=d.sta_connected?('Connected: '+d.sta_ssid+' ('+d.sta_ip+')'):'Not connected';
 if(d.ap_active)s+='  |  AP: '+d.ap_ssid+' ('+d.ap_ip+')';
 document.getElementById('wifist').textContent=s;}
function pick(){document.getElementById('wssid').value=document.getElementById('ssids').value}
async function scan(){document.getElementById('wifist').textContent='scanning...';
 const a=await(await fetch('/api/wifi/scan')).json();const sel=document.getElementById('ssids');sel.innerHTML='';
 a.forEach(n=>{const o=document.createElement('option');o.value=n.ssid;o.textContent=n.ssid+' ('+n.rssi+'dBm)'+(n.open?'':' [*]');sel.appendChild(o)});
 if(a.length)pick();wifiStat();}
async function wconn(){const ssid=document.getElementById('wssid').value,pass=document.getElementById('wpass').value;
 document.getElementById('wifist').textContent='connecting...';
 const d=await(await fetch('/api/wifi',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},
  body:'ssid='+encodeURIComponent(ssid)+'&pass='+encodeURIComponent(pass)})).json();
 document.getElementById('wifist').textContent=d.ok?('Connected ('+d.sta_ip+')'):'Connect failed';}
async function wforget(){await fetch('/api/wifi/forget',{method:'POST'});wifiStat();}
setInterval(poll,1000);poll();wifiStat();
</script></body></html>
)HTML";

void handleIndex() { server.send_P(200, "text/html", INDEX_HTML); }

void handleStatus() {
  JsonDocument doc;
  doc["eth"] = ethConnected;
  doc["eth_ip"] = ethConnected ? ETH.localIP().toString() : String("-");
  doc["wifi"] = wifiConnected;
  doc["wifi_ip"] = wifiConnected ? WiFi.localIP().toString() : String("-");
  doc["ap"] = apActive;
  doc["ap_ip"] = apIP.toString();
  JsonArray o = doc["opto"].to<JsonArray>();
  for (uint8_t i = 0; i < 4; i++) o.add(optoActive(i));
  doc["rs485"] = rs485Log;
  String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleRs485Send() {
  String body = server.arg("plain");
  if (body.length()) rs485Send(body);
  server.send(200, "text/plain", "ok");
}
void handleRs485Clear() { rs485Log = ""; server.send(200, "text/plain", "ok"); }

void handleOled() {
  oledMsg = server.arg("plain");
  oledRender();
  Serial.printf("[oled msg] %s\n", oledMsg.c_str());
  server.send(200, "text/plain", "ok");
}

void handleWifiStatus() {
  JsonDocument doc;
  doc["sta_connected"] = wifiConnected;
  doc["sta_ssid"] = staSsid;
  doc["sta_ip"] = wifiConnected ? WiFi.localIP().toString() : String("-");
  doc["ap_active"] = apActive;
  doc["ap_ssid"] = apSsid;
  doc["ap_ip"] = apIP.toString();
  String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleWifiScan() {
  int n = WiFi.scanNetworks();
  JsonDocument doc;
  JsonArray a = doc.to<JsonArray>();
  for (int i = 0; i < n; i++) {
    JsonObject o = a.add<JsonObject>();
    o["ssid"] = WiFi.SSID(i);
    o["rssi"] = WiFi.RSSI(i);
    o["open"] = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN);
  }
  WiFi.scanDelete();
  String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleWifiConnect() {
  String ssid = server.arg("ssid");
  String pass = server.arg("pass");
  if (!ssid.length()) { server.send(400, "application/json", "{\"ok\":false}"); return; }
  saveCreds(ssid, pass);
  bool ok = wifiConnectSta(ssid, pass, 8000);
  wifiConnected = ok;
  refreshStatus();
  oledRender();
  JsonDocument doc;
  doc["ok"] = ok;
  doc["sta_ip"] = ok ? WiFi.localIP().toString() : String("-");
  String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleWifiForget() {
  clearCreds();
  refreshStatus();
  oledRender();
  server.send(200, "text/plain", "ok");
}

void handleNotFound() {
  if (apActive) {  // captive portal: bounce everything to the setup page
    server.sendHeader("Location", "http://" + apIP.toString() + "/");
    server.send(302, "text/plain", "");
  } else {
    server.send(404, "text/plain", "not found");
  }
}

void startWebServer() {
  server.on("/", HTTP_GET, handleIndex);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/rs485", HTTP_POST, handleRs485Send);
  server.on("/api/rs485/clear", HTTP_POST, handleRs485Clear);
  server.on("/api/oled", HTTP_POST, handleOled);
  server.on("/api/wifi", HTTP_GET, handleWifiStatus);
  server.on("/api/wifi", HTTP_POST, handleWifiConnect);
  server.on("/api/wifi/scan", HTTP_GET, handleWifiScan);
  server.on("/api/wifi/forget", HTTP_POST, handleWifiForget);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("[web] server started on :80");
}

// ---------------- Network events ----------------
void onNetEvent(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      ETH.setHostname(HOSTNAME);
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.printf("[eth] got IP: %s\n", ETH.localIP().toString().c_str());
      ethConnected = true;
      if (MDNS.begin(HOSTNAME)) Serial.printf("[mdns] http://%s.local\n", HOSTNAME);
      refreshStatus(); oledRender();
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("[eth] link down");
      ethConnected = false;
      refreshStatus(); oledRender();
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.printf("[wifi] got IP: %s\n", WiFi.localIP().toString().c_str());
      wifiConnected = true;
      refreshStatus(); oledRender();
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      if (wifiConnected) Serial.println("[wifi] disconnected");
      wifiConnected = false;
      refreshStatus(); oledRender();
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Sinlog-ESP32EXv10 - PHASE 3 (Ethernet + WiFi + web) ===");
  bootMs = millis();

  statusLed.Begin();
  setLed(cRed);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  Serial.printf("[oled] init: %s\n", oledOk ? "OK" : "FAIL");
  oledMsg = "Booting...";
  oledRender();

  for (uint8_t i = 0; i < 4; i++) pinMode(OPTO_PINS[i], INPUT);

  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);
  RS485.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  Network.onEvent(onNetEvent);

  // Ethernet (W5500 over lwIP)
  SPI.begin(ETH_SCK_PIN, ETH_MISO_PIN, ETH_MOSI_PIN);
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_CS_PIN, ETH_IRQ_PIN, ETH_RST_PIN, SPI, ETH_SPI_MHZ);

  // WiFi STA from saved credentials (non-fatal if it fails; AP fallback below)
  loadCreds();
  if (staSsid.length()) {
    WiFi.setHostname(HOSTNAME);
    WiFi.mode(WIFI_STA);
    WiFi.begin(staSsid.c_str(), staPass.c_str());
    Serial.printf("[wifi] STA begin '%s'\n", staSsid.c_str());
  }

  startWebServer();
  oledMsg = "";
}

unsigned long lastReport = 0;

void loop() {
  server.handleClient();
  if (apActive) dns.processNextRequest();
  rs485AppendRx();

  // Raise the AP portal if nothing connected within the grace period.
  if (!apDecided && millis() - bootMs > AP_FALLBACK_MS) {
    apDecided = true;
    if (!haveNetwork()) {
      Serial.println("[net] no network - starting AP portal");
      startAP();
    }
  }

  if (millis() - lastReport >= 3000) {
    lastReport = millis();
    Serial.printf("[hb] ETH=%s(%s) WiFi=%s(%s) AP=%d opto=%d%d%d%d\n",
                  ethConnected ? "UP" : "dn", ethConnected ? ETH.localIP().toString().c_str() : "-",
                  wifiConnected ? "UP" : "dn", wifiConnected ? WiFi.localIP().toString().c_str() : "-",
                  apActive,
                  optoActive(0), optoActive(1), optoActive(2), optoActive(3));
  }
}
