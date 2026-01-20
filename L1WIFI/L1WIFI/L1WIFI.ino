#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiClient.h>

const char* ssid = "MechalinoAP";
const char* password = "12345679";
const int Mechalino_ID = 15;

ESP8266WebServer server(80);

// Laptop pose server
IPAddress laptopIP(192, 168, 50, 1);
const uint16_t POSE_PORT = 9000;

WiFiClient poseClient;
String serialBuf;

// UDP for Inter-swarm connection
#include <WiFiUdp.h>

#define UDP_PORT 4242
#define UDP_TX_PERIOD_MS 200   // 5 Hz

WiFiUDP udp;

static unsigned long udp_last_tx = 0;
static char udp_rx_buf[256];

static void ensurePoseClientConnected() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (poseClient.connected()) return;

  poseClient.stop();
  poseClient.setNoDelay(true);
  poseClient.setTimeout(1);

  bool ok = false;
  for (int k = 0; k < 5; k++) {
    if (poseClient.connect(laptopIP, POSE_PORT)) { ok = true; break; }
    poseClient.stop();
    delay(10);
    yield();
  }
}

static uint8_t requestPoseFromROS(float &x, float &y, float &yaw) {
  if (WiFi.status() != WL_CONNECTED) return 1;   // no WiFi

  ensurePoseClientConnected();
  if (!poseClient.connected()) return 2;

  if (poseClient.print("POSE\n") == 0) { poseClient.stop(); return 3; } // write fail

  char buf[96];
  size_t idx = 0;
  uint32_t t0 = millis();
  const uint32_t deadline_ms = 500; // increase for real WiFi jitter

  while ((millis() - t0) < deadline_ms) {
    while (poseClient.available() > 0) {
      char ch = (char)poseClient.read();
      if (ch == '\r') continue;

      if (ch == '\n') {
        buf[idx] = 0;

        // Accept both space and comma separated (just in case)
        for (size_t i = 0; buf[i]; i++) if (buf[i] == ',') buf[i] = ' ';

        float xx, yy, yyaw;
        if (sscanf(buf, " %f %f %f", &xx, &yy, &yyaw) == 3) {
          x = xx; y = yy; yaw = yyaw;
          return 0; // OK
        }
        return 5; // parse fail (format mismatch)
      }

      if (idx < sizeof(buf) - 1) buf[idx++] = ch;
      else { return 6; } // line too long
    }
    delay(1);
  }

  return 4; // timeout waiting for newline
}


static void sendPoseToSTM32(float x, float y, float yaw) {
  Serial.print("POS#");
  Serial.print(x, 6);
  Serial.print("#");
  Serial.print(y, 6);
  Serial.print("#");
  Serial.print(yaw, 6);
  Serial.print("\n");
}

void handleCMD() {
  if (!server.hasArg("cmd")) {
    server.send(400, "text/plain", "Error: required args are not provided.");
    return;
  }
  
  String cmd = server.arg("cmd");
  
  // Send to serial
  Serial.print("CMD#");
  Serial.print(cmd);
  if (server.hasArg("param1")){
    String param1 = server.arg("param1");
    Serial.print("#");
    Serial.print(param1);
  }
  if (server.hasArg("param2")){
    String param2 = server.arg("param2");
    Serial.print("#");
    Serial.print(param2);
  }
  if (server.hasArg("param3")){
    String param3 = server.arg("param3");
    Serial.print("#");
    Serial.print(param3);
  }
  if (server.hasArg("param4")){
    String param4 = server.arg("param4");
    Serial.print("#");
    Serial.print(param4);
  }
  if (server.hasArg("param5")){
    String param5 = server.arg("param5");
    Serial.print("#");
    Serial.print(param5);
  }
  Serial.print("\n");
  
  // wait for reply if needed
  if (cmd == "P")
  {
    String rep = "No reply from STM32";
    int timeout = 1000;
    while (!Serial.available() && timeout>0)
    {
      delay(1);
      timeout -= 1;
    }
    if (Serial.available())
      rep = Serial.readString();
    server.send(200, "text/plain", rep);
  }
  else
  {
    // no reply is expected from STM32
    server.send(200, "text/plain", "OK!");
  }
  return;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);

  WiFi.setSleepMode(WIFI_NONE_SLEEP);  // important for stable latency

  // ---- STATIC IP: 192.168.50.<Mechalino_ID> ----
  IPAddress localIP(192, 168, 50, Mechalino_ID);
  IPAddress gateway(192, 168, 50, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.config(localIP, gateway, subnet);
  // --------------------------------------------

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  delay(1500);

  server.on("/cmd", HTTP_GET, handleCMD);
  server.begin();
  delay(100);
  serialBuf.reserve(256);
  ensurePoseClientConnected();

  udp.begin(UDP_PORT);
}

void loop() {
  server.handleClient();
  
  ensurePoseClientConnected();

  static uint32_t last_yield = 0;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      serialBuf.trim();
      if (serialBuf.indexOf("POSE?") >= 0) {
        float x, y, yaw;
        uint8_t e = requestPoseFromROS(x, y, yaw);
        if (e == 0) sendPoseToSTM32(x, y, yaw);
        else {
          Serial.print("U#E");
          Serial.print(e);
          Serial.print("\n");
        }
      }
      else if (serialBuf.startsWith("BPOS#")) {
        // Broadcast everything after "BPOSE#"
        const char *payload = serialBuf.c_str() + 5; // skip "BPOS#"

        udp.beginPacket(IPAddress(255,255,255,255), UDP_PORT);
        udp.write((const uint8_t*)payload, strlen(payload));
        udp.endPacket();
      }
      serialBuf = "";
    } else {
      serialBuf += c;
      if (serialBuf.length() > 240) serialBuf = "";
    }
    
    if ((millis() - last_yield) > 5) {  // every ~5 ms
      last_yield = millis();
      yield();
    }
  }

  /* ---------- UDP RX ---------- */
  static uint32_t last_opos_fwd = 0;
  int pkt_len = udp.parsePacket();
  if (pkt_len > 0) {
      if (pkt_len >= (int)sizeof(udp_rx_buf))
          pkt_len = sizeof(udp_rx_buf) - 1;

      udp.read(udp_rx_buf, pkt_len);
      udp_rx_buf[pkt_len] = '\0';

      if (millis() - last_opos_fwd > 500) { // max 2 Hz forwarding
        last_opos_fwd = millis();
        Serial.print("OPOS#");
        Serial.println(udp_rx_buf);
      }
  }
  yield();
}
