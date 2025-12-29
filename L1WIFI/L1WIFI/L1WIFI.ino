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

static void ensurePoseClientConnected() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (poseClient.connected()) return;

  poseClient.stop();
  poseClient.setNoDelay(true);
  poseClient.setTimeout(1);

  if (!poseClient.connect(laptopIP, POSE_PORT)) {
    // Connection failed; leave it disconnected and try again later
    poseClient.stop();
  }
}

static uint8_t requestPoseFromROS(float &x, float &y, float &yaw) {
  if (WiFi.status() != WL_CONNECTED) return 1;   // no WiFi

  WiFiClient c;
  c.setNoDelay(true);
  c.setTimeout(2);

  bool ok = false;
  for (int k = 0; k < 5; k++) {
    if (c.connect(laptopIP, POSE_PORT)) { ok = true; break; }
    c.stop();
    delay(10);
    yield();
  }
  if (!ok) return 2; // connect fail

  if (c.print("POSE\n") == 0) { c.stop(); return 3; } // write fail

  char buf[96];
  size_t idx = 0;
  uint32_t t0 = millis();
  const uint32_t deadline_ms = 500; // increase for real WiFi jitter

  while ((millis() - t0) < deadline_ms) {
    while (c.available() > 0) {
      char ch = (char)c.read();
      if (ch == '\r') continue;

      if (ch == '\n') {
        buf[idx] = 0;
        c.stop();

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
      else { c.stop(); return 6; } // line too long
    }
    delay(1);
  }

  c.stop();
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
  serialBuf.reserve(64);
  ensurePoseClientConnected();
}

void loop() {
  server.handleClient();
  
  ensurePoseClientConnected();

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
      serialBuf = "";
    } else {
      serialBuf += c;
      if (serialBuf.length() > 80) serialBuf = "";
    }
  }
  yield();
}
