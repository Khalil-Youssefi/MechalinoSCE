#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const char* ssid = "MechalinoAP";
const char* password = "12345679";
const int Mechalino_ID = 15;

ESP8266WebServer server(80);

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  delay(1500);
  
  server.on("/updatePose", HTTP_GET, handleUpdatePose);
  server.on("/getPose", HTTP_GET, handleGetPose);
  server.on("/cmd", HTTP_GET, handleCMD);
  server.on("/id", HTTP_GET, handleID);
  server.begin();
  delay(100);
}

void loop() {
  server.handleClient();
  yield();
}

void handleUpdatePose() {
  if (!server.hasArg("x") || !server.hasArg("y") || !server.hasArg("theta")) {
    server.send(400, "text/plain", "Error: required args are not provided.");
    return;
  }
  
  String x = server.arg("x");
  String y = server.arg("y");
  String theta = server.arg("theta");
  
  // Send to serial
  Serial.print("UPOSE#");
  Serial.print(x);
  Serial.print("#");
  Serial.print(y);
  Serial.print("#");
  Serial.println(theta);
  
  // Wait for response (max 2 seconds)
  unsigned long startTime = millis();
  String response = "";
  
  while (millis() - startTime < 2000) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (response.length() > 0) {
          break;
        }
      } else {
        response += c;
      }
    }
    yield();
  }
  
  // Check if response is valid (x#y#theta format)
  if (response.length() > 0) {
    int firstHash = response.indexOf('#');
    int secondHash = response.indexOf('#', firstHash + 1);
    
    if (firstHash > 0 && secondHash > firstHash + 1) {
      server.send(200, "text/plain", response);
      return;
    }
  }
  
  server.send(200, "text/plain", "-1");
}

void handleGetPose() {
  Serial.println("GPOSE#");
  
  // Wait for response (max 2 seconds)
  unsigned long startTime = millis();
  String response = "";
  
  while (millis() - startTime < 2000) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (response.length() > 0) {
          break;
        }
      } else {
        response += c;
      }
    }
    yield();
  }
  
  // Check if response is valid (x#y#theta format)
  if (response.length() > 0) {
    int firstHash = response.indexOf('#');
    int secondHash = response.indexOf('#', firstHash + 1);
    
    if (firstHash > 0 && secondHash > firstHash + 1) {
      server.send(200, "text/plain", response);
      return;
    }
  }
  
  server.send(200, "text/plain", "-1");
}

void handleCMD() {
  if (!server.hasArg("cmd") || !server.hasArg("param1")) {
    server.send(400, "text/plain", "Error: required args are not provided.");
    return;
  }
  
  String cmd = server.arg("cmd");
  String param1 = server.arg("param1");
  
  // Send to serial
  Serial.print("CMD#");
  Serial.print(cmd);
  Serial.print("#");
  if (server.hasArg("param2")){
    Serial.print(param1);
    Serial.print("#");
    String param2 = server.arg("param2");
    Serial.println(param2);
  }
  else
    Serial.println(param1);
  
  // Wait for response (max 2 seconds)
  unsigned long startTime = millis();
  String response = "";
  
  // reply
  server.send(200, "text/plain", "OK!");
  return;
}

void handleID() {
  // reply
  server.send(200, "text/plain", String(Mechalino_ID));
  return;
}