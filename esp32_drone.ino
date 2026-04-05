#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>

// ─── WiFi & UDP ─────────────────────────────────────
const char* AP_SSID     = "DroneESP32";
const char* AP_PASSWORD = "drone1234";
const int   UDP_PORT    = 4210;

WiFiUDP udp;
char packetBuffer[256];

// ─── Drone Motor Pins & PWM ─────────────────────────
#define MOTOR_FL 13
#define MOTOR_FR 12
#define MOTOR_BR 14
#define MOTOR_BL 27

#define PWM_FREQ       50
#define PWM_RESOLUTION 16
#define PWM_MIN        3276
#define PWM_MAX        6553

int throttle = 0, pitch = 0, roll = 0, yaw = 0;
bool armed = false, emergency = false;

// ─── Web Server ────────────────────────────────────
WebServer server(80);

// ─── Helper Functions ──────────────────────────────
int percentToPWM(int percent) {
  percent = constrain(percent, 0, 100);
  return map(percent, 0, 100, PWM_MIN, PWM_MAX);
}

void updateMotors() {
  if (emergency || !armed) {
    ledcWrite(0, PWM_MIN); ledcWrite(1, PWM_MIN);
    ledcWrite(2, PWM_MIN); ledcWrite(3, PWM_MIN);
    return;
  }
  int fl = constrain(throttle + pitch + roll - yaw, 0, 100);
  int fr = constrain(throttle + pitch - roll + yaw, 0, 100);
  int br = constrain(throttle - pitch - roll - yaw, 0, 100);
  int bl = constrain(throttle - pitch + roll + yaw, 0, 100);

  ledcWrite(0, percentToPWM(fl));
  ledcWrite(1, percentToPWM(fr));
  ledcWrite(2, percentToPWM(br));
  ledcWrite(3, percentToPWM(bl));
}

void processCommand(String cmd) {
  cmd.trim(); cmd.toUpperCase();

  if (cmd == "ARM") { armed = true; emergency = false; }
  else if (cmd == "DISARM") { armed = false; throttle = pitch = roll = yaw = 0; }
  else if (cmd == "EMERGENCY") { emergency = true; armed = false; throttle = 0; }
  else if (cmd.startsWith("THROTTLE:")) throttle = constrain(cmd.substring(9).toInt(), 0, 100);
  else if (cmd.startsWith("PITCH:")) pitch = constrain(cmd.substring(6).toInt(), -100, 100);
  else if (cmd.startsWith("ROLL:")) roll = constrain(cmd.substring(5).toInt(), -100, 100);
  else if (cmd.startsWith("YAW:")) yaw = constrain(cmd.substring(4).toInt(), -100, 100);

  updateMotors();
}

// ─── Web Handlers ──────────────────────────────────
void handleRoot() {
  String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <title>ESP32 Drone Controller</title>
      <style>body{text-align:center;font-family:Arial;}</style>
    </head>
    <body>
      <h1>ESP32 Drone Controller</h1>
      <button onclick="sendCommand('ARM')">ARM</button>
      <button onclick="sendCommand('DISARM')">DISARM</button>
      <button onclick="sendCommand('EMERGENCY')">EMERGENCY</button>
      <br><br>
      Throttle: <input type="range" id="throttle" min="0" max="100" value="0" oninput="sendCommand('THROTTLE:'+this.value)">
      <br>
      Pitch: <input type="range" id="pitch" min="-100" max="100" value="0" oninput="sendCommand('PITCH:'+this.value)">
      <br>
      Roll: <input type="range" id="roll" min="-100" max="100" value="0" oninput="sendCommand('ROLL:'+this.value)">
      <br>
      Yaw: <input type="range" id="yaw" min="-100" max="100" value="0" oninput="sendCommand('YAW:'+this.value)">
      <script>
        function sendCommand(cmd) {
          fetch('/cmd?c=' + cmd);
        }
      </script>
    </body>
    </html>
  )rawliteral";
  server.send(200, "text/html", html);
}

void handleCommand() {
  if (server.hasArg("c")) {
    processCommand(server.arg("c"));
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Missing command");
  }
}

// ─── Setup PWM for Motors ──────────────────────────
void setupMotors() {
  ledcAttach(MOTOR_FL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_FR, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_BR, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_BL, PWM_FREQ, PWM_RESOLUTION);

  ledcWrite(0, PWM_MIN); ledcWrite(1, PWM_MIN);
  ledcWrite(2, PWM_MIN); ledcWrite(3, PWM_MIN);
  delay(3000);
}

// ─── Setup ────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  setupMotors();

  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.printf("AP SSID: %s | IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

  udp.begin(UDP_PORT);
  Serial.printf("UDP Port: %d\n", UDP_PORT);

  server.on("/", handleRoot);
  server.on("/cmd", handleCommand);
  server.begin();
  Serial.println("HTTP Server Started!");
}

// ─── Main Loop ────────────────────────────────────
void loop() {
  server.handleClient();

  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = '\0';
    String command = String(packetBuffer);
    processCommand(command);

    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    String response = armed ? "OK:ARMED" : "OK:DISARMED";
    if (emergency) response = "OK:EMERGENCY";
    udp.print(response);
    udp.endPacket();
  }

  delay(10);
}