#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

class Config {
public:
  double blinkFrequency;

  Config(double initialFrequency) : blinkFrequency(initialFrequency) {}
};

class WebServerHandler {
private:
  WebServer server;
  Config* config;

  void handleRoot() {
    server.sendHeader("Location", "/config");
    server.send(303);
  }

  void handleConfig() {
    String html = "<html><body>";
    html += "<h1>ESP32 LED Blink Frequency</h1>";
    html += "<form action=\"/setFrequency\" method=\"POST\">";
    html += "Frequency (ms): <input type=\"text\" name=\"frequency\">";
    html += "<input type=\"submit\" value=\"Set\">";
    html += "</form>";
    html += "</body></html>";
    server.send(200, "text/html", html);
  }

  void handleSetFrequency() {
    if (server.hasArg("frequency")) {
      config->blinkFrequency = server.arg("frequency").toDouble();
    }
    server.sendHeader("Location", "/config");
    server.send(303);
  }

public:
  WebServerHandler(int port, Config* config) : server(port), config(config) {}

  void begin() {
    server.on("/", HTTP_GET, std::bind(&WebServerHandler::handleRoot, this));
    server.on("/config", HTTP_GET, std::bind(&WebServerHandler::handleConfig, this));
    server.on("/setFrequency", HTTP_POST, std::bind(&WebServerHandler::handleSetFrequency, this));
    server.begin();
    Serial.println("HTTP server started");
  }

  void handleClient() {
    server.handleClient();
  }
};

const char* ssid = "ESP32_AP";
const char* password = "12345678";
const int ledPin = 2;  // LED connected to GPIO2 (D2)

Config config(1000); // Initial frequency in milliseconds
WebServerHandler webServerHandler(80, &config);
bool ledState = false;

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);

  Serial.println("Setting AP (Access Point)...");
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  if (!MDNS.begin("configuration")) {
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");

  webServerHandler.begin();
}

void loop() {
  static unsigned long lastBlinkTime = 0;

  unsigned long currentMillis = millis();
  if (currentMillis - lastBlinkTime >= config.blinkFrequency) {
    lastBlinkTime = currentMillis;
    ledState = !ledState;
    digitalWrite(ledPin, ledState ? HIGH : LOW);
  }

  webServerHandler.handleClient();
}
