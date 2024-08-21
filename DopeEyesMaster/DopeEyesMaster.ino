#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include "DopeEyesSlave.ino"
#include <WiFi.h>
#include <map>
#include <string>
#include <sstream>

class Config {

  public:
    std::map<std::string, double> properties;

    Config() {
      properties["blinkFrequency"] = 1000.0; // Initial frequency in milliseconds
      properties["eyelidPosition"] = 0; //eyelid position
    }

    double getProperty(const std::string& name) {
      return properties[name];
    }

    void setProperty(const std::string& name, double value) {
      properties[name] = value;
    }

    std::string getHTMLForm() {
      std::ostringstream html;
      html << "<html><body>";
      html << "<h1>ESP32 Configuration</h1>";
      html << "<form action=\"/setConfig\" method=\"POST\">";
      for (const auto& prop : properties) {
        html << prop.first << ": <input type=\"text\" name=\"" << prop.first << "\" value=\"" << prop.second << "\"><br>";
      }
      html << "<input type=\"submit\" value=\"Set\">";
      html << "</form>";
      html << "</body></html>";
      return html.str();
    }
};

class MasterServerHandler {
  private:
    WebServer server;
    Config* config;

    void handleRoot() {
      server.sendHeader("Location", "/config");
      server.send(303);
    }

    void handleConfig() {
      std::string html = config->getHTMLForm();
      server.send(200, "text/html", html.c_str());
    }

    void handleSetConfig() {
      for (auto& prop : config->properties) {
        if (server.hasArg(prop.first.c_str())) {
          config->setProperty(prop.first, server.arg(prop.first.c_str()).toDouble());
        }
      }
      server.sendHeader("Location", "/config");
      server.send(303);
    }

  public:
    MasterServerHandler(int port, Config* config) : server(port), config(config) {}

    void begin() {
      server.on("/", HTTP_GET, std::bind(&MasterServerHandler::handleRoot, this));
      server.on("/config", HTTP_GET, std::bind(&MasterServerHandler::handleConfig, this));
      server.on("/setConfig", HTTP_POST, std::bind(&MasterServerHandler::handleSetConfig, this));
      server.begin();
      Serial.println("HTTP server started");
    }

    void handleClient() {
      server.handleClient();
    }
};

const char* ap_ssid = "DopeEye";
const char* ap_password = "12345678";
const int ledPin = 2;  // LED connected to GPIO2 (D2)

Config config; 
MasterServerHandler masterServer(80, &config);
bool ledState = false;

DNSServer dnsServer;

WiFiConnector wifi(master_ssid, master_password, 10000);

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);

  Serial.println("Setting AP (Access Point)...");
  WiFi.softAP(ap_ssid, ap_password);
  WiFi.softAPConfig(local_IP, gateway, subnet);

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

  masterServer.begin();
}

void loop() {
  static unsigned long lastBlinkTime = 0;

  unsigned long currentMillis = millis();
  if (currentMillis - lastBlinkTime >= config.getProperty("blinkFrequency")) {
    lastBlinkTime = currentMillis;
    ledState = !ledState;
    digitalWrite(ledPin, ledState ? HIGH : LOW);
  }

  masterServer.handleClient();
}
