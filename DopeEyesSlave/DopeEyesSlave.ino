#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <map>
#include <string>
#include <sstream>
#include <ESP32Servo.h>
#include <math.h>
#include <FastLED.h>

#define PUPIL_LED_COUNT 15
#define PUPIL_PIN 18


class WiFiConnector {
  private:
      const char* ssid;
      const char* password;
      unsigned long timeout;
      unsigned long previousMillis;
      bool connected;
      
      void connectToWiFi() {
          Serial.print("Connecting to WiFi");
          WiFi.begin(ssid, password);
          if (WiFi.status() == WL_CONNECTED) {
              if (!connected) {
                  Serial.println("Connected to WiFi!");
                  connected = true;
              }
          } else {
            if(connected){
              Serial.println("Disconnected from WiFi");
            }
            connected = false;
          }
      }

  public:
      WiFiConnector(const char* ssid, const char* password, unsigned long timeout) 
          : ssid(ssid), password(password), timeout(timeout), connected(false), previousMillis(0) {}

      void begin() {
          connectToWiFi();
      }

      void handle() {
          unsigned long currentMillis = millis();
          if(connected){

          } else {
            if (currentMillis - previousMillis >= timeout) {
                previousMillis = currentMillis;
                Serial.println("Attempting to reconnect...");
                connectToWiFi();
            }
          }


      }

      bool isConnected() {
          return WiFi.status() == WL_CONNECTED;
      }

      void setReconnectTimeout(unsigned long newTimeout) {
          timeout = newTimeout;
      }
};

class Config {
  public:
    std::map<std::string, double> properties;

    Config() {
      properties["blinkFrequency"] = 1000.0; // Initial frequency in milliseconds
      properties["eyelidPosition"] = 0; //eyelid position
      properties["Upperlid.Limit.High"] = 170;
      properties["Upperlid.Limit.Low"] = 30;
      properties["Upperlid.Center"] = 110;
      properties["Upperlid.Setpoint"] = 110;
      properties["Upperlid.MaxSpeed"] = 1000;
      properties["Lowerlid.Limit.High"] = 145;
      properties["Lowerlid.Limit.Low"] = 0;
      properties["Lowerlid.Center"] = 70;
      properties["Lowerlid.Setpoint"] = 70;
      properties["Lowerlid.MaxSpeed"] = 1000;
      properties["EyeLR.Limit.High"] = 180;
      properties["EyeLR.Limit.Low"] = 0;
      properties["EyeLR.Center"] = 90;
      properties["EyeLR.Setpoint"] = 90;
      properties["EyeLR.MaxSpeed"] = 1000;
      properties["EyeUD.Limit.High"] = 180;
      properties["EyeUD.Limit.Low"] = 35;
      properties["EyeUD.Center"] = 105;
      properties["EyeUD.Setpoint"] = 105;
      properties["EyeUD.MaxSpeed"] = 1000;
    }

    double getProperty(const std::string& name) {
      return properties[name];
    }

    void setProperty(const std::string& name, double value) {
      properties[name] = value;
    }

    std::string getHTMLForm() {
        std::ostringstream html;
        html << "<html><head>";
        html << "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"; // Ensure the page is responsive
        html << "<style>";
        html << "body { font-family: Arial, sans-serif; background-color: #fff; color: #000; text-align: center; font-size: 18px; }"; // Increased base font size
        html << "h1 { color: #333; width: 90%; margin: 20px auto; font-size: 24px; }"; // Increased font size for h1
        html << "form { display: inline-block; text-align: left; width: 90%; margin: 0 auto; }";
        html << "label { display: block; margin-bottom: 15px; font-weight: bold; font-size: 20px; width: 90%; margin: 0 auto; }"; // Increased font size for labels
        html << "input[type=text] { width: 90%; padding: 12px; margin: 10px auto; box-sizing: border-box; display: block; font-size: 18px; }"; // Increased padding and font size for inputs
        html << "input[type=submit] { background-color: #000; color: #fff; padding: 15px 20px; border: none; cursor: pointer; width: 90%; margin: 15px auto; display: block; font-size: 18px; }"; // Increased padding and font size for submit button
        html << "input[type=submit]:hover { background-color: #333; }";
        html << ".section-comment { margin-top: 20px; font-style: italic; color: #666; width: 90%; margin: 0 auto; font-size: 16px; }"; // Updated for 90% width and font size
        html << "svg { width: 50%; margin: 20px auto; display: block; }"; // SVG adjusted to 50% width and centered
        html << "svg path { stroke: #000; stroke-width: 1; fill: none; stroke-dasharray: 1000; stroke-dashoffset: 1000; animation: draw 5s forwards; }";
        html << "@keyframes draw { to { stroke-dashoffset: 0; } }";
        html << "</style>";
        html << "</head><body>";

        // Insert SVG animation
        html << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 150 126\">";
        html << "<g transform=\"translate(-28.34617, -67.34671)\">";
        html << "<path d=\"M46.648479 131.26477v13.51339h-0.003v17.82217H159.91391V144.77816H64.562629V131.26477ZM126.77385 99.98706h18.61959v18.63263h-18.61959zm-62.580031 0h18.61959v18.63263H64.193819ZM28.346749 67.346711c-0.002 41.819719 0.002 84.474009 0 126.000059h0.0486 149.900931V72.846631h0.0501l-0.0501 -5.49992zm5.49992 5.49992H172.79633V187.84685H33.846669Z\" fill=\"black\"/>";
        html << "</g></svg>";

        html << "<h1>DopeEye Slave Configuration</h1>";
        html << "<form action=\"/setConfig\" method=\"POST\">";

        for (const auto& prop : properties) {
            html << "<label>" << prop.first << ":</label>";
            html << "<input type=\"text\" name=\"" << prop.first << "\" value=\"" << prop.second << "\"><br>";
        }

        html << "<input type=\"submit\" value=\"Set\">";
        html << "</form>";
        html << "</body></html>";
        return html.str();
    }



};

class SlaveServerHandler {
  private:
    WebServer server;
    DNSServer dnsServer;  // Add DNS server for captive portal
    Config* config;
    String ssid;
    String pw;
    IPAddress local_IP = IPAddress(192, 168, 4, 1);
    IPAddress gateway = IPAddress(192, 168, 4, 1);
    IPAddress subnet = IPAddress(255, 255, 255, 0);

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
    SlaveServerHandler(int port, Config* config, String ssid, String pw) : server(port), config(config), ssid(ssid), pw(pw) {}

    void begin() {
      server.on("/", HTTP_GET, std::bind(&SlaveServerHandler::handleRoot, this));
      server.on("/config", HTTP_GET, std::bind(&SlaveServerHandler::handleConfig, this));
      server.on("/setConfig", HTTP_POST, std::bind(&SlaveServerHandler::handleSetConfig, this));
      
      // Set up the Wi-Fi as an Access Point
      WiFi.softAP("DopeEye", "12345678");
      WiFi.softAPConfig(local_IP, gateway, subnet);
      WiFi.setSleep(false);

      // Start DNS server and redirect all queries to the AP IP address
      dnsServer.start(53, "*", local_IP);

      // Start the HTTP server
      server.begin();
      Serial.println("HTTP server started");
    }

    void handleClient() {
      dnsServer.processNextRequest(); // Process DNS requests
      server.handleClient(); // Handle HTTP requests
    }

    IPAddress getIP(){
      return local_IP;
    }
};

class LimitedServo {
  private:
    Servo servo;
    int pin;
    int lowerLimit;
    int upperLimit;
    int center;
    int currentAngle;       // Stores the current angle of the servo
    int targetAngle;        // Stores the target angle
    int maxVelocity;        // Maximum velocity (degrees per update cycle)
    unsigned long lastUpdateTime;  // Last time the servo was updated

  public:
    LimitedServo(int pin, int lowerLimit, int center, int upperLimit, int maxVelocity)
      : pin(pin), lowerLimit(lowerLimit), center(center), upperLimit(upperLimit), 
        currentAngle(0), targetAngle(0), maxVelocity(maxVelocity), lastUpdateTime(0) {
          currentAngle = center;
          targetAngle = center;
        }

    void attach() {
        servo.attach(pin);
        currentAngle = center; // Initialize the current angle
        targetAngle = center;  // Start with the current angle as the target
    }

    void detach() {
        servo.detach();
    }

    void write(int angle) {
        targetAngle = constrain(angle, lowerLimit, upperLimit); // Set the new target angle within the limits
    }

    int read() {
        return currentAngle;
    }

    void setUpperLimit(int uLim){
      upperLimit = uLim;
    }

    void setLowerLimit(int lLim){
      lowerLimit = lLim;
    }

    void setMaxVelocity(int maxVel){
      maxVelocity = maxVel;
    }

    int getCenter(){
      return center;
    }

    void update() {
        unsigned long currentTime = millis();
        unsigned long timeElapsed = currentTime - lastUpdateTime;

        int step = maxVelocity * timeElapsed / 1000; // Calculate how much to move based on velocity and time
        if (currentAngle < targetAngle) {
            currentAngle = min(currentAngle + step, targetAngle);
        } else if (currentAngle > targetAngle) {
            currentAngle = max(currentAngle - step, targetAngle);
        }

        servo.write(constrain(currentAngle, lowerLimit, upperLimit));
        lastUpdateTime = currentTime;
    }

    // Debug print method
    void debugPrint() {
        Serial.print("Pin: ");
        Serial.print(pin);
        Serial.print(", Current Angle: ");
        Serial.print(currentAngle);
        Serial.print(", Target Angle: ");
        Serial.print(targetAngle);
        Serial.print(", Lower Limit: ");
        Serial.print(lowerLimit);
        Serial.print(", Upper Limit: ");
        Serial.print(upperLimit);
        Serial.print(", Max Velocity: ");
        Serial.println(maxVelocity);
    }
};

class Eyelids {
  private:
    LimitedServo& upperLid;
    LimitedServo& lowerLid;

    int center;   // Center in degrees, relative to 0° (straight forward)
    int aperture; // Aperture in degrees, 0° to 180°

  public:
    // Constructor that takes references to the upper and lower lid servos
    Eyelids(LimitedServo& upper, LimitedServo& lower) 
      : upperLid(upper), lowerLid(lower), center(0), aperture(0) {} // Initialize with default center and aperture

    void attach(){
      upperLid.attach();
      lowerLid.attach();
    }

    // Set the center and aperture, adjusting the lid positions accordingly
    void setCenterAndAperture(int newCenter, int newAperture) {
        // Clamp the input values to ensure they are within valid ranges
        center = constrain(newCenter, -90, 90);
        aperture = constrain(newAperture, 0, 180);

        // Calculate the absolute positions based on the center and aperture
        int upperLidPosition = upperLid.getCenter() - (aperture / 2) - center;
        int lowerLidPosition = lowerLid.getCenter() + (aperture / 2) + center;

        // Ensure upper lid is always above lower lid to avoid collision
        // if (upperLidPosition < lowerLidPosition) {
        //     int average = (upperLidPosition + lowerLidPosition) / 2;
        //     upperLidPosition = average;
        //     lowerLidPosition = average;
        // }
        Serial.print("upperLidPosition");
        Serial.print(upperLidPosition);
        Serial.print("lowerLidPosition");
        Serial.println(lowerLidPosition);
        upperLid.write(upperLidPosition);
        lowerLid.write(lowerLidPosition);
    }

    // Get the current center and aperture as a pair
    std::pair<int, int> getCenterAndAperture() {
        int upperLidPosition = upperLid.read();
        int lowerLidPosition = lowerLid.read();
        int currentCenter = (upperLidPosition + lowerLidPosition) / 2 - upperLid.getCenter();
        int currentAperture = upperLidPosition - lowerLidPosition;
        return {currentCenter, currentAperture};
    }

    // Update function to be called in the loop to ensure smooth movement
    void update() {
        upperLid.update();
        lowerLid.update();
    }

    // Convenience functions to move eyelids to fully open or closed positions
    void openEyelids() {
        setCenterAndAperture(center, 180); // Maximum aperture
    }

    void closeEyelids() {
        setCenterAndAperture(center, 0); // Minimum aperture
    }

    // Set the eyelids' positions directly, ensuring they do not collide
    void setPosition(int upperLidPosition, int lowerLidPosition) {
        // Ensure that the positions do not cause a collision
        if (upperLidPosition < lowerLidPosition) {
            int average = (upperLidPosition + lowerLidPosition) / 2;
            upperLidPosition = average;
            lowerLidPosition = average;
        }

        upperLid.write(upperLidPosition);
        lowerLid.write(lowerLidPosition);
    }
};

class BlinkLED {
  private:
    int ledPin;
    unsigned long lastBlinkTime;
    unsigned long onTime;
    unsigned long offTime;
    bool ledState;

  public:
    // Constructor to initialize the pin and default settings
    BlinkLED(int pin) {
      ledPin = pin;
      pinMode(ledPin, OUTPUT);
      lastBlinkTime = 0;
      onTime = 500; // default on-time in ms
      offTime = 500; // default off-time in ms
      ledState = LOW;
      digitalWrite(ledPin, ledState);
    }

    // Method to update the blink rate and duty cycle
    void setBlinkRate(unsigned long frequency, float dutyCycle) {
      // Frequency is in Hz, dutyCycle is a percentage (0.0 to 1.0)
      unsigned long period = 1000 / frequency; // total period in ms
      onTime = period * dutyCycle;
      offTime = period - onTime;
    }

    // Method to update the LED state, should be called periodically
    void update() {
      unsigned long currentMillis = millis();
      if (ledState && currentMillis - lastBlinkTime >= onTime) {
        ledState = LOW;
        lastBlinkTime = currentMillis;
        digitalWrite(ledPin, ledState);
      } else if (!ledState && currentMillis - lastBlinkTime >= offTime) {
        ledState = HIGH;
        lastBlinkTime = currentMillis;
        digitalWrite(ledPin, ledState);
      }
    }
};

class Pupil {
  private:
    CRGB *leds;
    uint8_t gradientOffset = 0;  // Offset to create the rotating gradient

  public:
    // Enum for different modes
    enum Mode {
        STATIC_COLOR,
        NOISY_COLOR,
        GRADIENT,
        NOISE
    };

    // Constructor to initialize the Pupil with the number of LEDs and the pin
    Pupil() {
      leds = new CRGB[PUPIL_LED_COUNT];
      FastLED.addLeds<WS2812, PUPIL_PIN, GRB>(leds, PUPIL_LED_COUNT);
      FastLED.show();
    }

    // Update function to apply different modes
    void update(Mode mode, uint16_t params[8] = nullptr) {
      uint16_t parameters[8] = {0};

      // Copy provided parameters, if any
      if (params != nullptr) {
        for (int i = 0; i < 8; i++) {
          parameters[i] = params[i];
        }
      }

      switch (mode) {
        case STATIC_COLOR: {
          // Static color mode
          CRGB color = CRGB(parameters[0], parameters[1], parameters[2]);
          fill_solid(leds, PUPIL_LED_COUNT, color);
          break;
        }
        case NOISY_COLOR: {
          // Noisy color mode
          CRGB baseColor = CRGB(parameters[0], parameters[1], parameters[2]);
          for (int i = 0; i < PUPIL_LED_COUNT; i++) {
            // Apply slight randomization to the color
            int red = constrain(baseColor.r + random8(-parameters[3], parameters[3]), 0, 255);
            int green = constrain(baseColor.g + random8(-parameters[3], parameters[3]), 0, 255);
            int blue = constrain(baseColor.b + random8(-parameters[3], parameters[3]), 0, 255);

            leds[i] = CRGB(red, green, blue);
          }
          break;
        }
        case GRADIENT: {
          // Gradient mode with rotating gradient
          CRGB startColor = CRGB(parameters[0], parameters[1], parameters[2]);
          CRGB endColor = CRGB(parameters[3], parameters[4], parameters[5]);
          uint8_t speed = parameters[6];  // Speed of the gradient rotation

          // Calculate the gradient and apply rotation
          for (int i = 0; i < PUPIL_LED_COUNT; i++) {
            // Calculate the position within the gradient
            uint8_t colorIndex = (i + gradientOffset) % PUPIL_LED_COUNT;
            uint8_t gradientPosition = (colorIndex * 255) / PUPIL_LED_COUNT;
            leds[i] = blend(startColor, endColor, gradientPosition);
          }

          // Update the gradient offset for rotation
          gradientOffset = (gradientOffset + speed) % PUPIL_LED_COUNT;

          break;
        }
        case NOISE: {
          // Noise mode
          for (int i = 0; i < PUPIL_LED_COUNT; i++) {
            leds[i] = CHSV(random8(), parameters[0], parameters[1]);
          }
          break;
        }
        default:
          // Unknown mode, do nothing
          break;
      }

      FastLED.show();
    }

    // Destructor to free the allocated memory
    ~Pupil() {
      delete[] leds;
    }
};

const char* ap_ssid = "DopeEye";
const char* ap_password = "12345678";
const char* master_ssid = "DopeEyeMaster";
const char* master_password = "87654321!";
const int ledPin = 2;  // LED connected to GPIO2 (D2)

Config config; 
SlaveServerHandler slaveServer(80, &config, ap_ssid, ap_password);

WiFiConnector wifi(master_ssid, master_password, 100000);

LimitedServo servoUpperLid = LimitedServo(27, 30, 110, 170, 1000);
LimitedServo servoLowerLid = LimitedServo(12, 0, 70, 145, 1000);
LimitedServo servoEyeLR = LimitedServo(13, 0, 90, 180, 1000);
LimitedServo servoEyeUD = LimitedServo(14, 35, 105, 180, 1000);

Eyelids eyelids = Eyelids(servoUpperLid, servoLowerLid);
Pupil pupil = Pupil();

BlinkLED blinkled = BlinkLED(ledPin);

int currentMillis = millis();
int lastServoUpdateTime = millis();
int lastEyelidUpdate = millis();
int servoUpdateInterval = 20;
int eyelidUpdateInterval = 20;

unsigned long startTime;

void setup() {
  Serial.begin(115200);
  delay(100);
  pinMode(ledPin, OUTPUT);

  Serial.println("Setting AP (Access Point)...");
  WiFi.softAP(ap_ssid, ap_password);

  slaveServer.begin();
  IPAddress IP = slaveServer.getIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  blinkled.setBlinkRate(2, .7);

  // wifi.begin();

  // eyelids.attach();
  // eyelids.setCenterAndAperture(0, 25); // Initial settings
  // eyelids.update();

  startTime = millis();

  //Init Servos
  servoEyeLR.attach();
  servoEyeUD.attach();

}

void loop() {
  static unsigned long lastBlinkTime = 0;
  currentMillis = millis();
  blinkled.update();
  uint16_t pupilParams[8] = {0, 255, 0, 255, 100, 0, 50};
  // pupil.update(Pupil::NOISY_COLOR, pupilParams);
  slaveServer.handleClient();
  Serial.println("hallo");
  

  if (currentMillis - lastServoUpdateTime >= servoUpdateInterval) {
    lastServoUpdateTime = currentMillis;
    servoEyeLR.write(config.getProperty("EyeLR.Setpoint"));
    servoEyeLR.update();
    servoEyeUD.write(config.getProperty("EyeUD.Setpoint"));
    servoEyeUD.update();
  }



  //   // Check if it's time to update the eyelids
  // if (currentMillis - lastEyelidUpdate >= eyelidUpdateInterval) {
  //   lastEyelidUpdate = currentMillis; // Save the last time we updated

  //   float elapsedTime = (currentMillis - startTime) / 1000.0; // Time in seconds

  //   // Sine wave for center: moves between -20 and +20 degrees
  //   int center = 20 * sin(2 * PI * 0.1 * elapsedTime); // 0.1 Hz sine wave

  //   // Triangular wave for aperture: varies between 20 and 30 degrees
  //   float frequency = 0.1; // Frequency of the triangular wave
  //   int aperture = 25 + 5 * (2 * abs(2 * (elapsedTime * frequency - floor(elapsedTime * frequency + 0.5))) - 1);

  //   // Set the eyelids position
  //   eyelids.setCenterAndAperture(center, aperture);
  //   Serial.print("Center: ");
  //   Serial.print(center);
  //   Serial.print(", Aperture: ");
  //   Serial.println(aperture);
  //   eyelids.setCenterAndAperture(0, 45);

  //   // Update the servos to the new position
  //   eyelids.update();
  // }

  // wifi.handle();
  // if(wifi.isConnected()){
  //   //wait for commands from master through webserver methods
  // }
  // else{
  //   //play standalone animations and wait for master to connect
  // }
}
