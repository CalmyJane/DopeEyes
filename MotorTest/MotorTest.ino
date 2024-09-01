#include <WebServer.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <ESP32Servo.h>
#include <map>
#include <string>
#include <sstream>
#include <WiFi.h>
#include <Arduino.h>
#include <cmath>
#include <FastLED.h>

#define PUPIL_LED_COUNT 41  // Number of LEDs in the pupil
#define PUPIL_PIN 18        // Pin connected to the LED strip

class Pupil {
  private:
    CRGB leds[PUPIL_LED_COUNT];
    CRGB targetColors[PUPIL_LED_COUNT];  // Target colors for the smooth transition
    uint8_t gradientOffset = 0;          // Offset to create the rotating gradient
    bool stroboState = false;            // State to track strobe on/off
    unsigned long lastStrobeTime = 0;    // Last time the strobe was toggled

    // New variables for rotation
    float rotationOffset = 0;            // Current rotation position with fractional precision
    unsigned long lastUpdateTime = 0;    // Last time update() was called

  public:
    // Enum for different modes
    enum Mode {
        STATIC_COLOR,
        NOISY_COLOR,
        GRADIENT,
        NOISE,
        RAINBOW
    };

    // Constructor
    Pupil() {}

    // Initialize method to be called in setup
    void begin() {
      FastLED.addLeds<WS2812, PUPIL_PIN, GRB>(leds, PUPIL_LED_COUNT);
      FastLED.show();
      lastUpdateTime = millis();  // Initialize lastUpdateTime
    }

    // Update function to apply different modes
    void update(Mode mode, uint16_t params[8] = nullptr, uint8_t brightness = 255, uint16_t strobe = 0, float rotation = 0) {
      unsigned long currentTime = millis();
      float elapsedTime = (currentTime - lastUpdateTime) / 1000.0;  // Convert to seconds
      lastUpdateTime = currentTime;

      // Update rotation offset
      rotationOffset += rotation * elapsedTime;
      while (rotationOffset >= PUPIL_LED_COUNT) rotationOffset -= PUPIL_LED_COUNT;
      while (rotationOffset < 0) rotationOffset += PUPIL_LED_COUNT;

      uint16_t parameters[8] = {0};
      FastLED.setBrightness(brightness);

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
          for (int i = 0; i < PUPIL_LED_COUNT; i++) {
            leds[i] = color;
          }
          break;
        }
        case NOISY_COLOR: {
          // Noisy color mode with smooth transition (wobble)
          CRGB baseColor = CRGB(parameters[0], parameters[1], parameters[2]);
          uint8_t noiseRange = parameters[3];

          for (int i = 0; i < PUPIL_LED_COUNT; i++) {
            if (leds[i] == targetColors[i]) {
              int red = constrain(baseColor.r + random8(-noiseRange, noiseRange), 0, 255);
              int green = constrain(baseColor.g + random8(-noiseRange, noiseRange), 0, 255);
              int blue = constrain(baseColor.b + random8(-noiseRange, noiseRange), 0, 255);

              targetColors[i] = CRGB(red, green, blue);
            }

            leds[i] = leds[i].lerp8(targetColors[i], 8);  // 8 is the blend factor (higher is slower)
          }
          break;
        }
        case GRADIENT: {
          // Gradient mode with rotating gradient
          CRGB startColor = CRGB(parameters[0], parameters[1], parameters[2]);
          CRGB endColor = CRGB(parameters[3], parameters[4], parameters[5]);

          for (int i = 0; i < PUPIL_LED_COUNT; i++) {
            uint8_t colorPosition = map(i, 0, PUPIL_LED_COUNT - 1, 0, 255);
            colorPosition = (colorPosition + gradientOffset) % 255;
            CRGB color = blend(startColor, endColor, colorPosition);
            leds[i] = color;
          }
          break;
        }
        case NOISE: {
          // Noise mode
          uint8_t hueRange = parameters[0];
          uint8_t saturation = parameters[1];
          uint8_t brightnessParam = parameters[2];

          for (int i = 0; i < PUPIL_LED_COUNT; i++) {
            uint8_t hue = random8(hueRange);
            leds[i] = CHSV(hue, saturation, brightnessParam);
          }
          break;
        }
        case RAINBOW: {
          // Rainbow mode
          uint8_t speed = parameters[0];  // Speed of the rainbow transition

          gradientOffset = (gradientOffset + speed) % 255;

          for (int i = 0; i < PUPIL_LED_COUNT; i++) {
            uint8_t hue = ((i * 255 / PUPIL_LED_COUNT) + gradientOffset) % 255;
            leds[i] = CHSV(hue, 255, 255);
          }
          break;
        }
        default:
          // Unknown mode, do nothing
          break;
      }

      // Apply smooth rotation
      applySmoothRotation();

      // Strobe effect
      if (strobe > 0) {
        unsigned long interval = 1000 / strobe;  // Calculate interval in milliseconds based on strobe frequency in Hz

        if (currentTime - lastStrobeTime >= interval) {
          stroboState = !stroboState;             // Toggle the strobe state
          lastStrobeTime = currentTime;           // Update the last strobe time
        }

        if (!stroboState) {
          fill_solid(leds, PUPIL_LED_COUNT, CRGB::Black);  // Turn off the LEDs (black) for strobe
        }
      }

      FastLED.show();
    }

    // Smooth rotation function
    void applySmoothRotation() {
      CRGB tempLeds[PUPIL_LED_COUNT];
      for (int i = 0; i < PUPIL_LED_COUNT; i++) {
        float fractionalIndex = (i + rotationOffset);
        int index = (int)fractionalIndex;
        float fraction = fractionalIndex - index;

        CRGB color1 = leds[index % PUPIL_LED_COUNT];
        CRGB color2 = leds[(index + 1) % PUPIL_LED_COUNT];

        tempLeds[i] = blend(color1, color2, (int)(fraction * 255));
      }
      for (int i = 0; i < PUPIL_LED_COUNT; i++) {
        leds[i] = tempLeds[i];
      }
    }
};

enum WaveType {
    SINE,
    TRIANGLE,
    SQUARE,
    NOISE,
    SMOOTH_NOISE,
    SAMPLE_AND_HOLD
};

class Fgen {
  private:
    WaveType waveType;
    float frequency;
    float minValue;
    float maxValue;
    unsigned long startTime;
    float smoothNoiseValue;
    float sampleAndHoldValue;
    unsigned long sampleHoldTime;
    unsigned long lastSampleTime;

  public:
    Fgen(WaveType type, float freq, float minVal, float maxVal)
      : waveType(type), frequency(freq), minValue(minVal), maxValue(maxVal),
        smoothNoiseValue(0), sampleAndHoldValue(minVal), sampleHoldTime(0), lastSampleTime(0) {
        startTime = millis();
    }

    float getValue() {
        unsigned long currentTime = millis();
        unsigned long elapsedTime = currentTime - startTime;
        float t = (elapsedTime / 1000.0f) * frequency; // Time in seconds multiplied by frequency
        float value = 0.0;

        switch (waveType) {
            case SINE:
                value = sin(2.0f * PI * t);
                value = mapToRange(value);
                break;
            
            case TRIANGLE:
                value = 2.0f * fabs(2.0f * (t - floor(t + 0.5f))) - 1.0f;
                value = mapToRange(value);
                break;
            
            case SQUARE:
                value = (sin(2.0f * PI * t) >= 0) ? 1.0f : -1.0f;
                value = mapToRange(value);
                break;

            case NOISE:
                value = 2.0f * ((float)rand() / RAND_MAX) - 1.0f; // Random value between -1 and 1
                value = mapToRange(value);
                break;

            case SMOOTH_NOISE: {
                // Calculate the interpolation parameter based on elapsed time
                float interpolationTime = 1.0f / frequency;  // Time interval between changes
                float timeFraction = fmod(elapsedTime / 1000.0f, interpolationTime) / interpolationTime;

                if (timeFraction == 0.0f) {
                    // Every 'interpolationTime', choose a new random target value within the range
                    float randomValue = 2.0f * ((float)rand() / RAND_MAX) - 1.0f;
                    smoothNoiseValue = mapToRange(randomValue);
                } else {
                    // Linearly interpolate between the current value and the new target value
                    float randomValue = 2.0f * ((float)rand() / RAND_MAX) - 1.0f;
                    float targetValue = mapToRange(randomValue);
                    value = smoothNoiseValue + (targetValue - smoothNoiseValue) * timeFraction;
                }
                break;
            }

            case SAMPLE_AND_HOLD: {
                if (currentTime - lastSampleTime > sampleHoldTime) {
                    // Time to sample a new value
                    sampleAndHoldValue = minValue + ((float)rand() / RAND_MAX) * (maxValue - minValue);
                    lastSampleTime = currentTime;
                    // Random time to hold the sample based on the frequency (converted to milliseconds)
                    sampleHoldTime = (unsigned long)((1000.0f / frequency) * ((float)rand() / RAND_MAX));
                }
                value = sampleAndHoldValue;
                break;
            }

            default:
                value = 0.0;
                break;
        }

        return value;
    }

  private:
    float mapToRange(float value) {
        // Map the value from -1 to 1 to the desired min and max range
        return minValue + (value + 1.0f) * 0.5f * (maxValue - minValue);
    }
};




class Config {
  public:
    std::map<std::string, double> properties;

    Config() {
      properties["MotorPin"] = 12;
      properties["MotorSetpoint"] = 90;

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

class WebserverHandler {
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
    WebserverHandler(int port, Config* config, String ssid, String pw) : server(port), config(config), ssid(ssid), pw(pw) {}

    void begin() {
      server.on("/", HTTP_GET, std::bind(&WebserverHandler::handleRoot, this));
      server.on("/config", HTTP_GET, std::bind(&WebserverHandler::handleConfig, this));
      server.on("/setConfig", HTTP_POST, std::bind(&WebserverHandler::handleSetConfig, this));
      
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
    int offset;
    bool inverted;
    int currentAngle;       // Stores the current angle of the servo
    int lastAngle;

  public:
    LimitedServo(int pin, int upperLimit, int lowerLimit, int offset, bool inverted = false)
      : pin(pin), lowerLimit(lowerLimit), upperLimit(upperLimit), offset(offset),
        currentAngle(0), inverted(inverted)
        {
          center = lowerLimit + (upperLimit - lowerLimit) / 2;
          currentAngle = center;
          lastAngle = -1;
          attach();
        }

    void attach() {
        servo.attach(pin);
        currentAngle = center; // Initialize the current angle
    }

    void detach() {
        servo.detach();
    }

    void write(int angle) {
        currentAngle = angle; // Set the new target angle within the limits
    }

    int read() {
        return currentAngle;
    }

    void setMax(int uLim){
      upperLimit = uLim;
    }

    void setMin(int lLim){
      lowerLimit = lLim;
    }

    void setOffset(int oset){
      offset = oset;
    }

    void setInverted(bool inv){
      inverted = inv;
    }

    void update() {
        //apply inversion (=180-x) and offset
        if(currentAngle != lastAngle){
          servo.write(constrain(getFinalAngle(), lowerLimit, upperLimit));
          lastAngle = currentAngle;
        }
    }

    int getFinalAngle(){
        return ((inverted?-1:0) * 180) - (currentAngle + offset) * (inverted?1:-1);
    }

    int getPin(){
      return pin;
    }

    // Debug print method
    void debugPrint() {
        Serial.print("Pin: ");
        Serial.print(pin);
        Serial.print(", Current Angle: ");
        Serial.print(currentAngle);
        Serial.print(", Lower Limit: ");
        Serial.print(lowerLimit);
        Serial.print(", Upper Limit: ");
        Serial.print(upperLimit);

    }
};


const char* ap_ssid = "DopeEye";
const char* ap_password = "12345678";
Config config; 
WebserverHandler webserver(80, &config, ap_ssid, ap_password);
Servo servo;
int servo_pin = 12;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(300);
  //Start Webserver and Access Point
  Serial.println("Setting AP (Access Point)...");
  webserver.begin();
  IPAddress IP = webserver.getIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  //Init Motor
  servo.attach(servo_pin);
  servo.write(90);
}

void loop() {
  // put your main code here, to run repeatedly:
    webserver.handleClient();
    int pin = config.getProperty("MotorPin");
    if(servo.attached() && pin != servo_pin){
      //new pin selected
      servo.detach();
      Serial.println("detached servo");
    }

    if(!servo.attached()){
      servo.attach(pin);
      servo_pin = pin;
      Serial.print("attached servo: ");
      Serial.println(pin);
    }


    servo.write(config.getProperty("MotorSetpoint"));
}