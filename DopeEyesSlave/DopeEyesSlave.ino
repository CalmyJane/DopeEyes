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
#include <functional>
#include <Preferences.h>


#define PUPIL_LED_COUNT 41  // Number of LEDs in the pupil
#define PUPIL_PIN 18        // Pin connected to the LED strip

class Pupil {
  private:
  int params[8] = {0};  // Store integers directly
  int brightness = 255;
  int strobe = 0;
  float rotation = 0;

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
    void update() {
      unsigned long currentTime = millis();
      float elapsedTime = (currentTime - lastUpdateTime) / 1000.0;  // Convert to seconds
      lastUpdateTime = currentTime;

      // Update rotation offset
      rotationOffset += rotation * elapsedTime;
      while (rotationOffset >= PUPIL_LED_COUNT) rotationOffset -= PUPIL_LED_COUNT;
      while (rotationOffset < 0) rotationOffset += PUPIL_LED_COUNT;

      int parameters[8] = {0};
      FastLED.setBrightness(brightness);

      // Copy provided parameters, if any
      for (int i = 0; i < 8; i++) {
        parameters[i] = params[i];
      }

      switch (currentMode) {
        case STATIC_COLOR:
          // Static color mode
          {
            CRGB color = CRGB(parameters[0], parameters[1], parameters[2]);
            for (int i = 0; i < PUPIL_LED_COUNT; i++) {
              leds[i] = color;
            }
          }
          break;

        case NOISY_COLOR:
          // Noisy color mode with smooth transition (wobble)
          {
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
          }
          break;

        case GRADIENT:
          // Gradient mode with rotating gradient
          {
            CRGB startColor = CRGB(parameters[0], parameters[1], parameters[2]);
            CRGB endColor = CRGB(parameters[3], parameters[4], parameters[5]);

            for (int i = 0; i < PUPIL_LED_COUNT; i++) {
              uint8_t colorPosition = map(i, 0, PUPIL_LED_COUNT - 50, 0, 255);
              colorPosition = (colorPosition + gradientOffset) % 255;
              CRGB color = blend(startColor, endColor, colorPosition);
              leds[i] = color;
            }
          }
          break;

        case NOISE:
          // Noise mode
          {
            uint8_t hueRange = parameters[0];
            uint8_t saturation = parameters[1];
            uint8_t brightnessParam = parameters[2];

            for (int i = 0; i < PUPIL_LED_COUNT; i++) {
              uint8_t hue = random8(hueRange);
              leds[i] = CHSV(hue, saturation, brightnessParam);
            }
          }
          break;

        case RAINBOW:
          // Rainbow mode
          {
            uint8_t speed = parameters[0];  // Speed of the rainbow transition

            gradientOffset = (gradientOffset + speed) % 255;

            for (int i = 0; i < PUPIL_LED_COUNT; i++) {
              uint8_t hue = ((i * 255 / PUPIL_LED_COUNT) + gradientOffset) % 255;
              leds[i] = CHSV(hue, 255, 255);
            }
          }
          break;

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

      if(blackout){
        fill_solid(leds, PUPIL_LED_COUNT, CRGB::Black);
      }

      FastLED.show();
    }

    void setBlackout(bool blck){
      blackout = blck;
    }

    void setMode(Mode md){
      currentMode = md;
    }

    void changeMode() {
      // Generate a random index for the mode different from the current one
      Mode newMode;
      do {
        int randomIndex = random(0, sizeof(availableModes) / sizeof(availableModes[0]));
        newMode = availableModes[randomIndex];
      } while (newMode == currentMode);

      // Randomize some parameters for fun (these can be tailored as needed)
      int randomParams[8] = {
        255, 0, 0, // Random RGB or other values
        0, 0, 0, // Additional values
        random(0, 256), random(0, 256)                 // Additional values
      };
      int randomBrightness = random(2, 50);     // Random brightness between 50 and 255
      int randomStrobe = random(0, 20);          // Random strobe frequency
      float randomRotation = (float)random(-50, 50) / 100.0;  // Random rotation speed
      setMode(GRADIENT);
      setParams(randomParams);
      setBrightness(randomBrightness);
      setStrobe(0);
      setRotation(20);
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

    void setParams(int* parms){
      for (int i = 0; i < 8; i++) {
        params[i] = parms[i];
      }
    }

    void setBrightness(int brght){
      brightness = brght;
    }

    void setStrobe(int strb){
      strobe = strb;
    }

    void setRotation(float rot){
      rotation = rot;
    }

  private:
    CRGB leds[PUPIL_LED_COUNT];
    CRGB targetColors[PUPIL_LED_COUNT];  // Target colors for the smooth transition
    uint8_t gradientOffset = 0;          // Offset to create the rotating gradient
    bool stroboState = false;            // State to track strobe on/off
    unsigned long lastStrobeTime = 0;    // Last time the strobe was toggled

    // New variables for rotation
    float rotationOffset = 0;            // Current rotation position with fractional precision
    unsigned long lastUpdateTime = 0;    // Last time update() was called
    bool blackout = false;

    Mode currentMode = STATIC_COLOR;     // Track the current mode

    // Array of available modes
    Mode availableModes[5] = {STATIC_COLOR, NOISY_COLOR, GRADIENT, NOISE, RAINBOW};

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
    float dutyCycle;
    float uncertainty;
    unsigned long startTime;
    unsigned long lastFrequencyChangeTime;
    float currentFrequency;
    float smoothNoiseValue;
    float sampleAndHoldValue;
    unsigned long sampleHoldTime;
    unsigned long lastSampleTime;

  public:
    Fgen(WaveType type, float freq, float minVal, float maxVal, float duty = 0.5f, float uncert = 1.0f)
      : waveType(type), frequency(freq), minValue(minVal), maxValue(maxVal), dutyCycle(duty), uncertainty(uncert),
        currentFrequency(freq), smoothNoiseValue(0), sampleAndHoldValue(minVal), sampleHoldTime(0), lastSampleTime(0) {
        startTime = millis();
        lastFrequencyChangeTime = startTime;
    }

    float getValue() {
        unsigned long currentTime = millis();
        unsigned long elapsedTime = currentTime - startTime;

        // Calculate the period of the current frequency
        float period = 1000.0f / currentFrequency;

        // Check if it's time to change the frequency
        if (currentTime - lastFrequencyChangeTime >= period) {
            // Adjust frequency based on uncertainty
            currentFrequency = frequency * (1.0f + (uncertainty / 100.0f) * ((float)rand() / RAND_MAX - 0.5f) * 2.0f);
            if (currentFrequency < 0) currentFrequency = 0; // Ensure frequency doesn't go negative
            lastFrequencyChangeTime = currentTime;
        }

        float t = (elapsedTime / 1000.0f) * currentFrequency; // Time in seconds multiplied by current frequency
        float value = 0.0;

        switch (waveType) {
            case SINE:
                value = sin(2.0f * PI * t);
                value = mapToRange(value);
                break;
            
            case TRIANGLE:
                // Modify the triangle wave based on the duty cycle
                value = (t < dutyCycle) ? 
                    2.0f * (t / dutyCycle) - 1.0f : 
                    2.0f * ((1.0f - t) / (1.0f - dutyCycle)) - 1.0f;
                value = mapToRange(value);
                break;
            
            case SQUARE:
                // Modify the square wave based on the duty cycle
                value = (fmod(t, 1.0f) < dutyCycle) ? 1.0f : -1.0f;
                value = mapToRange(value);
                break;

            case NOISE:
                value = 2.0f * ((float)rand() / RAND_MAX) - 1.0f; // Random value between -1 and 1
                value = mapToRange(value);
                break;

            case SMOOTH_NOISE: {
                uint16_t timeInput = (uint16_t)(elapsedTime * currentFrequency); // Input for noise function
                uint8_t noiseValue = inoise8(timeInput);  // Perlin noise value [0, 255]
                value = map(noiseValue, 0, 255, minValue * 100, maxValue * 100) / 100.0f;  // Map to desired range
                break;
            }

            case SAMPLE_AND_HOLD: {
                if (currentTime - lastSampleTime > sampleHoldTime) {
                    // Time to sample a new value
                    sampleAndHoldValue = minValue + ((float)rand() / RAND_MAX) * (maxValue - minValue);
                    lastSampleTime = currentTime;
                    // Random time to hold the sample based on the current frequency (converted to milliseconds)
                    sampleHoldTime = (unsigned long)((1000.0f / currentFrequency) * ((float)rand() / RAND_MAX));
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

    void setFreq(float freq){
      frequency = freq;
    }

    void setDutyCycle(float duty) {
        dutyCycle = duty;
    }

    void setUncertainty(float uncert) {
        uncertainty = uncert;
    }

  private:
    float mapToRange(float value) {
        // Map the value from -1 to 1 to the desired min and max range
        return minValue + (value + 1.0f) * 0.5f * (maxValue - minValue);
    }
};

class Config {
  public:
    using ConfigChangeCallback = std::function<void()>;

  private:
    ConfigChangeCallback callback;
    Preferences preferences;  // Preferences object for NVS storage

    void notifyCallbacks() {
        if (callback) {
            callback();
        }
    }

  public:
    std::map<std::string, double> properties;

    // Constructor
    Config() {

    }

    void loadProperties(){
      // Initialize preferences
      if (!preferences.begin("config", false)) {
          Serial.println("Failed to initialize NVS");
      }

      // Load or set defaults for motor config
      properties["Motors.Toplid.Max"] = getOrDefault("Motors.Toplid.Max", 150);
      properties["Motors.Toplid.Min"] = getOrDefault("Motors.Toplid.Min", 40);
      properties["Motors.Toplid.Offset"] = getOrDefault("Motors.Toplid.Offset", 90);
      properties["Motors.Toplid.Inverted"] = getOrDefault("Motors.Toplid.Inverted", 0);

      properties["Motors.Bottomlid.Max"] = getOrDefault("Motors.Bottomlid.Max", 145);
      properties["Motors.Bottomlid.Min"] = getOrDefault("Motors.Bottomlid.Min", 45);
      properties["Motors.Bottomlid.Offset"] = getOrDefault("Motors.Bottomlid.Offset", 90);
      properties["Motors.Bottomlid.Inverted"] = getOrDefault("Motors.Bottomlid.Inverted", 0);

      properties["Motors.EyeLR.Max"] = getOrDefault("Motors.EyeLR.Max", 150);
      properties["Motors.EyeLR.Min"] = getOrDefault("Motors.EyeLR.Min", 30);
      properties["Motors.EyeLR.Offset"] = getOrDefault("Motors.EyeLR.Offset", 100);
      properties["Motors.EyeLR.Inverted"] = getOrDefault("Motors.EyeLR.Inverted", 0);

      properties["Motors.EyeUD.Max"] = getOrDefault("Motors.EyeUD.Max", 130);
      properties["Motors.EyeUD.Min"] = getOrDefault("Motors.EyeUD.Min", 40);
      properties["Motors.EyeUD.Offset"] = getOrDefault("Motors.EyeUD.Offset", 90);
      properties["Motors.EyeUD.Inverted"] = getOrDefault("Motors.EyeUD.Inverted", 0);

      // Other properties
      properties["Arperture"] = getOrDefault("Arperture", 90);
      properties["BlinkRate"] = getOrDefault("BlinkRate", 0.3);
      properties["EyelidsDamping"] = getOrDefault("EyelidsDamping", 1);
      properties["EyelidsOffset"] = getOrDefault("EyelidsOffset", 0);

      Serial.println("Config loaded successfully.");  
    }

    // Method to get a property or return default if not found in NVS
    double getOrDefault(const std::string& name, double defaultValue) {
      if (preferences.isKey(name.c_str())) {
        double value = preferences.getDouble(name.c_str(), defaultValue);
        Serial.print("Loaded ");
        Serial.print(name.c_str());
        Serial.print(": ");
        Serial.println(value);
        return value;
      } else {
        Serial.print("Default used for ");
        Serial.print(name.c_str());
        Serial.print(": ");
        Serial.println(defaultValue);
        return defaultValue;
      }
    }

    double getProperty(const std::string& name) {
      return properties[name];
    }

    // Method to set property and store in NVS
    void setProperty(const std::string& name, double value) {
      properties[name] = value;
      bool success = preferences.putDouble(name.c_str(), value);  // Store in NVS
      if (success) {
        Serial.print("Stored ");
        Serial.print(name.c_str());
        Serial.print(": ");
        Serial.println(value);
      } else {
        Serial.print("Failed to store ");
        Serial.println(name.c_str());
      }
      notifyCallbacks();
    }

    // Method to register callbacks
    void registerCallback(ConfigChangeCallback cb) {
      callback = cb;
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

    // Destructor
    ~Config() {
      preferences.end();  // Close the preferences to free up NVS
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
    int offset;
    bool inverted;
    int currentAngle;       // Stores the current angle of the servo

  public:
    LimitedServo(int pin, int upperLimit, int lowerLimit, int offset, bool inverted = false)
      : pin(pin), lowerLimit(lowerLimit), upperLimit(upperLimit), offset(offset),
        currentAngle(0), inverted(inverted)
        {
          center = lowerLimit + (upperLimit - lowerLimit) / 2;
          currentAngle = center;
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
        servo.write(constrain(getFinalAngle(), lowerLimit, upperLimit));
    }

    int getFinalAngle(){
        return ((inverted?-1:0) * 180) -  (currentAngle + offset) * (inverted?1:-1);
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

class Eyelids {
  private:
    int center;   // Center position for both eyelids
    int aperture; // Aperture of the eyelids
    int offset;

    int blinkState = 0;         // 0 = no blink, 1 = closing, 2 = waiting, 3 = opening
    unsigned long blinkStartTime = 0; // Time when the blink started
    int blinkDelayTime = 200;   // Duration of the blink (can be adjusted as needed)
    int previousAperture;       // To store the previous aperture before the blink

  public:
    LimitedServo topEyelid;
    LimitedServo bottomEyelid;
    Eyelids(int topPin, int bottomPin, int topMin, int topMax, int topOffset, bool topInverted,
            int bottomMin, int bottomMax, int bottomOffset, bool bottomInverted)
      : topEyelid(topPin, topMax, topMin, topOffset, topInverted),
        bottomEyelid(bottomPin, bottomMax, bottomMin, bottomOffset, bottomInverted) {}

    void setCenter(int newCenter) {
        center=newCenter;
        updateServos();
    }

    void setAperture(int newAperture) {
        aperture = constrain(newAperture, 0, 180);
        updateServos();
    }

    void setOffset(int offs){
      offset = offs;
    }

    void blink(int delayTime = 200) {
      if (blinkState == 0) {  // Only start a blink if one is not already in progress
          blinkState = 1;     // Start the blink process
          blinkDelayTime = delayTime;
          previousAperture = aperture;  // Save the current aperture
          setAperture(0);     // Close the eyelids
          blinkStartTime = millis();  // Record the start time
      }
    } 

    void updateServos() {
        // Calculate the angles for the top and bottom eyelids
        int topAngle = - (center + offset) - aperture / 2;
        int bottomAngle = center + offset - aperture / 2;

        // Write the angles to the servos
        topEyelid.write(topAngle);
        bottomEyelid.write(bottomAngle);
    }

    void update() {
        topEyelid.update();
        bottomEyelid.update();
    }

    void debugPrint() {
        Serial.println("Top Eyelid:");
        topEyelid.debugPrint();
        Serial.println("Bottom Eyelid:");
        bottomEyelid.debugPrint();
    }
    
    bool closed(){
      return aperture < 1;
    }

    void updateConfig(int topMax, int topMin, int topOffset, bool topInverted, int bottomMax, int bottomMin, int bottomOffset, bool bottomInverted){
      //TopEyelid update values
      topEyelid.setMin(topMin);
      topEyelid.setMax(topMax);
      topEyelid.setOffset(topOffset);
      topEyelid.setInverted(topInverted);
      //bottom eyelid update values
      bottomEyelid.setMin(bottomMin);
      bottomEyelid.setMax(bottomMax);
      bottomEyelid.setOffset(bottomOffset);
      bottomEyelid.setInverted(bottomInverted);
    }
};

class Channels {
  private:
    std::map<std::string, double> channelValues;

  public:
    // Set a channel value
    void setChannel(const std::string& name, double value) {
      channelValues[name] = value;
    }

    // Get a channel value, returns 0.0 if channel does not exist
    double getChannel(const std::string& name) const {
      auto it = channelValues.find(name);
      if (it != channelValues.end()) {
        return it->second;
      }
      return 0.0;
    }
};

const char* ap_ssid = "DopeEye";
const char* ap_password = "12345678";
Config config; 
SlaveServerHandler slaveServer(80, &config, ap_ssid, ap_password);
LimitedServo servoEyeLR = LimitedServo(13,
                                        config.getProperty("Motors.EyeLR.Max"),
                                        config.getProperty("Motors.EyeLR.Min"),
                                        config.getProperty("Motors.EyeLR.Offset"),
                                        config.getProperty("Motors.EyeLR.Inverted") > 0);
                                        
LimitedServo servoEyeUD = LimitedServo(14,
                                        config.getProperty("Motors.EyeUD.Max"),
                                        config.getProperty("Motors.EyeUD.Min"),
                                        config.getProperty("Motors.EyeUD.Offset"),
                                        config.getProperty("Motors.EyeUD.Inverted") > 0);

Fgen motorMove = Fgen(SINE, 0.1, -45, 45);
Fgen udMove = Fgen(SINE, 0.2, -25, 25);
Fgen smoothNoise = Fgen(TRIANGLE, 0.1, -100, 100);
Fgen arpertureMove = Fgen(SQUARE, config.getProperty("BlinkRate"), 0, 10, 0.9, 40);
Fgen arpertureMove2 = Fgen(SINE, config.getProperty("BlinkRate") * 0.37, 0, 90, 50);
Fgen arpertureMove3 = Fgen(SINE, config.getProperty("BlinkRate") * 0.73, -10, 10, 20);

Pupil myPupil;

bool wasClosed = false;

Eyelids eyelids = Eyelids(27, 12, config.getProperty("Motors.Toplid.Max"), config.getProperty("Motors.Toplid.Min"), 
                                  config.getProperty("Motors.Toplid.Offset"), config.getProperty("Motors.Toplid.Inverted"),
                                  config.getProperty("Motors.Bottomlid.Max"), config.getProperty("Motors.Bottomlid.Min"), 
                                  config.getProperty("Motors.Bottomlid.Offset"), config.getProperty("Motors.Bottomlid.Inverted"));
Channels channels;

void setup() {
  Serial.begin(115200);
  delay(100);
  config.loadProperties();

  slaveServer.begin();
  IPAddress IP = slaveServer.getIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  config.registerCallback([&]() { configUpdated(); });

  myPupil.begin();
  myPupil.setBrightness(10);
  configUpdated();
}

void loop() {
  slaveServer.handleClient();
  stimulateChannels();

  // Read motor channels from the Channels class
  servoEyeLR.write(channels.getChannel("MotorLR"));
  servoEyeUD.write(channels.getChannel("MotorUD"));
  servoEyeLR.update();
  servoEyeUD.update();

  // Pupil mode channel and parameters
  int pupilMode = static_cast<int>(channels.getChannel("PupilMode"));
  myPupil.setMode(static_cast<Pupil::Mode>(pupilMode));
  
  int params[8];
  for (int i = 0; i < 8; ++i) {
    params[i] = static_cast<int>(channels.getChannel("PupilParam" + std::to_string(i)));
  }
  myPupil.setParams(params);

  // Pupil brightness
  myPupil.setBrightness(static_cast<int>(channels.getChannel("PupilBrightness")));

  // Pupil strobe effect
  myPupil.setStrobe(static_cast<int>(channels.getChannel("PupilStrobe")));

  // Pupil rotation
  myPupil.setRotation(channels.getChannel("PupilRotation"));

  // Update the pupil
  myPupil.update();

  // Eyelids control via channels
  eyelids.setAperture(static_cast<int>(channels.getChannel("EyelidsAperture")));
  eyelids.setCenter(static_cast<int>(channels.getChannel("EyelidsCenter")));
  eyelids.update();
}

void configUpdated(){
  //called when config was updated
  eyelids.updateConfig(config.getProperty("Motors.Toplid.Max"), config.getProperty("Motors.Toplid.Min"),
  config.getProperty("Motors.Toplid.Offset"), config.getProperty("Motors.Toplid.Inverted") > 0,
  config.getProperty("Motors.Bottomlid.Max"), config.getProperty("Motors.Bottomlid.Min"),
  config.getProperty("Motors.Bottomlid.Offset"), config.getProperty("Motors.Bottomlid.Inverted") > 0);
  eyelids.setOffset(static_cast<int>(channels.getChannel("EyelidsOffset")));
  servoEyeLR.setOffset(config.getProperty("Motors.EyeLR.Offset"));
  servoEyeLR.setInverted(config.getProperty("Motors.EyeLR.Inverted"));
  servoEyeLR.setMax(config.getProperty("Motors.EyeLR.Max"));
  servoEyeLR.setMin(config.getProperty("Motors.EyeLR.Min"));
  servoEyeUD.setOffset(config.getProperty("Motors.EyeUD.Offset"));
  servoEyeUD.setInverted(config.getProperty("Motors.EyeUD.Inverted"));
  servoEyeUD.setMax(config.getProperty("Motors.EyeUD.Max"));
  servoEyeUD.setMin(config.getProperty("Motors.EyeUD.Min"));

  arpertureMove.setFreq(config.getProperty("BlinkRate"));
}

void stimulateChannels() {
  // MotorLR and MotorUD channels using sine waves for smooth movement
  channels.setChannel("MotorLR", motorMove.getValue());
  channels.setChannel("MotorUD", udMove.getValue());

  // Pupil mode, switching between different modes
  channels.setChannel("PupilMode", Pupil::GRADIENT);

  // Stimulate pupil parameters with random noise or sine waves
  uint16_t gradientParams[8] = {255, 0, 0,   // Start color: Red
                                0, 255, 0,   // End color: Blue
                                10, 20};    

  // Set pupil brightness with a sine wave for smooth changes
  channels.setChannel("PupilBrightness", 10); // Example: smooth brightness change

  // Strobe effect with a sine wave to modulate frequency
  channels.setChannel("PupilStrobe", 0); // Example: strobe frequency modulation

  // Rotation effect using a sine wave
  channels.setChannel("PupilRotation", 10); // Example: smooth rotation

  // Eyelid aperture controlled by sine waves
  channels.setChannel("EyelidsAperture", 90);

  // Eyelid center position control
  channels.setChannel("EyelidsCenter", udMove.getValue());
}
