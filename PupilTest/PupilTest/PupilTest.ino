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

// Create an instance of the Pupil class
Pupil myPupil;

void setup() {
  Serial.begin(115200);
  myPupil.begin();  // Initialize the pupil
}

void loop() {
  // Example: Gradient mode with rotation
  uint16_t gradientParams[8] = {255, 100, 0,   // Start color: Red
                                100, 100, 255,   // End color: Blue
                                };          

  float rotationSpeed = 8;  // Rotate smoothly at 1 LED per second

  myPupil.update(Pupil::GRADIENT, gradientParams, 255, 0, rotationSpeed);

  delay(5);  // Small delay to control update frequency
}
