#include <esp_now.h>
#include <WiFi.h>

class SlaveESP32 {
public:
    SlaveESP32() : lastReceivedTime(0), blinkInterval(1000), ledPin(2) {
        pinMode(ledPin, OUTPUT);
        WiFi.mode(WIFI_STA);
        WiFi.disconnect(); // Make sure WiFi is disconnected
        if (esp_now_init() != ESP_OK) {
            Serial.println("Error initializing ESP-NOW");
            return;
        }
        esp_now_register_recv_cb(onDataReceived);
        sendRegistration();  // Register with the master on startup
    }

    void loop() {
        if (millis() - lastReceivedTime > timeout) {
            Serial.println("Master not found, continuing with last known frequency");
        }

        if (millis() - previousMillis >= blinkInterval) {
            previousMillis = millis();
            ledState = !ledState;
            digitalWrite(ledPin, ledState);
        }
    }

    static SlaveESP32* instance;

private:
    void sendRegistration() {
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, broadcastAddress, 6);
        peerInfo.channel = 0;  // Use the same channel as the master
        peerInfo.encrypt = false;

        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            Serial.println("Failed to add peer");
            return;
        }

        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)"REGISTER", strlen("REGISTER"));
        Serial.println(result == ESP_OK ? "Registration sent" : "Failed to send registration");
    }

    static void onDataReceived(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
        char data[len + 1];
        memcpy(data, incomingData, len);
        data[len] = '\0';

        uint32_t frequency = atoi(data);
        instance->setBlinkInterval(frequency);
        instance->lastReceivedTime = millis();
        Serial.printf("Received: %s\n", data);
    }

    void setBlinkInterval(uint32_t interval) {
        blinkInterval = interval;
    }

    uint32_t lastReceivedTime;
    uint32_t blinkInterval;
    unsigned long previousMillis = 0;
    const uint32_t timeout = 10000;  // 10 seconds
    const int ledPin;
    bool ledState = false;

    // Change this to the master ESP32's MAC address
    uint8_t broadcastAddress[6] = {0x08, 0xA6, 0xF7, 0x12, 0x8D, 0x00};  
};

SlaveESP32* SlaveESP32::instance = nullptr;

void setup() {
    Serial.begin(115200);
    SlaveESP32::instance = new SlaveESP32();
}

void loop() {
    SlaveESP32::instance->loop();
}
