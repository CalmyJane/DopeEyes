#include <esp_now.h>
#include <WiFi.h>

class MasterESP32 {
public:
    MasterESP32() {
        WiFi.mode(WIFI_STA);
        WiFi.disconnect(); // Make sure WiFi is disconnected
        if (esp_now_init() != ESP_OK) {
            Serial.println("Error initializing ESP-NOW");
            return;
        }
        esp_now_register_recv_cb(onDataReceived);
    }

    void sendData(const char* data) {
        for (int i = 0; i < numSlaves; i++) {
            esp_err_t result = esp_now_send(slaveAddresses[i], (uint8_t *)data, strlen(data));
            Serial.printf("Sending to slave %d: %s\n", i, result == ESP_OK ? "Success" : "Fail");
        }
    }

    static MasterESP32* instance;

private:
    static void onDataReceived(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
        if (len == 0) return;

        // Check if the message is a registration message
        if (strcmp((const char*)incomingData, "REGISTER") == 0) {
            if (instance->isNewSlave(recv_info->src_addr)) {
                instance->addSlave(recv_info->src_addr);
                Serial.println("New slave registered.");
            }
        }
    }

    bool isNewSlave(const uint8_t *mac) {
        for (int i = 0; i < numSlaves; i++) {
            if (memcmp(slaveAddresses[i], mac, 6) == 0) {
                return false; // Already registered
            }
        }
        return true;
    }

    void addSlave(const uint8_t *mac) {
        if (numSlaves < maxSlaves) {
            memcpy(slaveAddresses[numSlaves], mac, 6);

            esp_now_peer_info_t peerInfo = {};
            memcpy(peerInfo.peer_addr, mac, 6);
            peerInfo.channel = 0;  // Use the same channel as the master
            peerInfo.encrypt = false;

            if (esp_now_add_peer(&peerInfo) != ESP_OK) {
                Serial.println("Failed to add peer");
                return;
            }

            numSlaves++;
        } else {
            Serial.println("Max slaves reached, cannot register more.");
        }
    }

    static const int maxSlaves = 10;
    int numSlaves = 0;
    uint8_t slaveAddresses[maxSlaves][6];
};

MasterESP32* MasterESP32::instance = nullptr;

void setup() {
    Serial.begin(115200);
    MasterESP32::instance = new MasterESP32();
}

void loop() {
    MasterESP32::instance->sendData("500");  // Send frequency in milliseconds
    delay(5000);  // Send every 5 seconds
}













// Use this to get Master Mac Address for the slaves
// #include <WiFi.h>

// void setup() {
//     delay(1000);  // Allow time for initialization
//     Serial.begin(115200);

//     // Initialize WiFi in STA mode
//     WiFi.mode(WIFI_STA);
//     WiFi.begin(); // Initialize WiFi without connecting to a network

//     // Retrieve and print the MAC address
//     String macAddress = WiFi.macAddress();
//     Serial.print("ESP32 MAC Address: ");
//     Serial.println(macAddress);
// }

// void loop() {
//     // Nothing to do here
// }

