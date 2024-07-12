#include <ESP8266WiFi.h>
#include <espnow.h>

// Struct to hold the received deltaX and deltaY data
struct MotionData {
  int16_t deltaX;
  int16_t deltaY;
};

MotionData receivedData;

// Callback function to handle received data
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  // Copy the incoming data to the receivedData variable
  memcpy(&receivedData, incomingData, sizeof(receivedData));
 
  // Print the received data
  //Serial.print("Received deltaX: ");
  Serial.print(receivedData.deltaX);
  Serial.print(",");
  Serial.println(receivedData.deltaY);
}

void setup() {
  // Initialize the serial monitor
  Serial.begin(115200);

  // Set WiFi mode to Station mode
  WiFi.mode(WIFI_STA);
 
  // Initialize ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Set the role of this device as a slave
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
 
  // Register callback for received data
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  // The loop function is intentionally left empty
  // All work is done in the callback function
}
