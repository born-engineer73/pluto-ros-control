#include <ESP8266WiFi.h>
#include <espnow.h>

#include "Bitcraze_PMW3901.h"

// MAC address to broadcast data to
uint8_t broadcastAddress[] = {0x80, 0x7D, 0x3A, 0x6E, 0x80, 0xB2};

// Struct to hold the deltaX and deltaY data
struct MotionData {
  int16_t deltaX;
  int16_t deltaY;
};

MotionData motionData;



// Create an instance of the PMW3901 sensor
#define CS_PIN D8
Bitcraze_PMW3901 flow(CS_PIN);

// Callback function to handle data send status
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("\r\nDelivery Status: ");
  Serial.println(sendStatus == 0 ? "Delivered Successfully" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);

  // Wait until serial port opens for native USB devices
  while (!Serial) {
    delay(1);
  }



  // Initialize the flow sensor
  if (!flow.begin()) {
    Serial.println("Initialization of the flow sensor failed");
    while(1) { }
  }



  // Set WiFi mode to Station mode
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Set the role of this device as a controller
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);

  // Register callback for data send status
  esp_now_register_send_cb(OnDataSent);

  // Add a peer (slave) with the specified broadcast address
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}

void loop() {

;

    // Get motion count since last call
    flow.readMotionCount(&motionData.deltaX, &motionData.deltaY);

    // Send the motion data to the specified broadcast address
    esp_now_send(broadcastAddress, (uint8_t *)&motionData, sizeof(motionData));

    // Print the sent data for debugging
    Serial.print("Sent deltaX: ");
    Serial.print(motionData.deltaX);
    Serial.print(", deltaY: ");
    Serial.println(motionData.deltaY);

    // Delay for 1 second before reading the next distance value
    delay(300);
  
}
