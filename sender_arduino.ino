#include <ESP8266WiFi.h>
#include <espnow.h>
#include "Adafruit_VL53L0X.h"

// MAC address to broadcast data to
uint8_t broadcastAddress[] = {0x80, 0x7D, 0x3A, 0x6E, 0x80, 0xB2};

// Variable to hold the integer data
int data = 0;

// Create an instance of the VL53L0X sensor
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

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

  Serial.println("Adafruit VL53L0X test.");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
  Serial.println(F("VL53L0X API Continuous Ranging example\n\n"));

  // Start continuous ranging
  lox.startRangeContinuous();

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
  if (lox.isRangeComplete()) {
    // Read the distance data from the sensor
    data = lox.readRange();

    // Send the integer data to the specified broadcast address
    esp_now_send(broadcastAddress, (uint8_t *)&data, sizeof(data));

    // Print the sent data for debugging
    Serial.print("Sent distance in mm: ");
    Serial.println(data);

    // Delay for 1 second before reading the next distance value
    delay(500);
  }
}
