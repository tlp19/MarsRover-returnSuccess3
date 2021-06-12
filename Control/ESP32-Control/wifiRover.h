#ifndef wifirover_h
#define wifirover_h

#include <WiFi.h>


//WiFi credentials
const char *SSID = "your_wifi_name";
const char *password = "your_wifi_password";


//Connects to WiFi using provided credentials
void connectToWiFi() {
  Serial.print("Connecting to WiFi: ");
  WiFi.begin(SSID, password);
  Serial.print(SSID);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("Connected.");
}

#endif
