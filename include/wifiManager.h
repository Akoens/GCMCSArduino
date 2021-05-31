#include <WiFiNINA.h>
#include <Arduino.h>

void printWiFiStatus() {
  IPAddress ip = WiFi.localIP();

  Serial.print("IP Address: ");
  Serial.println(ip);
}

boolean connectToWifi(char ssid[], char pass[], int retries) {
  while (WiFi.status() == WL_NO_MODULE);

  if (WiFi.firmwareVersion() < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Firmware upgrade available!");
  }
  int status = WL_IDLE_STATUS;
  int retry = 0;
  while (status != WL_CONNECTED) {
    retry += 1; 
    Serial.print("Connecting to ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(10000);
    if (retry >= retries)
    {
      return false;
    }
    
  }
  if (status == WL_CONNECTED)
  {
    printWiFiStatus();
    return true;
  }
}


