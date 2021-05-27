#include <WiFiNINA.h>
#include <Arduino.h>

boolean connectToWifi(String ssid, String pass, int retries) {
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

void printWiFiStatus() {
  IPAddress ip = WiFi.localIP();

  Serial.print("IP Address: ");
  Serial.println(ip);
}
