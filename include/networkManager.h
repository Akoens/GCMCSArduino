#include <WiFiNINA.h>
#include <Arduino.h>

// WiFiSSLClient client;
WiFiClient client;
WiFiServer ap_server(80);

int status = WL_IDLE_STATUS;

void printWiFiStatus() {
  IPAddress ip = WiFi.localIP();

  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.println();
}

boolean checkWifiModuleStatus() {
  while (WiFi.status() == WL_NO_MODULE);

  if (WiFi.firmwareVersion() < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Firmware upgrade available!");
  }
}


boolean connectToWifi(char ssid[], char pass[], int retries) {
  status = WL_IDLE_STATUS;
  int retry = 0;
  while (status != WL_CONNECTED) {
    retry += 1; 
    Serial.print("Connecting to [");
    Serial.print(ssid);
    Serial.println("]...");
    status = WiFi.begin(ssid, pass);
    delay(10000);
    if (retry >= retries) {
      Serial.print("Failed to connect to [");
      Serial.print(ssid);
      Serial.println("]");
      return false;
    }
  }
  if (status == WL_CONNECTED) {
    printWiFiStatus();
    return true;
  }
}

boolean createWifiAccessPoint(char ap_ssid[], char ap_pass[]) {
  Serial.println("Creating access point...");
  status = WL_IDLE_STATUS;
  status = WiFi.beginAP(ap_ssid, ap_pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("Failed to create access point");
    return false;
  }
  delay(10000);
  ap_server.begin();
  Serial.println("Succesfully created access point");
  printWiFiStatus();
  return true;

}

void listenForClients(String sensorData){
  // compare the previous status to the current status
  if (status != WiFi.status()) {
    // it has changed update the variable
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
    }
  }
 
  WiFiClient client = ap_server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      delayMicroseconds(10);                // This is required for the Arduino Nano RP2040 Connect - otherwise it will loop so fast that SPI will never be served.
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
           client.println(sensorData);

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          }
          else {      // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /H")) {
          Serial.println("High");               // GET /H turns the LED on
        }
        if (currentLine.endsWith("GET /L")) {
          Serial.println("Low");               // GET /L turns the LED off
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}


void sendData(String requestBody, char server[], int server_port, char api_point[]) {
  // Connect to server
  Serial.println("\nStarting connection to server...");
  if(client.connect(server, server_port)) {
    Serial.println("Connected to server");
  } else {
    Serial.println("connection failed");
    client.stop();
    return;
  }

  // Send HTTP Header
  String request = String("POST ") + api_point + " HTTP/1.1 \r\n" +
                 "Content-Type: application/json\r\n" +
                 "Content-Length: " + requestBody.length() + "\r\n" +
                 "Connection: close\r\n" +
                 "\r\n";
  client.print(request);
  client.print(requestBody);
  Serial.println(requestBody);

  // End

  // Wait and read Response
  while(client.available()) {
    char c = client.read();
    Serial.print(c);
  }

  // Disconnect from server
  if(!client.connected()) {
    Serial.print("disconnected");
    client.stop();
  }
}


