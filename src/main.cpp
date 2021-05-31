#include <Arduino.h>
#include <secrets.h>

#include <sensors.h>
#include <wifiManager.h>

#include <ArduinoJson.h>

#include <WifiNINA.h>

//** Global variables **//
#define UpdateDelay 5000


//*** WiFi / Network Variables ***//
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
char server[] = SECRET_SERVER;
int server_port = SECRET_PORT;
char api_point[] = SECRET_API;
int connectionRetries = 3;
boolean hasWifi = false;

// WiFiSSLClient client;
WiFiClient client;


//*** LED Lights ***//
// #define RED 2
// #define YELLOW 3
// #define GREEN 4


// Light sensor LEDs
// Lowest value || -2147483647-1 || Highest value || 2147483647
// #define RED_MIN_VALUE -2147483647-1
// #define RED_MAX_VALUE 100

// #define YELLOW_MIN_VALUE 100
// #define YELLOW_MAX_VALUE 500

// #define GREEN_MIN_VALUE 500
// #define GREEN_MAX_VALUE 2147483646

//*** PUMP VARIABLES ***//
#define PumpPin 12


//*** NETWORK DATA FUNCTION ***//
void sendData(JsonObject jsonData) {
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
  String requestBody;
  serializeJson(jsonData, requestBody);

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

//*** MAIN FUNCTIONS ***//
void setup() {
  // pinMode(RED, OUTPUT);
  // pinMode(YELLOW, OUTPUT);
  // pinMode(GREEN, OUTPUT);
  // pinMode(PumpPin, OUTPUT);

  Serial.begin(9600);
  while (!Serial);

  hasWifi = connectToWifi(ssid, pass, connectionRetries);
  // If has no wifi make AP.

  startSensors();
}

void loop() {
  // digitalWrite(RED, LOW);
  // digitalWrite(YELLOW, LOW);
  // digitalWrite(GREEN, LOW);

  // if(RED_MIN_VALUE < ldrData.ldrLux && ldrData.ldrLux < RED_MAX_VALUE){
  //   digitalWrite(RED, HIGH);
  // }
  // if(YELLOW_MIN_VALUE < ldrData.ldrLux && ldrData.ldrLux < YELLOW_MAX_VALUE){
  //   digitalWrite(YELLOW, HIGH);
  // }
  // if(GREEN_MIN_VALUE < ldrData.ldrLux && ldrData.ldrLux < GREEN_MAX_VALUE){
  //   digitalWrite(GREEN, HIGH);
  // }

  JsonObject sensorData = readSensors();
  if(hasWifi) sendData(sensorData);

  digitalWrite(PumpPin, HIGH);
  delay(UpdateDelay); 
  digitalWrite(PumpPin, LOW);
}