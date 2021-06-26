#include <Arduino.h>
#include <secrets.h>

#include <sensors.h>
#include <networkManager.h>

#include <ArduinoJson.h>

#include <WifiNINA.h>

//** Global variables **//
#define updateDelay 5
#define sensorUpdateTime 30 * 1000 / updateDelay
#define pumpFreqency 24 * 60 * 60 * 1000 / updateDelay
#define pumpTime 40 * 1000
#define ButtonPin 5
int count = 0;
int pump_count = 0;
String sensorData;

//*** WiFi / Network Variables ***//
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

char ap_ssid[] = "GCMCS-AP"; // Minimum of 8 charactes
char ap_pass[] = "88888888"; // Minimum of 8 charactes

char server[] = SECRET_SERVER;
int server_port = SECRET_PORT;
char api_point[] = SECRET_API;
int connectionRetries = 3;
boolean hasWifi = false;

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
#define PumpPin 4

//*** MAIN FUNCTIONS ***//
void setup() {
  // pinMode(RED, OUTPUT);
  // pinMode(YELLOW, OUTPUT);
  // pinMode(GREEN, OUTPUT);
  pinMode(PumpPin, OUTPUT);
  pinMode(ButtonPin, INPUT);

  Serial.begin(9600);
  while (!Serial);

  // Try to connect to wifi
  hasWifi = connectToWifi(ssid, pass, connectionRetries);
  // If has no wifi make AP.
  if(!hasWifi) createWifiAccessPoint(ap_ssid, ap_pass);

  startSensors();
}

void pump(int pump_delay) {
  digitalWrite(PumpPin, HIGH);
  delay(pump_delay); 
  digitalWrite(PumpPin, LOW);
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

  if(!hasWifi) listenForClients(sensorData);

  if (count > sensorUpdateTime) {
    count = 0;
    // pump(5000);
    sensorData = readSensors();
    if(hasWifi) sendData(sensorData, server, server_port, api_point);
  } else 
  count++;

  if (pump_count >= pumpFreqency) {
    pump(pumpTime);
    pump_count = 0;
  }

  static uint8_t lastButtonState = LOW;
  uint8_t state = digitalRead(ButtonPin);
  if (state != lastButtonState) {
    lastButtonState = state;
    if (state == HIGH) {
      pump(5 * 1000);
      Serial.println("Button pressed!");
    }
  }
  
  delay(updateDelay);
}
