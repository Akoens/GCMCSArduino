#include <Arduino.h>
#include <sensorDataStructs.h>
#include <utils.h>
#include <secrets.h>

#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <ArduinoJson.h>

#include <WifiNINA.h>
#include <Adafruit_Sensor.h>

#include <DHT.h>
#include <DHT_U.h>

//** Global variables **//
#define UpdateDelay 5000


//*** WiFi / Network Variables ***//
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
char server[] = SECRET_SERVER;
int server_port = SECRET_PORT;
char api_point[] = SECRET_API;

// WiFiSSLClient client;
WiFiClient client;


//*** LDR SENSOR VARIABLES ***//
#define LightSensorPin A0
#define MAX_ADC_READING 1023
#define ADC_REF_VOLTAGE 5
#define REF_RESISTANCE 4700
#define LUX_CALC_SCALAR 12518931 
#define LUX_CALC_EXPONENT -1.405

#define RED 2
#define YELLOW 3
#define GREEN 4


// Light sensor LEDs
// Lowest value || -2147483647-1 || Highest value || 2147483647
#define RED_MIN_VALUE -2147483647-1
#define RED_MAX_VALUE 100

#define YELLOW_MIN_VALUE 100
#define YELLOW_MAX_VALUE 500

#define GREEN_MIN_VALUE 500
#define GREEN_MAX_VALUE 2147483646


//*** TMP SERNSOR VARIABLES ***//
#define TMPSensorPin A1
#define TMPSensorVin 5


//*** DHT SENSOR VARIABLES ***//
#define DHTPIN 9
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);


//*** NETWORK FUNCTION ***//
void printWiFiStatus() {
  IPAddress ip = WiFi.localIP();

  Serial.print("IP Address: ");
  Serial.println(ip);
}

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

//*** LDR SENSOR CODE ***//
ldrDataStruct readLDR() {
  int ldrValue = analogRead(LightSensorPin);

  float resistorVoltage = (float)ldrValue / MAX_ADC_READING * ADC_REF_VOLTAGE;
  float ldrVoltage = ADC_REF_VOLTAGE - resistorVoltage;

  float ldrResistance = ldrVoltage / resistorVoltage * REF_RESISTANCE;
  float ldrLux = LUX_CALC_SCALAR * pow(ldrResistance, LUX_CALC_EXPONENT);

  return ldrDataStruct{ldrLux, ldrValue};
}

//*** TMP SERNSOR CODE ***//
tmpDataStruct readTMP() {
  int tmpValue = analogRead(TMPSensorPin);
  float tmpVoltage = tmpValue * TMPSensorVin / 1024.0;
  float tmpTemperature = (tmpVoltage - 0.5) * 100;
  
  return tmpDataStruct{tmpTemperature, tmpValue};
}

//*** DHT SENSOR CODE ***//
dhtDataStruct readDHT() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  // Check if any reads failed.
  if (isnan(h) || isnan(t)) {
    return dhtDataStruct{0,0};
  } 
  else {
    return dhtDataStruct{h, t};
  }
}

//*** MAIN FUNCTIONS ***//
void setup() {
  pinMode(RED, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  pinMode(GREEN, OUTPUT);

  Serial.begin(9600);
  while (!Serial);
  while (WiFi.status() == WL_NO_MODULE);

  if (WiFi.firmwareVersion() < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Firmware upgrade available!");
  }

  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }

  printWiFiStatus();
  Serial.println();

  dht.begin();
}

void loop() {
  // LDR
  ldrDataStruct ldrData = readLDR();
  String ldrString = "[LDR] Light: " + String((int)ldrData.ldrLux) + 
  " Lux | raw: " + String(ldrData.ldrValue);
  Serial.println(ldrString);

  digitalWrite(RED, LOW);
  digitalWrite(YELLOW, LOW);
  digitalWrite(GREEN, LOW);

  if(RED_MIN_VALUE < ldrData.ldrLux && ldrData.ldrLux < RED_MAX_VALUE){
    digitalWrite(RED, HIGH);
  }
  if(YELLOW_MIN_VALUE < ldrData.ldrLux && ldrData.ldrLux < YELLOW_MAX_VALUE){
    digitalWrite(YELLOW, HIGH);
  }
  if(GREEN_MIN_VALUE < ldrData.ldrLux && ldrData.ldrLux < GREEN_MAX_VALUE){
    digitalWrite(GREEN, HIGH);
  }

  // TMP
  tmpDataStruct tmpData = readTMP();
  String tmpString = "[TMP] Temperature: " + String(tmpData.tmpTemperature) + 
    "*C / " + String(calcKelvin(tmpData.tmpTemperature)) +
    "*K / " + String(calcFahrenheid(tmpData.tmpTemperature)) +
    "*F | Raw: " + String(tmpData.tmpValue);
  Serial.println(tmpString);
  
  // DHT
  dhtDataStruct dhtData = readDHT();
  String dhtString = "[DHT] Humidity: " + String(dhtData.h) + 
  "% | Temperature: " + String(dhtData.t) +
  "*C / " + String(calcFahrenheid(dhtData.t)) +
  "*K / " + String(calcKelvin(dhtData.t)) + 
  "*F | Heat index: " + String(dht.computeHeatIndex(dhtData.t, dhtData.h, false)) +
  "*C / " + String(dht.computeHeatIndex(calcFahrenheid(dhtData.t), dhtData.h)) + "*F";

  Serial.println(dhtString);

  // Data transmition
  StaticJsonDocument<250> doc;
  JsonObject obj = doc.to<JsonObject>();

  obj["tmp"] = tmpString;
  obj["ldr"] = ldrString;
  obj["dht"] = dhtString;

  sendData(obj);

  Serial.println();
  delay(UpdateDelay); 
}