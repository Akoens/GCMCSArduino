#include <Arduino.h>
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

WiFiSSLClient client;

//*** LDR SENSOR VARIABLES ***//
#define LightSensorPin A0
#define MAX_ADC_READING 1023
#define ADC_REF_VOLTAGE 5
#define REF_RESISTANCE 4700
#define LUX_CALC_SCALAR 12518931 
#define LUX_CALC_EXPONENT -1.405

int ldrValue = 0;

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
#define GREEN_MAX_VALUE 2147483647

//*** TMP SERNSOR VARIABLES ***//
#define TMPSensorPin A1
#define TMPSensorVin 5

//*** DHT SENSOR VARIABLES ***//
#define DHTPIN 9
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

//*** CONVERSION FUNCTIONS ***//
float calcKelvin(float t){
  return t + 273.15;
}

float calcFahrenheid(float t){
  return (t * 1.8) + 32;
}

//*** NETWORK FUNCTION ***//
void printWiFiStatus() {
  IPAddress ip = WiFi.localIP();

  Serial.print("IP Address: ");
  Serial.println(ip);
}

void sendData(JsonObject data){
  //Connect to server
  Serial.println("\nStarting connection to server...");
  if(client.connect(server, server_port)) {
    Serial.println("Connected to server");
  } else {
    Serial.println("connection failed");
  }

  //Send HTTP Header
  client.println("POST " + String(api_point) + " HTTP/1.1");
  client.print("Host: " + String(server));
  client.println("Content-Type: application/json" );
  client.println("Content-Length: " + String(data.size()));
  client.println("Connection: close");
  client.println();
  // End

  //Send HTTP Body
  client.println(data);
  //End

  //Wait and read Response
  while(client.available()) {
    char c = client.read();
    Serial.print(c);
  }

  //Disconnect from server
  if(!client.connected()) {
    Serial.println("disconnected");
    client.stop();
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
}

void loop() {
 //*** LDR SENSOR CODE ***//
  ldrValue = analogRead(LightSensorPin);

  float resistorVoltage = (float)ldrValue / MAX_ADC_READING * ADC_REF_VOLTAGE;
  float ldrVoltage = ADC_REF_VOLTAGE - resistorVoltage;

  float ldrResistance = ldrVoltage / resistorVoltage * REF_RESISTANCE;
  float ldrLux = LUX_CALC_SCALAR * pow(ldrResistance, LUX_CALC_EXPONENT);
  
  digitalWrite(RED, LOW);
  digitalWrite(YELLOW, LOW);
  digitalWrite(GREEN, LOW);

  if(RED_MIN_VALUE < ldrLux && ldrLux < RED_MAX_VALUE){
    digitalWrite(RED, HIGH);
  }
  if(YELLOW_MIN_VALUE < ldrLux && ldrLux < YELLOW_MAX_VALUE){
    digitalWrite(YELLOW, HIGH);
  }
  if(GREEN_MIN_VALUE < ldrLux && ldrLux > GREEN_MAX_VALUE){
    digitalWrite(GREEN, HIGH);
  }

  Serial.print("[LDR] Light: ");
  Serial.print((int)ldrLux);
  Serial.print(" Lux | raw: ");
  Serial.println(ldrValue);

  //*** TMP SERNSOR CODE ***//
  int tmpValue = analogRead(TMPSensorPin);
  float tmpVoltage = tmpValue * (TMPSensorVin / 1024.0);
  float tmpTemperature = (tmpVoltage - 0.5) * 100;

  Serial.print("[TMP] Temperature: ");
  Serial.print(tmpTemperature);
  Serial.print(F("°C / "));
  Serial.print(calcKelvin(tmpTemperature));
  Serial.print(F("°K / "));
  Serial.print(calcFahrenheid(tmpTemperature));
  Serial.print(F("°F | Raw: "));
  Serial.println(tmpValue);

  //*** DHT SENSOR CODE ***//
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  // Check if any reads failed.
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
  } 
  else {
    float k = calcKelvin(t);
    float f = calcFahrenheid(t);

    float hic = dht.computeHeatIndex(t, h, false);
    float hif = dht.computeHeatIndex(f, h);

    Serial.print(F("[DHT] Humidity: "));
    Serial.print(h);
    Serial.print(F("% | Temperature: "));
    Serial.print(t);
    Serial.print(F("°C / "));
    Serial.print(k);
    Serial.print(F("°K / "));
    Serial.print(f);
    Serial.print(F("°F | Heat index: "));
    Serial.print(hic);
    Serial.print(F("°C / "));
    Serial.print(hif);
    Serial.println(F("°F"));
  }

  Serial.println();
  delay(UpdateDelay); 
}