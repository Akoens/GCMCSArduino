#include <Arduino.h>
#include <secrets.h>

#include <Wire.h>
#include <SPI.h>
#include <math.h>

#include <WifiNINA.h>
#include <Adafruit_Sensor.h>

#include <DHT.h>
#include <DHT_U.h>

char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

char server[] = "www.nu.nl";  

WiFiSSLClient client;

void printWiFiStatus() {
  IPAddress ip = WiFi.localIP();

  Serial.print("IP Address: ");
  Serial.println(ip);

  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

//*** LDR SENSOR VARIABLES ***//
#define LightSensorPin A0
#define MAX_ADC_READING 1023
#define ADC_REF_VOLTAGE 5
#define REF_RESISTANCE 16000
#define LUX_CALC_SCALAR 12518931 
#define LUX_CALC_EXPONENT -1.405

int ldrValue = 0;


//*** TMP SERNSOR VARIABLES ***//
#define TMPSensorPin A1
#define TMPSensorVin 5


//*** DHT SENSOR VARIABLES ***//
#define DHTPIN 2
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

//*** CONVERSION FUNCTIONS ***//
float calcKelvin(float t){
  return t + 273.15;
}

float calcFahrenheid(float t){
  return (t * 1.8) + 32;
}

//*** MAIN FUNCTIONS ***//
void setup() {
  Serial.begin(9600);
  while (!Serial);
  while (WiFi.status() == WL_NO_MODULE);

  if (WiFi.firmwareVersion() < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Firmware upgrade available");
  }

  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }

  printWiFiStatus();
  
  Serial.println("\nStarting connection to server...");
  /*
  if (client.connect(server, 443)) {
    Serial.println("connected to server");
    client.println("GET /search?q=arduino HTTP/1.1");
    client.println("Host: www.google.com");
    client.println("Connection: close");
    client.println();
  }
  */
}

void loop() {
  /*
  while (client.available()) {
    char c = client.read();
    Serial.write(c);
  }

  if (!client.connected()) {
    Serial.println();
    Serial.println("disconnecting from server.");
    client.stop();

    while (true);
  }
  */

 //*** LDR SENSOR CODE ***//
  ldrValue = analogRead(LightSensorPin);

  float resistorVoltage = (float)ldrValue / MAX_ADC_READING * ADC_REF_VOLTAGE;
  float ldrVoltage = ADC_REF_VOLTAGE - resistorVoltage;

  float ldrResistance = ldrVoltage / resistorVoltage * REF_RESISTANCE;
  float ldrLux = LUX_CALC_SCALAR * pow(ldrResistance, LUX_CALC_EXPONENT);

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
  delay(2000); 
}