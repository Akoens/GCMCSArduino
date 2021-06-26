#include <DHT.h>
#include <DHT_U.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include <utils.h>
#include <ArduinoJson.h>

//*** SENSOR PINS ***//
#define LDRPIN A0
#define DHTPIN 2
#define ONE_WIRE_BUS 3
#define SoilMoisturePin A1

//*** TMP SERNSOR VARIABLES ***//
// #define TMPSensorPin A1
// #define TMPSensorVin 5

//*** LDR SENSOR VARIABLES ***//
#define MAX_ADC_READING 1023
#define ADC_REF_VOLTAGE 5
#define REF_RESISTANCE 4700
#define LUX_CALC_SCALAR 12518931 
#define LUX_CALC_EXPONENT -1.405

//*** DHT SENSOR VARIABLES ***//
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

//*** DS18B20 SENSOR VARIABLES ***//
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;

//*** MOISTURE SENSOR VARIABLES ***//
const int AirValue = 624;   //At 52.7 Humidiy
const int WaterValue = 261;  

//*** SENSOR STRUCT ***//
struct dhtDataStruct {
  float h;
  float t;
};

//*** Getters ***//
float getLDR() {
  int ldrValue = analogRead(LDRPIN);
  float resistorVoltage = (float)ldrValue / MAX_ADC_READING * ADC_REF_VOLTAGE;
  float ldrVoltage = ADC_REF_VOLTAGE - resistorVoltage;
  float ldrResistance = ldrVoltage / resistorVoltage * REF_RESISTANCE;
  return LUX_CALC_SCALAR * pow(ldrResistance, LUX_CALC_EXPONENT);
}

dhtDataStruct getDHT() {
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

float getD18B20() {
  sensors.requestTemperatures();
  return sensors.getTempC(insideThermometer);
}

float getCSM() {
  int soilMoistureValue = analogRead(SoilMoisturePin);
  return map(soilMoistureValue, AirValue, WaterValue, 0, 100);
}

// float getTMP() {
//   int tmpValue = analogRead(TMPSensorPin);
//   return((tmpValue * TMPSensorVin / 1024.0) - 0.5) * 100;
// }

//*** Prints ***//
void printLDR(float lux) {
  Serial.println(
    "[LDR] Light: " + String((int)lux) + " Lux"
  );
}

void printDHT(dhtDataStruct dhtData) {
  Serial.println(
    "[DHT] Humidity: " + String(dhtData.h) + 
    "% | Temperature: " + String(dhtData.t) +
    "*C / " + String(calcFahrenheid(dhtData.t)) +
    "*K / " + String(calcKelvin(dhtData.t)) + 
    "*F | Heat index: " + String(dht.computeHeatIndex(dhtData.t, dhtData.h, false)) +
    "*C / " + String(dht.computeHeatIndex(calcFahrenheid(dhtData.t), dhtData.h)) + "*F"
  );
}

void printD18B20(float groundTemperature) {
  Serial.println(
    "[DS20B18] Temperature: " + String(groundTemperature) + "*C"
  );
}

void printCSM(int soilMoisture) {
  Serial.println(
    "[CSM] Soil Moisture: " + String(soilMoisture) + "%"
  );
}

float printTMP(float temperature) {
  Serial.println(
    "[TMP] Temperature: " + String(temperature) + 
    "*C / " + String(calcKelvin(temperature)) +
    "*K / " + String(calcFahrenheid(temperature)) +
    "*F"
  );
}

void startSensors(){
  dht.begin();
  sensors.begin();
  sensors.setResolution(9);

  Serial.print("[DS18B20] Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");
  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0");
}

String readSensors(){
  // LDR
  float lux = getLDR();
  printLDR(lux);

  // DHT
  dhtDataStruct dhtData = getDHT();
  printDHT(dhtData);

  // DS20B18
  float groundTemperature = getD18B20();
  printD18B20(groundTemperature);

  //Moisture Sensor
  float soilMoisture = getCSM();
  printCSM(soilMoisture);

  //TMP
  // float temperature = getTMP();
  // printTMP(temperature);

  // Data transmition
  StaticJsonDocument<400> doc;
  JsonObject obj = doc.to<JsonObject>();

  obj["temperature"] = dhtData.t;
  obj["light"] = lux;
  obj["humidity"] = dhtData.h;
  obj["heat_index"] = dht.computeHeatIndex(dhtData.t, dhtData.h, false);
  obj["ground_temperature"] = groundTemperature;
  obj["ground_moisture"] = soilMoisture;

  String temp;
  serializeJson(obj, temp);

  return temp;
}






