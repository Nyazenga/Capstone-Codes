#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <DHT.h>
#include <WiFiUdp.h>
#include <HX711_ADC.h>
#include "ESPAsyncWebServer.h"
#include <ESP32Servo.h>
#include <Arduino_JSON.h>

Servo aerator;
Servo eggdoor;

AsyncWebServer server(80);

//Components
#define LDR 36
#define DHT_SENSOR_PIN 16
#define DHT_SENSOR_TYPE DHT22
#define BUZZER 32
#define INFRARED_LAMP 26
#define FAN 25
#define LIGHT 12
#define MOTOR_1 18
#define MOTOR_2 33
#define PUMP 27

int weight = 0;
const char* ssid = "FLYSOL";
const char* password = "11111111";

const char* serverName = "http://172.16.8.27/capstone/flysol/esp-outputs-action.php?action=outputs_state&board=1";


const long period = 100;
unsigned long prevMillis = 0;

String outputsState;

//pins:
const int HX711_dout = 5;  //mcu > HX711 dout pin
const int HX711_sck = 19;  //mcu > HX711 sck pin

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
unsigned long t = 0;

// Time variables for aeration motor
unsigned long previousMillis = 0;
const long interval = 86400000;      // 1 minutes in milliseconds
const int motorDuration = 10000;  // 20 seconds
unsigned long motorStartTime = 0;

// Time variables for egg door
unsigned long lastRun = 0;
const long timePeriod = 3600000;      // 2 minutes in milliseconds
const int openDuration = 2000;  // 20 seconds
unsigned long openStartTime = 0;

LiquidCrystal_I2C lcd(0x27, 20, 4);
DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());

  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  aerator.setPeriodHertz(50);      // standard 50 hz servo
  aerator.attach(33, 1000, 2000);  // attaches the servo on pin 18 to the servo object

  eggdoor.setPeriodHertz(50);      // standard 50 hz servo
  eggdoor.attach(18, 1000, 2000);  // attaches the servo on pin 18 to the servo object

  dht_sensor.begin();

  pinMode(BUZZER, OUTPUT);
  pinMode(INFRARED_LAMP, OUTPUT);
  pinMode(FAN, OUTPUT);
  pinMode(LIGHT, OUTPUT);
  pinMode(MOTOR_1, OUTPUT);
  pinMode(MOTOR_2, OUTPUT);
  pinMode(PUMP, OUTPUT);

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(23, OUTPUT);


  digitalWrite(FAN, HIGH);
  digitalWrite(PUMP, LOW);
  digitalWrite(LIGHT, LOW);
  digitalWrite(INFRARED_LAMP, HIGH);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.println("  Welcome !!        ");
  lcd.setCursor(0, 1);
  lcd.println("  Automated BSF Farm");
  lcd.setCursor(0, 2);
  lcd.println("Initializing ...    ");
  lcd.setCursor(0, 3);
  lcd.println("                    ");

  LoadCell.begin();
  LoadCell.setReverseOutput();  //uncomment to turn a negative output value to positive
  float calibrationValue;       // calibration value (see example file "Calibration.ino")
  calibrationValue = 257.68;    // uncomment this if you want to set the calibration value in the sketch

  unsigned long stabilizingtime = 2000;  // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                  //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    while (1)
      ;
  } else {
    LoadCell.setCalFactor(calibrationValue);  // set calibration value (float)
  }
}

void loop() {
  unsigned long currentMillis = millis();
  unsigned long curMillis = millis();
  unsigned long current_time = millis();
  
  float h = dht_sensor.readHumidity();
  float t = dht_sensor.readTemperature();
  String h_str = String(int(h));
  String t_str = String(int(t));
  int ldrValue = analogRead(36);
  uint16_t value = analogRead(39);
  uint16_t scaledValue = map(value, 3200, 2220, 0, 100);
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0;
  

  int egg_door_motor = digitalRead(2);
  int aeration_motor = digitalRead(3);
  int camera = digitalRead(23);
  int infrared_lamp = digitalRead(15);
  int light_bulb = digitalRead(17);
  int fan = digitalRead(4);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Light: ");
  lcd.print(ldrValue);
  lcd.print(" LUX ");
  lcd.setCursor(0, 1);
  lcd.print("Temp: ");
  lcd.print(t_str.substring(0, 2));
  lcd.print("\xDF");
  lcd.print("C ");
  lcd.print("Hum: ");
  lcd.print(h_str.substring(0, 2));
  lcd.print("%");
  if (scaledValue < 0 or scaledValue > 100) {
    lcd.setCursor(0, 2);
    lcd.print("Moisture: ...");
    lcd.print(" ");
    lcd.print("%");
  } else {
    lcd.setCursor(0, 2);
    lcd.print("Moisture: ");
    lcd.print(scaledValue);
    lcd.print(" ");
    lcd.print("%");
  }
  lcd.setCursor(0, 3);  // set the cursor to column 0, line 1
  lcd.print("Yield: ");

  HTTPClient http;
  String HOST_NAME = "http://172.16.8.27";
  String PATH_NAME = "/capstone/flysol/insert_readings.php";
  String queryString = "?light_intensity=" + String(ldrValue) + "&temperature=" + String(t_str.substring(0, 2))+ "&humidity=" + String(h_str.substring(0, 2))+ "&moisture=" + String(scaledValue)+ "&yield=" + String(weight);
  http.begin(HOST_NAME + PATH_NAME + queryString);
  int httpCode = http.GET();
  if (httpCode > 0) {
      if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        Serial.println(payload);
      } else {
        Serial.printf("[HTTP] GET... code: %d\n", httpCode);
      }
    } else {
      Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
  }
  
  if(curMillis - prevMillis >= period) {
    if(WiFi.status()== WL_CONNECTED ){ 
      outputsState = httpGETRequest(serverName);
      Serial.println(outputsState);
      JSONVar myObject = JSON.parse(outputsState);
  
      if (JSON.typeof(myObject) == "undefined") {
        Serial.println("Parsing input failed!");
        return;
      }
    
      Serial.print("JSON object = ");
      Serial.println(myObject);
      JSONVar keys = myObject.keys();
    
      for (int i = 0; i < keys.length(); i++) {
        JSONVar value = myObject[keys[i]];
        Serial.print("GPIO: ");
        Serial.print(atoi(keys[i]));
        Serial.print(" - SET to: ");
        Serial.println(value);
        digitalWrite(atoi(keys[i]), atoi(value));
      }
      prevMillis = curMillis;
    }
    else {
      Serial.println("WiFi Disconnected");
    }
  }

  //Turn on fan if humidity is too high
  if ((dht_sensor.readTemperature() > 32 || fan == HIGH) {
    digitalWrite(FAN, LOW);
  } else {
    digitalWrite(FAN, HIGH);
  }

      // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    float i = LoadCell.getData();
    weight = int(abs(i));
    lcd.print(String(int(abs(i))));
    lcd.print(" ");
    lcd.print("g");
    newDataReady = 0;
    delay(500);
  }

  //Turn on pump if moisture value is low
  if (scaledValue < 10 ) {
    digitalWrite(PUMP, HIGH);
  } else {
    digitalWrite(PUMP, LOW);
  }

  // Turn on light if LDR value is low
  if (analogRead(36) < 400 || light_bulb == HIGH) {
    digitalWrite(LIGHT, HIGH);
  } else {
    digitalWrite(LIGHT, LOW);
  }

  

  // Check if it's time to turn on the motor
  if ((currentMillis - previousMillis >= interval) || (aeration_motor == HIGH)) {
    // Save the last time the motor was turned on
    previousMillis = currentMillis;
    Serial.println("Stirring");
    aerator.attach(33, 1000, 2000);  // attaches the servo on pin 18 to the servo object
    aerator.write(0);
    // Save the time the motor was turned on
    motorStartTime = millis();
  }
  // Check if it's time to turn off the motor
  if (((motorStartTime != 0) && (millis() - motorStartTime >= motorDuration))) {
    // Turn off motor
    Serial.println("Done");
    aerator.detach();
    motorStartTime = 0;
  }

  // Check if it's time to turn on the motor
  if ((current_time - lastRun >= timePeriod) || (egg_door_motor == HIGH)) {
    // Save the last time the motor was turned on
    lastRun = current_time;
    eggdoor.write(90);
    // Turn on motor
    // Save the time the motor was turned on
    openStartTime = millis();
  }
  // Check if it's time to turn off the motor
  if ((openStartTime != 0) && (millis() - openStartTime >= openDuration)) {
    eggdoor.write(0);
    openStartTime = 0;
  }

  if (dht_sensor.readTemperature() < 27 || infrared_lamp == HIGH) {
    digitalWrite(INFRARED_LAMP, LOW);
  } else {
    digitalWrite(INFRARED_LAMP, HIGH);
  }
}

String httpGETRequest(const char* serverName) {
  WiFiClient client;
  HTTPClient http;
    
  // Your IP address with path or Domain name with URL path 
  http.begin(client, serverName);
  
  // Send HTTP POST request
  int httpResponseCode = http.GET();
  
  String payload = "{}"; 
  
  if (httpResponseCode>0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}