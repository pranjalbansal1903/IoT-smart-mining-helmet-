#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
// #include <ESP8266WiFi.h>
#include <ThingSpeak.h>
// WiFiClient client;
#include <ArduinoJson.h>
const int trigPin = 3;
const int echoPin = 4;
long durationUltra;
int distance;
SoftwareSerial nodemcu(5, 6);
// static const int RXPin = 10, TXPin = 11;
static const uint32_t GPSBaud = 9600;
// SoftwareSerial ss(10, 11);
TinyGPSPlus gps;
Adafruit_BMP085 bmp;
int HEART_CHECK = 500;
int MINING_LIMIT = -100;
int mq135_analog_pin = A0;
int piezoPin = A1; // analog input pin connected to the piezo sensor
int digitalPin = 9; // digital output pin connected to the piezo sensor
int buzzerPin = 8; 
int frequency = 1000; // The frequency of the tone in Hz
int duration = 1000; // The duration of the tone in ms



void setup() {
  nodemcu.begin(9600);
  if (!bmp.begin()) {
    // Serial.println("Could not find BMP180 or BMP085 sensor at default address.");
    while (1);
  }
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  // ss.begin(GPSBaud);
  pinMode(digitalPin, OUTPUT);
  // Serial.println("GPS Start");
  pinMode(buzzerPin, OUTPUT);
  Serial.begin(9600); // initialize serial communication at 9600 bps
}




void loop() {
  StaticJsonBuffer<1000> jsonBuffer;
  JsonObject& data = jsonBuffer.createObject();

  // Part for MQ135
  int sensor_value = analogRead(mq135_analog_pin);
  float voltage = sensor_value * (5.0 / 1023.0);
  float resistance = (5.0 * 10.0) / voltage - 10.0;
  float ppm = (1.0 / 3.6) * pow((resistance / 10000.0), -1.107);
  if(ppm > 400)
  {
    tone(buzzerPin, frequency, duration);
    delay(100);
    noTone(buzzerPin);
    delay(1000);

    tone(buzzerPin, frequency, duration);
    delay(100);
    noTone(buzzerPin);
    delay(1000);

    tone(buzzerPin, frequency, duration);
    delay(100);
    noTone(buzzerPin);
    delay(1000);
  }
  Serial.println(ppm);


  //Portion for bmp180 sensor
  // Serial.print("Temperature = ");
  Serial.println(bmp.readTemperature());
  // Serial.println(" *C");

  // Serial.print("Pressure = ");
  Serial.println(bmp.readPressure());
  // Serial.println(" Pa");

  // Serial.print("Altitude = ");
  Serial.println(bmp.readAltitude());
  // Serial.println(" m");
  if(bmp.readPressure()>110000 || bmp.readPressure()<3000 || bmp.readAltitude()< MINING_LIMIT)
  {
    tone(buzzerPin, frequency, duration);
    delay(100);
    noTone(buzzerPin);
    delay(100);
    tone(buzzerPin, frequency, duration);
    delay(100);
    noTone(buzzerPin);
    delay(1000);

    tone(buzzerPin, frequency, duration);
    delay(100);
    noTone(buzzerPin);
    delay(100);
    tone(buzzerPin, frequency, duration);
    delay(100);
    noTone(buzzerPin);
    delay(1000);
  
  }


  //GPS Module
  /*while (ss.available() > 0)
  {
    gps.encode(ss.read());
  }
  if (gps.location.isValid())
  {
    // Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    // Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
  }*/


  //Piezo
  int sensorValueP = analogRead(piezoPin);
  // Serial.print("Heart Rate through Piezo= ");
  Serial.println(sensorValueP); // print the sensor value to the serial monitor
  if(sensorValueP>HEART_CHECK)
  {
    tone(buzzerPin, frequency, duration);
    delay(100);
    noTone(buzzerPin);
    delay(100);
    tone(buzzerPin, frequency, duration);
    delay(100);
    noTone(buzzerPin);
    delay(100);
    tone(buzzerPin, frequency, duration);
    delay(100);
    noTone(buzzerPin);
    delay(1000);


    tone(buzzerPin, frequency, duration);
    delay(100);
    noTone(buzzerPin);
    delay(100);
    tone(buzzerPin, frequency, duration);
    delay(100);
    noTone(buzzerPin);
    delay(100);
    tone(buzzerPin, frequency, duration);
    delay(100);
    noTone(buzzerPin);
    delay(1000);
  }

  // UltraSonic Sensor 
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  durationUltra = pulseIn(echoPin, HIGH);
  distance = durationUltra * 0.034 / 2;
  // Serial.print("Distance: ");
  Serial.println(distance);
  // Serial.println(" cm");
  if(distance<30)
  {
    tone(buzzerPin,frequency,duration);
    delay(1000);
    noTone(buzzerPin);
    delay(1000);
    tone(buzzerPin,frequency,duration);
    delay(1000);
    noTone(buzzerPin);
    delay(1000);
  }
  else
  {
    noTone(buzzerPin);
  }
  data["pressure"] = bmp.readPressure();
  data["temperature"] = bmp.readTemperature();
  data["altitude"] = bmp.readAltitude();
  data["beat"] = sensorValueP;
  data["distance"] = distance;
  data["ppm"] = ppm;
  data.printTo(nodemcu);
  jsonBuffer.clear();

  delay(5000); 
}
