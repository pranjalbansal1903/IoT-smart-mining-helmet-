#define BUZZER_PIN D2
#include<ESP8266WiFi.h>
#include <espnow.h>
#include<WiFiClient.h>
#include<ESP8266HTTPClient.h>
#include <ThingSpeak.h>
#include <ArduinoJson.h>
#include<SoftwareSerial.h>
#include <TinyGPS++.h>
#include <ESP8266WebServer.h>
const char* ssid = "Pranjal";
const char* password = "voew9525";
uint8_t broadcastAddress[] = {0xE8,0x9F,0x6D,0x92,0x07,0x58};
long myChannelNumber = 2128605;
const char* apiKey = "72VSH0KDZ0HB7H5W";
typedef struct struct_message {
  bool Beep;
} struct_message;

struct_message messgo;
struct_message messcome;
WiFiClient client;
HTTPClient http;
String URL="/update?api_key=72VSH0KDZ0HB7H5W&field1=";
void OnDataSent(uint8_t *mac_addr, uint8_t status)
{
  Serial.println("Status:");
  Serial.println(status);
}

void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) 
{
  memcpy(&messcome, incomingData, sizeof(messcome)); 
}

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  Serial.begin(9600);
  // nodemcu.begin(9600);
  WiFi.disconnect();
  delay(2000);
  // ss.begin(9600);
  Serial.print("Start connection");
  WiFi.begin("Pranjal","voew9525");
  while((!(WiFi.status()== WL_CONNECTED))){
      delay(200);
      Serial.print("..");
    }
  Serial.println("Connected");
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}

void loop() {
  Serial.println(messcome.Beep);
  if(messcome.Beep == true)
  {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(1000);
    digitalWrite(BUZZER_PIN, LOW);
    delay(1000);
      // erial.println("Booyah");
  }
  else
  {
    digitalWrite(BUZZER_PIN, LOW);
  }
  // digitalWrite(BUZZER_PIN, HIGH);
  delay(1000);
  // digitalWrite(BUZZER_PIN, LOW);
  delay(1000);
}
