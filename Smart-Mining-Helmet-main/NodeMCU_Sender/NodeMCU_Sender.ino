#include<ESP8266WiFi.h>
#include <espnow.h>
#include<WiFiClient.h>
#include<ESP8266HTTPClient.h>
#include <ThingSpeak.h>
#include <ArduinoJson.h>
#include<SoftwareSerial.h>
#include <TinyGPS++.h>
#include <ESP8266WebServer.h>
SoftwareSerial nodemcu(D6, D5);
SoftwareSerial  ss(4, 5) ;
ESP8266WebServer server(80);
long durationUltra;
TinyGPSPlus gps;
int distance;
int Check = 350;
const char* ssid = "Pranjal";
const char* password = "voew9525";
uint8_t broadcastAddress[] = {0x2C,0xF4,0x32,0x5D,0xD9,0x23};
long myChannelNumber = 2128605;
const char* apiKey = "72VSH0KDZ0HB7H5W";
static const uint32_t GPSBaud = 9600;
float lat , longi;

typedef struct struct_message {
  bool Beep;
} struct_message;

struct_message messgo;
struct_message messcome;

void OnDataSent(uint8_t *mac_addr, uint8_t status)
{
  Serial.println("Status:");
  Serial.println(status);
}

void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) 
{
  memcpy(&messcome, incomingData, sizeof(messcome)); 
}

WiFiClient client;
HTTPClient http;
String URL="/update?api_key=72VSH0KDZ0HB7H5W&field1=";
void setup() {
  Serial.begin(9600);
  nodemcu.begin(9600);
  WiFi.disconnect();
  delay(2000);
  ss.begin(9600);
  Serial.print("Start connection");
  WiFi.begin("Pranjal","voew9525");
  while((!(WiFi.status()== WL_CONNECTED))){
      delay(200);
      Serial.print("..");
    }
  Serial.println("Connected");
  ThingSpeak.begin(client);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  server.begin();
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}

void loop() {
  StaticJsonBuffer<1000> jsonBuffer;
  JsonObject& data = jsonBuffer.parseObject(nodemcu);

  if (data == JsonObject::invalid()) {
    jsonBuffer.clear();
    return;
  }

  messgo.Beep = false;  
  messcome.Beep = false;
  delay(1000);  
  
  float ppm = data["ppm"];
  float temp = data["temperature"];
  int press = data["pressure"];
  float alt = data["altitude"];
  int beat = data["beat"];
  int distance = data["distance"];
  Serial.println(lat);
  Serial.println(longi);
  sendData(ppm,temp,press,alt,beat,distance,lat,longi);
  if(beat>400 || ppm>600)
  {
    messgo.Beep = true;
    esp_now_send(broadcastAddress, (uint8_t *) &messgo, sizeof(messgo));
  }
  else
  {
    messgo.Beep = false;
  }
  if(messcome.Beep == true)
  {
    Serial.println("Help required");
  }
  Serial.println();
  delay(15000);
  
}


void sendData(float ppm,float temp,int press,float alt,int beat,int distance,float lat,float longi)
{
  WiFiClient client1 ;
  String newUrl = URL + String(lat) + "&field2=" + String(longi) + "&field3=" + String(press) + "&field4=" + String(ppm) + "&field5=" + String(beat) + "&field6=" + String(temp) + "&field7=" +String(alt) + "&field8=" + String(distance);
  Serial.println(newUrl);
  if (client1.connect("api.thingspeak.com", 80)) 
    {
      client1.print(String("GET ") + newUrl + " HTTP/1.1\r\n" +
      "Host: api.thingspeak.com\r\n" +
      "Connection: close\r\n\r\n");
      Serial.println("Data sent to ThingSpeak");
    }
    client1.stop();
}