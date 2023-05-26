
#include <PubSubClient.h>
#include <WiFi.h>

#include "DHT.h" 

#include "SharpGP2Y10.h"
   
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

#include <MQUnifiedsensor.h>
#define placa "NodeMCU-32S"
#define Voltage_Resolution 5
#define pin 33
#define type "MQ-135" 
#define ADC_Bit_Resolution 10 
#define RatioMQ135CleanAir 3.6

MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);
Adafruit_BMP280 bmp; // I2C


const char* ssid = "Ken";
const char* password = "88888888"; 
const char* mqtt_server = "192.168.85.66";
const unsigned int mqtt_port = 1883;


int voPin = 34;
int ledPin = 16;
float dustDensity = 0;
SharpGP2Y10 dustSensor(voPin, ledPin);      

const int DHTPIN = 4;      
const int DHTTYPE = DHT11;  

long now = millis();
long lastMeasure = 0;

DHT dht(DHTPIN, DHTTYPE);

void setup_wifi() 
{
  delay(10);
  Serial.println();
  Serial.print("Đang Kết Nối Tới ");
  Serial.println(ssid);  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected"); 
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

WiFiClient wifi;
PubSubClient client(mqtt_server,mqtt_port,callback,wifi);

void callback(char* topic, byte* payload, unsigned int length)
{
   Serial.print("Message from [");
   Serial.print("]:");
}
  
void setup() 
{
  Serial.begin(115200);
  dht.begin(); 
  unsigned status;
  status = bmp.begin(0x76);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  
  MQ135.setA(110.47); MQ135.setB(-2.862); 
  MQ135.init(); 
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0/10);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);      
}

void reconnect()
{
 while (!client.connected()) 
{
 Serial.print(" Đang Kết Nối MQTT...");
 
 if (client.connect("ESP32Client")) 
{
 Serial.println("Kết Nối Thành Công");
 Serial.println("MQTT Đã Kết Nối");
 delay(3000);
}
 else
{
 Serial.print("Kết Nối MQTT Thất Bại ");
 Serial.println("Thử Lại Trong 5 Giây");
 delay(5000);
 
}
}
}

void loop() 
{
   if (!client.connected())
   {
    reconnect();
   }
   if(!client.loop())
    client.connect("ESP32Client");

  now = millis();
   if (now - lastMeasure >=  1000)
   {
    lastMeasure = now;
  
  MQ135.update(); 
  
  float h = dht.readHumidity();    
  float t = dht.readTemperature(); 
  float p =  bmp.readPressure()*0.01;
  float CO2 = MQ135.readSensor()+400; 
  dustDensity = dustSensor.getDustDensity();
  
   char Bui[8];
  dtostrf(dustDensity, 6, 2, Bui);
   char Nhiet_do[8];
  dtostrf(t, 6, 2, Nhiet_do);
   char Do_am[8];
  dtostrf(h, 6, 2, Do_am);
   char string [9];
  dtostrf(p, 8, 2, string);
   char Gas[8];
  dtostrf(CO2, 6, 2, Gas);

  client.publish("Tem", Nhiet_do);
  client.publish("Hum", Do_am); 
  client.publish("Per", string);
  client.publish("Gas", Gas);
  client.publish("BUI", Bui); 
  
  Serial.print(" Nhiet do: ");
  Serial.print(t);  
  Serial.println("*C");
            
  Serial.print(" Do am: ");
  Serial.print(h); 
  Serial.println("%"); 

  Serial.print("ÁP suất = ");
  Serial.print(p);
  Serial.println(" hPa");

  Serial.print("CO2: "); 
  Serial.print(CO2);
  Serial.println("ppm");

  Serial.print("Bụi: ");
  Serial.println(dustDensity);  
                  
                  
}
}
