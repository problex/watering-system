// ESP32 I2C Scanner
// Based on code of Nick Gammon  http://www.gammon.com.au/forum/?id=10896
// ESP32 DevKit - Arduino IDE 1.8.5
// Device tested PCF8574 - Use pullup resistors 3K3 ohms !
// PCF8574 Default Freq 100 KHz 

#include "credentials.h"

#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

Adafruit_BMP085 bmp;

//OTA includes
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <WiFi.h>
#include <WiFiClientSecure.h>

#include <SPI.h>

#include <TimeLib.h>


#define mphal_i2c_wait_a() os_delay_us(20)
#define mphal_i2c_wait_b() os_delay_us(10)




int trigPin = 5;            // HC-SR04 trigger pin
int echoPin = 4;            // HC-SR04 echo pin
float duration, distance;

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWD;

const char* mqtt_server = "10.1.1.22";

const int waterpumpPin = 32;
const int valve1Pin = 33;
const int valve2Pin = 25;
const int valve3Pin = 26;

WiFiClient espClient;
PubSubClient client(espClient);

long lastMsg = 0;
char msg[50];
int value = 0;

float temperature = 0;
float pressure = 0;
float humidity = 0;
float moisture = 0;
float tanklevel = 0;

char data[120];
StaticJsonBuffer<512> jsonBuffer;


void setup() {
  Serial.begin(115200);
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  //status = bme.begin();  
  
  pinMode(waterpumpPin, OUTPUT);
  pinMode(valve1Pin, OUTPUT);
  pinMode(valve2Pin, OUTPUT);
  pinMode(valve3Pin, OUTPUT);  
  pinMode(trigPin, OUTPUT); // define trigger pin as output
  Wire.begin (21, 22);   // sda= GPIO_21 /scl= GPIO_22
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1) {}
  }
  
  digitalWrite(valve1Pin, LOW);
  digitalWrite(waterpumpPin, LOW);
  digitalWrite(valve2Pin, LOW);
  digitalWrite(valve3Pin, LOW);  

  setup_wifi();
  

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  delay(1000);

}


void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
    ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

}

void callback(char* topic, byte* payload, unsigned int length) {
    
    if(strcmp(topic, "esp32/garden/valve1") == 0)
  {
            if (payload[0] == '1')
        {
           digitalWrite(valve1Pin, HIGH);
           client.publish("esp32/garden/valve1/state", "on");
        }
        //turn the light off if the payload is '0' and publish to the MQTT server a confirmation message
        else if (payload[0] == '0')
        {
           digitalWrite(valve1Pin, LOW);
           client.publish("esp32/garden/valve1/state", "off");
        }
  }
  else if(strcmp(topic, "esp32/garden/waterpump") == 0)
  {
            if (payload[0] == '1')
        {
           digitalWrite(waterpumpPin, HIGH);
           client.publish("esp32/garden/waterpump/state", "on");
        }
        //turn the light off if the payload is '0' and publish to the MQTT server a confirmation message
        else if (payload[0] == '0')
        {
           digitalWrite(waterpumpPin, LOW);
           client.publish("esp32/garden/waterpump/state", "off");
        }
  }
  else if(strcmp(topic, "esp32/garden/valve2") == 0)
  {
            if (payload[0] == '1')
        {
           digitalWrite(valve2Pin, HIGH);
           client.publish("esp32/garden/valve2", "Valve On");
        }
        //turn the light off if the payload is '0' and publish to the MQTT server a confirmation message
        else if (payload[0] == '0')
        {
           digitalWrite(valve2Pin, LOW);
           client.publish("esp32/garden/valve2", "Valve Off");
        }
  }
    else if(strcmp(topic, "esp32/garden/valve3") == 0)
  {
            if (payload[0] == '1')
        {
           digitalWrite(valve3Pin, HIGH);
           client.publish("esp32/garden/valve3", "Valve On");
        }
        //turn the light off if the payload is '0' and publish to the MQTT server a confirmation message
        else if (payload[0] == '0')
        {
           digitalWrite(valve3Pin, LOW);
           client.publish("esp32/garden/valve3", "Valve Off");
        }
  }
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client1",MQTT_USER,MQTT_PASSWD)) { //make the name unique on your mqtt server
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/garden/valve1");
      client.subscribe("esp32/garden/valve2");
      client.subscribe("esp32/garden/valve3");            
      client.subscribe("esp32/garden/waterpump");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}



void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  ArduinoOTA.handle();
  
  long now = millis();
  if (now - lastMsg > 600000) {
    lastMsg = now;
 
    // Temperature in Celsius
    temperature = bmp.readTemperature();   
    // Uncomment the next line to set temperature in Fahrenheit 
    // (and comment the previous temperature line)
    //temperature = 1.8 * bme.readTemperature() + 32; // Temperature in Fahrenheit

    if (temperature < 50) {
      
      // Convert the value to a char array
      char tempString[8];
      dtostrf(temperature, 1, 2, tempString);
    
      Serial.print("Temperature = ");
      Serial.print(tempString);
      client.publish("esp32/garden/temp", tempString);
      Serial.println(" *C");
    }
    pressure = bmp.readPressure();

    if (pressure < 120000) {
          
      // Convert the value to a char array
      char preString[8];
      dtostrf(pressure, 6, 0, preString);
      Serial.print("Pressure = ");
      Serial.println(preString);
      client.publish("esp32/garden/pressure", preString);
    }
    
    Serial.println("");
    digitalWrite(echoPin, LOW);   // set the echo pin LOW
    digitalWrite(trigPin, LOW);   // set the trigger pin LOW
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);  // set the trigger pin HIGH for 10μs
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);  // measure the echo time (μs)
    distance = (duration/2.0)*0.0343;   // convert echo time to distance (cm)
    if(distance>400 || distance<2) Serial.println("Out of range");
    else
    {
      Serial.print("Distance: ");
      Serial.print(distance, 1); Serial.println(" cm");
      // Convert the value to a char array
      char tanString[8];
      char tanLitresString[8];      
      int tankPer = 100 - (distance / 88 * 100);
      int tankLitres = 1000 - (distance / 88 * 1000);
      //dtostrf(distance, 4, 0, tanString);
      dtostrf(tankPer, 4, 0, tanString);
      dtostrf(tankLitres, 4, 0, tanLitresString);
      client.publish("esp32/garden/tanklevel", tanString);
      client.publish("esp32/garden/tanklitres", tanLitresString);
    }    
  }      
}
