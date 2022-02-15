#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include "DHT.h"
#include <Servo.h>

//WiFi Connection configuration
char ssid[] = "ESGI";     //  le nom du reseau WIFI
char password[] = "Reseau-GES";  // le mot de passe WIFI
char mqtt_server[] = "test.mosquitto.org";  //adresse IP serveur (mosquitto publique)
#define MQTT_USER "" 
#define MQTT_PASS ""
#define DHTPIN D4
#define DHTTYPE DHT11

WiFiClient espClient;
PubSubClient MQTTclient(espClient);
DHT dht(DHTPIN, DHTTYPE);
Servo servo;

const int buttonPin = D6;
const int fanPin = D8;
int buttonState = 0;


void callback(String topic, byte* payload, unsigned int length) {
  
  String data;
  for (int i = 0; i < length; i++) {
    //Serial.print((char)payload[i]);
    data += (char)payload[i];
  }

  if (topic == "peepoopee/fan/value") {
    if(data == "true") {
      analogWrite(fanPin, 255);
    }else if (data == "false") { 
     analogWrite(fanPin, 0);
    }else {
      Serial.println("Bad value : "+data);
    }
  } 

  if (topic == "peepoopee/servo/value") {
    if(data == "open") {
      servo.write(0);
      delay(3000);
    }else if (data == "close") {
      servo.write(180);
      delay(3000);
    }else {
      Serial.println("Bad value : "+data);
    }
  } 
}

//Fonction connection MQTT
void MQTTconnect() {
  
  while (!MQTTclient.connected()) {
      Serial.print("Attente  MQTT connection...");
      String clientId = "TestClient-";
      clientId += String(random(0xffff), HEX);

    // test connexion
    if (MQTTclient.connect(clientId.c_str(),"","")) {
      Serial.println("connected");
      MQTTclient.subscribe("peepoopee/servo/value");
      MQTTclient.subscribe("peepoopee/fan/value");
    } else {  // si echec affichage erreur
      Serial.print("ECHEC, rc=");
      Serial.print(MQTTclient.state());
      Serial.println(" nouvelle tentative dans 5 secondes");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT_PULLUP);
  analogWrite(fanPin, 0);
  
  servo.attach(D1);
  servo.write(180);
  delay(3000);
  
  // Conexion WIFI
   WiFi.begin(ssid, password);
   Serial.println("");
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
   Serial.println("Connected");
   MQTTclient.setServer(mqtt_server, 1883);
   MQTTclient.setCallback(callback);
}

void seatMotionReader() {
  buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH) {
    MQTTclient.publish("peepoopee/seatMotion/status","false");
  } else {
    MQTTclient.publish("peepoopee/seatMotion/status","true");
  }
}

void readTemp() {
  float temperature = dht.readTemperature();
  String payload = (String)temperature;
  Serial.println(payload);
  if(payload != "nan") {
    MQTTclient.publish("peepoopee/tempSensor/value",payload.c_str());
  }
}

void loop() {
  if (!MQTTclient.connected()) {
    MQTTconnect();
  }
  MQTTclient.loop();
  delay(1000);
  seatMotionReader();
  readTemp();
}
