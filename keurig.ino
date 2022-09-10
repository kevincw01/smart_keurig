/*
Author: Kevin Williams
Description: IOT interface to keurig classic K50 coffee maker.  Designed for Wemos D1 Mini clone but could use any with adapting
Features:
1) Remote on/off via MQTT subscription.
Pin D1 (wemos d1 pin 5) is soldered Keurig SW2 power +3.3V.  Ground is soldered to Keurig SW2 power ground. 
Bringing D1 low for 50ms and then back to high (via digital write) simulates keurig power button push.  
Keurig power status is sensed via power LED voltage on A0.  Power button is pushed only if in the correct state.
Pin D14 (wemos D5) monitors a float switch connected to ground and is pulled up.  Upon ground, water basin is full
Pin D2 (wemos D4) triggers NO relay to open house water valve to fill basin.
*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "credentials.h"

const long timeBetweenMessages = 1000 * 60;  //periodic status every minute
const long timeBetweenPowerCheck = 1000;  //check keurig power state every second
const long timeBasinFill = 1000 * 15; //length of time we fill basin
const long timeBetweenBasinFill = 1000 * 45; //dont fill basin sooner than this
int power_off_adc = 1024;  //value adc reads when keurig is off
bool keurigPowerOn = false;
bool fillingBasin = false;
const int relayPin = 16;  //wemos d0
const int floatPin = 14;

DynamicJsonDocument doc(200); //subscribe doc
DynamicJsonDocument doc2(200); //publish doc

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;  //flag for periodic status every minute
long lastMsg_A = 0;  //flag for perodic status every second
long lastFill = 0;  //last time we filled the basin

int status = WL_IDLE_STATUS;     // the starting Wifi radio's status

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pswd);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

int getQuality() {
  if (WiFi.status() != WL_CONNECTED)
    return -1;
  int dBm = WiFi.RSSI();
  if (dBm <= -100)
    return 0;
  if (dBm >= -50)
    return 100;
  return 2 * (dBm + 100);
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  DeserializationError error = deserializeJson(doc, payload);
  if (error)
    Serial.println("error deserializing json");
  else {
    Serial.println("success deserializing json");  
    const char* power_j  = doc["power"];
    
    if(strncmp(power_j,"on",3) == 0) {
      Serial.println("got on command");
      if(!isKeurigOn()) {
        togglePower();
        keurigPowerOn = true;
        publishKeurigStatus();
      } else
        Serial.println("keurig commanded on but already on");
    } else if(strncmp(power_j,"off",3) == 0) {
      Serial.println("got off command");
      if(isKeurigOn()) {
          togglePower();
          keurigPowerOn = false;
          publishKeurigStatus();
       } else
          Serial.println("keurig commanded off but already off");
    } else {
      Serial.println("unknown command");
    }
    
  }
}

void togglePower() {
  digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
  digitalWrite(5, LOW); //push pwr button
  delay(50);
  digitalWrite(5, HIGH);  //release power button
  digitalWrite(BUILTIN_LED, HIGH);
}

bool isKeurigOn() {
  int sensorValue = analogRead(A0);
  //Serial.print("analog read at ");
  //Serial.println(sensorValue);
  if(sensorValue == power_off_adc)
    return false;
  else
    return true;
}

String macToStr(const uint8_t* mac)
{
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
    if (i < 5)
      result += ':';
  }
  return result;
}

String composeClientID() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  String clientId;
  clientId += "esp-";
  clientId += macToStr(mac);
  return clientId;
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    String clientId = composeClientID() ;
    clientId += "-";
    clientId += String(micros() & 0xff, 16); // to randomise. sort of

    // Attempt to connect
    if (client.connect(clientId.c_str(),mqttuser,mqttpw)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(topic, ("connected " + composeClientID()).c_str() , true );
      // ... and resubscribe
      // topic + clientID + in
      String subscription;
      subscription += topic;
      subscription += "/";
      subscription += composeClientID() ;
      subscription += "/in";
      client.subscribe(subscription.c_str() );
      Serial.print("subscribed to : ");
      Serial.println(subscription);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.print(" wifi=");
      Serial.print(WiFi.status());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  pinMode(5, OUTPUT); //D1 connected to keurig power button
  digitalWrite(5, HIGH);  //pull D1 output high
  pinMode(floatPin, INPUT_PULLUP); //connected to float switch (float connected to ground)
  pinMode(relayPin, OUTPUT); //connected to NO valve basin fill relay
  digitalWrite(relayPin, HIGH);  //pull output high (relay is LOW on)
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  digitalWrite(BUILTIN_LED, HIGH);   
  Serial.begin(115200);

  //initialize keurig power status
  isKeurigOn(); 

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void publishKeurigStatus() {

  String payload;
  doc2["micros"] = micros();
  doc2["client"] = composeClientID();
  if(keurigPowerOn)
    doc2["power"] = "on";
  else
    doc2["power"] = "off";
  doc2["RSSI"] = getQuality();
  if(fillingBasin)
    doc2["basinFill"] = "on";
  else
    doc2["basinFill"] = "off";
  serializeJson(doc2, payload);

  String pubTopic;
   pubTopic += topic ;
   pubTopic += "/";
   pubTopic += composeClientID();
   pubTopic += "/out";

  Serial.print("Publish topic: ");
  Serial.println(pubTopic);
  Serial.print("Publish message: ");
  Serial.println(payload);
  
  client.publish( (char*) pubTopic.c_str() , payload.c_str(), true );
}
void loop() {
  // confirm still connected to mqtt server
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  
  //update keurig power status
  if(now - lastMsg_A > timeBetweenPowerCheck) {
    lastMsg_A = now;
    //check if we detect a power state change
    bool iko = isKeurigOn();
    //Serial.print("keurigPowerOn=");
    //Serial.println(keurigPowerOn);
    //Serial.print("isKeurigOn()=");
    //Serial.println(iko);
    if(keurigPowerOn != iko) {
      keurigPowerOn = iko;
      Serial.println("power state change");
      publishKeurigStatus();
    }
  }
  
  //send mqtt status on start and then perodically
  if (lastMsg == 0 || (now - lastMsg > timeBetweenMessages) ) {
    lastMsg = now;
    publishKeurigStatus();
  }

  //check basin water level
  if(keurigPowerOn && digitalRead(floatPin) == HIGH) {
    //basin water is low

    //dont fill if we're already filling and dont fill < timeBetweenBasnFIll
    if(!fillingBasin && (now - lastFill > timeBetweenBasinFill)) {
      //allow filling basin
      fillingBasin = true;
      lastFill = now;
      publishKeurigStatus();
      digitalWrite(relayPin, LOW);  // open valve relay
      Serial.println("valve open");
    }
  } else {
    //if fill switch says basin is full, ensure we aren't filling
    if(fillingBasin) {
      lastFill = now;
      fillingBasin = false;
      publishKeurigStatus();
      Serial.println("valve closed");
    }
    //always close valve just to be safe
    digitalWrite(relayPin, HIGH);  // close valve relay
  }
  
  //check if we need to stop filling basin
  if(fillingBasin == true) {
    if(now - lastFill > timeBasinFill) {
      //time to close basin fill valve relay
      fillingBasin = false;
      publishKeurigStatus();
      digitalWrite(relayPin, HIGH);  //close valve relay
      Serial.println("valve closed due to filling timeout");
    }
  }
}
