///import libraries///
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ESP32Servo.h>

// Wi-Fi credentials
const char* ssid = "BlazingSpeed-TIME2.4Ghz";
const char* password = "1Co1Bo1Cr";

// MQTT credentials
const char* mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883;  // EMQX default port
const char* mqtt_user = "MQTT_username";
const char* mqtt_password = "MQTT_password";

// MQTT topic
const char* topic = "humidity";
const char* topicc = "Temperature";
const char* topic_Flame = "Sensor/fire";
const char* topic_Gas = "Sensor/Gas";
const char* Fan_usage = "Fan/usage";
// declare the varibles  and pins
#define DHTPIN 4
#define DHTTYPE DHT11
// Ldr Sensor/////////
const int ldrSensorPin = 5;  // Connect the PIR sensor to GPIO 2
int ledPin = 16;
int ledPin2 = 17;  // Connect the LED to GPIO 13
////PIR Sensor///////
const int pirSensorPin = 2;  // Connect the PIR sensor to GPIO 2
int ledPin_pir = 12;         // Connect the LED to GPIO 13
int ledPin_pir2 = 14;
////////GAS Sensor ///////
const int gasSensorPin = 15;  // Connect the PIR sensor to GPIO 2
////////Light////////////////
int light_bedroom1 = 27;
int light_bedroom2 = 26;
int light_livingroom = 25;
int light_Kitchen = 32;
//////
Servo Serv, Serv1;
DHT dht(DHTPIN, DHTTYPE);
int buzzer = 22;
int flame_sensor = 13;
int flame_detected;
unsigned long previousMillis = 0;
const long interval = 5000;
bool continuousMonitoring = true;
bool continuousMonitoring_gas = true;
WiFiClient wifiClient;
int fan = 21;
int Temperature;
int value = 30;
PubSubClient mqttClient(wifiClient);
void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
}

void callback(char* topic, byte* payload, unsigned int length) {
  // Convert payload to a string
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  ///////////////Get the window topic from node red ////////////////
  if (String(topic) == "Servo/window") {
    int angle = message.toInt();
    Serial.println(angle);
    if (angle == 5) {
      Serv.write(180);
      delay(12);
    }  else {
      Serv.write(0);
    }
    ///////////////Get the Door  topic from node red///////////////
  } else if (String(topic) == "Servo/Door") {
    int pos = message.toInt();
    
    Serial.println(pos);
    if (pos == 5) {
      Serv1.write(180);
    } else {
      Serv1.write(0);
    }
  }


  ///////////////  Handle LED switch//////////////////////////////////
  if (String(topic) == "led/room") {
    if (message == "on") {
      digitalWrite(light_bedroom1, HIGH);
    } else if (message == "off") {
      digitalWrite(light_bedroom1, LOW);
    }
  } else if (String(topic) == "led/room2") {
    if (message == "on") {
      digitalWrite(light_bedroom2, HIGH);
    } else if (message == "off") {
      digitalWrite(light_bedroom2, LOW);
    }
  } else if (String(topic) == "led/livingRoom") {
    if (message == "on") {
      digitalWrite(light_livingroom, HIGH);
    } else if (message == "off") {
      digitalWrite(light_livingroom, LOW);
    }
  } else if (String(topic) == "led/Kitchen") {
    if (message == "on") {
      digitalWrite(light_Kitchen, HIGH);
    } else if (message == "off") {
      digitalWrite(light_Kitchen, LOW);
    }
  }
  ///////////// Activate Sensors///////////////////////
  if (String(topic) == "Activate/fire_sensor") {
    if (message == "on") {
      continuousMonitoring = true;  // Turn on continuous monitoring
    } else if (message == "off") {
      continuousMonitoring = false;  // Turn off continuous monitoring
    }
  } else if (String(topic) == "Activate/Gas_sensor") {
    if (message == "on") {
      continuousMonitoring_gas = true;  // Turn on continuous monitoring
    } else if (message == "off") {
      continuousMonitoring_gas = false;  // Turn off continuous monitoring
    }
  }

  if (String(topic) == "temperature/value")

    value = message.toInt();
  if (Temperature > value & value!=10 ) {
    digitalWrite(fan, HIGH);  // Turn off the fan

    mqttClient.publish(Fan_usage, String(HIGH).c_str());
    delay(90);
  }
  else {
    digitalWrite(fan, LOW);  // Turn on the fan
  }
}
///////// To make sure the the MQTT protocol always connected/////////
void reconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT broker....");
    // Create a random client ID
    String clientId = "ESP32Client";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("connected");
      mqttClient.subscribe("Servo/window");
      mqttClient.subscribe("Servo/Door");
      mqttClient.subscribe("led/livingRoom");
      mqttClient.subscribe("led/room");
      mqttClient.subscribe("led/room2");
      mqttClient.subscribe("led/Kitchen");
      mqttClient.subscribe("Activate/fire_sensor");
      mqttClient.subscribe("Activate/Gas_sensor");
      mqttClient.subscribe("temperature/value");
      mqttClient.subscribe("Fan/usage");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
//////this function To read DHT11 data and publish it to Node red/////////
void readDHT() {
  unsigned long currentMillis = millis();            // get the current time// i used this delay to update the DHT11 eveery each 5000
  if (currentMillis - previousMillis >= interval) {  // check if the delay time has elapsed
    previousMillis = currentMillis;                  // reset the timer
    float humidity = dht.readHumidity();
    Temperature = dht.readTemperature();
    Serial.println(humidity);
    Serial.println(Temperature);
    mqttClient.publish(topic, String(humidity).c_str());
    mqttClient.publish(topicc, String(Temperature).c_str());
  }
}
//////this function To read Fire sensor data and publish it to Node red and to turn on the buzzer/////////
void fireDetection() {
  if (!continuousMonitoring) {
    return;  // Exit the function if continuousMonitoring is false
  }
  flame_detected = digitalRead(flame_sensor);
  if (flame_detected == 0) {
    Serial.println("Flame detected...! Take action immediately.");
    digitalWrite(buzzer, HIGH);
    Serial.println(flame_detected);
    mqttClient.publish(topic_Flame, String(flame_detected).c_str());
    delay(10000);
  } else {
    digitalWrite(buzzer, LOW);
  }
}
//////////////To read the LDR and then Control the lights based on the Data received
void LDR_Sensor() {
  int light = digitalRead(ldrSensorPin);

  if (light == HIGH) {
    // No Light detected
    digitalWrite(ledPin, HIGH);  // Turn on the LED
    digitalWrite(ledPin2, HIGH);
  } else {
    //  Light detected
    digitalWrite(ledPin, LOW);  // Turn off the LED
    digitalWrite(ledPin2, LOW);
  }
}
//////////////To read the motion Detection  and then Control the lights based on the Data received
void PIR_Sensor() {
  int motionDetected = digitalRead(pirSensorPin);

  if (motionDetected == HIGH) {
    // Motion detected
    digitalWrite(ledPin_pir, HIGH);  // Turn on the LED
    digitalWrite(ledPin_pir2, HIGH);
  } else {
    // No motion detected
    digitalWrite(ledPin_pir, LOW);  // Turn off the LED
    digitalWrite(ledPin_pir2, LOW);
  }
}
//////////////To read the Gas sensor  and then send the data to node red
void GAS_Sensor() {
  if (!continuousMonitoring_gas) {
    return;  // Exit the function if continuousMonitoring is false
  }
  int Gas_detcted = digitalRead(gasSensorPin);

  if (Gas_detcted == 0) {
    // Gas detected
    Serial.println("GAS detected...! Take action immediately.");
    mqttClient.publish(topic_Gas, String(Gas_detcted).c_str());
    delay(10000);
  }
}

void setup() {
  Serial.begin(115200);

  setup_wifi();

  dht.begin();
  ////LDR Setup//////
  pinMode(ldrSensorPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  //////PIR Setup/////
  pinMode(pirSensorPin, INPUT);
  pinMode(ledPin_pir, OUTPUT);
  pinMode(ledPin_pir2, OUTPUT);
  ////////led Setup////////////
  pinMode(light_bedroom1, OUTPUT);
  pinMode(light_bedroom2, OUTPUT);
  pinMode(light_livingroom, OUTPUT);
  pinMode(light_Kitchen, OUTPUT);
  pinMode(fan, OUTPUT);
  ////////////////
  Serv.attach(18);   // Change this to the GPIO pin connected to Servo for Window
  Serv1.attach(19);  // Change this to the GPIO pin connected to Servo for Door
  pinMode(buzzer, OUTPUT);
  pinMode(flame_sensor, INPUT);
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(callback);
}

void loop() {
  if (!mqttClient.connected()) {
    reconnect();
  }
  readDHT();
  fireDetection();
  LDR_Sensor();
  PIR_Sensor();
  GAS_Sensor();
  mqttClient.loop();
}
